#include <cstdio>
#include "storage/FlightDirectory.h"
#include "storage/FlightHeaderStore.h"
#include "storage/FlightSampleStore.h"
#include "../fixtures/ReplayFixture.h"
#include "storage/BinaryEncoding.h"
#include <iostream>
#include "storage/ProfiledFlashDevice.h"
#include "storage/VerifiedPage.h"
#include <cassert>
#include <limits>
#include "storage/FlightSummaryStore.h"
#include <cstring>
#include <vector>
#include <algorithm>
#include <cmath>

// Each namespace isolates one component scenario and its fault model.
namespace test_storage_encoding {
using namespace storage;
struct Device : FlashDevice {
  std::vector<uint8_t> bytes = std::vector<uint8_t>(0x18000, 255);
  bool read(uint32_t a, void *p, std::size_t n) override {
    if (a > bytes.size() || n > bytes.size()-a) return false;
    memcpy(p, bytes.data()+a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    if (a > bytes.size() || n > bytes.size()-a) return false;
    const auto *in = static_cast<const uint8_t *>(p);
    for (std::size_t i = 0; i < n; ++i) { assert((bytes[a+i] & in[i]) == in[i]); bytes[a+i] &= in[i]; }
    return true;
  }
  bool eraseSector(uint32_t a) override {
    if (a%4096 || a > bytes.size() || 4096 > bytes.size()-a) return false;
    memset(bytes.data()+a, 255, 4096); return true;
  }
};
void formatFixture(Device &flash) {
  const auto config = replay_test::fixture();
  ConfigurationStore settings(flash); assert(settings.save(config, replay_test::limits, true));
  FlightDirectory directory(flash); assert(directory.format());
  FlightEntry entry; entry.sequence = UINT64_C(0x12345678ABCDEF01); entry.start = FlashLayout::DataBegin;
  entry.allocatedBytes = 8192; assert(directory.update(entry));
  FlightHeaderStore header(flash); assert(header.create(entry.start, config, replay_test::limits,
      SensorMode::Simulation, OutputMode::Virtual, sensors::FrameConvention::LoggedCalibratedBody));
  assert(header.recordStart(10, recovery::OutputKind::Pyro, 1, 0));
  assert(header.recordStart(10, recovery::OutputKind::Servo, 1, 14744)); assert(header.finish(true));
  FlightSampleStore samples(flash); assert(samples.begin(entry.sequence, entry.start+4096, 4096));
  for (unsigned i = 0; i < 5; ++i) {
    FlightSample sample; sample.timeMs = i*1234; sample.phase = recovery::FlightPhase::Descent;
    sample.values[FlightSample::AltitudeM] = 12000.5f + float(i);
    sample.values[FlightSample::PressureHpa] = 1013.25f;
    sample.values[FlightSample::VelocityMps] = -123.5f;
    assert(samples.append(sample));
  }
  assert(samples.finish()); entry.committedBytes = 4096+samples.committedBytes();
  entry.status = FlightStatus::Completed; assert(directory.update(entry));
}
void run() {
  using namespace storage::binary;
  const uint8_t expected[] = {0xEF,0xCD,0xAB,0x89,0x67,0x45,0x23,0x01};
  uint8_t bytes[10]; memset(bytes, 0xA5, sizeof(bytes));
  put(bytes+1, UINT64_C(0x0123456789ABCDEF), 8);
  assert(bytes[0] == 0xA5 && bytes[9] == 0xA5 && memcmp(bytes+1, expected, 8) == 0);
  assert(get64(bytes+1, 8) == UINT64_C(0x0123456789ABCDEF));
  assert(get32(bytes+1, 4) == UINT32_C(0x89ABCDEF));
  assert(get32(bytes+1, 2) == UINT32_C(0xCDEF) && get32(bytes+1, 1) == 0xEF);
  assert(get64(bytes+1, 5) == UINT64_C(0x6789ABCDEF));
  memset(bytes, 0xA5, sizeof(bytes)); put(bytes+1, UINT64_MAX, 2);
  assert(bytes[1] == 255 && bytes[2] == 255 && bytes[3] == 0xA5);
  assert(get32(nullptr, 0) == 0 && crc(nullptr, 0) == 0);
  put(nullptr, 0, 0);
  const uint8_t check[] = {'1','2','3','4','5','6','7','8','9'};
  assert(crc(check, sizeof(check)) == UINT32_C(0xCBF43926));
  for (unsigned split = 0; split <= sizeof(check); ++split) {
    const auto first = extendCrc(CrcInitial, check, split);
    assert((extendCrc(first, check+split, sizeof(check)-split) ^ CrcInitial) == UINT32_C(0xCBF43926));
  }
  Device flash; formatFixture(flash);
  uint64_t fingerprint = UINT64_C(0xCBF29CE484222325);
  for (auto byte : flash.bytes) fingerprint = (fingerprint ^ byte) * UINT64_C(0x100000001B3);
  // Fixed image with zero channel delays, directory, header,
  // summary reservation, historical start timestamps and sample blocks.
  if (fingerprint != UINT64_C(0x3000346afdd6427f)) std::cerr << "Storage image fingerprint: 0x" << std::hex << fingerprint << std::dec << '\n';
  assert(fingerprint == UINT64_C(0x3000346afdd6427f));
  std::cout << "Storage encoding tests passed: known CRC vector, incremental CRC, endian widths and unchanged full storage image.\n";
}
} // namespace test_storage_encoding

namespace test_flash_timing {
namespace {
uint32_t ticks = 0;
uint32_t clockUs() { return ticks; }
struct Device : storage::FlashDevice {
  bool success = true;
  unsigned calls = 0;
  bool read(uint32_t, void *, std::size_t) override { ticks += 7; ++calls; return success; }
  bool program(uint32_t, const void *, std::size_t) override { ticks += 31; ++calls; return success; }
  bool eraseSector(uint32_t) override { ticks += 500; ++calls; return success; }
};
struct PageDevice : storage::FlashDevice {
  uint8_t stored[256] = {};
  unsigned reads = 0, programs = 0;
  bool failProgram = false, failRead = false, corruptRead = false;
  bool read(uint32_t address, void *p, std::size_t n) override {
    assert(address == 0x15000 && n == sizeof(stored)); ++reads;
    if (failRead) return false;
    std::memcpy(p, stored, n); if (corruptRead) static_cast<uint8_t *>(p)[0] ^= 1;
    return true;
  }
  bool program(uint32_t address, const void *p, std::size_t n) override {
    assert(address == 0x15000 && n == sizeof(stored)); ++programs;
    if (failProgram) return false;
    std::memcpy(stored, p, n); return true;
  }
  bool eraseSector(uint32_t) override { assert(false); return false; }
};
}
void run() {
  for (unsigned mode = 0; mode < 4; ++mode) {
    PageDevice device; device.failProgram = mode == 1; device.failRead = mode == 2; device.corruptRead = mode == 3;
    uint8_t source[256], scratch[256]; std::memset(source, 0xA5, sizeof(source));
    const auto result = storage::writeVerifiedPage(device, 0x15000, source, scratch);
    assert(result == (mode == 0 ? storage::PageWriteError::None : mode == 3 ? storage::PageWriteError::Verify : storage::PageWriteError::Io));
    assert(device.programs == 1 && device.reads == (mode == 1 ? 0u : 1u)); // No retry or I/O after a failed program.
  }
  Device raw; storage::ProfiledFlashDevice flash(raw, clockUs); char bytes[16] = {};
  assert(flash.timingValid());
  assert(flash.read(0, bytes, sizeof(bytes)) && flash.program(0, bytes, sizeof(bytes)) && flash.eraseSector(0));
  const auto &stats = flash.timing();
  assert(stats.read.time.count == 1 && stats.read.time.totalUs == 7 && stats.read.time.maxUs == 7 && stats.read.bytes == 16);
  assert(stats.program.time.count == 1 && stats.program.time.totalUs == 31 && stats.program.time.maxUs == 31);
  assert(stats.erase.time.maxUs == 500 && stats.erase.bytes == 4096);
  ticks = std::numeric_limits<uint32_t>::max()-10; raw.success = false;
  assert(!flash.program(0, bytes, 8)); // Measured across micros() rollover, including failed calls.
  assert(stats.program.time.count == 2 && stats.program.time.totalUs == 62 && stats.program.bytes == 24);
  assert(stats.program.failures == 1 && stats.program.time.maxUs == 31);
  const auto calls = raw.calls; flash.resetTiming(); assert(raw.calls == calls);
  assert(stats.read.time.count == 0 && stats.program.failures == 0 && stats.erase.bytes == 0);
  storage::ProfiledFlashDevice untimed(raw); assert(!untimed.timingValid());
  assert(!untimed.read(0, bytes, 4));
  assert(untimed.timing().read.time.count == 1 && untimed.timing().read.time.totalUs == 0 && untimed.timing().read.failures == 1);
  timing::Statistics longRun; longRun.add(UINT32_MAX); longRun.add(UINT32_MAX);
  assert(longRun.totalUs == uint64_t(UINT32_MAX)*2 && longRun.maxUs == UINT32_MAX);
  puts("Flash timing tests passed: operation boundaries, failures, rollover, uint64 totals, reset without I/O and unavailable clocks.");
}
} // namespace test_flash_timing

namespace test_flight_summary {
using namespace storage;
struct Cut {};
struct Device : FlashDevice {
  std::vector<uint8_t> bytes=std::vector<uint8_t>(0x16000,255);
  long budget=-1; unsigned writes=0;
  bool read(uint32_t a,void *p,std::size_t n) override { memcpy(p,bytes.data()+a,n); return true; }
  bool program(uint32_t a,const void *p,std::size_t n) override {
    ++writes; const auto *in=static_cast<const uint8_t *>(p);
    for (std::size_t i=0;i<n;++i) { if (!budget) throw Cut{}; if (budget>0) --budget; assert((bytes[a+i]&in[i])==in[i]); bytes[a+i]&=in[i]; }
    return true;
  }
  bool eraseSector(uint32_t) override { assert(false); return false; }
};
void run() {
  constexpr uint32_t address=0x15400, header=0x15000, checksum=0x13572468;
  constexpr uint64_t sequence=UINT64_C(0xFEDCBA9812345678);
  RecordingSummary summary; summary.savedSamples=9203; summary.droppedSamples=230;
  summary.queuePeak=64; summary.queueCapacity=64; summary.reason=RecordingEnd::Stopped;
  for (unsigned cut=0;cut<=512;++cut) {
    Device flash; FlightSummaryStore store(flash); flash.budget=cut;
    try { store.write(address,sequence,header,checksum,summary); } catch (const Cut &) {}
    flash.budget=-1; SummaryState state; RecordingSummary result;
    assert(store.read(address,sequence,header,checksum,state,result));
    assert(state==SummaryState::Missing || state==SummaryState::Incomplete || state==SummaryState::Valid);
    if (state==SummaryState::Valid) assert(result.savedSamples==9203 && result.droppedSamples==230 && result.queuePeak==64 && result.reason==RecordingEnd::Stopped);
    else assert(result.droppedSamples==0); // State, never zero, determines availability.
  }
  Device flash; FlightSummaryStore store(flash); assert(store.write(address,sequence,header,checksum,summary));
  const auto original=flash.bytes; const auto writes=flash.writes;
  assert(!store.write(address,sequence,header,checksum,summary) && store.error()==SummaryError::NotErased);
  assert(flash.bytes==original && flash.writes==writes);
  SummaryState state; RecordingSummary result;
  assert(store.read(address,sequence+1,header,checksum,state,result) && state==SummaryState::Corrupt);
  assert(store.read(address,sequence,header+4096,checksum,state,result) && state==SummaryState::Corrupt);
  assert(store.read(address,sequence,header,checksum+1,state,result) && state==SummaryState::Corrupt);
  flash.bytes[address+28]^=1;
  assert(store.read(address,sequence,header,checksum,state,result) && state==SummaryState::Corrupt);
  flash.bytes=original;
  storage::binary::put(flash.bytes.data()+address+4,2,2);
  const auto crc=storage::binary::crc(flash.bytes.data()+address,252);
  storage::binary::put(flash.bytes.data()+address+252,crc,4);
  storage::binary::put(flash.bytes.data()+address+256+4,crc,4);
  storage::binary::put(flash.bytes.data()+address+256+8,~crc,4);
  assert(store.read(address,sequence,header,checksum,state,result) && state==SummaryState::Unsupported);
  summary.queuePeak=65; assert(!store.write(address+512,sequence,header,checksum,summary) && store.error()==SummaryError::InvalidArgument);
  assert(flash.writes==writes);
  std::cout<<"Flight summary passed: 513 byte cuts, identity binding, CRC, version handling, no retry/erase and explicit unavailable states.\n";
}
} // namespace test_flight_summary

namespace test_flight_sample_store {
using namespace storage;
struct Cut {};
struct MemoryFlash : FlashDevice {
  std::vector<uint8_t> bytes = std::vector<uint8_t>(0x18000, 255);
  long budget = -1;
  unsigned writes = 0;
  bool failRead = false, failProgram = false;
  unsigned failAfterProgram = 0;
  bool read(uint32_t a, void *p, std::size_t n) override {
    if (failRead || a > bytes.size() || n > bytes.size() - a) return false;
    memcpy(p, bytes.data() + a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    assert(a >= 0x15000 && a + n <= bytes.size() && n == 256 && a % 256 == 0);
    if (failProgram) return false;
    ++writes; const auto *in = static_cast<const uint8_t *>(p);
    for (std::size_t i = 0; i < n; ++i) {
      if (budget == 0) throw Cut{};
      if (budget > 0) --budget;
      assert((bytes[a+i] & in[i]) == in[i]); bytes[a+i] &= in[i];
    }
    return writes != failAfterProgram;
  }
  bool eraseSector(uint32_t) override { assert(false); return false; }
};
FlightSample sample(uint32_t t) {
  FlightSample s; s.timeMs = t; s.phase = recovery::FlightPhase::Coast;
  s.batteryMv=t ? 8240 : UINT16_MAX;
  for (unsigned i = 0; i < 14; ++i) s.values[i] = float(i) + 0.125f;
  s.values[FlightSample::AltitudeM] = 12000.5f; // No old int16 altitude clipping.
  s.values[FlightSample::PressureHpa] = 1013.25f;
  return s;
}
void run() {
  MemoryFlash flash; FlightSampleStore writer(flash);
  const auto pristine = flash.bytes;
  assert(!writer.begin(0, 0x15000, 4096));
  assert(!writer.begin(1, 0x14000, 4096));
  assert(!writer.begin(1, 0x15001, 4096));
  assert(!writer.begin(1, FlashLayout::DataEnd - 4096, 8192));
  assert(!writer.begin(1, 0xFFFFF000, 8192));
  assert(flash.bytes == pristine && flash.writes == 0);
  assert(writer.begin(7, 0x15000, 4096));
  assert(!writer.begin(8, 0x16000, 4096) && writer.error() == SampleError::Active);
  for (unsigned i = 0; i < 11; ++i) assert(writer.append(sample(i)));
  assert(flash.writes == 0 && writer.committedSamples() == 0);
  assert(!writer.append(sample(11)) && writer.error() == SampleError::BufferFull);
  assert(writer.flush() && writer.committedSamples() == 11 && writer.committedBytes() == 1024);
  assert(flash.writes == 4);
  SampleBlock block;
  assert(writer.readBlock(0x15000, 7, 0, block));
  assert(block.count == 11 && block.firstSample == 0 && block.samples[10].timeMs == 10);
  assert(block.samples[0].values[FlightSample::AltitudeM] == 12000.5f);
  assert(block.samples[0].batteryMv==UINT16_MAX && block.samples[1].batteryMv==8240);
  assert(flash.bytes[0x15000+32+64+61]==0x30 && flash.bytes[0x15000+32+64+62]==0x20); // 8240 mV, little endian.
  assert(flash.bytes[0x15000+32+64+63]==0 && FlightSampleStore::RecordBytes==64);
  // Golden little-endian layout and float encoding, independent of round-trip decoding.
  assert(flash.bytes[0x15000] == 'P' && flash.bytes[0x15001] == 'S');
  assert(flash.bytes[0x15008] == 7 && flash.bytes[0x15014] == 11);
  assert(flash.bytes[0x15024] == 0 && flash.bytes[0x15025] == 0 && flash.bytes[0x15026] == 0 && flash.bytes[0x15027] == 0x3E);
  const auto stable = flash;
  assert(!writer.append(sample(10)) && writer.error() == SampleError::Timestamp);
  auto invalid = sample(11); invalid.phase = static_cast<recovery::FlightPhase>(99);
  assert(!writer.append(invalid));
  auto missing = sample(UINT32_MAX); missing.values[0] = std::numeric_limits<float>::infinity();
  assert(writer.append(missing)); assert(writer.finish());
  assert(writer.readBlock(0x15400, 7, 1, block));
  assert(block.count == 1 && block.firstSample == 11 && block.samples[0].timeMs == UINT32_MAX);
  assert(std::isnan(block.samples[0].values[0]));
  assert(!writer.append(sample(12)) && writer.error() == SampleError::Inactive);
  assert(!writer.begin(8, 0x15000, 4096) && writer.error() == SampleError::NotErased);
  block.count = 77;
  assert(!writer.readBlock(0x15000, 8, 0, block) && writer.error() == SampleError::Identity && block.count == 77);
  assert(!writer.readBlock(0x15000, 7, 1, block) && writer.error() == SampleError::Identity);
  assert(!writer.readBlock(0x15800, 7, 2, block) && writer.error() == SampleError::Empty);
  std::copy(flash.bytes.begin() + 0x15000, flash.bytes.begin() + 0x15400, flash.bytes.begin() + 0x16000);
  assert(!writer.readBlock(0x16000, 7, 0, block) && writer.error() == SampleError::Identity);
  flash.bytes[0x15024] ^= 1;
  assert(!writer.readBlock(0x15000, 7, 0, block) && writer.error() == SampleError::Corrupt);
  // Every byte boundary of a block append, retaining the preceding block unchanged.
  for (long cut = 0; cut <= 1024; ++cut) {
    MemoryFlash f = stable; FlightSampleStore w(f);
    // Separate allocated extent: deliberately leave the prior committed block intact.
    assert(w.begin(8, 0x16000, 4096));
    assert(w.append(sample(100))); f.budget = cut;
    try { w.flush(); } catch (const Cut &) {}
    f.budget = -1;
    FlightSampleStore reader(f); SampleBlock recovered;
    assert(reader.readBlock(0x15000, 7, 0, recovered) && recovered.count == 11);
    const bool valid = reader.readBlock(0x16000, 8, 0, recovered);
    if (valid) assert(recovered.count == 1 && recovered.samples[0].timeMs == 100);
    else assert(reader.error() == SampleError::Empty || reader.error() == SampleError::Incomplete);
    assert(std::equal(stable.bytes.begin(), stable.bytes.begin() + 0x16000, f.bytes.begin()));
    if (cut > 0) assert(!reader.begin(9, 0x16000, 4096) && reader.error() == SampleError::NotErased);
  }
  MemoryFlash full; FlightSampleStore small(full);
  assert(small.begin(1, 0x15000, 4096));
  for (unsigned i = 0; i < 4; ++i) { assert(small.append(sample(i))); assert(small.flush()); }
  const auto beforeFull = full.bytes;
  assert(!small.append(sample(4)) && small.error() == SampleError::Full);
  assert(small.finish() && beforeFull == full.bytes); // No wrap or erase.
  MemoryFlash io; FlightSampleStore fault(io);
  io.failRead = true; assert(!fault.begin(1, 0x15000, 4096)); io.failRead = false;
  assert(fault.begin(1, 0x15000, 4096)); assert(fault.append(sample(0)));
  io.failProgram = true; assert(!fault.flush() && !fault.active() && fault.committedSamples() == 0);
  const auto attempted = io.writes;
  assert(!fault.flush() && io.writes == attempted);
  MemoryFlash ambiguous; FlightSampleStore uncertain(ambiguous);
  assert(uncertain.begin(99, 0x15000, 4096)); assert(uncertain.append(sample(0)));
  ambiguous.failAfterProgram = 4; // Commit is durable although the driver reports failure.
  assert(!uncertain.flush() && !uncertain.active() && uncertain.committedSamples() == 0);
  assert(!uncertain.flush() && ambiguous.writes == 4);
  FlightSampleStore boot(ambiguous);
  assert(boot.readBlock(0x15000, 99, 0, block) && block.samples[0].timeMs == 0);
  std::cout << "Flight sample storage tests passed: codec, bounds, buffering, CRC, full extent, I/O failure and 1025 power cuts.\n";
}
} // namespace test_flight_sample_store

int main() {
  std::puts("  storage_encoding"); std::fflush(stdout);
  test_storage_encoding::run();
  std::puts("  flash_timing"); std::fflush(stdout);
  test_flash_timing::run();
  std::puts("  flight_summary"); std::fflush(stdout);
  test_flight_summary::run();
  std::puts("  flight_sample_store"); std::fflush(stdout);
  test_flight_sample_store::run();
}
