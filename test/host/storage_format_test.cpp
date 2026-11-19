#include "storage/FlightLogDecoder.h"
#include "storage/PadHistory.h"
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
  std::vector<uint8_t> bytes = std::vector<uint8_t>(0x40000, 255);
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
  entry.allocatedBytes = FlashLayout::BlockBytes; assert(directory.update(entry));
  FlightHeaderStore header(flash); assert(header.create(entry.start, config, replay_test::limits,
      SensorMode::Simulation, OutputMode::Virtual, sensors::FrameConvention::LoggedCalibratedBody));
  assert(header.recordStart(10, recovery::OutputKind::Pyro, 1, 0));
  assert(header.recordStart(10, recovery::OutputKind::Servo, 1, 14744)); assert(header.finish(true));
  FlightSampleStore samples(flash); assert(samples.begin(entry.sequence, entry.start, 4096, 4096, FlashLayout::BlockBytes));
  for (unsigned i = 0; i < 5; ++i) {
    FlightSample sample; sample.timeUs = uint64_t(i)*1234000; sample.imuSequence = i+1; sample.phase = recovery::FlightPhase::Descent;
    sample.values[FlightSample::AltitudeM] = 12000.5f + float(i);
    sample.baroFresh = true; sample.values[FlightSample::PressurePa] = 101325.f;
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
  // The data area ends one sector early: the last sector holds the IMU identity policy.
  if (fingerprint != UINT64_C(0xf00e48afbf5b4dc9)) std::cerr << "Storage image fingerprint: 0x" << std::hex << fingerprint << std::dec << '\n';
  assert(fingerprint == UINT64_C(0xf00e48afbf5b4dc9));
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
  // Bulk erase: default implementation erases each sector; the profile records one operation.
  const auto callsBefore = raw.calls;
  assert(flash.eraseRange(0x15000, 3 * 4096) && raw.calls == callsBefore + 3);
  assert(stats.erase.time.count == 2 && stats.erase.time.maxUs == 1500 && stats.erase.bytes == 4096 + 3 * 4096);
  flash.resetTiming();
  assert(flash.read(0, bytes, sizeof(bytes)) && flash.program(0, bytes, sizeof(bytes)) && flash.eraseSector(0));
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
  std::vector<uint8_t> bytes=std::vector<uint8_t>(0x31000,255);
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
  constexpr uint32_t address=0x30400, header=0x30000, checksum=0x13572468;
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
  std::vector<uint8_t> bytes = std::vector<uint8_t>(0x50000, 255); // Ring blocks 0..1 (resized for wrap tests).
  long budget = -1;
  unsigned writes = 0, erasePolls = 0, eraseLatency = 0;
  bool failRead = false, failProgram = false, erasing = false;
  uint32_t eraseAt = 0;
  unsigned failAfterProgram = 0;
  bool read(uint32_t a, void *p, std::size_t n) override {
    assert(!erasing); // Nothing may touch the flash during an erase.
    if (failRead || a > bytes.size() || n > bytes.size() - a) return false;
    memcpy(p, bytes.data() + a, n); return true;
  }
  bool startBlockErase(uint32_t a) override {
    assert(!erasing && a >= FlashLayout::RingBegin && a % FlashLayout::BlockBytes == 0);
    erasing = true; eraseAt = a; erasePolls = eraseLatency; return true;
  }
  bool eraseDone(bool &done) override {
    done = !erasing;
    if (erasing && !erasePolls--) { memset(bytes.data() + eraseAt, 255, FlashLayout::BlockBytes); erasing = false; done = true; }
    return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    assert(!erasing && a >= FlashLayout::RingBegin && a + n <= bytes.size() && n == 256 && a % 256 == 0);
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
// Firmware-equivalent floats: the driver converts codes with these float constants.
float accelFromCode(int code) { return float(code) * 0.976f * 9.80665f / 1000.0f; }
float gyroFromCode(int code) { return float(code) * 70.0f * 0.017453292519943295f / 1000.0f; }
const int AccelCodes[3] = {-32768, 1024, 32767}, GyroCodes[3] = {-1, 0, 28571};
FlightSample sample(uint32_t n, recovery::FlightPhase phase = recovery::FlightPhase::Coast) {
  FlightSample s; s.timeUs = uint64_t(n) * 2100 + 7; s.imuSequence = n + 1; s.phase = phase;
  s.batteryMv = n ? 8240 : UINT16_MAX;
  s.airbrakes.channelMask = 3; s.airbrakes.flags = 3; s.airbrakes.opening = {{2500, 10000, 65535, 65535}};
  for (unsigned i = 0; i < 3; ++i) {
    s.values[FlightSample::AccelX + i] = accelFromCode(AccelCodes[i]);
    s.values[FlightSample::GyroX + i] = gyroFromCode(GyroCodes[i]);
  }
  s.values[FlightSample::AltitudeM] = 12000.5f; // No int16 altitude clipping.
  s.values[FlightSample::VelocityMps] = -123.5f; s.values[FlightSample::VerticalAccelMps2] = 35.25f;
  s.values[FlightSample::RollDegrees] = 1.5f; s.values[FlightSample::PitchDegrees] = -2.25f; s.values[FlightSample::YawDegrees] = 179.99f;
  s.baroFresh = n % 4 == 0; s.baroAgeUs = 300;
  s.values[FlightSample::PressurePa] = 101325.5f; s.values[FlightSample::TemperatureC] = 22.5f;
  return s;
}
flightlog::DecodedBlock decoded(const MemoryFlash &flash, uint32_t address) {
  flightlog::DecodedBlock block; assert(flightlog::decodeBody(flash.bytes.data() + address, block)); return block;
}
void codec() {
  using namespace flightlog;
  // GAP and TIME records, history flags and absolute values at block starts.
  Encoder encoder; encoder.reset(); uint8_t body[BodyBytes];
  auto s = sample(0, recovery::FlightPhase::Pad); s.imuValid = false;
  s.values[FlightSample::AltitudeM] = std::numeric_limits<float>::quiet_NaN();
  s.values[FlightSample::TemperatureC] = std::numeric_limits<float>::quiet_NaN();
  assert(encoder.add(s, true, true) == Encoder::Result::Ok);
  s = sample(3, recovery::FlightPhase::Pad); s.timeUs += 100000; // 2 samples deliberately skipped, 106 ms later.
  assert(encoder.add(s, true, true) == Encoder::Result::Ok);
  s = sample(5); s.timeUs += 100000; assert(encoder.add(s, false, false) == Encoder::Result::Ok); // 1 sample lost.
  assert(encoder.add(s, false, false) == Encoder::Result::Invalid); // Same sequence/time.
  auto far = sample(5 + 70000); assert(encoder.add(far, false, false) == Encoder::Result::Full); // Gap beyond 16 bits.
  encoder.seal(body, 3, 0, 0x20000); DecodedBlock block; assert(decodeBody(body, block) && block.frames == 3);
  assert(!block.frame[0].imuValid && block.frame[0].imu[0] == 0 && block.frame[0].altitudeCm == -8388608);
  assert(block.frame[0].temperatureCenti == INT16_MIN && block.frame[0].history && block.frame[0].state.phase == 0);
  assert(block.frame[1].history && block.frame[1].gapBefore == 2 && block.frame[1].deliberateGap && block.frame[1].sequence == 4);
  assert(block.frame[1].timeUs == sample(3).timeUs + 100000); // Delta 106300 us needs a TIME record.
  assert(!block.frame[2].history && block.frame[2].gapBefore == 1 && !block.frame[2].deliberateGap && block.frame[2].sequence == 6);
  assert(block.frame[2].state.phase == uint8_t(recovery::FlightPhase::Coast) && block.start.phase == 0);
  assert(encoder.add(far, false, false) == Encoder::Result::Ok); encoder.seal(body, 3, 1, 0x20800);
  assert(decodeBody(body, block) && block.frames == 1 && block.frame[0].sequence == 70006 && block.frame[0].gapBefore == 65535);
  assert(block.start.phase == uint8_t(recovery::FlightPhase::Coast) && block.start.batteryMv == 8240); // Standalone snapshot.
  // Any single damaged byte is rejected by the CRC or the record syntax.
  for (unsigned i = 0; i < CrcOffset; ++i) { body[i] ^= 0x10; assert(!decodeBody(body, block)); body[i] ^= 0x10; }
}
void padHistory() {
  // 20 s of PAD at 476 Hz with a barometer reading every 5th sample.
  PadHistory history; history.clear();
  for (uint32_t n = 0; n < 9520; ++n) {
    FlightSample s; s.phase = recovery::FlightPhase::Pad; s.timeUs = uint64_t(n) * 2100; s.imuSequence = n + 1;
    s.baroFresh = n % 5 == 0; s.values[FlightSample::PressurePa] = 100000.f + float(n);
    history.push(s);
  }
  uint64_t previous = 0; unsigned slow = 0, fast = 0, slowBaro = 0; bool first = true;
  const uint64_t newest = uint64_t(9519) * 2100;
  history.forEach([&](const FlightSample &s) {
    if (!first) assert(s.timeUs > previous);
    if (newest - s.timeUs > PadHistory::FastWindowUs) { ++slow; if (!first) assert(s.timeUs - previous >= PadHistory::SlowSpacingUs); slowBaro += s.baroFresh; }
    else { ++fast; if (fast > 1) assert(s.timeUs - previous == 2100); }
    first = false; previous = s.timeUs; return true;
  });
  assert(slow == PadHistory::SlowCapacity && fast == 239 && history.size() == slow + fast);
  assert(slowBaro == slow); // Each 10 Hz sample carries the newest barometer reading.
  std::cout << "Pre-launch history passed: 0.5 s full rate, 10 Hz for 10 s, barometer carried.\n";
}
void run() {
  codec(); padHistory();
  MemoryFlash flash; FlightSampleStore writer(flash);
  const auto pristine = flash.bytes;
  constexpr uint32_t Block = FlashLayout::BlockBytes;
  assert(!writer.begin(0, 0x30000, 0, 4096, Block));
  assert(!writer.begin(1, 0x20000, 0, 4096, Block)); // Below the ring.
  assert(!writer.begin(1, 0x30800, 0, 4096, Block)); // Not on a 64 KiB block.
  assert(!writer.begin(1, 0x30000, 0, 4096, FlashLayout::RingBytes + Block));
  assert(!writer.begin(1, 0xFFFFF000, 0, 4096, Block));
  assert(flash.bytes == pristine && flash.writes == 0);
  assert(writer.begin(7, 0x30000, 0, 4096, Block));
  assert(!writer.begin(8, 0x40000, 0, 4096, Block) && writer.error() == SampleError::Active);
  // Samples stay in RAM until a block is full; then 7 body pages and the commit page.
  uint32_t n = 0;
  while (!writer.committedSamples()) assert(writer.append(sample(n++)));
  const uint32_t first = writer.committedSamples();
  assert(first >= 50 && first <= 62 && flash.writes == 8 && writer.committedBytes() == 2048 && writer.bufferedSamples() == 1);
  RawBlock raw; assert(writer.readBlock(0x30000, 7, 0, raw) && raw.frames == first && raw.firstSequence == 1 && raw.firstTimeUs == 7);
  const auto block = decoded(flash, 0x30000);
  for (unsigned i = 0; i < block.frames; ++i) {
    const auto &f = block.frame[i];
    assert(f.timeUs == sample(i).timeUs && f.sequence == i + 1 && f.imuValid && f.estimates && !f.history && !f.gapBefore);
    for (unsigned a = 0; a < 3; ++a) assert(f.imu[a] == AccelCodes[a] && f.imu[3 + a] == GyroCodes[a]); // Exact raw codes.
    assert(f.altitudeCm == 1200050 && f.velocity == -6175 && f.verticalAccel == 3525);
    assert(f.angles[0] == 150 && f.angles[1] == -225 && f.angles[2] == 17999);
    assert(f.baroFresh == (i % 4 == 0));
    if (f.baroFresh) assert(f.pressure64 == 6484832 && f.temperatureCenti == 2250 && f.baroAgeUs == 300);
    assert(f.state.batteryMv == (i ? 8240 : UINT16_MAX) && f.state.airbrakes.opening[1] == 10000 && f.state.airbrakes.flags == 3);
  }
  // Golden little-endian layout, independent of the reference decoder.
  const uint8_t *b = flash.bytes.data() + 0x30000;
  assert(b[0] == 'P' && b[1] == 'S' && b[2] == 'B' && b[3] == 'K' && b[4] == 4 && b[5] == 0 && b[6] == 0x00 && b[7] == 0x07);
  assert(b[8] == 7 && b[16] == 0 && b[20] == 0x00 && b[21] == 0x00 && b[22] == 0x03 && b[24] == 7 && b[32] == 1);
  assert(b[40] == 0 && b[42] == 0xFF && b[43] == 0xFF && b[47] == 0); // Default state before the first record.
  assert(b[64] == 0x60 && b[65] == 2 && b[66] == 0);                  // STATUS: COAST.
  assert(b[67] == 0xA0 && b[68] == 3 && b[71] == 3 && b[72] == 0xC4 && b[73] == 0x09 && b[74] == 0x10 && b[75] == 0x27); // AIRBRAKE.
  assert(b[76] == 0x40 && b[77] == 0x2C && b[78] == 0x01);           // BARO, age 300 us.
  assert(b[84] == 0x21 && b[85] == 0 && b[86] == 0 && b[87] == 0x00 && b[88] == 0x80); // FRAME with estimates, ax = -32768.
  assert(b[0x800 - 256] == 'P' && b[0x700] == 'P' && b[0x701] == 'S' && b[0x702] == 'C' && b[0x703] == 'M'); // Commit page.
  const auto stable = flash;
  assert(!writer.append(sample(n - 1)) && writer.error() == SampleError::Timestamp);
  auto invalid = sample(n); invalid.phase = static_cast<recovery::FlightPhase>(99);
  assert(!writer.append(invalid));
  auto missing = sample(n); missing.values[0] = std::numeric_limits<float>::infinity();
  assert(writer.append(missing)); assert(writer.finish());
  assert(writer.readBlock(0x30800, 7, 1, raw) && raw.frames == 2 && raw.firstSequence == first + 1);
  const auto tail = decoded(flash, 0x30800);
  assert(tail.frame[1].sequence == n + 1 && !tail.frame[1].imuValid && tail.start.batteryMv == 8240);
  assert(!writer.append(sample(n + 1)) && writer.error() == SampleError::Inactive);
  assert(!writer.begin(8, 0x30000, 0, 4096, Block) && writer.error() == SampleError::NotErased);
  raw.frames = 77;
  assert(!writer.readBlock(0x30000, 8, 0, raw) && writer.error() == SampleError::Identity && raw.frames == 77);
  assert(!writer.readBlock(0x30000, 7, 1, raw) && writer.error() == SampleError::Identity);
  assert(!writer.readBlock(0x31000, 7, 2, raw) && writer.error() == SampleError::Empty);
  std::copy(flash.bytes.begin() + 0x30000, flash.bytes.begin() + 0x30800, flash.bytes.begin() + 0x40000);
  assert(!writer.readBlock(0x40000, 7, 0, raw) && writer.error() == SampleError::Identity);
  flash.bytes[0x30024] ^= 1;
  assert(!writer.readBlock(0x30000, 7, 0, raw) && writer.error() == SampleError::Corrupt);
  // Every byte boundary of a block write, retaining the preceding block unchanged.
  for (long cut = 0; cut <= 2048; ++cut) {
    MemoryFlash f = stable; FlightSampleStore w(f);
    // Separate allocated extent: deliberately leave the prior committed block intact.
    assert(w.begin(8, 0x40000, 0, 4096, Block));
    assert(w.append(sample(100))); f.budget = cut;
    try { w.flush(); } catch (const Cut &) {}
    f.budget = -1;
    FlightSampleStore reader(f); RawBlock recovered;
    assert(reader.readBlock(0x30000, 7, 0, recovered) && recovered.frames == first);
    const bool valid = reader.readBlock(0x40000, 8, 0, recovered);
    // Committed once the commit magic, CRC and complement (12 bytes) follow the body; the rest is FF.
    if (valid) assert(recovered.frames == 1 && recovered.firstSequence == 101 && cut >= 1792 + 12);
    else assert(cut < 1792 + 12 && (reader.error() == SampleError::Empty || reader.error() == SampleError::Incomplete));
    assert(std::equal(stable.bytes.begin(), stable.bytes.begin() + 0x40000, f.bytes.begin()));
    if (cut > 0) assert(!reader.begin(9, 0x40000, 0, 4096, Block) && reader.error() == SampleError::NotErased);
  }
  // A 4 KiB extent holds two blocks; then Full, with no wrap or erase.
  MemoryFlash full; FlightSampleStore small(full);
  assert(small.begin(1, 0x30000, 0, 4096, Block));
  uint32_t m = 0;
  while (small.append(sample(m))) ++m;
  assert(small.error() == SampleError::Full && small.committedBytes() == 4096 && small.committedSamples() == m);
  const auto beforeFull = full.bytes;
  assert(!small.append(sample(m + 1)) && small.error() == SampleError::Full);
  assert(small.finish() && beforeFull == full.bytes);
  MemoryFlash io; FlightSampleStore fault(io);
  io.failRead = true; assert(!fault.begin(1, 0x30000, 0, 4096, Block)); io.failRead = false;
  assert(fault.begin(1, 0x30000, 0, 4096, Block)); assert(fault.append(sample(0)));
  io.failProgram = true; assert(!fault.flush() && !fault.active() && fault.committedSamples() == 0);
  const auto attempted = io.writes;
  assert(!fault.flush() && io.writes == attempted);
  MemoryFlash ambiguous; FlightSampleStore uncertain(ambiguous);
  assert(uncertain.begin(99, 0x30000, 0, 4096, Block)); assert(uncertain.append(sample(0)));
  ambiguous.failAfterProgram = 8; // Commit is durable although the driver reports failure.
  assert(!uncertain.flush() && !uncertain.active() && uncertain.committedSamples() == 0);
  assert(!uncertain.flush() && ambiguous.writes == 8);
  FlightSampleStore boot(ambiguous);
  assert(boot.readBlock(0x30000, 99, 0, raw) && raw.frames == 1 && raw.firstTimeUs == 7);
  // A flight in the last ring block continues in block 0.
  MemoryFlash ring; ring.bytes.resize(FlashLayout::Capacity, 255); FlightSampleStore wrap(ring);
  const uint32_t last = FlashLayout::blockAddress(FlashLayout::RingBlocks - 1);
  assert(wrap.begin(5, last, 0, 2 * Block, 2 * Block));
  assert(wrap.address(31) == last + 31 * 2048 && wrap.address(32) == FlashLayout::RingBegin);
  uint32_t k = 0;
  while (wrap.committedSamples() == 0 || wrap.committedBytes() < 33 * 2048) assert(wrap.append(sample(k++)));
  assert(wrap.readBlock(FlashLayout::RingBegin, 5, 32, raw) && raw.firstSequence > 1);
  // Growth: added blocks are erased asynchronously. The whole chip is busy during an erase, so
  // every sealed block waits in RAM, then is written once the erase has finished.
  MemoryFlash slow; slow.eraseLatency = 1000000; FlightSampleStore grow(slow);
  assert(grow.begin(6, 0x30000, 0, 4 * Block, Block) && grow.extend(1) && grow.service() && slow.erasing);
  k = 0;
  for (; k < 200; ++k) assert(grow.append(sample(k)));
  assert(grow.committedBytes() == 0 && slow.erasing && grow.erasing() && grow.service() && slow.erasing);
  slow.erasePolls = 0; assert(grow.service() && !slow.erasing && !grow.erasing());
  assert(grow.committedBytes() == 3 * 2048 && grow.committedSamples() == k - grow.bufferedSamples());
  assert(grow.finish());
  // More than 16 sealed blocks during one erase: the writer waits for the erase, losing nothing.
  MemoryFlash busy; busy.eraseLatency = 5000; FlightSampleStore wait(busy);
  assert(wait.begin(7, 0x30000, 0, 4 * Block, Block) && wait.extend(1) && wait.service() && busy.erasing);
  for (k = 0; k < 1200; ++k) assert(wait.append(sample(k)));
  assert(!busy.erasing && wait.committedBytes() >= 16 * 2048 && wait.finish() && wait.committedSamples() == k);
  std::cout << "Flight sample storage tests passed: v4 codec, raw codes, bounds, CRC, full extent, I/O failure, 2049 power cuts, ring wrap and asynchronous growth.\n";
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
