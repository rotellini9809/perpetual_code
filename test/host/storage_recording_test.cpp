#include "../fixtures/FlightLogSupport.h"
#include <cmath>
#include <cstdio>
#include "storage/FlightRecorder.h"
#include "../fixtures/ReplayFixture.h"
#include <algorithm>
#include <iostream>
#include "storage/QueuedRecorder.h"
#include <string>
#include <atomic>
#include <thread>

// Each namespace isolates one component scenario and its fault model.
namespace test_flight_recorder {
using namespace storage;
using namespace replay_test;
struct PowerCut {};
struct MemoryDevice : FlashDevice {
  std::vector<uint8_t> bytes = std::vector<uint8_t>(0x35000, 255);
  long budget = -1;
  unsigned programs = 0, erases = 0, failAfterProgram = 0;
  bool failErase = false, failRead = false;
  void tick() { if (!budget) throw PowerCut{}; if (budget > 0) --budget; }
  bool read(uint32_t a, void *p, std::size_t n) override {
    if (failRead || a > bytes.size() || n > bytes.size()-a) return false;
    memcpy(p, bytes.data()+a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    assert(a >= 0x5000 && a+n <= bytes.size() && a%256 == 0 && n == 256);
    ++programs; const auto *source = static_cast<const uint8_t *>(p);
    for (std::size_t i = 0; i < n; ++i) { tick(); assert((bytes[a+i] & source[i]) == source[i]); bytes[a+i] &= source[i]; }
    return programs != failAfterProgram;
  }
  bool eraseSector(uint32_t a) override {
    assert(a >= 0x5000 && a+4096 <= bytes.size() && a%4096 == 0);
    if (a >= 0x15000) { // A durable reservation must exist before erasing any flight sector.
      FlightDirectory proof(*this); assert(proof.mount()); bool owned = false;
      for (std::size_t i = 0; i < proof.count(); ++i) {
        const auto &e = proof.entries()[i];
        if (e.status == FlightStatus::Recording && a >= e.start && a-e.start < e.allocatedBytes) owned = true;
      }
      assert(owned);
    }
    ++erases; if (failErase) return false;
    for (unsigned i = 0; i < 4096; ++i) { tick(); bytes[a+i] = 255; }
    return true;
  }
};
bool begin(FlightRecorder &r) {
  return r.begin(fixture(), limits, 4096, SensorMode::Simulation, OutputMode::Virtual, sensors::FrameConvention::LoggedCalibratedBody);
}
// Airborne samples go straight to blocks (PAD would stay in the pre-launch history).
FlightSample sample(uint32_t t) {
  FlightSample s; s.timeUs = uint64_t(t) * 1000; s.imuSequence = t + 1; s.phase = recovery::FlightPhase::Coast;
  s.values[FlightSample::AltitudeM] = float(t); return s;
}
void format(MemoryDevice &f) { FlightDirectory d(f); assert(d.format()); }
std::vector<uint8_t> encoded(const recovery::Configuration &c) {
  std::vector<uint8_t> bytes(ConfigurationStore::MaxPayload); std::size_t size;
  assert(ConfigurationStore::encode(c, bytes.data(), bytes.size(), size)); bytes.resize(size); return bytes;
}
void run() {

  MemoryDevice empty; FlightRecorder missing(empty);
  const auto unknown = empty.bytes;
  assert(!missing.initialize() && missing.error() == RecorderError::Directory && empty.bytes == unknown);
  format(empty);
  MemoryDevice flash = empty; FlightRecorder recorder(flash); assert(recorder.initialize());
  const auto protectedBytes = std::vector<uint8_t>(flash.bytes.begin(), flash.bytes.begin()+0x5000);
  const auto before = flash.bytes;
  assert(!recorder.begin(fixture(), limits, UINT32_MAX-4095, SensorMode::Simulation, OutputMode::Virtual,
                         sensors::FrameConvention::LoggedCalibratedBody) && flash.bytes == before);
  assert(begin(recorder) && recorder.current()->sequence == 1);
  assert(recorder.current()->committedBytes == 4096 && recorder.current()->allocatedBytes == 8192);
  assert(!begin(recorder) && recorder.error() == RecorderError::Busy);
  RecordingInfo info; assert(!recorder.inspect(1, info) && recorder.error() == RecorderError::Busy);
  const auto programs = flash.programs;
  // Samples accumulate in the RAM block; a full block is written by the append that overflows it.
  uint32_t perBlock = 0;
  while (!recorder.committedSamples()) assert(recorder.append(sample(perBlock++)));
  --perBlock; assert(perBlock == 61 && recorder.committedSamples() == perBlock); // 28-byte FRAMEs (+ one STATUS).
  assert(flash.programs == programs + 8);
  assert(recorder.recordStart(10, recovery::OutputKind::Pyro, 1, 0) && recorder.service());
  assert(recorder.current()->committedBytes == 4096); // Directory progress is published only at close.
  for (unsigned i = perBlock + 1; i < perBlock + 9; ++i) assert(recorder.append(sample(i)));
  assert(!recorder.finish(false) && recorder.error() == RecorderError::ProducerActive);
  assert(recorder.recordStart(10, recovery::OutputKind::Servo, 1, 250) && recorder.finish(true));
  assert(recorder.current()->status == FlightStatus::Completed);
  assert(recorder.inspect(1, info) && info.samples == perBlock + 9 && info.entry.committedBytes == 8192);
  assert(recorder.metadata()->starts[0].state == StartState::Recorded && recorder.metadata()->starts[0].timeMs == 0);
  RawBlock block; assert(recorder.readBlock(1, 1, block) && block.firstSequence == perBlock + 1 && block.frames == 9);
  assert(block.firstTimeUs == uint64_t(perBlock) * 1000);
  block.frames = 99; assert(!recorder.readBlock(1, 2, block) && block.frames == 99);
  assert(!recorder.inspect(999, info) && recorder.error() == RecorderError::StaleSelection && !recorder.metadata());
  const auto retained = std::vector<uint8_t>(flash.bytes.begin()+0x15000, flash.bytes.begin()+0x17000);
  assert(begin(recorder) && recorder.current()->start == 0x17000 && recorder.current()->sequence == 2);
  assert(std::equal(retained.begin(), retained.end(), flash.bytes.begin()+0x15000));
  // A 4 KiB budget holds two blocks; the sample that cannot fit is refused and the log is full.
  uint32_t stored = 0;
  while (recorder.append(sample(stored))) { assert(recorder.service()); ++stored; }
  assert(recorder.error() == RecorderError::Full && recorder.samplesFull() && stored == 2 * perBlock);
  assert(!recorder.append(sample(stored + 1)) && recorder.error() == RecorderError::Full);
  assert(recorder.recordStart(10, recovery::OutputKind::Pyro, 1, 99) && recorder.finish(true));
  assert(recorder.current()->status == FlightStatus::StoppedFull);
  assert(recorder.inspect(2, info) && info.samples == stored && recorder.metadata()->starts[0].timeMs == 99);
  assert(std::equal(protectedBytes.begin(), protectedBytes.end(), flash.bytes.begin()));
  // Recover a committed block beyond the last directory checkpoint, plus ignore a torn tail.
  MemoryDevice crash = empty; FlightRecorder running(crash); assert(running.initialize() && begin(running));
  for (unsigned i = 0; i <= perBlock; ++i) assert(running.append(sample(i))); // Writes block 0.
  for (unsigned i = perBlock + 1; i < 2 * perBlock; ++i) assert(running.append(sample(i)));
  crash.budget = 100; // The append that would write block 1 loses power mid-block.
  try { running.append(sample(2 * perBlock)); } catch (const PowerCut &) {}
  crash.budget = -1; FlightRecorder boot(crash); assert(boot.initialize());
  assert(!boot.active() && boot.directory().entries()[0].status == FlightStatus::Interrupted);
  assert(boot.inspect(1, info) && info.samples == perBlock && info.entry.committedBytes == 6144);
  const auto afterRecovery = crash.bytes; assert(boot.initialize() && crash.bytes == afterRecovery);
  // Every byte cut of full-block flush and final directory close; metadata commit may lag data.
  for (unsigned cut = 0; cut <= 3072; ++cut) { // Block (2048) + summary (512) + directory slot (512).
    MemoryDevice f = empty; FlightRecorder r(f); assert(r.initialize() && begin(r));
    assert(r.append(sample(0))); f.budget = cut;
    try { r.finish(true); } catch (const PowerCut &) {}
    f.budget = -1; FlightRecorder recovered(f); assert(recovered.initialize());
    const auto &entry = recovered.directory().entries()[0];
    assert(entry.status == FlightStatus::Completed || entry.status == FlightStatus::Interrupted);
    assert(recovered.inspect(entry.sequence, info) && info.samples <= 1);
    if (info.samples) assert(recovered.readBlock(entry.sequence, 0, block) && block.firstSequence == 1 && block.firstTimeUs == 0);
  }
  // Reservation cuts, erase cuts, header cuts and the header-ready checkpoint.
  const unsigned preparationBytes = 512 + 8192 + 768 + 512;
  for (unsigned cut = 0; cut <= preparationBytes; cut += 17) {
    MemoryDevice f = empty; FlightRecorder r(f); assert(r.initialize()); f.budget = cut;
    try { begin(r); } catch (const PowerCut &) {}
    f.budget = -1; FlightRecorder recovered(f); assert(recovered.initialize());
    assert(!recovered.active());
    if (recovered.directory().count()) {
      const auto &e = recovered.directory().entries()[0]; assert(e.status == FlightStatus::Interrupted);
      assert(e.committedBytes == 0 || e.committedBytes == 4096);
    }
  }
  // Old metadata in a reused/unprepared extent must not be mistaken for the new reservation.
  MemoryDevice stale = empty;
  {
    FlightDirectory d(stale); assert(d.mount());
    FlightEntry e; e.sequence = 1; e.start = 0x15000; e.allocatedBytes = 8192;
    assert(d.update(e));
    // Existing bytes deliberately look like a valid old header, before erases have begun.
    FlightHeaderStore old(stale);
    assert(old.create(e.start, fixture(), limits, SensorMode::Simulation, OutputMode::Virtual, sensors::FrameConvention::LoggedCalibratedBody));
  }
  FlightRecorder staleBoot(stale); assert(staleBoot.initialize());
  assert(staleBoot.directory().entries()[0].committedBytes == 0);
  assert(!staleBoot.inspect(1, info) && staleBoot.error() == RecorderError::EmptyRecording);
  // A committed damaged block must be reported, rather than shortening published data.
  MemoryDevice damaged = flash; damaged.bytes[0x15000+4096+36] ^= 1;
  FlightRecorder damagedReader(damaged); assert(damagedReader.initialize());
  assert(!damagedReader.inspect(1, info) && damagedReader.error() == RecorderError::Corrupt);
  // I/O errors leave a durable Recording entry; explicit reinitialize reconciles it.
  MemoryDevice faulty = empty; FlightRecorder fault(faulty); assert(fault.initialize());
  faulty.failErase = true; assert(!begin(fault) && !fault.initialized());
  faulty.failErase = false; assert(fault.initialize());
  assert(fault.directory().entries()[0].status == FlightStatus::Interrupted);
  MemoryDevice uncertain = empty; FlightRecorder u(uncertain); assert(u.initialize() && begin(u));
  assert(u.append(sample(0))); uncertain.failAfterProgram = uncertain.programs + 12; // 8 block + 2 summary + 2 directory pages.
  assert(!u.finish(true) && !u.initialized());
  FlightRecorder reconciled(uncertain); assert(reconciled.initialize());
  assert(reconciled.directory().entries()[0].status == FlightStatus::Completed);
  // Exhausted free extents do not reclaim any retained flight implicitly.
  MemoryDevice full = empty;
  {
    FlightDirectory d(full); assert(d.mount());
    FlightEntry e; e.sequence = 1; e.start = FlashLayout::DataBegin;
    e.allocatedBytes = FlashLayout::DataEnd - FlashLayout::DataBegin;
    assert(d.update(e)); e.status = FlightStatus::Interrupted; assert(d.update(e));
  }
  FlightRecorder bounded(full); assert(bounded.initialize());
  const auto fullBytes = full.bytes;
  assert(!begin(bounded) && bounded.error() == RecorderError::NoSpace && full.bytes == fullBytes);
  std::cout << "Flight recorder tests passed: ownership before erase, lifecycle, recovery, stopped-full, stable reads, 3073 close cuts and preparation cuts.\n";
}
} // namespace test_flight_recorder

namespace test_queued_recorder {
using namespace storage;
using namespace replay_test;
namespace {
struct Device : FlashDevice {
  std::vector<uint8_t> bytes = std::vector<uint8_t>(FlashLayout::Capacity, 255);
  const std::thread::id producer = std::this_thread::get_id();
  std::atomic<bool> workerOnly{false}, failProgram{false}, failErase{false}, holdErase{false}, erasing{false};
  std::atomic<unsigned> operations{0};
  void touch() { assert(!workerOnly.load() || std::this_thread::get_id() != producer); ++operations; }
  bool read(uint32_t a, void *p, std::size_t n) override {
    touch(); assert(a <= bytes.size() && n <= bytes.size()-a); memcpy(p, bytes.data()+a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    touch(); if (failProgram.load()) return false;
    assert(a <= bytes.size() && n <= bytes.size()-a); const auto *in = static_cast<const uint8_t *>(p);
    for (std::size_t i = 0; i < n; ++i) { assert((bytes[a+i] & in[i]) == in[i]); bytes[a+i] &= in[i]; }
    return true;
  }
  bool eraseSector(uint32_t a) override { touch(); erasing.store(true); while (holdErase.load()) std::this_thread::yield(); if (failErase.load()) return false; assert(a%4096 == 0 && a+4096 <= bytes.size()); memset(bytes.data()+a, 255, 4096); return true; }
};
struct Runner {
  QueuedRecorder &recorder; std::atomic<bool> paused{true}, pauseAcknowledged{false}, quit{false}; std::thread thread;
  explicit Runner(QueuedRecorder &r) : recorder(r), thread([this] {
    recorder.workerStarted();
    while (!quit.load()) {
      if (paused.load()) { pauseAcknowledged.store(true); std::this_thread::yield(); continue; }
      pauseAcknowledged.store(false);
      if (!recorder.poll()) std::this_thread::yield();
    }
  }) { while (!recorder.online()) std::this_thread::yield(); }
  ~Runner() { assert(!recorder.busy()); quit.store(true); thread.join(); }
  void pause() { paused.store(true); while (!pauseAcknowledged.load()) std::this_thread::yield(); }
};
recovery::Configuration allOutputs() {
  auto c = fixture(); c.ruleCount = 1; c.rules[0].assignmentCount = 8;
  for (unsigned i = 0; i < 4; ++i) {
    c.pyros[i].pulseMs = 100; c.servos[i] = c.servos[0];
    c.rules[0].assignments[i].kind = recovery::OutputKind::Pyro; c.rules[0].assignments[i].channel = uint8_t(i+1);
    c.rules[0].assignments[i+4].kind = recovery::OutputKind::Servo; c.rules[0].assignments[i+4].channel = uint8_t(i+1);
  }
  return c;
}
// Airborne 15-byte FRAMEs (no estimates): 114 per block.
FlightSample sample(unsigned i) {
  FlightSample s; s.timeUs = uint64_t(i) * 5000; s.imuSequence = i + 1; s.phase = recovery::FlightPhase::Coast;
  for (unsigned a = 0; a < 6; ++a) s.values[a] = 0;
  s.values[0] = float(i); return s;
}
bool begin(QueuedRecorder &q, const recovery::Configuration &c, uint32_t bytes = 12288) {
  return q.begin(c, limits, bytes, SensorMode::Simulation, OutputMode::Virtual, sensors::FrameConvention::LoggedCalibratedBody);
}
}
void run() {
  Device flash; FlightRecorder writer(flash); assert(writer.initializeBlank());
  QueuedRecorder queued(writer); const auto config = allOutputs();
  const auto protectedBytes = std::vector<uint8_t>(flash.bytes.begin(), flash.bytes.begin()+FlashLayout::DirectoryA);
  assert(!begin(queued, config) && queued.error() == RecorderError::NotInitialized);
  Runner runner(queued); assert(begin(queued, config)); flash.workerOnly.store(true);
  const auto before = flash.operations.load();
  const unsigned capacity = QueuedRecorder::SampleCapacity;
  for (unsigned i = 0; i < capacity; ++i) assert(queued.append(sample(i)));
  assert(!queued.append(sample(capacity)) && queued.error() == RecorderError::Backpressure);
  for (const auto &a : config.rules[0].assignments) assert(queued.recordStart(10, a.kind, a.channel, 0));
  assert(queued.recordStart(10, recovery::OutputKind::Pyro, 1, 0));
  assert(!queued.recordStart(10, recovery::OutputKind::Pyro, 1, 1) && queued.error() == RecorderError::Header);
  assert(!queued.recordStart(99, recovery::OutputKind::Pyro, 1, 0));
  assert(queued.service() && queued.pendingSamples() == capacity && queued.peakSamples() == capacity);
  assert(!queued.finish(false) && queued.error() == RecorderError::ProducerActive);
  assert(flash.operations.load() == before); // Enqueue, notification and health checks perform no flash access.
  RecordingSummary closing; closing.droppedSamples=1;
  // A stopped worker cannot delay close submission or release native ownership.
  assert(queued.finish(true,closing) && queued.busy() && !queued.active());
  assert(flash.operations.load() == before && queued.pendingSamples() == capacity);
  assert(!begin(queued,config) && queued.error()==RecorderError::Busy);
  runner.paused.store(false); while (queued.busy()) std::this_thread::yield(); flash.workerOnly.store(false);
  assert(!queued.busy() && queued.committedSamples() == capacity && queued.backgroundDroppedSamples() == 0);
  assert(queued.current()->status == FlightStatus::Completed);
  RecordingInfo info; assert(writer.inspect(1, info) && info.samples == capacity && info.entry.status == FlightStatus::Completed);
  assert(info.summaryState==SummaryState::Valid && info.summary.savedSamples==capacity && info.summary.droppedSamples==1 && info.summary.queuePeak==capacity && info.summary.queueCapacity==capacity);
  for (const auto &a : writer.metadata()->starts) assert(a.state == StartState::Recorded && a.timeMs == 0);
  RawBlock block;
  for (unsigned i = 0, ordinal = 0; i < 2; ++i) {
    assert(writer.readBlock(1, i, block)); const auto decoded = decodeRaw(block);
    for (unsigned j = 0; j < decoded.frames; ++j, ++ordinal) {
      const auto &f = decoded.frame[j];
      assert(f.timeUs == ordinal * 5000 && f.sequence == ordinal + 1 && f.imuValid && !f.estimates);
      assert(f.imu[0] == int16_t(std::lround(ordinal / flightlog::AccelMps2PerCode)));
    }
    if (i == 1) assert(ordinal == capacity);
  }
  // A 4 KiB budget holds two 114-frame blocks; samples queued past that are dropped by the worker.
  runner.pause(); assert(begin(queued, config, 4096)); flash.workerOnly.store(true);
  for (unsigned i = 0; i < capacity; ++i) assert(queued.append(sample(i)));
  runner.paused.store(false); while (queued.pendingSamples()) std::this_thread::yield(); runner.pause();
  for (unsigned i = capacity; i < 2 * capacity; ++i) assert(queued.append(sample(i)));
  runner.paused.store(false);
  while (queued.pendingSamples() || !queued.samplesFull()) std::this_thread::yield();
  assert(!queued.append(sample(2 * capacity)) && queued.error() == RecorderError::Full);
  assert(queued.recordStart(10, recovery::OutputKind::Pyro, 1, 12345));
  assert(queued.finish(true,closing)); while (queued.busy()) std::this_thread::yield(); flash.workerOnly.store(false);
  assert(queued.samplesFull() && queued.committedSamples() == 228 && queued.backgroundDroppedSamples() == 2 * capacity - 228);
  assert(writer.inspect(2, info) && info.entry.status == FlightStatus::StoppedFull && writer.metadata()->starts[0].timeMs == 12345);
  assert(info.summaryState==SummaryState::Valid && info.summary.droppedSamples==1 + 2 * capacity - 228 && info.summary.reason==RecordingEnd::StorageFull);
  runner.pause(); assert(begin(queued, config)); flash.failProgram.store(true); flash.workerOnly.store(true);
  for (unsigned i = 0; i < 9; ++i) assert(queued.append(sample(i)));
  assert(queued.finish(true)); // Acknowledgment of the request is not persistence success.
  runner.paused.store(false); while (queued.busy()) std::this_thread::yield();
  assert(!queued.initialized() && queued.error() == RecorderError::Sample && !queued.service());
  assert(queued.failure().error==RecorderError::Sample && std::string(queued.failure().cause)=="Io");
  flash.workerOnly.store(false); flash.failProgram.store(false); assert(writer.initialize());
  assert(writer.inspect(3, info) && info.entry.status == FlightStatus::Interrupted && info.samples == 0);
  runner.pause(); assert(begin(queued, config)); flash.workerOnly.store(true);
  for (unsigned i = 0; i < 10; ++i) assert(queued.append(sample(i)));
  const auto beforeAbort=flash.operations.load();
  assert(queued.finish(true)); queued.abort(); queued.abort();
  assert(queued.busy() && !queued.active() && flash.operations.load()==beforeAbort);
  runner.paused.store(false); while (queued.busy()) std::this_thread::yield(); flash.workerOnly.store(false);
  assert(!queued.initialized() && queued.error() == RecorderError::NotRecording && writer.initialize());
  assert(writer.inspect(4, info) && info.entry.status == FlightStatus::Interrupted && info.samples == 0);
  runner.pause(); assert(begin(queued, config)); flash.workerOnly.store(true);
  const auto discardedSequence=queued.current()->sequence;
  const auto dataBeforeDiscard=std::vector<uint8_t>(flash.bytes.begin()+FlashLayout::DataBegin,flash.bytes.end());
  for (unsigned i=0;i<64;++i) assert(queued.append(sample(i))); // Less than a block: RAM only.
  assert(queued.recordStart(10,recovery::OutputKind::Pyro,1,0));
  const auto beforeDiscard=flash.operations.load();
  assert(queued.finish(true,RecordingSummary{},RecordingRetention::Discard));
  assert(queued.busy() && !queued.active() && flash.operations.load()==beforeDiscard);
  assert(!begin(queued,config) && queued.error()==RecorderError::Busy);
  runner.paused.store(false); while (queued.busy()) std::this_thread::yield(); flash.workerOnly.store(false);
  assert(queued.current()->status==FlightStatus::Invalidated && queued.committedSamples()==0);
  assert(std::equal(dataBeforeDiscard.begin(),dataBeforeDiscard.end(),flash.bytes.begin()+FlashLayout::DataBegin));
  assert(writer.initialize() && !writer.inspect(discardedSequence,info) && writer.error()==RecorderError::StaleSelection);
  runner.pause(); assert(begin(queued,config)); const auto failedDiscard=queued.current()->sequence;
  flash.failProgram.store(true); flash.workerOnly.store(true);
  assert(queued.finish(true,RecordingSummary{},RecordingRetention::Discard));
  runner.paused.store(false); while (queued.busy()) std::this_thread::yield();
  assert(!queued.initialized() && queued.error()==RecorderError::Directory);
  flash.workerOnly.store(false); flash.failProgram.store(false);
  assert(writer.initialize() && writer.inspect(failedDiscard,info) && info.entry.status==FlightStatus::Interrupted);
  assert(std::equal(protectedBytes.begin(), protectedBytes.end(), flash.bytes.begin()));
  // Exercise concurrent queue wraparound independently of flash speed.
  RecordingQueue<uint32_t, 64> queue; const uint32_t count = 100000;
  std::thread consumer([&] { for (uint32_t expected = 0; expected < count; ++expected) {
    uint32_t value; while (!queue.pop(value)) std::this_thread::yield(); assert(value == expected);
  }});
  for (uint32_t i = 0; i < count; ++i) { uint32_t occupancy; while (!queue.push(i, occupancy)) std::this_thread::yield(); assert(occupancy <= 64); }
  consumer.join(); assert(queue.empty());
  puts("Queued recorder tests passed: producer performs no flash I/O, concurrent FIFO, starts, drain, fullness, faults, abort, asynchronous discard and recovery.");
}

void testAsyncPreparation() {
  Device flash; FlightRecorder writer(flash); assert(writer.initializeBlank());
  QueuedRecorder q(writer); Runner runner(q); auto config=allOutputs();
  auto begin=[&] { return q.beginAsync(config,limits,12288,SensorMode::Hardware,OutputMode::Physical,sensors::FrameConvention::SensorAxes,FlightCalibration{}); };
  auto drain=[&] { runner.paused.store(false); while(q.busy()) std::this_thread::yield(); runner.pause(); };
  flash.workerOnly.store(true);
  const auto before=flash.operations.load(); assert(begin());
  assert(q.busy() && q.preparing() && !q.active() && !q.current() && q.initialized());
  assert(flash.operations.load()==before && !q.activatePrepared());
  runner.paused.store(false); while(q.preparing()) std::this_thread::yield(); runner.pause();
  assert(q.busy() && !q.active() && !q.current() && q.activatePrepared() && q.active());
  assert(q.current()->sequence==1);
  assert(q.finish(true,RecordingSummary{},RecordingRetention::Discard)); drain();
  // Cancellation before the worker runs never starts flash preparation.
  const auto cancelledBefore=flash.operations.load(); assert(begin()); q.abort(); drain();
  assert(q.initialized() && q.error()==RecorderError::None && flash.operations.load()==cancelledBefore);
  // A blocked erase cannot delay core 0 cancellation or diagnostic snapshots.
  flash.holdErase.store(true); flash.erasing.store(false); assert(begin());
  runner.paused.store(false); while(!flash.erasing.load()) std::this_thread::yield();
  const auto blocked=flash.operations.load();
  for(unsigned i=0;i<100;++i) assert(q.busy() && q.preparing() && !q.active() && !q.current() && q.initialized());
  q.abort(); assert(q.busy() && !q.active() && flash.operations.load()==blocked);
  flash.holdErase.store(false); drain();
  assert(q.initialized() && q.error()==RecorderError::None && writer.directory().retainedCount()==0);
  // Failed preparation never becomes active and exposes the error after acknowledgment.
  flash.failErase.store(true); assert(begin()); drain();
  assert(!q.active() && !q.initialized() && q.error()!=RecorderError::None && !q.activatePrepared());
  flash.workerOnly.store(false); flash.failErase.store(false); assert(writer.initialize());
  puts("Async preparation passed: worker-only flash, activation barrier, cancellation before/during erase, fault acknowledgment.");
}
} // namespace test_queued_recorder

int main() {
  std::puts("  flight_recorder"); std::fflush(stdout);
  test_flight_recorder::run();
  std::puts("  queued_recorder"); std::fflush(stdout);
  test_queued_recorder::run();
  test_queued_recorder::testAsyncPreparation();
}
