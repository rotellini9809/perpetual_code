#include "storage/FlightRecorder.h"
#include <limits>
#include <cstring>

namespace storage {
const char *FlightRecorder::errorName(RecorderError error) { return recorderErrorName(error); }
void FlightRecorder::abort() { fault(error_ == RecorderError::None ? RecorderError::NotRecording : error_); }
bool FlightRecorder::fail(RecorderError error, const char *component, const char *cause) {
  error_ = error;
  if (faultLatched_) return false;
  failure_ = {}; failure_.error = error;
  failure_.component = "Recorder"; failure_.cause = errorName(error);
  if (error == RecorderError::Directory) { failure_.component="Directory"; failure_.cause=FlightDirectory::errorName(directory_.error()); }
  if (error == RecorderError::Header) { failure_.component="Header"; failure_.cause=FlightHeaderStore::errorName(header_.error()); }
  if (error == RecorderError::Sample) { failure_.component="Sample"; failure_.cause=FlightSampleStore::errorName(samples_.error()); }
  if (error == RecorderError::Summary) { failure_.component="Summary"; failure_.cause=FlightSummaryStore::errorName(summary_.error()); }
  if (component) { failure_.component=component; failure_.cause=cause; }
  if (error == RecorderError::Io || !strcmp(failure_.cause,"Io")) failure_.flash=flash_.lastFailure();
  return false;
}
bool FlightRecorder::initializeBlank() {
  if (active_) return fail(RecorderError::Busy);
  if (initialized_) { error_ = RecorderError::None; return true; }
  if (directory_.mount()) return initialize();
  if (directory_.error() != DirectoryError::Missing) return fail(RecorderError::Directory);
  uint8_t page[FlashLayout::PageBytes];
  const uint32_t banks[] = {FlashLayout::DirectoryA, FlashLayout::DirectoryB};
  for (auto bank : banks)
    for (uint32_t i = 0; i < FlashLayout::DirectoryBankBytes; i += sizeof(page)) {
      if (!flash_.read(bank+i, page, sizeof(page))) return fail(RecorderError::Io);
      for (auto byte : page) if (byte != 255) return fail(RecorderError::NotBlank);
    }
  if (!directory_.format()) return fault(RecorderError::Directory);
  return initialize();
}
bool FlightRecorder::prepareAtBoot() {
  if (initialize()) return true;
  if (directory_.error() != DirectoryError::Missing) return false; // Keep the mount diagnosis.
  return initializeBlank(); // Writes only when both directory banks are entirely erased.
}
bool FlightRecorder::format() {
  if (active_) return fail(RecorderError::NotRecording);
  initialized_=metadataValid_=false; faultLatched_=false; failure_={}; error_=RecorderError::None;
  if (!directory_.format()) return fault(RecorderError::Directory);
  return initialize();
}
bool FlightRecorder::fault(RecorderError error) {
  fail(error); faultLatched_=true;
  initialized_ = active_ = metadataValid_ = false; header_.abandon(); samples_.abandon(); return false;
}
const FlightEntry *FlightRecorder::resolve(uint64_t sequence) const {
  for (std::size_t i = 0; i < directory_.count(); ++i) {
    const auto &entry = directory_.entries()[i];
    if (entry.sequence == sequence && entry.status != FlightStatus::Invalidated) return &entry;
  }
  return nullptr;
}
uint32_t FlightRecorder::nextStartBlock() const {
  // Ring placement: right after the newest retained flight, so the oldest flights lie ahead.
  const FlightEntry *newest = nullptr;
  for (std::size_t i = 0; i < directory_.count(); ++i) {
    const auto &e = directory_.entries()[i];
    if (e.status != FlightStatus::Invalidated && (!newest || e.sequence > newest->sequence)) newest = &e;
  }
  if (!newest) return 0;
  return (FlightDirectory::blockOf(newest->start) + newest->allocatedBytes / FlashLayout::BlockBytes) % FlashLayout::RingBlocks;
}
bool FlightRecorder::makeRoom(uint32_t first, uint32_t blocks, bool newFlight) {
  // Overwrite policy: every other flight owning these blocks is invalidated, whatever its
  // status (Corrupt included), before any erase. A flight is always invalidated whole.
  for (;;) {
    const FlightEntry *victim = nullptr;
    for (std::size_t i = 0; i < directory_.count(); ++i) {
      const auto &e = directory_.entries()[i];
      if (e.status == FlightStatus::Invalidated || e.sequence == current_.sequence) continue;
      const bool overlap = FlightDirectory::overlaps(first, blocks, FlightDirectory::blockOf(e.start), e.allocatedBytes / FlashLayout::BlockBytes);
      if (overlap && (!victim || e.sequence < victim->sequence)) victim = &e;
    }
    if (!victim && newFlight && directory_.retainedCount() >= FlightDirectory::MaxEntries) {
      for (std::size_t i = 0; i < directory_.count(); ++i) {
        const auto &e = directory_.entries()[i];
        if (e.status != FlightStatus::Invalidated && (!victim || e.sequence < victim->sequence)) victim = &e;
      }
    }
    if (!victim) return true;
    auto entry = *victim; entry.status = FlightStatus::Invalidated;
    if (!directory_.update(entry)) return fault(RecorderError::Directory);
    ++overwritten_;
  }
}
bool FlightRecorder::grow() {
  if (!active_ || ringFull_ || samples_.erasing() || samples_.limitCovered()) return true;
  if (samples_.erasedAheadBytes() >= GrowAheadBytes) return true;
  const uint32_t owned = samples_.ownedBytes() / FlashLayout::BlockBytes;
  const uint32_t room = FlashLayout::RingBlocks - owned; // Never reach this flight's own first block.
  const uint32_t missing = uint32_t((uint64_t(headerBytes_) + sampleBytes_ - samples_.ownedBytes() + FlashLayout::BlockBytes - 1) / FlashLayout::BlockBytes);
  uint32_t blocks = GrowBlocks < room ? GrowBlocks : room;
  if (missing < blocks) blocks = missing;
  if (!blocks) { ringFull_ = true; return true; }
  const uint32_t first = (FlightDirectory::blockOf(current_.start) + owned) % FlashLayout::RingBlocks;
  if (!makeRoom(first, blocks, false)) return false;
  // Ownership is durable before the asynchronous erase of the new blocks starts.
  auto entry = current_; entry.allocatedBytes += blocks * FlashLayout::BlockBytes;
  if (!directory_.update(entry)) return fault(RecorderError::Directory);
  current_ = entry;
  if (!samples_.extend(blocks) || !samples_.service()) return fault(RecorderError::Sample);
  ++growths_; return true;
}
bool FlightRecorder::openMetadata(const FlightEntry &entry) {
  if (!entry.committedBytes) return fail(RecorderError::EmptyRecording);
  if (!header_.load(entry.start)) return fail(header_.error() == HeaderError::Io ? RecorderError::Io : RecorderError::Corrupt,"Header",FlightHeaderStore::errorName(header_.error()));
  const auto bytes = header_.view()->layout.headerBytes;
  if (bytes >= entry.allocatedBytes || entry.committedBytes < bytes ||
      (entry.committedBytes - bytes) % FlightSampleStore::BlockBytes) return fail(RecorderError::Corrupt);
  return true;
}
bool FlightRecorder::scan(const FlightEntry &entry, bool recover, RecordingInfo &info) {
  if (!openMetadata(entry)) return false;
  RecordingInfo result; result.entry = entry; result.headerBytes = header_.view()->layout.headerBytes;
  const uint32_t published = (entry.committedBytes - result.headerBytes) / FlightSampleStore::BlockBytes;
  const uint32_t limit = recover ? (entry.allocatedBytes - result.headerBytes) / FlightSampleStore::BlockBytes : published;
  uint32_t blocks = 0, nextSequence = 0; uint64_t lastTime = 0;
  for (uint32_t i = 0; i < limit; ++i) {
    const auto a = FlashLayout::ringAddress(entry.start, result.headerBytes + i*FlightSampleStore::BlockBytes);
    if (!samples_.readBlock(a, entry.sequence, i, block_)) {
      const auto error = samples_.error();
      if (error == SampleError::Io) return fail(RecorderError::Io,"Sample",FlightSampleStore::errorName(error));
      if (recover && i >= published && (error == SampleError::Empty || error == SampleError::Incomplete)) break;
      return fail(RecorderError::Corrupt,"Sample",FlightSampleStore::errorName(error));
    }
    // Headers alone prove ordering: each block starts after every sample of the previous one.
    if (i && (block_.firstSequence < nextSequence || block_.firstTimeUs <= lastTime)) return fail(RecorderError::Corrupt);
    nextSequence = block_.firstSequence + block_.frames; lastTime = block_.firstTimeUs;
    result.samples += block_.frames; ++blocks;
  }
  result.entry.committedBytes = result.headerBytes + blocks*FlightSampleStore::BlockBytes;
  const auto offset=header_.view()->layout.summaryOffset;
  if (!summary_.read(entry.start+offset,entry.sequence,entry.start,header_.checksum(),result.summaryState,result.summary)) return fail(RecorderError::Summary);
  if (result.summaryState==SummaryState::Valid && result.summary.savedSamples!=result.samples) result.summaryState=SummaryState::Corrupt;
  info = result; return true;
}
bool FlightRecorder::corruptionDetected() const {
  // An unsupported header is not evidence of damaged media. I/O failures also
  // never authorize a persistent corruption flag.
  return error_==RecorderError::Corrupt &&
      !(strcmp(failure_.component,"Header")==0 && strcmp(failure_.cause,"UnsupportedVersion")==0);
}
bool FlightRecorder::markCorrupt(const FlightEntry &entry) {
  auto damaged=entry; damaged.status=FlightStatus::Corrupt;
  metadataValid_=false;
  if (!directory_.update(damaged)) return fault(RecorderError::Directory);
  if (current_.sequence==entry.sequence) current_=damaged;
  return true;
}
bool FlightRecorder::rejectRead(const FlightEntry &entry) {
  // Successful quarantine must not replace the original read/CRC diagnostics.
  if (corruptionDetected()) markCorrupt(entry);
  return false;
}
bool FlightRecorder::initialize() {
  if (active_) return fail(RecorderError::Busy);
  faultLatched_=false; failure_={};
  initialized_ = metadataValid_ = false; header_.abandon(); samples_.abandon(); current_ = {}; samplesFull_ = false;
  if (!directory_.mount()) return fail(RecorderError::Directory);
  for (std::size_t i = 0; i < directory_.count(); ++i) {
    auto entry = directory_.entries()[i];
    if (entry.status != FlightStatus::Recording) continue;
    // A reservation alone cannot prove that an old header was erased/replaced.
    // No samples/starts are accepted until the committed header checkpoint exists.
    if (entry.committedBytes) {
      RecordingInfo recovered;
      if (!scan(entry, true, recovered)) {
        if (!corruptionDetected() || !markCorrupt(entry)) return false;
        // Retain the damaged recording without blocking other retained flights.
        continue;
      }
      entry.committedBytes = recovered.entry.committedBytes;
    }
    entry.status = FlightStatus::Interrupted;
    if (!directory_.update(entry)) return fail(RecorderError::Directory);
    current_ = entry; metadataValid_ = entry.committedBytes != 0;
  }
  initialized_ = true; error_ = RecorderError::None; failure_={}; return true;
}
bool FlightRecorder::begin(const recovery::Configuration &config, const recovery::ValidationLimits &limits, uint32_t sampleBytes,
                           SensorMode sensorMode, OutputMode outputMode, sensors::FrameConvention frame, const FlightCalibration &calibration) {
  if (active_) return fail(RecorderError::Busy);
  if (!initialized_) return fail(RecorderError::NotInitialized);
  faultLatched_=false; failure_={}; growths_=overwritten_=0;
  if (!sampleBytes || sampleBytes % FlashLayout::SectorBytes) return fail(RecorderError::InvalidArgument);
  FlightHeaderLayout planned;
  if (!header_.plan(config, limits, planned)) return fail(RecorderError::Header);
  if ((sensorMode != SensorMode::Hardware && sensorMode != SensorMode::Simulation) ||
      (outputMode != OutputMode::Physical && outputMode != OutputMode::Virtual) ||
      (frame != sensors::FrameConvention::SensorAxes && frame != sensors::FrameConvention::LoggedCalibratedBody)) return fail(RecorderError::InvalidArgument);
  // The flight header (descriptor, configuration, starts, summary) stays inside the first block.
  if (planned.headerBytes >= FlashLayout::BlockBytes) return fail(RecorderError::Bounds);
  if (directory_.highWater() == std::numeric_limits<uint64_t>::max()) return fail(RecorderError::SequenceExhausted);
  // Compact now if needed, so the directory never compacts during a flight.
  if (directory_.freeSlots() < FlightUpdates && !directory_.compact()) return fault(RecorderError::Directory);
  const uint32_t limit = sampleBytes < FlashLayout::RingBytes - planned.headerBytes ? sampleBytes : FlashLayout::RingBytes - planned.headerBytes;
  const uint32_t needed = uint32_t((uint64_t(planned.headerBytes) + limit + FlashLayout::BlockBytes - 1) / FlashLayout::BlockBytes);
  const uint32_t base = needed < BaseBlocks ? needed : BaseBlocks;
  const uint32_t first = nextStartBlock();
  current_ = {}; // No current flight yet: every overlapping flight may be invalidated.
  if (!makeRoom(first, base, true)) return false;
  FlightEntry reservation; reservation.sequence = directory_.highWater() + 1; reservation.start = FlashLayout::blockAddress(first);
  reservation.allocatedBytes = base * FlashLayout::BlockBytes;
  if (!directory_.update(reservation)) return fault(RecorderError::Directory);
  current_ = reservation; headerBytes_ = planned.headerBytes; sampleBytes_ = limit; samplesFull_ = ringFull_ = false;
  pad_.clear(); launched_ = haveLanded_ = landedBaroPending_ = false;
  // Ownership is durable before any erase. Erase only the new blocks (one 64 KiB block erase
  // each; the ring may wrap). Header create and sample begin verify them erased.
  for (uint32_t i = 0; i < base; ++i)
    if (!flash_.eraseRange(FlashLayout::blockAddress((first + i) % FlashLayout::RingBlocks), FlashLayout::BlockBytes)) return fault(RecorderError::Io);
  if (!header_.create(reservation.start, config, limits, sensorMode, outputMode, frame, calibration)) return fault(RecorderError::Header);
  if (!samples_.begin(reservation.sequence, reservation.start, headerBytes_, limit, reservation.allocatedBytes)) return fault(RecorderError::Sample);
  if (!persistProgress(FlightStatus::Recording)) return false;
  active_ = metadataValid_ = true; error_ = RecorderError::None; return true;
}
bool FlightRecorder::store(const FlightSample &sample, bool history, bool deliberate) {
  if (!grow()) return false;
  if (samples_.append(sample, history, deliberate)) { error_ = RecorderError::None; return true; }
  if (samples_.error() == SampleError::Full) { samplesFull_ = true; return fail(RecorderError::Full); }
  if (samples_.error() == SampleError::Timestamp) return fail(RecorderError::Sample);
  return fault(RecorderError::Sample); // A block write failed: never retry an attempted mutation.
}
bool FlightRecorder::writeHistory() {
  bool ok = true;
  pad_.forEach([&](const FlightSample &sample) { ok = store(sample, true, true); return ok; });
  pad_.clear();
  // A full extent only truncates the history; other failures are reported.
  return ok || error_ == RecorderError::Full;
}
bool FlightRecorder::append(const FlightSample &sample) {
  if (!active_) return fail(RecorderError::NotRecording);
  if (samplesFull_) return fail(RecorderError::Full);
  if (!launched_) {
    if (sample.phase == recovery::FlightPhase::Pad) { pad_.push(sample); error_ = RecorderError::None; return true; }
    launched_ = true;
    if (!writeHistory()) return false;
    if (samplesFull_) return fail(RecorderError::Full);
  }
  if (sample.phase != recovery::FlightPhase::Landed) return store(sample, false, false);
  // LANDED is kept at 10 Hz; the newest barometer reading travels with each kept sample.
  if (sample.baroFresh) {
    landedBaroUs_ = sample.timeUs - sample.baroAgeUs; landedBaroPending_ = true;
    landedPressure_ = sample.values[FlightSample::PressurePa]; landedTemperature_ = sample.values[FlightSample::TemperatureC];
  }
  if (haveLanded_ && sample.timeUs - lastLandedUs_ < LandedSpacingUs) { error_ = RecorderError::None; return true; }
  FlightSample kept = sample;
  if (!kept.baroFresh && landedBaroPending_ && kept.timeUs - landedBaroUs_ <= 0xFFFF) {
    kept.baroFresh = true; kept.baroAgeUs = uint16_t(kept.timeUs - landedBaroUs_);
    kept.values[FlightSample::PressurePa] = landedPressure_; kept.values[FlightSample::TemperatureC] = landedTemperature_;
  }
  landedBaroPending_ = false;
  const bool deliberate = haveLanded_; haveLanded_ = true; lastLandedUs_ = kept.timeUs;
  return store(kept, false, deliberate);
}
bool FlightRecorder::recordStart(recovery::RuleId rule, recovery::OutputKind kind, uint32_t channel, uint32_t timeMs) {
  if (!active_) return fail(RecorderError::NotRecording);
  if (!header_.recordStart(rule, kind, channel, timeMs)) return fail(RecorderError::Header);
  error_ = RecorderError::None; return true;
}
bool FlightRecorder::service() {
  if (!active_) return fail(RecorderError::NotRecording);
  if (!header_.flushStarts()) return fault(RecorderError::Header);
  if (!samples_.service()) return fault(RecorderError::Sample);
  if (!grow()) return false;
  error_ = RecorderError::None; return true;
}
bool FlightRecorder::persistProgress(FlightStatus status) {
  auto entry = current_; entry.committedBytes = headerBytes_ + samples_.committedBytes(); entry.status = status;
  if (entry.committedBytes != current_.committedBytes || entry.status != current_.status)
    if (!directory_.update(entry)) return fault(RecorderError::Directory);
  current_ = entry; error_ = RecorderError::None; return true;
}
bool FlightRecorder::finish(bool producerClosed, const RecordingSummary &summary, RecordingRetention retention) {
  if (!active_) return fail(RecorderError::NotRecording);
  if (!producerClosed) return fail(RecorderError::ProducerActive);
  if (unsigned(summary.reason)<1 || unsigned(summary.reason)>4 || summary.queuePeak>summary.queueCapacity) return fail(RecorderError::InvalidArgument);
  if (retention != RecordingRetention::Keep && retention != RecordingRetention::Discard) return fail(RecorderError::InvalidArgument);
  if (retention == RecordingRetention::Discard) {
    // One durable directory transition is sufficient. Until its commit, recovery
    // retains the uncertain Recording entry; never infer PAD from missing samples.
    auto entry = current_; entry.status = FlightStatus::Invalidated;
    if (!directory_.update(entry)) return fault(RecorderError::Directory);
    current_ = entry; error_ = RecorderError::None;
    active_ = metadataValid_ = false; header_.abandon(); samples_.abandon();
    return true;
  }
  // A run that never left PAD (simulation/HIL keeps it) still saves its pre-launch history.
  if (!launched_ && !writeHistory() && !active_) return false;
  if (!header_.finish(true)) return fault(RecorderError::Header);
  // Full means a sample was refused, not that the last block happens to be in use.
  if (!samples_.finish()) return fault(RecorderError::Sample);
  auto finalSummary=summary; finalSummary.savedSamples=samples_.committedSamples();
  if (samplesFull_) finalSummary.reason=RecordingEnd::StorageFull;
  if (!summary_.write(current_.start+header_.view()->layout.summaryOffset,current_.sequence,current_.start,header_.checksum(),finalSummary)) return fault(RecorderError::Summary);
  if (!persistProgress(samplesFull_ ? FlightStatus::StoppedFull : FlightStatus::Completed)) return false;
  active_ = false; error_ = RecorderError::None; return true;
}
bool FlightRecorder::remove(uint64_t sequence) {
  if (active_) return fail(RecorderError::Busy);
  if (!initialized_) return fail(RecorderError::NotInitialized);
  for (std::size_t i=0; i<directory_.count(); ++i) {
    auto entry=directory_.entries()[i];
    if (entry.sequence!=sequence) continue;
    if (entry.status==FlightStatus::Recording) return fail(RecorderError::Busy);
    if (entry.status==FlightStatus::Invalidated) { error_=RecorderError::None; return true; }
    entry.status=FlightStatus::Invalidated; metadataValid_=false;
    if (!directory_.update(entry)) return fault(RecorderError::Directory);
    error_=RecorderError::None; return true;
  }
  return fail(RecorderError::StaleSelection);
}
bool FlightRecorder::inspect(uint64_t sequence, RecordingInfo &output) {
  if (active_) return fail(RecorderError::Busy);
  metadataValid_ = false;
  if (!initialized_) return fail(RecorderError::NotInitialized);
  const auto *entry = resolve(sequence);
  if (!entry) return fail(RecorderError::StaleSelection);
  const auto selected=*entry; // Directory compaction can move RAM entries.
  if (selected.status==FlightStatus::Corrupt) return fail(RecorderError::Corrupt);
  RecordingInfo info;
  if (!scan(selected, false, info)) return rejectRead(selected);
  output = info; metadataValid_ = true; error_ = RecorderError::None; return true;
}
bool FlightRecorder::readBlock(uint64_t sequence, uint32_t index, RawBlock &output) {
  if (active_) return fail(RecorderError::Busy);
  metadataValid_ = false;
  if (!initialized_) return fail(RecorderError::NotInitialized);
  const auto *resolved = resolve(sequence);
  if (!resolved) return fail(RecorderError::StaleSelection);
  const auto selected=*resolved;
  const auto *entry=&selected;
  if (entry->status==FlightStatus::Corrupt) return fail(RecorderError::Corrupt);
  if (!openMetadata(*entry)) return rejectRead(*entry);
  const auto headerBytes = header_.view()->layout.headerBytes;
  const auto blocks = (entry->committedBytes - headerBytes) / FlightSampleStore::BlockBytes;
  if (index >= blocks) return fail(RecorderError::Bounds);
  // Cross-block ordering was proven by inspect(); a single read validates this block only.
  if (!samples_.readBlock(FlashLayout::ringAddress(entry->start, headerBytes + index*FlightSampleStore::BlockBytes), sequence, index, block_)) {
    fail(samples_.error() == SampleError::Io ? RecorderError::Io : RecorderError::Corrupt,"Sample",FlightSampleStore::errorName(samples_.error()));
    return rejectRead(*entry);
  }
  output = block_; metadataValid_ = true; error_ = RecorderError::None; return true;
}
}
