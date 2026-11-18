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
bool FlightRecorder::available(uint32_t start, uint32_t bytes) const {
  for (std::size_t i = 0; i < directory_.count(); ++i) {
    const auto &other = directory_.entries()[i];
    if (other.status == FlightStatus::Invalidated) continue;
    if (start < other.start + other.allocatedBytes && other.start < start + bytes) return false;
  }
  return true;
}
bool FlightRecorder::findSpace(uint32_t bytes, uint32_t &start) const {
  if (bytes > FlashLayout::DataEnd - FlashLayout::DataBegin) return false;
  for (uint32_t a = FlashLayout::DataBegin; bytes <= FlashLayout::DataEnd - a; a += FlashLayout::SectorBytes)
    if (available(a, bytes)) { start = a; return true; }
  return false;
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
  uint32_t blocks = 0, lastTime = 0; bool haveTime = false;
  for (uint32_t i = 0; i < limit; ++i) {
    const auto a = entry.start + result.headerBytes + i*FlightSampleStore::BlockBytes;
    if (!samples_.readBlock(a, entry.sequence, i, block_)) {
      const auto error = samples_.error();
      if (error == SampleError::Io) return fail(RecorderError::Io,"Sample",FlightSampleStore::errorName(error));
      if (recover && i >= published && (error == SampleError::Empty || error == SampleError::Incomplete)) break;
      return fail(RecorderError::Corrupt,"Sample",FlightSampleStore::errorName(error));
    }
    if (block_.firstSample != result.samples || (haveTime && block_.samples[0].timeMs <= lastTime)) return fail(RecorderError::Corrupt);
    lastTime = block_.samples[block_.count - 1].timeMs; haveTime = true;
    result.samples += block_.count; ++blocks;
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
  faultLatched_=false; failure_={};
  if (!sampleBytes || sampleBytes % FlashLayout::SectorBytes) return fail(RecorderError::InvalidArgument);
  FlightHeaderLayout planned;
  if (!header_.plan(config, limits, planned)) return fail(RecorderError::Header);
  if ((sensorMode != SensorMode::Hardware && sensorMode != SensorMode::Simulation) ||
      (outputMode != OutputMode::Physical && outputMode != OutputMode::Virtual) ||
      (frame != sensors::FrameConvention::SensorAxes && frame != sensors::FrameConvention::LoggedCalibratedBody)) return fail(RecorderError::InvalidArgument);
  const auto dataBytes = FlashLayout::DataEnd - FlashLayout::DataBegin;
  if (planned.headerBytes > dataBytes || sampleBytes > dataBytes - planned.headerBytes) return fail(RecorderError::Bounds);
  if (directory_.retainedCount() == FlightDirectory::MaxEntries) return fail(RecorderError::DirectoryFull);
  if (directory_.highWater() == std::numeric_limits<uint64_t>::max()) return fail(RecorderError::SequenceExhausted);
  uint32_t start;
  const auto allocated = planned.headerBytes + sampleBytes;
  if (!findSpace(allocated, start)) return fail(RecorderError::NoSpace);
  FlightEntry reservation; reservation.sequence = directory_.highWater() + 1; reservation.start = start;
  reservation.allocatedBytes = allocated;
  if (!directory_.update(reservation)) return fault(RecorderError::Directory);
  current_ = reservation; headerBytes_ = planned.headerBytes; sampleBytes_ = sampleBytes; samplesFull_ = false;
  // Ownership is durable before any erase. Erase only the new contiguous extent.
  // eraseRange may skip read-back: header create and sample begin each verify their
  // part of the extent as erased, together covering all of it exactly once.
  if (!flash_.eraseRange(start, allocated)) return fault(RecorderError::Io);
  if (!header_.create(start, config, limits, sensorMode, outputMode, frame, calibration)) return fault(RecorderError::Header);
  if (!samples_.begin(reservation.sequence, start + headerBytes_, sampleBytes)) return fault(RecorderError::Sample);
  if (!persistProgress(FlightStatus::Recording)) return false;
  active_ = metadataValid_ = true; error_ = RecorderError::None; return true;
}
bool FlightRecorder::append(const FlightSample &sample) {
  if (!active_) return fail(RecorderError::NotRecording);
  if (samplesFull_) return fail(RecorderError::Full);
  if (!samples_.append(sample)) {
    if (samples_.error() == SampleError::BufferFull) return fail(RecorderError::Backpressure);
    if (samples_.error() == SampleError::Full) { samplesFull_ = true; return fail(RecorderError::Full); }
    return fail(RecorderError::Sample);
  }
  error_ = RecorderError::None; return true;
}
bool FlightRecorder::recordStart(recovery::RuleId rule, recovery::OutputKind kind, uint32_t channel, uint32_t timeMs) {
  if (!active_) return fail(RecorderError::NotRecording);
  if (!header_.recordStart(rule, kind, channel, timeMs)) return fail(RecorderError::Header);
  error_ = RecorderError::None; return true;
}
bool FlightRecorder::service() {
  if (!active_) return fail(RecorderError::NotRecording);
  if (!header_.flushStarts()) return fault(RecorderError::Header);
  if (samples_.bufferedSamples() == FlightSampleStore::SamplesPerBlock && !samples_.flush()) return fault(RecorderError::Sample);
  samplesFull_ = samples_.committedBytes() == sampleBytes_;
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
  if (!header_.finish(true)) return fault(RecorderError::Header);
  if (!samples_.finish()) return fault(RecorderError::Sample);
  samplesFull_ = samples_.committedBytes() == sampleBytes_;
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
bool FlightRecorder::readBlock(uint64_t sequence, uint32_t index, SampleBlock &output) {
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
  const uint32_t base = entry->start + headerBytes;
  uint32_t first = 0, lastTime = 0;
  if (index) {
    if (!samples_.readBlock(base + (index-1)*FlightSampleStore::BlockBytes, sequence, index-1, block_)) {
      fail(samples_.error() == SampleError::Io ? RecorderError::Io : RecorderError::Corrupt,"Sample",FlightSampleStore::errorName(samples_.error()));
      return rejectRead(*entry);
    }
    first = block_.firstSample + block_.count; lastTime = block_.samples[block_.count-1].timeMs;
  }
  if (!samples_.readBlock(base + index*FlightSampleStore::BlockBytes, sequence, index, block_)) {
    fail(samples_.error() == SampleError::Io ? RecorderError::Io : RecorderError::Corrupt,"Sample",FlightSampleStore::errorName(samples_.error()));
    return rejectRead(*entry);
  }
  if (block_.firstSample != first || (index && block_.samples[0].timeMs <= lastTime)) {
    fail(RecorderError::Corrupt); return rejectRead(*entry);
  }
  output = block_; metadataValid_ = true; error_ = RecorderError::None; return true;
}
}
