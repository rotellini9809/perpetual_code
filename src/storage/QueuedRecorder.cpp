#include "storage/QueuedRecorder.h"

namespace storage {
bool QueuedRecorder::busy() const {
  const auto s = state_.load(std::memory_order_acquire);
  return s == State::Preparing || s == State::Prepared || s == State::Running || s == State::Closing || s == State::Aborting;
}
bool QueuedRecorder::active() const { return state_.load(std::memory_order_acquire) == State::Running; }
const FlightEntry *QueuedRecorder::current() const {
  if (!busy() && current_.sequence && writer_.current() && writer_.current()->sequence == current_.sequence)
    return writer_.current();
  return current_.sequence ? &current_ : nullptr;
}
bool QueuedRecorder::initialized() const { return busy() ? ready_.load(std::memory_order_relaxed) : writer_.initialized(); }
RecorderError QueuedRecorder::error() const {
  const auto error = workerError_.load(std::memory_order_acquire);
  return error != RecorderError::None ? error : frontError_;
}
RecordingFailure QueuedRecorder::failure() const {
  // Never read worker-owned diagnostics before its release acknowledgment.
  if (!busy()) return writer_.failure();
  RecordingFailure result; result.error=error(); return result;
}
void QueuedRecorder::publish() {
  saved_.store(writer_.committedSamples(), std::memory_order_relaxed);
  full_.store(writer_.samplesFull(), std::memory_order_relaxed);
  ready_.store(writer_.initialized(), std::memory_order_relaxed);
}
bool QueuedRecorder::begin(const recovery::Configuration &config, const recovery::ValidationLimits &limits, uint32_t bytes,
                           SensorMode sensorMode, OutputMode outputMode, sensors::FrameConvention frame, const FlightCalibration &calibration) {
  if (busy()) return fail(RecorderError::Busy);
  if (!online()) return fail(RecorderError::NotInitialized);
  workerError_.store(RecorderError::None);
  if (!writer_.begin(config, limits, bytes, sensorMode, outputMode, frame, calibration)) return fail(writer_.error());
  samples_.reset(); starts_.reset(); peak_ = 0; dropped_.store(0); workerTiming_ = {}; preparationUs_ = 0;
  current_ = *writer_.current(); directoryCount_ = writer_.directory().count();
  assignments_ = writer_.metadata()->starts; assignmentCount_ = writer_.metadata()->layout.slotCount;
  frontError_ = RecorderError::None; publish();
  preparationOnly_=false;
  state_.store(State::Running, std::memory_order_release);
  if (notify_) notify_();
  return true;
}
bool QueuedRecorder::beginAsync(const recovery::Configuration &config, const recovery::ValidationLimits &limits, uint32_t bytes,
                              SensorMode sensorMode, OutputMode outputMode, sensors::FrameConvention frame, const FlightCalibration &calibration) {
  if (busy()) return fail(RecorderError::Busy);
  if (!online() || !writer_.initialized()) return fail(RecorderError::NotInitialized);
  preparationConfig_=config; preparationLimits_=limits; preparationBytes_=bytes;
  preparationSensor_=sensorMode; preparationOutput_=outputMode; preparationFrame_=frame; preparationCalibration_=calibration;
  samples_.reset(); starts_.reset(); peak_=0; dropped_.store(0); saved_.store(0); full_.store(false);
  current_={}; assignmentCount_=0; directoryCount_=writer_.directory().count(); workerTiming_={}; preparationUs_=0;
  ready_.store(true); workerError_.store(RecorderError::None); frontError_=RecorderError::None;
  preparationOnly_=true;
  state_.store(State::Preparing,std::memory_order_release);
  if (notify_) notify_();
  return true;
}
bool QueuedRecorder::activatePrepared() {
  if (state_.load(std::memory_order_acquire)!=State::Prepared) return false;
  // Worker is parked in Prepared; transfer immutable metadata before resuming it.
  current_=*writer_.current(); directoryCount_=writer_.directory().count();
  assignments_=writer_.metadata()->starts; assignmentCount_=writer_.metadata()->layout.slotCount;
  preparationOnly_=false;
  state_.store(State::Running,std::memory_order_release);
  if (notify_) notify_();
  return true;
}
bool QueuedRecorder::append(const FlightSample &sample) {
  if (!active()) return fail(RecorderError::NotRecording);
  if (samplesFull()) return fail(RecorderError::Full);
  uint32_t occupancy = 0;
  if (!samples_.push(sample, occupancy)) return fail(RecorderError::Backpressure);
  if (occupancy > peak_) peak_ = occupancy;
  if (notify_) notify_();
  frontError_ = RecorderError::None; return true;
}
bool QueuedRecorder::recordStart(recovery::RuleId rule, recovery::OutputKind kind, uint32_t channel, uint32_t timeMs) {
  if (!active()) return fail(RecorderError::NotRecording);
  for (unsigned i = 0; i < assignmentCount_; ++i) {
    auto &a = assignments_[i];
    if (a.rule != rule || a.kind != kind || a.channel != channel) continue;
    if (a.state == StartState::Pending) return a.timeMs == timeMs ? (frontError_ = RecorderError::None, true) : fail(RecorderError::Header);
    Start start; start.rule = rule; start.kind = kind; start.channel = channel; start.timeMs = timeMs;
    uint32_t occupancy = 0;
    if (!starts_.push(start, occupancy)) return fail(RecorderError::Backpressure);
    a.state = StartState::Pending; a.timeMs = timeMs; frontError_ = RecorderError::None;
    if (notify_) notify_();
    return true;
  }
  return fail(RecorderError::Header);
}
bool QueuedRecorder::service() {
  if (workerError_.load(std::memory_order_acquire) != RecorderError::None) return false;
  if (!active()) return fail(RecorderError::NotRecording);
  frontError_ = RecorderError::None; return true;
}
bool QueuedRecorder::finish(bool producerClosed, const RecordingSummary &summary, RecordingRetention retention) {
  if (!producerClosed) return fail(RecorderError::ProducerActive);
  if (unsigned(summary.reason)<1 || unsigned(summary.reason)>4) return fail(RecorderError::InvalidArgument);
  if (retention != RecordingRetention::Keep && retention != RecordingRetention::Discard) return fail(RecorderError::InvalidArgument);
  // Publication is synchronized by the following Running -> Closing handoff.
  if (!active()) return fail(RecorderError::NotRecording);
  closingSummary_=summary; closingSummary_.queuePeak=peak_; closingSummary_.queueCapacity=SampleCapacity;
  closingRetention_=retention;
  State expected = State::Running;
  if (!state_.compare_exchange_strong(expected, State::Closing, std::memory_order_acq_rel)) return fail(RecorderError::NotRecording);
  if (notify_) notify_();
  frontError_ = RecorderError::None; return true;
}
void QueuedRecorder::abort() {
  auto s = state_.load(std::memory_order_acquire);
  while (s == State::Preparing || s == State::Prepared || s == State::Running || s == State::Closing) {
    if (state_.compare_exchange_weak(s, State::Aborting, std::memory_order_acq_rel)) break;
  }
  if (notify_) notify_();
}
void QueuedRecorder::fault(RecorderError error, uint32_t start) {
  if (writer_.active()) writer_.abort();
  publish(); workerTiming_.add(uint32_t(now()-start));
  workerError_.store(error, std::memory_order_release);
  state_.store(State::Fault, std::memory_order_release); // Last writer access before handoff.
  if (notify_) notify_();
}
void QueuedRecorder::close(uint32_t start) {
  auto summary=closingSummary_; summary.droppedSamples+=dropped_.load(std::memory_order_relaxed);
  if (!writer_.finish(true,summary,closingRetention_)) { fault(writer_.error(), start); return; }
  publish(); workerTiming_.add(uint32_t(now()-start));
  state_.store(State::Closed, std::memory_order_release);
  if (notify_) notify_();
}
bool QueuedRecorder::poll() {
  auto state = state_.load(std::memory_order_acquire);
  if (state==State::Preparing) {
    const auto start=now();
    const bool ok=writer_.begin(preparationConfig_,preparationLimits_,preparationBytes_,preparationSensor_,preparationOutput_,preparationFrame_,preparationCalibration_);
    if (!ok) { fault(writer_.error(),start); return true; }
    // Kept apart from recording jobs: the initial erase runs before any sample exists.
    publish(); preparationUs_=uint32_t(now()-start);
    State expected=State::Preparing;
    if (state_.compare_exchange_strong(expected,State::Prepared,std::memory_order_acq_rel)) return true;
    state=expected; // Cancellation arrived during erase: finish on this core.
  }
  if (state != State::Running && state != State::Closing && state != State::Aborting) return false;
  const auto start = now();
  if (state==State::Aborting && preparationOnly_) {
    if (writer_.active() && !writer_.finish(true,RecordingSummary{},RecordingRetention::Discard)) { fault(writer_.error(),start); return true; }
    publish(); state_.store(State::Closed,std::memory_order_release); return true;
  }
  if (state == State::Aborting) { fault(RecorderError::NotRecording, start); return true; }
  // An orderly PAD discard need not persist queued samples/starts. The producer
  // cannot reuse flash until the directory invalidation is acknowledged here.
  if (state == State::Closing && closingRetention_ == RecordingRetention::Discard) { close(start); return true; }
  bool work = false; Start notification;
  while (starts_.pop(notification)) {
    work = true;
    if (!writer_.recordStart(notification.rule, notification.kind, notification.channel, notification.timeMs)) {
      fault(writer_.error(), start); return true;
    }
  }
  FlightSample sample;
  if (samples_.pop(sample)) {
    work = true;
    if (!writer_.append(sample)) {
      if (writer_.error() == RecorderError::Full) dropped_.fetch_add(1, std::memory_order_relaxed);
      else { fault(writer_.error(), start); return true; }
    }
  }
  if (work && !writer_.service()) { fault(writer_.error(), start); return true; }
  state = state_.load(std::memory_order_acquire);
  if (state == State::Aborting) { fault(RecorderError::NotRecording, start); return true; }
  if (state == State::Closing && (closingRetention_ == RecordingRetention::Discard || (samples_.empty() && starts_.empty()))) {
    close(start);
    return true;
  }
  if (work) { publish(); workerTiming_.add(uint32_t(now()-start)); }
  return work;
}
}
