#include "flight/FlightRuntime.h"
#include "storage/FlashLayout.h"
#include <cmath>
#include <limits>

namespace flight {
const char *FlightRuntime::preparationError() const {
  if (run_.active() || config_.runLocked()) return "BUSY";
  if (recorder_ && recorder_->busy()) return "BUSY";
  if (config_.hasDraft()) return "DRAFT_ACTIVE";
  if (!config_.applied()) return "CONFIG_INVALID";
  return nullptr;
}
const char *FlightRuntime::begin(sensors::SensorSource &source, float referenceAltitude, float nominalHz, const navigation::SensorCalibration &calibration, uint32_t recordBytes, storage::SensorMode sensorMode, storage::OutputMode outputMode) {
  if (const char *error = preparationError()) return error;
  if (!std::isfinite(referenceAltitude) || !std::isfinite(nominalHz) || nominalHz < 10) return "INVALID_ARGUMENT";
  if (sensorMode != storage::SensorMode::Simulation && sensorMode != storage::SensorMode::Hardware) return "INVALID_ARGUMENT";
  if (outputMode != storage::OutputMode::Virtual && outputMode != storage::OutputMode::Physical) return "INVALID_ARGUMENT";
  if (outputMode == storage::OutputMode::Physical) {
    if (!outputs_) return "HARDWARE_CHECKS_UNAVAILABLE";
    if (!outputs_->timingAvailable()) return "TIMING_UNAVAILABLE";
  }
  if (recordBytes && (!recorder_ || !recorder_->initialized())) return "STORAGE_NOT_READY";
  if (recordBytes % storage::FlashLayout::SectorBytes) return "INVALID_ARGUMENT";
  pipeline_.end(); source_ = nullptr; started_ = false;
  if (!pipeline_.begin(source, calibration, referenceAltitude, nominalHz)) return "INITIALIZATION_FAILED";
  if (const char *error = run_.begin(RunMode::Sensors, config_)) { pipeline_.end(); return error; }
  recordingRequested_ = recordBytes != 0; saved_ = dropped_ = 0; full_ = false;
  boostReached_ = processingFailed_ = landed_ = closingStarts_ = false; sensorMode_ = sensorMode; outputMode_ = outputMode;
  recordingSequence_ = 0; recordingError_ = storage::RecorderError::None;
  timing_ = {}; timing_.nominalPeriodUs = uint32_t(1000000.0f / nominalHz);
  if (recordingRequested_) {
    const auto start = clock_ ? clock_() : 0;
    const bool prepared = recorder_->begin(*config_.applied(), config_.limits(), recordBytes, sensorMode,
                                           outputMode, source.convention());
    timing_.prepareUs = clock_ ? uint32_t(clock_() - start) : 0;
    if (!prepared) {
      recordingError_ = recorder_->error(); run_.stop(RunMode::Sensors); pipeline_.end();
      return storage::recorderErrorName(recordingError_);
    }
    recordingSequence_ = recorder_->current()->sequence;
  }
  source_ = &source; identity_ = run_.identity(); started_ = true; maxComputeUs_ = 0;
  if (outputMode_ == storage::OutputMode::Physical) {
    if (const auto error = outputs_->begin(*config_.applied(),identity_,sensorMode==storage::SensorMode::Hardware)) { stop(); return error; }
  }
  return nullptr;
}
RuntimeStatus FlightRuntime::status() const {
  const auto &progress = run_.progress(RunMode::Sensors);
  RuntimeStatus status; status.active = active(); status.timingValid = clock_ != nullptr;
  status.outputMode = outputMode_;
  status.outputsActive = outputMode_ == storage::OutputMode::Physical && outputs_ && outputs_->owner() == identity_ && outputs_->active();
  status.samples = progress.samples; status.timeMs = progress.timeMs; status.maxComputeUs = maxComputeUs_;
  status.recordingRequested = recordingRequested_; status.recordingSequence = recordingSequence_;
  status.recordingActive = active() && recordingRequested_ && recorder_ && recorder_->active() && recorder_->current()->sequence == recordingSequence_;
  status.savedSamples = saved_; status.droppedSamples = dropped_; status.samplesFull = full_;
  status.boostReached = boostReached_;
  status.recordingError = recordingError_;
  refreshRecordingStatus(status);
  return status;
}
void FlightRuntime::refreshRecordingStatus(RuntimeStatus &status) const {
  status.recordingBusy = false;
  status.pendingSamples = 0;
  if (recordingRequested_ && recorder_ && recorder_->current() && recorder_->current()->sequence == recordingSequence_) {
    // Acquire release acknowledgment before reading terminal counters. If it
    // happens later, report busy for this reply and refresh on the next poll.
    status.recordingBusy = recorder_->busy();
    status.recordingDiscarded = !status.recordingBusy && recorder_->current()->status == storage::FlightStatus::Invalidated;
    status.savedSamples = recorder_->committedSamples(); status.samplesFull = recorder_->samplesFull();
    status.droppedSamples = dropped_ + recorder_->backgroundDroppedSamples(); status.recordingAsync = recorder_->queued();
    status.pendingSamples = recorder_->pendingSamples(); status.queueCapacity = recorder_->sampleQueueCapacity();
    if (status.recordingError == storage::RecorderError::None && recorder_->error() != storage::RecorderError::None)
      status.recordingError = recorder_->error();
  }
}
void FlightRuntime::recordingFault(storage::RecorderError error) {
  if (recorder_ && recorder_->active() && recorder_->current()->sequence == recordingSequence_) recorder_->abort();
  if (error == storage::RecorderError::None && recorder_) error = recorder_->error();
  recordingError_ = error == storage::RecorderError::None ? storage::RecorderError::NotRecording : error;
}
unsigned FlightRuntime::stop(storage::RecordingEnd reason) {
  const bool owned = active();
  unsigned outputPending=0;
  if (outputMode_ == storage::OutputMode::Physical && outputs_ && outputs_->owner() == identity_) {
    recovery::VirtualStart starts[recovery::MaxAssignments];
    const auto count = outputs_->drain(starts,recovery::MaxAssignments);
    outputPending=outputs_->stop(identity_); // GPIO/PWM off before storage finalization.
    if (owned && recordingRequested_ && recorder_ && recorder_->active() && recorder_->current()->sequence == recordingSequence_)
      for (std::size_t i=0;i<count;++i)
        if (!recorder_->recordStart(starts[i].rule,starts[i].kind,starts[i].channel,uint32_t(starts[i].timeMs))) {
          recordingFault(recorder_->error()); break;
        }
  }
  const auto pending = owned ? run_.stop(RunMode::Sensors) : 0;
  started_ = false; pipeline_.end(); source_ = nullptr;
  if (recordingRequested_ && recorder_ && recorder_->current() && recorder_->current()->sequence == recordingSequence_ &&
      !recorder_->busy() && !recorder_->initialized())
    recordingFault(recorder_->error());
  if (recordingRequested_ && recorder_ && recorder_->active() && recorder_->current()->sequence == recordingSequence_) {
    if (!owned) recordingFault(storage::RecorderError::NotRecording);
    else {
      const auto start = clock_ ? clock_() : 0;
      storage::RecordingSummary summary; summary.droppedSamples=dropped_; summary.reason=reason;
      // Keep development recordings for diagnosis, including PAD-only HIL. Live
      // retention uses actual navigation history, never buffered/persisted samples.
      const auto retention = sensorMode_ == storage::SensorMode::Hardware && !boostReached_ && !processingFailed_
          ? storage::RecordingRetention::Discard : storage::RecordingRetention::Keep;
      const bool finished = recorder_->finish(true,summary,retention);
      timing_.finishUs = clock_ ? uint32_t(clock_() - start) : 0;
      if (!finished) recordingFault(recorder_->error());
    }
    saved_ = recorder_->committedSamples(); full_ = recorder_->samplesFull();
  }
  return pending+outputPending;
}
void FlightRuntime::save(const sensors::Reading &input, const FrameResult &result) {
  if (!recordingRequested_ || recordingError_ != storage::RecorderError::None) return;
  if (!recorder_->active() || recorder_->current()->sequence != recordingSequence_) {
    const auto error = recorder_->error();
    recordingFault(error == storage::RecorderError::None ? storage::RecorderError::NotRecording : error); return;
  }
  for (std::size_t i = 0; i < result.events.startCount; ++i) {
    const auto &a = result.events.starts[i];
    if (!recorder_->recordStart(a.rule, a.kind, a.channel, uint32_t(a.timeMs))) { recordingFault(recorder_->error()); return; }
  }
  storage::FlightSample sample; sample.timeMs = result.timeMs; sample.phase = result.navigation.phase;
  if (battery_) sample.batteryMv = battery_->millivolts();
  for (unsigned i = 0; i < 3; ++i) {
    sample.values[storage::FlightSample::AccelX+i] = input.imuValid ? input.accelMps2[i] : std::numeric_limits<float>::quiet_NaN();
    sample.values[storage::FlightSample::GyroX+i] = input.imuValid ? input.gyroRadps[i] : std::numeric_limits<float>::quiet_NaN();
  }
  sample.values[storage::FlightSample::PressureHpa] = input.baroValid ? input.pressurePa / 100.0f : std::numeric_limits<float>::quiet_NaN();
  sample.values[storage::FlightSample::TemperatureC] = input.temperatureC;
  const auto &nav = result.navigation;
  if (nav.valid) {
    sample.values[storage::FlightSample::AltitudeM] = nav.altitude;
    sample.values[storage::FlightSample::VelocityMps] = nav.verticalVelocity;
    for (unsigned i = 0; i < 3; ++i) sample.values[storage::FlightSample::RollDegrees+i] = nav.eulerRadians[i] * 57.29577951308232f;
    sample.values[storage::FlightSample::VerticalAccelMps2] = nav.verticalAcceleration;
  }
  if (!recorder_->append(sample)) {
    if (recorder_->error() == storage::RecorderError::Full || recorder_->error() == storage::RecorderError::Backpressure) ++dropped_;
    else { recordingFault(recorder_->error()); return; }
  }
  if (!recorder_->service()) { recordingFault(recorder_->error()); return; }
  saved_ = recorder_->committedSamples(); full_ = recorder_->samplesFull();
}
PollResult FlightRuntime::poll(FrameResult &result) {
  result = {};
  PollResult polled;
  if (blockedByOtherRun()) { polled.error = "BUSY"; return polled; }
  if (!active()) { polled.error = "NOT_RUNNING"; return polled; }
  if (outputMode_ == storage::OutputMode::Physical) {
    outputs_->poll();
    if (landed_ && (run_.landingWindowExpired(RunMode::Sensors) || (clock_ && uint32_t(clock_()-landingClockUs_)>=15000000u))) {
      run_.closeStarts(); closingStarts_=true;
    }
    // Holding PWM is not an unfinished action. Finish pulses before normal landing
    // closure; explicit stop/disarm still disables everything immediately.
    if (closingStarts_ && !outputs_->pulsing()) { stop(storage::RecordingEnd::Completed); return polled; }
  }
  sensors::Reading input;
  if (!source_->poll(input)) return polled;
  polled.error = process(input, result);
  polled.processed = polled.error == nullptr;
  return polled;
}
const char *FlightRuntime::process(const sensors::Reading &input, FrameResult &result) {
  if (input.timeUs / 1000 > std::numeric_limits<uint32_t>::max()) return "INVALID_TIMESTAMP";
  const auto time = uint32_t(input.timeUs / 1000);
  if (!run_.validTime(RunMode::Sensors, time)) return "INVALID_TIMESTAMP";
  const auto start = clock_ ? clock_() : 0;
  if (!pipeline_.process(input, result.navigation)) { processingFailed_ = true; stop(); return "SOURCE_ERROR"; }
  result.events = run_.evaluate(RunMode::Sensors, time, result.navigation.ruleSnapshot());
  if (result.events.accepted && outputMode_ == storage::OutputMode::Physical) {
    for (std::size_t i=0;i<result.events.startCount;++i)
      if (!outputs_->submit(result.events.starts[i])) { processingFailed_=true; stop(); return "OUTPUT_ERROR"; }
    outputs_->poll();
    result.events.startCount=outputs_->drain(result.events.starts.data(),result.events.starts.size());
  }
  const auto computeEnd = clock_ ? clock_() : 0;
  result.computeUs = uint32_t(computeEnd - start);
  if (result.computeUs > maxComputeUs_) maxComputeUs_ = result.computeUs;
  if (!result.events.accepted) { processingFailed_ = true; stop(); return "PROCESSING_ERROR"; }
  // The detector's phase history remains authoritative even if this frame's
  // numerical estimate is invalid after the detector has advanced.
  if (result.navigation.phase == recovery::FlightPhase::Boost) boostReached_ = true;
  if (!landed_ && result.navigation.phase == recovery::FlightPhase::Landed) {
    landed_=true; landingClockUs_=clock_ ? clock_() : 0;
  }
  result.timeMs = time;
  const bool recordingWork = recordingRequested_ && recordingError_ == storage::RecorderError::None;
  save(input, result); // The selected sink either queues work or performs synchronous development writes.
  const auto stepEnd = clock_ ? clock_() : 0;
  timing_.compute.add(result.computeUs);
  if (recordingWork) timing_.storage.add(uint32_t(stepEnd - computeEnd));
  const auto stepUs = uint32_t(stepEnd - start); timing_.step.add(stepUs);
  if (clock_ && timing_.nominalPeriodUs && stepUs > timing_.nominalPeriodUs) ++timing_.overBudget;
  return nullptr;
}
}
