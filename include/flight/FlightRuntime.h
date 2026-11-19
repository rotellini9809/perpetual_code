#pragma once
#include "flight/RunController.h"
#include "control/AirbrakeController.h"
#include "storage/RecordingSink.h"
#include "navigation/SensorPipeline.h"
#include "shared/Timing.h"
#include "flight/OutputExecutor.h"
#include "hardware/BatteryMonitor.h"

namespace flight {
struct FrameResult {
  uint32_t timeMs = 0, computeUs = 0;
  navigation::NavigationOutput navigation;
  RunEvaluation events;
};
struct RuntimeStatus {
  control::AirbrakeStatus airbrakes;
  bool active = false, timingValid = false;
  storage::OutputMode outputMode = storage::OutputMode::Virtual;
  bool outputsActive = false;
  uint32_t samples = 0, timeMs = 0, maxComputeUs = 0;
  bool recordingRequested = false, recordingActive = false, samplesFull = false;
  bool bistage = false;
  recovery::AbortReason abortReason = recovery::AbortReason::None;
  float coastVelocityMedian = 0, tiltRadians = 0;
  bool boostReached = false, recordingDiscarded = false;
  uint64_t recordingSequence = 0;
  uint32_t savedSamples = 0, droppedSamples = 0;
  bool recordingAsync = false, recordingBusy = false;
  uint32_t pendingSamples = 0, queueCapacity = 0;
  storage::RecorderError recordingError = storage::RecorderError::None;
};
struct RuntimeTiming {
  uint32_t prepareUs = 0, finishUs = 0, nominalPeriodUs = 0, overBudget = 0;
  timing::Statistics compute, storage, step;
  // Frame quality as navigation saw it: dt between consecutive IMU frames and breaks in the
  // IMU sequence (skipped simulation ticks, FIFO overruns or lost hardware frames).
  uint32_t frameDtMinUs = 0, frameDtMaxUs = 0, sequenceGaps = 0;
};
struct PollResult {
  bool processed = false;
  const char *error = nullptr;
};
// Source-independent sensor -> navigation -> rules -> recording loop.
// Sensor source and output mode are independent; virtual runs never touch the executor.
class FlightRuntime {
public:
  using Clock = timing::Clock;
  FlightRuntime(ConfigurationService &config, RunController &run, Clock clock = nullptr,
                storage::RecordingSink *recorder = nullptr, OutputExecutor *outputs = nullptr)
      : config_(config), run_(run), clock_(clock), recorder_(recorder), outputs_(outputs) {}
  FlightRuntime(const FlightRuntime &) = delete;
  FlightRuntime &operator=(const FlightRuntime &) = delete;
  const char *preparationError() const;
  // Calibration/reference are already prepared by the caller. Simulation skips
  // physical arming calibration; the runtime contains no source-specific calibration loop.
  const char *begin(sensors::SensorSource &source, float referenceAltitude, float nominalHz,
                    const navigation::SensorCalibration &calibration = navigation::SensorCalibration{},
                    uint32_t recordBytes = 0, storage::SensorMode sensorMode = storage::SensorMode::Simulation,
                    storage::OutputMode outputMode = storage::OutputMode::Virtual, bool asyncPreparation = false);
  bool preparing() const { return active() && preparing_; }
  PollResult poll(FrameResult &result);
  unsigned stop(storage::RecordingEnd reason = storage::RecordingEnd::Stopped);
  bool active() const { return started_ && identity_ == run_.identity() && run_.active(RunMode::Sensors); }
  bool blockedByOtherRun() const { return run_.active() && !active(); }
  RuntimeStatus status() const;
  void refreshRecordingStatus(RuntimeStatus &status) const;
  const RuntimeTiming &timing() const { return timing_; }
  void setBattery(const BatteryMonitor &battery) { battery_=&battery; }
private:
  const char *process(const sensors::Reading &input, FrameResult &result);
  void save(const sensors::Reading &input, const FrameResult &result);
  void recordingFault(storage::RecorderError error);
  bool applyAirbrakes();
  control::AirbrakeController airbrakes_;
  uint32_t lastControlUs_=0;
  ConfigurationService &config_;
  RunController &run_;
  Clock clock_;
  sensors::SensorSource *source_ = nullptr;
  navigation::SensorPipeline pipeline_;
  storage::RecordingSink *recorder_;
  OutputExecutor *outputs_;
  const BatteryMonitor *battery_ = nullptr;
  storage::OutputMode outputMode_ = storage::OutputMode::Virtual;
  bool recordingRequested_ = false, full_ = false;
  bool boostReached_ = false, processingFailed_ = false;
  storage::SensorMode sensorMode_ = storage::SensorMode::Simulation;
  uint64_t recordingSequence_ = 0;
  uint32_t saved_ = 0, dropped_ = 0;
  uint64_t lastBaroSequence_ = 0; // Barometer identity already sent to the recorder.
  storage::RecorderError recordingError_ = storage::RecorderError::None;
  uint64_t identity_ = 0;
  bool started_ = false;
  bool preparing_ = false;
  uint32_t preparationStartUs_ = 0;
  bool landed_ = false, closingStarts_ = false;
  uint32_t landingClockUs_ = 0;
  uint32_t maxComputeUs_ = 0;
  RuntimeTiming timing_;
  bool haveFrame_ = false;
  uint64_t lastFrameUs_ = 0, lastSequence_ = 0;
  recovery::StateMachineSettings machine_;
  recovery::AbortReason abortReason_ = recovery::AbortReason::None;
  float velocityMedian_ = 0, tilt_ = 0;
};
}
