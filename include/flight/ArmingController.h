#pragma once
#include "flight/FlightRuntime.h"
#include "flight/ArmingCalibration.h"

namespace flight {
enum class FlightMode { Physical, Simulation };
enum class ArmingState { Disarmed, Arming, Armed, Closing };
enum class ArmBlocker : uint8_t {
  Busy, DraftActive, ConfigInvalid, NotDisarmed, TimingUnavailable, ImuUnavailable,
  BarometerUnavailable, StorageNotReady, ContinuityFault, ServosNotPrepared,
  HardwareChecksUnavailable, ServosNotConfigured, PhysicalServosPrepared, OrientationNotReady, Count
};
struct Preflight {
  uint32_t blockers = 0;
  uint8_t failedPyros = 0, unpreparedServos = 0, unconfiguredServos = 0, preparedServos = 0;
  bool ready() const { return blockers == 0; }
  void block(ArmBlocker b) { blockers |= uint32_t(1) << unsigned(b); }
  bool has(ArmBlocker b) const { return (blockers & (uint32_t(1) << unsigned(b))) != 0; }
  const char *error() const;
};
struct PhysicalReadiness {
  bool imuReady = false, baroReady = false;
};
class PhysicalChecks {
public:
  virtual ~PhysicalChecks() = default;
  // Real sensor readiness only. Actuator checks are shared through OutputExecutor.
  virtual PhysicalReadiness inspect() = 0;
};
const char *blockerName(ArmBlocker blocker);
const char *armingStateName(ArmingState state);
const char *flightModeName(FlightMode mode);

// Shared lifecycle; physical simulation checks outputs but skips real sensors/calibration.
class ArmingController {
public:
  ArmingController(ConfigurationService &config, RunController &run, timing::Clock clock = nullptr,
                   storage::RecordingSink *recorder = nullptr, OutputExecutor *outputs = nullptr)
      : config_(config), clock_(clock), recorder_(recorder), runtime_(config, run, clock, recorder, outputs), outputs_(outputs) {}
  Preflight preflight(FlightMode mode, bool record, PhysicalChecks *checks = nullptr,
                      storage::OutputMode outputMode = storage::OutputMode::Virtual) const;
  const char *arm(sensors::SensorSource &source, FlightMode mode, float reference, float hz,
                  const navigation::SensorCalibration &device = navigation::SensorCalibration{},
                  uint32_t recordBytes = 0, PhysicalChecks *checks = nullptr,
                  storage::OutputMode outputMode = storage::OutputMode::Virtual);
  PollResult poll(FrameResult &result);
  // Explicit development stop also supports virtual replay cancellation mid-flight.
  unsigned stop(storage::RecordingEnd reason = storage::RecordingEnd::Stopped);
  const char *disarmError() const;
  const char *disarm(); // Ordinary flight disarm: PAD/LANDED or cancel an arming attempt.
  const char *releaseServos();
  const char *releaseForVirtualSimulation(bool record);
  const char *prepareServos(uint32_t channel = 0); // Zero selects every assigned servo.
  const char *positionServo(uint32_t channel, uint32_t pulseUs); // Zero PWM releases only this preview.
  bool hasServoPreview() const { return outputs_ && outputs_->hasPreview(); }
  void releaseServoPreview() { if (outputs_) outputs_->releasePreview(); }
  const char *checkPyroContinuity(uint8_t &configured, uint8_t &connected) const; // Read-only; no cached preflight approval.
  ArmingState state() const;
  bool active() const { return state() == ArmingState::Arming || (state() == ArmingState::Armed && runtime_.active()); }
  uint8_t deviceUpAxis() const { return config_.upAxis(); }
  bool orientationReady() const { return config_.orientationReady(); }
  bool armed() const { return state() == ArmingState::Armed && runtime_.active(); }
  FlightMode mode() const { return mode_; }
  storage::OutputMode outputMode() const { return mode_==FlightMode::Physical ? storage::OutputMode::Physical : outputMode_; }
  bool outputsActive() const { return (outputs_ && outputs_->hasPreparation()) || runtime_.status().outputsActive; }
  bool hasPreparedServos() const { return outputs_ && outputs_->hasPreparation(); }
  const char *error() const { return error_; }
  const Preflight &lastPreflight() const { return lastPreflight_; }
  uint32_t calibrationSamples() const { return calibration_.samples(); }
  const navigation::NavigationOutput &navigation() const { return navigation_; }
  const FlightRuntime &runtime() const { return runtime_; }
  void setBattery(const BatteryMonitor &battery) { runtime_.setBattery(battery); }
  void refreshRecordingStatus(RuntimeStatus &status) const { runtime_.refreshRecordingStatus(status); }
private:
  const char *startRuntime(float reference, const navigation::SensorCalibration &calibration);
  ConfigurationService &config_;
  timing::Clock clock_;
  storage::RecordingSink *recorder_;
  FlightRuntime runtime_;
  OutputExecutor *outputs_;
  storage::OutputMode outputMode_ = storage::OutputMode::Virtual;
  ArmingCalibration calibration_;
  sensors::SensorSource *source_ = nullptr;
  PhysicalChecks *checks_ = nullptr;
  FlightMode mode_ = FlightMode::Physical;
  ArmingState state_ = ArmingState::Disarmed;
  bool reserved_ = false;
  float hz_ = 0;
  uint32_t recordBytes_ = 0;
  navigation::NavigationOutput navigation_;
  const char *error_ = "None";
  Preflight lastPreflight_;
};
}
