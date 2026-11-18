#include "flight/ArmingController.h"
#include "storage/FlashLayout.h"
#include <cmath>
#include <cstring>

namespace flight {
const char *blockerName(ArmBlocker b) {
  switch (b) {
    case ArmBlocker::Busy: return "BUSY";
    case ArmBlocker::DraftActive: return "DRAFT_ACTIVE";
    case ArmBlocker::ConfigInvalid: return "CONFIG_INVALID";
    case ArmBlocker::NotDisarmed: return "ARMED";
    case ArmBlocker::TimingUnavailable: return "TIMING_UNAVAILABLE";
    case ArmBlocker::ImuUnavailable: return "IMU_UNAVAILABLE";
    case ArmBlocker::BarometerUnavailable: return "BAROMETER_UNAVAILABLE";
    case ArmBlocker::StorageNotReady: return "STORAGE_NOT_READY";
    case ArmBlocker::ContinuityFault: return "CONTINUITY_FAULT";
    case ArmBlocker::ServosNotPrepared: return "SERVOS_NOT_PREPARED";
    case ArmBlocker::ServosNotConfigured: return "SERVOS_NOT_CONFIGURED";
    case ArmBlocker::OrientationNotReady: return "ORIENTATION_NOT_READY";
    case ArmBlocker::ImuCalibrationMissing: return "IMU_CALIBRATION_MISSING";
    case ArmBlocker::ImuVerificationRequired: return "IMU_VERIFICATION_REQUIRED";
    case ArmBlocker::PhysicalServosPrepared: return "PHYSICAL_SERVOS_PREPARED";
    case ArmBlocker::HardwareChecksUnavailable: return "HARDWARE_CHECKS_UNAVAILABLE";
    case ArmBlocker::Count: break;
  }
  return "UNKNOWN";
}
const char *Preflight::error() const {
  for (unsigned i = 0; i < unsigned(ArmBlocker::Count); ++i)
    if (has(ArmBlocker(i))) return blockerName(ArmBlocker(i));
  return nullptr;
}
const char *armingStateName(ArmingState s) {
  switch (s) {
    case ArmingState::Disarmed: return "disarmed";
    case ArmingState::Arming: return "arming";
    case ArmingState::Preparing: return "preparing";
    case ArmingState::Armed: return "armed";
    case ArmingState::Closing: return "closing";
  }
  return "unknown";
}
const char *flightModeName(FlightMode m) { return m == FlightMode::Simulation ? "simulation" : "physical"; }
const char *ArmingController::checkPyroContinuity(uint8_t &configured, uint8_t &connected) const {
  configured = connected = 0;
  if (state() != ArmingState::Disarmed || config_.runLocked() || runtime_.blockedByOtherRun() ||
      (recorder_ && recorder_->busy()) || (outputs_ && outputs_->owner())) return "BUSY";
  if (!config_.disarmed()) return "ARMED";
  if (config_.hasDraft()) return "DRAFT_ACTIVE";
  const auto *plan = config_.applied();
  if (!plan) return "CONFIG_INVALID";
  if (!outputs_) return "HARDWARE_CHECKS_UNAVAILABLE";
  for (std::size_t i = 0; i < plan->ruleCount; ++i)
    for (std::size_t j = 0; j < plan->rules[i].assignmentCount; ++j) {
      const auto &a = plan->rules[i].assignments[j];
      if (a.kind == recovery::OutputKind::Pyro) configured |= uint8_t(1u << (a.channel - 1));
    }
  connected = outputs_->connectedPyros(configured);
  return nullptr;
}
Preflight ArmingController::preflight(FlightMode mode, bool record, PhysicalChecks *checks, storage::OutputMode outputMode) const {
  if (mode == FlightMode::Physical) outputMode = storage::OutputMode::Physical;
  Preflight p;
  p.imuCalibrationState=imuCalibrationState(); p.imuCalibrationReason=imuCalibrationReason();
  if (state() != ArmingState::Disarmed || config_.runLocked() || runtime_.blockedByOtherRun() || (recorder_ && recorder_->busy()) ||
      (outputs_ && (outputs_->hasPreview() || (outputMode == storage::OutputMode::Physical && outputs_->owner())))) {
    p.block(ArmBlocker::Busy); return p; // Never touch hardware while another owner is active.
  }
  if (config_.hasDraft()) p.block(ArmBlocker::DraftActive);
  if (!config_.configValid()) p.block(ArmBlocker::ConfigInvalid);
  if (mode==FlightMode::Physical && !config_.orientationReady()) p.block(ArmBlocker::OrientationNotReady);
  if (mode==FlightMode::Physical) {
    if (!imuStore_ || (!imuStore_->present() && !imuStore_->identityAllowed()) || imuStore_->needsCalibration()) p.block(ArmBlocker::ImuCalibrationMissing);
    else if (imuStore_->present() && !imuStore_->verified() && !approvalReserved_) p.block(ArmBlocker::ImuVerificationRequired);
  }
  if (!config_.disarmed()) p.block(ArmBlocker::NotDisarmed);
  if ((record || mode == FlightMode::Physical) && (!recorder_ || !recorder_->initialized())) p.block(ArmBlocker::StorageNotReady);
  if (mode == FlightMode::Simulation && outputMode == storage::OutputMode::Virtual) {
    return p;
  }
  if (!clock_) p.block(ArmBlocker::TimingUnavailable);
  if (outputMode == storage::OutputMode::Physical) {
    if (!outputs_) p.block(ArmBlocker::HardwareChecksUnavailable);
    else if (!outputs_->timingAvailable()) p.block(ArmBlocker::TimingUnavailable);
  }
  uint8_t pyros = 0, servos = 0;
  if (const auto *config = config_.applied()) {
    servos=config_.airbrakesEnabled() ? config->airbrakes.channelMask : 0;
    for (std::size_t i = 0; i < config->ruleCount; ++i)
      for (std::size_t j = 0; j < config->rules[i].assignmentCount; ++j) {
        const auto &a = config->rules[i].assignments[j];
        (a.kind == recovery::OutputKind::Pyro ? pyros : servos) |= uint8_t(1u << (a.channel-1));
      }
    for (std::size_t i = 0; i < recovery::ChannelCount; ++i)
      if ((servos & (1u << i)) && !config->servos[i].calibrated)
        p.unconfiguredServos |= uint8_t(1u << i);
    if (outputs_) p.preparedServos = uint8_t(servos & outputs_->preparedServos(*config));
  }
  if (p.unconfiguredServos) p.block(ArmBlocker::ServosNotConfigured);
  if (outputs_) p.failedPyros = uint8_t(pyros & ~outputs_->connectedPyros(pyros));
  if (p.failedPyros) p.block(ArmBlocker::ContinuityFault);
  if (mode == FlightMode::Simulation) {
    // Initial servo positions are commanded during runtime preparation. Real
    // sensors and gyro calibration are irrelevant to this simulated source.
    return p;
  }
  if (!checks) { p.block(ArmBlocker::HardwareChecksUnavailable); return p; }
  const auto r = checks->inspect();
  if (!r.imuReady) p.block(ArmBlocker::ImuUnavailable);
  if (!r.baroReady) p.block(ArmBlocker::BarometerUnavailable);
  p.unpreparedServos = uint8_t(servos & ~p.preparedServos & ~p.unconfiguredServos);
  if (p.unpreparedServos) p.block(ArmBlocker::ServosNotPrepared);
  return p;
}
ArmingState ArmingController::state() const {
  if (state_ == ArmingState::Closing && !runtime_.status().recordingBusy) return ArmingState::Disarmed;
  if (state_ == ArmingState::Armed && !runtime_.active()) return ArmingState::Disarmed;
  return state_;
}
const char *ArmingController::startRuntime(float reference, const navigation::SensorCalibration &calibration) {
  // Core 0 is cooperative: transfer the configuration reservation without yielding.
  if (reserved_) { config_.releaseRun(); reserved_ = false; }
  const auto error = runtime_.begin(*source_, reference, hz_, calibration, recordBytes_,
      mode_ == FlightMode::Simulation ? storage::SensorMode::Simulation : storage::SensorMode::Hardware, outputMode_, mode_==FlightMode::Physical);
  if (error) { error_ = error; stop(); return error; }
  if (runtime_.preparing()) state_=ArmingState::Preparing;
  else { source_->start(); state_=ArmingState::Armed; }
  return nullptr;
}
const char *ArmingController::arm(sensors::SensorSource &source, FlightMode mode, float reference, float hz,
                                 const navigation::SensorCalibration &device, uint32_t bytes, PhysicalChecks *checks, storage::OutputMode outputMode) {
  if (mode != FlightMode::Simulation && mode != FlightMode::Physical) return "INVALID_ARGUMENT";
  if (mode == FlightMode::Physical) outputMode = storage::OutputMode::Physical;
  if (outputMode != storage::OutputMode::Virtual && outputMode != storage::OutputMode::Physical) return "INVALID_ARGUMENT";
  if (!std::isfinite(reference) || !std::isfinite(hz) || hz < 10 || bytes % storage::FlashLayout::SectorBytes ||
      (mode == FlightMode::Physical && (!bytes || source.convention() != sensors::FrameConvention::SensorAxes))) return "INVALID_ARGUMENT";
  if (mode == FlightMode::Physical) {
    if (unsigned(device.upAxis) >= 6) return "INVALID_ARGUMENT";
    for (auto bias : device.accelBias) if (!std::isfinite(bias)) return "INVALID_ARGUMENT";
  }
  lastPreflight_ = preflight(mode, bytes != 0, checks, outputMode);
  if (const auto error = lastPreflight_.error()) { error_ = error; return error; }
  if (source_) source_->stop();
  mode_ = mode; outputMode_ = outputMode; source_ = &source; checks_ = checks; hz_ = hz; recordBytes_ = bytes;
  navigation_ = {}; calibration_.clear(); error_ = "None"; state_ = ArmingState::Disarmed;
  if (mode == FlightMode::Simulation) return startRuntime(reference, device);
  if (!config_.acquireRun()) return "BUSY";
  reserved_ = true;
  // First physical arm persists identity so the existing approval journal also
  // tracks flights/resets when no custom accelerometer calibration was saved.
  if (!imuStore_->present()) {
    storage::ImuCalibrationRecord identity;
    identity.revision=1; identity.approval=storage::ImuApproval::Verified; identity.identity=true;
    if (!imuStore_->save(identity)) { error_="IMU_CALIBRATION_IO"; stop(); return error_; }
  }
  if (!imuStore_->approve(storage::ImuApproval::InUse)) {
    error_="IMU_CALIBRATION_IO"; stop(); return error_;
  }
  approvalReserved_=true; boostConsumed_=false;
  auto calibrated=device; imuStore_->apply(calibrated);
  calibration_.begin(calibrated); source_->start(); state_ = ArmingState::Arming; return nullptr;
}
PollResult ArmingController::poll(FrameResult &result) {
  result = {};
  if (imuCalibration_.active()) {
    if (clock_ && uint32_t(clock_()-imuOperationTime_)>180000000u) {
      imuError_="IMU_CALIBRATION_TIMEOUT"; cancelImuCalibration(); return {};
    }
    sensors::Reading r;
    if (source_->poll(r)) imuCalibration_.add(r);
    if (imuCalibration_.failed()) {
      // After use (Boost reached, or a reset while armed) the failure means the sensor changed: the
      // factory identity is revoked for good. Before any use only this calibration attempt is dropped.
      const bool afterUse=boostConsumed_ || imuStore_->record().approval==storage::ImuApproval::InUse;
      const bool stored=afterUse ? imuStore_->invalidate() : imuStore_->discard();
      imuError_=stored ? "IMU_VERIFICATION_FAILED" : "IMU_CALIBRATION_IO";
      cancelImuCalibration(); return {};
    }
    if (imuCalibration_.complete()) {
      storage::ImuCalibrationRecord record;
      if (!imuCalibration_.result(record)) imuError_="IMU_CALIBRATION_INVALID";
      else if (!(imuCalibration_.verification() ? imuStore_->approve(storage::ImuApproval::Verified) : imuStore_->save(record))) imuError_="IMU_CALIBRATION_IO";
      else boostConsumed_=false;
      cancelImuCalibration();
    }
    return {};
  }
  settleImuApproval();
  if (state_ == ArmingState::Arming) {
    sensors::Reading reading;
    if (source_->poll(reading) && calibration_.add(reading)) {
      // Recheck channels/health after calibration without dropping our reservation.
      const auto device = calibration_.calibration();
      const float reference = calibration_.referenceAltitude();
      config_.releaseRun(); reserved_ = false; state_ = ArmingState::Disarmed;
      lastPreflight_ = preflight(mode_, true, checks_, outputMode_);
      if (const auto error = lastPreflight_.error()) { error_ = error; stop(); PollResult p; p.error = error; return p; }
      if (const auto error = startRuntime(reference, device)) { PollResult p; p.error = error; return p; }
    }
    return {}; // Calibration frames never enter navigation/rules/recording.
  }
  if (state() == ArmingState::Closing) return {};
  const auto p = runtime_.poll(result);
  if (state_==ArmingState::Preparing && !runtime_.preparing() && !p.error) state_=ArmingState::Armed;
  if (approvalReserved_ && runtime_.status().boostReached) { boostConsumed_=true; approvalReserved_=false; }
  if (p.processed && result.navigation.valid) navigation_ = result.navigation;
  if (p.error && std::strcmp(p.error, "INVALID_TIMESTAMP")) { error_ = p.error; stop(); }
  else if (state_ == ArmingState::Armed && !runtime_.active()) {
    if (source_) source_->stop();
    source_=nullptr; checks_=nullptr; calibration_.clear();
    state_=runtime_.status().recordingBusy ? ArmingState::Closing : ArmingState::Disarmed;
  }
  return p;
}
unsigned ArmingController::stop(storage::RecordingEnd reason) {
  if (imuCalibration_.active()) cancelImuCalibration();
  if (reserved_) { config_.releaseRun(); reserved_ = false; }
  const auto pending = runtime_.stop(reason);
  if (source_) source_->stop();
  source_ = nullptr; checks_ = nullptr; calibration_.clear();
  state_ = runtime_.status().recordingBusy ? ArmingState::Closing : ArmingState::Disarmed;
  settleImuApproval();
  return pending;
}
const char *ArmingController::imuCalibrationState() const {
  if (!imuStore_) return "absent";
  if (!imuStore_->present()) return imuStore_->identityAllowed() ? "identity" : "absent";
  if (imuStore_->needsCalibration()) return "invalid";
  return (imuStore_->verified() || approvalReserved_) && !boostConsumed_ ? "verified" : "required";
}
const char *ArmingController::imuCalibrationReason() const {
  if (!imuStore_) return "MISSING";
  if (imuStore_->needsCalibration()) return "VERIFICATION_FAILED";
  if (!imuStore_->present()) return imuStore_->identityAllowed() ? "IDENTITY" : "MISSING";
  if (boostConsumed_) return "AFTER_BOOST";
  if (imuStore_->record().approval==storage::ImuApproval::InUse && !approvalReserved_) return "FLIGHT_OR_RESET";
  return !std::strcmp(imuCalibrationState(),"verified") ? "None" : "NOT_VERIFIED";
}
const char *ArmingController::beginImuCalibration(sensors::SensorSource &source,bool verify) {
  if (!imuStore_) return "IMU_CALIBRATION_UNAVAILABLE";
  if (!config_.storageReady()) return "STORAGE_NOT_READY";
  if (active() || state()!=ArmingState::Disarmed || config_.runLocked() || runtime_.blockedByOtherRun() ||
      (recorder_ && recorder_->busy()) || (outputs_ && (outputs_->owner() || outputs_->hasPreview()))) return "BUSY";
  if (!config_.disarmed()) return "ARMED";
  if (config_.hasDraft()) return "DRAFT_ACTIVE";
  if (verify && (!imuStore_->present() || imuStore_->needsCalibration())) return "IMU_CALIBRATION_MISSING";
  if (!clock_ || source.convention()!=sensors::FrameConvention::SensorAxes) return "INVALID_ARGUMENT";
  if (!config_.acquireDeviceOperation()) return "BUSY";
  reserved_=true; source_=&source; source_->start();
  imuCalibration_.begin(verify,imuStore_->record()); imuOperationTime_=clock_(); imuError_="None";
  return nullptr;
}
const char *ArmingController::captureImuFace(unsigned face) {
  if (!imuCalibration_.capture(face)) return "INVALID_ARGUMENT";
  imuOperationTime_=clock_(); return nullptr;
}
void ArmingController::cancelImuCalibration() {
  if (!imuCalibration_.active()) return;
  imuCalibration_.cancel();
  if (source_) source_->stop();
  source_=nullptr;
  if (reserved_) { config_.releaseRun(); reserved_=false; }
}
void ArmingController::settleImuApproval() {
  if (!approvalReserved_ || state()!=ArmingState::Disarmed || (recorder_ && recorder_->busy())) return;
  approvalReserved_=false;
  if (!imuStore_->approve(storage::ImuApproval::Verified)) imuError_="IMU_CALIBRATION_IO";
}
const char *ArmingController::disarmError() const {
  if (runtime_.blockedByOtherRun()) return "BUSY";
  if (armed() && navigation_.phase != recovery::FlightPhase::Pad && navigation_.phase != recovery::FlightPhase::Landed && navigation_.phase!=recovery::FlightPhase::Abort)
    return "DISARM_PROHIBITED_IN_FLIGHT";
  return nullptr;
}
const char *ArmingController::disarm() {
  if (const auto error = disarmError()) return error;
  stop(); return nullptr;
}
const char *ArmingController::releaseServos() {
  if (state()!=ArmingState::Disarmed || config_.runLocked() || runtime_.blockedByOtherRun() ||
      (recorder_ && recorder_->busy()) || (outputs_ && outputs_->owner())) return "BUSY";
  if (!config_.disarmed()) return "ARMED";
  if (!outputs_) return "HARDWARE_CHECKS_UNAVAILABLE";
  outputs_->releasePrepared(); return nullptr;
}
const char *ArmingController::releaseForVirtualSimulation(bool record) {
  lastPreflight_=preflight(FlightMode::Simulation,record,nullptr,storage::OutputMode::Virtual);
  if (outputs_ && outputs_->owner()) lastPreflight_.block(ArmBlocker::Busy);
  if (const auto error=lastPreflight_.error()) return error;
  if (outputs_ && outputs_->hasPreparation()) outputs_->releasePrepared();
  return nullptr;
}
const char *ArmingController::prepareServos(uint32_t channel) {
  if (state()!=ArmingState::Disarmed || config_.runLocked() || runtime_.blockedByOtherRun() || (recorder_ && recorder_->busy())) return "BUSY";
  if (!config_.disarmed()) return "ARMED";
  if (config_.hasDraft()) return "DRAFT_ACTIVE";
  if (!config_.applied()) return "CONFIG_INVALID";
  if (!outputs_) return "HARDWARE_CHECKS_UNAVAILABLE";
  if (channel>recovery::ChannelCount) return "INVALID_ARGUMENT";
  uint8_t assigned=0;
  const auto &config=*config_.applied();
  assigned=config.airbrakes.channelMask;
  for (std::size_t i=0;i<config.ruleCount;++i)
    for (std::size_t j=0;j<config.rules[i].assignmentCount;++j) {
      const auto &a=config.rules[i].assignments[j];
      if (a.kind==recovery::OutputKind::Servo) assigned |= uint8_t(1u<<(a.channel-1));
    }
  const uint8_t mask=channel ? uint8_t(1u<<(channel-1)) : assigned;
  if (mask & ~assigned) return "SERVO_NOT_ASSIGNED";
  outputs_->reconcile(&config);
  return outputs_->prepare(config,mask);
}
const char *ArmingController::positionServo(uint32_t channel, uint32_t pulseUs) {
  if (state()!=ArmingState::Disarmed || config_.runLocked() || runtime_.blockedByOtherRun() ||
      (recorder_ && recorder_->busy())) return "BUSY";
  if (!config_.disarmed()) return "ARMED";
  if (!outputs_) return "HARDWARE_CHECKS_UNAVAILABLE";
  const auto &limits=config_.limits();
  if (channel<1 || channel>recovery::ChannelCount || (pulseUs && (pulseUs<limits.minServoUs || pulseUs>limits.maxServoUs))) return "INVALID_ARGUMENT";
  if (!pulseUs) { outputs_->releasePreview(uint8_t(channel)); return nullptr; }
  return outputs_->preview(uint8_t(channel),uint16_t(pulseUs));
}
}
