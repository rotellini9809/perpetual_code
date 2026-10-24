#include "flight/ArmingCalibration.h"
#include <cmath>

namespace flight {
void ArmingCalibration::resetWindow() {
  samples_ = pressures_ = 0; altitudeSum_ = 0;
  for (auto &sum : gyroSum_) sum = 0;
}
void ArmingCalibration::begin(const navigation::SensorCalibration &device) {
  clear(); device_ = device; device_.configured = false;
  for (auto &bias : device_.gyroBias) bias = 0; // Never reuse another arming session's gyro bias.
}
bool ArmingCalibration::add(const sensors::Reading &r) {
  if (ready()) return true;
  if (haveImu_ && (r.timeUs <= lastTime_ || r.imuSequence <= lastSequence_)) return false;
  if (haveImu_ && r.timeUs-lastTime_ > 100000) resetWindow(); // Unknown motion across an acquisition gap.
  lastTime_ = r.timeUs; lastSequence_ = r.imuSequence; haveImu_ = true;
  const uint64_t baroTime = haveBaro_ && r.baroSequence == lastBaroSequence_ ? lastBaroTime_ : r.baroTimeUs;
  bool valid = r.imuValid && r.baroValid && baroTime <= r.timeUs && r.timeUs - baroTime <= 100000;
  if (haveBaro_ && (r.baroSequence < lastBaroSequence_ || (r.baroSequence > lastBaroSequence_ && r.baroTimeUs <= lastBaroTime_))) valid = false;
  float a2 = 0, g2 = 0;
  for (unsigned i = 0; i < 3; ++i) {
    const float a = r.accelMps2[i] - device_.accelBias[i];
    valid = valid && std::isfinite(a) && std::isfinite(r.gyroRadps[i]);
    a2 += a*a; g2 += r.gyroRadps[i]*r.gyroRadps[i];
  }
  // Development stationary gate; qualify thresholds with the repaired physical IMU.
  valid = valid && std::isfinite(a2) && std::isfinite(g2) && std::fabs(std::sqrt(a2)-9.80665f) <= 1.0f && g2 <= 0.25f;
  const float altitude = navigation::Navigation::pressureAltitude(r.pressurePa);
  valid = valid && std::isfinite(altitude);
  if (!valid) { resetWindow(); return false; }
  if (!haveBaro_ || r.baroSequence > lastBaroSequence_) {
    lastBaroSequence_ = r.baroSequence; lastBaroTime_ = r.baroTimeUs; haveBaro_ = true;
    altitudeSum_ += altitude; ++pressures_;
  }
  if (samples_ && r.timeUs - acceptedTime_ < SampleSpacingUs) return false;
  acceptedTime_ = r.timeUs;
  for (unsigned i = 0; i < 3; ++i) gyroSum_[i] += r.gyroRadps[i];
  ++samples_; return ready();
}
navigation::SensorCalibration ArmingCalibration::calibration() const {
  auto result = device_; result.configured = ready();
  if (ready()) for (unsigned i = 0; i < 3; ++i) result.gyroBias[i] = float(gyroSum_[i] / samples_);
  return result;
}
float ArmingCalibration::referenceAltitude() const {
  return ready() ? float(altitudeSum_ / pressures_) : std::numeric_limits<float>::quiet_NaN();
}
}
