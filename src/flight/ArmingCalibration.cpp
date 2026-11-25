#include "flight/ArmingCalibration.h"
#include <cmath>

namespace flight {
void ArmingCalibration::resetWindow() {
  samples_ = pressureSamples_ = 0; altitudeSum_ = 0;
  for (auto &sum : gyroSum_) sum = 0;
  for (auto &sum : accelSum_) sum = 0;
}
void ArmingCalibration::begin(const navigation::SensorCalibration &baseCalibration) {
  clear(); baseCalibration_ = baseCalibration; baseCalibration_.configured = false;
  baseCalibration_.initialQuaternion={{1,0,0,0}}; // Never reuse a previous arming attitude.
  for (auto &bias : baseCalibration_.gyroBias) bias = 0; // Never reuse another arming session's gyro bias.
}
bool ArmingCalibration::add(const sensors::Reading &reading) {
  if (ready()) return true;
  if (haveImu_ && (reading.timeUs <= lastImuTimeUs_ || reading.imuSequence <= lastImuSequence_)) return false;
  if (haveImu_ && reading.timeUs-lastImuTimeUs_ > 100000) resetWindow(); // Unknown motion across an acquisition gap.
  lastImuTimeUs_ = reading.timeUs; lastImuSequence_ = reading.imuSequence; haveImu_ = true;
  const uint64_t baroTime = haveBaro_ && reading.baroSequence == lastBaroSequence_ ? lastBaroTimeUs_ : reading.baroTimeUs;
  bool valid = reading.imuValid && reading.baroValid && baroTime <= reading.timeUs && reading.timeUs - baroTime <= 100000;
  if (haveBaro_ && (reading.baroSequence < lastBaroSequence_ || (reading.baroSequence > lastBaroSequence_ && reading.baroTimeUs <= lastBaroTimeUs_))) valid = false;
  float accelNormSquared = 0, gyroNormSquared = 0;
  for (unsigned i = 0; i < 3; ++i) {
    const float accel = (reading.accelMps2[i] - baseCalibration_.accelBias[i])*baseCalibration_.accelScale[i];
    valid = valid && std::isfinite(accel) && std::isfinite(reading.gyroRadps[i]);
    accelNormSquared += accel*accel; gyroNormSquared += reading.gyroRadps[i]*reading.gyroRadps[i];
  }
  // Development stationary gate; qualify thresholds with the physical IMU.
  valid = valid && std::isfinite(accelNormSquared) && std::isfinite(gyroNormSquared) && std::fabs(std::sqrt(accelNormSquared)-9.80665f) <= 1.0f && gyroNormSquared <= 0.25f;
  const float altitude = navigation::Navigation::pressureAltitude(reading.pressurePa);
  valid = valid && std::isfinite(altitude);
  if (!valid) { resetWindow(); return false; }
  if (!haveBaro_ || reading.baroSequence > lastBaroSequence_) {
    lastBaroSequence_ = reading.baroSequence; lastBaroTimeUs_ = reading.baroTimeUs; haveBaro_ = true;
    altitudeSum_ += altitude; ++pressureSamples_;
  }
  if (samples_ && reading.timeUs - lastAcceptedUs_ < SampleSpacingUs) return false;
  lastAcceptedUs_ = reading.timeUs;
  for (unsigned i = 0; i < 3; ++i) {
    gyroSum_[i] += reading.gyroRadps[i];
    accelSum_[i] += (reading.accelMps2[i]-baseCalibration_.accelBias[i])*baseCalibration_.accelScale[i];
  }
  // Opposing directions must not average to a fictitious stationary gravity: the mean vector must
  // keep a magnitude within 1 m/s^2 of g (each sample already passed the same gate on its own).
  if (samples_+1==TargetSamples) {
    double accelSumNormSquared=0; for (auto sum:accelSum_) accelSumNormSquared+=sum*sum;
    if (std::sqrt(accelSumNormSquared)/TargetSamples<8.80665) { resetWindow(); return false; }
  }
  ++samples_; return ready();
}
navigation::SensorCalibration ArmingCalibration::calibration() const {
  auto result = baseCalibration_; result.configured = ready();
  if (ready()) {
    float sensor[3],body[3];
    for (unsigned i = 0; i < 3; ++i) {
      result.gyroBias[i] = float(gyroSum_[i] / samples_);
      sensor[i]=float(accelSum_[i]/samples_);
    }
    if (unsigned(result.upAxis)>=6) { result.configured=false; return result; }
    // The same bias-corrected SENSOR -> BODY rotation used during flight.
    navigation::sensorToBody(result.upAxis,sensor,body);
    result.configured=navigation::AttitudeEstimator::fromGravity(body,result.initialQuaternion);
  }
  return result;
}
float ArmingCalibration::referenceAltitude() const {
  return ready() ? float(altitudeSum_ / pressureSamples_) : std::numeric_limits<float>::quiet_NaN();
}
}
