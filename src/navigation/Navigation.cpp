#include "navigation/Navigation.h"
#include <cmath>

namespace navigation {
float Navigation::pressureAltitude(float pressurePa) {
  // Same legacy standard-atmosphere conversion, with Pa explicit at the boundary.
  if (!std::isfinite(pressurePa) || pressurePa <= 0) return std::numeric_limits<float>::quiet_NaN();
  return 44330.0f * (1.0f - std::pow(pressurePa / 101325.0f, 0.1903f));
}
bool Navigation::begin(float referenceAltitude, float nominalImuHz, const std::array<float, 4> &initialQuaternion, const recovery::StateMachineSettings &settings) {
  initialized_ = false;
  if (!std::isfinite(referenceAltitude) || !std::isfinite(nominalImuHz) || nominalImuHz < 10 || !std::isfinite(settings.minimumCoastVelocity) || settings.minimumCoastVelocity<0 || !attitude_.reset(initialQuaternion)) return false;
  filter_.reset(); detector_.reset(settings); attitudeAligned_=!settings.bistage; gyroOnlyActive_=false; referenceAltitude_ = referenceAltitude; nominalDt_ = 1.0f / nominalImuHz;
  padBaseline_ = padStillSeconds_ = 0; haveImu_ = haveBaro_ = false;
  lastImuTimeUs_ = lastImuSequence_ = lastBaroSequence_ = 0;
  initialized_ = true; return true;
}
NavigationOutput Navigation::output(const SensorFrame &frame) const {
  NavigationOutput out;
  out.timeUs = frame.timeUs; out.sequence = frame.imuSequence; out.phase = detector_.phase();
  out.startsAllowed=detector_.startsAllowed(); out.abortReason=detector_.abortReason(); out.coastVelocityMedian=detector_.velocityMedian();
  out.upBody=attitude_.verticalInBody();
  out.tiltRadians=std::acos(std::fmax(-1.0f,std::fmin(1.0f,out.upBody[0])));
  if (gyroOnlyActive_) {
    out.gyroOnly=true; out.gyroOnlyUpBody=gyroOnly_.verticalInBody();
    out.gyroOnlyTiltRadians=std::acos(std::fmax(-1.0f,std::fmin(1.0f,out.gyroOnlyUpBody[0])));
  }
  for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) out.covariance[i][j]=filter_.covariance(i,j);
  out.altitude = filter_.altitude(); out.verticalVelocity = filter_.velocity();
  out.accelerationBias = filter_.bias(); out.eulerRadians = attitude_.eulerRadians(); out.padBaseline = padBaseline_;
  return out;
}
bool Navigation::startGyroOnly() {
  if (!initialized_ || !attitudeAligned_ || !gyroOnly_.reset(attitude_.quaternion())) return false;
  gyroOnlyActive_=true; return true;
}
NavigationOutput Navigation::step(const SensorFrame &frame) {
  auto out = output(frame);
  if (!initialized_ || (haveImu_ && (frame.timeUs <= lastImuTimeUs_ || frame.imuSequence <= lastImuSequence_))) {
    detector_.interruptConfirmation(); return out;
  }
  const float dt = haveImu_ ? float(frame.timeUs - lastImuTimeUs_) * 1e-6f : nominalDt_;
  if (haveImu_ && (frame.imuSequence!=lastImuSequence_+1 || dt>2*nominalDt_)) detector_.interruptConfirmation();
  // Consume fresh frames even if invalid: recovery must not integrate across gaps.
  lastImuTimeUs_ = frame.timeUs; lastImuSequence_ = frame.imuSequence; haveImu_ = true;
  if (!frame.imuValid || !std::isfinite(dt) || dt <= 0 || dt > 0.1f) { detector_.interruptConfirmation(); return out; }
  float accelNormSquared = 0, gyroNormSquared = 0;
  for (unsigned i = 0; i < 3; ++i) {
    if (!std::isfinite(frame.accel[i]) || !std::isfinite(frame.gyro[i])) { detector_.interruptConfirmation(); return out; }
    accelNormSquared += frame.accel[i]*frame.accel[i]; gyroNormSquared += frame.gyro[i]*frame.gyro[i];
  }
  if (!std::isfinite(accelNormSquared) || !std::isfinite(gyroNormSquared)) { detector_.interruptConfirmation(); return out; }
  if (!attitudeAligned_) {
    const float magnitude=std::sqrt(accelNormSquared);
    if (std::fabs(magnitude-9.80665f)>1.0f) return out;
    const float ax=frame.accel[0]/magnitude, ay=frame.accel[1]/magnitude, az=frame.accel[2]/magnitude;
    const std::array<float,4> aligned=az<-.9999f ? std::array<float,4>{{0,1,0,0}} : std::array<float,4>{{1+az,ay,-ax,0}};
    if (!attitude_.reset(aligned)) return out;
    attitudeAligned_=true;
  }
  if (!attitude_.step(frame.accel, frame.gyro, dt, detector_.navigationPhase())) { detector_.interruptConfirmation(); return out; }
  if (gyroOnlyActive_) { const float none[3]={0,0,0}; gyroOnly_.step(none, frame.gyro, dt, Phase::Coast); } // Zero force: accelerometer correction gate closed.
  const auto vertical = attitude_.verticalInBody();
  const float verticalForce = vertical[0]*frame.accel[0] + vertical[1]*frame.accel[1] + vertical[2]*frame.accel[2];
  float baroAltitude = std::numeric_limits<float>::quiet_NaN();
  const bool newBaro = !haveBaro_ || frame.baroSequence > lastBaroSequence_;
  if (newBaro && frame.baroValid) {
    lastBaroSequence_ = frame.baroSequence; haveBaro_ = true;
    if (frame.baroTimeUs <= frame.timeUs && frame.timeUs - frame.baroTimeUs <= 100000)
      baroAltitude = pressureAltitude(frame.pressurePa) - referenceAltitude_;
  }
  const float magnitude = std::sqrt(accelNormSquared);
  if (std::isfinite(baroAltitude)) {
    // Preserve legacy baseline scheduling: update only when the barometer is fresh.
    if (detector_.phase() == Phase::Pad) {
      if (std::fabs(magnitude - 9.81f) < 2.0f) {
        padStillSeconds_ += dt;
        if (padStillSeconds_ >= 3.0f) { const float alpha = dt / (10.0f + dt); padBaseline_ = (1-alpha)*padBaseline_ + alpha*baroAltitude; }
      } else padStillSeconds_ = 0;
    }
    baroAltitude -= padBaseline_;
  }
  detector_.update(dt, magnitude, frame.accel[0], std::sqrt(gyroNormSquared), baroAltitude, filter_.velocity(),std::acos(std::fmax(-1.0f,std::fmin(1.0f,vertical[0]))));
  // An ignition being confirmed is integrated as BOOST (see FlightDetector::navigationPhase).
  const Phase filterPhase=detector_.navigationPhase();
  if (!filter_.predict(dt, verticalForce, filterPhase)) return output(frame);
  filter_.constrainPadVelocity(filterPhase);
  bool corrected = false;
  if (std::isfinite(baroAltitude)) corrected = filter_.correctAltitude(baroAltitude, detector_.phase());
  if (std::isfinite(filter_.altitude()) && std::isfinite(filter_.velocity()) && std::isfinite(filter_.bias())) detector_.confirmCoastVelocity(filter_.velocity());
  out = output(frame); out.valid = std::isfinite(out.altitude) && std::isfinite(out.verticalVelocity) && std::isfinite(out.accelerationBias);
  out.dtSeconds=dt;
  out.verticalSpecificForce = verticalForce; out.verticalAcceleration = filter_.acceleration();
  out.barometerAltitude = baroAltitude; out.barometerUpdated = corrected; return out;
}
recovery::FlightSnapshot NavigationOutput::ruleSnapshot() const {
  recovery::FlightSnapshot snapshot;
  snapshot.startsAllowed=startsAllowed;
  snapshot.phase = phase; snapshot.phaseValid = valid; snapshot.phaseMeasuredMs = timeUs / 1000; snapshot.sequence = sequence;
  snapshot.altitude.value = altitude; snapshot.altitude.valid = valid;
  snapshot.altitude.measuredMs = timeUs / 1000; snapshot.altitude.sequence = sequence;
  snapshot.verticalVelocity = snapshot.altitude; snapshot.verticalVelocity.value = verticalVelocity; return snapshot;
}
}
