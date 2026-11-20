#include "navigation/Navigation.h"
#include <cmath>

namespace navigation {
float Navigation::pressureAltitude(float pressurePa) {
  // Same legacy standard-atmosphere conversion, with Pa explicit at the boundary.
  if (!std::isfinite(pressurePa) || pressurePa <= 0) return std::numeric_limits<float>::quiet_NaN();
  return 44330.0f * (1.0f - std::pow(pressurePa / 101325.0f, 0.1903f));
}
bool Navigation::begin(float reference, float hz, const std::array<float, 4> &q, const recovery::StateMachineSettings &settings) {
  initialized_ = false;
  if (!std::isfinite(reference) || !std::isfinite(hz) || hz < 10 || !std::isfinite(settings.minimumCoastVelocity) || settings.minimumCoastVelocity<0 || !attitude_.reset(q)) return false;
  filter_.reset(); detector_.reset(settings); attitudeAligned_=!settings.bistage; gyroOnlyActive_=false; reference_ = reference; firstDt_ = 1.0f / hz;
  baseline_ = padGate_ = 0; haveImu_ = haveBaro_ = false;
  lastTime_ = lastImuSequence_ = lastBaroSequence_ = 0;
  initialized_ = true; return true;
}
NavigationOutput Navigation::output(const SensorFrame &f) const {
  NavigationOutput out;
  out.timeUs = f.timeUs; out.sequence = f.imuSequence; out.phase = detector_.phase();
  out.startsAllowed=detector_.startsAllowed(); out.abortReason=detector_.abortReason(); out.coastVelocityMedian=detector_.velocityMedian();
  out.upBody=attitude_.verticalInBody();
  out.tiltRadians=std::acos(std::fmax(-1.0f,std::fmin(1.0f,out.upBody[0])));
  if (gyroOnlyActive_) {
    out.gyroOnly=true; out.gyroOnlyUpBody=gyroOnly_.verticalInBody();
    out.gyroOnlyTiltRadians=std::acos(std::fmax(-1.0f,std::fmin(1.0f,out.gyroOnlyUpBody[0])));
  }
  for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) out.covariance[i][j]=filter_.covariance(i,j);
  out.altitude = filter_.altitude(); out.verticalVelocity = filter_.velocity();
  out.accelerationBias = filter_.bias(); out.eulerRadians = attitude_.eulerRadians(); out.padBaseline = baseline_;
  return out;
}
bool Navigation::startGyroOnly() {
  if (!initialized_ || !attitudeAligned_ || !gyroOnly_.reset(attitude_.quaternion())) return false;
  gyroOnlyActive_=true; return true;
}
NavigationOutput Navigation::step(const SensorFrame &f) {
  auto out = output(f);
  if (!initialized_ || (haveImu_ && (f.timeUs <= lastTime_ || f.imuSequence <= lastImuSequence_))) {
    detector_.interruptConfirmation(); return out;
  }
  const float dt = haveImu_ ? float(f.timeUs - lastTime_) * 1e-6f : firstDt_;
  if (haveImu_ && (f.imuSequence!=lastImuSequence_+1 || dt>2*firstDt_)) detector_.interruptConfirmation();
  // Consume fresh frames even if invalid: recovery must not integrate across gaps.
  lastTime_ = f.timeUs; lastImuSequence_ = f.imuSequence; haveImu_ = true;
  if (!f.imuValid || !std::isfinite(dt) || dt <= 0 || dt > 0.1f) { detector_.interruptConfirmation(); return out; }
  float a2 = 0, g2 = 0;
  for (unsigned i = 0; i < 3; ++i) {
    if (!std::isfinite(f.accel[i]) || !std::isfinite(f.gyro[i])) { detector_.interruptConfirmation(); return out; }
    a2 += f.accel[i]*f.accel[i]; g2 += f.gyro[i]*f.gyro[i];
  }
  if (!std::isfinite(a2) || !std::isfinite(g2)) { detector_.interruptConfirmation(); return out; }
  if (!attitudeAligned_) {
    const float magnitude=std::sqrt(a2);
    if (std::fabs(magnitude-9.80665f)>1.0f) return out;
    const float ax=f.accel[0]/magnitude, ay=f.accel[1]/magnitude, az=f.accel[2]/magnitude;
    const std::array<float,4> aligned=az<-.9999f ? std::array<float,4>{{0,1,0,0}} : std::array<float,4>{{1+az,ay,-ax,0}};
    if (!attitude_.reset(aligned)) return out;
    attitudeAligned_=true;
  }
  if (!attitude_.step(f.accel, f.gyro, dt, detector_.navigationPhase())) { detector_.interruptConfirmation(); return out; }
  if (gyroOnlyActive_) { const float none[3]={0,0,0}; gyroOnly_.step(none, f.gyro, dt, Phase::Coast); } // Zero force: gate closed.
  const auto vertical = attitude_.verticalInBody();
  const float force = vertical[0]*f.accel[0] + vertical[1]*f.accel[1] + vertical[2]*f.accel[2];
  float baro = std::numeric_limits<float>::quiet_NaN();
  const bool newBaro = !haveBaro_ || f.baroSequence > lastBaroSequence_;
  if (newBaro && f.baroValid) {
    lastBaroSequence_ = f.baroSequence; haveBaro_ = true;
    if (f.baroTimeUs <= f.timeUs && f.timeUs - f.baroTimeUs <= 100000)
      baro = pressureAltitude(f.pressurePa) - reference_;
  }
  const float magnitude = std::sqrt(a2);
  if (std::isfinite(baro)) {
    // Preserve legacy baseline scheduling: update only when baro is fresh.
    if (detector_.phase() == Phase::Pad) {
      if (std::fabs(magnitude - 9.81f) < 2.0f) {
        padGate_ += dt;
        if (padGate_ >= 3.0f) { const float alpha = dt / (10.0f + dt); baseline_ = (1-alpha)*baseline_ + alpha*baro; }
      } else padGate_ = 0;
    }
    baro -= baseline_;
  }
  detector_.update(dt, magnitude, f.accel[0], std::sqrt(g2), baro, filter_.velocity(),std::acos(std::fmax(-1.0f,std::fmin(1.0f,vertical[0]))));
  // An ignition being confirmed is integrated as BOOST (see FlightDetector::navigationPhase).
  const Phase filterPhase=detector_.navigationPhase();
  if (!filter_.predict(dt, force, filterPhase)) return output(f);
  filter_.constrainPadVelocity(filterPhase);
  bool corrected = false;
  if (std::isfinite(baro)) corrected = filter_.correctAltitude(baro, detector_.phase());
  if (std::isfinite(filter_.altitude()) && std::isfinite(filter_.velocity()) && std::isfinite(filter_.bias())) detector_.confirmCoastVelocity(filter_.velocity());
  out = output(f); out.valid = std::isfinite(out.altitude) && std::isfinite(out.verticalVelocity) && std::isfinite(out.accelerationBias);
  out.dtSeconds=dt;
  out.verticalSpecificForce = force; out.verticalAcceleration = filter_.acceleration();
  out.barometerAltitude = baro; out.barometerUpdated = corrected; return out;
}
recovery::FlightSnapshot NavigationOutput::ruleSnapshot() const {
  recovery::FlightSnapshot s;
  s.startsAllowed=startsAllowed;
  s.phase = phase; s.phaseValid = valid; s.phaseMeasuredMs = timeUs / 1000; s.sequence = sequence;
  s.altitude.value = altitude; s.altitude.valid = valid;
  s.altitude.measuredMs = timeUs / 1000; s.altitude.sequence = sequence;
  s.verticalVelocity = s.altitude; s.verticalVelocity.value = verticalVelocity; return s;
}
}
