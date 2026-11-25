#include "navigation/FlightDetector.h"
#include <cmath>
#include <algorithm>

namespace navigation {
void FlightDetector::reset(const recovery::StateMachineSettings &settings) {
  settings_=settings; phase_=Phase::Pad; landingStillSeconds_=landingAltitudeMin_=landingAltitudeMax_=0; landingWindowOpen_=false;
  coastVelocitySamples_=0; coastVelocities_={}; coastVelocityMedian_=0; abortReason_=recovery::AbortReason::None;
  boostElapsed_=coastElapsed_=-1;
}
void FlightDetector::confirmCoastVelocity(float velocity) {
  if (!settings_.bistage || phase_!=Phase::Coast || coastVelocitySamples_>=CoastVelocitySampleCount || !std::isfinite(velocity)) return;
  coastVelocities_[coastVelocitySamples_++]=velocity;
  if (coastVelocitySamples_==CoastVelocitySampleCount) {
    auto sorted=coastVelocities_; std::sort(sorted.begin(),sorted.end()); coastVelocityMedian_=sorted[3]*.5f+sorted[4]*.5f;
    if (coastVelocityMedian_<=settings_.minimumCoastVelocity) { phase_=Phase::Abort; abortReason_=recovery::AbortReason::Velocity; }
  }
}
bool FlightDetector::ignition(float magnitude, float dt) {
  if (boostElapsed_ < 0 ? magnitude > BoostTriggerMps2 : magnitude > BoostHoldMps2) return confirm(boostElapsed_,dt,BoostConfirmSeconds);
  boostElapsed_=-1; return false;
}
void FlightDetector::update(float dt, float accelerationMagnitude, float bodyX, float gyroMagnitude, float barometerAltitude, float previousVerticalVelocity, float tiltRadians) {
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.1f || !std::isfinite(accelerationMagnitude) ||
      !std::isfinite(bodyX) || !std::isfinite(gyroMagnitude) || !std::isfinite(previousVerticalVelocity)) {
    landingStillSeconds_ = 0; landingWindowOpen_ = false; interruptConfirmation(); return;
  }
  if (settings_.bistage && phase_==Phase::Coast && std::isfinite(tiltRadians) && tiltRadians>=BistageAbortTiltRadians) {
    phase_=Phase::Abort; abortReason_=recovery::AbortReason::Tilt;
  }
  switch (phase_) {
    case Phase::Pad:
      if (ignition(accelerationMagnitude,dt)) phase_=Phase::Boost;
      break;
    case Phase::Boost:
      if (bodyX>0) coastElapsed_=-1;
      else if (confirm(coastElapsed_,dt,CoastConfirmSeconds)) {
        phase_=Phase::Coast;
        if (settings_.bistage && std::isfinite(tiltRadians) && tiltRadians>=BistageAbortTiltRadians) {
          phase_=Phase::Abort; abortReason_=recovery::AbortReason::Tilt;
        }
      }
      break;
    case Phase::Coast:
      if (previousVerticalVelocity<=-.5f) phase_=Phase::Descent;
      else if (settings_.bistage && coastVelocitySamples_==CoastVelocitySampleCount && ignition(accelerationMagnitude,dt)) phase_=Phase::Boost2; // Same rule as launch.
      break;
    case Phase::Boost2:
      if (bodyX>0) coastElapsed_=-1;
      else if (confirm(coastElapsed_,dt,CoastConfirmSeconds)) phase_=Phase::Coast2;
      break;
    case Phase::Coast2: if (previousVerticalVelocity<=-.5f) phase_=Phase::Descent; break;
    case Phase::Abort: break;
    case Phase::Descent: case Phase::Landed: break;
  }
  if (phase_ != Phase::Descent) { landingStillSeconds_ = 0; landingWindowOpen_ = false; return; }
  if (std::fabs(accelerationMagnitude - 9.81f) >= 2.0f || !std::isfinite(gyroMagnitude) || gyroMagnitude >= 0.5f) {
    landingStillSeconds_ = 0; landingWindowOpen_ = false;
  } else if (std::isfinite(barometerAltitude)) {
    if (!landingWindowOpen_) { landingAltitudeMin_ = landingAltitudeMax_ = barometerAltitude; landingWindowOpen_ = true; }
    else { landingAltitudeMin_ = std::fmin(landingAltitudeMin_, barometerAltitude); landingAltitudeMax_ = std::fmax(landingAltitudeMax_, barometerAltitude); }
    if (landingAltitudeMax_ - landingAltitudeMin_ < 1.0f) {
      landingStillSeconds_ += dt; if (landingStillSeconds_ >= 2.0f) phase_ = Phase::Landed;
    } else { landingStillSeconds_ = 0; landingWindowOpen_ = false; }
  } else if (landingWindowOpen_) landingStillSeconds_ += dt; // Legacy: only a fresh barometer reading can confirm landing.
}
}
