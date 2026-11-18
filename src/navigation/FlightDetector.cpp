#include "navigation/FlightDetector.h"
#include <cmath>
#include <algorithm>

namespace navigation {
void FlightDetector::reset(const recovery::StateMachineSettings &settings) {
  settings_=settings; phase_=Phase::Pad; stable_=minimum_=maximum_=0; window_=false;
  samples_=0; velocities_={}; median_=0; abortReason_=recovery::AbortReason::None;
  boostSamples_=coastSamples_=0;
}
void FlightDetector::confirmCoastVelocity(float velocity) {
  if (!settings_.bistage || phase_!=Phase::Coast || samples_>=8 || !std::isfinite(velocity)) return;
  velocities_[samples_++]=velocity;
  if (samples_==8) {
    auto sorted=velocities_; std::sort(sorted.begin(),sorted.end()); median_=sorted[3]*.5f+sorted[4]*.5f;
    if (median_<=settings_.minimumCoastVelocity) { phase_=Phase::Abort; abortReason_=recovery::AbortReason::Velocity; }
  }
}
void FlightDetector::update(float dt, float magnitude, float bodyX, float gyro, float baro, float velocity, float tilt) {
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.1f || !std::isfinite(magnitude) ||
      !std::isfinite(bodyX) || !std::isfinite(gyro) || !std::isfinite(velocity)) {
    stable_ = 0; window_ = false; interruptConfirmation(); return;
  }
  if (settings_.bistage && phase_==Phase::Coast && std::isfinite(tilt) && tilt>=40.0f*0.017453292519943295f) {
    phase_=Phase::Abort; abortReason_=recovery::AbortReason::Tilt;
  }
  switch (phase_) {
    case Phase::Pad:
      if (magnitude > 3.0f * 9.81f) { if (++boostSamples_>=3) phase_=Phase::Boost; }
      else interruptConfirmation();
      break;
    case Phase::Boost:
      if (bodyX>0) coastSamples_=0;
      else if (++coastSamples_>=3) {
        coastSamples_=0; phase_=Phase::Coast;
        if (settings_.bistage && std::isfinite(tilt) && tilt>=40.0f*0.017453292519943295f) {
          phase_=Phase::Abort; abortReason_=recovery::AbortReason::Tilt;
        }
      }
      break;
    case Phase::Coast:
      if (velocity<=-.5f) phase_=Phase::Descent;
      else if (settings_.bistage && samples_==8 && magnitude>3.0f*9.81f) phase_=Phase::Boost2;
      break;
    case Phase::Boost2:
      if (bodyX>0) coastSamples_=0;
      else if (++coastSamples_>=3) { coastSamples_=0; phase_=Phase::Coast2; }
      break;
    case Phase::Coast2: if (velocity<=-.5f) phase_=Phase::Descent; break;
    case Phase::Abort: break;
    case Phase::Descent: case Phase::Landed: break;
  }
  if (phase_ != Phase::Descent) { stable_ = 0; window_ = false; return; }
  if (std::fabs(magnitude - 9.81f) >= 2.0f || !std::isfinite(gyro) || gyro >= 0.5f) {
    stable_ = 0; window_ = false;
  } else if (std::isfinite(baro)) {
    if (!window_) { minimum_ = maximum_ = baro; window_ = true; }
    else { minimum_ = std::fmin(minimum_, baro); maximum_ = std::fmax(maximum_, baro); }
    if (maximum_ - minimum_ < 1.0f) {
      stable_ += dt; if (stable_ >= 2.0f) phase_ = Phase::Landed;
    } else { stable_ = 0; window_ = false; }
  } else if (window_) stable_ += dt; // Legacy: only a fresh baro can confirm landing.
}
}
