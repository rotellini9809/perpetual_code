#include "navigation/FlightDetector.h"
#include <cmath>

namespace navigation {
void FlightDetector::reset() { phase_ = Phase::Pad; stable_ = minimum_ = maximum_ = 0; window_ = false; }
void FlightDetector::update(float dt, float magnitude, float bodyX, float gyro, float baro, float velocity) {
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.1f || !std::isfinite(magnitude) ||
      !std::isfinite(bodyX) || !std::isfinite(gyro) || !std::isfinite(velocity)) {
    stable_ = 0; window_ = false; return;
  }
  switch (phase_) {
    case Phase::Pad: if (magnitude > 3.0f * 9.81f) phase_ = Phase::Boost; break;
    case Phase::Boost: if (bodyX <= 0) phase_ = Phase::Coast; break;
    case Phase::Coast: if (velocity <= -0.5f) phase_ = Phase::Descent; break;
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
