#pragma once
#include "navigation/VerticalKalman.h"
#include <array>

namespace navigation {
class FlightDetector {
public:
  void reset(const recovery::StateMachineSettings &settings = recovery::StateMachineSettings{});
  void update(float dt, float accelerationMagnitude, float bodyX, float gyroMagnitude,
              float barometerAltitude, float previousVerticalVelocity, float tiltRadians = 0);
  void confirmCoastVelocity(float velocity);
  bool startsAllowed() const { return phase_!=Phase::Abort && !(settings_.bistage && phase_==Phase::Coast && samples_<8); }
  bool bistage() const { return settings_.bistage; }
  float velocityMedian() const { return median_; }
  recovery::AbortReason abortReason() const { return abortReason_; }
  Phase phase() const { return phase_; }
private:
  recovery::StateMachineSettings settings_;
  std::array<float,8> velocities_{};
  unsigned samples_ = 0;
  float median_ = 0;
  recovery::AbortReason abortReason_ = recovery::AbortReason::None;
  Phase phase_ = Phase::Pad;
  float stable_ = 0, minimum_ = 0, maximum_ = 0;
  bool window_ = false;
};
}
