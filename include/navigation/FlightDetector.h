#pragma once
#include "navigation/VerticalKalman.h"

namespace navigation {
class FlightDetector {
public:
  void reset();
  void update(float dt, float accelerationMagnitude, float bodyX, float gyroMagnitude,
              float barometerAltitude, float previousVerticalVelocity);
  Phase phase() const { return phase_; }
private:
  Phase phase_ = Phase::Pad;
  float stable_ = 0, minimum_ = 0, maximum_ = 0;
  bool window_ = false;
};
}
