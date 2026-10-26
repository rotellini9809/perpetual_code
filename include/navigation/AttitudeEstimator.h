#pragma once
#include "navigation/VerticalKalman.h"
#include <array>

namespace navigation {
// IMU-only legacy Madgwick-style correction. Quaternion rotates BODY -> NAV Z-up.
class AttitudeEstimator {
public:
  // Stationary specific force points upward. Align it with NAV +Z; heading
  // is unobservable without an external reference (choose shortest rotation).
  static bool fromGravity(const float (&accel)[3], std::array<float,4> &quaternion);
  bool reset(const std::array<float, 4> &quaternion = {{1, 0, 0, 0}});
  bool step(const float (&accel)[3], const float (&gyro)[3], float dt, Phase phase);
  std::array<float, 3> verticalInBody() const;
  std::array<float, 3> eulerRadians() const;
  const std::array<float, 4> &quaternion() const { return q_; }
private:
  std::array<float, 4> q_{{1, 0, 0, 0}};
};
}
