#include "navigation/AttitudeEstimator.h"
#include <cmath>

namespace navigation {
bool AttitudeEstimator::reset(const std::array<float, 4> &q) {
  float norm = 0; for (float v : q) { if (!std::isfinite(v)) return false; norm += v * v; }
  if (!std::isfinite(norm) || norm < 1e-12f) return false;
  norm = 1.0f / std::sqrt(norm);
  q_ = q; for (auto &v : q_) v *= norm; return true;
}
bool AttitudeEstimator::step(const float (&a)[3], const float (&g)[3], float dt, Phase phase) {
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.1f) return false;
  for (unsigned i = 0; i < 3; ++i) if (!std::isfinite(a[i]) || !std::isfinite(g[i])) return false;
  const float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
  float derivative[4] = {0.5f * (-q1*g[0] - q2*g[1] - q3*g[2]),
      0.5f * (q0*g[0] + q2*g[2] - q3*g[1]),
      0.5f * (q0*g[1] - q1*g[2] + q3*g[0]),
      0.5f * (q0*g[2] + q1*g[1] - q2*g[0])};
  float beta = 0.3f;
  if (recovery::boostPhase(phase)) beta = 0.1f;
  else if (recovery::coastPhase(phase)) beta = 0.5f;
  else if (phase == Phase::Landed) beta = 0.05f;
  const float magnitude = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
  if (std::fabs(magnitude - 9.81f) < 0.2f) {
    const float ax = a[0] / magnitude, ay = a[1] / magnitude, az = a[2] / magnitude;
    const float f1 = 2*q1*q3 - 2*q0*q2 - ax;
    const float f2 = 2*q0*q1 + 2*q2*q3 - ay;
    const float f3 = 1 - 2*q1*q1 - 2*q2*q2 - az;
    float s[4] = {-2*q2*f1 + 2*q1*f2, 2*q3*f1 + 2*q0*f2 - 4*q1*f3,
                  -2*q0*f1 + 2*q3*f2 - 4*q2*f3, 2*q1*f1 + 2*q2*f2};
    const float inverse = 1.0f / std::sqrt(s[0]*s[0] + s[1]*s[1] + s[2]*s[2] + s[3]*s[3] + 1e-9f);
    for (unsigned i = 0; i < 4; ++i) { s[i] *= inverse; derivative[i] -= beta * s[i]; }
  }
  auto next = q_; float norm = 0;
  for (unsigned i = 0; i < 4; ++i) { next[i] += derivative[i] * dt; norm += next[i] * next[i]; }
  if (!std::isfinite(norm) || norm < 1e-12f) return false;
  const float inverse = 1.0f / std::sqrt(norm);
  for (auto &v : next) v *= inverse;
  q_ = next; return true;
}
std::array<float, 3> AttitudeEstimator::verticalInBody() const {
  const float w=q_[0], x=q_[1], y=q_[2], z=q_[3];
  return {{2*(x*z-w*y), 2*(w*x+y*z), w*w-x*x-y*y+z*z}};
}
std::array<float, 3> AttitudeEstimator::eulerRadians() const {
  const float w=q_[0], x=q_[1], y=q_[2], z=q_[3];
  return {{std::atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
           std::asin(std::fmax(-1.0f, std::fmin(1.0f, 2*(w*y-z*x)))),
           std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z))}};
}
}
