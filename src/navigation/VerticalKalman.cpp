#include "navigation/VerticalKalman.h"
#include <cmath>
#include <cstring>

namespace navigation {
void VerticalKalman::reset(float altitude) {
  x_[0] = altitude; x_[1] = x_[2] = acceleration_ = 0;
  memset(p_, 0, sizeof(p_)); p_[0][0] = p_[1][1] = 10; p_[2][2] = 1;
}
float VerticalKalman::barometerVariance(Phase phase) const {
  if (phase == Phase::Pad || phase == Phase::Landed) return 0.0225f;
  const float speed = std::fmin(std::fabs(x_[1]) / 120.0f, 1.0f);
  return 0.49f + (6.25f - 0.49f) * speed * speed;
}
bool VerticalKalman::predict(float dt, float force, Phase phase) {
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.1f || !std::isfinite(force)) return false;
  if (phase == Phase::Pad || phase == Phase::Landed) { acceleration_ = 0; x_[1] = 0; }
  else acceleration_ = force - 9.80665f - x_[2];
  const float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
  x_[0] += x_[1] * dt + 0.5f * acceleration_ * dt2;
  x_[1] += acceleration_ * dt;
  const float f[3][3] = {{1, dt, -0.5f * dt2}, {0, 1, -dt}, {0, 0, 1}};
  const float q[3][3] = {{0.25f * 9.0f * dt4, 0.5f * 9.0f * dt3, 0},
                         {0.5f * 9.0f * dt3, 9.0f * dt2, 0}, {0, 0, 1e-4f * dt}};
  float fp[3][3] = {}, next[3][3] = {};
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      for (unsigned k = 0; k < 3; ++k) fp[i][j] += f[i][k] * p_[k][j];
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j) {
      for (unsigned k = 0; k < 3; ++k) next[i][j] += fp[i][k] * f[j][k];
      next[i][j] += q[i][j];
    }
  memcpy(p_, next, sizeof(p_)); return true;
}
bool VerticalKalman::correct(unsigned observed, float value, float variance) {
  if (!std::isfinite(value)) return false;
  const float s = p_[observed][observed] + variance;
  if (!std::isfinite(s) || s <= 0) return false;
  const float innovation = value - x_[observed];
  float k[3], next[3][3] = {};
  for (unsigned i = 0; i < 3; ++i) { k[i] = p_[i][observed] / s; x_[i] += k[i] * innovation; }
  // Same (I-KH)P update as legacy; no unreviewed covariance/noise retuning.
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j) {
      for (unsigned n = 0; n < 3; ++n)
        next[i][j] += ((i == n ? 1.0f : 0.0f) - (n == observed ? k[i] : 0.0f)) * p_[n][j];
    }
  memcpy(p_, next, sizeof(p_)); return true;
}
bool VerticalKalman::correctAltitude(float altitude, Phase phase) {
  return phase != Phase::Landed && correct(0, altitude, barometerVariance(phase));
}
void VerticalKalman::constrainPadVelocity(Phase phase) {
  if (phase == Phase::Pad) correct(1, 0, 0.0025f);
}
}
