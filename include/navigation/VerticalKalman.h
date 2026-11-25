#pragma once
#include "shared/FlightTypes.h"

namespace navigation {
using Phase = recovery::FlightPhase;
// Legacy 3-state model: launch-relative z [m], vz [m/s], vertical accel bias [m/s^2].
class VerticalKalman {
public:
  void reset(float altitude = 0);
  bool predict(float dt, float verticalSpecificForce, Phase phase);
  bool correctAltitude(float altitude, Phase phase);
  void constrainPadVelocity(Phase phase);
  float altitude() const { return x_[0]; }
  float velocity() const { return x_[1]; }
  float bias() const { return x_[2]; }
  float acceleration() const { return acceleration_; }
  float barometerVariance(Phase phase) const;
  float covariance(unsigned row, unsigned col) const { return row < 3 && col < 3 ? p_[row][col] : 0; }
private:
  bool correct(unsigned observed, float value, float variance);
  float x_[3] = {}; // State: altitude [m], vertical velocity [m/s], acceleration bias [m/s^2].
  float p_[3][3] = {}; // State covariance.
  float acceleration_ = 0;
};
}
