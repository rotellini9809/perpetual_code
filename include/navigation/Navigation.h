#pragma once
#include "navigation/AttitudeEstimator.h"
#include "navigation/FlightDetector.h"
#include "shared/FlightTypes.h"
#include <cstdint>
#include <limits>

namespace navigation {
struct SensorFrame {
  uint64_t timeUs = 0, imuSequence = 0;
  bool imuValid = false;
  float accel[3] = {}; // Calibrated BODY specific force, m/s^2; +X tail -> nose.
  float gyro[3] = {}; // Calibrated BODY angular velocity, rad/s.
  bool baroValid = false;
  uint64_t baroTimeUs = 0, baroSequence = 0;
  float pressurePa = std::numeric_limits<float>::quiet_NaN();
};
struct NavigationOutput {
  bool valid = false, barometerUpdated = false;
  uint64_t timeUs = 0, sequence = 0;
  Phase phase = Phase::Pad;
  float altitude = 0, verticalVelocity = 0, accelerationBias = 0;
  float verticalAcceleration = std::numeric_limits<float>::quiet_NaN();
  float verticalSpecificForce = std::numeric_limits<float>::quiet_NaN();
  float barometerAltitude = std::numeric_limits<float>::quiet_NaN();
  float padBaseline = 0;
  float dtSeconds = 0;
  std::array<std::array<float,3>,3> covariance{};
  bool startsAllowed = true;
  recovery::AbortReason abortReason = recovery::AbortReason::None;
  float coastVelocityMedian = 0, tiltRadians = 0;
  std::array<float, 3> eulerRadians{};
  recovery::FlightSnapshot ruleSnapshot() const;
};
class Navigation {
public:
  // Reference is absolute pressure altitude in metres from arming calibration.
  // Default quaternion matches legacy; no assumption about the new mounting map.
  bool begin(float referenceAltitude, float nominalImuHz = 120,
             const std::array<float, 4> &initialQuaternion = {{1, 0, 0, 0}},
             const recovery::StateMachineSettings &settings = recovery::StateMachineSettings{});
  NavigationOutput step(const SensorFrame &frame);
  static float pressureAltitude(float pressurePa);
  const VerticalKalman &filter() const { return filter_; }
private:
  NavigationOutput output(const SensorFrame &frame) const;
  AttitudeEstimator attitude_;
  VerticalKalman filter_;
  FlightDetector detector_;
  float reference_ = 0, firstDt_ = 0, baseline_ = 0, padGate_ = 0;
  uint64_t lastTime_ = 0, lastImuSequence_ = 0, lastBaroSequence_ = 0;
  bool attitudeAligned_ = false;
  bool initialized_ = false, haveImu_ = false, haveBaro_ = false;
};
}
