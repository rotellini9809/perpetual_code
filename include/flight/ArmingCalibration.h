#pragma once
#include "navigation/SensorPipeline.h"

namespace flight {
// RAM-only physical arming collector. Simulation never invokes this class.
class ArmingCalibration {
public:
  static constexpr uint32_t TargetSamples = 500;
  static constexpr uint64_t SampleSpacingUs = 10000;
  void begin(const navigation::SensorCalibration &device);
  bool add(const sensors::Reading &reading);
  void clear() { *this = ArmingCalibration{}; }
  uint32_t samples() const { return samples_; }
  uint32_t pressureSamples() const { return pressures_; }
  bool ready() const { return samples_ == TargetSamples && pressures_ != 0; }
  navigation::SensorCalibration calibration() const;
  float referenceAltitude() const;
private:
  void resetWindow();
  navigation::SensorCalibration device_;
  uint32_t samples_ = 0, pressures_ = 0;
  uint64_t lastTime_ = 0, lastSequence_ = 0, acceptedTime_ = 0, lastBaroSequence_ = 0, lastBaroTime_ = 0;
  bool haveImu_ = false, haveBaro_ = false;
  double gyroSum_[3] = {}, accelSum_[3] = {}, altitudeSum_ = 0;
};
}
