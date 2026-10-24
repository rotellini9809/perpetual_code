#pragma once
#include "hardware/SensorSource.h"
#include "navigation/Navigation.h"

namespace navigation {
// Signed sensor axis pointing toward the nose (up when upright on the pad).
enum class UpAxis : uint8_t { PositiveX, NegativeX, PositiveY, NegativeY, PositiveZ, NegativeZ };
struct SensorCalibration {
  bool configured = false; // Arming-session gyro bias ready; no saved accel calibration required.
  // Per-run snapshot. Gyro bias is measured anew at arming and remains RAM-only.
  float gyroBias[3] = {}; // Sensor coordinates, rad/s; never persisted.
  float accelBias[3] = {}; // Sensor coordinates, m/s^2; zero without saved device calibration.
  UpAxis upAxis = UpAxis::PositiveX; // Other body axes follow a fixed right-handed convention.
};
// Shared processing path for hardware, HIL and future autonomous sensor generator.
class SensorPipeline {
public:
  bool begin(sensors::SensorSource &source, const SensorCalibration &calibration,
             float referenceAltitude, float nominalHz);
  bool poll(NavigationOutput &output);
  // Same preprocessing/navigation step when the runtime has already polled the source.
  bool process(const sensors::Reading &reading, NavigationOutput &output);
  void end() { source_ = nullptr; calibration_ = {}; }
  bool active() const { return source_ != nullptr; }
private:
  Navigation navigation_;
  sensors::SensorSource *source_ = nullptr;
  sensors::FrameConvention convention_ = sensors::FrameConvention::SensorAxes;
  SensorCalibration calibration_{};
};
}
