#pragma once
#include <cstdint>
#include <limits>

namespace sensors {
enum class FrameConvention : uint8_t { SensorAxes, LoggedCalibratedBody };
struct Reading {
  uint64_t timeUs = 0, imuSequence = 0;
  bool imuValid = false;
  float accelMps2[3] = {}, gyroRadps[3] = {};
  bool baroValid = false;
  uint64_t baroTimeUs = 0, baroSequence = 0;
  float temperatureC = std::numeric_limits<float>::quiet_NaN(); // Optional barometer temperature.
  float pressurePa = std::numeric_limits<float>::quiet_NaN();
};
// Source implementations publish fresh SI measurements; no navigation or outputs.
// HardwareSensorSource adapts SensorDevice; mounting/bias correction belongs to the pipeline.
class SensorSource {
public:
  virtual ~SensorSource() = default;
  virtual bool poll(Reading &reading) = 0; // false means no new frame.
  virtual FrameConvention convention() const = 0;
  virtual void start() {} // Begin a run's acquisition epoch; HIL already supplies relative time.
  virtual void stop() {}
};
}
