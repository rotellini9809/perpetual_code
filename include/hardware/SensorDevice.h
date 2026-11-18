#pragma once
#include <cstdint>
#include <limits>

// Driver boundary: fresh flags describe newly acquired register data, not calls to read().
struct SensorSample {
  bool imuValid = false, baroValid = false;
  bool imuFresh = false, baroFresh = false;
  float accelMps2[3] = {}, gyroRadps[3] = {};
  float imuTempC = std::numeric_limits<float>::quiet_NaN();
  float pressurePa = std::numeric_limits<float>::quiet_NaN();
  float baroTempC = std::numeric_limits<float>::quiet_NaN();
  // Queued (FIFO) IMU timing. Zero period means "acquired now" (no queue/timing model).
  float imuPeriodUs = 0;   // Nominal sample spacing on the IMU's own clock.
  uint32_t imuAgeUs = 0;   // Estimated age of a fresh frame when read() returned.
  bool imuGap = false;     // Samples were lost before this fresh frame (FIFO overrun/mismatch).
};
class SensorDevice {
public:
  virtual ~SensorDevice() = default;
  virtual bool begin() = 0;
  virtual bool read() = 0;
  virtual bool imuOk() const = 0;
  virtual bool baroOk() const = 0;
  virtual const SensorSample &sample() const = 0;
};
