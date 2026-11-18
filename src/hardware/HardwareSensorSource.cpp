#include "hardware/HardwareSensorSource.h"
#include <cmath>

namespace sensors {
bool HardwareSensorSource::begin() {
  started_ = false; elapsed_ = 0; latest_ = {};
  if (!clock_) return false;
  const bool ready = device_.begin();
  start();
  return ready;
}
void HardwareSensorSource::start() {
  if (!clock_) return;
  lastClock_ = clock_(); elapsed_ = 0; latest_ = {}; started_ = true; imuInterrupted_=false;
}
bool HardwareSensorSource::poll(Reading &reading) {
  if (!started_) return false;
  device_.read(); // Read both independently; a missing IMU must not suppress the barometer.
  const auto now = clock_(); elapsed_ += uint32_t(now - lastClock_); lastClock_ = now;
  const auto &sample = device_.sample();
  latest_.imuValid = device_.imuOk() && sample.imuValid;
  const bool validPressure = device_.baroOk() && sample.baroValid && std::isfinite(sample.pressurePa) && sample.pressurePa > 0;
  if (sample.baroFresh && validPressure) {
    ++latest_.baroSequence; latest_.baroTimeUs = elapsed_;
    latest_.pressurePa = sample.pressurePa; latest_.temperatureC = sample.baroTempC;
  }
  latest_.baroValid = validPressure && latest_.baroSequence != 0;
  // Barometer-only bring-up is diagnostic, not an IMU substitute or flight frame.
  // In particular, never synthesize zero gyro / 1g acceleration for the bad-solder IMU.
  if (!sample.imuFresh || !latest_.imuValid) {
    if (!latest_.imuValid) imuInterrupted_=true;
    return false;
  }
  for (unsigned i = 0; i < 3; ++i) {
    latest_.accelMps2[i] = sample.accelMps2[i]; latest_.gyroRadps[i] = sample.gyroRadps[i];
    latest_.imuValid = latest_.imuValid && std::isfinite(sample.accelMps2[i]) && std::isfinite(sample.gyroRadps[i]);
  }
  // A skipped invalid acquisition interrupts consecutive-frame confirmation.
  if (imuInterrupted_ && latest_.imuSequence) ++latest_.imuSequence;
  imuInterrupted_=false;
  latest_.timeUs = elapsed_; ++latest_.imuSequence;
  reading = latest_; return true;
}
}
