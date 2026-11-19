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
  lastClock_ = clock_(); elapsed_ = 0; latest_ = {}; started_ = true; imuInterrupted_=false; imuTimed_=false;
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
  // Barometer-only data is diagnostic, not an IMU substitute or flight frame.
  // In particular, never synthesize zero gyro / 1g acceleration for a missing IMU.
  if (!sample.imuFresh || !latest_.imuValid) {
    if (!latest_.imuValid) imuInterrupted_=true;
    return false;
  }
  for (unsigned i = 0; i < 3; ++i) {
    latest_.accelMps2[i] = sample.accelMps2[i]; latest_.gyroRadps[i] = sample.gyroRadps[i];
    latest_.imuValid = latest_.imuValid && std::isfinite(sample.accelMps2[i]) && std::isfinite(sample.gyroRadps[i]);
  }
  // A skipped invalid acquisition or lost FIFO data interrupts consecutive-frame confirmation.
  if (sample.imuGap) imuInterrupted_=true;
  if (imuInterrupted_ && latest_.imuSequence) ++latest_.imuSequence;
  const double observed = elapsed_ > sample.imuAgeUs ? double(elapsed_ - sample.imuAgeUs) : 0.0;
  if (sample.imuPeriodUs <= 0) imuTimeUs_ = double(elapsed_);
  else {
    double next = observed; // First frame, or re-anchor after lost samples.
    if (imuTimed_ && !imuInterrupted_) {
      const double predicted = imuTimeUs_ + sample.imuPeriodUs;
      next = predicted + (observed - predicted) / 32;
      if (next > observed) next = observed; // A frame cannot be later than its observation.
    }
    // Consecutive FIFO frames are one IMU period apart; never emit them closer than
    // half of it. At 476 Hz that keeps every frame in a distinct millisecond, which
    // the run controller requires (a repeated ms would skip navigation entirely).
    if (imuTimed_ && next < imuTimeUs_ + sample.imuPeriodUs / 2) next = imuTimeUs_ + sample.imuPeriodUs / 2;
    imuTimeUs_ = next;
  }
  imuTimed_ = sample.imuPeriodUs > 0;
  imuInterrupted_=false;
  latest_.timeUs = uint64_t(imuTimeUs_); ++latest_.imuSequence;
  // The barometer is stamped when read, the FIFO frame when sampled (never later than its
  // observation), so a fresh pressure is usually a few ms "newer" than the frame it travels with.
  // Consumers require barometer time <= frame time and would drop it: clamp it to the frame.
  if (latest_.baroTimeUs > latest_.timeUs) latest_.baroTimeUs = latest_.timeUs;
  reading = latest_; return true;
}
}
