#pragma once
#include "hardware/SensorDevice.h"
#include "hardware/SensorSource.h"
#include "shared/Timing.h"

namespace sensors {
// Polling adapter shared with the flight runtime. Timestamps are register acquisition
// times, not interrupt-edge timestamps; FIFO/overrun qualification remains future work.
class HardwareSensorSource : public SensorSource {
public:
  HardwareSensorSource(SensorDevice &device, timing::Clock clock) : device_(device), clock_(clock) {}
  bool begin(); // Can return false for missing IMU while barometer diagnostics remain usable.
  void suspend() { started_=false; latest_={}; }
  void start() override; // Reset acquisition epoch after arming calibration, without reinitializing chips.
  bool poll(Reading &reading) override; // Emits a flight frame only for a fresh IMU pair.
  FrameConvention convention() const override { return FrameConvention::SensorAxes; }
  const Reading &latest() const { return latest_; } // Barometer also updates without an IMU.
  bool imuReady() const { return device_.imuOk(); }
  bool baroReady() const { return device_.baroOk(); }
private:
  SensorDevice &device_;
  timing::Clock clock_;
  bool started_ = false;
  bool imuInterrupted_ = false;
  uint32_t lastClock_ = 0;
  uint64_t elapsed_ = 0;
  Reading latest_;
};
}
