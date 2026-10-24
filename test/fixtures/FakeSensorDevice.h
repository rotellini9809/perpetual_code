#pragma once
#include "hardware/SensorDevice.h"

// PC-only register acquisition fake. The board always uses the real Sensors driver.
struct FakeSensorDevice : SensorDevice {
  bool imu = true, baro = true, pending = false;
  unsigned reads = 0, begins = 0;
  SensorSample next, current;
  bool begin() override { ++begins; current = {}; return imu && baro; }
  bool read() override {
    ++reads;
    current.imuFresh = current.baroFresh = false;
    if (pending) { current = next; pending = false; }
    return current.imuValid && current.baroValid;
  }
  bool imuOk() const override { return imu; }
  bool baroOk() const override { return baro; }
  const SensorSample &sample() const override { return current; }
};
