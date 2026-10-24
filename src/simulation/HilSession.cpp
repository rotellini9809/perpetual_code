#include "simulation/HilSession.h"
#include <cmath>

namespace simulation {
const char *HilSession::begin(float reference, float hz, const navigation::SensorCalibration &calibration, uint32_t bytes) {
  if (const char *error = controller_.runtime().preparationError()) return error;
  source_.clear();
  if (const char *error = controller_.arm(source_, flight::FlightMode::Simulation, reference, hz, calibration, bytes)) return error;
  baroSequence_ = 0; return nullptr;
}
const char *HilSession::sample(uint32_t timeMs, const float accel[3], const float gyro[3], float pressure,
                               flight::FrameResult &result, float temperature) {
  sensors::Reading input;
  input.timeUs = uint64_t(timeMs) * 1000;
  input.imuSequence = uint64_t(controller_.runtime().status().samples) + 1;
  input.imuValid = true;
  for (unsigned i = 0; i < 3; ++i) {
    input.accelMps2[i] = accel[i]; input.gyroRadps[i] = gyro[i];
    input.imuValid = input.imuValid && std::isfinite(accel[i]) && std::isfinite(gyro[i]);
  }
  input.temperatureC = temperature;
  input.pressurePa = pressure; input.baroValid = std::isfinite(pressure);
  input.baroSequence = baroSequence_ + (input.baroValid ? 1u : 0u); input.baroTimeUs = input.timeUs;
  return process(input, result);
}
const char *HilSession::process(const sensors::Reading &input, flight::FrameResult &result) {
  result = {};
  if (blockedByOtherRun()) return "BUSY";
  if (!active()) return "NOT_REPLAYING";
  if (!source_.submit(input)) { stop(); return "SOURCE_ERROR"; }
  const auto polled = controller_.poll(result);
  if (polled.error) return polled.error;
  if (!polled.processed) { stop(); return "SOURCE_ERROR"; }
  baroSequence_ = input.baroSequence; return nullptr;
}
}
