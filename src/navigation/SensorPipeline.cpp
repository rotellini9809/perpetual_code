#include "navigation/SensorPipeline.h"
#include <cmath>

namespace navigation {
namespace {
// BODY X is the nose axis. Signed one-based sensor axes for each BODY component.
// Each mapping is a proper rotation; apply the same mapping to accel and gyro.
constexpr int8_t MountingAxes[6][3] = {
  {1,2,3}, {-1,2,-3}, {2,-1,3}, {-2,1,3}, {3,2,-1}, {-3,2,1}
};
bool validCalibration(const SensorCalibration &c) {
  if (!c.configured || static_cast<unsigned>(c.upAxis) >= 6) return false;
  for (unsigned i = 0; i < 3; ++i)
    if (!std::isfinite(c.gyroBias[i]) || !std::isfinite(c.accelBias[i])) return false;
  return true;
}
}
bool SensorPipeline::begin(sensors::SensorSource &source, const SensorCalibration &calibration, float reference, float hz, const recovery::StateMachineSettings &settings) {
  if (active()) return false;
  if (source.convention() == sensors::FrameConvention::SensorAxes) {
    if (!validCalibration(calibration)) return false;
  } else if (source.convention() != sensors::FrameConvention::LoggedCalibratedBody) return false;
  if (!navigation_.begin(reference,hz,{{1,0,0,0}},settings)) return false;
  calibration_ = calibration; convention_ = source.convention(); source_ = &source; return true;
}
bool SensorPipeline::poll(NavigationOutput &out) {
  sensors::Reading input;
  return source_ && source_->poll(input) && process(input, out);
}
bool SensorPipeline::process(const sensors::Reading &input, NavigationOutput &out) {
  if (!source_) return false;
  if (source_->convention() != convention_) { end(); return false; }
  SensorFrame frame;
  frame.timeUs = input.timeUs; frame.imuSequence = input.imuSequence; frame.imuValid = input.imuValid;
  frame.baroTimeUs = input.baroTimeUs; frame.baroSequence = input.baroSequence;
  frame.baroValid = input.baroValid; frame.pressurePa = input.pressurePa;
  for (unsigned i = 0; i < 3; ++i) {
    if (convention_ == sensors::FrameConvention::LoggedCalibratedBody) {
      frame.accel[i] = input.accelMps2[i]; frame.gyro[i] = input.gyroRadps[i];
    } else {
      const int axis = MountingAxes[static_cast<unsigned>(calibration_.upAxis)][i];
      const unsigned j = unsigned(axis < 0 ? -axis : axis) - 1;
      const float sign = axis < 0 ? -1.0f : 1.0f;
      frame.accel[i] = sign * (input.accelMps2[j] - calibration_.accelBias[j]);
      frame.gyro[i] = sign * (input.gyroRadps[j] - calibration_.gyroBias[j]);
    }
  }
  out = navigation_.step(frame); return true;
}
}
