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
void sensorToBody(UpAxis up, const float (&sensor)[3], float (&body)[3]) {
  for (unsigned i=0;i<3;++i) {
    const int axis=MountingAxes[static_cast<unsigned>(up)][i];
    const unsigned j=unsigned(axis<0 ? -axis : axis)-1;
    body[i]=(axis<0 ? -1.0f : 1.0f)*sensor[j];
  }
}
bool SensorPipeline::begin(sensors::SensorSource &source, const SensorCalibration &calibration, float reference, float hz) {
  if (active()) return false;
  if (source.convention() == sensors::FrameConvention::SensorAxes) {
    if (!validCalibration(calibration)) return false;
  } else if (source.convention() != sensors::FrameConvention::LoggedCalibratedBody) return false;
  if (!navigation_.begin(reference, hz, calibration.initialQuaternion)) return false;
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
  float accel[3],gyro[3];
  for (unsigned i = 0; i < 3; ++i) {
    if (convention_ == sensors::FrameConvention::LoggedCalibratedBody) {
      frame.accel[i] = input.accelMps2[i]; frame.gyro[i] = input.gyroRadps[i];
    } else {
      accel[i]=input.accelMps2[i]-calibration_.accelBias[i];
      gyro[i]=input.gyroRadps[i]-calibration_.gyroBias[i];
    }
  }
  if (convention_==sensors::FrameConvention::SensorAxes) {
    sensorToBody(calibration_.upAxis,accel,frame.accel);
    sensorToBody(calibration_.upAxis,gyro,frame.gyro);
  }
  out = navigation_.step(frame); return true;
}
}
