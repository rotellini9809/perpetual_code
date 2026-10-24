// Run the actual embedded Hoppino source and shared sensor pipeline on a fake clock.
#include "simulation/SimulatedSensorSource.h"
#include "navigation/SensorPipeline.h"
#include <iomanip>
#include <iostream>

namespace { uint32_t nowUs=0; uint32_t clockUs() { return nowUs; } }
int main() {
  sensors::SimulatedSensorSource source(clockUs); navigation::SensorPipeline pipeline;
  if (!pipeline.begin(source,navigation::SensorCalibration{},source.referenceAltitude(),source.nominalHz())) return 1;
  source.start();
  std::cout << "t_ms,z_est,vz_est,ba_est,az_est,f_z_nav,roll_rad,pitch_rad,yaw_rad,phase,imu_valid,baro_updated,baro_altitude,pad_baseline\n";
  std::cout << std::setprecision(9);
  for (uint32_t ms=0;ms<=source.durationMs();++ms) {
    nowUs=ms*1000; navigation::NavigationOutput out;
    if (!pipeline.poll(out)) continue;
    std::cout << out.timeUs/1000 << ',' << out.altitude << ',' << out.verticalVelocity << ',' << out.accelerationBias << ','
              << out.verticalAcceleration << ',' << out.verticalSpecificForce << ',' << out.eulerRadians[0] << ','
              << out.eulerRadians[1] << ',' << out.eulerRadians[2] << ',' << unsigned(out.phase) << ','
              << out.valid << ',' << out.barometerUpdated << ',' << out.barometerAltitude << ',' << out.padBaseline << '\n';
  }
  return source.finished() && !source.missed() ? 0 : 2;
}
