#include "navigation/Navigation.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <stdexcept>
#include <cmath>

// Host-only stdin adapter: reference altitude + nominal Hz, then rows of time (us),
// logged BODY IMU and pressure (Pa; nan = no fresh reading). The Python CSV boundary
// converts hPa to Pa and dataset milliseconds to microseconds.
int main() {
  try {
    float reference, hz;
    if (!(std::cin >> reference >> hz)) throw std::runtime_error("Missing replay calibration metadata");
    std::string line; std::getline(std::cin, line);
    navigation::Navigation nav;
    if (!nav.begin(reference, hz)) throw std::runtime_error("Invalid initialization");
    uint64_t sequence = 0, baroSequence = 0;
    std::cout << "t_ms,z_est,vz_est,ba_est,az_est,f_z_nav,roll_rad,pitch_rad,yaw_rad,phase,imu_valid,baro_updated,baro_altitude,pad_baseline,t_us\n";
    std::cout << std::setprecision(9);
    while (std::getline(std::cin, line)) {
      if (line.empty()) continue;
      std::istringstream row(line); uint64_t stamp; std::string values[7], extra;
      if (!(row >> stamp)) throw std::runtime_error("Missing timestamp");
      for (auto &value : values) if (!(row >> value)) throw std::runtime_error("Missing sensor field");
      if (row >> extra) throw std::runtime_error("Extra sensor field");
      navigation::SensorFrame f; f.timeUs = stamp; f.imuSequence = ++sequence; f.imuValid = true;
      for (unsigned i = 0; i < 3; ++i) { f.accel[i] = std::stof(values[i]); f.gyro[i] = std::stof(values[i+3]); }
      f.pressurePa = std::stof(values[6]); f.baroValid = std::isfinite(f.pressurePa);
      if (f.baroValid) ++baroSequence;
      f.baroSequence = baroSequence; f.baroTimeUs = f.timeUs;
      const auto out = nav.step(f);
      std::cout << stamp / 1000 << ',' << out.altitude << ',' << out.verticalVelocity << ',' << out.accelerationBias << ','
                << out.verticalAcceleration << ',' << out.verticalSpecificForce << ',' << out.eulerRadians[0] << ','
                << out.eulerRadians[1] << ',' << out.eulerRadians[2] << ',' << unsigned(out.phase) << ','
                << out.valid << ',' << out.barometerUpdated << ',' << out.barometerAltitude << ',' << out.padBaseline << ',' << stamp << '\n';
    }
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
