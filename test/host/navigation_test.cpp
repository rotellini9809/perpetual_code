#include <cstdio>
#include "navigation/Navigation.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "flight/ArmingCalibration.h"
#include "hardware/HardwareSensorSource.h"
#include "../fixtures/FakeSensorDevice.h"

// Each namespace isolates one component scenario and its fault model.
namespace test_navigation {
using namespace navigation;
void near(float a, float b, float tolerance = 1e-4f) { assert(std::fabs(a - b) < tolerance); }
SensorFrame frame(uint64_t time, uint64_t sequence) {
  SensorFrame f; f.timeUs = time; f.imuSequence = sequence; f.imuValid = true;
  f.accel[2] = 9.80665f; f.baroValid = true; f.baroTimeUs = time; f.baroSequence = sequence; f.pressurePa = 101325;
  return f;
}
void run() {
  VerticalKalman filter; filter.reset();
  assert(filter.predict(0.1f, 9.80665f + 10, Phase::Boost));
  near(filter.altitude(), 0.05f); near(filter.velocity(), 1);
  const float previous = filter.altitude();
  assert(!filter.predict(-1, 10, Phase::Boost)); near(filter.altitude(), previous);
  assert(!filter.correctAltitude(std::numeric_limits<float>::quiet_NaN(), Phase::Boost));
  near(filter.barometerVariance(Phase::Pad), 0.0225f);
  filter.reset();
  for (unsigned i = 0; i < 5000; ++i) {
    assert(filter.predict(0.005f, 9.80665f, Phase::Pad));
    filter.constrainPadVelocity(Phase::Pad); assert(filter.correctAltitude(0, Phase::Pad));
    near(filter.altitude(), 0); near(filter.velocity(), 0);
    for (unsigned r = 0; r < 3; ++r) {
      assert(std::isfinite(filter.covariance(r, r)) && filter.covariance(r, r) >= 0);
      for (unsigned c = 0; c < 3; ++c) near(filter.covariance(r, c), filter.covariance(c, r), 1e-5f);
    }
  }
  // Bias is observable under independent altitude measurements in airborne state.
  filter.reset();
  for (unsigned i = 0; i < 20000; ++i) {
    assert(filter.predict(0.005f, 9.80665f + 0.4f, Phase::Coast));
    if (i % 4 == 0) assert(filter.correctAltitude(0, Phase::Coast));
  }
  near(filter.bias(), 0.4f, 0.01f);
  AttitudeEstimator attitude; assert(!attitude.reset({{0,0,0,0}}));
  const float s = std::sqrt(0.5f); assert(attitude.reset({{s,0,-s,0}}));
  auto up = attitude.verticalInBody(); near(up[0], 1); near(up[2], 0);
  float a[] = {9.80665f,0,0}, gyro[] = {0,0,0};
  for (unsigned i = 0; i < 1000; ++i) assert(attitude.step(a, gyro, 0.005f, Phase::Pad));
  float norm = 0; for (float q : attitude.quaternion()) norm += q*q; near(norm, 1);
  FlightDetector detector; detector.reset();
  detector.update(0.01f, 35, 35, 0, 0, 0); assert(detector.phase() == Phase::Boost);
  detector.update(0.01f, 1, -1, 0, 1, 10); assert(detector.phase() == Phase::Coast);
  detector.update(0.01f, 1, -1, 0, 1, -0.5f); assert(detector.phase() == Phase::Descent);
  for (unsigned i = 0; i < 220; ++i) detector.update(0.01f, 9.81f, 9.81f, 0, 0, 0);
  assert(detector.phase() == Phase::Landed);
  near(Navigation::pressureAltitude(101325), 0);
  assert(!std::isfinite(Navigation::pressureAltitude(0)));
  Navigation nav; assert(nav.begin(0, 200));
  auto f = frame(0, 1); auto out = nav.step(f); assert(out.valid && out.barometerUpdated);
  auto snapshot = out.ruleSnapshot(); assert(snapshot.phaseValid && snapshot.altitude.valid);
  auto repeat = nav.step(f); assert(!repeat.valid); near(repeat.altitude, out.altitude);
  f = frame(5000, 2); f.baroSequence = 1;
  out = nav.step(f); assert(out.valid && !out.barometerUpdated);
  f = frame(10000, 3); f.baroTimeUs = 10001; assert(!nav.step(f).barometerUpdated);
  f = frame(15000, 4); f.accel[0] = std::numeric_limits<float>::quiet_NaN();
  assert(!nav.step(f).valid);
  f = frame(20000, 5); assert(nav.step(f).valid);
  f = frame(200000, 6); assert(!nav.step(f).valid); // Do not integrate long unknown gap.
  f = frame(205000, 7); assert(nav.step(f).valid);
  f = frame(400000, 8); nav.step(f);
  f = frame(405000, 9); f.baroTimeUs = 200000; assert(!nav.step(f).barometerUpdated);
  assert(!nav.begin(0, 0)); assert(!nav.step(frame(0,1)).valid);
  std::cout << "Navigation tests passed: Kalman, attitude, phases, freshness and invalid-input freeze.\n";
}
} // namespace test_navigation

namespace test_arming_calibration {
sensors::Reading reading(unsigned n) {
  sensors::Reading r;
  r.timeUs = uint64_t(n) * 5000; r.imuSequence = n + 1; r.imuValid = r.baroValid = true;
  r.accelMps2[0] = 9.80665f + .25f;
  r.gyroRadps[0] = .03125f; r.gyroRadps[1] = -.0625f; r.gyroRadps[2] = .125f;
  r.baroSequence = n / 4 + 1; r.baroTimeUs = uint64_t(n / 4) * 20000;
  r.pressurePa = 100000; return r;
}
void run() {
  flight::ArmingCalibration collector;
  navigation::SensorCalibration device;
  device.accelBias[0] = .25f; device.gyroBias[0] = 999;
  device.upAxis = navigation::UpAxis::NegativeZ;
  collector.begin(device);
  assert(!collector.ready() && !collector.calibration().configured && std::isnan(collector.referenceAltitude()));
  assert(collector.calibration().gyroBias[0] == 0);
  for (unsigned n = 0; n < 999; ++n) {
    const auto r = reading(n);
    assert(collector.add(r) == (n == 998));
    assert(collector.samples() == n / 2 + 1); // Independent of the 200 Hz caller.
    collector.add(r); assert(collector.samples() == n / 2 + 1); // Duplicates never count.
  }
  assert(collector.ready() && collector.pressureSamples() == 250);
  const auto result = collector.calibration();
  assert(result.configured && result.upAxis == device.upAxis && result.accelBias[0] == .25f);
  for (unsigned i = 0; i < 3; ++i) assert(result.gyroBias[i] == reading(0).gyroRadps[i]);
  assert(std::fabs(collector.referenceAltitude() - navigation::Navigation::pressureAltitude(100000)) < 1e-5f);
  collector.clear(); assert(!collector.ready() && collector.samples() == 0 && collector.calibration().gyroBias[0] == 0);
  collector.begin(device);
  for (unsigned n = 0; n < 10; ++n) collector.add(reading(n));
  auto moving = reading(10); moving.gyroRadps[2] = 1;
  assert(!collector.add(moving) && collector.samples() == 0);
  for (unsigned n = 11; n < 20; ++n) collector.add(reading(n));
  auto invalid = reading(20); invalid.imuValid = false;
  assert(!collector.add(invalid) && collector.samples() == 0);
  invalid = reading(21); invalid.accelMps2[0] = std::numeric_limits<float>::quiet_NaN();
  assert(!collector.add(invalid) && collector.samples() == 0);
  invalid = reading(22); invalid.pressurePa = -1;
  assert(!collector.add(invalid) && collector.samples() == 0);
  invalid = reading(23); invalid.baroTimeUs = invalid.timeUs + 1; ++invalid.baroSequence;
  assert(!collector.add(invalid) && collector.samples() == 0);
  // Repeated barometer identity cannot be made fresh by changing its supplied timestamp.
  collector.begin(device); collector.add(reading(0));
  invalid = reading(100); invalid.baroSequence = 1; invalid.baroTimeUs = invalid.timeUs;
  assert(!collector.add(invalid) && collector.samples() == 0 && !collector.ready());
  collector.begin(device); collector.add(reading(0)); collector.add(reading(100));
  assert(collector.samples() == 1 && collector.pressureSamples() == 1); // A gap starts a new stationary window.
  // Every accepted arming session computes new bias, preserving device accel bias only.
  collector.begin(result);
  for (unsigned n = 0; n < 999; ++n) {
    auto r = reading(n); for (auto &gyro : r.gyroRadps) gyro = 0;
    collector.add(r);
  }
  assert(collector.ready()); for (auto gyro : collector.calibration().gyroBias) assert(gyro == 0);
  puts("Arming calibration passed: fresh/spaced samples, independent barometer, stationary reset, invalid data and session-only bias.");
}
} // namespace test_arming_calibration

namespace test_hardware_sensor_source {
uint32_t nowUs = 0;
uint32_t clockUs() { return nowUs; }
void run() {
  FakeSensorDevice device;
  sensors::HardwareSensorSource noClock(device, nullptr);
  assert(!noClock.begin() && device.begins == 0);
  sensors::Reading frame; frame.timeUs = 123;
  sensors::HardwareSensorSource source(device, clockUs);
  assert(!source.poll(frame) && device.reads == 0);
  // Current board condition: failed IMU initialization must not hide healthy baro data.
  device.imu = false; nowUs = 0xFFFFF000u;
  assert(!source.begin() && !source.imuReady() && source.baroReady());
  device.next.baroValid = device.next.baroFresh = true;
  device.next.pressurePa = 101325; device.next.baroTempC = 20; device.pending = true;
  assert(!source.poll(frame) && frame.timeUs == 123);
  assert(source.latest().baroValid && source.latest().baroSequence == 1);
  assert(!source.latest().imuValid && source.latest().imuSequence == 0);
  assert(source.latest().pressurePa == 101325 && source.latest().temperatureC == 20);
  nowUs += 2000;
  assert(!source.poll(frame) && source.latest().baroSequence == 1); // No duplicate acquisition.
  nowUs = 0x1000;
  device.next.pressurePa = 101300; device.pending = true;
  assert(!source.poll(frame) && source.latest().baroSequence == 2);
  assert(source.latest().baroTimeUs == 8192); // Extended timer rollover.
  // Healthy IMU frames repeat the previous barometer identity until fresh pressure arrives.
  device.imu = true; assert(source.begin());
  device.next.imuFresh = device.next.imuValid = true;
  device.next.accelMps2[0] = 9.80665f; device.pending = true;
  assert(source.poll(frame) && frame.imuValid && frame.imuSequence == 1 && frame.baroSequence == 1);
  assert(source.convention() == sensors::FrameConvention::SensorAxes);
  assert(!source.poll(frame) && source.latest().imuSequence == 1);
  nowUs += 8333;
  device.next.baroFresh = false; device.pending = true;
  assert(source.poll(frame) && frame.imuSequence == 2 && frame.baroSequence == 1 && frame.baroTimeUs == 0);
  // Invalid fresh IMU vectors reach navigation with invalidity, never as plausible defaults.
  nowUs += 8333; device.next.accelMps2[1] = std::numeric_limits<float>::quiet_NaN(); device.pending = true;
  assert(source.poll(frame) && !frame.imuValid && frame.imuSequence == 3);
  device.next.baroFresh = true; device.next.pressurePa = -1; device.pending = true;
  source.poll(frame); assert(!source.latest().baroValid && source.latest().baroSequence == 1);
  puts("Hardware source passed: barometer-only diagnostics, freshness, missing/invalid IMU and timer rollover.");
}
} // namespace test_hardware_sensor_source

void test_bistage_detector() {
  using namespace navigation;
  recovery::StateMachineSettings machine; machine.bistage=true; machine.minimumCoastVelocity=20;
  FlightDetector detector;
  auto coast=[&]() { detector.reset(machine); detector.update(.01f,40,40,0,0,0,0); assert(detector.phase()==Phase::Boost); detector.update(.01f,0,0,0,0,25,0); assert(detector.phase()==Phase::Coast && !detector.startsAllowed()); };
  coast(); detector.confirmCoastVelocity(std::numeric_limits<float>::quiet_NaN());
  const float values[]={-100,24,25,26,27,28,29,1000};
  for (unsigned i=0;i<8;++i) { detector.confirmCoastVelocity(values[i]); assert(detector.startsAllowed()==(i==7)); }
  assert(detector.velocityMedian()==26.5f); detector.confirmCoastVelocity(-1000); assert(detector.phase()==Phase::Coast);
  detector.update(.01f,40,40,100,0,25,0); assert(detector.phase()==Phase::Boost2); // No angular-rate abort.
  detector.update(.01f,0,0,0,0,25,0); assert(detector.phase()==Phase::Coast2);
  detector.update(.01f,0,0,0,0,-1,0); assert(detector.phase()==Phase::Descent);
  coast(); for (unsigned i=0;i<8;++i) detector.confirmCoastVelocity(20);
  assert(detector.phase()==Phase::Abort && detector.abortReason()==recovery::AbortReason::Velocity);
  detector.update(.01f,100,100,0,0,100,0); assert(detector.phase()==Phase::Abort && !detector.startsAllowed());
  coast(); detector.update(.01f,0,0,0,0,25,40.0f*.017453292519943295f);
  assert(detector.phase()==Phase::Abort && detector.abortReason()==recovery::AbortReason::Tilt);
  coast(); for (unsigned i=0;i<8;++i) detector.confirmCoastVelocity(25);
  detector.update(.01f,0,0,0,0,-1,0); assert(detector.phase()==Phase::Descent); // No second ignition is not ABORT.
  machine.minimumCoastVelocity=0; Navigation nav; assert(nav.begin(0,100,{{1,0,0,0}},machine));
  SensorFrame frame; frame.imuValid=true;
  auto step=[&](float axial,float spin,float tumble,uint64_t elapsed) {
    frame.timeUs+=elapsed; ++frame.imuSequence; frame.accel[0]=axial; frame.gyro[0]=spin; frame.gyro[2]=tumble;
    return nav.step(frame);
  };
  auto state=step(9.80665f,0,0,10000); assert(state.valid && state.tiltRadians<.001f);
  for (unsigned i=0;i<10;++i) state=step(40,0,0,10000);
  for (unsigned i=0;i<8;++i) state=step(0,20,0,10000);
  assert(state.phase==Phase::Coast && state.startsAllowed && state.tiltRadians<.001f); // Axial spin alone is allowed.
  state=step(0,0,20,50000);
  assert(state.phase==Phase::Abort && state.abortReason==recovery::AbortReason::Tilt && !state.startsAllowed);

}

int main() {
  test_bistage_detector();
  std::puts("  navigation"); std::fflush(stdout);
  test_navigation::run();
  std::puts("  arming_calibration"); std::fflush(stdout);
  test_arming_calibration::run();
  std::puts("  hardware_sensor_source"); std::fflush(stdout);
  test_hardware_sensor_source::run();
}
