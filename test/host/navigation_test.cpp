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
  for (unsigned i=0;i<21;++i) detector.update(0.01f, 35, 35, 0, 0, 0);
  assert(detector.phase() == Phase::Boost);
  for (unsigned i=0;i<4;++i) detector.update(0.01f, 1, -1, 0, 1, 10);
  assert(detector.phase() == Phase::Coast);
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
  struct InvalidWindow { const char *name; void (*change)(sensors::Reading &); };
  const InvalidWindow cases[]={
    {"motion", [](sensors::Reading &r) { r.gyroRadps[2]=1; }},
    {"invalid IMU", [](sensors::Reading &r) { r.imuValid=false; }},
    {"nonfinite acceleration", [](sensors::Reading &r) { r.accelMps2[0]=std::numeric_limits<float>::quiet_NaN(); }},
    {"invalid pressure", [](sensors::Reading &r) { r.pressurePa=-1; }},
    {"future pressure", [](sensors::Reading &r) { r.baroTimeUs=r.timeUs+1; ++r.baroSequence; }}
  };
  for (const auto &test:cases) {
    collector.begin(device);
    for (unsigned n=0;n<10;++n) collector.add(reading(n));
    auto input=reading(10); test.change(input);
    const bool accepted=collector.add(input);
    if (accepted || collector.samples()!=0) std::fprintf(stderr,"Arming window not reset: %s\n",test.name);
    assert(!accepted && collector.samples()==0);
  }
  sensors::Reading invalid;
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
  // Gravity initialization uses corrected sensor data and exactly one mounting
  // transform. Check tilted gravity for every supported physical mounting.
  const int axes[6][3]={{1,2,3},{-1,2,-3},{2,-1,3},{-2,1,3},{3,2,-1},{-3,2,1}};
  struct Source : sensors::SensorSource {
    bool poll(sensors::Reading &) override { return false; }
    sensors::FrameConvention convention() const override { return sensors::FrameConvention::SensorAxes; }
  } source;
  for (unsigned mount=0;mount<6;++mount) {
    device.upAxis=navigation::UpAxis(mount);
    device.initialQuaternion={{0,1,0,0}}; // Must not leak from a prior session.
    collector.begin(device);
    assert(collector.calibration().initialQuaternion[0]==1);
    const float sensor[3]={3,4,std::sqrt(9.80665f*9.80665f-25)};
    for (unsigned n=0;n<999;++n) {
      auto r=reading(n);
      for (unsigned i=0;i<3;++i) r.accelMps2[i]=sensor[i]+device.accelBias[i];
      collector.add(r);
    }
    const auto calibrated=collector.calibration(); assert(calibrated.configured);
    navigation::AttitudeEstimator attitude; assert(attitude.reset(calibrated.initialQuaternion));
    const auto vertical=attitude.verticalInBody();
    for (unsigned i=0;i<3;++i) {
      const int axis=axes[mount][i];
      const float expected=(axis<0 ? -1.f:1.f)*sensor[unsigned(std::abs(axis))-1]/9.80665f;
      assert(std::fabs(vertical[i]-expected)<1e-6f);
    }
    navigation::SensorPipeline pipeline;
    assert(pipeline.begin(source,calibrated,collector.referenceAltitude(),200));
    auto r=reading(1000);
    for (unsigned i=0;i<3;++i) r.accelMps2[i]=sensor[i]+device.accelBias[i];
    navigation::NavigationOutput out; assert(pipeline.process(r,out) && out.valid);
    assert(std::fabs(out.verticalSpecificForce-9.80665f)<1e-5f); // Correct from first flight frame.
  }
  for (unsigned axis=0;axis<3;++axis) for (float sign:{-1.f,1.f}) {
    float a[3]={}; a[axis]=sign*9.80665f; std::array<float,4> q;
    assert(navigation::AttitudeEstimator::fromGravity(a,q));
    navigation::AttitudeEstimator attitude; assert(attitude.reset(q));
    const auto up=attitude.verticalInBody();
    for (unsigned i=0;i<3;++i) assert(std::fabs(up[i]-a[i]/9.80665f)<1e-6f);
  }
  float zero[3]={}; std::array<float,4> invalidQ;
  assert(!navigation::AttitudeEstimator::fromGravity(zero,invalidQ));
  zero[0]=std::numeric_limits<float>::quiet_NaN();
  assert(!navigation::AttitudeEstimator::fromGravity(zero,invalidQ));
  device.upAxis=navigation::UpAxis::PositiveX; collector.begin(device);
  for (unsigned n=0;n<999;++n) {
    auto r=reading(n); r.accelMps2[0]=(n<500 ? 9.80665f:-9.80665f)+device.accelBias[0];
    collector.add(r);
  }
  assert(!collector.ready() && collector.samples()==0); // Reject cancelling gravity directions.
  for (unsigned n=1000;n<1999;++n) {
    auto r=reading(n); r.accelMps2[0]=-9.80665f+device.accelBias[0]; collector.add(r);
  }
  assert(collector.ready());
  navigation::AttitudeEstimator fresh; assert(fresh.reset(collector.calibration().initialQuaternion));
  assert(std::fabs(fresh.verticalInBody()[0]+1)<1e-6f); // Reset erased old gravity as well as bias.
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
  device.next.accelMps2[1]=0; device.next.pressurePa=101325;
  device.next.imuValid=false; device.pending=true;
  const auto sequence=source.latest().imuSequence;
  assert(!source.poll(frame));
  device.next.imuValid=true; device.pending=true; nowUs+=8333;
  assert(source.poll(frame) && frame.imuValid && frame.imuSequence==sequence+2);
  puts("Hardware source passed: barometer-only diagnostics, freshness, missing/invalid IMU and timer rollover.");
  // FIFO timing: frames advance by the device period, never later than observed, and a
  // reported FIFO gap breaks sequence continuity and re-anchors on the observation.
  nowUs = 0; assert(source.begin());
  device.next = {}; device.next.imuFresh = device.next.imuValid = true; device.next.imuPeriodUs = 2100;
  device.next.imuAgeUs = 300; nowUs = 10000; device.pending = true;
  assert(source.poll(frame) && frame.timeUs == 9700);
  const auto first = frame.imuSequence;
  // Burst after a stall: two queued frames read back to back at the same MCU time.
  nowUs = 14500; device.next.imuAgeUs = 2600; device.pending = true; // Observed 11900.
  assert(source.poll(frame) && frame.imuSequence == first + 1 && frame.timeUs == 11803); // 11800 + 100/32.
  device.next.imuAgeUs = 500; device.pending = true; // Observed 14000; predicted 13903.
  assert(source.poll(frame) && frame.imuSequence == first + 2 && frame.timeUs == 13906);
  // Observation earlier than prediction clamps to it; time stays monotonic.
  nowUs = 15000; device.next.imuAgeUs = 0; device.pending = true; // Predicted 16006 > observed 15000.
  assert(source.poll(frame) && frame.timeUs == 15000);
  nowUs = 30000; device.next.imuAgeUs = 100; device.next.imuGap = true; device.pending = true;
  assert(source.poll(frame) && frame.imuSequence == first + 5 && frame.timeUs == 29900);
  // Frames are never closer than half a period, even when the observation (or a
  // re-anchor after a gap) would put them closer: distinct milliseconds at 476 Hz.
  nowUs = 30500; device.next.imuAgeUs = 0; device.next.imuGap = false; device.pending = true; // Observed 30500.
  assert(source.poll(frame) && frame.timeUs == 30950); // 29900 + 2100/2.
  nowUs = 31000; device.next.imuAgeUs = 0; device.next.imuGap = true; device.pending = true; // Gap, observed 31000.
  assert(source.poll(frame) && frame.timeUs == 32000 && frame.imuSequence == first + 8);
  puts("Hardware source FIFO timing passed: period spacing, observation bound, gap re-anchor, minimum spacing.");
  // Board case: a barometer read together with a FIFO frame sampled earlier would be "newer"
  // than the frame; it is clamped to the frame, so the arming collector (and navigation, which
  // applies the same barometer <= frame rule) accept it and reach their sample target.
  nowUs = 0; assert(source.begin());
  flight::ArmingCalibration collector; collector.begin(navigation::SensorCalibration{});
  device.next = {}; device.next.imuFresh = device.next.imuValid = device.next.baroValid = true;
  device.next.imuPeriodUs = 2083; device.next.imuAgeUs = 400; device.next.accelMps2[2] = 9.80665f;
  device.next.pressurePa = 101325; device.next.baroTempC = 20;
  unsigned polls = 0;
  for (; polls < 4000 && !collector.ready(); ++polls) {
    nowUs += 2083; device.next.baroFresh = polls % 5 == 0; device.pending = true;
    assert(source.poll(frame) && frame.baroTimeUs <= frame.timeUs);
    collector.add(frame);
  }
  assert(collector.ready() && collector.samples() == flight::ArmingCalibration::TargetSamples);
  std::printf("Barometer clamped to the FIFO frame: arming collector ready after %u frames.\n", polls);
}
} // namespace test_hardware_sensor_source

void test_bistage_detector() {
  using namespace navigation;
  recovery::StateMachineSettings machine; machine.bistage=true; machine.minimumCoastVelocity=20;
  FlightDetector detector;
  auto coast=[&]() { detector.reset(machine); for (unsigned i=0;i<21;++i) detector.update(.01f,40,40,0,0,0,0); assert(detector.phase()==Phase::Boost); for (unsigned i=0;i<4;++i) detector.update(.01f,0,0,0,0,25,0); assert(detector.phase()==Phase::Coast && !detector.startsAllowed()); };
  coast(); detector.confirmCoastVelocity(std::numeric_limits<float>::quiet_NaN());
  const float values[]={-100,24,25,26,27,28,29,1000};
  for (unsigned i=0;i<8;++i) { detector.confirmCoastVelocity(values[i]); assert(detector.startsAllowed()==(i==7)); }
  assert(detector.velocityMedian()==26.5f); detector.confirmCoastVelocity(-1000); assert(detector.phase()==Phase::Coast);
  // BOOST 2 uses the launch rule: a 3 g sample, then 1.5 g held for 200 ms (navigation runs it as BOOST 2 meanwhile).
  for (unsigned i=0;i<20;++i) detector.update(.01f,40,40,100,0,25,0);
  assert(detector.phase()==Phase::Coast && detector.navigationPhase()==Phase::Boost2);
  detector.update(.01f,9.81f,0,0,0,25,0); assert(detector.phase()==Phase::Coast && detector.navigationPhase()==Phase::Coast); // A jolt is rejected.
  detector.update(.01f,4*9.81f,40,100,0,25,0); for (unsigned i=0;i<20;++i) detector.update(.01f,2*9.81f,40,100,0,25,0);
  assert(detector.phase()==Phase::Boost2); // No angular-rate abort.
  for (unsigned i=0;i<4;++i) detector.update(.01f,0,0,0,0,25,0);
  assert(detector.phase()==Phase::Coast2);
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
  for (unsigned i=0;i<21;++i) state=step(40,0,0,10000);
  for (unsigned i=0;i<11;++i) state=step(0,20,0,10000); // COAST after 30 ms, then 8 velocity samples.
  assert(state.phase==Phase::Coast && state.startsAllowed && state.tiltRadians<.001f); // Axial spin alone is allowed.
  state=step(0,0,20,50000);
  assert(state.phase==Phase::Abort && state.abortReason==recovery::AbortReason::Tilt && !state.startsAllowed);

}


void testGyroOnlyAttitude() {
  using namespace navigation; using test_navigation::near;
  Navigation n; assert(!n.startGyroOnly()); // Not begun.
  const float up[3]={9.81f,0,0}; std::array<float,4> q{}; assert(AttitudeEstimator::fromGravity(up,q));
  assert(n.begin(0,200,q));
  uint64_t time=0, sequence=0;
  auto frame=[&](float gyroY) { SensorFrame f; f.timeUs=(time+=5000); f.imuSequence=++sequence; f.imuValid=true;
    f.accel[0]=9.81f; f.gyro[1]=gyroY; return f; };
  for (unsigned i=0;i<10;++i) assert(!n.step(frame(0)).gyroOnly);
  assert(n.startGyroOnly());
  // Rotate 40 deg about BODY Y while the accelerometer still reads upright: only the
  // gyro-only copy follows the whole rotation; the flight attitude is pulled back by gravity.
  const float rate=0.5f; const unsigned steps=unsigned(40*0.017453292f/rate/0.005f+0.5f);
  for (unsigned i=0;i<steps;++i) n.step(frame(rate));
  auto out=n.step(frame(0));
  assert(out.gyroOnly); near(out.gyroOnlyTiltRadians*57.29578f,40,0.5f);
  near(out.gyroOnlyUpBody[0]*out.gyroOnlyUpBody[0]+out.gyroOnlyUpBody[1]*out.gyroOnlyUpBody[1]+out.gyroOnlyUpBody[2]*out.gyroOnlyUpBody[2],1,1e-3f);
  assert(out.tiltRadians*57.29578f<35 && out.phase==Phase::Pad); // Flight logic unchanged.
  for (unsigned i=0;i<200;++i) out=n.step(frame(0)); // Holding still: no correction, no drift.
  near(out.gyroOnlyTiltRadians*57.29578f,40,0.5f);
  assert(n.begin(0,200,q) && !n.step(frame(0)).gyroOnly); // A new run starts without it.
}

void testBoostConfirmation() {
  using namespace navigation;
  FlightDetector d;
  auto high=[&] { d.update(.01f,40,40,0,0,0); };
  auto low=[&] { d.update(.01f,9.81f,9.81f,0,0,0); };
  // BOOST: a sample above 3 g, then above 1.5 g for 200 ms: 21 samples at 100 Hz, restarted by a low or invalid one.
  auto highs=[&](unsigned n) { for (unsigned i=0;i<n;++i) high(); };
  d.reset(); assert(!d.launchPending()); highs(20); assert(d.phase()==Phase::Pad && d.launchPending());
  low(); assert(!d.launchPending()); highs(20); assert(d.phase()==Phase::Pad); high(); assert(d.phase()==Phase::Boost);
  d.reset(); highs(5); d.update(.2f,40,40,0,0,0);
  highs(20); assert(d.phase()==Phase::Pad); high(); assert(d.phase()==Phase::Boost);
  // 2 g alone never starts it; a 3 g peak followed by a 2 g burn does (the Hoppino flight shape).
  d.reset(); for (unsigned i=0;i<100;++i) d.update(.01f,2*9.81f,2*9.81f,0,0,0);
  assert(d.phase()==Phase::Pad && !d.launchPending());
  d.update(.01f,4*9.81f,4*9.81f,0,0,0); for (unsigned i=0;i<19;++i) d.update(.01f,2*9.81f,2*9.81f,0,0,0);
  assert(d.phase()==Phase::Pad); d.update(.01f,2*9.81f,2*9.81f,0,0,0); assert(d.phase()==Phase::Boost);
  // Bench shakes at 476 Hz: 90 ms above 1.5 g after a 6 g peak, then a drop to 1 g, never leave PAD.
  d.reset();
  for (unsigned shake=0;shake<5;++shake) {
    for (unsigned i=0;i<43;++i) d.update(.0021f,i<20 ? 60 : 20,0,0,0,0);
    d.update(.0021f,9.81f,9.81f,0,0,0);
  }
  assert(d.phase()==Phase::Pad && !d.launchPending());
  for (unsigned i=0;i<96;++i) d.update(.0021f,60,60,0,0,0);
  assert(d.phase()==Phase::Pad); d.update(.0021f,60,60,0,0,0); assert(d.phase()==Phase::Boost); // 201.6 ms.
  { // The confirmation delay loses no velocity; a rejected knock leaves it pinned at zero.
    Navigation n; assert(n.begin(0,100,{{1,0,0,0}})); SensorFrame f; f.imuValid=true;
    auto step=[&](float up) { f.timeUs+=10000; ++f.imuSequence; f.accel[2]=up; return n.step(f); }; // Identity attitude: Z is up.
    step(9.80665f); for (unsigned i=0;i<5;++i) step(40);
    auto s=step(9.80665f); assert(s.phase==Phase::Pad && s.verticalVelocity==0);
    for (unsigned i=0;i<20;++i) s=step(40);
    assert(s.phase==Phase::Pad && s.verticalVelocity>5.5f);
    s=step(40); assert(s.phase==Phase::Boost && s.verticalVelocity>5.8f);
  }
  for (unsigned scenario=0;scenario<5;++scenario) {
    Navigation n; assert(n.begin(0,100));
    uint64_t time=0, sequence=0;
    auto frame=[&] { SensorFrame f; f.timeUs=(time+=10000); f.imuSequence=++sequence;
      f.imuValid=true; f.accel[0]=40; return f; };
    n.step(frame()); assert(n.step(frame()).phase==Phase::Pad);
    auto bad=frame(); // Third high sample.
    if (scenario==0) bad.imuValid=false;
    if (scenario==1) bad.accel[0]=std::numeric_limits<float>::quiet_NaN();
    if (scenario==2) { time+=200000; bad.timeUs=time; }
    if (scenario==3) { bad.imuSequence=2; bad.timeUs=20000; }
    if (scenario==4) { bad.imuSequence=++sequence; } // Missing sequence restarts at this good sample.
    n.step(bad);
    for (unsigned i=0;i<19;++i) assert(n.step(frame()).phase==Phase::Pad);
    if (scenario!=4) assert(n.step(frame()).phase==Phase::Pad);
    assert(n.step(frame()).phase==Phase::Boost);
  }
  for (bool secondStage : {false,true}) {
    recovery::StateMachineSettings settings; settings.bistage=secondStage;
    FlightDetector detector; detector.reset(settings);
    auto high=[&] { detector.update(.01f,40,40,0,0,25); };
    auto low=[&] { detector.update(.01f,0,0,0,0,25); };
    for (unsigned i=0;i<21;++i) high();
    if (secondStage) {
      low(); low(); low(); low();
      for (unsigned i=0;i<8;++i) detector.confirmCoastVelocity(25);
      for (unsigned i=0;i<21;++i) high();
    }
    const auto boost=secondStage ? Phase::Boost2 : Phase::Boost;
    const auto coast=secondStage ? Phase::Coast2 : Phase::Coast;
    low(); low(); assert(detector.phase()==boost);
    high(); low(); low(); assert(detector.phase()==boost);
    detector.update(.2f,0,0,0,0,25); // Invalid timing breaks the sequence.
    low(); low(); low(); assert(detector.phase()==boost); // 20 ms since the first low sample.
    low(); assert(detector.phase()==coast);
  }
  puts("BOOST confirmation passed: 3 g then 1.5 g for 200 ms, knock rejected, no velocity lost, low/invalid/gap/duplicate/missing-sequence reset.");
}

int main() {
  testBoostConfirmation();
  testGyroOnlyAttitude();
  test_bistage_detector();
  std::puts("  navigation"); std::fflush(stdout);
  test_navigation::run();
  std::puts("  arming_calibration"); std::fflush(stdout);
  test_arming_calibration::run();
  std::puts("  hardware_sensor_source"); std::fflush(stdout);
  test_hardware_sensor_source::run();
}
