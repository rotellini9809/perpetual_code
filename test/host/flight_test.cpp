#include <cstdio>
#include "flight/RunController.h"
#include "communication/RunReply.h"
#include "../fixtures/ReplayFixture.h"
#include <cassert>
#include <string>
#include "flight/FlightRuntime.h"
#include "hardware/HardwareSensorSource.h"
#include "simulation/HilSession.h"
#include "../fixtures/FakeSensorDevice.h"
#include <cmath>
#include "../fixtures/CommandFixture.h"
#include "communication/CommandRouter.h"
#include "simulation/HilSensorSource.h"
#include "storage/QueuedRecorder.h"
#include <algorithm>
#include "../fixtures/FakeOutputs.h"

// Each namespace isolates one component scenario and its fault model.
namespace test_run_controller {
void run() {
  using namespace flight;
  using namespace replay_test;
  MemoryFlash flash; storage::ConfigurationStore store(flash);
  assert(store.save(fixture(), limits, true));
  ConfigurationService config(store, limits); config.boot(true); assert(!config.setOrientation(0));
  RunController run;
  assert(!config.beginDraft()); assert(config.hasDraft());
  assert(strcmp(run.begin(RunMode::Estimates, config), "DRAFT_ACTIVE") == 0);
  config.discardDraft(); config.setDisarmed(false);
  assert(strcmp(run.begin(RunMode::Estimates, config), "ARMED") == 0);
  assert(!run.active() && !config.runLocked()); config.setDisarmed(true);
  const auto mutations = flash.mutations;
  assert(!run.begin(RunMode::Estimates, config));
  assert(config.runLocked() && !config.hasDraft());
  assert(strcmp(config.beginDraft(), "BUSY") == 0);
  assert(strcmp(config.apply(), "BUSY") == 0);
  assert(strcmp(config.discardDraft(), "BUSY") == 0);
  assert(strcmp(run.begin(RunMode::Sensors, config), "BUSY") == 0);
  assert(run.stop(RunMode::Sensors) == 0 && run.active());
  recovery::FlightSnapshot s; s.phaseValid = true;
  s.phase = recovery::FlightPhase::Boost; s.phaseMeasuredMs = 0; s.sequence = 1;
  assert(run.evaluate(RunMode::Estimates, 0, s).accepted);
  assert(!run.validTime(RunMode::Estimates, 0));
  s.phase = recovery::FlightPhase::Descent; s.phaseMeasuredMs = 100; s.sequence = 2;
  const auto result = run.evaluate(RunMode::Estimates, 100, s);
  assert(result.accepted && result.startCount == 2);
  assert(result.starts[0].timeMs == 100 && result.starts[1].timeMs == 100);
  protocol::Json events; protocol::appendRunEvents(events, result);
  assert(strstr(events.bytes, "\"triggered\":[10]"));
  protocol::Json again; protocol::appendRunEvents(again, {});
  assert(strstr(again.bytes, "\"triggered\":[]"));
  assert(run.stop(RunMode::Estimates) == 0); // Both outputs already started with the rule.
  assert(!config.runLocked() && run.progress(RunMode::Estimates).samples == 2);
  assert(!run.begin(RunMode::Sensors, config));
  assert(run.progress(RunMode::Sensors).samples == 0);
  assert(run.progress(RunMode::Estimates).samples == 2);
  run.disconnect(); assert(!run.active() && !config.runLocked());
  assert(flash.mutations == mutations);

  uint32_t n; float f;
  assert(protocol::unsignedNumber("4294967295", n) && n == UINT32_MAX);
  assert(!protocol::unsignedNumber("4294967296", n));
  assert(!protocol::unsignedNumber("-1", n));
  assert(protocol::floating("nan", f) && !protocol::finiteNumber("nan", f));
  assert(!protocol::floating("1e999", f));
  protocol::Tokens quoted("add 10 \"escaped \\\"name\\\"\"");
  assert(quoted.valid && quoted.count == 3 && quoted.is(2, "escaped \"name\""));
  assert(!protocol::Tokens("\"unfinished").valid);
  assert(!protocol::Tokens(std::string(129, 'x').c_str()).valid);
  assert(!protocol::Tokens("a b c d e f g h i j k l m").valid);
  protocol::Json json; json.quoted("a\n\"\\");
  assert(strcmp(json.bytes, "\"a\\u000a\\\"\\\\\"") == 0);
  json.add("%s", std::string(800, 'x').c_str()); assert(!json.ok);
  const auto length = json.length; json.add("ignored"); assert(json.length == length);
  puts("Run/controller and protocol tests passed: ownership, settings lock, cancellation, bounds and escaping.");
}
} // namespace test_run_controller

namespace test_flight_runtime {
using namespace replay_test;
uint32_t nowUs = 0;
uint32_t clockUs() { return nowUs; }
unsigned batteryReads=0;
uint16_t readBattery() { ++batteryReads; return 2018; }
struct Capture : storage::RecordingSink {
  bool running = false, failAppend = false;
  storage::FlightEntry entry;
  storage::SensorMode mode = storage::SensorMode::Simulation;
  storage::OutputMode outputs = storage::OutputMode::Physical;
  sensors::FrameConvention frame = sensors::FrameConvention::LoggedCalibratedBody;
  storage::FlightCalibration calibrationSnapshot;
  std::vector<storage::FlightSample> samples;
  unsigned starts = 0;
  bool begin(const recovery::Configuration &, const recovery::ValidationLimits &, uint32_t,
             storage::SensorMode m, storage::OutputMode o, sensors::FrameConvention f, const storage::FlightCalibration &calibration) override {
    calibrationSnapshot=calibration; mode = m; outputs = o; frame = f; running = true; entry.sequence = 1; return true;
  }
  bool append(const storage::FlightSample &s) override { if (failAppend) return false; samples.push_back(s); return true; }
  bool recordStart(recovery::RuleId, recovery::OutputKind, uint32_t, uint32_t) override { ++starts; return true; }
  bool service() override { return true; }
  bool finish(bool, const storage::RecordingSummary &, storage::RecordingRetention) override { running = false; return true; }
  void abort() override { running = false; }
  bool initialized() const override { return true; }
  bool active() const override { return running; }
  bool samplesFull() const override { return false; }
  uint32_t committedSamples() const override { return uint32_t(samples.size()); }
  const storage::FlightEntry *current() const override { return entry.sequence ? &entry : nullptr; }
  storage::RecorderError error() const override { return failAppend ? storage::RecorderError::Io : storage::RecorderError::None; }
  storage::RecordingFailure failure() const override { return {}; }
};
void run() {
  MemoryFlash flash; storage::ConfigurationStore store(flash);
  auto plan = fixture();
  plan.rules[0].conditions[0].kind = recovery::ConditionKind::Phase;
  plan.rules[0].conditions[0].phases = recovery::BoostMask;
  assert(store.save(plan, limits, true));
  ConfigurationService config(store, limits), referenceConfig(store, limits); config.boot(true); assert(!config.setOrientation(0)); referenceConfig.boot(true);
  flight::RunController run, referenceRun;
  Capture capture;
  flight::FlightRuntime runtime(config, run, nullptr, &capture), other(config, run);
  BatteryMonitor battery;
  assert(battery.millivolts()==BatteryMonitor::Missing);
  assert(BatteryMonitor::fromAdc(0)==0 && BatteryMonitor::fromAdc(4095)==BatteryMonitor::Missing);
  assert(BatteryMonitor::fromAdc(UINT16_MAX)==BatteryMonitor::Missing);
  assert(BatteryMonitor::fromAdc(2048)==8527); // 3.3 V reference and exact 31/6 divider ratio.
  battery.poll(UINT32_MAX-100,readBattery);
  battery.poll(98,readBattery); assert(batteryReads==1); // Wrap-safe: 199 ms, no new ADC conversion.
  battery.poll(99,readBattery); assert(batteryReads==2);
  battery.poll(299,[]{ return uint16_t(4095); }); assert(battery.millivolts()==BatteryMonitor::Missing);
  battery.poll(499,readBattery); assert(batteryReads==3 && battery.millivolts()!=BatteryMonitor::Missing);
  runtime.setBattery(battery);
  simulation::HilSession reference(referenceConfig, referenceRun);
  FakeSensorDevice device; sensors::HardwareSensorSource hardware(device, clockUs);
  assert(hardware.begin());
  navigation::SensorCalibration calibration; calibration.configured = true;
  calibration.upAxis = navigation::UpAxis::NegativeZ;
  calibration.accelBias[0] = .25f; calibration.accelBias[1] = -.5f; calibration.accelBias[2] = .125f;
  calibration.gyroBias[0] = -.125f; calibration.gyroBias[1] = .25f; calibration.gyroBias[2] = .5f;
  assert(!runtime.begin(hardware, 0, 200, calibration, 4096, storage::SensorMode::Hardware));
  assert(capture.mode == storage::SensorMode::Hardware && capture.outputs == storage::OutputMode::Virtual);
  assert(capture.frame == sensors::FrameConvention::SensorAxes);
  assert(capture.calibrationSnapshot.available && capture.calibrationSnapshot.gyroCalibrated);
  assert(capture.calibrationSnapshot.upAxis==uint8_t(navigation::UpAxis::NegativeZ));
  assert(capture.calibrationSnapshot.gyroBias[2]==.5f && capture.calibrationSnapshot.accelBias[1]==-.5f);
  assert(!reference.begin(0, 200)); // Simulation requires no gyro acquisition/calibration stage.
  flight::FrameResult a, b;
  auto polled = runtime.poll(a); assert(!polled.processed && !polled.error && runtime.status().samples == 0);
  assert(!strcmp(other.begin(hardware, 0, 200, calibration), "BUSY"));
  other.stop(); assert(runtime.active() && config.runLocked());
  const auto writes = flash.mutations;
  for (unsigned n = 0; n < 80; ++n) {
    nowUs = n * 5000;
    sensors::Reading expected;
    expected.timeUs = nowUs; expected.imuSequence = n + 1; expected.imuValid = true;
    expected.accelMps2[0] = n < 20 ? 9.75f : n < 60 ? 35.0f : -.25f;
    expected.accelMps2[1] = .25f; expected.accelMps2[2] = -.125f;
    expected.gyroRadps[0] = .03125f; expected.gyroRadps[1] = -.015625f; expected.gyroRadps[2] = .0078125f;
    expected.baroValid = true; expected.baroSequence = n / 4 + 1; expected.baroTimeUs = (n / 4) * 20000;
    expected.pressurePa = 101325; expected.temperatureC = 20;
    device.next.imuValid = device.next.imuFresh = device.next.baroValid = true;
    device.next.baroFresh = n % 4 == 0; device.next.pressurePa = 101325; device.next.baroTempC = 20;
    const int axes[] = {3,2,-1}; // Inverse of mounting -Z: sensor = (bodyZ, bodyY, -bodyX).
    for (unsigned i = 0; i < 3; ++i) {
      const int axis = axes[i]; const unsigned j = unsigned(axis < 0 ? -axis : axis) - 1;
      const float sign = axis < 0 ? -1.f : 1.f;
      device.next.accelMps2[i] = sign * expected.accelMps2[j] + calibration.accelBias[i];
      device.next.gyroRadps[i] = sign * expected.gyroRadps[j] + calibration.gyroBias[i];
    }
    device.pending = true;
    polled = runtime.poll(a); assert(polled.processed && !polled.error);
    assert(!reference.process(expected, b));
    assert(a.timeMs == b.timeMs && a.navigation.valid && b.navigation.valid);
    assert(a.navigation.phase == b.navigation.phase && a.navigation.altitude == b.navigation.altitude);
    assert(a.navigation.verticalVelocity == b.navigation.verticalVelocity && a.navigation.eulerRadians == b.navigation.eulerRadians);
    assert(a.navigation.verticalSpecificForce == b.navigation.verticalSpecificForce);
    assert(a.events.startCount == b.events.startCount && a.events.triggerCount == b.events.triggerCount);
    for (std::size_t i = 0; i < a.events.startCount; ++i)
      assert(a.events.starts[i].rule == b.events.starts[i].rule && a.events.starts[i].timeMs == b.events.starts[i].timeMs);
    assert(!runtime.poll(a).processed && runtime.status().samples == n + 1);
  }
  assert(capture.samples.size() == 80 && capture.starts == 2 && flash.mutations == writes);
  assert(std::all_of(capture.samples.begin(),capture.samples.end(),[&](const storage::FlightSample &s) { return s.batteryMv==BatteryMonitor::fromAdc(2018); }));
  assert(capture.samples[0].values[storage::FlightSample::AccelX] == .125f);
  assert(capture.samples[0].values[storage::FlightSample::PressureHpa] == 1013.25f);
  device.pending = true; // A fresh register read with repeated flight time is rejected before navigation.
  polled = runtime.poll(a);
  assert(!polled.processed && !strcmp(polled.error, "INVALID_TIMESTAMP"));
  assert(runtime.active() && runtime.status().samples == 80);
  // Recording errors cannot stop navigation/rules, regardless of sensor source.
  capture.failAppend = true; nowUs += 5000; device.pending = true;
  assert(runtime.poll(a).processed && a.navigation.valid && runtime.active());
  assert(runtime.status().recordingError == storage::RecorderError::Io);
  reference.stop(); runtime.stop(); assert(!config.runLocked() && !capture.active());
  assert(!runtime.poll(a).processed && !strcmp(runtime.poll(a).error, "NOT_RUNNING"));
  puts("Shared runtime passed: direct hardware source / HIL equivalence, common rules/recording, ownership and storage-fault isolation.");
}
} // namespace test_flight_runtime

namespace test_arming_controller {
using command_test::send;

using namespace replay_test;
uint32_t nowUs = 0;
uint32_t clockUs() { return nowUs; }
struct Nor : storage::FlashDevice {
  std::vector<uint8_t> bytes = std::vector<uint8_t>(storage::FlashLayout::Capacity,255);
  bool fail=false;
  unsigned writes = 0;
  bool read(uint32_t a, void *p, std::size_t n) override {
    if (fail || a > bytes.size() || n > bytes.size()-a) return false;
    memcpy(p,bytes.data()+a,n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    if (fail || a > bytes.size() || n > bytes.size()-a) return false;
    ++writes; const auto *source = static_cast<const uint8_t *>(p);
    for (std::size_t i=0;i<n;++i) { assert((bytes[a+i]&source[i])==source[i]); bytes[a+i]&=source[i]; }
    return true;
  }
  bool eraseSector(uint32_t a) override { if (fail) return false; assert(a%4096==0 && a<=bytes.size()-4096); ++writes; memset(bytes.data()+a,255,4096); return true; }
};
struct Checks : flight::PhysicalChecks {
  flight::PhysicalReadiness readiness;
  unsigned calls = 0;
  flight::PhysicalReadiness inspect() override { ++calls; return readiness; }
  void healthy() { readiness.imuReady=readiness.baroReady=true; }
};

void has(const std::string &s, const char *expected) { if (s.find(expected)==std::string::npos) { puts(s.c_str()); puts(expected); assert(false); } }
void acquire(FakeSensorDevice &device, unsigned n) {
  device.next.imuValid=device.next.imuFresh=device.next.baroValid=true;
  device.next.accelMps2[0]=9.80665f+.25f;
  device.next.gyroRadps[0]=.03125f; device.next.gyroRadps[1]=-.0625f; device.next.gyroRadps[2]=.125f;
  device.next.pressurePa=101325; device.next.baroTempC=20; device.next.baroFresh=n%2==0;
  device.pending=true;
}
void run() {
  MemoryFlash flash; storage::ConfigurationStore store(flash); assert(store.save(fixture(),limits,true));
  ConfigurationService config(store,limits); config.boot(true); assert(!config.setOrientation(0));
  Nor recordingFlash; storage::FlightRecorder recorder(recordingFlash); assert(recorder.initializeBlank());
  flight::RunController run;
  FakeSensorDevice device; device.imu=false;
  sensors::HardwareSensorSource hardware(device,clockUs); assert(!hardware.begin());
  FakeOutputs gpio; gpio.connected=0; flight::OutputExecutor outputs(gpio,clockUs);
  flight::ArmingController physical(config,run,clockUs,&recorder,&outputs);
  Checks checks; checks.readiness.baroReady=true;
  simulation::HilSession hilSession(config,run,clockUs); SensorHilCommands hil(hilSession);
  simulation::SimulationSession sim(config,run,clockUs,&recorder,&outputs); SimulationCommands simulation(sim);
  ConfigurationCommands configuration(config); EstimateReplayCommands estimates(config,run);
  FlightCommands flights(recorder,config,run);
  ArmingCommands arming(physical,hardware,checks,&sim,&hilSession);
  CommandRouter router(configuration,estimates,hil,config,run,&flights,&simulation,&arming);
  struct NoSuspend : PowerHardware {
    const char *suspend() override { assert(false); return nullptr; }
    const char *resume() override { assert(false); return nullptr; }
  } noSuspend;
  PowerManager power(noSuspend); router.setPower(power);
  CommandService usb; usb.setExtension(&router);
  // Factory reset must require an explicit mounting choice, even after rules are restored.
  assert(!config.factoryReset()); assert(store.save(fixture(),limits,true)); config.boot(true);
  assert(!config.orientationReady()); checks.healthy(); gpio.connected=15;
  assert(!physical.prepareServos());
  auto missingOrientation=physical.preflight(flight::FlightMode::Physical,true,&checks);
  assert(missingOrientation.has(flight::ArmBlocker::OrientationNotReady) && !missingOrientation.ready());
  assert(!strcmp(physical.arm(hardware,flight::FlightMode::Physical,0,120,{},4096,&checks),"ORIENTATION_NOT_READY"));
  assert(!physical.active() && !config.runLocked());
  assert(!physical.preflight(flight::FlightMode::Simulation,false,nullptr,storage::OutputMode::Physical).has(flight::ArmBlocker::OrientationNotReady));
  assert(!physical.releaseServos()); assert(!config.setOrientation(0));
  checks.readiness.imuReady=false; gpio.connected=0;
  const auto writes=recordingFlash.writes; auto configWrites=flash.mutations;
  auto p=physical.preflight(flight::FlightMode::Physical,true,&checks);
  assert(p.has(flight::ArmBlocker::ImuUnavailable) && !p.has(flight::ArmBlocker::BarometerUnavailable));
  assert(p.failedPyros==3 && p.unpreparedServos==1 && gpio.checkedMask==3);
  const auto sensorChecks=checks.calls, gpioWrites=gpio.writes;
  has(send(usb,"pyro continuity"),"\"configured_pyro_mask\":3,\"connected_pyro_mask\":0,\"failed_pyro_mask\":3,\"all_connected\":false");
  assert(checks.calls==sensorChecks && gpio.writes==gpioWrites && !gpio.enabled && !gpio.mask);
  gpio.connected=3;
  has(send(usb,"pyro continuity"),"\"all_connected\":true");
  gpio.connected=0; gpio.checkedMask=0;
  assert(physical.preflight(flight::FlightMode::Physical,true,&checks).failedPyros==3 && gpio.checkedMask==3);
  gpio.checkedMask=0; has(send(usb,"pyro continuity 1"),"INVALID_ARGUMENT"); assert(!gpio.checkedMask);
  has(send(usb,"@1 preflight"),"IMU_UNAVAILABLE");
  has(send(usb,"@2 arm"),"\"code\":\"IMU_UNAVAILABLE\"");
  has(send(usb,"@2 arm"),"\"code\":\"IMU_UNAVAILABLE\""); // ID retry performs no new checks.
  assert(physical.state()==flight::ArmingState::Disarmed && !config.runLocked());
  assert(recordingFlash.writes==writes && flash.mutations==configWrites && recorder.directory().retainedCount()==0);
  const auto checked=checks.calls;
  has(send(usb,"preflight simulation"),"\"ready\":true"); assert(checks.calls==checked);
  has(send(usb,"arm simulation"),"INVALID_ARGUMENT");
  has(send(usb,"sim begin"),"\"arming_state\":\"armed\"");
  assert(sim.controller().armed() && sim.controller().calibrationSamples()==0 && checks.calls==checked);
  gpio.checkedMask=0; has(send(usb,"pyro continuity"),"BUSY"); assert(!gpio.checkedMask);
  assert(sim.poll());
  has(send(usb,"arm"),"BUSY");
  router.disconnect(); assert(sim.active());
  has(send(usb,"disarm"),"\"arming_state\":\"disarmed\"");
  assert(!sim.active() && !config.runLocked() && recordingFlash.writes==writes);
  // The production runtime/executor is exercised through fake sensor readiness and GPIO/PWM.
  device.imu=true; checks.healthy(); gpio.connected=15;
  // Positioning uses the physical executor but neither saves nor certifies preparation.
  const auto beforePreview=gpio.writes;
  for (const auto *command : {"servo preview 0 1500","servo preview 1 0","servo preview 1 999",
       "servo preview 1 2001","servo preview 1 70000","servo preview 1 1500 extra","servo release 5"})
    has(send(usb,command),"INVALID_ARGUMENT");
  assert(gpio.writes==beforePreview);
  has(send(usb,"config begin"),"\"draft_active\":true");
  has(send(usb,"servo preview 1 1730"),"\"pwm\":1730");
  outputs.reconcile(config.applied()); // The normal idle loop must keep the preview PWM.
  assert(gpio.positions[0]==1730 && outputs.hasPreview() && !outputs.preparedServos(*config.applied()));
  has(send(usb,"sleep"),"BUSY");
  has(send(usb,"preflight"),"BUSY"); has(send(usb,"sim begin physical"),"BUSY");
  assert(!gpio.enabled && !gpio.mask && recordingFlash.writes==writes && flash.mutations==configWrites);
  has(send(usb,"servo release 1"),"\"pwm\":0");
  assert(!gpio.positions[0] && !outputs.hasPreparation());
  send(usb,"config discard");
  has(send(usb,"servo preview 2 1600"),"\"pwm\":1600"); // New local assignments need no prior saved endpoints.
  router.disconnect(); assert(!gpio.positions[1] && !outputs.hasPreview());
  has(send(usb,"servo arm 1"),"\"prepared_servo_mask\":1");
  has(send(usb,"servo preview 1 1800"),"\"pwm\":1800");
  assert(!outputs.preparedServos(*config.applied())); // Trying Final invalidates the held Initial.
  send(usb,"disarm"); assert(gpio.positions[0] && outputs.hasPreparation());
  has(send(usb,"servo release"),"\"released\":true"); assert(!gpio.positions[0] && !outputs.hasPreparation());
  gpio.failServo=true; has(send(usb,"servo preview 1 1500"),"OUTPUT_ERROR");
  gpio.failServo=false; assert(!outputs.hasPreparation());
  has(send(usb,"servo arm 9"),"INVALID_ARGUMENT");
  has(send(usb,"servo arm 2"),"SERVO_NOT_ASSIGNED");
  has(send(usb,"config begin"),"\"draft_active\":true");
  gpio.checkedMask=0; has(send(usb,"pyro continuity"),"DRAFT_ACTIVE"); assert(!gpio.checkedMask);
  has(send(usb,"servo arm 1"),"DRAFT_ACTIVE"); send(usb,"config discard");
  has(send(usb,"@3 servo arm 1"),"\"prepared_servo_mask\":1");
  auto preparedCommands=gpio.servoCommands.size();
  send(usb,"@3 servo arm 1"); assert(gpio.servoCommands.size()==preparedCommands);
  // Invalid requests and failed physical checks preserve manual preparation.
  has(send(usb,"sim begin garbage"),"INVALID_ARGUMENT"); assert(outputs.hasPreparation());
  gpio.connected=0; has(send(usb,"sim begin physical"),"CONTINUITY_FAULT"); assert(outputs.hasPreparation());
  gpio.connected=3;
  has(send(usb,"sim begin physical"),"\"output_mode\":\"PHYSICAL\"");
  assert(sim.active() && gpio.servoCommands.size()==preparedCommands); // Adopt valid PWM without commanding again.
  send(usb,"sim stop"); assert(!outputs.hasPreparation());
  send(usb,"servo arm 1"); assert(outputs.hasPreparation());
  const auto virtualCommands=gpio.servoCommands.size();
  has(send(usb,"sim begin"),"\"output_mode\":\"VIRTUAL\"");
  assert(sim.active() && !outputs.hasPreparation() && !gpio.positions[0] && !gpio.enabled && !gpio.mask);
  assert(gpio.servoCommands.size()==virtualCommands); // Release only; virtual simulation never moves hardware.
  send(usb,"sim stop"); send(usb,"servo arm 1");
  preparedCommands=gpio.servoCommands.size();
  has(send(usb,"hil begin 0 200 logged_body"),"\"ok\":true");
  assert(outputs.hasPreparation() && gpio.positions[0]); send(usb,"hil stop");
  has(send(usb,"replay begin"),"\"ok\":true");
  assert(outputs.hasPreparation() && gpio.positions[0]); send(usb,"replay stop");
  has(send(usb,"config orientation -X"),"\"verified\":true");
  assert(outputs.hasPreparation() && gpio.positions[0]);
  has(send(usb,"config orientation +X"),"\"verified\":true"); configWrites=flash.mutations;
  checks.readiness.imuReady=false;
  has(send(usb,"arm"),"IMU_UNAVAILABLE"); assert(outputs.hasPreparation());
  router.disconnect(); assert(outputs.hasPreparation()); // A rejected arm/link loss preserves held preparation.
  checks.healthy();
  navigation::SensorCalibration calibration; calibration.accelBias[0]=.25f; calibration.gyroBias[0]=999;
  assert(!physical.prepareServos());
  assert(!physical.arm(hardware,flight::FlightMode::Physical,0,120,calibration,4096,&checks));
  assert(physical.state()==flight::ArmingState::Arming && config.runLocked() && !run.active());
  has(send(usb,"sleep"),"BUSY");
  assert(physical.calibrationSamples()==0 && !recorder.active() && recordingFlash.writes==writes);
  const auto beforeBusyChecks=checks.calls;
  assert(physical.preflight(flight::FlightMode::Physical,true,&checks).has(flight::ArmBlocker::Busy));
  assert(checks.calls==beforeBusyChecks);
  has(send(usb,"config begin"),"BUSY"); has(send(usb,"flights recover"),"BUSY"); has(send(usb,"sim begin"),"BUSY");
  const auto whileArming=gpio.writes;
  has(send(usb,"servo preview 1 1700"),"BUSY"); has(send(usb,"servo release 1"),"BUSY");
  assert(gpio.writes==whileArming);
  router.disconnect(); assert(physical.active() && config.runLocked());
  flight::FrameResult result;
  for (unsigned n=0;n<499;++n) {
    nowUs=n*10000; acquire(device,n);
    const auto tick=physical.poll(result); assert(!tick.error && !tick.processed);
    assert(physical.calibrationSamples()==n+1 && !recorder.active() && recordingFlash.writes==writes);
  }
  nowUs=4990000; acquire(device,499); physical.poll(result);
  assert(physical.armed() && physical.calibrationSamples()==500 && recorder.active());
  const auto &savedCalibration=recorder.metadata()->calibration;
  assert(savedCalibration.available && savedCalibration.gyroCalibrated && savedCalibration.sensorAxes);
  assert(savedCalibration.gyroBias[0]==.03125f && savedCalibration.gyroBias[1]==-.0625f && savedCalibration.gyroBias[2]==.125f);
  assert(savedCalibration.accelBias[0]==.25f && std::fabs(savedCalibration.referenceAltitude)<.001f);
  assert(gpio.servoCommands.size()==preparedCommands); // Live arm adopts PWM without a new position command.
  assert(!config.disarmed());
  assert(recorder.metadata()->sensorMode==storage::SensorMode::Hardware && recorder.metadata()->outputMode==storage::OutputMode::Physical);
  nowUs+=10000; acquire(device,500); const auto tick=physical.poll(result);
  assert(tick.processed && result.navigation.valid && result.timeMs==10); // Flight epoch excludes calibration.
  assert(std::fabs(result.navigation.verticalSpecificForce-9.80665f)<1e-5f);
  assert(std::fabs(result.navigation.eulerRadians[1]+1.57079632679f)<1e-3f); // BODY +X up immediately.
  assert(physical.disarm()==nullptr && physical.calibrationSamples()==0 && !recorder.active() && config.disarmed());
  assert(physical.runtime().status().recordingDiscarded && !physical.runtime().status().boostReached);
  assert(recorder.directory().retainedCount()==0);
  has(send(usb,"ds"),"\"recording_discarded\":true"); // Repeated disarm reports the committed result.
  const auto retained=recorder.directory().retainedCount();
  const auto afterClose=recordingFlash.writes;
  assert(!physical.prepareServos());
  assert(!physical.arm(hardware,flight::FlightMode::Physical,0,120,calibration,4096,&checks));
  acquire(device,0); nowUs+=10000; physical.poll(result);
  has(send(usb,"ds"),"\"arming_state\":\"disarmed\"");
  assert(!config.runLocked() && physical.calibrationSamples()==0 && recordingFlash.writes==afterClose && recorder.directory().retainedCount()==retained);
  // Lost continuity during calibration prevents arming and leaves no retained flight.
  assert(!physical.prepareServos());
  assert(!physical.arm(hardware,flight::FlightMode::Physical,0,120,calibration,4096,&checks));
  for (unsigned n=0;n<500;++n) {
    nowUs+=10000; acquire(device,n);
    if (n==499) gpio.connected=0;
    physical.poll(result);
  }
  assert(!physical.active() && !config.runLocked() && !strcmp(physical.error(),"CONTINUITY_FAULT"));
  assert(recordingFlash.writes==afterClose && recorder.directory().retainedCount()==retained);
  gpio.connected=15; assert(outputs.hasPreparation());
  has(send(usb,"ds"),"\"arming_state\":\"disarmed\""); assert(outputs.hasPreparation());
  has(send(usb,"servo release"),"\"released\":true"); assert(!outputs.hasPreparation());
  // Ordinary disarm uses the shared phase gate; virtual debug stop remains available.
  has(send(usb,"sim begin"),"\"active\":true"); const auto origin=nowUs;
  for (unsigned n=0;n<=sensors::SimulatedSensorSource::durationMs() && sim.navigation().phase!=recovery::FlightPhase::Boost;++n) { nowUs=origin+n*1000; sim.poll(); }
  assert(sim.navigation().phase==recovery::FlightPhase::Boost);
  has(send(usb,"ds"),"DISARM_PROHIBITED_IN_FLIGHT"); assert(sim.active());
  has(send(usb,"sim stop"),"\"active\":false");
  assert(config.disarmed() && flash.mutations==configWrites);
  has(send(usb,"replay begin"),"\"active\":true"); has(send(usb,"disarm"),"BUSY");
  has(send(usb,"replay stop"),"\"active\":false");
  // An unfinished servo setup is persistable, but even healthy hardware cannot arm it.
  auto unsetServo=fixture(); unsetServo.servos[0]={};
  assert(store.save(unsetServo,limits,true)); config.boot(true); assert(!config.setOrientation(0));
  assert(config.configValid() && !config.applied()->servos[0].calibrated);
  checks.healthy(); const auto beforeRejectedArm=recordingFlash.writes;
  p=physical.preflight(flight::FlightMode::Physical,true,&checks);
  assert(p.has(flight::ArmBlocker::ServosNotConfigured) && p.unconfiguredServos==1);
  assert(!p.has(flight::ArmBlocker::ServosNotPrepared)); // Prepared hardware cannot bypass missing endpoints.
  assert(!strcmp(physical.arm(hardware,flight::FlightMode::Physical,0,120,{},4096,&checks),"SERVOS_NOT_CONFIGURED"));
  assert(physical.state()==flight::ArmingState::Disarmed && !config.runLocked());
  assert(physical.calibrationSamples()==0 && recordingFlash.writes==beforeRejectedArm);
  has(send(usb,"preflight"),"SERVOS_NOT_CONFIGURED");
  has(send(usb,"preflight"),"\"unconfigured_servo_mask\":1");
  assert(physical.preflight(flight::FlightMode::Physical,true).has(flight::ArmBlocker::ServosNotConfigured));
  assert(physical.preflight(flight::FlightMode::Simulation,false).ready());
  has(send(usb,"sim begin"),"\"active\":true"); has(send(usb,"sim stop"),"\"active\":false");
  assert(!config.beginDraft()); assert(!config.setServo(1,1100,1900)); assert(!config.apply()); config.boot(true); assert(!config.setOrientation(0));
  assert(!physical.prepareServos());
  assert(config.configValid() && physical.preflight(flight::FlightMode::Physical,true,&checks).ready());
  assert(!physical.arm(hardware,flight::FlightMode::Physical,0,120,calibration,4096,&checks));
  for (unsigned n=0;n<500;++n) {
    nowUs+=10000; acquire(device,n);
    if (n==499) recordingFlash.fail=true;
    physical.poll(result);
  }
  assert(!physical.active() && !config.runLocked() && outputs.hasPreparation() && gpio.mask==0);
  assert(std::strcmp(physical.error(),"None")); // Failed recording preparation never enables an armed run.
  recordingFlash.fail=false;
  assert(!physical.disarm() && outputs.hasPreparation());
  assert(!physical.releaseServos() && !outputs.hasPreparation());
  puts("Arming controller/USB passed: physical rejection, shared immediate simulation, fresh calibration, locking, cancellation and phase restrictions.");
}
} // namespace test_arming_controller

namespace test_flight_retention {
using namespace storage;
using namespace replay_test;
struct Nor : FlashDevice {
  std::vector<uint8_t> bytes=std::vector<uint8_t>(FlashLayout::Capacity,255);
  bool read(uint32_t a,void *p,std::size_t n) override {
    assert(a<=bytes.size() && n<=bytes.size()-a); memcpy(p,bytes.data()+a,n); return true;
  }
  bool program(uint32_t a,const void *p,std::size_t n) override {
    assert(a<=bytes.size() && n<=bytes.size()-a); const auto *in=static_cast<const uint8_t *>(p);
    for (std::size_t i=0;i<n;++i) { assert((bytes[a+i]&in[i])==in[i]); bytes[a+i]&=in[i]; }
    return true;
  }
  bool eraseSector(uint32_t a) override {
    assert(a%4096==0 && a+4096<=bytes.size()); memset(bytes.data()+a,255,4096); return true;
  }
};
void run() {
  MemoryFlash configFlash; ConfigurationStore store(configFlash); assert(store.save(fixture(),limits,true));
  ConfigurationService config(store,limits); config.boot(true); assert(!config.setOrientation(0));
  Nor flash; FlightRecorder writer(flash); assert(writer.initializeBlank());
  QueuedRecorder queued(writer); queued.workerStarted();
  flight::RunController run;
  flight::FlightRuntime runtime(config,run,nullptr,&queued);
  sensors::HilSensorSource source;
  navigation::SensorCalibration calibration; calibration.configured=true;
  const auto protectedBytes=std::vector<uint8_t>(flash.bytes.begin(),flash.bytes.begin()+FlashLayout::DirectoryA);
  auto begin=[&](SensorMode mode) {
    source.clear(); assert(!runtime.begin(source,0,200,calibration,4096,mode));
    assert(!runtime.status().boostReached && !runtime.status().recordingDiscarded);
    return runtime.status().recordingSequence;
  };
  auto sample=[&](unsigned ordinal,float acceleration) {
    sensors::Reading reading; reading.timeUs=uint64_t(ordinal)*5000; reading.imuSequence=ordinal+1;
    reading.imuValid=reading.baroValid=true; reading.accelMps2[0]=acceleration;
    reading.pressurePa=101325; reading.baroSequence=ordinal+1; reading.baroTimeUs=reading.timeUs;
    assert(source.submit(reading)); flight::FrameResult result;
    const auto polled=runtime.poll(result); assert(polled.processed && !polled.error && result.navigation.valid);
    return result;
  };
  auto drain=[&]() { while (queued.busy()) assert(queued.poll()); };
  // PC injection exercises the same runtime in Hardware mode, with virtual outputs.
  const auto pad=begin(SensorMode::Hardware); sample(0,9.80665f); runtime.stop();
  assert(runtime.status().recordingBusy && !runtime.status().recordingDiscarded);
  assert(!strcmp(runtime.preparationError(),"BUSY"));
  drain(); assert(runtime.status().recordingDiscarded && writer.directory().retainedCount()==0);
  RecordingInfo info; assert(!writer.inspect(pad,info) && writer.error()==RecorderError::StaleSelection);
  begin(SensorMode::Hardware); runtime.stop(); drain();
  assert(runtime.status().recordingDiscarded && writer.directory().retainedCount()==0); // No first sample needed.
  // Simulation/HIL deliberately retain diagnostic PAD-only runs.
  const auto simulated=begin(SensorMode::Simulation); sample(0,9.80665f); runtime.stop(); drain();
  assert(!runtime.status().recordingDiscarded && writer.inspect(simulated,info) && info.samples==1);
  // Worker stays paused: BOOST is computed on a sample rejected by a full queue.
  const auto launched=begin(SensorMode::Hardware);
  for (unsigned i=0;i<QueuedRecorder::SampleCapacity;++i) sample(i,9.80665f);
  assert(sample(QueuedRecorder::SampleCapacity,40).navigation.phase==recovery::FlightPhase::Boost);
  assert(runtime.status().boostReached && runtime.status().droppedSamples==1 && runtime.active());
  runtime.stop(); drain();
  assert(!runtime.status().recordingDiscarded && writer.inspect(launched,info));
  assert(info.entry.status==FlightStatus::StoppedFull && info.samples==36 && info.summaryState==SummaryState::Valid);
  SampleBlock block;
  for (unsigned i=0;i<4;++i) {
    assert(writer.readBlock(launched,i,block));
    for (unsigned j=0;j<block.count;++j) assert(block.samples[j].phase==recovery::FlightPhase::Pad);
  }
  // Full PAD-only logs are discarded too; fullness is not evidence of launch.
  const auto fullPad=begin(SensorMode::Hardware);
  for (unsigned i=0;i<64;++i) sample(i,9.80665f);
  while (queued.pendingSamples()) assert(queued.poll());
  assert(runtime.status().samplesFull && !runtime.status().boostReached);
  runtime.stop(); drain(); assert(runtime.status().recordingDiscarded);
  assert(!writer.inspect(fullPad,info) && writer.error()==RecorderError::StaleSelection);
  // Abrupt cancellation preserves an uncertain PAD log for recovery.
  const auto interrupted=begin(SensorMode::Hardware); sample(0,9.80665f);
  run.disconnect(); runtime.stop(); drain(); assert(!runtime.status().recordingDiscarded);
  assert(writer.initialize() && writer.inspect(interrupted,info) && info.entry.status==FlightStatus::Interrupted);
  assert(writer.directory().retainedCount()==3);
  assert(std::equal(protectedBytes.begin(),protectedBytes.end(),flash.bytes.begin()));
  puts("Flight retention passed: asynchronous PAD discard, diagnostic retention, unsaved BOOST, full logs and interrupted-log preservation.");
}
} // namespace test_flight_retention

namespace test_live_outputs {
using namespace test_arming_controller;
void run() {
  nowUs=0;
  auto plan=fixture(); plan.rules[0].delayMs=23500; plan.rules[1].delayMs=60000;
  MemoryFlash configFlash; storage::ConfigurationStore store(configFlash); assert(store.save(plan,limits,true));
  ConfigurationService config(store,limits); config.boot(true); assert(!config.setOrientation(0));
  Nor flash; storage::FlightRecorder writer(flash); assert(writer.initializeBlank());
  storage::QueuedRecorder queued(writer); queued.workerStarted();
  FakeOutputs gpio; flight::OutputExecutor outputs(gpio,clockUs);
  flight::RunController run;
  FakeSensorDevice device; sensors::HardwareSensorSource hardware(device,clockUs); assert(hardware.begin());
  Checks checks; checks.healthy();
  flight::ArmingController live(config,run,clockUs,&queued,&outputs);
  assert(live.preflight(flight::FlightMode::Physical,true,&checks).has(flight::ArmBlocker::ServosNotPrepared));
  assert(!live.prepareServos());
  navigation::SensorCalibration calibration; calibration.accelBias[0]=.25f;
  // The caller's virtual flag cannot turn normal physical flight into a virtual run.
  assert(!live.arm(hardware,flight::FlightMode::Physical,0,200,calibration,1048576,&checks,storage::OutputMode::Virtual));
  flight::FrameResult result;
  for (unsigned n=0;n<500;++n) { nowUs+=10000; acquire(device,n); live.poll(result); }
  assert(live.armed() && live.runtime().status().outputMode==storage::OutputMode::Physical);
  assert(writer.metadata()->sensorMode==storage::SensorMode::Hardware && writer.metadata()->outputMode==storage::OutputMode::Physical);
  const auto commands=gpio.servoCommands.size(); assert(commands==1); // Only explicit preparation commanded Initial.
  const auto origin=nowUs;
  uint32_t landed=0; bool windowWithPulse=false, deployed=false;
  // A controlled vertical trajectory places the final pulse at landing + 15 s.
  // This tail-shutdown fixture is independent of the embedded Hoppino replay.
  const auto altitude=[](float time) {
    if (time<5) return 0.0f;
    if (time<7) return 15*(time-5)*(time-5);
    return std::fmax(0.0f,60+60*(time-7)-.5f*9.80665f*(time-7)*(time-7));
  };
  for (uint32_t t=0;t<=40000000 && live.active();t+=5000) {
    nowUs=origin+t;
    acquire(device,t/5000);
    if (t<=25000000) {
      const float time=t*1e-6f;
      const float force=time>=5 && time<7 ? 39.80665f : time>=7 && altitude(time)>0 ? 0 : 9.80665f;
      device.next.accelMps2[0]=force+.25f;
      device.next.pressurePa=101325.0f*std::pow(1-altitude(float(t-t%20000)*1e-6f)/44330.0f,1.0f/.1903f);
      device.next.baroFresh=t%20000==0;
    }
    if (t>=37000000) device.pending=false; // Deadline/pulse drain must work without any more IMU frames.
    const auto p=live.poll(result); assert(!p.error);
    if (!landed && live.navigation().phase==recovery::FlightPhase::Landed) landed=t;
    if (gpio.positions[0]==plan.servos[0].finalUs) deployed=true;
    if (landed && t>=landed+15000000 && gpio.mask==1) {
      assert(live.armed() && run.active()); windowWithPulse=true;
    }
    if (live.state()==flight::ArmingState::Closing) {
      assert(queued.busy() && !gpio.mask && !gpio.positions[0] && !gpio.enabled);
      break; // Output shutdown precedes the worker's finalization acknowledgement.
    }
    while (queued.poll()) {}
  }
  assert(landed && windowWithPulse && deployed && gpio.servoCommands.size()==commands+1);
  assert(!live.armed() && live.state()==flight::ArmingState::Closing && !outputs.owner());
  while (queued.poll()) {}
  assert(live.state()==flight::ArmingState::Disarmed && config.disarmed() && !config.runLocked());
  assert(writer.directory().retainedCount()==1 && writer.current()->status==storage::FlightStatus::Completed);
  assert(live.runtime().status().savedSamples>7000 && live.runtime().status().droppedSamples==0);
  assert(!gpio.mask && !gpio.positions[0] && !outputs.hasPreparation());
  unsigned starts=0;
  for (const auto &slot:writer.metadata()->starts) {
    if (slot.state==storage::StartState::Recorded) { assert(slot.rule==10); ++starts; }
  }
  assert(starts==2); // The later rule was cancelled at the landing deadline.
  puts("Live outputs passed: explicit preparation, real-source calibration/runtime, physical starts, pulse drain and asynchronous landing close.");
}
}

#include "airbrakes_cases.h"
int main() {
  test_airbrakes::run();
  std::puts("  run_controller"); std::fflush(stdout);
  test_run_controller::run();
  std::puts("  flight_runtime"); std::fflush(stdout);
  test_flight_runtime::run();
  std::puts("  arming_controller"); std::fflush(stdout);
  test_arming_controller::run();
  std::puts("  flight_retention"); std::fflush(stdout);
  test_flight_retention::run();
  std::puts("  live_outputs"); std::fflush(stdout);
  test_live_outputs::run();
}
