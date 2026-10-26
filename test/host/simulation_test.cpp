#include <cstdio>
#include "simulation/HilSession.h"
#include "../fixtures/ReplayFixture.h"
#include <cmath>
#include <limits>
#include "../fixtures/CommandFixture.h"
#include "communication/CommandRouter.h"
#include <iostream>
#include <string>
#include "configuration/ActuatorPolicy.h"
#include "../fixtures/FakeOutputs.h"

// Each namespace isolates one component scenario and its fault model.
namespace test_hil_session {
using namespace replay_test;
uint32_t tick() { static uint32_t time = 0xFFFFFFF0u; time += 20; return time; }
uint32_t slowTick() { static uint32_t time = 0xFFFFF000u; time += 3000; return time; }
void run() {
  MemoryFlash flash; storage::ConfigurationStore store(flash); assert(store.save(fixture(), limits, true));
  ConfigurationService config(store, limits); config.boot(true);
  flight::RunController run;
  simulation::HilSession session(config, run, tick), second(config, run);
  flight::FrameResult result;
  const float accel[] = {0,0,9.80665f}, gyro[] = {0,0,0};
  auto errorIs = [](const char *actual, const char *expected) { assert(actual && strcmp(actual, expected) == 0); };
  errorIs(session.sample(0, accel, gyro, 101325, result), "NOT_REPLAYING");
  config.beginDraft(); errorIs(session.begin(0, 200), "DRAFT_ACTIVE"); config.discardDraft();
  errorIs(session.begin(0, 0), "INVALID_ARGUMENT");
  errorIs(session.begin(std::numeric_limits<float>::quiet_NaN(), 200), "INVALID_ARGUMENT");
  assert(!session.active() && !config.runLocked());
  const auto writes = flash.mutations;
  assert(!session.begin(0, 200)); assert(config.runLocked());
  errorIs(second.begin(0, 200), "BUSY");
  second.stop(); assert(session.active() && config.runLocked());
  errorIs(second.sample(0, accel, gyro, 101325, result), "BUSY"); assert(session.active());
  assert(!session.sample(0, accel, gyro, 101325, result));
  assert(result.navigation.valid && result.navigation.barometerUpdated && result.computeUs == 20);
  errorIs(session.sample(0, accel, gyro, 101325, result), "INVALID_TIMESTAMP");
  assert(session.status().samples == 1);
  assert(!session.sample(5, accel, gyro, std::numeric_limits<float>::quiet_NaN(), result));
  assert(!result.navigation.barometerUpdated && result.navigation.valid);
  auto invalid = sensors::Reading{}; invalid.timeUs = 10000; invalid.imuValid = false; invalid.imuSequence = 3;
  assert(!session.process(invalid, result)); assert(!result.navigation.valid);
  session.stop(); assert(!config.runLocked());
  assert(session.status().samples == 3 && session.status().maxComputeUs == 20);
  assert(session.timing().compute.count == 3 && session.timing().compute.totalUs == 60);
  assert(session.timing().step.maxUs == 40 && session.timing().storage.count == 0 && session.timing().overBudget == 0);
  assert(!session.begin(0, 200)); assert(session.status().samples == 0 && session.status().maxComputeUs == 0);
  assert(session.timing().step.count == 0 && session.timing().nominalPeriodUs == 5000);
  assert(!session.sample(0, accel, gyro, 101325, result)); assert(result.navigation.valid);
  const auto first = result.navigation; session.stop();
  // Use the session without USB in raw sensor coordinates, with mounting calibration.
  simulation::HilSession raw(config, run, nullptr, sensors::FrameConvention::SensorAxes);
  errorIs(raw.begin(0, 200), "INITIALIZATION_FAILED"); assert(!config.runLocked());
  navigation::SensorCalibration calibration; calibration.configured = true; calibration.gyroBias[1] = 0.1f;
  calibration.upAxis = navigation::UpAxis::NegativeZ;
  assert(!raw.begin(0, 200, calibration));
  const float rawAccel[] = {9.80665f,0,0}, rawGyro[] = {0,0.1f,0};
  assert(!raw.sample(0, rawAccel, rawGyro, 101325, result));
  assert(result.navigation.valid && result.navigation.altitude == first.altitude);
  assert(result.navigation.verticalSpecificForce == first.verticalSpecificForce);
  assert(result.navigation.eulerRadians == first.eulerRadians);
  assert(!raw.status().timingValid && result.computeUs == 0);
  raw.stop();
  // Device accel bias defaults to zero above. Nonzero sensor-axis offsets must
  // be removed before mounting rotation, without subtracting gravity itself.
  calibration.accelBias[0] = 0.25f; calibration.accelBias[1] = -0.5f; calibration.accelBias[2] = 0.125f;
  calibration.gyroBias[0] = 0.125f; calibration.gyroBias[1] = -0.25f; calibration.gyroBias[2] = 0.5f;
  const float biasedAccel[] = {9.80665f + 0.25f, -0.5f, 0.125f};
  const float biasedGyro[] = {0.125f, -0.25f, 0.5f};
  assert(!raw.begin(0, 200, calibration));
  // A run freezes its snapshot; changes to the caller cannot alter a live run.
  calibration.accelBias[0] = 100; calibration.gyroBias[0] = 100;
  assert(!raw.sample(0, biasedAccel, biasedGyro, 101325, result));
  assert(result.navigation.valid && result.navigation.altitude == first.altitude);
  assert(std::fabs(result.navigation.verticalSpecificForce - first.verticalSpecificForce) < 1e-5f);
  assert(result.navigation.eulerRadians == first.eulerRadians);
  raw.stop();
  // New sessions use newly supplied gyro bias; they never inherit the last run.
  navigation::SensorCalibration next; next.configured = true;
  next.upAxis = navigation::UpAxis::NegativeZ;
  assert(!raw.begin(0, 200, next));
  assert(!raw.sample(0, rawAccel, gyro, 101325, result));
  assert(result.navigation.valid && result.navigation.eulerRadians == first.eulerRadians);
  assert(result.navigation.verticalSpecificForce == first.verticalSpecificForce);
  raw.stop();
  for (unsigned axis = 0; axis < 3; ++axis) {
    next.accelBias[axis] = std::numeric_limits<float>::quiet_NaN();
    errorIs(raw.begin(0, 200, next), "INITIALIZATION_FAILED"); assert(!config.runLocked());
    next.accelBias[axis] = std::numeric_limits<float>::infinity();
    errorIs(raw.begin(0, 200, next), "INITIALIZATION_FAILED"); assert(!config.runLocked());
    next.accelBias[axis] = 0;
  }
  // Logged BODY measurements already contain corrections and bypass both biases.
  navigation::SensorCalibration ignored;
  ignored.accelBias[0] = ignored.gyroBias[0] = std::numeric_limits<float>::quiet_NaN();
  assert(!session.begin(0, 200, ignored));
  assert(!session.sample(0, accel, gyro, 101325, result));
  assert(result.navigation.valid && result.navigation.eulerRadians == first.eulerRadians);
  assert(result.navigation.verticalSpecificForce == first.verticalSpecificForce);
  session.stop();
  assert(!run.begin(flight::RunMode::Estimates, config));
  session.stop(); assert(run.active(flight::RunMode::Estimates));
  errorIs(session.begin(0, 200), "BUSY"); run.disconnect();
  assert(!session.begin(0, 200));
  run.disconnect(); // External cancellation leaves an old session object alive.
  assert(!second.begin(0, 200));
  assert(!session.active() && second.active());
  session.stop(); assert(second.active() && config.runLocked());
  second.stop();
  simulation::HilSession slow(config, run, slowTick);
  assert(!slow.begin(0, 200) && !slow.sample(0, accel, gyro, 101325, result));
  assert(slow.timing().step.maxUs == 6000 && slow.timing().compute.maxUs == 3000 && slow.timing().overBudget == 1);
  errorIs(slow.sample(0, accel, gyro, 101325, result), "INVALID_TIMESTAMP");
  assert(slow.timing().step.count == 1 && slow.timing().overBudget == 1); slow.stop();
  assert(flash.mutations == writes);
  puts("HIL session tests passed without USB: lifecycle, ownership, timestamps, calibration and timing.");
}
} // namespace test_hil_session

namespace test_simulation {
using Outputs=FakeOutputs;
using command_test::send;
using namespace replay_test;
uint32_t nowUs = 0;
uint32_t clockUs() { return nowUs; }

void has(const std::string &reply, const char *part) {
  if (reply.find(part) == std::string::npos) { std::cerr << reply << " missing " << part << '\n'; assert(false); }
}
struct Device : storage::FlashDevice {
  std::vector<uint8_t> bytes = std::vector<uint8_t>(storage::FlashLayout::Capacity, 255);
  unsigned writes = 0; bool fail = false;
  bool read(uint32_t a, void *p, std::size_t n) override {
    if (a > bytes.size() || n > bytes.size()-a) return false;
    memcpy(p, bytes.data()+a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    if (fail || a > bytes.size() || n > bytes.size()-a) return false;
    ++writes; const auto *in = static_cast<const uint8_t *>(p);
    for (std::size_t i = 0; i < n; ++i) { assert((bytes[a+i]&in[i]) == in[i]); bytes[a+i] &= in[i]; }
    return true;
  }
  bool eraseSector(uint32_t a) override {
    if (fail || a%4096 || a > bytes.size()-4096) return false;
    ++writes; memset(bytes.data()+a,255,4096); return true;
  }
};
void run() {
  sensors::SimulatedSensorSource source(clockUs); sensors::Reading reading;
  assert(!source.poll(reading)); nowUs = 0xFFFFF000u; source.start();
  assert(source.poll(reading) && reading.timeUs == 0 && reading.accelMps2[0] > 9);
  assert(!reading.baroValid && std::isnan(reading.pressurePa));
  assert(!source.poll(reading)); nowUs += 5999; assert(!source.poll(reading));
  nowUs += 1; assert(source.poll(reading) && reading.timeUs == 6000 && reading.baroTimeUs == 6000);
  assert(reading.baroValid && reading.pressurePa==98560 && reading.temperatureC>36);
  nowUs += 16000; assert(source.poll(reading) && reading.timeUs == 20000);
  assert(source.missed() == 2 && source.maxLatenessUs() == 11000);
  assert(!source.poll(reading)); // No burst and no deadline drift.
  source.stop(); assert(!source.poll(reading));
  Device flash; storage::ConfigurationStore store(flash); assert(store.save(fixture(), limits, true));
  ConfigurationService config(store, limits); config.boot(true); ConfigurationCommands settings(config);
  flight::RunController run; storage::FlightRecorder recorder(flash);
  simulation::HilSession hilSession(config, run, clockUs); SensorHilCommands hil(hilSession);
  EstimateReplayCommands estimates(config, run);
  simulation::SimulationSession sim(config, run, clockUs, &recorder); SimulationCommands commands(sim);
  FlightCommands flights(recorder, config, run);
  CommandRouter router(settings, estimates, hil, config, run, &flights, &commands);
  CommandService usb; usb.setExtension(&router);
  simulation::SimulationSession noClock(config, run, nullptr);
  assert(std::string(noClock.begin()) == "TIMING_UNAVAILABLE");
  has(send(usb,"sim begin record"),"STORAGE_NOT_READY");
  has(send(usb,"sim begin physical"),"HARDWARE_CHECKS_UNAVAILABLE"); // This fixture has no GPIO executor.
  has(send(usb,"sim begin unknown"),"INVALID_ARGUMENT");
  config.beginDraft(); has(send(usb,"sim begin"),"DRAFT_ACTIVE"); config.discardDraft();
  const auto writes = flash.writes;
  has(send(usb,"@1 sim begin"),"\"active\":true"); const auto origin = nowUs;
  has(send(usb,"@1 sim begin"),"\"active\":true"); // Exact retry must not reset origin.
  has(send(usb,"sim begin"),"BUSY");
  for (const auto *command : {"hil stop", "hil sample 0 0 0 9.8 0 0 0 101325", "replay begin", "config begin", "flights recover"})
    has(send(usb,command),"BUSY");
  router.disconnect(); usb.reset(); assert(sim.active() && config.runLocked());
  for (uint32_t t = 0; t <= source.durationMs(); ++t) { nowUs = origin+t*1000; sim.poll(); assert(!sim.poll()); }
  assert(!sim.active() && !config.runLocked() && std::string(sim.reason()) == "completed");
  assert(sim.status().samples == 3600 && sim.invalidFrames() == 0 && sim.missed() == 0);
  assert(sim.phaseCount() == 4 && sim.startCount() == 3); // The 6..24 s crop ends during descent.
  for (unsigned i=0; i<4; ++i) {
    assert(unsigned(sim.phases()[i].phase) == i);
    std::cout << "Phase " << i << ": " << sim.phases()[i].timeMs << " ms\n";
  }
  assert(sim.starts()[0].timeMs == sim.phases()[3].timeMs);
  assert(sim.starts()[1].timeMs == sim.starts()[0].timeMs);
  assert(flash.writes == writes && sim.cycles().count == 3600);
  has(send(usb,"sim status"),"\"samples\":3600"); has(send(usb,"sim status"),"\"source\":\"hoppino\"");
  has(send(usb,"sim events"),"\"phases\":[[0,0],[1,4199],[2,6192],[3,8553]");
  has(send(usb,"hil begin 0 200 logged_body"),"\"active\":true");
  has(send(usb,"sim begin"),"BUSY"); has(send(usb,"sim status"),"\"samples\":3600"); send(usb,"sim stop");
  assert(hil.active()); send(usb,"hil stop");
  assert(recorder.initializeBlank());
  assert(!sim.begin(true)); const auto recordOrigin = nowUs;
  for (uint32_t t = 0; t <= source.durationMs(); ++t) { nowUs = recordOrigin+t*1000; sim.poll(); }
  assert(sim.status().savedSamples == 3600 && sim.status().droppedSamples == 0);
  assert(recorder.current()->status == storage::FlightStatus::Completed);
  assert(!config.setStateMachine(recovery::StateMachineSettings{}));
  storage::RecordingInfo info; assert(recorder.inspect(1,info));
  assert(info.samples == 3600);
  assert(!sim.begin()); assert(sim.poll()); nowUs += 17000; assert(sim.poll());
  assert(sim.missed() == 2 && sim.status().samples == 2 && sim.status().timeMs == 15);
  sim.stop(); assert(std::string(sim.reason()) == "stopped" && !config.runLocked());
  assert(!sim.begin(true)); flash.fail = true;
  for (unsigned i=0; i<15; ++i) { nowUs += 5000; sim.poll(); }
  assert(sim.active() && sim.status().recordingError != storage::RecorderError::None);
  sim.stop(); flash.fail = false;
  has(send(usb,"sim timing"),"\"period_us\":5000");
  has(send(usb,"sim stop extra"),"INVALID_ARGUMENT");
  assert(recorder.initialize()); // Recover the preceding injected storage failure.
  // The physical executor is the same sensor-independent runtime path, with a fake GPIO/PWM adapter.
  Outputs hardware; flight::OutputExecutor outputs(hardware,clockUs);
  simulation::SimulationSession physical(config,run,clockUs,&recorder,&outputs);
  SimulationCommands physicalCommands(physical); usb.setExtension(&physicalCommands); usb.reset();
  const auto untouched=hardware.writes;
  assert(!physical.begin()); physical.poll(); physical.stop();
  assert(hardware.writes==untouched); // Virtual mode must not even reset GPIO/PWM.
  has(send(usb,"sim begin record physical"),"INVALID_ARGUMENT");
  hardware.connected=0;
  has(send(usb,"sim begin physical"),"CONTINUITY_FAULT");
  has(send(usb,"sim begin physical"),"\"failed_pyro_mask\":3");
  assert(!physical.active() && !config.runLocked() && hardware.writes==untouched);
  hardware.connected=15;
  auto unset=fixture(); unset.servos[0]={}; assert(store.save(unset,limits,true)); config.boot(true);
  has(send(usb,"sim begin physical"),"SERVOS_NOT_CONFIGURED");
  assert(hardware.writes==untouched);
  assert(store.save(fixture(),limits,true)); config.boot(true);
  hardware.failServo=true;
  has(send(usb,"sim begin physical"),"SERVO_PREPARATION_FAILED");
  assert(!physical.active() && !hardware.enabled && !hardware.mask && !hardware.positions[0] && !config.runLocked());
  hardware.failServo=false;
  has(send(usb,"@42 sim begin physical record"),"\"output_mode\":\"PHYSICAL\"");
  const auto physicalOrigin=nowUs;
  assert(hardware.positions[0]==fixture().servos[0].initialUs && physical.controller().calibrationSamples()==0);
  const auto prepareWrites=hardware.writes;
  send(usb,"@42 sim begin physical record"); assert(hardware.writes==prepareWrites); // No replay of accepted preparation.
  assert(recorder.metadata()->sensorMode==storage::SensorMode::Simulation && recorder.metadata()->outputMode==storage::OutputMode::Physical);
  for (uint32_t t=0;t<=source.durationMs();++t) { nowUs=physicalOrigin+t*1000; physical.poll(); }
  assert(!physical.active() && !hardware.mask && !hardware.enabled && !hardware.positions[0]);
  assert(!strcmp(physical.reason(),"completed") && !strcmp(physical.error(),"None"));
  assert(physical.startCount()==3 && physical.status().outputMode==storage::OutputMode::Physical && !physical.status().outputsActive);
  const auto trigger=physical.phases()[3].timeMs;
  bool servoSeen=false, pyroSeen=false;
  for (unsigned i=0;i<physical.startCount();++i) {
    const auto &s=physical.starts()[i];
    if (s.kind==recovery::OutputKind::Servo) { assert(s.timeMs==trigger); servoSeen=true; }
    if (s.kind==recovery::OutputKind::Pyro && s.channel==1) { assert(s.timeMs==trigger+recovery::PyroEnableSettleMs); pyroSeen=true; }
  }
  assert(servoSeen && pyroSeen);
  has(send(usb,"sim events"),"\"output_mode\":\"PHYSICAL\"");
  assert(recorder.inspect(physical.status().recordingSequence,info));
  storage::FlightHeaderStore stored(flash); assert(stored.load(info.entry.start));
  assert(stored.view()->outputMode==storage::OutputMode::Physical);
  for (unsigned i=0;i<physical.startCount();++i) {
    const auto &start=physical.starts()[i]; bool found=false;
    for (const auto &slot:stored.view()->starts)
      if (slot.rule==start.rule && slot.kind==start.kind && slot.channel==start.channel) {
        assert(slot.state==storage::StartState::Recorded && slot.timeMs==start.timeMs); found=true;
      }
    assert(found);
  }

  // Pulse deadlines progress without sensor frames, including timer rollover and concurrent channels.
  recovery::VirtualStart first,second; first.rule=10; first.channel=1; second.rule=20; second.channel=2;
  nowUs=0xfffff000u; assert(!outputs.begin(fixture(),100));
  assert(outputs.submit(first));
  assert(outputs.submit(second));
  assert(!outputs.submit(first));
  outputs.poll(); assert(hardware.enabled && hardware.mask==0);
  nowUs+=recovery::PyroEnableSettleMs*1000-1; outputs.poll(); assert(hardware.mask==0);
  nowUs+=1; outputs.poll(); assert(hardware.mask==3);
  recovery::VirtualStart starts[8]; assert(outputs.drain(starts,8)==2);
  nowUs+=recovery::PyroPulseMs*1000-1; outputs.poll(); assert(hardware.mask==3);
  nowUs+=1; outputs.poll(); assert(hardware.mask==0 && !hardware.enabled);
  outputs.stop(99); assert(outputs.owner()==100); // A stale owner cannot stop a later run.
  outputs.stop(100); assert(!outputs.owner() && !hardware.positions[0]);
  assert(!outputs.begin(fixture(),101));
  assert(outputs.submit(first)); outputs.poll();
  assert(outputs.stop(101)==1); nowUs+=100000; outputs.poll(); assert(!hardware.mask && !hardware.enabled);
  assert(!outputs.begin(fixture(),102));
  assert(outputs.submit(first)); outputs.poll();
  nowUs+=recovery::PyroEnableSettleMs*1000; outputs.poll(); assert(hardware.mask==1);
  outputs.stop(102); assert(!hardware.mask && !hardware.enabled && !hardware.positions[0]);
  auto prepared=fixture(); prepared.servos[1]=prepared.servos[0];
  assert(!outputs.prepare(prepared,1)); hardware.failChannel=2;
  assert(std::string(outputs.prepare(prepared,3))=="SERVO_PREPARATION_FAILED");
  assert(outputs.preparedServos(prepared)==1 && hardware.positions[0] && !hardware.positions[1]);
  outputs.releasePrepared();
  assert(std::string(outputs.prepare(prepared,3))=="SERVO_PREPARATION_FAILED");
  assert(!outputs.hasPreparation() && !hardware.positions[0] && !hardware.positions[1]);
  hardware.failChannel=0; assert(!outputs.prepare(prepared,1));
  prepared.servos[0].finalUs=1500;
  assert(outputs.preparedServos(prepared)==0); outputs.reconcile(&prepared);
  assert(!outputs.hasPreparation() && !hardware.positions[0]);
  assert(!outputs.prepare(prepared,1));
  for (std::size_t i=0;i<prepared.rules[0].assignmentCount;++i)
    if (prepared.rules[0].assignments[i].kind==recovery::OutputKind::Servo) prepared.rules[0].assignments[i].kind=recovery::OutputKind::Pyro;
  outputs.reconcile(&prepared); assert(!outputs.hasPreparation() && !hardware.positions[0]);
  puts("Standalone simulation passed: clock rollover, skipped deadlines, sensor-derived phases, shared rules, recording, isolation and USB lifecycle.");
}
void bistage() {
  MemoryFlash settingsFlash; storage::ConfigurationStore store(settingsFlash); assert(store.save(fixture(),limits,true));
  ConfigurationService config(store,limits); config.boot(true);
  recovery::StateMachineSettings machine; machine.bistage=true; machine.minimumCoastVelocity=5;
  assert(!config.setStateMachine(machine));
  flight::RunController run; simulation::SimulationSession session(config,run,clockUs);
  nowUs=0; assert(!session.begin());
  assert(session.bistage() && session.durationMs()==25000 && session.totalSamples()==5001);
  for (unsigned t=0;t<=25000;++t) { nowUs=t*1000; session.poll(); }
  const recovery::FlightPhase expected[]={recovery::FlightPhase::Pad,recovery::FlightPhase::Boost,recovery::FlightPhase::Coast,recovery::FlightPhase::Boost2,recovery::FlightPhase::Coast2,recovery::FlightPhase::Descent,recovery::FlightPhase::Landed};
  assert(!session.active() && session.phaseCount()==7);
  for (unsigned i=0;i<7;++i) { if (session.phases()[i].phase!=expected[i]) { std::cerr<<"bistage phase "<<i<<" got "<<unsigned(session.phases()[i].phase)<<'\n'; assert(false); } }
  assert(!strcmp(session.error(),"None"));
  assert(!config.setStateMachine(recovery::StateMachineSettings{}));
  assert(!session.begin()); assert(!session.bistage() && session.durationMs()==sensors::SimulatedSensorSource::durationMs());
  session.stop(); assert(!config.setStateMachine(machine));

  // A physical ABORT cancels future starts but lets an already-high pulse complete.
  auto plan=fixture(); plan.rules[0].conditions[0].kind=recovery::ConditionKind::Phase;
  plan.rules[0].conditions[0].phases=recovery::BoostMask; plan.rules[0].delayMs=960;
  plan.rules[1].conditions[0].kind=recovery::ConditionKind::Phase; plan.rules[1].conditions[0].phases=recovery::CoastMask;
  assert(store.save(plan,limits,true)); config.boot(true); machine.minimumCoastVelocity=1000; assert(!config.setStateMachine(machine));
  Device flash; storage::FlightRecorder recorder(flash); assert(recorder.initializeBlank());
  Outputs hardware; flight::OutputExecutor outputs(hardware,clockUs); flight::RunController physicalRun;
  simulation::SimulationSession aborted(config,physicalRun,clockUs,&recorder,&outputs);
  nowUs=0; assert(!aborted.begin(true,true)); bool sawAbort=false,sawHigh=false,sawEnd=false;
  for (unsigned t=0;t<=25000;++t) {
    nowUs=t*1000; aborted.poll();
    if (t==100) assert(!strcmp(config.setStateMachine(recovery::StateMachineSettings{}),"BUSY"));
    if (aborted.navigation().phase==recovery::FlightPhase::Abort) {
      sawAbort=true; assert(aborted.status().abortReason==recovery::AbortReason::Velocity);
      assert(!(hardware.mask&2)); if (hardware.mask&1) sawHigh=true;
      if (sawHigh && !hardware.mask) sawEnd=true;
    }
  }
  assert(sawAbort && sawHigh && sawEnd && aborted.startCount()==2);
  assert(!aborted.active() && !hardware.enabled && !hardware.mask && !hardware.positions[0]);
  assert(!config.setStateMachine(recovery::StateMachineSettings{}));
  storage::RecordingInfo info; assert(recorder.inspect(1,info));
  assert(recorder.metadata()->calibration.machine.bistage && recorder.metadata()->calibration.machine.minimumCoastVelocity==1000);
  storage::SampleBlock block; bool savedAbort=false;
  for (uint32_t index=0;index<(info.entry.committedBytes-info.headerBytes)/storage::FlightSampleStore::BlockBytes;++index) {
    assert(recorder.readBlock(1,index,block));
    for (unsigned i=0;i<block.count;++i) if (block.samples[i].abortReason==recovery::AbortReason::Velocity) savedAbort=true;
  }
  assert(savedAbort);
}
} // namespace test_simulation

int main() {
  std::puts("  hil_session"); std::fflush(stdout);
  test_hil_session::run();
  std::puts("  simulation"); std::fflush(stdout);
  test_simulation::run();
  test_simulation::bistage();
}
