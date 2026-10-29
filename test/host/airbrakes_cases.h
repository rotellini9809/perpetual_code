#pragma once
// Included by flight_test after its common fixtures.
namespace test_airbrakes {
using namespace recovery;
uint32_t ticks=0;
uint32_t clockUs() { return ticks; }
control::ControlInput captured;
control::ControlOutput law(const control::ControlInput &in,control::LawState &state) {
  captured=in; state.memory[0]+=1;
  control::ControlOutput out; out.opening={{state.memory[0]/4,2,-1,.5f}}; return out;
}
control::ControlOutput badLaw(const control::ControlInput &,control::LawState &) {
  control::ControlOutput out; out.opening[0]=NAN; return out;
}
Configuration plan() {
  Configuration c; c.airbrakes.channelMask=3; c.airbrakes.conditionCount=2;
  c.servos[0].calibrated=c.servos[1].calibrated=true;
  c.servos[0].initialUs=1000; c.servos[0].finalUs=2000; c.servos[1].initialUs=1900; c.servos[1].finalUs=1100;
  c.airbrakes.conditions[0].kind=ConditionKind::Phase; c.airbrakes.conditions[0].phases=CoastMask|Coast2Mask;
  auto &h=c.airbrakes.conditions[1]; h.kind=ConditionKind::AltitudeComparison; h.comparison=Comparison::Greater; h.value=10;
  return c;
}
void run() {
  const auto limits=replay_test::limits;
  auto c=plan(); assert(validate(c,limits).ok());
  auto invalid=c; invalid.servos[0].finalUs=1000; assert(!validate(invalid,limits).ok());
  invalid=c; invalid.ruleCount=1; invalid.rules[0]=replay_test::fixture().rules[0];
  assert(validate(invalid,limits).error==ValidationError::DuplicateOutput);
  replay_test::MemoryFlash flash; storage::ConfigurationStore store(flash);
  assert(store.save(c,limits,true)); Configuration loaded; assert(store.load(loaded,limits));
  assert(loaded.airbrakes.channelMask==3 && loaded.airbrakes.conditionCount==2);
  ConfigurationService config(store,limits); config.boot(true); assert(!config.airbrakesEnabled()); assert(!config.setAirbrakes(true)); config.boot(true); assert(config.airbrakesEnabled()); flight::RunController owner;
  assert(!owner.begin(flight::RunMode::Sensors,config)); assert(!strcmp(config.setAirbrakeChannels(0),"BUSY")); assert(!strcmp(config.setAirbrakes(false),"BUSY"));
  owner.stop(flight::RunMode::Sensors);
  RuleEngine rules; assert(rules.begin(c,limits,100));
  FlightSnapshot s; s.phaseValid=true; s.phase=FlightPhase::Coast; s.altitude.value=20; s.altitude.valid=true; s.altitude.sequence=1; s.verticalVelocity=s.altitude;
  assert(rules.evaluate(0,s).accepted && rules.airbrakeConditions(0,s)==0);
  s.altitude.value=5; assert(rules.airbrakeConditions(0,s)==2);
  s.phase=FlightPhase::Boost; assert(rules.airbrakeConditions(0,s)==3);
  assert(rules.airbrakeConditions(101,s)==3);

  control::AirbrakeController ctl; ctl.begin(3); ctl.setLaw(law);
  navigation::NavigationOutput n; n.valid=true; n.phase=FlightPhase::Coast; n.altitude=20;
  n.verticalVelocity=7; n.accelerationBias=.2f; n.verticalAcceleration=-3; n.dtSeconds=.005f; n.timeUs=12345;
  n.covariance={{{{1,.1f,.2f}},{{.1f,2,.3f}},{{.2f,.3f,3}}}};
  ctl.update(n,0); assert(ctl.status().opening[0]==2500 && ctl.status().opening[1]==10000);
  assert(captured.state[2]==n.accelerationBias && captured.covariance==n.covariance && captured.acceleration==-3);
  assert(captured.dtSeconds==n.dtSeconds && captured.timeUs==12345 && captured.valid);
  ctl.update(n,0); assert(ctl.status().opening[0]==5000);
  ctl.update(n,2); assert(ctl.status().opening[0]==0 && (ctl.status().flags&control::RuleFalse));
  ctl.update(n,0); assert(ctl.status().opening[0]==2500); // Re-enable resets controller memory.
  n.covariance[1][2]=NAN; ctl.update(n,0); assert(ctl.status().flags&control::InvalidEstimate);
  n.covariance[1][2]=.3f; ctl.update(n,0); assert(ctl.status().opening[0]==2500);
  n.phase=FlightPhase::Abort; ctl.update(n,0); assert(ctl.status().flags&control::Aborted);
  n.phase=FlightPhase::Coast; ctl.update(n,0); assert(ctl.status().opening[0]==0 && (ctl.status().flags&control::Aborted));
  ctl.begin(3); ctl.setLaw(badLaw); ctl.update(n,0); assert(ctl.status().flags&control::InvalidRequest);
  ctl.begin(3); ctl.setLaw(nullptr); ctl.update(n,0); assert(ctl.status().flags&control::Enabled); assert(ctl.commands()[0]==0);

  FakeOutputs gpio; flight::OutputExecutor outputs(gpio,clockUs);
  assert(!strcmp(outputs.begin(c,1,true),"SERVOS_NOT_PREPARED")); assert(!outputs.prepare(c,3));
  assert(!outputs.begin(c,1,true)); assert(outputs.airbrakes(1,{{.25f,.25f,0,0}}));
  assert(gpio.positions[0]==1250 && gpio.positions[1]==1700); // Reversed endpoints.
  assert(!outputs.airbrakes(2,{{1,1,0,0}})); assert(!outputs.airbrakes(1,{{NAN,1,0,0}}));
  gpio.failChannel=1; assert(!outputs.airbrakes(1,{{0,0,0,0}}) && gpio.positions[1]==1900);
  gpio.failChannel=0; outputs.stop(1); assert(gpio.positions[0]==1000 && outputs.preparedServos(c)==3);
  outputs.releasePrepared(); assert(!gpio.positions[0] && !gpio.positions[1]);

  // Shared runtime: physical timeout closes, immutable header and samples round-trip.
  test_flight_retention::Nor memory; storage::FlightRecorder recorder(memory); assert(recorder.initializeBlank());
  flight::RunController run; flight::FlightRuntime runtime(config,run,clockUs,&recorder,&outputs);
  sensors::HilSensorSource source; navigation::SensorCalibration calibration; calibration.configured=true;
  ticks=0; assert(!runtime.begin(source,0,200,calibration,8192,storage::SensorMode::Simulation,storage::OutputMode::Physical));
  sensors::Reading frame; frame.imuValid=frame.baroValid=true; frame.accelMps2[0]=9.80665f;
  frame.pressurePa=101325; frame.imuSequence=frame.baroSequence=1;
  assert(source.submit(frame)); flight::FrameResult result; assert(runtime.poll(result).processed);
  assert(runtime.status().airbrakes.flags&control::RuleFalse);
  ticks=100001; assert(!runtime.poll(result).error && (runtime.status().airbrakes.flags&control::StaleEstimate));
  assert(gpio.positions[0]==1000 && gpio.positions[1]==1900);
  frame.timeUs=5000; frame.imuSequence=2; frame.imuValid=false; assert(source.submit(frame)); assert(runtime.poll(result).processed);
  assert(runtime.status().airbrakes.flags&control::InvalidEstimate);
  runtime.stop(); assert(outputs.hasPreparation());
  storage::RecordingInfo info; assert(recorder.inspect(1,info));
  assert(recorder.metadata()->calibration.airbrakeLawVersion==control::AirbrakeLawVersion && recorder.metadata()->calibration.airbrakesEnabled);
  assert(recorder.metadata()->configuration.airbrakes.channelMask==3);
  storage::SampleBlock block; assert(recorder.readBlock(1,0,block) && block.count==2);
  assert(block.samples[1].airbrakes.flags&control::InvalidEstimate);
  assert(block.samples[1].airbrakes.opening[0]==0 && block.samples[1].airbrakes.opening[2]==65535);
  outputs.releasePrepared(); assert(!config.setAirbrakes(false));
  const auto servoWrites=gpio.servoCommands.size();
  assert(!runtime.begin(source,0,200,calibration,0,storage::SensorMode::Simulation,storage::OutputMode::Physical));
  assert(!runtime.status().airbrakes.channelMask && gpio.servoCommands.size()==servoWrites);
  runtime.stop(); assert(gpio.servoCommands.size()==servoWrites);
  assert(!config.setAirbrakes(true)); const auto writes=gpio.writes;
  assert(!runtime.begin(source,0,200,calibration)); frame.timeUs=0; frame.imuSequence=1; frame.imuValid=true;
  assert(source.submit(frame)); assert(runtime.poll(result).processed); runtime.stop(); assert(gpio.writes==writes);
  assert(!config.factoryReset() && !config.applied() && !config.airbrakesEnabled());
  puts("Airbrakes passed: AND gate, Kalman contract, normalized/reversed travel, faults, reset, exclusive channels, timeout, virtual isolation and frozen logs.");
}
}
