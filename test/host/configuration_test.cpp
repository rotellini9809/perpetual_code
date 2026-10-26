#include <cstdio>
#include "configuration/RecoveryConfiguration.h"
#include <cassert>
#include <cstring>
#include <limits>
#include "configuration/RecoveryRuleEngine.h"
#include <utility>
#include "configuration/ConfigurationService.h"
#include "flight/RunController.h"
#include "../fixtures/ReplayFixture.h"
#include <functional>
#include <vector>
#include <string>

// Each namespace isolates one component scenario and its fault model.
namespace test_recovery_configuration {
using namespace recovery;
// Test policy only, not production actuator limits.
const ValidationLimits limits{1000, 2000, 1000, 60000};

Configuration plan() {
  Configuration c;
  c.ruleCount = 1;
  auto &r = c.rules[0];
  r.id = 10;
  std::strcpy(r.name.data(), "Main");
  r.conditionCount = 1;
  r.conditions[0].kind = ConditionKind::ApogeeDetected;
  r.assignmentCount = 1;
  r.assignments[0].channel = 1;
  c.pyros[0].pulseMs = 100;
  return c;
}
void expect(const Configuration &c, ValidationError error) {
  const auto result = validate(c, limits);
  if (result.error != error) {
    std::printf("Expected %s, got %s\n", errorName(error), errorName(result.error));
    assert(false);
  }
}
void run() {
  auto c = plan();
  expect(c, ValidationError::None);
  expect(Configuration{}, ValidationError::EmptyPlan);
  c.ruleCount = MaxRules + 1; expect(c, ValidationError::TooManyRules);
  c = plan(); c.rules[0].conditionCount = MaxConditions + 1;
  expect(c, ValidationError::InvalidConditionCount);
  c = plan(); c.rules[0].assignmentCount = MaxAssignments + 1;
  expect(c, ValidationError::InvalidAssignmentCount);
  c = plan(); c.rules[0].name.fill('x'); expect(c, ValidationError::InvalidName);
  c = plan(); c.rules[0].assignments[0].channel = 0; expect(c, ValidationError::InvalidOutput);
  c = plan(); c.rules[0].assignments[0].kind = static_cast<OutputKind>(99);
  expect(c, ValidationError::InvalidOutput);
  c = plan(); c.rules[0].assignmentCount = 2;
  c.rules[0].assignments[1] = c.rules[0].assignments[0];
  const auto duplicate = validate(c, limits);
  assert(duplicate.error == ValidationError::DuplicateOutput && duplicate.rule == 10 && duplicate.index == 1);
  c = plan(); c.ruleCount = 2; c.rules[1] = c.rules[0];
  expect(c, ValidationError::DuplicateRuleId);
  c.rules[1].id = 20; expect(c, ValidationError::DuplicateOutput);
  c.rules[1].assignments[0].channel = 2; c.pyros[1].pulseMs = 100;
  expect(c, ValidationError::None); // Duplicate names are permitted.
  c.rules[0].conditions[0].kind = ConditionKind::TimeSinceRule;
  c.rules[0].conditions[0].reference = 20;
  expect(c, ValidationError::None);
  c.rules[1].conditions[0].kind = ConditionKind::TimeSinceRule;
  c.rules[1].conditions[0].reference = 10;
  expect(c, ValidationError::DependencyCycle);
  c.rules[1].conditions[0].reference = 999; expect(c, ValidationError::InvalidReference);
  c.rules[1].conditions[0].reference = 20; expect(c, ValidationError::InvalidReference);
  c = plan(); auto &condition = c.rules[0].conditions[0];
  condition.kind = ConditionKind::AltitudeRange;
  condition.value = -50; condition.upper = -10; expect(c, ValidationError::None);
  condition.upper = -100; expect(c, ValidationError::InvalidCondition);
  condition.upper = std::numeric_limits<float>::infinity(); expect(c, ValidationError::InvalidCondition);
  condition.kind = ConditionKind::AltitudeComparison;
  condition.value = std::numeric_limits<float>::quiet_NaN(); expect(c, ValidationError::InvalidCondition);
  condition.value = 100; condition.comparison = Comparison::LessEqual;
  expect(c, ValidationError::InvalidCondition);
  condition.comparison = Comparison::Less; expect(c, ValidationError::None);
  condition.kind = ConditionKind::AbsoluteVelocityComparison;
  expect(c, ValidationError::InvalidCondition);
  condition.comparison = Comparison::GreaterEqual; expect(c, ValidationError::None);
  condition.value = -1; expect(c, ValidationError::InvalidCondition);
  condition.kind = ConditionKind::Phase; condition.phases = AirborneMask;
  expect(c, ValidationError::None);
  condition.phases |= 1; expect(c, ValidationError::InvalidCondition);
  condition.phases = 0; expect(c, ValidationError::InvalidCondition);
  condition.kind = static_cast<ConditionKind>(99); expect(c, ValidationError::InvalidCondition);
  c = plan(); c.rules[0].delayMs = limits.maxTimingMs + 1;
  expect(c, ValidationError::InvalidTiming);
  c = plan(); c.rules[0].assignments[0].delayMs = limits.maxTimingMs + 1;
  const auto timing = validate(c, limits);
  assert(timing.error == ValidationError::InvalidTiming && timing.field == Field::Assignment && timing.rule == 10 && timing.index == 0);
  c = plan(); c.pyros[0].pulseMs = 0; expect(c, ValidationError::InvalidPyroSettings);
  c = plan(); c.rules[0].assignments[0].kind = OutputKind::Servo;
  expect(c, ValidationError::None); // Save an assigned servo before defining its endpoints.
  c.servos[0].initialUs = 1000; expect(c, ValidationError::InvalidServoSettings);
  c.servos[0].calibrated = true; c.servos[0].initialUs = 1000;
  c.servos[0].finalUs = 2000;
  expect(c, ValidationError::None);
  c.servos[0].finalUs = 999; expect(c, ValidationError::InvalidServoSettings);
  auto invalidLimits = limits; invalidLimits.minServoUs = 0;
  assert(validate(plan(), invalidLimits).error == ValidationError::InvalidLimits);
  std::puts("Recovery configuration tests passed.");
}
} // namespace test_recovery_configuration

namespace test_recovery_rule_engine {
using namespace recovery;
const ValidationLimits limits{1000, 2000, 1000, 60000};
Configuration plan() {
  Configuration c;
  c.ruleCount = 1;
  auto &r = c.rules[0]; r.id = 10; std::strcpy(r.name.data(), "Main");
  r.conditionCount = r.assignmentCount = 1;
  r.conditions[0].kind = ConditionKind::ApogeeDetected;
  r.assignments[0].channel = 1; c.pyros[0].pulseMs = 100;
  return c;
}
FlightSnapshot sample(uint64_t t, FlightPhase phase = FlightPhase::Descent) {
  FlightSnapshot s;
  s.sequence = t + 1; s.phase = phase; s.phaseValid = true; s.phaseMeasuredMs = t;
  s.altitude.value = 100; s.altitude.valid = true; s.altitude.measuredMs = t; s.altitude.sequence = t + 1;
  s.verticalVelocity = s.altitude; s.verticalVelocity.value = -20;
  return s;
}
void start(RuleEngine &e, const Configuration &c) { assert(e.begin(c, limits, 100)); }
void ruleDelayAndValidity() {
  auto c = plan(); c.rules[0].delayMs = 100;
  c.rules[0].conditions[0].kind = ConditionKind::AltitudeComparison;
  c.rules[0].conditions[0].comparison = Comparison::Less;
  c.rules[0].conditions[0].value = 200;
  RuleEngine e; start(e, c);
  assert(e.evaluate(0, sample(0, FlightPhase::Pad)).startCount == 0);
  assert(e.evaluate(10, sample(10)).startCount == 0);
  assert(e.rules()[0].triggered && e.rules()[0].triggeredMs == 10);
  assert(e.actions()[0].state == ActionState::Pending);
  auto s = sample(60); s.altitude.valid = false;
  assert(e.evaluate(60, s).startCount == 0);
  s = sample(100); s.altitude.value = 300; // Condition is now false; the delay keeps running.
  assert(e.evaluate(100, s).startCount == 0);
  s = sample(110); s.phaseValid = false; s.altitude.valid = false;
  const auto result = e.evaluate(110, s);
  assert(result.startCount == 1 && result.starts[0].timeMs == 110);
  assert(e.evaluate(180, sample(180)).startCount == 0);
  e.end(); start(e, c); // Re-arm clears per-flight state.
  e.evaluate(0, sample(0)); e.evaluate(50, sample(50));
  e.evaluate(75, sample(50)); // Reused measurements cannot cancel an already scheduled action.
  assert(e.evaluate(100, sample(100)).startCount == 1);
  e.end(); start(e, c); e.evaluate(0, sample(0));
  s = sample(201); s.phaseValid = false;
  assert(e.evaluate(201, s).startCount == 1); // A gap does not restart a latched timer.
  e.end(); start(e, c); e.evaluate(0, sample(0)); e.closeStarts();
  assert(e.actions()[0].state == ActionState::Cancelled);
  assert(e.evaluate(100, sample(100)).startCount == 0);
  e.end(); start(e, c);
  s = sample(0); s.altitude.value = std::numeric_limits<float>::quiet_NaN();
  e.evaluate(0, s); assert(!e.rules()[0].triggered);
  s = sample(200); s.altitude.measuredMs = 0;
  e.evaluate(200, s); assert(!e.rules()[0].triggered);
  s = sample(210); s.altitude.measuredMs = 211;
  e.evaluate(210, s); assert(!e.rules()[0].triggered);
  s = sample(220); s.altitude.sequence = 211;
  e.evaluate(220, s); assert(!e.rules()[0].triggered);
  // A single rule delay applies to every output, including concurrent pyro/servo starts.
  c = plan(); c.rules[0].delayMs = 100; c.rules[0].assignmentCount = 2;
  c.rules[0].assignments[1].kind = OutputKind::Servo; c.rules[0].assignments[1].channel = 1;
  c.servos[0].calibrated=true; c.servos[0].initialUs=1000;
  c.servos[0].finalUs=2000;
  e.end(); start(e,c); assert(e.evaluate(0,sample(0)).startCount==0);
  const auto both=e.evaluate(100,sample(100));
  assert(both.startCount==2 && both.starts[0].timeMs==100 && both.starts[1].timeMs==100);
  // Channel waits add to the common rule wait; missing/false inputs do not reset them.
  c.rules[0].assignments[0].delayMs=50; c.rules[0].assignments[1].delayMs=200;
  e.end(); start(e,c); assert(e.evaluate(0,sample(0)).startCount==0);
  assert(e.evaluate(100,sample(100)).startCount==0);
  assert(e.evaluate(149,sample(149)).startCount==0);
  auto delayed=sample(150); delayed.altitude.valid=false;
  const auto pyro=e.evaluate(150,delayed);
  assert(pyro.startCount==1 && pyro.starts[0].kind==OutputKind::Pyro && pyro.starts[0].timeMs==150);
  assert(e.actions()[1].state==ActionState::Pending);
  assert(e.evaluate(299,sample(299,FlightPhase::Landed)).startCount==0);
  const auto servo=e.evaluate(300,sample(300,FlightPhase::Landed));
  assert(servo.startCount==1 && servo.starts[0].kind==OutputKind::Servo && servo.starts[0].timeMs==300);
  assert(e.evaluate(400,sample(400)).startCount==0);
  e.end(); start(e,c); e.evaluate(0,sample(0)); e.evaluate(150,sample(150)); e.closeStarts();
  assert(e.actions()[1].state==ActionState::Cancelled && e.evaluate(300,sample(300)).startCount==0);
}
void dependencies(bool reverse) {
  auto c = plan(); c.ruleCount = 2; c.rules[1] = c.rules[0];
  c.rules[1].id = 20; c.rules[1].assignments[0].channel = 2; c.pyros[1].pulseMs = 100;
  c.rules[1].conditions[0].kind = ConditionKind::TimeSinceRule;
  c.rules[1].conditions[0].reference = 10;
  if (reverse) std::swap(c.rules[0], c.rules[1]);
  RuleEngine e; start(e, c);
  auto result = e.evaluate(0, sample(0));
  assert(result.startCount == 1 && result.starts[0].rule == 10);
  result = e.evaluate(1, sample(1));
  assert(result.startCount == 1 && result.starts[0].rule == 20);
}
void conditions() {
  auto c = plan(); auto &r = c.rules[0]; r.conditionCount = 3;
  r.conditions[0].kind = ConditionKind::Phase; r.conditions[0].phases = DescentMask;
  r.conditions[1].kind = ConditionKind::AltitudeRange;
  r.conditions[1].value = r.conditions[1].upper = 100;
  r.conditions[2].kind = ConditionKind::AbsoluteVelocityRange;
  r.conditions[2].value = r.conditions[2].upper = 20;
  for (float velocity : {-20.f, 20.f}) {
    RuleEngine e; start(e, c); auto s = sample(0); s.verticalVelocity.value = velocity;
    assert(e.evaluate(0, s).startCount == 1);
  }
  r.conditionCount = 1; auto &condition = r.conditions[0];
  condition.kind = ConditionKind::AltitudeComparison; condition.value = 100;
  for (Comparison comparison : {Comparison::Less, Comparison::Greater}) {
    condition.comparison = comparison; RuleEngine e; start(e, c);
    assert(e.evaluate(0, sample(0)).startCount == 0); // Strict boundary.
  }
  condition.kind = ConditionKind::AbsoluteVelocityComparison; condition.value = 20;
  for (Comparison comparison : {Comparison::LessEqual, Comparison::GreaterEqual}) {
    condition.comparison = comparison; RuleEngine e; start(e, c);
    assert(e.evaluate(0, sample(0)).startCount == 1);
  }
  for (ConditionKind kind : {ConditionKind::TimeSinceLaunch, ConditionKind::TimeSinceApogee}) {
    condition.kind = kind; condition.elapsedMs = 100;
    RuleEngine e; start(e, c);
    assert(e.evaluate(0, sample(0, FlightPhase::Pad)).startCount == 0);
    assert(e.evaluate(10, sample(10)).startCount == 0);
    assert(e.evaluate(109, sample(109)).startCount == 0);
    assert(e.evaluate(110, sample(110)).startCount == 1);
  }
}
void lifecycleAndConcurrency() {
  auto c = plan(); auto &r = c.rules[0]; r.assignmentCount = 2;
  r.assignments[1].channel = 2; c.pyros[1].pulseMs = 100;
  RuleEngine e; start(e, c);
  assert(!e.begin(c, limits, 100));
  assert(e.evaluate(0, sample(0)).startCount == 2);
  assert(e.actions()[0].startedMs == 0); // Zero is a valid start timestamp.
  e.end(); start(e, c);
  assert(e.evaluate(10, sample(10)).startCount == 2);
  assert(e.evaluate(50, sample(50, FlightPhase::Landed)).startCount == 0);
  auto s = sample(110, FlightPhase::Landed); s.phaseValid = false;
  assert(e.evaluate(110, s).startCount == 0); // Starts never repeat, including after landing.
  e.end(); start(e, c);
  e.closeStarts(); assert(!e.rules()[0].triggered);
  assert(e.evaluate(110, sample(110)).startCount == 0);
  e.end(); assert(!e.evaluate(120, sample(120)).accepted);
  start(e, c); e.evaluate(10, sample(10));
  assert(!e.evaluate(9, sample(9)).accepted);
  assert(e.evaluate(110, sample(110)).startCount == 0);
  e.end(); start(e, c);
  e.evaluate(0, sample(0, FlightPhase::Landed));
  assert(e.evaluate(1, sample(1)).startCount == 0); // Landing is latched.
  assert(!e.rules()[0].triggered);
  e.end(); assert(!e.begin(Configuration{}, limits, 100));
  assert(!e.active());
  // All eight possible virtual starts are returned together without serialization.
  c = plan(); c.rules[0].assignmentCount = 8;
  for (std::size_t i = 0; i < 8; ++i) {
    auto &a = c.rules[0].assignments[i];
    a.kind = i < 4 ? OutputKind::Pyro : OutputKind::Servo;
    a.channel = static_cast<uint8_t>(i % 4 + 1);
    c.pyros[i % 4].pulseMs = 100;
    auto &servo = c.servos[i % 4]; servo.calibrated = true;
    servo.initialUs = 1000; servo.finalUs = 2000;
  }
  start(e, c); assert(e.evaluate(0, sample(0)).startCount == 8);
}
void simulatedFlight() {
  auto c = plan();
  c.rules[0].delayMs = 50;
  RuleEngine e; start(e, c);
  std::puts("Simulated flight: PAD -> BOOST -> COAST -> DESCENT -> LANDED");
  e.evaluate(0, sample(0, FlightPhase::Pad));
  e.evaluate(100, sample(100, FlightPhase::Boost));
  e.evaluate(200, sample(200, FlightPhase::Coast));
  e.evaluate(300, sample(300, FlightPhase::Descent));
  const auto result = e.evaluate(350, sample(350, FlightPhase::Descent));
  assert(e.rules()[0].triggeredMs == 300);
  assert(result.startCount == 1);
  assert(result.starts[0].timeMs == 350);
  assert(e.evaluate(450, sample(450, FlightPhase::Descent)).startCount == 0);
  std::printf("VIRTUAL: rule %lu, pyro %u started at %llu ms (triggered at 300 ms).\n",
      static_cast<unsigned long>(result.starts[0].rule),
      static_cast<unsigned>(result.starts[0].channel),
      static_cast<unsigned long long>(result.starts[0].timeMs));
  e.evaluate(550, sample(550, FlightPhase::Landed));
  e.closeStarts(); e.end();
}
void run() {
  ruleDelayAndValidity(); dependencies(false); dependencies(true);
  conditions(); lifecycleAndConcurrency(); simulatedFlight();
  std::puts("Recovery rule engine tests passed (virtual outputs only).");
}
} // namespace test_recovery_rule_engine

namespace test_configuration_service {
using namespace replay_test;
std::vector<uint8_t> encoded(const recovery::Configuration &config) {
  std::vector<uint8_t> bytes(storage::ConfigurationStore::MaxPayload); std::size_t n = 0;
  assert(storage::ConfigurationStore::encode(config, bytes.data(), bytes.size(), n)); bytes.resize(n); return bytes;
}
void run() {
  MemoryFlash flash; storage::ConfigurationStore store(flash);
  assert(store.save(fixture(), limits, true));
  ConfigurationService config(store, limits); config.boot(true);
  const auto writes = flash.mutations;
  const auto applied = encoded(*config.applied());
  recovery::Condition condition;
  using recovery::OutputKind;
  std::vector<std::function<const char *()>> edits = {
    [&]() { return config.setPyro(1, 150); },
    [&]() { return config.setServo(1, 1100, 1900); },
    [&]() { return config.addRule(30, "New"); },
    [&]() { return config.renameRule(10, "Renamed"); },
    [&]() { return config.setRuleDelay(10, 50); },
    [&]() { return config.removeRule(20); },
    [&]() { return config.addCondition(10, condition); },
    [&]() { return config.replaceCondition(10, 0, condition); },
    [&]() { return config.removeCondition(10, 0); },
    [&]() { return config.assignOutput(10, OutputKind::Pyro, 3); },
    [&]() { return config.unassignOutput(10, OutputKind::Pyro, 1); }
  };
  auto guarded = [&](const char *error) {
    const auto before = encoded(config.view(true));
    for (auto &edit : edits) assert(strcmp(edit(), error) == 0);
    assert(encoded(config.view(true)) == before && encoded(*config.applied()) == applied);
    assert(flash.mutations == writes);
  };
  guarded("NO_DRAFT");
  assert(!config.beginDraft()); config.setDisarmed(false); guarded("ARMED");
  config.setDisarmed(true); config.discardDraft();
  flight::RunController run; assert(!run.begin(flight::RunMode::Estimates, config));
  guarded("BUSY"); run.disconnect();
  assert(!config.beginDraft());
  auto rejected = [&](const std::function<const char *()> &edit, const char *error) {
    const auto before = encoded(config.view(true)); assert(strcmp(edit(), error) == 0);
    assert(encoded(config.view(true)) == before);
  };
  rejected([&]() { return config.setPyro(0, 100); }, "INVALID_ARGUMENT");
  rejected([&]() { return config.setPyro(5, 100); }, "INVALID_ARGUMENT");
  rejected([&]() { return config.setPyro(1, limits.maxPyroPulseMs + 1); }, "INVALID_ARGUMENT");
  rejected([&]() { return config.setServo(1, 65536, 2000); }, "INVALID_ARGUMENT");
  rejected([&]() { return config.setServo(1, 999, 2000); }, "INVALID_ARGUMENT");
  rejected([&]() { return config.addRule(10, "Duplicate"); }, "DUPLICATE_RULE_ID");
  rejected([&]() { return config.addRule(0, "Invalid"); }, "INVALID_ARGUMENT");
  rejected([&]() { return config.renameRule(10, std::string(48, 'x').c_str()); }, "INVALID_ARGUMENT");
  rejected([&]() { return config.setRuleDelay(10, limits.maxTimingMs + 1); }, "INVALID_ARGUMENT");
  rejected([&]() { return config.replaceCondition(10, 8, condition); }, "INVALID_ARGUMENT");
  rejected([&]() { return config.removeCondition(10, 8); }, "INVALID_ARGUMENT");
  rejected([&]() { return config.assignOutput(20, OutputKind::Pyro, 1); }, "OUTPUT_ALREADY_ASSIGNED");
  rejected([&]() { return config.assignOutput(10, static_cast<OutputKind>(255), 3); }, "INVALID_ARGUMENT");
  rejected([&]() { return config.assignOutput(10, OutputKind::Pyro, 3, limits.maxTimingMs + 1); }, "INVALID_ARGUMENT");
  condition.kind = recovery::ConditionKind::AltitudeComparison; condition.value = std::numeric_limits<float>::quiet_NaN();
  rejected([&]() { return config.addCondition(10, condition); }, "INVALID_ARGUMENT");
  condition = {}; condition.kind = recovery::ConditionKind::TimeSinceRule; condition.reference = 10;
  assert(!config.addCondition(20, condition));
  recovery::RuleId referencing = 0;
  rejected([&]() { return config.removeRule(10, &referencing); }, "RULE_REFERENCED"); assert(referencing == 20);
  assert(!config.removeCondition(20, 2)); condition = {};
  assert(!config.addCondition(10, condition)); assert(!config.replaceCondition(10, 1, condition));
  assert(!config.removeCondition(10, 1));
  assert(!config.assignOutput(10, OutputKind::Pyro, 3));
  assert(!config.unassignOutput(10, OutputKind::Pyro, 3));
  assert(!config.addRule(30, "Temporary")); assert(!config.removeRule(30));
  assert(!config.renameRule(10, config.view(true).rules[0].name.data()));
  assert(strcmp(config.view(true).rules[0].name.data(), "Example apogee") == 0);
  assert(!config.renameRule(10, "Service edit")); assert(!config.setRuleDelay(10, 20));
  assert(!config.setPyro(1, 150)); assert(!config.setServo(1, 1100, 1900));
  assert(config.validateDraft().ok());
  assert(encoded(*config.applied()) == applied && flash.mutations == writes);
  assert(!config.apply());
  ConfigurationService reboot(store, limits); reboot.boot(true);
  assert(reboot.applied() && reboot.applied()->pyros[0].pulseMs == 150);
  assert(strcmp(reboot.applied()->rules[0].name.data(), "Service edit") == 0);
  // Mounting survives reboot and rule saves; invalid/corrupt settings cannot be armed silently.
  assert(!config.orientationReady());
  for (uint8_t axis=0; axis<6; ++axis) {
    assert(!config.setOrientation(axis));
    ConfigurationService mounted(store,limits); mounted.boot(true);
    assert(mounted.orientationReady() && mounted.upAxis()==axis && mounted.configValid());
  }
  auto bytes=flash.bytes;
  assert(!strcmp(config.setOrientation(6),"INVALID_ARGUMENT") && flash.bytes==bytes);
  config.setDisarmed(false); assert(!strcmp(config.setOrientation(0),"ARMED") && flash.bytes==bytes); config.setDisarmed(true);
  assert(!config.beginDraft()); assert(!strcmp(config.setOrientation(0),"DRAFT_ACTIVE") && flash.bytes==bytes); config.discardDraft();
  assert(config.acquireRun()); assert(!strcmp(config.setOrientation(0),"BUSY") && flash.bytes==bytes); config.releaseRun();
  flash.bytes[storage::ConfigurationStore::Begin+storage::FlashLayout::SectorBytes+5]^=1;
  ConfigurationService damaged(store,limits); damaged.boot(true); assert(!damaged.orientationReady() && damaged.configValid());
  assert(!damaged.setOrientation(3) && damaged.orientationReady());
  recovery::StateMachineSettings machine; machine.bistage=true; machine.minimumCoastVelocity=35.5f;
  assert(!damaged.setStateMachine(machine));
  ConfigurationService machineReboot(store,limits); machineReboot.boot(true);
  assert(machineReboot.stateMachine().bistage && machineReboot.stateMachine().minimumCoastVelocity==35.5f);
  auto beforeMachine=flash.bytes;
  machine.minimumCoastVelocity=-1; assert(!strcmp(damaged.setStateMachine(machine),"INVALID_ARGUMENT") && flash.bytes==beforeMachine);
  assert(!damaged.factoryReset());
  ConfigurationService reset(store,limits); reset.boot(true);
  assert(!reset.configValid() && !reset.orientationReady() && !reset.stateMachine().bistage);
  puts("Configuration service tests passed: direct edits, guards, rejection atomicity and persistence.");
}
} // namespace test_configuration_service

int main() {
  std::puts("  recovery_configuration"); std::fflush(stdout);
  test_recovery_configuration::run();
  std::puts("  recovery_rule_engine"); std::fflush(stdout);
  test_recovery_rule_engine::run();
  std::puts("  configuration_service"); std::fflush(stdout);
  test_configuration_service::run();
}
