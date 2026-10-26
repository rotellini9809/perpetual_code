#include "configuration/ConfigurationCommands.h"
#include "communication/CommandProtocol.h"
#include "configuration/ActuatorPolicy.h"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
using protocol::Tokens;
using protocol::Json;
using protocol::unsignedNumber;
using protocol::finiteNumber;
bool outputKind(const char *text, recovery::OutputKind &kind) {
  if (strcmp(text, "pyro") == 0) { kind = recovery::OutputKind::Pyro; return true; }
  if (strcmp(text, "servo") == 0) { kind = recovery::OutputKind::Servo; return true; }
  return false;
}
bool parseCondition(const Tokens &t, unsigned at, recovery::Condition &c) {
  using namespace recovery;
  const auto n = t.count - at;
  if (t.is(at, "apogee") && n == 1) { c.kind = ConditionKind::ApogeeDetected; return true; }
  if (t.is(at, "phase") && n == 2) {
    c.kind = ConditionKind::Phase;
    if (t.is(at + 1, "ascent")) c.phases = BoostMask | CoastMask | Boost2Mask | Coast2Mask;
    else if (t.is(at + 1, "descent")) c.phases = DescentMask;
    else if (t.is(at + 1, "airborne")) c.phases = AirborneMask;
    else { uint32_t mask; if (!unsignedNumber(t[at + 1], mask) || mask > 255) return false; c.phases = uint8_t(mask); }
    return true;
  }
  if ((t.is(at, "altitude") || t.is(at, "speed")) && n == 3) {
    const bool speed = t.is(at, "speed");
    c.kind = speed ? ConditionKind::AbsoluteVelocityComparison : ConditionKind::AltitudeComparison;
    if (t.is(at + 1, "<") && !speed) c.comparison = Comparison::Less;
    else if (t.is(at + 1, ">") && !speed) c.comparison = Comparison::Greater;
    else if (t.is(at + 1, "<=") && speed) c.comparison = Comparison::LessEqual;
    else if (t.is(at + 1, ">=") && speed) c.comparison = Comparison::GreaterEqual;
    else return false;
    return finiteNumber(t[at + 2], c.value) && (!speed || c.value >= 0);
  }
  if ((t.is(at, "altitude_range") || t.is(at, "speed_range")) && n == 3) {
    c.kind = t.is(at, "speed_range") ? ConditionKind::AbsoluteVelocityRange : ConditionKind::AltitudeRange;
    return finiteNumber(t[at + 1], c.value) && finiteNumber(t[at + 2], c.upper) &&
           (c.kind != ConditionKind::AbsoluteVelocityRange || c.value >= 0);
  }
  if ((t.is(at, "since_launch_ms") || t.is(at, "since_apogee_ms")) && n == 2) {
    c.kind = t.is(at, "since_launch_ms") ? ConditionKind::TimeSinceLaunch : ConditionKind::TimeSinceApogee;
    return unsignedNumber(t[at + 1], c.elapsedMs);
  }
  if (t.is(at, "since_rule_ms") && n == 3) {
    c.kind = ConditionKind::TimeSinceRule;
    return unsignedNumber(t[at + 1], c.reference) && unsignedNumber(t[at + 2], c.elapsedMs);
  }
  return false;
}
}

bool ConfigurationCommands::dispatch(const char *command, const char *arguments, bool hasId, uint32_t id,
                                     char *output, std::size_t capacity) {
  const auto &limits_ = service_.limits();
  const auto &applied_ = service_.view(false);
  const auto &draftView = service_.view(true);
  const bool drafting_ = service_.hasDraft(), appliedValid_ = service_.configValid();
  const bool config = strcmp(command, "config") == 0;
  const bool rule = strcmp(command, "rule") == 0;
  const bool condition = strcmp(command, "condition") == 0;
  const bool assignment = strcmp(command, "output") == 0;
  const bool pyro = strcmp(command, "pyro") == 0;
  const bool servo = strcmp(command, "servo") == 0;
  const bool capabilities = strcmp(command, "capabilities") == 0;
  if (!config && !rule && !condition && !assignment && !pyro && !servo && !capabilities) return false;
  Tokens t(arguments); Json data;
  const char *error = nullptr;
  auto finish = [&]() {
    return protocol::finish(command, data, error, hasId, id, output, capacity, true);
  };
  if (!t.valid) { error = "INVALID_ARGUMENT"; return finish(); }
  if (capabilities && t.count == 0) {
    data.add(",\"max_rules\":%u,\"max_conditions\":%u,\"max_assignments\":%u,\"name_bytes\":%u,"
             "\"servo_min_us\":%u,\"servo_max_us\":%u,\"max_pyro_ms\":%lu,\"max_timing_ms\":%lu,\"physical_actions\":%s",
             unsigned(recovery::MaxRules), unsigned(recovery::MaxConditions), unsigned(recovery::MaxAssignments), unsigned(recovery::MaxNameBytes - 1),
             limits_.minServoUs, limits_.maxServoUs, static_cast<unsigned long>(limits_.maxPyroPulseMs),
             static_cast<unsigned long>(limits_.maxTimingMs),physicalActions_ ? "true" : "false");
    data.add(",\"physical_simulation\":%s",physicalSimulation_ ? "true" : "false");
    data.add(",\"output_delays\":true");
    data.add(",\"pyro_pulse_ms\":%lu,\"pyro_enable_settle_ms\":%lu,\"rule_timing\":\"delay\"",
        (unsigned long)recovery::PyroPulseMs, (unsigned long)recovery::PyroEnableSettleMs);
    return finish();
  }
  if (config && t.is(0,"machine")) {
    recovery::StateMachineSettings requested;
    if (t.count==2 && t.is(1,"normal")) error=service_.setStateMachine(requested);
    else if (t.count==3 && t.is(1,"bistage") && finiteNumber(t[2],requested.minimumCoastVelocity)) {
      requested.bistage=true; error=service_.setStateMachine(requested);
    } else if (t.count!=1) error="INVALID_ARGUMENT";
    if (!error && t.count>1) data.add(",\"persisted\":true,\"verified\":true");
    const auto &machine=service_.stateMachine();
    data.add(",\"machine_ready\":%s",service_.machineReady() ? "true" : "false");
    data.add(",\"bistage\":%s",machine.bistage ? "true" : "false"); data.number("minimum_coast_velocity",machine.minimumCoastVelocity);
    return finish();
  }
  if (config && t.is(0,"orientation")) {
    static const char *axes[]={"+X","-X","+Y","-Y","+Z","-Z"};
    if (t.count==2) {
      uint8_t axis=0; while (axis<6 && !t.is(1,axes[axis])) ++axis;
      error=service_.setOrientation(axis);
      if (!error) data.add(",\"persisted\":true,\"verified\":true");
    } else if (t.count!=1) error="INVALID_ARGUMENT";
    data.add(",\"up_axis\":");
    if (service_.orientationReady()) data.quoted(axes[service_.upAxis()]); else data.add("null");
    data.add(",\"orientation_ready\":%s",service_.orientationReady() ? "true" : "false");
    return finish();
  }
  if (config && t.count == 1 && (t.is(0, "show") || t.is(0, "draft"))) {
    const bool draft = t.is(0, "draft");
    const auto &plan = draft ? draftView : applied_;
    data.add(",\"view\":\"%s\",\"available\":%s,\"draft_active\":%s,\"store_error\":\"%s\",\"rule_ids\":[",
             draft ? "draft" : "applied", (draft ? drafting_ : appliedValid_) ? "true" : "false", drafting_ ? "true" : "false", configError());
    if (draft ? drafting_ : appliedValid_)
      for (std::size_t i = 0; i < plan.ruleCount; ++i) data.add("%s%lu", i ? "," : "", static_cast<unsigned long>(plan.rules[i].id));
    data.add("]"); return finish();
  }
  // Detailed inspection selects draft while editing, otherwise applied settings.
  const bool inspection = t.is(0, "show") || (rule && t.is(0, "list"));
  if (inspection && (drafting_ || appliedValid_)) {
    const auto &plan = drafting_ ? draftView : applied_;
    data.add(",\"view\":\"%s\"", drafting_ ? "draft" : "applied");
    uint32_t number;
    if ((pyro || servo) && t.count == 2 && unsignedNumber(t[1], number) && number >= 1 && number <= 4) {
      data.add(",\"channel\":%lu", static_cast<unsigned long>(number));
      if (pyro) data.add(",\"pulse_ms\":%lu", static_cast<unsigned long>(plan.pyros[number - 1].pulseMs));
      else {
        const auto &s = plan.servos[number - 1];
        data.add(",\"calibrated\":%s,\"initial_us\":%u,\"final_us\":%u", s.calibrated ? "true" : "false", s.initialUs, s.finalUs);
      }
      return finish();
    }
    if (rule && t.is(0, "list") && t.count == 1) {
      data.add(",\"rule_ids\":[");
      for (std::size_t i = 0; i < plan.ruleCount; ++i) data.add("%s%lu", i ? "," : "", static_cast<unsigned long>(plan.rules[i].id));
      data.add("]"); return finish();
    }
    if (t.count >= 2 && unsignedNumber(t[1], number)) {
      for (std::size_t i = 0; i < plan.ruleCount; ++i) {
        const auto &r = plan.rules[i]; if (r.id != number) continue;
        if (rule && t.count == 2) {
          data.add(",\"rule_id\":%lu,\"name\":", static_cast<unsigned long>(r.id)); data.quoted(r.name.data());
          data.add(",\"delay_ms\":%lu,\"conditions\":%u,\"outputs\":%u", static_cast<unsigned long>(r.delayMs), unsigned(r.conditionCount), unsigned(r.assignmentCount));
          return finish();
        }
        uint32_t index;
        if (t.count == 3 && unsignedNumber(t[2], index)) {
          data.add(",\"rule_id\":%lu,\"index\":%lu", static_cast<unsigned long>(r.id), static_cast<unsigned long>(index));
          if (condition && index < r.conditionCount) {
            const auto &c = r.conditions[index];
            data.add(",\"kind\":%u,\"comparison\":%u,\"phase_mask\":%u,\"value\":", unsigned(c.kind), unsigned(c.comparison), unsigned(c.phases));
            if (std::isfinite(c.value)) data.add("%.9g", double(c.value)); else data.add("null");
            data.add(",\"value_valid\":%s,\"upper\":", std::isfinite(c.value) ? "true" : "false");
            if (std::isfinite(c.upper)) data.add("%.9g", double(c.upper)); else data.add("null");
            data.add(",\"upper_valid\":%s,\"elapsed_ms\":%lu,\"reference\":%lu", std::isfinite(c.upper) ? "true" : "false",
                     static_cast<unsigned long>(c.elapsedMs), static_cast<unsigned long>(c.reference));
            return finish();
          }
          if (assignment && index < r.assignmentCount) {
            const auto &a = r.assignments[index];
            data.add(",\"kind\":\"%s\",\"channel\":%u,\"delay_ms\":%lu", a.kind == recovery::OutputKind::Pyro ? "pyro" : "servo", a.channel, (unsigned long)a.delayMs);
            return finish();
          }
        }
      }
    }
    error = "INVALID_ARGUMENT"; return finish();
  }
  if (service_.runLocked()) { error = "BUSY"; return finish(); }
  if (!service_.disarmed()) { error = "ARMED"; return finish(); }
  if (config && t.count == 1 && t.is(0, "begin")) {
    error = service_.beginDraft();
    if (!error) data.add(",\"draft_active\":true");
    return finish();
  }
  if (config && t.count == 1 && t.is(0, "discard")) {
    error = service_.discardDraft(); if (!error) data.add(",\"draft_active\":false"); return finish();
  }
  if (config && t.count==1 && t.is(0,"reset")) {
    error=service_.factoryReset();
    if (!error) data.add(",\"reset\":true,\"persisted\":true,\"verified\":true");
    return finish();
  }
  if (!drafting_) { error = "NO_DRAFT"; return finish(); }
  if (config && t.count == 1 && (t.is(0, "validate") || t.is(0, "apply"))) {
    const auto v = service_.validateDraft();
    if (!v.ok()) {
      error = "CONFIG_INVALID";
      data.add(",\"validation\":\"%s\",\"field\":%u,\"rule_id\":%lu,\"index\":%u", recovery::errorName(v.error), unsigned(v.field), static_cast<unsigned long>(v.rule), unsigned(v.index));
      return finish();
    }
    if (t.is(0, "validate")) { data.add(",\"valid\":true"); return finish(); }
    error = service_.apply();
    if (error) { data.add(",\"store_error\":\"%s\"", configError()); return finish(); }
    data.add(",\"applied\":true,\"persisted\":true,\"verified\":true"); return finish();
  }
  uint32_t ruleId, a, b;
  error = "INVALID_ARGUMENT";
  bool servoChanged = false;
  if ((pyro || servo) && t.is(0, "set") && unsignedNumber(t[1], ruleId)) {
    if (pyro && t.count == 3 && unsignedNumber(t[2], a)) error = service_.setPyro(ruleId, a);
    if (servo && t.count == 4 && unsignedNumber(t[2], a) && unsignedNumber(t[3], b)) {
      error = service_.setServo(ruleId, a, b); servoChanged = !error;
    }
  } else if (rule && t.count >= 2 && unsignedNumber(t[1], ruleId)) {
    if (t.is(0, "add") && t.count == 3) error = service_.addRule(ruleId, t[2]);
    else if (t.is(0, "rename") && t.count == 3) error = service_.renameRule(ruleId, t[2]);
    else if (t.is(0, "delay") && t.count == 3 && unsignedNumber(t[2], a)) error = service_.setRuleDelay(ruleId, a);
    else if (t.is(0, "remove") && t.count == 2) {
      recovery::RuleId referencing = 0;
      error = service_.removeRule(ruleId, &referencing);
      if (referencing) data.add(",\"referencing_rule_id\":%lu", static_cast<unsigned long>(referencing));
    }
  } else if (condition && t.count >= 2 && unsignedNumber(t[1], ruleId)) {
    recovery::Condition value;
    if (t.is(0, "add") && t.count >= 3 && parseCondition(t, 2, value)) error = service_.addCondition(ruleId, value);
    else if (t.is(0, "replace") && t.count >= 4 && unsignedNumber(t[2], a) && parseCondition(t, 3, value))
      error = service_.replaceCondition(ruleId, a, value);
    else if (t.is(0, "remove") && t.count == 3 && unsignedNumber(t[2], a)) error = service_.removeCondition(ruleId, a);
  } else if (assignment && t.count >= 4 && unsignedNumber(t[1], ruleId)) {
    recovery::OutputKind kind;
    if (outputKind(t[2], kind) && unsignedNumber(t[3], a)) {
      if (t.is(0, "assign") && (t.count == 4 || (t.count == 5 && unsignedNumber(t[4], b))))
        error = service_.assignOutput(ruleId, kind, a, t.count == 5 ? b : 0);
      else if (t.is(0, "unassign") && t.count == 4) error = service_.unassignOutput(ruleId, kind, a);
    }
  }
  if (!error) {
    data.add(",\"draft_changed\":true");
    if (servoChanged) data.add(",\"hardware_moved\":false");
  }
  return finish();
}
