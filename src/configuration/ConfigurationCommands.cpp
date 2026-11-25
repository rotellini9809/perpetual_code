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
bool parseCondition(const Tokens &t, unsigned at, recovery::Condition &condition) {
  using namespace recovery;
  const auto remaining = t.count - at;
  if (t.is(at, "apogee") && remaining == 1) { condition.kind = ConditionKind::ApogeeDetected; return true; }
  if (t.is(at, "phase") && remaining == 2) {
    condition.kind = ConditionKind::Phase;
    if (t.is(at + 1, "ascent")) condition.phases = BoostMask | CoastMask | Boost2Mask | Coast2Mask;
    else if (t.is(at + 1, "descent")) condition.phases = DescentMask;
    else if (t.is(at + 1, "airborne")) condition.phases = AirborneMask;
    else { uint32_t mask; if (!unsignedNumber(t[at + 1], mask) || mask > 255) return false; condition.phases = uint8_t(mask); }
    return true;
  }
  if ((t.is(at, "altitude") || t.is(at, "speed")) && remaining == 3) {
    const bool speed = t.is(at, "speed");
    condition.kind = speed ? ConditionKind::AbsoluteVelocityComparison : ConditionKind::AltitudeComparison;
    if (t.is(at + 1, "<") && !speed) condition.comparison = Comparison::Less;
    else if (t.is(at + 1, ">") && !speed) condition.comparison = Comparison::Greater;
    else if (t.is(at + 1, "<=") && speed) condition.comparison = Comparison::LessEqual;
    else if (t.is(at + 1, ">=") && speed) condition.comparison = Comparison::GreaterEqual;
    else return false;
    return finiteNumber(t[at + 2], condition.value) && (!speed || condition.value >= 0);
  }
  if ((t.is(at, "altitude_range") || t.is(at, "speed_range")) && remaining == 3) {
    condition.kind = t.is(at, "speed_range") ? ConditionKind::AbsoluteVelocityRange : ConditionKind::AltitudeRange;
    return finiteNumber(t[at + 1], condition.value) && finiteNumber(t[at + 2], condition.upper) &&
           (condition.kind != ConditionKind::AbsoluteVelocityRange || condition.value >= 0);
  }
  if ((t.is(at, "since_launch_ms") || t.is(at, "since_apogee_ms")) && remaining == 2) {
    condition.kind = t.is(at, "since_launch_ms") ? ConditionKind::TimeSinceLaunch : ConditionKind::TimeSinceApogee;
    return unsignedNumber(t[at + 1], condition.elapsedMs);
  }
  if (t.is(at, "since_rule_ms") && remaining == 3) {
    condition.kind = ConditionKind::TimeSinceRule;
    return unsignedNumber(t[at + 1], condition.reference) && unsignedNumber(t[at + 2], condition.elapsedMs);
  }
  return false;
}
void appendCondition(Json &data,const recovery::Condition &condition) {
  data.add(",\"kind\":%u,\"comparison\":%u,\"phase_mask\":%u,\"value\":", unsigned(condition.kind), unsigned(condition.comparison), unsigned(condition.phases));
  if (std::isfinite(condition.value)) data.add("%.9g", double(condition.value)); else data.add("null");
  data.add(",\"value_valid\":%s,\"upper\":", std::isfinite(condition.value) ? "true" : "false");
  if (std::isfinite(condition.upper)) data.add("%.9g", double(condition.upper)); else data.add("null");
  data.add(",\"upper_valid\":%s,\"elapsed_ms\":%lu,\"reference\":%lu", std::isfinite(condition.upper) ? "true" : "false",
           static_cast<unsigned long>(condition.elapsedMs), static_cast<unsigned long>(condition.reference));
}

}

bool ConfigurationCommands::dispatch(const char *command, const char *arguments, bool hasId, uint32_t id,
                                     char *output, std::size_t capacity) {
  const auto &limits = service_.limits();
  const auto &appliedView = service_.view(false);
  const auto &draftView = service_.view(true);
  const bool drafting = service_.hasDraft(), appliedValid = service_.configValid();
  const bool config = strcmp(command, "config") == 0;
  const bool rule = strcmp(command, "rule") == 0;
  const bool condition = strcmp(command, "condition") == 0;
  const bool assignment = strcmp(command, "output") == 0;
  const bool pyro = strcmp(command, "pyro") == 0;
  const bool servo = strcmp(command, "servo") == 0;
  const bool airbrake = strcmp(command, "airbrake") == 0;
  const bool capabilities = strcmp(command, "capabilities") == 0;
  if (!config && !rule && !condition && !assignment && !pyro && !servo && !airbrake && !capabilities) return false;
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
             limits.minServoUs, limits.maxServoUs, static_cast<unsigned long>(limits.maxPyroPulseMs),
             static_cast<unsigned long>(limits.maxTimingMs),physicalActions_ ? "true" : "false");
    data.add(",\"physical_simulation\":%s",physicalSimulation_ ? "true" : "false");
    data.add(R"(,"output_delays":true,"airbrakes":true)");
    data.add(",\"pyro_pulse_ms\":%lu,\"pyro_enable_settle_ms\":%lu,\"rule_timing\":\"delay\"",
        (unsigned long)recovery::PyroPulseMs, (unsigned long)recovery::PyroEnableSettleMs);
    return finish();
  }
  if (config && t.is(0,"airbrakes")) {
    if (t.count==2 && (t.is(1,"on") || t.is(1,"off"))) error=service_.setAirbrakes(t.is(1,"on"));
    else if (t.count!=1) error="INVALID_ARGUMENT";
    if (!error && t.count>1) data.add(R"(,"persisted":true,"verified":true)");
    data.add(R"(,"enabled":%s,"ready":%s)",service_.airbrakesEnabled() ? "true" : "false",service_.airbrakesReady() ? "true" : "false");
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
    const auto &plan = draft ? draftView : appliedView;
    data.add(",\"view\":\"%s\",\"available\":%s,\"draft_active\":%s,\"store_error\":\"%s\",\"rule_ids\":[",
             draft ? "draft" : "applied", (draft ? drafting : appliedValid) ? "true" : "false", drafting ? "true" : "false", configError());
    if (draft ? drafting : appliedValid)
      for (std::size_t i = 0; i < plan.ruleCount; ++i) data.add("%s%lu", i ? "," : "", static_cast<unsigned long>(plan.rules[i].id));
    data.add("]"); return finish();
  }
  // Detailed inspection selects draft while editing, otherwise applied settings.
  const bool inspection = t.is(0, "show") || (rule && t.is(0, "list"));
  if (inspection && (drafting || appliedValid)) {
    const auto &plan = drafting ? draftView : appliedView;
    data.add(",\"view\":\"%s\"", drafting ? "draft" : "applied");
    uint32_t number;
    if ((pyro || servo) && t.count == 2 && unsignedNumber(t[1], number) && number >= 1 && number <= 4) {
      data.add(",\"channel\":%lu", static_cast<unsigned long>(number));
      if (pyro) data.add(",\"pulse_ms\":%lu", static_cast<unsigned long>(plan.pyros[number - 1].pulseMs));
      else {
        const auto &servoSettings = plan.servos[number - 1];
        data.add(",\"calibrated\":%s,\"initial_us\":%u,\"final_us\":%u", servoSettings.calibrated ? "true" : "false", servoSettings.initialUs, servoSettings.finalUs);
      }
      return finish();
    }
    if (airbrake) {
      const auto &airbrakeRule=plan.airbrakes;
      if (t.count==1) { data.add(R"(,"channel_mask":%u,"conditions":%u,"logic":"and")",unsigned(airbrakeRule.channelMask),unsigned(airbrakeRule.conditionCount)); return finish(); }
      uint32_t index;
      if (t.count==2 && unsignedNumber(t[1],index) && index<airbrakeRule.conditionCount) {
        data.add(R"(,"index":%lu)",(unsigned long)index); appendCondition(data,airbrakeRule.conditions[index]); return finish();
      }
    }
    if (rule && t.is(0, "list") && t.count == 1) {
      data.add(",\"rule_ids\":[");
      for (std::size_t i = 0; i < plan.ruleCount; ++i) data.add("%s%lu", i ? "," : "", static_cast<unsigned long>(plan.rules[i].id));
      data.add("]"); return finish();
    }
    if (t.count >= 2 && unsignedNumber(t[1], number)) {
      for (std::size_t i = 0; i < plan.ruleCount; ++i) {
        const auto &selectedRule = plan.rules[i]; if (selectedRule.id != number) continue;
        if (rule && t.count == 2) {
          data.add(",\"rule_id\":%lu,\"name\":", static_cast<unsigned long>(selectedRule.id)); data.quoted(selectedRule.name.data());
          data.add(",\"delay_ms\":%lu,\"conditions\":%u,\"outputs\":%u", static_cast<unsigned long>(selectedRule.delayMs), unsigned(selectedRule.conditionCount), unsigned(selectedRule.assignmentCount));
          return finish();
        }
        uint32_t index;
        if (t.count == 3 && unsignedNumber(t[2], index)) {
          data.add(",\"rule_id\":%lu,\"index\":%lu", static_cast<unsigned long>(selectedRule.id), static_cast<unsigned long>(index));
          if (condition && index < selectedRule.conditionCount) {
            const auto &selectedCondition = selectedRule.conditions[index];
            appendCondition(data,selectedCondition);
            return finish();
          }
          if (assignment && index < selectedRule.assignmentCount) {
            const auto &selectedAssignment = selectedRule.assignments[index];
            data.add(",\"kind\":\"%s\",\"channel\":%u,\"delay_ms\":%lu", selectedAssignment.kind == recovery::OutputKind::Pyro ? "pyro" : "servo", selectedAssignment.channel, (unsigned long)selectedAssignment.delayMs);
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
  if (!drafting) { error = "NO_DRAFT"; return finish(); }
  if (config && t.count == 1 && (t.is(0, "validate") || t.is(0, "apply"))) {
    const auto validation = service_.validateDraft();
    if (!validation.ok()) {
      error = "CONFIG_INVALID";
      data.add(",\"validation\":\"%s\",\"field\":%u,\"rule_id\":%lu,\"index\":%u", recovery::errorName(validation.error), unsigned(validation.field), static_cast<unsigned long>(validation.rule), unsigned(validation.index));
      return finish();
    }
    if (t.is(0, "validate")) { data.add(",\"valid\":true"); return finish(); }
    error = service_.apply();
    if (error) { data.add(",\"store_error\":\"%s\"", configError()); return finish(); }
    data.add(",\"applied\":true,\"persisted\":true,\"verified\":true"); return finish();
  }
  // Numeric arguments of the edit commands below. target is the rule ID, or the channel for
  // `pyro set`/`servo set`; firstNumber/secondNumber are the values that follow (mask, index,
  // pulse, delay or channel, depending on the command).
  uint32_t target, firstNumber, secondNumber;
  error = "INVALID_ARGUMENT";
  bool servoChanged = false;
  if (airbrake) {
    recovery::Condition value;
    if (t.is(0,"channels") && t.count==2 && unsignedNumber(t[1],firstNumber)) error=service_.setAirbrakeChannels(firstNumber);
    else if (t.is(0,"add") && t.count>=2 && parseCondition(t,1,value)) error=service_.addAirbrakeCondition(value);
    else if (t.is(0,"replace") && t.count>=3 && unsignedNumber(t[1],firstNumber) && parseCondition(t,2,value)) error=service_.replaceAirbrakeCondition(firstNumber,value);
    else if (t.is(0,"remove") && t.count==2 && unsignedNumber(t[1],firstNumber)) error=service_.removeAirbrakeCondition(firstNumber);
  } else if ((pyro || servo) && t.is(0, "set") && unsignedNumber(t[1], target)) {
    if (pyro && t.count == 3 && unsignedNumber(t[2], firstNumber)) error = service_.setPyro(target, firstNumber);
    if (servo && t.count == 4 && unsignedNumber(t[2], firstNumber) && unsignedNumber(t[3], secondNumber)) {
      error = service_.setServo(target, firstNumber, secondNumber); servoChanged = !error;
    }
  } else if (rule && t.count >= 2 && unsignedNumber(t[1], target)) {
    if (t.is(0, "add") && t.count == 3) error = service_.addRule(target, t[2]);
    else if (t.is(0, "rename") && t.count == 3) error = service_.renameRule(target, t[2]);
    else if (t.is(0, "delay") && t.count == 3 && unsignedNumber(t[2], firstNumber)) error = service_.setRuleDelay(target, firstNumber);
    else if (t.is(0, "remove") && t.count == 2) {
      recovery::RuleId referencing = 0;
      error = service_.removeRule(target, &referencing);
      if (referencing) data.add(",\"referencing_rule_id\":%lu", static_cast<unsigned long>(referencing));
    }
  } else if (condition && t.count >= 2 && unsignedNumber(t[1], target)) {
    recovery::Condition value;
    if (t.is(0, "add") && t.count >= 3 && parseCondition(t, 2, value)) error = service_.addCondition(target, value);
    else if (t.is(0, "replace") && t.count >= 4 && unsignedNumber(t[2], firstNumber) && parseCondition(t, 3, value))
      error = service_.replaceCondition(target, firstNumber, value);
    else if (t.is(0, "remove") && t.count == 3 && unsignedNumber(t[2], firstNumber)) error = service_.removeCondition(target, firstNumber);
  } else if (assignment && t.count >= 4 && unsignedNumber(t[1], target)) {
    recovery::OutputKind kind;
    if (outputKind(t[2], kind) && unsignedNumber(t[3], firstNumber)) {
      if (t.is(0, "assign") && (t.count == 4 || (t.count == 5 && unsignedNumber(t[4], secondNumber))))
        error = service_.assignOutput(target, kind, firstNumber, t.count == 5 ? secondNumber : 0);
      else if (t.is(0, "unassign") && t.count == 4) error = service_.unassignOutput(target, kind, firstNumber);
    }
  }
  if (!error) {
    data.add(",\"draft_changed\":true");
    if (servoChanged) data.add(",\"hardware_moved\":false");
  }
  return finish();
}
