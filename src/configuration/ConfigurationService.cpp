#include "configuration/StateMachineValidation.h"
#include "configuration/ConfigurationService.h"
#include "configuration/ActuatorPolicy.h"
#include <cstring>
#include <cmath>
void ConfigurationService::boot(bool storageReady) {
  if (runLocked_) return;
  storageReady_ = storageReady; drafting_ = false; appliedValid_ = false;
  upAxis_=0; orientationReady_=false; machine_={}; machineReady_=false; airbrakesEnabled_=false; airbrakesReady_=false;
  if (!storageReady) { loadError_ = storage::ConfigError::Io; return; }
  appliedValid_ = store_.load(applied_, limits_); loadError_ = store_.error();
  orientationReady_=store_.loadOrientation(upAxis_);
  machineReady_=store_.loadStateMachine(machine_);
  airbrakesReady_=store_.loadAirbrakes(airbrakesEnabled_);
}

const char *ConfigurationService::factoryReset() {
  if (runLocked_) return "BUSY";
  if (!disarmed_) return "ARMED";
  if (!storageReady_) return "STORAGE_ERROR";
  appliedValid_=false; drafting_=false; orientationReady_=false; machineReady_=false;
  if (!store_.clear(disarmed_)) { loadError_=store_.error(); storageReady_=false; return "STORAGE_ERROR"; }
  upAxis_=0; orientationReady_=false;
  machine_={}; machineReady_=true; airbrakesEnabled_=false; airbrakesReady_=true;
  applied_={}; draft_={}; loadError_=storage::ConfigError::Missing;
  return nullptr;
}

const char *ConfigurationService::setAirbrakes(bool enabled) {
  if (runLocked_) return "BUSY";
  if (!disarmed_) return "ARMED";
  if (drafting_) return "DRAFT_ACTIVE";
  if (!storageReady_) return "STORAGE_ERROR";
  bool verified=false; airbrakesReady_=false;
  if (!store_.saveAirbrakes(enabled,disarmed_) || !store_.loadAirbrakes(verified) || verified!=enabled) return "STORAGE_ERROR";
  airbrakesEnabled_=verified; airbrakesReady_=true; return nullptr;
}
const char *ConfigurationService::setStateMachine(const recovery::StateMachineSettings &settings) {
  if (runLocked_) return "BUSY";
  if (!disarmed_) return "ARMED";
  if (drafting_) return "DRAFT_ACTIVE";
  if (!recovery::validStateMachineSettings(settings)) return "INVALID_ARGUMENT";
  if (!storageReady_) return "STORAGE_ERROR";
  machineReady_=false; recovery::StateMachineSettings verified;
  if (!store_.saveStateMachine(settings,disarmed_) || !store_.loadStateMachine(verified) ||
      verified.bistage!=settings.bistage || verified.minimumCoastVelocity!=settings.minimumCoastVelocity) return "STORAGE_ERROR";
  machine_=verified; machineReady_=true; return nullptr;
}
const char *ConfigurationService::setOrientation(uint8_t axis) {
  if (runLocked_) return "BUSY";
  if (!disarmed_) return "ARMED";
  if (drafting_) return "DRAFT_ACTIVE";
  if (axis>=6) return "INVALID_ARGUMENT";
  if (!storageReady_) return "STORAGE_ERROR";
  orientationReady_=false;
  uint8_t verified=255;
  if (!store_.saveOrientation(axis,disarmed_) || !store_.loadOrientation(verified) || verified!=axis) return "STORAGE_ERROR";
  upAxis_=axis; orientationReady_=true; return nullptr;
}

const char *ConfigurationService::beginDraft() {
  if (runLocked_) return "BUSY";
  if (!disarmed_) return "ARMED";
  if (drafting_) return "DRAFT_ACTIVE";
  static const recovery::Configuration empty{}; // No temporary plan copy on the core 0 stack.
  if (appliedValid_) draft_ = applied_; else draft_ = empty;
  drafting_ = true; return nullptr;
}
const char *ConfigurationService::discardDraft() {
  if (runLocked_) return "BUSY";
  if (!disarmed_) return "ARMED";
  drafting_ = false; return nullptr;
}
const char *ConfigurationService::apply() {
  if (runLocked_) return "BUSY";
  if (!disarmed_) return "ARMED";
  if (!drafting_) return "NO_DRAFT";
  if (!validateDraft().ok()) return "CONFIG_INVALID";
  if (!storageReady_) return "STORAGE_ERROR";
  // Fail closed: after a failed save the outcome on flash is uncertain until the next boot reads the journal.
  appliedValid_ = false;
  if (!store_.save(draft_, limits_, disarmed_) || !store_.load(verified_, limits_)) {
    loadError_ = store_.error(); storageReady_ = false; return "STORAGE_ERROR";
  }
  applied_ = verified_; appliedValid_ = true; drafting_ = false;
  loadError_ = storage::ConfigError::None; return nullptr;
}

namespace {
bool validName(const char *name) {
  if (!name || !*name || strlen(name) >= recovery::MaxNameBytes) return false;
  for (const char *p = name; *p; ++p)
    if (static_cast<unsigned char>(*p) < 32 || *p == 127) return false;
  return true;
}
bool validOutput(recovery::OutputKind kind, uint32_t channel) {
  return (kind == recovery::OutputKind::Pyro || kind == recovery::OutputKind::Servo) &&
      channel >= 1 && channel <= recovery::ChannelCount;
}
// Drafts may contain incomplete plans, forward references or invalid ranges.
// Complete cross-rule/range/timing validation remains explicit validate/apply.
bool editableCondition(const recovery::Condition &condition) {
  using namespace recovery;
  switch (condition.kind) {
    case ConditionKind::Phase: case ConditionKind::ApogeeDetected:
    case ConditionKind::TimeSinceLaunch: case ConditionKind::TimeSinceApogee:
    case ConditionKind::TimeSinceRule: return true;
    case ConditionKind::AltitudeComparison:
      return std::isfinite(condition.value) && (condition.comparison == Comparison::Less || condition.comparison == Comparison::Greater);
    case ConditionKind::AbsoluteVelocityComparison:
      return std::isfinite(condition.value) && condition.value >= 0 &&
          (condition.comparison == Comparison::LessEqual || condition.comparison == Comparison::GreaterEqual);
    case ConditionKind::AltitudeRange: return std::isfinite(condition.value) && std::isfinite(condition.upper);
    case ConditionKind::AbsoluteVelocityRange:
      return std::isfinite(condition.value) && std::isfinite(condition.upper) && condition.value >= 0;
  }
  return false;
}
}
const char *ConfigurationService::editError() const {
  if (runLocked_) return "BUSY";
  if (!disarmed_) return "ARMED";
  return drafting_ ? nullptr : "NO_DRAFT";
}
recovery::Rule *ConfigurationService::findRule(recovery::RuleId id) {
  for (std::size_t i = 0; i < draft_.ruleCount; ++i)
    if (draft_.rules[i].id == id) return &draft_.rules[i];
  return nullptr;
}
const char *ConfigurationService::setPyro(uint32_t channel, uint32_t pulseMs) {
  if (const char *error = editError()) return error;
  if (channel < 1 || channel > recovery::ChannelCount || !pulseMs || pulseMs > limits_.maxPyroPulseMs) return "INVALID_ARGUMENT";
  draft_.pyros[channel - 1].pulseMs = pulseMs; return nullptr;
}
const char *ConfigurationService::setServo(uint32_t channel, uint32_t initialUs, uint32_t finalUs) {
  if (const char *error = editError()) return error;
  if (channel < 1 || channel > recovery::ChannelCount || initialUs < limits_.minServoUs || initialUs > limits_.maxServoUs ||
      finalUs < limits_.minServoUs || finalUs > limits_.maxServoUs)
    return "INVALID_ARGUMENT";
  auto &servo = draft_.servos[channel - 1]; servo.calibrated = true;
  servo.initialUs = uint16_t(initialUs); servo.finalUs = uint16_t(finalUs); return nullptr;
}
const char *ConfigurationService::addRule(recovery::RuleId id, const char *name) {
  if (const char *error = editError()) return error;
  if (!id || !validName(name)) return "INVALID_ARGUMENT";
  if (findRule(id)) return "DUPLICATE_RULE_ID";
  if (draft_.ruleCount == recovery::MaxRules) return "CAPACITY";
  auto &rule = draft_.rules[draft_.ruleCount++]; rule = {}; rule.id = id; strcpy(rule.name.data(), name); return nullptr;
}
const char *ConfigurationService::renameRule(recovery::RuleId id, const char *name) {
  if (const char *error = editError()) return error;
  auto *rule = findRule(id); if (!rule || !validName(name)) return "INVALID_ARGUMENT";
  std::array<char, recovery::MaxNameBytes> copy{}; strcpy(copy.data(), name);
  rule->name = copy; return nullptr;
}
const char *ConfigurationService::setRuleDelay(recovery::RuleId id, uint32_t delayMs) {
  if (const char *error = editError()) return error;
  auto *rule = findRule(id); if (!rule || delayMs > limits_.maxTimingMs) return "INVALID_ARGUMENT";
  rule->delayMs = delayMs; return nullptr;
}
const char *ConfigurationService::removeRule(recovery::RuleId id, recovery::RuleId *referencingRule) {
  if (referencingRule) *referencingRule = 0;
  if (const char *error = editError()) return error;
  auto *rule = findRule(id); if (!rule) return "INVALID_ARGUMENT";
  for (std::size_t j=0;j<draft_.airbrakes.conditionCount;++j)
    if (draft_.airbrakes.conditions[j].kind==recovery::ConditionKind::TimeSinceRule && draft_.airbrakes.conditions[j].reference==id)
      return "RULE_REFERENCED";
  for (std::size_t i = 0; i < draft_.ruleCount; ++i)
    for (std::size_t j = 0; j < draft_.rules[i].conditionCount; ++j) {
      const auto &condition = draft_.rules[i].conditions[j];
      if (condition.kind == recovery::ConditionKind::TimeSinceRule && condition.reference == id) {
        if (referencingRule) *referencingRule = draft_.rules[i].id;
        return "RULE_REFERENCED";
      }
    }
  const auto index = std::size_t(rule - draft_.rules.data());
  for (std::size_t i = index + 1; i < draft_.ruleCount; ++i) draft_.rules[i - 1] = draft_.rules[i];
  draft_.rules[--draft_.ruleCount] = {}; return nullptr;
}
const char *ConfigurationService::addCondition(recovery::RuleId id, const recovery::Condition &condition) {
  if (const char *error = editError()) return error;
  auto *rule = findRule(id);
  if (!rule || rule->conditionCount == recovery::MaxConditions || !editableCondition(condition)) return "INVALID_ARGUMENT";
  rule->conditions[rule->conditionCount++] = condition; return nullptr;
}
const char *ConfigurationService::replaceCondition(recovery::RuleId id, std::size_t index, const recovery::Condition &condition) {
  if (const char *error = editError()) return error;
  auto *rule = findRule(id);
  if (!rule || index >= rule->conditionCount || !editableCondition(condition)) return "INVALID_ARGUMENT";
  rule->conditions[index] = condition; return nullptr;
}
const char *ConfigurationService::removeCondition(recovery::RuleId id, std::size_t index) {
  if (const char *error = editError()) return error;
  auto *rule = findRule(id); if (!rule || index >= rule->conditionCount) return "INVALID_ARGUMENT";
  for (std::size_t i = index + 1; i < rule->conditionCount; ++i) rule->conditions[i - 1] = rule->conditions[i];
  rule->conditions[--rule->conditionCount] = {}; return nullptr;
}
const char *ConfigurationService::assignOutput(recovery::RuleId id, recovery::OutputKind kind, uint32_t channel, uint32_t delayMs) {
  if (const char *error = editError()) return error;
  auto *rule = findRule(id);
  if (!rule || !validOutput(kind, channel) || delayMs > limits_.maxTimingMs || rule->assignmentCount == recovery::MaxAssignments ||
      (kind == recovery::OutputKind::Pyro && recovery::PyroPulseMs > limits_.maxPyroPulseMs))
    return "INVALID_ARGUMENT";
  for (std::size_t i = 0; i < draft_.ruleCount; ++i)
    for (std::size_t j = 0; j < draft_.rules[i].assignmentCount; ++j) {
      const auto &assignment = draft_.rules[i].assignments[j];
      if (assignment.kind == kind && assignment.channel == channel) return "OUTPUT_ALREADY_ASSIGNED";
    }
  auto &assignment = rule->assignments[rule->assignmentCount++]; assignment = {}; assignment.kind = kind; assignment.channel = uint8_t(channel);
  assignment.delayMs = delayMs;
  if (kind == recovery::OutputKind::Pyro) draft_.pyros[channel-1].pulseMs = recovery::PyroPulseMs;
  return nullptr;
}
const char *ConfigurationService::unassignOutput(recovery::RuleId id, recovery::OutputKind kind, uint32_t channel) {
  if (const char *error = editError()) return error;
  auto *rule = findRule(id); if (!rule || !validOutput(kind, channel)) return "INVALID_ARGUMENT";
  for (std::size_t i = 0; i < rule->assignmentCount; ++i) {
    if (rule->assignments[i].kind != kind || rule->assignments[i].channel != channel) continue;
    for (std::size_t j = i + 1; j < rule->assignmentCount; ++j) rule->assignments[j - 1] = rule->assignments[j];
    rule->assignments[--rule->assignmentCount] = {}; return nullptr;
  }
  return "INVALID_ARGUMENT";
}

const char *ConfigurationService::setAirbrakeChannels(uint32_t mask) {
  if (const char *error=editError()) return error;
  if (mask>15) return "INVALID_ARGUMENT";
  draft_.airbrakes.channelMask=uint8_t(mask); return nullptr;
}
const char *ConfigurationService::addAirbrakeCondition(const recovery::Condition &condition) {
  if (const char *error=editError()) return error;
  auto &airbrakes=draft_.airbrakes;
  if (airbrakes.conditionCount>=recovery::MaxConditions || !editableCondition(condition)) return "INVALID_ARGUMENT";
  airbrakes.conditions[airbrakes.conditionCount++]=condition; return nullptr;
}
const char *ConfigurationService::replaceAirbrakeCondition(std::size_t index,const recovery::Condition &condition) {
  if (const char *error=editError()) return error;
  auto &airbrakes=draft_.airbrakes;
  if (index>=airbrakes.conditionCount || !editableCondition(condition)) return "INVALID_ARGUMENT";
  airbrakes.conditions[index]=condition; return nullptr;
}
const char *ConfigurationService::removeAirbrakeCondition(std::size_t index) {
  if (const char *error=editError()) return error;
  auto &airbrakes=draft_.airbrakes;
  if (index>=airbrakes.conditionCount) return "INVALID_ARGUMENT";
  for (std::size_t i=index+1;i<airbrakes.conditionCount;++i) airbrakes.conditions[i-1]=airbrakes.conditions[i];
  airbrakes.conditions[--airbrakes.conditionCount]={}; return nullptr;
}
