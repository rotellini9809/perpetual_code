#include "configuration/RecoveryConfiguration.h"

#include <cmath>

namespace recovery {
namespace {
ValidationResult failure(ValidationError error, Field field = Field::Plan,
                         RuleId rule = 0, std::size_t index = 0) {
  ValidationResult result;
  result.error = error; result.field = field; result.rule = rule; result.index = index;
  return result;
}
std::size_t findRule(const Configuration &config, RuleId id) {
  for (std::size_t i = 0; i < config.ruleCount; ++i)
    if (config.rules[i].id == id) return i;
  return MaxRules;
}
bool cycle(const Configuration &config, std::size_t index, std::array<uint8_t, MaxRules> &visitState) {
  // Depth-first search: 0 unvisited, 1 on the current path (seen again: a cycle), 2 finished.
  if (visitState[index] == 1) return true;
  if (visitState[index] == 2) return false;
  visitState[index] = 1;
  const auto &rule = config.rules[index];
  for (std::size_t i = 0; i < rule.conditionCount; ++i) {
    const auto &condition = rule.conditions[i];
    if (condition.kind == ConditionKind::TimeSinceRule &&
        cycle(config, findRule(config, condition.reference), visitState)) return true;
  }
  visitState[index] = 2;
  return false;
}
bool validCondition(const Condition &condition, uint32_t maxTimingMs) {
  switch (condition.kind) {
    case ConditionKind::Phase: return condition.phases != 0 && (condition.phases & ~AirborneMask) == 0;
    case ConditionKind::ApogeeDetected: return true;
    case ConditionKind::AltitudeComparison:
      return std::isfinite(condition.value) && (condition.comparison == Comparison::Less || condition.comparison == Comparison::Greater);
    case ConditionKind::AltitudeRange:
      return std::isfinite(condition.value) && std::isfinite(condition.upper) && condition.value <= condition.upper;
    case ConditionKind::AbsoluteVelocityComparison:
      return std::isfinite(condition.value) && condition.value >= 0 &&
             (condition.comparison == Comparison::LessEqual || condition.comparison == Comparison::GreaterEqual);
    case ConditionKind::AbsoluteVelocityRange:
      return std::isfinite(condition.value) && std::isfinite(condition.upper) && condition.value >= 0 && condition.value <= condition.upper;
    case ConditionKind::TimeSinceLaunch:
    case ConditionKind::TimeSinceApogee:
    case ConditionKind::TimeSinceRule: return condition.elapsedMs <= maxTimingMs;
  }
  return false;
}
} // namespace

ValidationResult validate(const Configuration &config, const ValidationLimits &limits) {
  if (limits.minServoUs == 0 || limits.minServoUs >= limits.maxServoUs ||
      limits.maxPyroPulseMs == 0 || limits.maxTimingMs == 0)
    return failure(ValidationError::InvalidLimits);
  if (config.ruleCount == 0 && !config.airbrakes.channelMask) return failure(ValidationError::EmptyPlan);
  if (config.ruleCount > MaxRules) return failure(ValidationError::TooManyRules);

  // Check all counts/IDs before following any dependency or indexing an array.
  for (std::size_t i = 0; i < config.ruleCount; ++i) {
    const auto &rule = config.rules[i];
    if (rule.id == 0) return failure(ValidationError::InvalidRuleId, Field::Rule, rule.id, i);
    for (std::size_t j = 0; j < i; ++j)
      if (config.rules[j].id == rule.id) return failure(ValidationError::DuplicateRuleId, Field::Rule, rule.id, i);
    bool terminated = false, visible = false;
    for (char ch : rule.name) {
      if (ch == '\0') { terminated = true; break; }
      if (static_cast<unsigned char>(ch) < 32 || ch == 127)
        return failure(ValidationError::InvalidName, Field::Rule, rule.id);
      if (ch != ' ') visible = true;
    }
    if (!terminated || !visible) return failure(ValidationError::InvalidName, Field::Rule, rule.id);
    if (rule.conditionCount == 0 || rule.conditionCount > MaxConditions)
      return failure(ValidationError::InvalidConditionCount, Field::Rule, rule.id);
    if (rule.assignmentCount == 0 || rule.assignmentCount > MaxAssignments)
      return failure(ValidationError::InvalidAssignmentCount, Field::Rule, rule.id);
    if (rule.delayMs > limits.maxTimingMs)
      return failure(ValidationError::InvalidTiming, Field::Rule, rule.id);
  }

  bool assigned[2][ChannelCount] = {};
  for (std::size_t i = 0; i < config.ruleCount; ++i) {
    const auto &rule = config.rules[i];
    for (std::size_t j = 0; j < rule.conditionCount; ++j) {
      const auto &condition = rule.conditions[j];
      if (!validCondition(condition, limits.maxTimingMs))
        return failure(ValidationError::InvalidCondition, Field::Condition, rule.id, j);
      if (condition.kind == ConditionKind::TimeSinceRule &&
          (condition.reference == 0 || condition.reference == rule.id || findRule(config, condition.reference) == MaxRules))
        return failure(ValidationError::InvalidReference, Field::Condition, rule.id, j);
    }
    for (std::size_t j = 0; j < rule.assignmentCount; ++j) {
      const auto &assignment = rule.assignments[j];
      if ((assignment.kind != OutputKind::Pyro && assignment.kind != OutputKind::Servo) || assignment.channel == 0 || assignment.channel > ChannelCount)
        return failure(ValidationError::InvalidOutput, Field::Assignment, rule.id, j);
      if (assignment.delayMs > limits.maxTimingMs)
        return failure(ValidationError::InvalidTiming, Field::Assignment, rule.id, j);
      const auto kind = static_cast<std::size_t>(assignment.kind);
      const auto channel = assignment.channel - 1;
      if (assigned[kind][channel]) return failure(ValidationError::DuplicateOutput, Field::Assignment, rule.id, j);
      assigned[kind][channel] = true;
    }
  }
  const auto &airbrakes=config.airbrakes;
  if (airbrakes.channelMask>15 || airbrakes.conditionCount>MaxConditions || (airbrakes.channelMask && !airbrakes.conditionCount))
    return failure(ValidationError::InvalidConditionCount);
  for (std::size_t j=0;j<airbrakes.conditionCount;++j) {
    const auto &condition=airbrakes.conditions[j];
    if (!validCondition(condition,limits.maxTimingMs)) return failure(ValidationError::InvalidCondition,Field::Condition,0,j);
    if (condition.kind==ConditionKind::TimeSinceRule && (!condition.reference || findRule(config,condition.reference)==MaxRules))
      return failure(ValidationError::InvalidReference,Field::Condition,0,j);
  }
  for (unsigned i=0;i<ChannelCount;++i) if (airbrakes.channelMask&(1u<<i)) {
    if (assigned[1][i]) return failure(ValidationError::DuplicateOutput,Field::Servo,0,i);
    const auto &servo=config.servos[i];
    if (!servo.calibrated || servo.initialUs==servo.finalUs) return failure(ValidationError::InvalidServoSettings,Field::Servo,0,i);
  }
  std::array<uint8_t, MaxRules> visitState{};
  for (std::size_t i = 0; i < config.ruleCount; ++i)
    if (cycle(config, i, visitState)) return failure(ValidationError::DependencyCycle, Field::Rule, config.rules[i].id);

  // Rules may be saved before servo setup. Physical preflight checks that assigned
  // servos are configured; explicitly supplied settings must still be valid here.
  for (std::size_t i = 0; i < ChannelCount; ++i) {
    const auto &pyro = config.pyros[i];
    if ((assigned[0][i] && pyro.pulseMs == 0) || pyro.pulseMs > limits.maxPyroPulseMs)
      return failure(ValidationError::InvalidPyroSettings, Field::Pyro, 0, i);
    const auto &servo = config.servos[i];
    const bool supplied = servo.calibrated || servo.initialUs != 0 || servo.finalUs != 0;
    if (supplied &&
        (!servo.calibrated || servo.initialUs < limits.minServoUs || servo.initialUs > limits.maxServoUs ||
         servo.finalUs < limits.minServoUs || servo.finalUs > limits.maxServoUs))
      return failure(ValidationError::InvalidServoSettings, Field::Servo, 0, i);
  }
  return {};
}

const char *errorName(ValidationError error) {
  switch (error) {
#define RECOVERY_ERROR(name) case ValidationError::name: return #name
    RECOVERY_ERROR(None); RECOVERY_ERROR(InvalidLimits); RECOVERY_ERROR(EmptyPlan);
    RECOVERY_ERROR(TooManyRules); RECOVERY_ERROR(InvalidRuleId); RECOVERY_ERROR(DuplicateRuleId);
    RECOVERY_ERROR(InvalidName); RECOVERY_ERROR(InvalidConditionCount); RECOVERY_ERROR(InvalidAssignmentCount);
    RECOVERY_ERROR(InvalidCondition); RECOVERY_ERROR(InvalidReference); RECOVERY_ERROR(DependencyCycle);
    RECOVERY_ERROR(InvalidOutput); RECOVERY_ERROR(DuplicateOutput); RECOVERY_ERROR(InvalidPyroSettings);
    RECOVERY_ERROR(InvalidServoSettings); RECOVERY_ERROR(InvalidTiming);
#undef RECOVERY_ERROR
  }
  return "UnknownValidationError";
}
} // namespace recovery
