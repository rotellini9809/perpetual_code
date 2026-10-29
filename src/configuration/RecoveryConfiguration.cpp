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
bool cycle(const Configuration &config, std::size_t index, std::array<uint8_t, MaxRules> &colors) {
  if (colors[index] == 1) return true;
  if (colors[index] == 2) return false;
  colors[index] = 1;
  const auto &rule = config.rules[index];
  for (std::size_t i = 0; i < rule.conditionCount; ++i) {
    const auto &condition = rule.conditions[i];
    if (condition.kind == ConditionKind::TimeSinceRule &&
        cycle(config, findRule(config, condition.reference), colors)) return true;
  }
  colors[index] = 2;
  return false;
}
bool validCondition(const Condition &c, uint32_t maxTimingMs) {
  switch (c.kind) {
    case ConditionKind::Phase: return c.phases != 0 && (c.phases & ~AirborneMask) == 0;
    case ConditionKind::ApogeeDetected: return true;
    case ConditionKind::AltitudeComparison:
      return std::isfinite(c.value) && (c.comparison == Comparison::Less || c.comparison == Comparison::Greater);
    case ConditionKind::AltitudeRange:
      return std::isfinite(c.value) && std::isfinite(c.upper) && c.value <= c.upper;
    case ConditionKind::AbsoluteVelocityComparison:
      return std::isfinite(c.value) && c.value >= 0 &&
             (c.comparison == Comparison::LessEqual || c.comparison == Comparison::GreaterEqual);
    case ConditionKind::AbsoluteVelocityRange:
      return std::isfinite(c.value) && std::isfinite(c.upper) && c.value >= 0 && c.value <= c.upper;
    case ConditionKind::TimeSinceLaunch:
    case ConditionKind::TimeSinceApogee:
    case ConditionKind::TimeSinceRule: return c.elapsedMs <= maxTimingMs;
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
    const auto &r = config.rules[i];
    if (r.id == 0) return failure(ValidationError::InvalidRuleId, Field::Rule, r.id, i);
    for (std::size_t j = 0; j < i; ++j)
      if (config.rules[j].id == r.id) return failure(ValidationError::DuplicateRuleId, Field::Rule, r.id, i);
    bool terminated = false, visible = false;
    for (char ch : r.name) {
      if (ch == '\0') { terminated = true; break; }
      if (static_cast<unsigned char>(ch) < 32 || ch == 127)
        return failure(ValidationError::InvalidName, Field::Rule, r.id);
      if (ch != ' ') visible = true;
    }
    if (!terminated || !visible) return failure(ValidationError::InvalidName, Field::Rule, r.id);
    if (r.conditionCount == 0 || r.conditionCount > MaxConditions)
      return failure(ValidationError::InvalidConditionCount, Field::Rule, r.id);
    if (r.assignmentCount == 0 || r.assignmentCount > MaxAssignments)
      return failure(ValidationError::InvalidAssignmentCount, Field::Rule, r.id);
    if (r.delayMs > limits.maxTimingMs)
      return failure(ValidationError::InvalidTiming, Field::Rule, r.id);
  }

  bool assigned[2][ChannelCount] = {};
  for (std::size_t i = 0; i < config.ruleCount; ++i) {
    const auto &r = config.rules[i];
    for (std::size_t j = 0; j < r.conditionCount; ++j) {
      const auto &c = r.conditions[j];
      if (!validCondition(c, limits.maxTimingMs))
        return failure(ValidationError::InvalidCondition, Field::Condition, r.id, j);
      if (c.kind == ConditionKind::TimeSinceRule &&
          (c.reference == 0 || c.reference == r.id || findRule(config, c.reference) == MaxRules))
        return failure(ValidationError::InvalidReference, Field::Condition, r.id, j);
    }
    for (std::size_t j = 0; j < r.assignmentCount; ++j) {
      const auto &a = r.assignments[j];
      if ((a.kind != OutputKind::Pyro && a.kind != OutputKind::Servo) || a.channel == 0 || a.channel > ChannelCount)
        return failure(ValidationError::InvalidOutput, Field::Assignment, r.id, j);
      if (a.delayMs > limits.maxTimingMs)
        return failure(ValidationError::InvalidTiming, Field::Assignment, r.id, j);
      const auto kind = static_cast<std::size_t>(a.kind);
      const auto channel = a.channel - 1;
      if (assigned[kind][channel]) return failure(ValidationError::DuplicateOutput, Field::Assignment, r.id, j);
      assigned[kind][channel] = true;
    }
  }
  const auto &air=config.airbrakes;
  if (air.channelMask>15 || air.conditionCount>MaxConditions || (air.channelMask && !air.conditionCount))
    return failure(ValidationError::InvalidConditionCount);
  for (std::size_t j=0;j<air.conditionCount;++j) {
    const auto &c=air.conditions[j];
    if (!validCondition(c,limits.maxTimingMs)) return failure(ValidationError::InvalidCondition,Field::Condition,0,j);
    if (c.kind==ConditionKind::TimeSinceRule && (!c.reference || findRule(config,c.reference)==MaxRules))
      return failure(ValidationError::InvalidReference,Field::Condition,0,j);
  }
  for (unsigned i=0;i<ChannelCount;++i) if (air.channelMask&(1u<<i)) {
    if (assigned[1][i]) return failure(ValidationError::DuplicateOutput,Field::Servo,0,i);
    const auto &servo=config.servos[i];
    if (!servo.calibrated || servo.initialUs==servo.finalUs) return failure(ValidationError::InvalidServoSettings,Field::Servo,0,i);
  }
  std::array<uint8_t, MaxRules> colors{};
  for (std::size_t i = 0; i < config.ruleCount; ++i)
    if (cycle(config, i, colors)) return failure(ValidationError::DependencyCycle, Field::Rule, config.rules[i].id);

  // Rules may be saved before servo setup. Physical preflight checks that assigned
  // servos are configured; explicitly supplied settings must still be valid here.
  for (std::size_t i = 0; i < ChannelCount; ++i) {
    const auto &p = config.pyros[i];
    if ((assigned[0][i] && p.pulseMs == 0) || p.pulseMs > limits.maxPyroPulseMs)
      return failure(ValidationError::InvalidPyroSettings, Field::Pyro, 0, i);
    const auto &s = config.servos[i];
    const bool supplied = s.calibrated || s.initialUs != 0 || s.finalUs != 0;
    if (supplied &&
        (!s.calibrated || s.initialUs < limits.minServoUs || s.initialUs > limits.maxServoUs ||
         s.finalUs < limits.minServoUs || s.finalUs > limits.maxServoUs))
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
