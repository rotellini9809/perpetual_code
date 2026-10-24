#include "configuration/RecoveryRuleEngine.h"
#include <cmath>

namespace recovery {
bool RuleEngine::begin(const Configuration &config, const ValidationLimits &limits,
                       uint32_t maxAgeMs, uint64_t nowMs) {
  if (active_ || maxAgeMs == 0 || !validate(config, limits).ok()) return false;
  config_ = config;
  runtime_ = {};
  actions_ = {};
  actionCount_ = 0;
  for (std::size_t i = 0; i < config_.ruleCount; ++i) {
    const auto &r = config_.rules[i];
    for (std::size_t j = 0; j < r.assignmentCount; ++j) {
      const auto &a = r.assignments[j];
      auto &out = actions_[actionCount_++];
      out.rule = r.id; out.kind = a.kind; out.channel = a.channel;
    }
  }
  maxAgeMs_ = maxAgeMs; lastMs_ = nowMs;
  havePrevious_ = false;
  startsClosed_ = launched_ = apogee_ = landed_ = false;
  launchMs_ = apogeeMs_ = 0;
  active_ = true;
  return true;
}
bool RuleEngine::fresh(uint64_t now, uint64_t measured) const {
  return measured <= now && now - measured <= maxAgeMs_;
}
bool RuleEngine::conditionTrue(const Condition &c, uint64_t now,
                              const FlightSnapshot &s, bool altitudeFresh, bool velocityFresh) const {
  switch (c.kind) {
    case ConditionKind::Phase:
      return (c.phases & (1u << static_cast<uint8_t>(s.phase))) != 0;
    case ConditionKind::ApogeeDetected: return apogee_;
    case ConditionKind::AltitudeComparison:
      return altitudeFresh && (c.comparison == Comparison::Less ? s.altitude.value < c.value : s.altitude.value > c.value);
    case ConditionKind::AltitudeRange:
      return altitudeFresh && s.altitude.value >= c.value && s.altitude.value <= c.upper;
    case ConditionKind::AbsoluteVelocityComparison:
      return velocityFresh && (c.comparison == Comparison::LessEqual ? std::fabs(s.verticalVelocity.value) <= c.value :
                                                                    std::fabs(s.verticalVelocity.value) >= c.value);
    case ConditionKind::AbsoluteVelocityRange:
      return velocityFresh && std::fabs(s.verticalVelocity.value) >= c.value && std::fabs(s.verticalVelocity.value) <= c.upper;
    case ConditionKind::TimeSinceLaunch: return launched_ && now - launchMs_ >= c.elapsedMs;
    case ConditionKind::TimeSinceApogee: return apogee_ && now - apogeeMs_ >= c.elapsedMs;
    case ConditionKind::TimeSinceRule:
      for (std::size_t i = 0; i < config_.ruleCount; ++i)
        if (config_.rules[i].id == c.reference)
          return runtime_[i].triggered && now - runtime_[i].triggeredMs >= c.elapsedMs;
      return false;
  }
  return false;
}
Evaluation RuleEngine::evaluate(uint64_t now, const FlightSnapshot &s) {
  Evaluation result;
  if (!active_ || now < lastMs_) return result;
  result.accepted = true;
  lastMs_ = now;
  const bool newSnapshot = !havePrevious_ || s.sequence > previous_.sequence;
  const bool phaseOk = newSnapshot && s.phaseValid && fresh(now, s.phaseMeasuredMs) &&
      static_cast<uint8_t>(s.phase) <= static_cast<uint8_t>(FlightPhase::Landed) &&
      (!havePrevious_ || s.phaseMeasuredMs >= previous_.phaseMeasuredMs);
  const bool altitudeFresh = newSnapshot && s.altitude.valid && std::isfinite(s.altitude.value) && fresh(now, s.altitude.measuredMs) &&
      (!havePrevious_ || (s.altitude.sequence > previous_.altitude.sequence && s.altitude.measuredMs >= previous_.altitude.measuredMs));
  const bool velocityFresh = newSnapshot && s.verticalVelocity.valid && std::isfinite(s.verticalVelocity.value) && fresh(now, s.verticalVelocity.measuredMs) &&
      (!havePrevious_ || (s.verticalVelocity.sequence > previous_.verticalVelocity.sequence && s.verticalVelocity.measuredMs >= previous_.verticalVelocity.measuredMs));
  if (phaseOk) {
    if (!launched_ && (s.phase == FlightPhase::Boost || s.phase == FlightPhase::Coast || s.phase == FlightPhase::Descent)) {
      launched_ = true; launchMs_ = s.phaseMeasuredMs;
    }
    if (launched_ && !apogee_ && s.phase == FlightPhase::Descent) {
      apogee_ = true; apogeeMs_ = s.phaseMeasuredMs;
    }
    if (s.phase == FlightPhase::Landed) landed_ = true;
  }
  const bool airborne = phaseOk && launched_ && !landed_ &&
      (s.phase == FlightPhase::Boost || s.phase == FlightPhase::Coast || s.phase == FlightPhase::Descent);
  std::array<bool, MaxRules> trigger{};
  for (std::size_t i = 0; i < config_.ruleCount; ++i) {
    auto &state = runtime_[i];
    if (state.triggered) continue;
    bool satisfied = airborne && !startsClosed_;
    const auto &r = config_.rules[i];
    for (std::size_t j = 0; satisfied && j < r.conditionCount; ++j)
      satisfied = conditionTrue(r.conditions[j], now, s, altitudeFresh, velocityFresh);
    trigger[i] = satisfied;
  }
  // Commit only after every rule has evaluated the previous trigger snapshot.
  for (std::size_t i = 0; i < config_.ruleCount; ++i) {
    if (!trigger[i]) continue;
    runtime_[i].triggered = true; runtime_[i].triggeredMs = now;
    for (std::size_t j = 0; j < actionCount_; ++j)
      if (actions_[j].rule == config_.rules[i].id) actions_[j].state = ActionState::Pending;
  }
  // Once triggered, rule + channel delays run independently of later inputs.
  // Actions and assignments are built in the same order in begin().
  std::size_t action = 0;
  for (std::size_t i = 0; i < config_.ruleCount && !startsClosed_; ++i) {
    const auto &rule = config_.rules[i];
    for (std::size_t j = 0; j < rule.assignmentCount; ++j) {
      auto &a = actions_[action++];
      const uint64_t delay = uint64_t(rule.delayMs) + rule.assignments[j].delayMs;
      if (a.state != ActionState::Pending || now - runtime_[i].triggeredMs < delay) continue;
      a.state = ActionState::Started; a.startedMs = now;
      auto &start = result.starts[result.startCount++];
      start.rule = a.rule; start.kind = a.kind; start.channel = a.channel; start.timeMs = now;
    }
  }
  if (newSnapshot) { previous_ = s; havePrevious_ = true; }
  return result;
}
void RuleEngine::closeStarts() {
  startsClosed_ = true;
  for (auto &a : actions_) if (a.state == ActionState::Pending) a.state = ActionState::Cancelled;
}
void RuleEngine::end() { closeStarts(); active_ = false; }
} // namespace recovery
