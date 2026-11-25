#include "configuration/RecoveryRuleEngine.h"
#include <cmath>

namespace recovery {
bool RuleEngine::begin(const Configuration &config, const ValidationLimits &limits,
                       uint32_t maxAgeMs, uint64_t nowMs) {
  if (active_ || maxAgeMs == 0 || !validate(config, limits).ok()) return false;
  config_ = config;
  ruleStates_ = {};
  actions_ = {};
  actionCount_ = 0;
  for (std::size_t i = 0; i < config_.ruleCount; ++i) {
    const auto &rule = config_.rules[i];
    for (std::size_t j = 0; j < rule.assignmentCount; ++j) {
      const auto &assignment = rule.assignments[j];
      auto &action = actions_[actionCount_++];
      action.rule = rule.id; action.kind = assignment.kind; action.channel = assignment.channel;
    }
  }
  maxAgeMs_ = maxAgeMs; lastMs_ = nowMs;
  havePrevious_ = false;
  startsClosed_ = launched_ = apogeeDetected_ = landed_ = false;
  launchMs_ = apogeeMs_ = 0;
  active_ = true;
  return true;
}
bool RuleEngine::fresh(uint64_t now, uint64_t measured) const {
  return measured <= now && now - measured <= maxAgeMs_;
}
bool RuleEngine::conditionTrue(const Condition &condition, uint64_t now,
                              const FlightSnapshot &snapshot, bool altitudeFresh, bool velocityFresh) const {
  switch (condition.kind) {
    case ConditionKind::Phase:
      return (condition.phases & (1u << static_cast<uint8_t>(snapshot.phase))) != 0;
    case ConditionKind::ApogeeDetected: return apogeeDetected_;
    case ConditionKind::AltitudeComparison:
      return altitudeFresh && (condition.comparison == Comparison::Less ? snapshot.altitude.value < condition.value : snapshot.altitude.value > condition.value);
    case ConditionKind::AltitudeRange:
      return altitudeFresh && snapshot.altitude.value >= condition.value && snapshot.altitude.value <= condition.upper;
    case ConditionKind::AbsoluteVelocityComparison:
      return velocityFresh && (condition.comparison == Comparison::LessEqual ? std::fabs(snapshot.verticalVelocity.value) <= condition.value :
                                                                    std::fabs(snapshot.verticalVelocity.value) >= condition.value);
    case ConditionKind::AbsoluteVelocityRange:
      return velocityFresh && std::fabs(snapshot.verticalVelocity.value) >= condition.value && std::fabs(snapshot.verticalVelocity.value) <= condition.upper;
    case ConditionKind::TimeSinceLaunch: return launched_ && now - launchMs_ >= condition.elapsedMs;
    case ConditionKind::TimeSinceApogee: return apogeeDetected_ && now - apogeeMs_ >= condition.elapsedMs;
    case ConditionKind::TimeSinceRule:
      for (std::size_t i = 0; i < config_.ruleCount; ++i)
        if (config_.rules[i].id == condition.reference)
          return ruleStates_[i].triggered && now - ruleStates_[i].triggeredMs >= condition.elapsedMs;
      return false;
  }
  return false;
}
uint8_t RuleEngine::airbrakeConditions(uint64_t now, const FlightSnapshot &snapshot) const {
  uint8_t failedMask=0;
  const bool phaseValid=snapshot.phaseValid && uint8_t(snapshot.phase)<=uint8_t(FlightPhase::Abort) && fresh(now,snapshot.phaseMeasuredMs);
  const bool altitudeFresh=snapshot.altitude.valid && std::isfinite(snapshot.altitude.value) && fresh(now,snapshot.altitude.measuredMs);
  const bool velocityFresh=snapshot.verticalVelocity.valid && std::isfinite(snapshot.verticalVelocity.value) && fresh(now,snapshot.verticalVelocity.measuredMs);
  for (std::size_t i=0;i<config_.airbrakes.conditionCount;++i)
    if (!active_ || !phaseValid || !airbornePhase(snapshot.phase) || !conditionTrue(config_.airbrakes.conditions[i],now,snapshot,altitudeFresh,velocityFresh)) failedMask|=uint8_t(1u<<i);
  return failedMask;
}
Evaluation RuleEngine::evaluate(uint64_t now, const FlightSnapshot &snapshot) {
  Evaluation result;
  if (!active_ || now < lastMs_) return result;
  result.accepted = true;
  lastMs_ = now;
  const bool newSnapshot = !havePrevious_ || snapshot.sequence > previous_.sequence;
  const bool phaseOk = newSnapshot && snapshot.phaseValid && fresh(now, snapshot.phaseMeasuredMs) &&
      static_cast<uint8_t>(snapshot.phase) <= static_cast<uint8_t>(FlightPhase::Abort) &&
      (!havePrevious_ || snapshot.phaseMeasuredMs >= previous_.phaseMeasuredMs);
  const bool altitudeFresh = newSnapshot && snapshot.altitude.valid && std::isfinite(snapshot.altitude.value) && fresh(now, snapshot.altitude.measuredMs) &&
      (!havePrevious_ || (snapshot.altitude.sequence > previous_.altitude.sequence && snapshot.altitude.measuredMs >= previous_.altitude.measuredMs));
  const bool velocityFresh = newSnapshot && snapshot.verticalVelocity.valid && std::isfinite(snapshot.verticalVelocity.value) && fresh(now, snapshot.verticalVelocity.measuredMs) &&
      (!havePrevious_ || (snapshot.verticalVelocity.sequence > previous_.verticalVelocity.sequence && snapshot.verticalVelocity.measuredMs >= previous_.verticalVelocity.measuredMs));
  if (phaseOk && snapshot.phase==FlightPhase::Abort) closeStarts();
  if (phaseOk) {
    if (!launched_ && airbornePhase(snapshot.phase)) {
      launched_ = true; launchMs_ = snapshot.phaseMeasuredMs;
    }
    if (launched_ && !apogeeDetected_ && snapshot.phase == FlightPhase::Descent) {
      apogeeDetected_ = true; apogeeMs_ = snapshot.phaseMeasuredMs;
    }
    if (snapshot.phase == FlightPhase::Landed) landed_ = true;
  }
  const bool airborne = phaseOk && launched_ && !landed_ &&
      airbornePhase(snapshot.phase);
  std::array<bool, MaxRules> newlyTriggered{};
  for (std::size_t i = 0; i < config_.ruleCount; ++i) {
    auto &state = ruleStates_[i];
    if (state.triggered) continue;
    bool satisfied = airborne && !startsClosed_ && snapshot.startsAllowed;
    const auto &rule = config_.rules[i];
    for (std::size_t j = 0; satisfied && j < rule.conditionCount; ++j)
      satisfied = conditionTrue(rule.conditions[j], now, snapshot, altitudeFresh, velocityFresh);
    newlyTriggered[i] = satisfied;
  }
  // Commit only after every rule has evaluated the previous trigger state.
  for (std::size_t i = 0; i < config_.ruleCount; ++i) {
    if (!newlyTriggered[i]) continue;
    ruleStates_[i].triggered = true; ruleStates_[i].triggeredMs = now;
    for (std::size_t j = 0; j < actionCount_; ++j)
      if (actions_[j].rule == config_.rules[i].id) actions_[j].state = ActionState::Pending;
  }
  // Once triggered, rule + channel delays run independently of later inputs.
  // Actions and assignments are built in the same order in begin().
  std::size_t actionIndex = 0;
  for (std::size_t i = 0; i < config_.ruleCount && !startsClosed_ && snapshot.startsAllowed; ++i) {
    const auto &rule = config_.rules[i];
    for (std::size_t j = 0; j < rule.assignmentCount; ++j) {
      auto &action = actions_[actionIndex++];
      const uint64_t delay = uint64_t(rule.delayMs) + rule.assignments[j].delayMs;
      if (action.state != ActionState::Pending || now - ruleStates_[i].triggeredMs < delay) continue;
      action.state = ActionState::Started; action.startedMs = now;
      auto &start = result.starts[result.startCount++];
      start.rule = action.rule; start.kind = action.kind; start.channel = action.channel; start.timeMs = now;
    }
  }
  if (newSnapshot) { previous_ = snapshot; havePrevious_ = true; }
  return result;
}
void RuleEngine::cancelPending() {
  for (auto &action:actions_) if (action.state==ActionState::Pending) action.state=ActionState::Cancelled;
}
void RuleEngine::closeStarts() { startsClosed_=true; cancelPending(); }
void RuleEngine::end() { closeStarts(); active_ = false; }
} // namespace recovery
