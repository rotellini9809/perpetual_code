#pragma once
#include "configuration/RecoveryConfiguration.h"

namespace recovery {
struct RuleRuntime {
  bool triggered = false;
  uint64_t triggeredMs = 0;
};
enum class ActionState : uint8_t { Unscheduled, Pending, Started, Cancelled };
struct VirtualAction {
  RuleId rule = 0;
  OutputKind kind = OutputKind::Pyro;
  uint8_t channel = 0;
  ActionState state = ActionState::Unscheduled;
  uint64_t startedMs = 0;
};
struct VirtualStart {
  RuleId rule = 0;
  OutputKind kind = OutputKind::Pyro;
  uint8_t channel = 0;
  uint64_t timeMs = 0;
};
struct Evaluation {
  bool accepted = false; // False for inactive engine or backward clock.
  std::size_t startCount = 0;
  std::array<VirtualStart, MaxAssignments> starts{};
};

class RuleEngine {
public:
  // Copies a validated immutable plan. No physical output API exists here.
  // Rejects replacement while active; failed begin never activates the engine.
  bool begin(const Configuration &config, const ValidationLimits &limits,
             uint32_t maxAgeMs, uint64_t nowMs = 0);
  Evaluation evaluate(uint64_t nowMs, const FlightSnapshot &snapshot);
  // Call at post-landing expiry or accepted manual disarm: cancel pending starts.
  void closeStarts();
  void end();
  bool active() const { return active_; }
  const std::array<RuleRuntime, MaxRules> &rules() const { return runtime_; }
  const std::array<VirtualAction, MaxAssignments> &actions() const { return actions_; }
  std::size_t actionCount() const { return actionCount_; }
private:
  bool conditionTrue(const Condition &condition, uint64_t now,
                     const FlightSnapshot &snapshot, bool altitudeFresh, bool velocityFresh) const;
  bool fresh(uint64_t now, uint64_t measured) const;
  Configuration config_{};
  std::array<RuleRuntime, MaxRules> runtime_{};
  std::array<VirtualAction, MaxAssignments> actions_{};
  std::size_t actionCount_ = 0;
  uint32_t maxAgeMs_ = 0;
  uint64_t lastMs_ = 0;
  FlightSnapshot previous_{};
  bool havePrevious_ = false;
  bool active_ = false;
  bool startsClosed_ = false;
  bool launched_ = false;
  bool apogee_ = false;
  bool landed_ = false;
  uint64_t launchMs_ = 0;
  uint64_t apogeeMs_ = 0;
};
} // namespace recovery
