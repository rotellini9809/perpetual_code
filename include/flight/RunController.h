#pragma once
#include "configuration/ConfigurationService.h"
#include "configuration/RecoveryRuleEngine.h"

namespace flight {
enum class RunMode { None, Estimates, Sensors };
struct RunEvaluation : recovery::Evaluation {
  std::size_t triggerCount = 0;
  std::array<recovery::RuleId, recovery::MaxRules> triggered{};
};
struct RunProgress { uint32_t samples = 0, timeMs = 0; };
// One immutable rule-engine plan and one run at a time, independent of transport.
class RunController {
public:
  static constexpr uint32_t LandingStartWindowMs = 15000; // After LANDED, pending starts may still fire this long.
  const char *begin(RunMode mode, ConfigurationService &config);
  unsigned stop(RunMode mode);
  void disconnect() { if (mode_ != RunMode::None) stop(mode_); }
  bool active(RunMode mode) const { return mode_ == mode && mode != RunMode::None; }
  uint64_t identity() const { return identity_; }
  bool active() const { return mode_ != RunMode::None; }
  const RunProgress &progress(RunMode mode) const { return progress_[mode == RunMode::Sensors ? 1 : 0]; }
  bool validTime(RunMode mode, uint32_t time) const;
  RunEvaluation evaluate(RunMode mode, uint32_t time, const recovery::FlightSnapshot &snapshot);
  bool landingWindowExpired(RunMode mode) const { return active(mode) && landed_ && progress(mode).timeMs-landedAtMs_>=LandingStartWindowMs; }
  uint8_t airbrakeConditions(uint32_t time, const recovery::FlightSnapshot &snapshot) const { return engine_.airbrakeConditions(time,snapshot); }
  void closeStarts() { if (active()) engine_.closeStarts(); }
private:
  ConfigurationService *settings_ = nullptr;
  uint64_t identity_ = 0;
  RunMode mode_ = RunMode::None;
  recovery::RuleEngine engine_;
  RunProgress progress_[2]; // [0] estimate replay, [1] sensor runs.
  std::array<recovery::RuleId, recovery::MaxRules> ruleIds_{};
  std::array<bool, recovery::MaxRules> triggerReported_{}; // Each trigger is reported in one RunEvaluation only.
  std::size_t ruleCount_ = 0;
  uint32_t landedAtMs_ = 0;
  recovery::FlightPhase previousPhase_ = recovery::FlightPhase::Pad;
  bool landed_ = false;
};
}
