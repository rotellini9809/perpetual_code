#include "flight/RunController.h"
#include <limits>

namespace flight {
const char *RunController::begin(RunMode mode, ConfigurationService &config) {
  if (active() || config.runLocked()) return "BUSY";
  if (config.hasDraft()) return "DRAFT_ACTIVE";
  if ((mode != RunMode::Estimates && mode != RunMode::Sensors) || !config.applied() || !config.configValid()) return "CONFIG_INVALID";
  if (identity_ == std::numeric_limits<uint64_t>::max()) return "SESSION_EXHAUSTED";
  if (!engine_.begin(*config.applied(), config.limits(), 100)) return "CONFIG_INVALID";
  if (!config.acquireRun()) { engine_.end(); return "ARMED"; }
  settings_ = &config;
  settings_->setDisarmed(false);
  ruleCount_ = config.applied()->ruleCount;
  for (std::size_t i = 0; i < ruleCount_; ++i) ruleIds_[i] = config.applied()->rules[i].id;
  triggerReported_ = {}; previousPhase_=recovery::FlightPhase::Pad; landed_ = false; landedAtMs_ = 0;
  progress_[mode == RunMode::Sensors ? 1 : 0] = {};
  ++identity_; mode_ = mode; return nullptr;
}
unsigned RunController::stop(RunMode mode) {
  if (!active(mode)) return 0;
  unsigned pending = 0;
  for (std::size_t i = 0; i < engine_.actionCount(); ++i)
    if (engine_.actions()[i].state == recovery::ActionState::Pending) ++pending;
  engine_.end(); settings_->releaseRun(); settings_->setDisarmed(true); settings_ = nullptr; mode_ = RunMode::None; return pending;
}
bool RunController::validTime(RunMode mode, uint32_t time) const {
  const auto &runProgress = progress(mode);
  return active(mode) && (!runProgress.samples || time > runProgress.timeMs) && runProgress.samples != std::numeric_limits<uint32_t>::max();
}
RunEvaluation RunController::evaluate(RunMode mode, uint32_t time, const recovery::FlightSnapshot &snapshot) {
  if (!validTime(mode, time)) return {};
  if (snapshot.phaseValid && settings_->stateMachine().bistage && previousPhase_==recovery::FlightPhase::Coast && snapshot.phase==recovery::FlightPhase::Descent) engine_.cancelPending();
  if (snapshot.phaseValid) previousPhase_=snapshot.phase;
  if (snapshot.phaseValid && snapshot.phase == recovery::FlightPhase::Landed && !landed_) {
    landed_ = true; landedAtMs_ = time;
  }
  if (landed_ && time - landedAtMs_ >= LandingStartWindowMs) engine_.closeStarts();
  RunEvaluation result;
  static_cast<recovery::Evaluation &>(result) = engine_.evaluate(time, snapshot);
  if (result.accepted) {
    for (std::size_t i = 0; i < ruleCount_; ++i) {
      if (triggerReported_[i] || !engine_.rules()[i].triggered) continue;
      triggerReported_[i] = true; result.triggered[result.triggerCount++] = ruleIds_[i];
    }
    auto &runProgress = progress_[mode == RunMode::Sensors ? 1 : 0]; ++runProgress.samples; runProgress.timeMs = time;
  }
  return result;
}
}
