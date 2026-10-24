#include "flight/RunController.h"
#include <limits>

namespace flight {
const char *RunController::begin(RunMode mode, ConfigurationService &config) {
  if (active() || config.runLocked()) return "BUSY";
  if (config.hasDraft()) return "DRAFT_ACTIVE";
  if ((mode != RunMode::Estimates && mode != RunMode::Sensors) || !config.applied()) return "CONFIG_INVALID";
  if (identity_ == std::numeric_limits<uint64_t>::max()) return "SESSION_EXHAUSTED";
  if (!engine_.begin(*config.applied(), config.limits(), 100)) return "CONFIG_INVALID";
  if (!config.acquireRun()) { engine_.end(); return "ARMED"; }
  settings_ = &config;
  settings_->setDisarmed(false);
  ruleCount_ = config.applied()->ruleCount;
  for (std::size_t i = 0; i < ruleCount_; ++i) ids_[i] = config.applied()->rules[i].id;
  reported_ = {}; landed_ = false; landedAt_ = 0;
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
  const auto &p = progress(mode);
  return active(mode) && (!p.samples || time > p.timeMs) && p.samples != std::numeric_limits<uint32_t>::max();
}
RunEvaluation RunController::evaluate(RunMode mode, uint32_t time, const recovery::FlightSnapshot &snapshot) {
  if (!validTime(mode, time)) return {};
  if (snapshot.phaseValid && snapshot.phase == recovery::FlightPhase::Landed && !landed_) {
    landed_ = true; landedAt_ = time;
  }
  if (landed_ && time - landedAt_ >= 15000) engine_.closeStarts();
  RunEvaluation result;
  static_cast<recovery::Evaluation &>(result) = engine_.evaluate(time, snapshot);
  if (result.accepted) {
    for (std::size_t i = 0; i < ruleCount_; ++i) {
      if (reported_[i] || !engine_.rules()[i].triggered) continue;
      reported_[i] = true; result.triggered[result.triggerCount++] = ids_[i];
    }
    auto &p = progress_[mode == RunMode::Sensors ? 1 : 0]; ++p.samples; p.timeMs = time;
  }
  return result;
}
}
