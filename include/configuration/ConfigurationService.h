#pragma once
#include "storage/ConfigurationStore.h"

// Owns settings independently of USB or any other command transport.
class ConfigurationService {
public:
  ConfigurationService(storage::ConfigurationStore &store, recovery::ValidationLimits limits)
      : store_(store), limits_(limits) {}
  void boot(bool storageReady);
  void disconnect() { drafting_ = false; }
  void setDisarmed(bool value) { disarmed_ = value; }
  bool disarmed() const { return disarmed_; }
  bool configValid() const { return appliedValid_ && machineReady_ && airbrakesReady_; }
  bool storageReady() const { return storageReady_; }
  bool hasDraft() const { return drafting_; }
  const recovery::Configuration *applied() const { return appliedValid_ ? &applied_ : nullptr; }
  const recovery::Configuration &view(bool draft) const { return draft ? draft_ : applied_; }
  const recovery::ValidationLimits &limits() const { return limits_; }
  const char *configError() const { return storageReady_ && !airbrakesReady_ ? "AIRBRAKES_SETTING_INVALID" : storageReady_ && !machineReady_ ? "STATE_MACHINE_INVALID" : storage::ConfigurationStore::errorName(loadError_); }
  bool acquireRun() {
    if (runLocked_ || drafting_ || !appliedValid_ || !machineReady_ || !airbrakesReady_ || !disarmed_) return false;
    runLocked_ = true; return true;
  }
  void releaseRun() { runLocked_ = false; }
  bool runLocked() const { return runLocked_; }
  const char *beginDraft();
  const char *discardDraft();
  const char *setPyro(uint32_t channel, uint32_t pulseMs);
  const char *setServo(uint32_t channel, uint32_t initialUs, uint32_t finalUs);
  const char *addRule(recovery::RuleId id, const char *name);
  const char *renameRule(recovery::RuleId id, const char *name);
  const char *setRuleDelay(recovery::RuleId id, uint32_t delayMs);
  const char *removeRule(recovery::RuleId id, recovery::RuleId *referencingRule = nullptr);
  const char *addCondition(recovery::RuleId id, const recovery::Condition &condition);
  const char *replaceCondition(recovery::RuleId id, std::size_t index, const recovery::Condition &condition);
  const char *removeCondition(recovery::RuleId id, std::size_t index);
  const char *setAirbrakeChannels(uint32_t mask);
  const char *addAirbrakeCondition(const recovery::Condition &condition);
  const char *replaceAirbrakeCondition(std::size_t index, const recovery::Condition &condition);
  const char *removeAirbrakeCondition(std::size_t index);
  const char *assignOutput(recovery::RuleId id, recovery::OutputKind kind, uint32_t channel, uint32_t delayMs = 0);
  const char *unassignOutput(recovery::RuleId id, recovery::OutputKind kind, uint32_t channel);
  recovery::ValidationResult validateDraft() const { return recovery::validate(draft_, limits_); }
  const char *apply();
  const char *factoryReset();
  const char *setStateMachine(const recovery::StateMachineSettings &settings);
  bool machineReady() const { return machineReady_; }
  const recovery::StateMachineSettings &stateMachine() const { return machine_; }
  const char *setAirbrakes(bool enabled);
  bool airbrakesEnabled() const { return airbrakesEnabled_; }
  bool airbrakesReady() const { return airbrakesReady_; }
  const char *setOrientation(uint8_t axis);
  uint8_t upAxis() const { return upAxis_; }
  bool orientationReady() const { return orientationReady_; }
private:
  const char *editError() const;
  recovery::Rule *findRule(recovery::RuleId id);
  storage::ConfigurationStore &store_;
  recovery::ValidationLimits limits_;
  recovery::Configuration applied_{}, draft_{}, verified_{};
  bool appliedValid_ = false, drafting_ = false, storageReady_ = false, disarmed_ = true;
  bool runLocked_ = false;
  uint8_t upAxis_ = 0;
  bool orientationReady_ = false;
  recovery::StateMachineSettings machine_;
  bool machineReady_ = false, airbrakesEnabled_=false, airbrakesReady_=false;
  storage::ConfigError loadError_ = storage::ConfigError::Missing;
};
