#pragma once
#include "configuration/ConfigurationService.h"
#include "communication/CommandService.h"

// Disarmed-only development command layer; no actuator API or physical commands.
class ConfigurationCommands : public CommandExtension {
public:
  explicit ConfigurationCommands(ConfigurationService &service, bool physicalSimulation = false, bool physicalActions = false)
      : service_(service), physicalSimulation_(physicalSimulation), physicalActions_(physicalActions) {}
  void boot(bool ready) { service_.boot(ready); }
  void disconnect() { service_.disconnect(); }
  void setDisarmed(bool value) { service_.setDisarmed(value); }
  bool configValid() const { return service_.configValid(); }
  bool storageReady() const { return service_.storageReady(); }
  bool hasDraft() const { return service_.hasDraft(); }
  const recovery::Configuration *applied() const { return service_.applied(); }
  const recovery::ValidationLimits &limits() const { return service_.limits(); }
  const char *configError() const { return service_.configError(); }
  bool dispatch(const char *command, const char *arguments, bool hasId, uint32_t id,
                char *output, std::size_t capacity) override;
private:
  ConfigurationService &service_;
  const bool physicalSimulation_;
  const bool physicalActions_;
};
