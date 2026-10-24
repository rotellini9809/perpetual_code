#pragma once
#include "communication/CommandService.h"
#include "flight/RunController.h"

class EstimateReplayCommands : public CommandExtension {
public:
  EstimateReplayCommands(ConfigurationService &config, flight::RunController &run) : config_(config), run_(run) {}
  bool dispatch(const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity) override;
  bool active() const { return run_.active(flight::RunMode::Estimates); }
  void disconnect() { run_.stop(flight::RunMode::Estimates); }
private:
  ConfigurationService &config_;
  flight::RunController &run_;
};
