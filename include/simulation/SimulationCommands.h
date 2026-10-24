#pragma once
#include "simulation/SimulationSession.h"
#include "communication/CommandService.h"

class SimulationCommands : public CommandExtension {
public:
  explicit SimulationCommands(simulation::SimulationSession &session) : session_(session) {}
  bool active() const { return session_.active(); }
  void refreshRecording() { session_.refreshRecording(); }
  bool dispatch(const char *, const char *, bool, uint32_t, char *, std::size_t) override;
private:
  simulation::SimulationSession &session_;
};
