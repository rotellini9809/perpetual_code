#pragma once
#include "simulation/HilSession.h"
#include "communication/CommandService.h"

// Parses/formats the development USB protocol; session owns processing and state.
class SensorHilCommands : public CommandExtension {
public:
  explicit SensorHilCommands(simulation::HilSession &session) : session_(session) {}
  bool dispatch(const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity) override;
  flight::RuntimeStatus status() const { return session_.status(); }
  bool active() const { return session_.active(); }
  void disconnect() { session_.stop(storage::RecordingEnd::Disconnected); }
private:
  simulation::HilSession &session_;
};
