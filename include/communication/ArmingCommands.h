#pragma once
#include "flight/ArmingController.h"
#include "communication/CommandService.h"

namespace simulation { class SimulationSession; class HilSession; }
class ArmingCommands : public CommandExtension {
public:
  ArmingCommands(flight::ArmingController &physical, sensors::SensorSource &source, flight::PhysicalChecks &checks,
                 simulation::SimulationSession *simulation = nullptr, simulation::HilSession *hil = nullptr)
      : physical_(physical), source_(source), checks_(checks), simulation_(simulation), hil_(hil) {}
  bool dispatch(const char *, const char *, bool, uint32_t, char *, std::size_t) override;
  flight::RuntimeStatus status() const { return physical_.runtime().status(); }
  bool active() const { return physical_.active(); }
  bool hasPreparedServos() const { return physical_.hasPreparedServos(); }
  bool hasServoPreview() const { return physical_.hasServoPreview(); }
  void releaseServoPreview() { physical_.releaseServoPreview(); }
private:
  flight::ArmingController &physical_;
  sensors::SensorSource &source_;
  flight::PhysicalChecks &checks_;
  simulation::SimulationSession *simulation_;
  simulation::HilSession *hil_;
};
