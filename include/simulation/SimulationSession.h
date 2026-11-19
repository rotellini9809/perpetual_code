#pragma once
#include "flight/ArmingController.h"
#include "simulation/SimulatedSensorSource.h"

namespace simulation {
class SimulationSession {
public:
  SimulationSession(ConfigurationService &config, flight::RunController &run, timing::Clock clock,
                    storage::RecordingSink *recorder = nullptr, flight::OutputExecutor *outputs = nullptr)
      : controller_(config, run, clock, recorder, outputs), source_(clock), clock_(clock), config_(config) {}
  // stress selects the 480 Hz Hoppino profile; it is single-stage and needs the normal machine.
  const char *begin(bool record = false, bool physical = false, bool stress = false);
  bool poll(); // One due sensor frame per call, independent of USB connection.
  void stop();
  void refreshRecording() { if (!running_) controller_.refreshRecordingStatus(finalStatus_); }
  bool active() const { return running_ && controller_.armed(); }
  const flight::FlightRuntime &runtime() const { return controller_.runtime(); }
  const flight::ArmingController &controller() const { return controller_; }
  void setBattery(const BatteryMonitor &battery) { controller_.setBattery(battery); }
  const char *disarm() { if (const auto error = controller_.disarmError()) return error; stop(); return nullptr; }
  flight::RuntimeStatus status() const {
    auto result = running_ ? controller_.runtime().status() : finalStatus_;
    controller_.refreshRecordingStatus(result); return result;
  }
  const navigation::NavigationOutput &navigation() const { return navigation_; }
  const timing::Statistics &cycles() const { return cycles_; }
  uint32_t missed() const { return source_.missed(); }
  uint32_t maxLatenessUs() const { return source_.maxLatenessUs(); }
  uint32_t overBudget() const { return overBudget_; }
  uint32_t invalidFrames() const { return invalid_; }
  unsigned cancelled() const { return cancelled_; }
  const char *reason() const { return reason_; }
  const char *error() const { return error_; }
  struct PhaseEvent { recovery::FlightPhase phase = recovery::FlightPhase::Pad; uint32_t timeMs = 0; };
  const std::array<PhaseEvent, 8> &phases() const { return phases_; }
  bool bistage() const { return source_.bistage(); }
  const char *sourceName() const { return source_.profileName(); }
  uint32_t periodUs() const { return source_.activePeriodUs(); }
  uint32_t durationMs() const { return source_.activeDurationMs(); }
  uint32_t totalSamples() const { return source_.activeSampleCount(); }
  unsigned phaseCount() const { return phaseCount_; }
  const std::array<recovery::VirtualStart, recovery::MaxAssignments> &starts() const { return starts_; }
  unsigned startCount() const { return startCount_; }
private:
  void close(const char *reason);
  flight::ArmingController controller_;
  sensors::SimulatedSensorSource source_;
  timing::Clock clock_;
  ConfigurationService &config_;
  bool running_ = false;
  const char *reason_ = "idle", *error_ = "None";
  navigation::NavigationOutput navigation_;
  flight::RuntimeStatus finalStatus_;
  timing::Statistics cycles_;
  uint32_t overBudget_ = 0, invalid_ = 0;
  unsigned cancelled_ = 0, phaseCount_ = 0, startCount_ = 0;
  std::array<PhaseEvent, 8> phases_{};
  std::array<recovery::VirtualStart, recovery::MaxAssignments> starts_{};
};
}
