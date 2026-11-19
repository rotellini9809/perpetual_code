#include "simulation/SimulationSession.h"
#include "storage/FlightRecorder.h"
#include <cstring>

namespace simulation {
const char *SimulationSession::begin(bool record, bool physical, bool stress) {
  if (!clock_) return "TIMING_UNAVAILABLE";
  const bool bistage=config_.stateMachine().bistage;
  if (stress && bistage) return "PROFILE_REQUIRES_NORMAL_MACHINE";
  using Profile=sensors::SimulatedSensorSource::Profile;
  if (!source_.selectProfile(stress ? Profile::Hoppino480 : bistage ? Profile::Bistage : Profile::Hoppino)) return "BUSY";
  if (!physical) if (const auto error=controller_.releaseForVirtualSimulation(record)) return error;
  if (const char *error = controller_.arm(source_, flight::FlightMode::Simulation, source_.activeReferenceAltitude(), source_.activeNominalHz(), navigation::SensorCalibration{},
                                       record ? storage::FlightRecorder::Unlimited : 0, nullptr, physical ? storage::OutputMode::Physical : storage::OutputMode::Virtual)) return error;
  navigation_ = {}; cycles_ = {}; overBudget_ = invalid_ = 0;
  cancelled_ = phaseCount_ = startCount_ = 0; phases_ = {}; starts_ = {};
  reason_ = "running"; error_ = "None"; running_ = true;
  return nullptr;
}
void SimulationSession::close(const char *reason) {
  running_ = false; reason_ = reason;
  cancelled_ = controller_.stop(!strcmp(reason,"completed") ? storage::RecordingEnd::Completed : storage::RecordingEnd::Stopped);
  finalStatus_ = controller_.runtime().status();
}
void SimulationSession::stop() { if (running_) close("stopped"); }
bool SimulationSession::poll() {
  if (!running_) { refreshRecording(); return false; }
  if (!controller_.armed()) { error_ = "SESSION_CANCELLED"; close("error"); return false; }
  const auto start = clock_();
  flight::FrameResult result;
  const auto polled = controller_.poll(result);
  if (polled.error) { error_ = polled.error; close("error"); return false; }
  // Shared physical runtime may complete its landing window before dataset EOF.
  if (!controller_.armed()) {
    if (strcmp(controller_.error(),"None")) { error_=controller_.error(); close("error"); }
    else close("completed");
    return false;
  }
  if (!polled.processed) return false;
  navigation_ = result.navigation;
  if (!navigation_.valid) ++invalid_;
  if (!phaseCount_ || phases_[phaseCount_-1].phase != navigation_.phase) {
    if (phaseCount_ < phases_.size()) {
      phases_[phaseCount_].phase = navigation_.phase; phases_[phaseCount_++].timeMs = result.timeMs;
    }
  }
  for (std::size_t i = 0; i < result.events.startCount; ++i)
    if (startCount_ < starts_.size()) starts_[startCount_++] = result.events.starts[i];
  const auto elapsed = uint32_t(clock_()-start); cycles_.add(elapsed);
  if (elapsed > source_.activePeriodUs()) ++overBudget_;
  if (source_.finished()) close("completed");
  return true;
}
}
