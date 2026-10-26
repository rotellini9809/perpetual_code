#include "communication/ArmingCommands.h"
#include "communication/ArmingReply.h"
#include "simulation/SimulationSession.h"
#include "simulation/HilSession.h"

namespace {
void appendState(protocol::Json &json, const flight::ArmingController &controller) {
  json.add(",\"arming_state\":\"%s\",\"flight_mode\":\"%s\",\"armed\":%s,\"gyro_samples\":%lu,\"gyro_target\":%lu,\"last_error\":\"%s\"",
      flight::armingStateName(controller.state()), flight::flightModeName(controller.mode()), controller.armed() ? "true" : "false",
      static_cast<unsigned long>(controller.calibrationSamples()), static_cast<unsigned long>(flight::ArmingCalibration::TargetSamples), controller.error());
  const auto s = controller.runtime().status();
  json.add(",\"output_mode\":\"%s\",\"outputs_active\":%s",controller.outputMode()==storage::OutputMode::Physical ? "PHYSICAL" : "VIRTUAL",controller.outputsActive() ? "true" : "false");
  json.add(",\"boost_reached\":%s,\"recording_busy\":%s,\"recording_discarded\":%s,\"recording_error\":\"%s\"",
      s.boostReached ? "true" : "false", s.recordingBusy ? "true" : "false", s.recordingDiscarded ? "true" : "false",
      storage::recorderErrorName(s.recordingError));
}

}
bool ArmingCommands::dispatch(const char *command, const char *arguments, bool hasId, uint32_t id, char *out, std::size_t capacity) {
  const bool arm = !std::strcmp(command, "arm"), preflight = !std::strcmp(command, "preflight");
  const bool disarm = !std::strcmp(command, "disarm") || !std::strcmp(command, "ds");
  protocol::Tokens t(arguments); protocol::Json data; const char *error = nullptr;
  const bool prepare=!std::strcmp(command,"servo") && t.is(0,"arm");
  const bool continuity=!std::strcmp(command,"pyro") && t.is(0,"continuity");
  if (!std::strcmp(command,"servo") && t.valid && t.count==1 && t.is(0,"release")) {
    error=physical_.releaseServos();
    if (!error) data.add(",\"released\":true,\"prepared_servo_mask\":0");
    return protocol::finish(command,data,error,hasId,id,out,capacity,true);
  }
  if (!std::strcmp(command,"servo") && (t.is(0,"preview") || t.is(0,"release"))) {
    uint32_t channel=0, pwm=0; const bool release=t.is(0,"release");
    if (!t.valid || t.count!=(release ? 2u : 3u) || !protocol::unsignedNumber(t[1],channel) ||
        (!release && (!protocol::unsignedNumber(t[2],pwm) || !pwm))) error="INVALID_ARGUMENT";
    else error=physical_.positionServo(channel,pwm);
    if (!error) data.add(",\"channel\":%lu,\"pwm\":%lu",static_cast<unsigned long>(channel),static_cast<unsigned long>(pwm));
    return protocol::finish(command,data,error,hasId,id,out,capacity,true);
  }
  if (!arm && !preflight && !disarm && !prepare && !continuity) return false;
  const flight::ArmingController *owner = &physical_;
  if (!t.valid) error="INVALID_ARGUMENT";
  else if (continuity) {
    uint8_t configured=0, connected=0;
    if (t.count!=1) error="INVALID_ARGUMENT";
    else error=physical_.checkPyroContinuity(configured,connected);
    if (!error) data.add(",\"configured_pyro_mask\":%u,\"connected_pyro_mask\":%u,\"failed_pyro_mask\":%u,\"all_connected\":%s",
        unsigned(configured),unsigned(connected),unsigned(configured & ~connected),configured==connected ? "true" : "false");
  }
  else if (prepare) {
    uint32_t channel=0;
    if (t.count>2 || (t.count==2 && (!protocol::unsignedNumber(t[1],channel) || channel<1 || channel>recovery::ChannelCount))) error="INVALID_ARGUMENT";
    else {
      error=physical_.prepareServos(channel);
      const auto report=physical_.preflight(flight::FlightMode::Physical,true,&checks_);
      data.add(",\"initial_position_commanded\":%s",!error && report.preparedServos ? "true" : "false");
      protocol::appendPreflight(data,report);
    }
  }
  else if (preflight ? (t.count > 1 || (t.count && !t.is(0,"physical") && !t.is(0,"simulation"))) : t.count != 0) error = "INVALID_ARGUMENT";
  else if (preflight) {
    const auto mode = t.is(0,"simulation") ? flight::FlightMode::Simulation : flight::FlightMode::Physical;
    protocol::appendPreflight(data, physical_.preflight(mode, mode == flight::FlightMode::Physical, &checks_));
    data.add(",\"requested_mode\":\"%s\"", flight::flightModeName(mode));
  } else if (arm) {
    navigation::SensorCalibration device;
    device.upAxis=static_cast<navigation::UpAxis>(physical_.deviceUpAxis());
    error = physical_.arm(source_, flight::FlightMode::Physical, 0, 120, device, 1048576, &checks_);
    protocol::appendPreflight(data, physical_.lastPreflight());
  } else if (simulation_ && (simulation_->active() || simulation_->controller().state() == flight::ArmingState::Closing)) { owner = &simulation_->controller(); error = simulation_->disarm(); }
  else if (hil_ && (hil_->active() || hil_->controller().state() == flight::ArmingState::Closing)) { owner = &hil_->controller(); error = hil_->disarm(); }
  else error = physical_.disarm();
  appendState(data, *owner);
  return protocol::finish(command, data, error, hasId, id, out, capacity, true);
}
