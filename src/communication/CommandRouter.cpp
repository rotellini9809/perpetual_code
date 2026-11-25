#include "communication/CommandRouter.h"
#include "communication/CommandProtocol.h"

// Every command passes through dispatchFrom()/dispatchCommand() on the core 0 stack. Replies
// built here live in separate functions, so their buffers are reserved only when used.
namespace {
__attribute__((noinline)) bool finishEmpty(const char *command, const char *error, bool hasId, uint32_t id,
                                           char *output, std::size_t capacity, bool includeErrorData = false) {
  return protocol::finish(command, protocol::Json{}, error, hasId, id, output, capacity, includeErrorData);
}
}

bool CommandRouter::occupied() const {
  return settings_.hasDraft() || settings_.runLocked() || run_.active() ||
      (flights_ && flights_->storageBusy()) || (arming_ && arming_->active()) ||
      (simulation_ && simulation_->active()) || (arming_ && arming_->hasServoPreview());
}

bool CommandRouter::powerBusy() const {
  return occupied() || !settings_.disarmed() || (flights_ && flights_->transferActive()) ||
      (transportBusy_ && transportBusy_());
}
void CommandRouter::pollPower() {
  if (!power_) return;
  // Actual work starts its idle interval on completion. Passive response traffic
  // only defers entry; status/heartbeat replies must not reset the idle timer.
  if (occupied() || !settings_.disarmed()) power_->activity();
  power_->poll(powerBusy());
}

void CommandRouter::disconnect(Source source) {
  if (owner_ != source) return;
  if (arming_) arming_->releaseServoPreview();
  // Autonomous simulation and physical arming/flight survive loss of their client.
  if ((!simulation_ || !simulation_->active()) && (!arming_ || !arming_->active())) {
    hil_.disconnect(); run_.disconnect();
  }
  settings_.disconnect();
  owner_ = Source::None; // A new connection may take over a surviving autonomous run.
}

bool CommandRouter::dispatchFrom(Source source, const char *command, const char *arguments,
                                bool hasId, uint32_t id, char *output, std::size_t capacity) {
  using protocol::Tokens;
  Tokens t(arguments);
  const auto policy=protocol::classifyCommand(command,t);
  using Group=protocol::CommandGroup;
  if (power_ && (policy.operational || policy.group==Group::Power)) power_->activity();
  if (!occupied()) owner_ = Source::None;
  if (policy.group==Group::Control)
    return controlReply(source,command,t.valid && t.count == 1 && t.is(0,"status"),hasId,id,output,capacity);
  if (policy.mutation && owner_ != Source::None && owner_ != source)
    return finishEmpty(command,"CONTROL_BUSY",hasId,id,output,capacity);
  if (power_ && policy.group==Group::Power)
    return powerReply(command,t.valid && !t.count,hasId,id,output,capacity);
  if (power_ && power_->sleeping() && policy.operational) {
    if (const auto error=power_->wake()) return finishEmpty(command,error,hasId,id,output,capacity,true);
  }
  const bool handled = dispatchCommand(command,arguments,policy,hasId,id,output,capacity);
  if (policy.mutation && owner_ == Source::None && occupied()) owner_ = source;
  return handled;
}

bool CommandRouter::controlReply(Source source, const char *command, bool valid, bool hasId, uint32_t id, char *output, std::size_t capacity) {
  protocol::Json data;
  data.add(",\"transport\":\"%s\",\"owner\":\"%s\"", source == Source::Bluetooth ? "bluetooth" : "usb",
           owner_ == Source::None ? "none" : owner_ == Source::Usb ? "usb" : "bluetooth");
  return protocol::finish(command, data, valid ? nullptr : "INVALID_ARGUMENT", hasId,id,output,capacity);
}
bool CommandRouter::powerReply(const char *command, bool valid, bool hasId, uint32_t id, char *output, std::size_t capacity) {
  protocol::Json data; const char *error=nullptr;
  if (!valid) error="INVALID_ARGUMENT";
  else if (!strcmp(command,"wake")) error=power_->wake();
  else error=power_->sleep(powerBusy());
  data.add(",\"power_mode\":\"%s\"",power_->name());
  return protocol::finish(command,data,error,hasId,id,output,capacity,true);
}
bool CommandRouter::bluetoothNameReply(const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity) {
  protocol::Tokens t(arguments); protocol::Json data; char name[24]={};
  const char *error=!bluetooth_ ? "UNKNOWN_COMMAND" : !t.valid || t.count!=2 ? "INVALID_ARGUMENT" :
      occupied() || !settings_.disarmed() ? "BUSY" : bluetooth_->requestName(t.values[1],name,sizeof(name));
  if (!error) data.add(R"(,"name":"%s","ble_name":"%s_BLE","restarting":true)",name,name);
  return protocol::finish(command,data,error,hasId,id,output,capacity);
}
bool CommandRouter::airbrakeStatusReply(const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity) {
  const bool simActive = simulation_ && simulation_->active();
  const bool physicalActive = arming_ && arming_->active();
  protocol::Tokens t(arguments); protocol::Json data;
  auto runtime=physicalActive ? arming_->status() : simActive ? simulation_->status() : hil_.status();
  const auto *plan=settings_.applied(); const auto &airbrakes=runtime.airbrakes;
  data.add(R"(,"enabled":%s,"active":%s,"physical":%s,"channel_mask":%u,"flags":%u,"failed_conditions":%u,"opening_pct":[)",
    settings_.airbrakesEnabled() ? "true" : "false",runtime.active ? "true" : "false",
    runtime.outputMode==storage::OutputMode::Physical ? "true" : "false",plan ? unsigned(plan->airbrakes.channelMask) : 0,
    runtime.active ? unsigned(airbrakes.flags) : 0,runtime.active ? unsigned(airbrakes.failedConditions) : 0);
  for (unsigned i=0;i<4;++i) {
    if (i) data.add(",");
    if (runtime.active && airbrakes.opening[i]!=65535) data.add("%.2f",double(airbrakes.opening[i])/100); else data.add("null");
  }
  data.add("]");
  return protocol::finish(command,data,t.valid && t.count==1 ? nullptr : "INVALID_ARGUMENT",hasId,id,output,capacity);
}

bool CommandRouter::dispatch(const char *command, const char *arguments, bool hasId, uint32_t id,
                             char *output, std::size_t capacity) {
  return dispatchFrom(Source::Usb,command,arguments,hasId,id,output,capacity);
}

bool CommandRouter::dispatchCommand(const char *command, const char *arguments, const protocol::CommandPolicy &policy, bool hasId, uint32_t id,
                                    char *output, std::size_t capacity) {
  using Group=protocol::CommandGroup;
  const bool simActive = simulation_ && simulation_->active();
  const bool physicalActive = arming_ && arming_->active();
  const bool runGroup=policy.group==Group::Replay || policy.group==Group::Hil || policy.group==Group::Simulation;
  const bool otherRun=(policy.group==Group::Replay && (hil_.active() || simActive)) ||
      (policy.group==Group::Hil && (estimates_.active() || simActive)) || (physicalActive && runGroup);
  if (policy.maintenance && (simActive || physicalActive || (arming_ && arming_->hasServoPreview())))
    return finishEmpty(command,"BUSY",hasId,id,output,capacity);
  const bool configWrite=policy.group==Group::Configuration && !policy.readOnly;
  const bool imuSessionCommand=!strcmp(command,"imu") && arming_ && arming_->imuCalibrationActive();
  if (((run_.active() || settings_.runLocked()) && ((configWrite && !imuSessionCommand) || otherRun)) ||
      (flights_ && flights_->storageBusy() && (configWrite || policy.beginRun)))
    return finishEmpty(command,"BUSY",hasId,id,output,capacity);
  if (!strcmp(command,"bluetooth") && protocol::Tokens(arguments).is(0,"name"))
    return bluetoothNameReply(command,arguments,hasId,id,output,capacity);
  if (!strcmp(command,"airbrake") && protocol::Tokens(arguments).is(0,"status"))
    return airbrakeStatusReply(command,arguments,hasId,id,output,capacity);
  // Freeze the previous simulation's completed storage progress before a new
  // run can reuse the shared recorder counters. The busy guard above owns handoff.
  if (policy.beginRun && simulation_) simulation_->refreshRecording();
  if (arming_ && arming_->dispatch(command, arguments, hasId, id, output, capacity)) return true;
  if (flights_ && policy.group==Group::Flights) return flights_->dispatch(command, arguments, hasId, id, output, capacity);
  if (simulation_ && policy.group==Group::Simulation) return simulation_->dispatch(command, arguments, hasId, id, output, capacity);
  if (policy.group==Group::Replay) return estimates_.dispatch(command, arguments, hasId, id, output, capacity);
  if (policy.group==Group::Hil) return hil_.dispatch(command, arguments, hasId, id, output, capacity);
  return configuration_.dispatch(command, arguments, hasId, id, output, capacity);
}
