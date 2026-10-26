#include "communication/CommandRouter.h"
#include "communication/CommandProtocol.h"

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
  const bool configGroup = !strcmp(command,"config") || !strcmp(command,"rule") ||
      !strcmp(command,"condition") || !strcmp(command,"output") || !strcmp(command,"pyro") || !strcmp(command,"servo");
  const bool runGroup = !strcmp(command,"sim") || !strcmp(command,"hil") || !strcmp(command,"replay");
  const bool flights = !strcmp(command,"flights");
  const bool powerCommand=!strcmp(command,"sleep") || !strcmp(command,"wake");
  const bool runRead=runGroup && (t.is(0,"status") || t.is(0,"events") || t.is(0,"timing"));
  const bool operational=configGroup || flights || (runGroup && !runRead) || !strcmp(command,"sensors") ||
      !strcmp(command,"preflight") || !strcmp(command,"arm") || !strcmp(command,"disarm") || !strcmp(command,"ds");
  if (power_ && (operational || powerCommand)) power_->activity();
  const bool mutation = (configGroup && !(t.is(0,"show") || t.is(0,"list") || (!strcmp(command,"config") && (t.is(0,"draft") || ((t.is(0,"orientation") || t.is(0,"machine")) && t.count==1))) || (!strcmp(command,"pyro") && t.is(0,"continuity")))) ||
      (runGroup && !(t.is(0,"status") || t.is(0,"events") || t.is(0,"timing"))) ||
      (flights && (t.is(0,"format") || t.is(0,"init") || t.is(0,"recover") || t.is(0,"delete") || (t.is(0,"timing") && t.is(1,"reset")))) ||
      !strcmp(command,"arm") || !strcmp(command,"disarm") || !strcmp(command,"ds") || powerCommand;
  if (!occupied()) owner_ = Source::None;
  if (!strcmp(command,"control")) {
    protocol::Json data;
    data.add(",\"transport\":\"%s\",\"owner\":\"%s\"", source == Source::Bluetooth ? "bluetooth" : "usb",
             owner_ == Source::None ? "none" : owner_ == Source::Usb ? "usb" : "bluetooth");
    return protocol::finish(command, data, t.valid && t.count == 1 && t.is(0,"status") ? nullptr : "INVALID_ARGUMENT", hasId,id,output,capacity);
  }
  if (mutation && owner_ != Source::None && owner_ != source) {
    return protocol::finish(command, protocol::Json{}, "CONTROL_BUSY", hasId,id,output,capacity);
  }
  if (power_ && powerCommand) {
    protocol::Json data; const char *error=nullptr;
    if (!t.valid || t.count) error="INVALID_ARGUMENT";
    else if (!strcmp(command,"wake")) error=power_->wake();
    else error=power_->sleep(powerBusy());
    data.add(",\"power_mode\":\"%s\"",power_->name());
    return protocol::finish(command,data,error,hasId,id,output,capacity,true);
  }
  if (power_ && power_->sleeping() && operational) {
    if (const auto error=power_->wake())
      return protocol::finish(command,protocol::Json{},error,hasId,id,output,capacity,true);
  }
  const bool handled = dispatchCommand(command,arguments,hasId,id,output,capacity);
  if (mutation && owner_ == Source::None && occupied()) owner_ = source;
  return handled;
}

bool CommandRouter::dispatch(const char *command, const char *arguments, bool hasId, uint32_t id,
                             char *output, std::size_t capacity) {
  return dispatchFrom(Source::Usb,command,arguments,hasId,id,output,capacity);
}

bool CommandRouter::dispatchCommand(const char *command, const char *arguments, bool hasId, uint32_t id,
                                    char *output, std::size_t capacity) {
  protocol::Tokens tokens(arguments);
  const bool group = strcmp(command, "config") == 0 || strcmp(command, "rule") == 0 ||
      strcmp(command, "condition") == 0 || strcmp(command, "output") == 0 ||
      strcmp(command, "pyro") == 0 || strcmp(command, "servo") == 0;
  const bool read = tokens.is(0, "show") || tokens.is(0, "list") ||
      (strcmp(command, "config") == 0 && (tokens.is(0, "draft") || ((tokens.is(0,"orientation") || tokens.is(0,"machine")) && tokens.count==1))) ||
      (strcmp(command, "pyro") == 0 && tokens.is(0, "continuity"));
  const bool simActive = simulation_ && simulation_->active();
  const bool physicalActive = arming_ && arming_->active();
  const bool otherRun = (strcmp(command, "replay") == 0 && (hil_.active() || simActive)) ||
      (strcmp(command, "hil") == 0 && (estimates_.active() || simActive)) ||
      (physicalActive && (!strcmp(command,"replay") || !strcmp(command,"hil") || !strcmp(command,"sim")));
  const bool maintenance=(!strcmp(command,"config") && (tokens.is(0,"reset") || ((tokens.is(0,"orientation") || tokens.is(0,"machine")) && tokens.count>1))) || (!strcmp(command,"flights") && tokens.is(0,"format"));
  if (maintenance && (simActive || physicalActive || (arming_ && arming_->hasServoPreview())))
    return protocol::finish(command,protocol::Json{},"BUSY",hasId,id,output,capacity);
  const bool beginRun = !strcmp(command,"arm") || (tokens.is(0,"begin") &&
      (strcmp(command,"replay")==0 || strcmp(command,"hil")==0 || strcmp(command,"sim")==0));
  if (((run_.active() || settings_.runLocked()) && ((group && !read) || otherRun)) ||
      (flights_ && flights_->storageBusy() && ((group && !read) || beginRun))) {
    const protocol::Json data;
    return protocol::finish(command, data, "BUSY", hasId, id, output, capacity);
  }
  // Freeze the previous simulation's completed storage progress before a new
  // run can reuse the shared recorder counters. The busy guard above owns handoff.
  if (beginRun && simulation_) simulation_->refreshRecording();
  if (arming_ && arming_->dispatch(command, arguments, hasId, id, output, capacity)) return true;
  if (flights_ && strcmp(command, "flights") == 0) return flights_->dispatch(command, arguments, hasId, id, output, capacity);
  if (simulation_ && strcmp(command, "sim") == 0) return simulation_->dispatch(command, arguments, hasId, id, output, capacity);
  if (strcmp(command, "replay") == 0) return estimates_.dispatch(command, arguments, hasId, id, output, capacity);
  if (strcmp(command, "hil") == 0) return hil_.dispatch(command, arguments, hasId, id, output, capacity);
  return configuration_.dispatch(command, arguments, hasId, id, output, capacity);
}
