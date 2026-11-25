#pragma once
#include "communication/CommandProtocol.h"

namespace protocol {
enum class CommandGroup { Other, Configuration, Flights, Simulation, Hil, Replay, Control, Power };
struct CommandPolicy {
  CommandGroup group = CommandGroup::Other;
  bool readOnly = true, mutation = false, operational = false, maintenance = false, beginRun = false;
};
// Classify once for ownership, busy and power policy. Handlers still validate grammar.
// Do not reject malformed arguments here: existing guard/error precedence is intentional.
inline CommandPolicy classifyCommand(const char *command, const Tokens &t) {
  CommandPolicy policy;
  const bool config=!strcmp(command,"config");
  const bool flights=!strcmp(command,"flights");
  const bool arm=!strcmp(command,"arm");
  const bool disarm=!strcmp(command,"disarm") || !strcmp(command,"ds");
  const bool imu=!strcmp(command,"imu");
  if (config || !strcmp(command,"rule") || !strcmp(command,"condition") || !strcmp(command,"output") ||
      !strcmp(command,"airbrake") || !strcmp(command,"pyro") || !strcmp(command,"servo") || imu) policy.group=CommandGroup::Configuration;
  else if (flights) policy.group=CommandGroup::Flights;
  else if (!strcmp(command,"sim")) policy.group=CommandGroup::Simulation;
  else if (!strcmp(command,"hil")) policy.group=CommandGroup::Hil;
  else if (!strcmp(command,"replay")) policy.group=CommandGroup::Replay;
  else if (!strcmp(command,"control")) policy.group=CommandGroup::Control;
  else if (!strcmp(command,"sleep") || !strcmp(command,"wake")) policy.group=CommandGroup::Power;
  const bool configuration=policy.group==CommandGroup::Configuration;
  const bool run=policy.group==CommandGroup::Simulation || policy.group==CommandGroup::Hil || policy.group==CommandGroup::Replay;
  const bool deviceSetting=config && (t.is(0,"orientation") || t.is(0,"machine") || t.is(0,"airbrakes"));
  if (configuration)
    policy.readOnly=(imu && t.is(0,"status")) || (!strcmp(command,"airbrake") && t.is(0,"status")) || t.is(0,"show") || t.is(0,"list") || (config && t.is(0,"draft")) ||
               (deviceSetting && t.count==1) || (!strcmp(command,"pyro") && t.is(0,"continuity"));
  else if (run) policy.readOnly=t.is(0,"status") || t.is(0,"events") || t.is(0,"timing");
  else if (flights)
    policy.readOnly=!(t.is(0,"format") || t.is(0,"init") || t.is(0,"recover") || t.is(0,"delete") || (t.is(0,"timing") && t.is(1,"reset")));
  else if (arm || disarm || policy.group==CommandGroup::Power) policy.readOnly=false;
  policy.mutation=!policy.readOnly;
  policy.operational=(configuration && !(!strcmp(command,"airbrake") && t.is(0,"status"))) || flights || (run && !policy.readOnly) || !strcmp(command,"sensors") || !strcmp(command,"preflight") || arm || disarm;
  if (imu) policy.operational=!policy.readOnly;
  policy.maintenance=(config && t.is(0,"reset")) || (deviceSetting && t.count>1) || (flights && t.is(0,"format"));
  policy.beginRun=arm || (run && t.is(0,"begin"));
  if (!strcmp(command,"bluetooth") && t.is(0,"name")) { // Writes module memory and restarts the radio.
    policy.readOnly=false; policy.mutation=true; policy.maintenance=true;
  }
  return policy;
}
}
