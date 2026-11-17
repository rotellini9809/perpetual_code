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
  CommandPolicy p;
  const bool config=!strcmp(command,"config");
  const bool flights=!strcmp(command,"flights");
  const bool arm=!strcmp(command,"arm");
  const bool disarm=!strcmp(command,"disarm") || !strcmp(command,"ds");
  const bool imu=!strcmp(command,"imu");
  if (config || !strcmp(command,"rule") || !strcmp(command,"condition") || !strcmp(command,"output") ||
      !strcmp(command,"airbrake") || !strcmp(command,"pyro") || !strcmp(command,"servo") || imu) p.group=CommandGroup::Configuration;
  else if (flights) p.group=CommandGroup::Flights;
  else if (!strcmp(command,"sim")) p.group=CommandGroup::Simulation;
  else if (!strcmp(command,"hil")) p.group=CommandGroup::Hil;
  else if (!strcmp(command,"replay")) p.group=CommandGroup::Replay;
  else if (!strcmp(command,"control")) p.group=CommandGroup::Control;
  else if (!strcmp(command,"sleep") || !strcmp(command,"wake")) p.group=CommandGroup::Power;
  const bool configuration=p.group==CommandGroup::Configuration;
  const bool run=p.group==CommandGroup::Simulation || p.group==CommandGroup::Hil || p.group==CommandGroup::Replay;
  const bool deviceSetting=config && (t.is(0,"orientation") || t.is(0,"machine") || t.is(0,"airbrakes"));
  if (configuration)
    p.readOnly=(imu && t.is(0,"status")) || (!strcmp(command,"airbrake") && t.is(0,"status")) || t.is(0,"show") || t.is(0,"list") || (config && t.is(0,"draft")) ||
               (deviceSetting && t.count==1) || (!strcmp(command,"pyro") && t.is(0,"continuity"));
  else if (run) p.readOnly=t.is(0,"status") || t.is(0,"events") || t.is(0,"timing");
  else if (flights)
    p.readOnly=!(t.is(0,"format") || t.is(0,"init") || t.is(0,"recover") || t.is(0,"delete") || (t.is(0,"timing") && t.is(1,"reset")));
  else if (arm || disarm || p.group==CommandGroup::Power) p.readOnly=false;
  p.mutation=!p.readOnly;
  p.operational=(configuration && !(!strcmp(command,"airbrake") && t.is(0,"status"))) || flights || (run && !p.readOnly) || !strcmp(command,"sensors") || !strcmp(command,"preflight") || arm || disarm;
  if (imu) p.operational=!p.readOnly;
  p.maintenance=(config && t.is(0,"reset")) || (deviceSetting && t.count>1) || (flights && t.is(0,"format"));
  p.beginRun=arm || (run && t.is(0,"begin"));
  return p;
}
}
