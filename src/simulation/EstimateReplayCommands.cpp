#include "simulation/EstimateReplayCommands.h"
#include "communication/RunReply.h"

bool EstimateReplayCommands::dispatch(const char *command, const char *arguments, bool hasId, uint32_t id,
                                      char *output, std::size_t capacity) {
  if (strcmp(command, "replay") != 0) return false;
  using flight::RunMode;
  const auto mode = RunMode::Estimates;
  const auto &progress = run_.progress(mode);
  protocol::Tokens t(arguments); protocol::Json data; const char *error = nullptr;
  if (run_.active() && !active()) error = "BUSY";
  else if (!t.valid) error = "INVALID_ARGUMENT";
  else if (t.count == 1 && t.is(0, "begin")) {
    error = run_.begin(mode, config_);
    if (!error) data.add(",\"active\":true,\"source\":\"recorded_estimates\",\"output_mode\":\"VIRTUAL\"");
  } else if (t.count == 1 && t.is(0, "status")) {
    data.add(",\"active\":%s,\"samples\":%lu,\"t_ms\":%lu,\"output_mode\":\"VIRTUAL\"", active() ? "true" : "false",
             static_cast<unsigned long>(progress.samples), static_cast<unsigned long>(progress.timeMs));
  } else if (t.count == 1 && t.is(0, "stop")) {
    const auto pending = run_.stop(mode);
    data.add(",\"active\":false,\"samples\":%lu,\"cancelled_pending\":%u", static_cast<unsigned long>(progress.samples), pending);
  } else if (t.count == 5 && t.is(0, "sample")) {
    uint32_t time = 0, legacy = 0; float altitude = 0, velocity = 0;
    if (!active()) error = "NOT_REPLAYING";
    else if (!protocol::unsignedNumber(t[1], time) || !protocol::unsignedNumber(t[2], legacy) || legacy > 5 ||
             !protocol::floating(t[3], altitude) || !protocol::floating(t[4], velocity)) error = "INVALID_ARGUMENT";
    else if (!run_.validTime(mode, time)) error = "INVALID_TIMESTAMP";
    else {
      recovery::FlightSnapshot s;
      s.phase = legacy <= 2 ? static_cast<recovery::FlightPhase>(legacy) :
          (legacy == 5 ? recovery::FlightPhase::Landed : recovery::FlightPhase::Descent);
      s.phaseValid = true; s.phaseMeasuredMs = time; s.sequence = uint64_t(progress.samples) + 1;
      s.altitude.value = altitude; s.altitude.valid = std::isfinite(altitude);
      s.altitude.measuredMs = time; s.altitude.sequence = s.sequence;
      s.verticalVelocity = s.altitude; s.verticalVelocity.value = velocity; s.verticalVelocity.valid = std::isfinite(velocity);
      const auto result = run_.evaluate(mode, time, s);
      if (!result.accepted) error = "INVALID_TIMESTAMP";
      else {
        data.add(",\"t_ms\":%lu,\"phase\":%u,\"output_mode\":\"VIRTUAL\"", static_cast<unsigned long>(time), unsigned(s.phase));
        protocol::appendRunEvents(data, result);
      }
    }
  } else error = "INVALID_ARGUMENT";
  return protocol::finish(command, data, error, hasId, id, output, capacity);
}
