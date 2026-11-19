#include "simulation/SimulationCommands.h"
#include "communication/RunReply.h"
#include "communication/TimingReply.h"
#include "communication/ArmingReply.h"
#include "communication/RecordingReply.h"

bool SimulationCommands::dispatch(const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity) {
  if (strcmp(command, "sim") != 0) return false;
  protocol::Tokens tokens(arguments); protocol::Json data; const char *error = nullptr;
  auto status = [&]() {
    const auto s = session_.status();
    data.add(",\"source\":\"%s\",\"duration_ms\":%lu,\"total_samples\":%lu", session_.sourceName(),
             static_cast<unsigned long>(session_.durationMs()), static_cast<unsigned long>(session_.totalSamples()));
    data.add(",\"arming_state\":\"%s\",\"flight_mode\":\"simulation\"", flight::armingStateName(session_.controller().state()));
    data.add(",\"active\":%s,\"reason\":\"%s\",\"error\":\"%s\",\"output_mode\":\"%s\",\"outputs_active\":%s,\"samples\":%lu,\"t_ms\":%lu,\"phase\":%u",
             session_.active() ? "true" : "false", session_.reason(), session_.error(),
             s.outputMode == storage::OutputMode::Physical ? "PHYSICAL" : "VIRTUAL", s.outputsActive ? "true" : "false", static_cast<unsigned long>(s.samples),
             static_cast<unsigned long>(s.timeMs), unsigned(session_.navigation().phase));
    data.add(",\"bistage\":%s,\"abort_reason\":%u",s.bistage ? "true" : "false",unsigned(s.abortReason));
    data.number("coast_velocity_median",s.coastVelocityMedian); data.number("tilt_deg",s.tiltRadians*57.2957795131f);
    data.number("z_est", session_.navigation().altitude); data.number("vz_est", session_.navigation().verticalVelocity);
    protocol::appendRecordingBusy(data,s);
    protocol::appendRecordingCounts(data,s);
    protocol::appendRecordingError(data,s);
    protocol::appendRecordingPending(data,s);
    data.add(R"(,"invalid_frames":%lu,"starts":%u,"cancelled_pending":%u)",
             static_cast<unsigned long>(session_.invalidFrames()), session_.startCount(), session_.cancelled());
  };
  if (!tokens.valid) error = "INVALID_ARGUMENT";
  else if (tokens.is(0,"begin") && tokens.count<=4) {
    const bool physical=tokens.is(1,"physical");
    const bool record=tokens.is(physical ? 2 : 1,"record");
    const bool stress=tokens.is(1u+unsigned(physical)+unsigned(record),"480");
    if (tokens.count != 1u+unsigned(physical)+unsigned(record)+unsigned(stress)) error="INVALID_ARGUMENT";
    else {
      error=session_.begin(record,physical,stress);
      if (!error) status();
      else if (!session_.controller().lastPreflight().ready()) protocol::appendPreflight(data,session_.controller().lastPreflight());
    }
  } else if (tokens.count == 1 && tokens.is(0, "status")) status();
  else if (tokens.count == 1 && tokens.is(0, "stop")) { session_.stop(); status(); }
  else if (tokens.count == 1 && tokens.is(0, "timing")) {
    const auto &h = session_.runtime().timing();
    data.add(",\"period_us\":%lu,\"duration_ms\":%lu,\"missed_ticks\":%lu,\"max_lateness_us\":%lu,\"over_budget\":%lu,\"prepare_us\":%lu,\"finish_us\":%lu",
             static_cast<unsigned long>(session_.periodUs()), static_cast<unsigned long>(session_.durationMs()),
             static_cast<unsigned long>(session_.missed()), static_cast<unsigned long>(session_.maxLatenessUs()),
             static_cast<unsigned long>(session_.overBudget()), static_cast<unsigned long>(h.prepareUs), static_cast<unsigned long>(h.finishUs));
    protocol::appendTiming(data, "cycle", session_.cycles()); protocol::appendTiming(data, "compute", h.compute);
    protocol::appendTiming(data, "storage", h.storage);
    data.add(",\"frame_dt_us\":[%lu,%lu],\"sequence_gaps\":%lu",static_cast<unsigned long>(h.frameDtMinUs),
             static_cast<unsigned long>(h.frameDtMaxUs),static_cast<unsigned long>(h.sequenceGaps));
  } else if (tokens.count == 1 && tokens.is(0, "events")) {
    data.add(",\"output_mode\":\"%s\",\"phases\":[",session_.status().outputMode == storage::OutputMode::Physical ? "PHYSICAL" : "VIRTUAL");
    for (unsigned i = 0; i < session_.phaseCount(); ++i)
      data.add("%s[%u,%lu]", i ? "," : "", unsigned(session_.phases()[i].phase), static_cast<unsigned long>(session_.phases()[i].timeMs));
    data.add("]"); flight::RunEvaluation events;
    events.startCount = session_.startCount();
    for (std::size_t i = 0; i < events.startCount; ++i) events.starts[i] = session_.starts()[i];
    protocol::appendRunEvents(data, events);
  } else error = "INVALID_ARGUMENT";
  return protocol::finish(command, data, error, hasId, id, output, capacity, true);
}
