#include "simulation/SensorHilCommands.h"
#include "communication/RunReply.h"
#include "communication/TimingReply.h"

bool SensorHilCommands::dispatch(const char *command, const char *arguments, bool hasId, uint32_t id,
                                 char *output, std::size_t capacity) {
  if (strcmp(command, "hil") != 0) return false;
  protocol::Tokens tokens(arguments);
  protocol::Json data; const char *error = nullptr;
  auto finish = [&]() { return protocol::finish(command, data, error, hasId, id, output, capacity); };
  if (session_.blockedByOtherRun()) { error = "BUSY"; return finish(); }
  if (!tokens.valid) { error = "INVALID_ARGUMENT"; return finish(); }
  auto recordingStatus = [&]() {
    const auto s = session_.status();
    data.add(",\"recording_requested\":%s,\"recording_active\":%s,\"recording_sequence\":\"%llu\",\"saved_samples\":%lu,\"dropped_samples\":%lu,\"samples_full\":%s,\"recording_error\":\"%s\"",
             s.recordingRequested ? "true" : "false", s.recordingActive ? "true" : "false", static_cast<unsigned long long>(s.recordingSequence),
             static_cast<unsigned long>(s.savedSamples), static_cast<unsigned long>(s.droppedSamples), s.samplesFull ? "true" : "false",
             storage::recorderErrorName(s.recordingError));
    data.add(",\"recording_async\":%s,\"recording_busy\":%s,\"pending_samples\":%lu,\"queue_capacity\":%lu", s.recordingAsync ? "true" : "false", s.recordingBusy ? "true" : "false",
             static_cast<unsigned long>(s.pendingSamples), static_cast<unsigned long>(s.queueCapacity));
  };
  if ((tokens.count == 4 || tokens.count == 6) && tokens.is(0, "begin")) {
    float reference = 0, hz = 0; uint32_t recordBytes = 0;
    error = session_.preparationError();
    if (!error && (session_.convention() != sensors::FrameConvention::LoggedCalibratedBody || !tokens.is(3, "logged_body") || !protocol::finiteNumber(tokens[1], reference) ||
                  !protocol::finiteNumber(tokens[2], hz))) error = "INVALID_ARGUMENT";
    if (!error && tokens.count == 6 && (!tokens.is(4, "record") || !protocol::unsignedNumber(tokens[5], recordBytes) || !recordBytes)) error = "INVALID_ARGUMENT";
    if (!error) error = session_.begin(reference, hz, navigation::SensorCalibration{}, recordBytes);
    if (!error) data.add(",\"active\":true,\"source\":\"hil_sensors\",\"frame\":\"logged_body\",\"output_mode\":\"VIRTUAL\",\"timing_valid\":%s",
                         session_.status().timingValid ? "true" : "false");
    if (!error) recordingStatus();
  } else if (tokens.count == 1 && tokens.is(0, "timing")) {
    const auto &s = session_.timing();
    data.add(",\"timing_valid\":%s,\"prepare_us\":%lu,\"finish_us\":%lu,\"nominal_period_us\":%lu,\"over_budget\":%lu",
             session_.status().timingValid ? "true" : "false", static_cast<unsigned long>(s.prepareUs),
             static_cast<unsigned long>(s.finishUs), static_cast<unsigned long>(s.nominalPeriodUs), static_cast<unsigned long>(s.overBudget));
    protocol::appendTiming(data, "compute", s.compute); protocol::appendTiming(data, "storage", s.storage);
    protocol::appendTiming(data, "step", s.step);
  } else if (tokens.count == 1 && tokens.is(0, "status")) {
    const auto status = session_.status();
    data.add(",\"arming_state\":\"%s\",\"flight_mode\":\"simulation\"", flight::armingStateName(session_.controller().state()));
    data.add(",\"active\":%s,\"samples\":%lu,\"t_ms\":%lu,\"max_compute_us\":%lu,\"timing_valid\":%s,\"output_mode\":\"VIRTUAL\"",
             status.active ? "true" : "false", static_cast<unsigned long>(status.samples), static_cast<unsigned long>(status.timeMs),
             static_cast<unsigned long>(status.maxComputeUs), status.timingValid ? "true" : "false");
    recordingStatus();
  } else if ((tokens.count == 1 || (tokens.count == 2 && tokens.is(1,"completed"))) && tokens.is(0, "stop")) {
    const auto pending = session_.stop(tokens.count==2 ? storage::RecordingEnd::Completed : storage::RecordingEnd::Stopped); const auto status = session_.status();
    data.add(",\"active\":false,\"samples\":%lu,\"cancelled_pending\":%u,\"max_compute_us\":%lu,\"timing_valid\":%s",
             static_cast<unsigned long>(status.samples), pending, static_cast<unsigned long>(status.maxComputeUs), status.timingValid ? "true" : "false");
    recordingStatus();
  } else if ((tokens.count == 9 || tokens.count == 10) && tokens.is(0, "sample")) {
    uint32_t time = 0; float values[8] = {};
    values[7] = std::numeric_limits<float>::quiet_NaN();
    bool valid = protocol::unsignedNumber(tokens[1], time);
    for (unsigned i = 0; i < tokens.count-2; ++i) valid = protocol::floating(tokens[i + 2], values[i]) && valid;
    if (!active()) error = "NOT_REPLAYING";
    else if (!valid) error = "INVALID_ARGUMENT";
    else {
      flight::FrameResult result;
      error = session_.sample(time, values, values + 3, values[6], result, values[7]);
      if (!error) {
        const auto &nav = result.navigation;
        data.add(",\"t_ms\":%lu,\"phase\":%u,\"imu_valid\":%s,\"baro_updated\":%s,\"compute_us\":%lu,\"output_mode\":\"VIRTUAL\"",
                 static_cast<unsigned long>(result.timeMs), unsigned(nav.phase), nav.valid ? "true" : "false", nav.barometerUpdated ? "true" : "false",
                 static_cast<unsigned long>(result.computeUs));
        data.number("z_est", nav.altitude); data.number("vz_est", nav.verticalVelocity);
        data.number("ba_est", nav.accelerationBias); data.number("az_est", nav.verticalAcceleration);
        protocol::appendRunEvents(data, result.events);
      }
    }
  } else error = "INVALID_ARGUMENT";
  return finish();
}
