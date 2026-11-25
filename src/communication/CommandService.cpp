#include "communication/CommandService.h"
#include "communication/CommandProtocol.h"
#include "communication/TimingReply.h"
#include <cstdio>
#include <cstring>
#include <limits>
#include <cmath>

namespace { bool space(char character) { return character == ' ' || character == '\t'; } }
void CommandService::reset() { length_ = 0; overflow_ = invalid_ = false; haveId_ = false; }
bool CommandService::feed(char byte, const Status &status, char *reply, std::size_t capacity) {
  if (reply == nullptr || capacity < MaxReply) return false;
  if (byte == '\n') {
    line_[length_] = 0;
    char request[MaxLine + 1]; memcpy(request, line_, length_ + 1);
    bool tracked = false; uint32_t requestId = 0;
    if (extension_ && !invalid_ && !overflow_) {
      const char *cursor = request; while (space(*cursor)) ++cursor;
      if (*cursor++ == '@') {
        tracked = *cursor >= '0' && *cursor <= '9';
        while (*cursor >= '0' && *cursor <= '9') {
          const uint32_t digit = uint32_t(*cursor++ - '0');
          if (requestId > (std::numeric_limits<uint32_t>::max() - digit) / 10) tracked = false;
          if (tracked) requestId = requestId * 10 + digit;
        }
        tracked = tracked && space(*cursor);
      }
    }
    if (tracked && haveId_ && requestId <= lastId_) {
      if (requestId == lastId_ && strcmp(request, lastRequest_) == 0) strcpy(reply, lastReply_);
      else std::snprintf(reply, capacity, "{\"id\":%lu,\"type\":\"error\",\"ok\":false,\"code\":\"%s\"}\n",
                         static_cast<unsigned long>(requestId), requestId == lastId_ ? "ID_CONFLICT" : "STALE_REQUEST");
      length_ = 0; overflow_ = invalid_ = false; return true;
    }
    const bool ready = dispatch(status, reply, capacity);
    if (ready && tracked) {
      haveId_ = true; lastId_ = requestId; strcpy(lastRequest_, request); strcpy(lastReply_, reply);
    }
    length_ = 0; overflow_ = invalid_ = false; return ready;
  }
  if (overflow_) return false;
  if (length_ == MaxLine) { overflow_ = true; return false; }
  if ((static_cast<unsigned char>(byte) < 32 && byte != '\t' && byte != '\r') ||
      static_cast<unsigned char>(byte) > 126) invalid_ = true;
  line_[length_++] = byte;
  return false;
}
bool CommandService::dispatch(const Status &status, char *reply, std::size_t capacity) {
  if (length_ && line_[length_ - 1] == '\r') --length_;
  line_[length_] = '\0';
  char *cursor = line_;
  while (space(*cursor)) ++cursor;
  bool identified = false;
  uint32_t id = 0;
  const char *code = nullptr;
  const char *message = nullptr;
  if (*cursor == '@') {
    ++cursor;
    bool valid = *cursor >= '0' && *cursor <= '9';
    while (*cursor >= '0' && *cursor <= '9') {
      const uint32_t digit = static_cast<uint32_t>(*cursor++ - '0');
      if (id > (std::numeric_limits<uint32_t>::max() - digit) / 10) valid = false;
      if (valid) id = id * 10 + digit;
    }
    if (!valid || !space(*cursor)) { code = "INVALID_ARGUMENT"; message = "Malformed request ID."; }
    else { identified = true; while (space(*cursor)) ++cursor; }
  }
  if (!code && overflow_) { code = "LINE_TOO_LONG"; message = "Line discarded through LF."; }
  if (!code && (invalid_ || std::strchr(cursor, '\r'))) {
    code = "INVALID_ARGUMENT"; message = "Invalid command characters.";
  }
  if (!code && !*cursor && !identified) return false;
  // Split in place: "<command> <arguments>" becomes two NUL-terminated strings inside line_.
  char *command = cursor;
  while (*cursor && !space(*cursor)) ++cursor;
  if (*cursor) *cursor++ = '\0';
  while (space(*cursor)) ++cursor;
  const char *arguments = cursor;
  if (!code && extension_ && extension_->dispatch(command, arguments, identified, id, reply, capacity)) return true;
  return localReply(command, arguments, identified, id, code, message, status, reply, capacity);
}
// Built-in commands and errors. Kept out of dispatch(): its reply buffers would otherwise stay
// reserved on the core 0 stack while extensions handle every other command below it.
__attribute__((noinline)) bool CommandService::localReply(const char *command, const char *arguments, bool identified, uint32_t id,
                                                          const char *code, const char *message, const Status &status, char *reply, std::size_t capacity) {
  if (!code && !std::strcmp(command,"bluetooth")) {
    protocol::Tokens t(arguments); protocol::Json data;
    data.add(",\"ready\":%s,\"connected\":%s,\"error\":\"%s\",\"baud\":%lu,\"uart_rx_overflows\":%lu",
             status.bluetoothReady ? "true" : "false", status.bluetoothConnected ? "true" : "false", status.bluetoothError, static_cast<unsigned long>(status.bluetoothBaud),static_cast<unsigned long>(status.bluetoothRxOverflows));
    data.add(",\"name\":\"%s\",\"rename\":\"%s\",\"configured\":%s,\"stage\":\"%s\"",status.bluetoothName,status.bluetoothRename,status.bluetoothConfigured ? "true" : "false",status.bluetoothStage);
    return protocol::finish(command,data,t.valid && t.count == 1 && t.is(0,"status") ? nullptr : "INVALID_ARGUMENT",identified,id,reply,capacity);
  }
  if (!code && !std::strcmp(command,"diag")) {
    protocol::Tokens t(arguments); protocol::Json data;
    const bool valid = t.valid && t.count == 0;
    if (valid && status.diagnostics) {
      auto &diag = *status.diagnostics;
      if (diag.refresh) diag.refresh(diag);
      data.add(",\"stack_bytes\":[[%lu,%lu],[%lu,%lu]],\"stack_exhausted\":[%s,%s]",
               static_cast<unsigned long>(diag.stack[0].usedBytes), static_cast<unsigned long>(diag.stack[0].capacityBytes),
               static_cast<unsigned long>(diag.stack[1].usedBytes), static_cast<unsigned long>(diag.stack[1].capacityBytes),
               diag.stack[0].exhausted ? "true" : "false", diag.stack[1].exhausted ? "true" : "false");
      data.add(",\"stack_intact\":[[%lu,%lu,%lu],[%lu,%lu,%lu]]",
               static_cast<unsigned long>(diag.stack[0].intactBytes), static_cast<unsigned long>(diag.stack[0].intactLowOffset), static_cast<unsigned long>(diag.stack[0].intactHighOffset),
               static_cast<unsigned long>(diag.stack[1].intactBytes), static_cast<unsigned long>(diag.stack[1].intactLowOffset), static_cast<unsigned long>(diag.stack[1].intactHighOffset));
      protocol::appendTiming(data,"loop",diag.loop);
      data.add(",\"loop_period_us\":%lu,\"loop_over_period\":%lu,\"run\":\"%s\",\"run_active\":%s",
               static_cast<unsigned long>(diag.loopPeriodUs), static_cast<unsigned long>(diag.loopOverPeriod), diag.run, diag.runActive ? "true" : "false");
      data.add(",\"loop_max_at\":{\"run_ms\":%lu,\"phase\":%d,\"usb_input\":%s}",
               static_cast<unsigned long>(diag.loopMaxRunMs), diag.loopMaxPhase, diag.loopMaxUsbInput ? "true" : "false");
      protocol::appendTiming(data,"step",diag.step); protocol::appendTiming(data,"compute",diag.compute); protocol::appendTiming(data,"storage",diag.storage);
      data.add(",\"over_budget\":%lu,\"frame_dt_us\":[%lu,%lu],\"sequence_gaps\":%lu,\"fifo_overruns\":%lu",
               static_cast<unsigned long>(diag.overBudget), static_cast<unsigned long>(diag.frameDtMinUs), static_cast<unsigned long>(diag.frameDtMaxUs),
               static_cast<unsigned long>(diag.sequenceGaps), static_cast<unsigned long>(diag.fifoOverruns));
      data.add(",\"reset_reason\":\"%s\",\"reset_bits\":[%lu,%lu]",diag.resetReason,
               static_cast<unsigned long>(diag.resetChipBits),static_cast<unsigned long>(diag.resetWatchdogBits));
      if (!diag.lastFault) data.add(",\"last_fault\":null");
      else data.add(",\"last_fault\":{\"core\":%lu,\"stack_overflow\":%s,\"cfsr\":%lu,\"hfsr\":%lu,\"sp\":%lu}",
                    static_cast<unsigned long>(diag.lastFaultCore), diag.lastFaultStackOverflow ? "true" : "false",
                    static_cast<unsigned long>(diag.lastFaultCfsr), static_cast<unsigned long>(diag.lastFaultHfsr), static_cast<unsigned long>(diag.lastFaultSp));
    }
    if (!identified && valid && status.diagnostics) { // Readable summary for a serial monitor.
      const auto &diag = *status.diagnostics;
      std::snprintf(reply, capacity,
          "Stack core 0: %lu/%lu bytes%s | core 1: %lu/%lu bytes%s\n"
          "Loop: %lu periods, max %lu us at %lu ms (phase %d%s), %lu over %lu us | Run: %s%s\n"
          "Step max %lu us, compute max %lu us, storage max %lu us, over budget %lu | Frame dt %lu..%lu us, gaps %lu, FIFO overruns %lu\n",
          static_cast<unsigned long>(diag.stack[0].usedBytes), static_cast<unsigned long>(diag.stack[0].capacityBytes), diag.stack[0].exhausted ? " EXHAUSTED" : "",
          static_cast<unsigned long>(diag.stack[1].usedBytes), static_cast<unsigned long>(diag.stack[1].capacityBytes), diag.stack[1].exhausted ? " EXHAUSTED" : "",
          static_cast<unsigned long>(diag.loop.count), static_cast<unsigned long>(diag.loop.maxUs), static_cast<unsigned long>(diag.loopMaxRunMs),
          diag.loopMaxPhase, diag.loopMaxUsbInput ? ", USB input" : "", static_cast<unsigned long>(diag.loopOverPeriod),
          static_cast<unsigned long>(diag.loopPeriodUs), diag.run, diag.runActive ? " (active)" : "",
          static_cast<unsigned long>(diag.step.maxUs), static_cast<unsigned long>(diag.compute.maxUs), static_cast<unsigned long>(diag.storage.maxUs),
          static_cast<unsigned long>(diag.overBudget), static_cast<unsigned long>(diag.frameDtMinUs), static_cast<unsigned long>(diag.frameDtMaxUs),
          static_cast<unsigned long>(diag.sequenceGaps), static_cast<unsigned long>(diag.fifoOverruns));
      return true;
    }
    return protocol::finish(command,data,!valid ? "INVALID_ARGUMENT" : !status.diagnostics ? "UNAVAILABLE" : nullptr,identified,id,reply,capacity);
  }
  const bool known = std::strcmp(command, "ping") == 0 || std::strcmp(command, "info") == 0 ||
                     std::strcmp(command, "status") == 0 || std::strcmp(command, "help") == 0 ||
                     std::strcmp(command, "sensors") == 0;
  if (!code && !known) { code = "UNKNOWN_COMMAND"; message = "Unknown command. Use help."; }
  if (!code && *arguments) { code = "INVALID_ARGUMENT"; message = "This command takes no arguments."; }
  if (code) {
    if (identified) std::snprintf(reply, capacity,
        "{\"id\":%lu,\"type\":\"error\",\"ok\":false,\"code\":\"%s\",\"message\":\"%s\"}\n",
        static_cast<unsigned long>(id), code, message);
    else std::snprintf(reply, capacity, "%s: %s\n", code, message);
    return true;
  }
  const char *imuState = status.imuReady ? "ready" : "unavailable";
  const char *baroState = status.baroReady ? "ready" : "unavailable";
  char pressure[32] = "null", temperature[32] = "null";
  if (status.baroSampleValid && std::isfinite(status.pressurePa)) std::snprintf(pressure, sizeof(pressure), "%.3f", double(status.pressurePa));
  if (status.baroSampleValid && std::isfinite(status.temperatureC)) std::snprintf(temperature, sizeof(temperature), "%.3f", double(status.temperatureC));
  char acceleration[96] = "null", angularRate[96] = "null";
  bool imuValid = status.imuReady && status.imuSampleValid;
  for (unsigned i=0;i<3;++i) imuValid = imuValid && std::isfinite(status.accelMps2[i]) && std::isfinite(status.gyroRadps[i]);
  if (imuValid && !std::strcmp(command,"sensors")) {
    std::snprintf(acceleration,sizeof(acceleration),"[%.6g,%.6g,%.6g]",double(status.accelMps2[0]),double(status.accelMps2[1]),double(status.accelMps2[2]));
    std::snprintf(angularRate,sizeof(angularRate),"[%.6g,%.6g,%.6g]",double(status.gyroRadps[0]),double(status.gyroRadps[1]),double(status.gyroRadps[2]));
  }
  const char *state = !std::strcmp(status.armingState,"armed") ? "Armed" : !std::strcmp(status.armingState,"arming") ? "Arming" : !std::strcmp(status.armingState,"closing") ? "Closing" : "Disarmed";
  char phase[8] = "null";
  const char *phaseNames[] = {"PAD","BOOST","COAST","DESCENT","LANDED","BOOST 2","COAST 2","ABORT"};
  const bool validPhase = status.phase >= 0 && status.phase <= 7;
  if (validPhase) std::snprintf(phase,sizeof(phase),"%d",status.phase);
  char attitude[176] = "null", tiltText[24] = "";
  if (status.attitudeValid) {
    std::snprintf(attitude,sizeof(attitude),"{\"tilt_deg\":%.1f,\"up_body\":[%.3f,%.3f,%.3f]}",
        double(status.tiltDeg),double(status.upBody[0]),double(status.upBody[1]),double(status.upBody[2]));
    std::snprintf(tiltText,sizeof(tiltText)," | Tilt: %.1f deg",double(status.tiltDeg));
  }
  if (status.attitudeValid && status.gyroOnlyValid) { // Test builds only (`nav gyro start`); absent otherwise.
    const std::size_t used=std::strlen(attitude)-1; // Before the closing brace.
    std::snprintf(attitude+used,sizeof(attitude)-used,",\"gyro_only\":{\"tilt_deg\":%.1f,\"up_body\":[%.3f,%.3f,%.3f]}}",
        double(status.gyroOnlyTiltDeg),double(status.gyroOnlyUpBody[0]),double(status.gyroOnlyUpBody[1]),double(status.gyroOnlyUpBody[2]));
  }
  const bool gyroSkipped = !std::strcmp(status.flightMode,"simulation");
  char battery[24]="null";
  if (status.batteryMv!=UINT16_MAX) std::snprintf(battery,sizeof(battery),"%.3f",double(status.batteryMv)/1000.0);
  char gyroProgress[80];
  if (gyroSkipped) std::snprintf(gyroProgress,sizeof(gyroProgress),"Gyro calibration: skipped (simulation)");
  else std::snprintf(gyroProgress,sizeof(gyroProgress),"Arming gyro samples: %lu/%lu",static_cast<unsigned long>(status.gyroSamples),static_cast<unsigned long>(status.gyroTarget));
  if (!identified) {
    if (std::strcmp(command, "ping") == 0) std::snprintf(reply, capacity, "PONG\n");
    else if (std::strcmp(command, "help") == 0) std::snprintf(reply, capacity,
        "Commands: ping, info, status, sensors, diag, help; sleep, wake; bluetooth status, bluetooth name <suffix|default>; control status; preflight [physical|simulation], arm, disarm (ds); imu status/begin/verify/capture <axis>/cancel; capabilities; config machine [normal|bistage <m/s>]; config show/begin/draft/validate/apply/discard/reset/orientation [+X|-X|+Y|-Y|+Z|-Z]; rule, condition, output, pyro, servo (servo arm [channel] prepares outputs; servo release releases held PWM); replay begin/sample/status/stop (estimates); hil begin/sample/status/stop/timing (sensors); sim begin [physical] [record]/status/stop/timing/events; flights status/init/recover/format/list/info/summary/calibration/config/block/delete/timing/failure; flights timing reset. LF/CRLF; @<uint32> for JSON.\n");
    else if (std::strcmp(command, "info") == 0) std::snprintf(reply, capacity,
        "Perpetual | RP2350 | USB/Bluetooth command service v1 | Readiness: use preflight\n");
    else if (std::strcmp(command, "sensors") == 0) std::snprintf(reply, capacity,
        "IMU: %s | ID: 0x%02X | Error: %ld\nBarometer: %s | ID: 0x%02X | Error: %ld\nPressure: %s Pa | Temperature: %s C | Sample valid: %s | Age: %lu ms\nAcceleration: %s m/s2 | Angular rate: %s rad/s | IMU sample valid: %s | Age: %lu ms\nPhysical arming readiness: use preflight.\n",
        imuState, unsigned(status.imuId), static_cast<long>(status.imuError), baroState, unsigned(status.baroId), static_cast<long>(status.baroError),
        pressure, temperature, status.baroSampleValid ? "yes" : "no", static_cast<unsigned long>(status.baroSampleAgeMs),
        acceleration,angularRate,imuValid ? "yes" : "no",static_cast<unsigned long>(status.imuSampleAgeMs));
    else std::snprintf(reply, capacity,
        "%s | Flight mode: %s | Phase: %s%s\n"
        "Configuration: %s (%s) | Readiness: use preflight\n"
        "IMU: %s | Barometer: %s | Recording storage: %s\n"
        "%s\n"
        "Flash JEDEC: 0x%06lX (%s; identification only)\n"
        "Outputs: %s (%s) | Replay: %s | Power: %s (%s) | Battery: %s V | Uptime: %lu ms\n",
        state, status.flightMode, validPhase ? phaseNames[status.phase] : "unavailable", tiltText,
        status.configValid ? "valid" : "not loaded", status.configError, imuState, baroState, status.recordingReady ? "ready for HIL" : "not initialized",
        gyroProgress,
        static_cast<unsigned long>(status.flashJedec), status.flashIdentified ? "match" : "mismatch",
        status.outputsActive ? "active" : "inactive", status.outputMode,
        status.simulationActive ? "onboard simulation" : status.sensorHilActive ? "sensor HIL (VIRTUAL)" : (status.estimateReplayActive ? "estimates (VIRTUAL)" : "inactive"),
        status.powerMode,status.powerError,battery,static_cast<unsigned long>(status.uptimeMs));
  } else {
    char payload[832] = {};
    if (std::strcmp(command, "ping") == 0) std::snprintf(payload, sizeof(payload), "\"message\":\"PONG\"");
    else if (std::strcmp(command, "help") == 0) std::snprintf(payload, sizeof(payload),
        "\"commands\":[\"ping\",\"info\",\"status\",\"sensors\",\"diag\",\"help\",\"sleep\",\"wake\",\"bluetooth status\",\"control status\"],\"max_line_bytes\":128,\"id_bits\":32");
    else if (std::strcmp(command, "info") == 0) std::snprintf(payload, sizeof(payload),
        "\"device\":\"Perpetual\",\"mcu\":\"RP2350\",\"protocol_version\":1");
    else if (std::strcmp(command, "sensors") == 0) std::snprintf(payload, sizeof(payload),
        "\"imu_ready\":%s,\"imu_id\":%u,\"imu_error\":%ld,\"baro_ready\":%s,\"baro_id\":%u,\"baro_error\":%ld,\"baro_sample_valid\":%s,\"pressure_pa\":%s,\"temperature_c\":%s,\"sample_age_ms\":%lu,\"imu_sample_valid\":%s,\"imu_sample_age_ms\":%lu,\"accel_mps2\":%s,\"gyro_radps\":%s,\"uptime_ms\":%lu,\"idle_poll\":[%lu,%lu,%lu],\"idle_samples\":[%lu,%lu],\"idle_gap\":[%lu,%lu],\"idle_imu\":[%lu,%lu,%lu,%lu,%lu]",
        status.imuReady ? "true" : "false", unsigned(status.imuId), static_cast<long>(status.imuError),
        status.baroReady ? "true" : "false", unsigned(status.baroId), static_cast<long>(status.baroError), status.baroSampleValid ? "true" : "false",
        pressure, temperature, static_cast<unsigned long>(status.baroSampleAgeMs),
        imuValid ? "true" : "false",static_cast<unsigned long>(status.imuSampleAgeMs),acceleration,angularRate,static_cast<unsigned long>(status.uptimeMs),
        static_cast<unsigned long>(status.idlePolls),static_cast<unsigned long>(status.idleReadUs),static_cast<unsigned long>(status.idleReadMaxUs),
        static_cast<unsigned long>(status.idleImuSamples),static_cast<unsigned long>(status.idleBaroSamples),
        static_cast<unsigned long>(status.idleLateGaps),static_cast<unsigned long>(status.idleGapMaxUs),
        static_cast<unsigned long>(status.idleDtMinUs),static_cast<unsigned long>(status.idleDtMaxUs),static_cast<unsigned long>(status.idleSequenceGaps),
        static_cast<unsigned long>(status.imuFifoOverruns),static_cast<unsigned long>(status.imuFifoErrors));
    else std::snprintf(payload, sizeof(payload),
        "\"mode\":\"%s\",\"arming_state\":\"%s\",\"flight_mode\":\"%s\",\"phase\":%s,\"attitude\":%s,\"gyro_calibration_skipped\":%s,\"gyro_samples\":%lu,\"gyro_target\":%lu,\"config_valid\":%s,\"config_error\":\"%s\","
        "\"imu_ready\":%s,\"baro_ready\":%s,"
        "\"estimate_replay_active\":%s,\"sensor_hil_active\":%s,\"simulation_active\":%s,\"flash_driver_ready\":%s,"
        "\"hil_recording_ready\":%s,\"outputs_active\":%s,\"output_mode\":\"%s\",\"flash_jedec\":%lu,\"flash_identified\":%s,\"power_mode\":\"%s\",\"power_error\":\"%s\",\"battery_v\":%s,\"uptime_ms\":%lu",
        status.armingState, status.armingState, status.flightMode, phase, attitude, gyroSkipped ? "true" : "false", static_cast<unsigned long>(status.gyroSamples), static_cast<unsigned long>(status.gyroTarget),
        status.configValid ? "true" : "false", status.configError,
        status.imuReady ? "true" : "false", status.baroReady ? "true" : "false",
        status.estimateReplayActive ? "true" : "false", status.sensorHilActive ? "true" : "false", status.simulationActive ? "true" : "false", status.flashReady ? "true" : "false", status.recordingReady ? "true" : "false", status.outputsActive ? "true" : "false", status.outputMode, static_cast<unsigned long>(status.flashJedec), status.flashIdentified ? "true" : "false",
        status.powerMode,status.powerError,battery,static_cast<unsigned long>(status.uptimeMs));
    std::snprintf(reply, capacity, "{\"id\":%lu,\"type\":\"reply\",\"command\":\"%s\",\"ok\":true,%s}\n",
        static_cast<unsigned long>(id), command, payload);
  }
  return true;
}
