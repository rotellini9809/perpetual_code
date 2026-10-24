#include "communication/CommandService.h"
#include "communication/CommandProtocol.h"
#include <cstdio>
#include <cstring>
#include <limits>
#include <cmath>

namespace { bool space(char c) { return c == ' ' || c == '\t'; } }
void CommandService::reset() { length_ = 0; overflow_ = invalid_ = false; haveId_ = false; }
bool CommandService::feed(char byte, const Status &s, char *out, std::size_t cap) {
  if (out == nullptr || cap < MaxReply) return false;
  if (byte == '\n') {
    line_[length_] = 0;
    char request[MaxLine + 1]; memcpy(request, line_, length_ + 1);
    bool tracked = false; uint32_t requestId = 0;
    if (extension_ && !invalid_ && !overflow_) {
      const char *p = request; while (space(*p)) ++p;
      if (*p++ == '@') {
        tracked = *p >= '0' && *p <= '9';
        while (*p >= '0' && *p <= '9') {
          const uint32_t digit = uint32_t(*p++ - '0');
          if (requestId > (std::numeric_limits<uint32_t>::max() - digit) / 10) tracked = false;
          if (tracked) requestId = requestId * 10 + digit;
        }
        tracked = tracked && space(*p);
      }
    }
    if (tracked && haveId_ && requestId <= lastId_) {
      if (requestId == lastId_ && strcmp(request, lastRequest_) == 0) strcpy(out, lastReply_);
      else std::snprintf(out, cap, "{\"id\":%lu,\"type\":\"error\",\"ok\":false,\"code\":\"%s\"}\n",
                         static_cast<unsigned long>(requestId), requestId == lastId_ ? "ID_CONFLICT" : "STALE_REQUEST");
      length_ = 0; overflow_ = invalid_ = false; return true;
    }
    const bool ready = dispatch(s, out, cap);
    if (ready && tracked) {
      haveId_ = true; lastId_ = requestId; strcpy(lastRequest_, request); strcpy(lastReply_, out);
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
bool CommandService::dispatch(const Status &s, char *out, std::size_t cap) {
  if (length_ && line_[length_ - 1] == '\r') --length_;
  line_[length_] = '\0';
  char *p = line_;
  while (space(*p)) ++p;
  bool identified = false;
  uint32_t id = 0;
  const char *code = nullptr;
  const char *message = nullptr;
  if (*p == '@') {
    ++p;
    bool valid = *p >= '0' && *p <= '9';
    while (*p >= '0' && *p <= '9') {
      const uint32_t digit = static_cast<uint32_t>(*p++ - '0');
      if (id > (std::numeric_limits<uint32_t>::max() - digit) / 10) valid = false;
      if (valid) id = id * 10 + digit;
    }
    if (!valid || !space(*p)) { code = "INVALID_ARGUMENT"; message = "Malformed request ID."; }
    else { identified = true; while (space(*p)) ++p; }
  }
  if (!code && overflow_) { code = "LINE_TOO_LONG"; message = "Line discarded through LF."; }
  if (!code && (invalid_ || std::strchr(p, '\r'))) {
    code = "INVALID_ARGUMENT"; message = "Invalid command characters.";
  }
  if (!code && !*p && !identified) return false;
  char *command = p;
  while (*p && !space(*p)) ++p;
  if (*p) *p++ = '\0';
  while (space(*p)) ++p;
  if (!code && extension_ && extension_->dispatch(command, p, identified, id, out, cap)) return true;
  if (!code && !std::strcmp(command,"bluetooth")) {
    protocol::Tokens t(p); protocol::Json data;
    data.add(",\"ready\":%s,\"connected\":%s,\"error\":\"%s\",\"baud\":9600",
             s.bluetoothReady ? "true" : "false", s.bluetoothConnected ? "true" : "false", s.bluetoothError);
    return protocol::finish(command,data,t.valid && t.count == 1 && t.is(0,"status") ? nullptr : "INVALID_ARGUMENT",identified,id,out,cap);
  }
  const bool known = std::strcmp(command, "ping") == 0 || std::strcmp(command, "info") == 0 ||
                     std::strcmp(command, "status") == 0 || std::strcmp(command, "help") == 0 ||
                     std::strcmp(command, "sensors") == 0;
  if (!code && !known) { code = "UNKNOWN_COMMAND"; message = "Unknown command. Use help."; }
  if (!code && *p) { code = "INVALID_ARGUMENT"; message = "This command takes no arguments."; }
  if (code) {
    if (identified) std::snprintf(out, cap,
        "{\"id\":%lu,\"type\":\"error\",\"ok\":false,\"code\":\"%s\",\"message\":\"%s\"}\n",
        static_cast<unsigned long>(id), code, message);
    else std::snprintf(out, cap, "%s: %s\n", code, message);
    return true;
  }
  const char *imuState = s.imuReady ? "ready" : "unavailable";
  const char *baroState = s.baroReady ? "ready" : "unavailable";
  char pressure[32] = "null", temperature[32] = "null";
  if (s.baroSampleValid && std::isfinite(s.pressurePa)) std::snprintf(pressure, sizeof(pressure), "%.3f", double(s.pressurePa));
  if (s.baroSampleValid && std::isfinite(s.temperatureC)) std::snprintf(temperature, sizeof(temperature), "%.3f", double(s.temperatureC));
  char acceleration[96] = "null", angularRate[96] = "null";
  bool imuValid = s.imuReady && s.imuSampleValid;
  for (unsigned i=0;i<3;++i) imuValid = imuValid && std::isfinite(s.accelMps2[i]) && std::isfinite(s.gyroRadps[i]);
  if (imuValid && !std::strcmp(command,"sensors")) {
    std::snprintf(acceleration,sizeof(acceleration),"[%.6g,%.6g,%.6g]",double(s.accelMps2[0]),double(s.accelMps2[1]),double(s.accelMps2[2]));
    std::snprintf(angularRate,sizeof(angularRate),"[%.6g,%.6g,%.6g]",double(s.gyroRadps[0]),double(s.gyroRadps[1]),double(s.gyroRadps[2]));
  }
  const char *state = !std::strcmp(s.armingState,"armed") ? "Armed" : !std::strcmp(s.armingState,"arming") ? "Arming" : !std::strcmp(s.armingState,"closing") ? "Closing" : "Disarmed";
  char phase[8] = "null";
  const char *phaseNames[] = {"PAD","BOOST","COAST","DESCENT","LANDED"};
  const bool validPhase = s.phase >= 0 && s.phase <= 4;
  if (validPhase) std::snprintf(phase,sizeof(phase),"%d",s.phase);
  const bool gyroSkipped = !std::strcmp(s.flightMode,"simulation");
  char battery[24]="null";
  if (s.batteryMv!=UINT16_MAX) std::snprintf(battery,sizeof(battery),"%.3f",double(s.batteryMv)/1000.0);
  char gyroProgress[80];
  if (gyroSkipped) std::snprintf(gyroProgress,sizeof(gyroProgress),"Gyro calibration: skipped (simulation)");
  else std::snprintf(gyroProgress,sizeof(gyroProgress),"Arming gyro samples: %lu/%lu",static_cast<unsigned long>(s.gyroSamples),static_cast<unsigned long>(s.gyroTarget));
  if (!identified) {
    if (std::strcmp(command, "ping") == 0) std::snprintf(out, cap, "PONG\n");
    else if (std::strcmp(command, "help") == 0) std::snprintf(out, cap,
        "Commands: ping, info, status, sensors, help; sleep, wake; bluetooth status; control status; preflight [physical|simulation], arm, disarm (ds); capabilities; config show/begin/draft/validate/apply/discard; rule, condition, output, pyro, servo (servo arm [channel] prepares outputs); replay begin/sample/status/stop (estimates); hil begin/sample/status/stop/timing (sensors); sim begin [physical] [record]/status/stop/timing/events; flights status/init/recover/list/info/summary/config/read/delete/timing/failure; flights timing reset. LF/CRLF; @<uint32> for JSON.\n");
    else if (std::strcmp(command, "info") == 0) std::snprintf(out, cap,
        "Perpetual | RP2350 | USB/Bluetooth command service v1 | Readiness: use preflight\n");
    else if (std::strcmp(command, "sensors") == 0) std::snprintf(out, cap,
        "IMU: %s | ID: 0x%02X | Error: %ld\nBarometer: %s | ID: 0x%02X | Error: %ld\nPressure: %s Pa | Temperature: %s C | Sample valid: %s | Age: %lu ms\nAcceleration: %s m/s2 | Angular rate: %s rad/s | IMU sample valid: %s | Age: %lu ms\nPhysical arming readiness: use preflight.\n",
        imuState, unsigned(s.imuId), static_cast<long>(s.imuError), baroState, unsigned(s.baroId), static_cast<long>(s.baroError),
        pressure, temperature, s.baroSampleValid ? "yes" : "no", static_cast<unsigned long>(s.baroSampleAgeMs),
        acceleration,angularRate,imuValid ? "yes" : "no",static_cast<unsigned long>(s.imuSampleAgeMs));
    else std::snprintf(out, cap,
        "%s | Flight mode: %s | Phase: %s\n"
        "Configuration: %s (%s) | Readiness: use preflight\n"
        "IMU: %s | Barometer: %s | Recording storage: %s\n"
        "%s\n"
        "Flash JEDEC: 0x%06lX (%s; identification only)\n"
        "Outputs: %s (%s) | Replay: %s | Power: %s (%s) | Battery: %s V | Uptime: %lu ms\n",
        state, s.flightMode, validPhase ? phaseNames[s.phase] : "unavailable",
        s.configValid ? "valid" : "not loaded", s.configError, imuState, baroState, s.recordingReady ? "ready for HIL" : "not initialized",
        gyroProgress,
        static_cast<unsigned long>(s.flashJedec), s.flashIdentified ? "match" : "mismatch",
        s.outputsActive ? "active" : "inactive", s.outputMode,
        s.simulationActive ? "onboard simulation" : s.sensorHilActive ? "sensor HIL (VIRTUAL)" : (s.estimateReplayActive ? "estimates (VIRTUAL)" : "inactive"),
        s.powerMode,s.powerError,battery,static_cast<unsigned long>(s.uptimeMs));
  } else {
    char payload[832] = {};
    if (std::strcmp(command, "ping") == 0) std::snprintf(payload, sizeof(payload), "\"message\":\"PONG\"");
    else if (std::strcmp(command, "help") == 0) std::snprintf(payload, sizeof(payload),
        "\"commands\":[\"ping\",\"info\",\"status\",\"sensors\",\"help\",\"sleep\",\"wake\",\"bluetooth status\",\"control status\"],\"max_line_bytes\":128,\"id_bits\":32");
    else if (std::strcmp(command, "info") == 0) std::snprintf(payload, sizeof(payload),
        "\"device\":\"Perpetual\",\"mcu\":\"RP2350\",\"protocol_version\":1");
    else if (std::strcmp(command, "sensors") == 0) std::snprintf(payload, sizeof(payload),
        "\"imu_ready\":%s,\"imu_id\":%u,\"imu_error\":%ld,\"baro_ready\":%s,\"baro_id\":%u,\"baro_error\":%ld,\"baro_sample_valid\":%s,\"pressure_pa\":%s,\"temperature_c\":%s,\"sample_age_ms\":%lu,\"imu_sample_valid\":%s,\"imu_sample_age_ms\":%lu,\"accel_mps2\":%s,\"gyro_radps\":%s,\"uptime_ms\":%lu",
        s.imuReady ? "true" : "false", unsigned(s.imuId), static_cast<long>(s.imuError),
        s.baroReady ? "true" : "false", unsigned(s.baroId), static_cast<long>(s.baroError), s.baroSampleValid ? "true" : "false",
        pressure, temperature, static_cast<unsigned long>(s.baroSampleAgeMs),
        imuValid ? "true" : "false",static_cast<unsigned long>(s.imuSampleAgeMs),acceleration,angularRate,static_cast<unsigned long>(s.uptimeMs));
    else std::snprintf(payload, sizeof(payload),
        "\"mode\":\"%s\",\"arming_state\":\"%s\",\"flight_mode\":\"%s\",\"phase\":%s,\"gyro_calibration_skipped\":%s,\"gyro_samples\":%lu,\"gyro_target\":%lu,\"config_valid\":%s,\"config_error\":\"%s\","
        "\"imu_ready\":%s,\"baro_ready\":%s,"
        "\"estimate_replay_active\":%s,\"sensor_hil_active\":%s,\"simulation_active\":%s,\"flash_driver_ready\":%s,"
        "\"hil_recording_ready\":%s,\"outputs_active\":%s,\"output_mode\":\"%s\",\"flash_jedec\":%lu,\"flash_identified\":%s,\"power_mode\":\"%s\",\"power_error\":\"%s\",\"battery_v\":%s,\"uptime_ms\":%lu",
        s.armingState, s.armingState, s.flightMode, phase, gyroSkipped ? "true" : "false", static_cast<unsigned long>(s.gyroSamples), static_cast<unsigned long>(s.gyroTarget),
        s.configValid ? "true" : "false", s.configError,
        s.imuReady ? "true" : "false", s.baroReady ? "true" : "false",
        s.estimateReplayActive ? "true" : "false", s.sensorHilActive ? "true" : "false", s.simulationActive ? "true" : "false", s.flashReady ? "true" : "false", s.recordingReady ? "true" : "false", s.outputsActive ? "true" : "false", s.outputMode, static_cast<unsigned long>(s.flashJedec), s.flashIdentified ? "true" : "false",
        s.powerMode,s.powerError,battery,static_cast<unsigned long>(s.uptimeMs));
    std::snprintf(out, cap, "{\"id\":%lu,\"type\":\"reply\",\"command\":\"%s\",\"ok\":true,%s}\n",
        static_cast<unsigned long>(id), command, payload);
  }
  return true;
}
