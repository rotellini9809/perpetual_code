#pragma once
#include "communication/CommandProtocol.h"
#include "flight/ArmingController.h"

namespace protocol {
inline void appendPreflight(Json &json, const flight::Preflight &report) {
  json.add(",\"ready\":%s,\"reasons\":[", report.ready() ? "true" : "false");
  bool comma=false;
  for (unsigned i=0;i<unsigned(flight::ArmBlocker::Count);++i) {
    if (!report.has(flight::ArmBlocker(i))) continue;
    json.add("%s\"%s\"",comma ? "," : "",flight::blockerName(flight::ArmBlocker(i))); comma=true;
  }
  json.add("],\"failed_pyro_mask\":%u,\"unprepared_servo_mask\":%u,\"unconfigured_servo_mask\":%u,\"prepared_servo_mask\":%u",
           unsigned(report.failedPyros),unsigned(report.unpreparedServos),unsigned(report.unconfiguredServos),unsigned(report.preparedServos));
}
}
