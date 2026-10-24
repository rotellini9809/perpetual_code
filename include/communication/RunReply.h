#pragma once
#include "communication/CommandProtocol.h"
#include "flight/RunController.h"

namespace protocol {
inline void appendRunEvents(Json &json, const flight::RunEvaluation &result) {
  json.add(",\"triggered\":[");
  for (std::size_t i = 0; i < result.triggerCount; ++i)
    json.add("%s%lu", i ? "," : "", static_cast<unsigned long>(result.triggered[i]));
  json.add("],\"starts\":[");
  for (std::size_t i = 0; i < result.startCount; ++i) {
    const auto &a = result.starts[i];
    json.add("%s[%lu,\"%s\",%u,%lu]", i ? "," : "", static_cast<unsigned long>(a.rule),
             a.kind == recovery::OutputKind::Pyro ? "pyro" : "servo", a.channel, static_cast<unsigned long>(a.timeMs));
  }
  json.add("]");
}
}
