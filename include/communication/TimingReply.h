#pragma once
#include "communication/CommandProtocol.h"
#include "shared/Timing.h"

namespace protocol {
// Compact statistics: [operation count, total microseconds, max microseconds].
inline void appendTiming(Json &data, const char *name, const timing::Statistics &stats) {
  data.add(",\"%s\":[%lu,%llu,%lu]", name, static_cast<unsigned long>(stats.count),
           static_cast<unsigned long long>(stats.totalUs), static_cast<unsigned long>(stats.maxUs));
}
}
