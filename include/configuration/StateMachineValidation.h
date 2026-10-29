#pragma once
#include "shared/FlightTypes.h"
#include <cmath>

namespace recovery {
constexpr float MaximumMinimumCoastVelocity = 100000.0f;
// Shared device/run-snapshot validity; missing-record policy belongs to the store.
inline bool validStateMachineSettings(const StateMachineSettings &settings) {
  return std::isfinite(settings.minimumCoastVelocity) && settings.minimumCoastVelocity >= 0 &&
         settings.minimumCoastVelocity <= MaximumMinimumCoastVelocity;
}
}
