#pragma once
#include <cstdint>

namespace recovery {
// New mapping: never cast the legacy APOGEE/CHUTE_DEPLOYED enum into this type.
enum class FlightPhase : uint8_t { Pad = 0, Boost = 1, Coast = 2, Descent = 3, Landed = 4 };
constexpr uint8_t BoostMask = 1u << 1;
constexpr uint8_t CoastMask = 1u << 2;
constexpr uint8_t DescentMask = 1u << 3;
constexpr uint8_t AirborneMask = BoostMask | CoastMask | DescentMask;

// Monotonic milliseconds within one flight; caller must extend hardware timer rollover.
struct Estimate {
  float value = 0;
  bool valid = false;
  uint64_t measuredMs = 0;
  uint64_t sequence = 0;
};
struct FlightSnapshot {
  FlightPhase phase = FlightPhase::Pad;
  bool phaseValid = false;
  uint64_t phaseMeasuredMs = 0;
  uint64_t sequence = 0;
  Estimate altitude;
  Estimate verticalVelocity;
};
} // namespace recovery
