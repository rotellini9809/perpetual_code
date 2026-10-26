#pragma once
#include <cstdint>

namespace recovery {
// New mapping: never cast the legacy APOGEE/CHUTE_DEPLOYED enum into this type.
enum class FlightPhase : uint8_t { Pad = 0, Boost = 1, Coast = 2, Descent = 3, Landed = 4, Boost2 = 5, Coast2 = 6, Abort = 7 };
constexpr uint8_t BoostMask = 1u << 1;
constexpr uint8_t CoastMask = 1u << 2;
constexpr uint8_t DescentMask = 1u << 3;
constexpr uint8_t Boost2Mask = 1u << 5, Coast2Mask = 1u << 6;
constexpr uint8_t AirborneMask = BoostMask | CoastMask | DescentMask | Boost2Mask | Coast2Mask;
inline bool boostPhase(FlightPhase phase) { return phase==FlightPhase::Boost || phase==FlightPhase::Boost2; }
inline bool coastPhase(FlightPhase phase) { return phase==FlightPhase::Coast || phase==FlightPhase::Coast2; }
inline bool airbornePhase(FlightPhase phase) { return boostPhase(phase) || coastPhase(phase) || phase==FlightPhase::Descent; }
enum class AbortReason : uint8_t { None, Velocity, Tilt };

struct StateMachineSettings {
  bool bistage = false;
  float minimumCoastVelocity = 0;
};

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
  bool startsAllowed = true;
  uint64_t phaseMeasuredMs = 0;
  uint64_t sequence = 0;
  Estimate altitude;
  Estimate verticalVelocity;
};
} // namespace recovery
