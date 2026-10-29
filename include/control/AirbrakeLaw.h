#pragma once
#include "shared/FlightTypes.h"
#include <array>
#include <cstdint>

namespace control {
constexpr uint32_t AirbrakeLawVersion = 1; // Bump when the compiled law/state changes.
struct ControlInput {
  std::array<float,3> state{}; // z [m], vz [m/s], vertical acceleration bias [m/s^2].
  std::array<std::array<float,3>,3> covariance{}; // Same posterior update as state.
  float acceleration = 0; // Net vertical acceleration used in prediction, m/s^2.
  uint64_t timeUs = 0;
  float dtSeconds = 0;
  recovery::FlightPhase phase = recovery::FlightPhase::Pad;
  bool valid = false;
};
struct ControlOutput {
  std::array<float,4> opening{}; // Normalized calibrated travel [0,1]; no PWM API.
  bool valid = true;
};
// User-owned bounded controller memory. Reset on every disabled -> enabled edge.
struct LawState { std::array<float,4> memory{}; };
using UpdateLaw = ControlOutput (*)(const ControlInput &, LawState &);
ControlOutput updateAirbrakes(const ControlInput &input, LawState &state);
}
