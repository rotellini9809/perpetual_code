#pragma once

#include "shared/FlightTypes.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace recovery {

constexpr std::size_t ChannelCount = 4;
// Initial bounded RAM capacities, exposed for future capabilities reporting.
constexpr std::size_t MaxRules = 8;
constexpr std::size_t MaxConditions = 8;
constexpr std::size_t MaxAssignments = 8;
constexpr std::size_t MaxNameBytes = 48;
using RuleId = uint32_t; // Zero is reserved for an unset reference.

enum class OutputKind : uint8_t { Pyro, Servo };
enum class ConditionKind : uint8_t {
  Phase, ApogeeDetected, AltitudeComparison, AltitudeRange,
  AbsoluteVelocityComparison, AbsoluteVelocityRange,
  TimeSinceLaunch, TimeSinceApogee, TimeSinceRule
};
enum class Comparison : uint8_t { Less, Greater, LessEqual, GreaterEqual };

struct PyroSettings { uint32_t pulseMs = 0; };
struct ServoSettings {
  bool calibrated = false;
  uint16_t initialUs = 0;
  uint16_t finalUs = 0;
};

// Tagged parameters: only fields belonging to kind are meaningful.
// Heights are launch-relative metres; velocity conditions use abs(vz) in m/s.
// Time values are integer milliseconds, including event-relative thresholds.
struct Condition {
  ConditionKind kind = ConditionKind::ApogeeDetected;
  Comparison comparison = Comparison::Less;
  float value = 0;
  float upper = 0;
  uint8_t phases = 0;
  uint32_t elapsedMs = 0;
  RuleId reference = 0;
};
struct Assignment {
  OutputKind kind = OutputKind::Pyro;
  uint8_t channel = 0; // One-based, matching current hardware controllers.
  uint32_t delayMs = 0; // Additional wait after the rule's delay, not pulse duration.
};
struct Rule {
  RuleId id = 0;
  std::array<char, MaxNameBytes> name{};
  uint32_t delayMs = 0;
  std::size_t conditionCount = 0;
  std::array<Condition, MaxConditions> conditions{};
  std::size_t assignmentCount = 0;
  std::array<Assignment, MaxAssignments> assignments{};
};
struct Configuration {
  std::array<PyroSettings, ChannelCount> pyros{};
  std::array<ServoSettings, ChannelCount> servos{};
  std::size_t ruleCount = 0;
  std::array<Rule, MaxRules> rules{};
};

// Board/application policy supplied explicitly; these are not recommended flight values.
struct ValidationLimits {
  uint16_t minServoUs;
  uint16_t maxServoUs;
  uint32_t maxPyroPulseMs;
  uint32_t maxTimingMs;
};
enum class ValidationError : uint8_t {
  None, InvalidLimits, EmptyPlan, TooManyRules, InvalidRuleId, DuplicateRuleId,
  InvalidName, InvalidConditionCount, InvalidAssignmentCount, InvalidCondition,
  InvalidReference, DependencyCycle, InvalidOutput, DuplicateOutput,
  InvalidPyroSettings, InvalidServoSettings, InvalidTiming
};
enum class Field : uint8_t { Plan, Rule, Condition, Assignment, Pyro, Servo };
struct ValidationResult {
  ValidationError error = ValidationError::None;
  Field field = Field::Plan;
  RuleId rule = 0;
  std::size_t index = 0; // Zero-based condition/assignment or actuator index.
  bool ok() const { return error == ValidationError::None; }
};

// Pure, allocation-free validation. Returns the first actionable failure.
ValidationResult validate(const Configuration &config, const ValidationLimits &limits);
const char *errorName(ValidationError error);

} // namespace recovery
