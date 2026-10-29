#pragma once
#include "storage/FlashLayout.h"
#include "storage/FlashDevice.h"
#include "configuration/RecoveryConfiguration.h"

namespace storage {
enum class ConfigError : uint8_t { None, Armed, InvalidConfiguration, Io, Missing, UnsupportedVersion, Corrupt };
class ConfigurationStore {
public:
  static constexpr uint32_t Begin = FlashLayout::ConfigurationBegin;
  static constexpr uint32_t Bytes = FlashLayout::ConfigurationBytes;
  static constexpr std::size_t MaxPayload = 4096;
  static constexpr uint16_t FormatVersion = 3;
  explicit ConfigurationStore(FlashDevice &flash) : flash_(flash) {}
  // Output is assigned only after integrity, version and semantic validation.
  bool load(recovery::Configuration &output, const recovery::ValidationLimits &limits);
  // Explicit operation only; caller supplies trusted runtime disarmed state.
  // An interrupted save may destroy the old copy. No fallback or active defaults.
  bool save(const recovery::Configuration &config, const recovery::ValidationLimits &limits, bool disarmed);
  bool clear(bool disarmed);
  bool loadStateMachine(recovery::StateMachineSettings &settings);
  bool saveStateMachine(const recovery::StateMachineSettings &settings, bool disarmed);
  bool loadAirbrakes(bool &enabled);
  bool saveAirbrakes(bool enabled, bool disarmed);
  bool loadOrientation(uint8_t &axis);
  bool saveOrientation(uint8_t axis, bool disarmed);
  ConfigError error() const { return error_; }
  recovery::ValidationResult validation() const { return validation_; }
  static const char *errorName(ConfigError error);
  static bool encode(const recovery::Configuration &config, uint8_t *output, std::size_t capacity, std::size_t &length);
  static bool decode(const uint8_t *bytes, std::size_t length, recovery::Configuration &output);
private:
  bool fail(ConfigError e) { error_ = e; return false; }
  FlashDevice &flash_;
  uint8_t scratch_[MaxPayload] = {};
  recovery::Configuration decoded_{}; // Avoid stacking multiple large plans on RP2350.
  ConfigError error_ = ConfigError::None;
  recovery::ValidationResult validation_{};
};
}
