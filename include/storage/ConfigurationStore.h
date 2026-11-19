#pragma once
#include "storage/FlashLayout.h"
#include "storage/FlashDevice.h"
#include "storage/DeviceJournal.h"
#include "configuration/RecoveryConfiguration.h"

namespace storage {
enum class ConfigError : uint8_t { None, Armed, InvalidConfiguration, Io, Missing, UnsupportedVersion, Corrupt };
class ConfigurationStore {
public:
  // Encoded plan limit; the journal record adds a 2-byte codec version.
  static constexpr std::size_t MaxPayload = DeviceJournal::MaxPayload - 2;
  static constexpr uint16_t FormatVersion = 3;
  explicit ConfigurationStore(FlashDevice &flash) : journal_(flash) {}
  // Output is assigned only after integrity, version and semantic validation.
  bool load(recovery::Configuration &output, const recovery::ValidationLimits &limits);
  // Explicit operation only; caller supplies trusted runtime disarmed state.
  // Written beside the previous copy in the device journal: a power cut keeps the old one.
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
  bool readSetting(DeviceJournal::Type type, std::size_t length, bool &present);
  bool writeSetting(DeviceJournal::Type type, const uint8_t *payload, std::size_t length);
  DeviceJournal journal_;
  uint8_t scratch_[DeviceJournal::MaxPayload] = {};
  recovery::Configuration decoded_{}; // Avoid stacking multiple large plans on RP2350.
  ConfigError error_ = ConfigError::None;
  recovery::ValidationResult validation_{};
};
}
