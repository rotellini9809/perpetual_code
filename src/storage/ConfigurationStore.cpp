#include "configuration/StateMachineValidation.h"
#include "storage/ConfigurationStore.h"
#include "storage/BinaryEncoding.h"
#include <cstring>
#include <limits>
#include <cmath>

namespace storage {
bool ConfigurationStore::readSetting(DeviceJournal::Type type, std::size_t length, bool &present) {
  DeviceJournal::Found found;
  if (!journal_.read(type, scratch_, sizeof(scratch_), found)) return fail(ConfigError::Io);
  // A committed setting damaged later is never silently replaced by an older copy.
  if (found.corruptAfter) return fail(ConfigError::Corrupt);
  present = found.present;
  if (present && found.length != length) return fail(ConfigError::Corrupt);
  return true;
}
bool ConfigurationStore::writeSetting(DeviceJournal::Type type, const uint8_t *payload, std::size_t length) {
  if (!journal_.write(type, payload, length)) return fail(ConfigError::Io);
  error_ = ConfigError::None; return true;
}
bool ConfigurationStore::loadAirbrakes(bool &enabled) {
  bool present = false;
  if (!readSetting(DeviceJournal::Airbrakes, 1, present)) return false;
  if (present && scratch_[0] > 1) return fail(ConfigError::Corrupt);
  enabled = present && scratch_[0] == 1; error_ = ConfigError::None; return true;
}
bool ConfigurationStore::saveAirbrakes(bool enabled, bool disarmed) {
  if (!disarmed) return fail(ConfigError::Armed);
  const uint8_t value = enabled; return writeSetting(DeviceJournal::Airbrakes, &value, 1);
}
bool ConfigurationStore::loadStateMachine(recovery::StateMachineSettings &output) {
  bool present = false;
  if (!readSetting(DeviceJournal::StateMachine, 5, present)) return false;
  if (!present) { output = {}; error_ = ConfigError::None; return true; }
  recovery::StateMachineSettings decoded; decoded.bistage = scratch_[0] == 1;
  const uint32_t bits = binary::get32(scratch_ + 1, 4); memcpy(&decoded.minimumCoastVelocity, &bits, 4);
  if (scratch_[0] > 1 || !recovery::validStateMachineSettings(decoded)) return fail(ConfigError::Corrupt);
  output = decoded; error_ = ConfigError::None; return true;
}
bool ConfigurationStore::saveStateMachine(const recovery::StateMachineSettings &machine, bool disarmed) {
  if (!disarmed) return fail(ConfigError::Armed);
  if (!recovery::validStateMachineSettings(machine)) return fail(ConfigError::InvalidConfiguration);
  uint8_t payload[5]; uint32_t bits; memcpy(&bits, &machine.minimumCoastVelocity, 4);
  payload[0] = machine.bistage; binary::put(payload + 1, bits, 4);
  return writeSetting(DeviceJournal::StateMachine, payload, sizeof(payload));
}
bool ConfigurationStore::loadOrientation(uint8_t &axis) {
  bool present = false;
  if (!readSetting(DeviceJournal::Orientation, 1, present)) return false;
  if (!present) return fail(ConfigError::Missing);
  if (scratch_[0] >= 6) return fail(ConfigError::Corrupt);
  axis = scratch_[0]; error_ = ConfigError::None; return true;
}
bool ConfigurationStore::saveOrientation(uint8_t axis, bool disarmed) {
  if (!disarmed) return fail(ConfigError::Armed);
  if (axis >= 6) return fail(ConfigError::InvalidConfiguration);
  return writeSetting(DeviceJournal::Orientation, &axis, 1);
}

bool ConfigurationStore::clear(bool disarmed) {
  if (!disarmed) return fail(ConfigError::Armed);
  // Factory reset: rules and settings go; IMU calibration and the identity revocation stay.
  if (!journal_.reset(uint8_t(1u << DeviceJournal::ImuCalibration | 1u << DeviceJournal::IdentityRevoked))) return fail(ConfigError::Io);
  error_=ConfigError::None; return true;
}

namespace {
using binary::put;
using binary::get32;
using binary::crc;
constexpr uint16_t Version = ConfigurationStore::FormatVersion;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "Requires IEEE-754 binary32");
static_assert(39 + recovery::MaxConditions * 19 + recovery::MaxRules * (58 + recovery::MaxConditions * 19 + recovery::MaxAssignments * 6)
              <= ConfigurationStore::MaxPayload, "Supported configuration must fit one journal record");
struct Writer {
  uint8_t *data; std::size_t capacity, pos = 0; bool ok = true;
  Writer(uint8_t *p, std::size_t n) : data(p), capacity(n) {}
  void integer(uint32_t value, unsigned n) {
    if (!ok || n > capacity - pos) { ok = false; return; }
    put(data + pos, value, n); pos += n;
  }
  void floating(float value) { uint32_t bits; memcpy(&bits, &value, 4); integer(bits, 4); }
};
struct Reader {
  const uint8_t *data; std::size_t length, pos = 0; bool ok = true;
  Reader(const uint8_t *p, std::size_t n) : data(p), length(n) {}
  uint32_t integer(unsigned n) {
    if (!ok || n > length - pos) { ok = false; return 0; }
    const auto value = get32(data + pos, n); pos += n; return value;
  }
  float floating() { const uint32_t bits = integer(4); float value; memcpy(&value, &bits, 4); return value; }
};
}
const char *ConfigurationStore::errorName(ConfigError e) {
  switch (e) {
#define CFG_ERROR(x) case ConfigError::x: return #x
    CFG_ERROR(None); CFG_ERROR(Armed); CFG_ERROR(InvalidConfiguration); CFG_ERROR(Io);
    CFG_ERROR(Missing); CFG_ERROR(UnsupportedVersion); CFG_ERROR(Corrupt);
#undef CFG_ERROR
  }
  return "Unknown";
}
bool ConfigurationStore::encode(const recovery::Configuration &c, uint8_t *out, std::size_t capacity, std::size_t &length) {
  length = 0;
  if (!out || c.ruleCount > recovery::MaxRules || c.airbrakes.conditionCount > recovery::MaxConditions) return false;
  Writer w(out, capacity);
  w.integer(uint32_t(c.ruleCount), 1);
  for (const auto &p : c.pyros) w.integer(p.pulseMs, 4);
  for (const auto &s : c.servos) {
    w.integer(s.calibrated ? 1 : 0, 1); w.integer(s.initialUs, 2); w.integer(s.finalUs, 2);
  }
  for (std::size_t i = 0; i < c.ruleCount; ++i) {
    const auto &r = c.rules[i];
    if (r.conditionCount > recovery::MaxConditions || r.assignmentCount > recovery::MaxAssignments) return false;
    w.integer(r.id, 4);
    for (char ch : r.name) w.integer(static_cast<uint8_t>(ch), 1);
    w.integer(r.delayMs, 4); w.integer(uint32_t(r.conditionCount), 1); w.integer(uint32_t(r.assignmentCount), 1);
    for (std::size_t j = 0; j < r.conditionCount; ++j) {
      const auto &condition = r.conditions[j];
      w.integer(static_cast<uint8_t>(condition.kind), 1); w.integer(static_cast<uint8_t>(condition.comparison), 1);
      w.integer(condition.phases, 1); w.floating(condition.value); w.floating(condition.upper);
      w.integer(condition.elapsedMs, 4); w.integer(condition.reference, 4);
    }
    for (std::size_t j = 0; j < r.assignmentCount; ++j) {
      const auto &a = r.assignments[j]; w.integer(static_cast<uint8_t>(a.kind), 1);
      w.integer(a.channel, 1); w.integer(a.delayMs, 4);
    }
  }
  w.integer(c.airbrakes.channelMask,1); w.integer(uint32_t(c.airbrakes.conditionCount),1);
  for (std::size_t j=0;j<c.airbrakes.conditionCount;++j) {
    const auto &v=c.airbrakes.conditions[j];
    w.integer(uint8_t(v.kind),1); w.integer(uint8_t(v.comparison),1); w.integer(v.phases,1);
    w.floating(v.value); w.floating(v.upper); w.integer(v.elapsedMs,4); w.integer(v.reference,4);
  }
  if (!w.ok) return false;
  length = w.pos; return true;
}
bool ConfigurationStore::decode(const uint8_t *bytes, std::size_t length, recovery::Configuration &out) {
  if (!bytes || length > MaxPayload) return false;
  Reader r(bytes, length); recovery::Configuration c;
  c.ruleCount = r.integer(1);
  if (c.ruleCount > recovery::MaxRules) return false;
  for (auto &p : c.pyros) p.pulseMs = r.integer(4);
  for (auto &s : c.servos) {
    const auto calibrated = r.integer(1); if (calibrated > 1) return false;
    s.calibrated = calibrated != 0; s.initialUs = uint16_t(r.integer(2)); s.finalUs = uint16_t(r.integer(2));
  }
  for (std::size_t i = 0; i < c.ruleCount; ++i) {
    auto &rule = c.rules[i]; rule.id = r.integer(4);
    for (auto &ch : rule.name) ch = static_cast<char>(r.integer(1));
    rule.delayMs = r.integer(4); rule.conditionCount = r.integer(1); rule.assignmentCount = r.integer(1);
    if (rule.conditionCount > recovery::MaxConditions || rule.assignmentCount > recovery::MaxAssignments) return false;
    for (std::size_t j = 0; j < rule.conditionCount; ++j) {
      auto &cond = rule.conditions[j]; cond.kind = static_cast<recovery::ConditionKind>(r.integer(1));
      cond.comparison = static_cast<recovery::Comparison>(r.integer(1)); cond.phases = uint8_t(r.integer(1));
      cond.value = r.floating(); cond.upper = r.floating(); cond.elapsedMs = r.integer(4); cond.reference = r.integer(4);
    }
    for (std::size_t j = 0; j < rule.assignmentCount; ++j) {
      auto &a = rule.assignments[j]; a.kind = static_cast<recovery::OutputKind>(r.integer(1));
      a.channel = uint8_t(r.integer(1)); a.delayMs = r.integer(4);
    }
  }
  c.airbrakes.channelMask=uint8_t(r.integer(1)); c.airbrakes.conditionCount=r.integer(1);
  if (c.airbrakes.channelMask>15 || c.airbrakes.conditionCount>recovery::MaxConditions) return false;
  for (std::size_t j=0;j<c.airbrakes.conditionCount;++j) {
    auto &v=c.airbrakes.conditions[j]; v.kind=static_cast<recovery::ConditionKind>(r.integer(1));
    v.comparison=static_cast<recovery::Comparison>(r.integer(1)); v.phases=uint8_t(r.integer(1));
    v.value=r.floating(); v.upper=r.floating(); v.elapsedMs=r.integer(4); v.reference=r.integer(4);
  }
  if (!r.ok || r.pos != length) return false;
  out = c; return true;
}
bool ConfigurationStore::load(recovery::Configuration &output, const recovery::ValidationLimits &limits) {
  validation_ = {};
  DeviceJournal::Found found;
  if (!journal_.read(DeviceJournal::Rules, scratch_, sizeof(scratch_), found)) return fail(ConfigError::Io);
  if (found.corruptAfter) return fail(ConfigError::Corrupt);
  if (!found.present) return fail(ConfigError::Missing);
  if (!found.length || found.length < 2) return fail(ConfigError::Corrupt);
  // The package starts with its codec version (u16), followed by the encoded plan.
  if (get32(scratch_, 2) != Version) return fail(ConfigError::UnsupportedVersion);
  if (!decode(scratch_ + 2, found.length - 2, decoded_)) return fail(ConfigError::Corrupt);
  validation_ = recovery::validate(decoded_, limits);
  if (!validation_.ok()) return fail(ConfigError::InvalidConfiguration);
  output = decoded_; error_ = ConfigError::None; return true;
}
bool ConfigurationStore::save(const recovery::Configuration &config, const recovery::ValidationLimits &limits, bool disarmed) {
  validation_ = {};
  if (!disarmed) return fail(ConfigError::Armed);
  validation_ = recovery::validate(config, limits);
  if (!validation_.ok()) return fail(ConfigError::InvalidConfiguration);
  std::size_t length;
  if (!encode(config, scratch_ + 2, sizeof(scratch_) - 2, length)) return fail(ConfigError::InvalidConfiguration);
  put(scratch_, Version, 2);
  // Written beside the previous rules; a power cut keeps the old ones.
  if (!journal_.write(DeviceJournal::Rules, scratch_, length + 2)) return fail(ConfigError::Io);
  // Verify via the same boot load path, including semantic validation.
  return load(decoded_, limits);
}
}
