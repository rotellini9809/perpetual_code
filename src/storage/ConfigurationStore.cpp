#include "configuration/StateMachineValidation.h"
#include "storage/ConfigurationStore.h"
#include "storage/BinaryEncoding.h"
#include "storage/VerifiedPage.h"
#include <cstring>
#include <limits>
#include <initializer_list>
#include <cmath>

namespace storage {
namespace {
constexpr uint32_t OrientationAddress=ConfigurationStore::Begin+FlashLayout::SectorBytes;
constexpr uint32_t AirbrakeAddress=ConfigurationStore::Begin+3*FlashLayout::SectorBytes;
constexpr uint32_t MachineAddress=ConfigurationStore::Begin+2*FlashLayout::SectorBytes;
}
bool ConfigurationStore::loadAirbrakes(bool &enabled) {
  uint8_t page[FlashLayout::PageBytes];
  if (!flash_.read(AirbrakeAddress,page,sizeof(page))) return fail(ConfigError::Io);
  bool blank=true; for (auto byte:page) blank &= byte==255;
  if (blank) { enabled=false; error_=ConfigError::None; return true; }
  if (binary::get32(page,4)!=0x52424150 || page[4]!=1 || page[5]>1 || binary::get32(page+252,4)!=binary::crc(page,252)) return fail(ConfigError::Corrupt);
  enabled=page[5]; error_=ConfigError::None; return true;
}
bool ConfigurationStore::saveAirbrakes(bool enabled,bool disarmed) {
  if (!disarmed) return fail(ConfigError::Armed);
  uint8_t page[FlashLayout::PageBytes],check[FlashLayout::PageBytes]; memset(page,255,sizeof(page));
  binary::put(page,0x52424150,4); page[4]=1; page[5]=enabled; binary::put(page+252,binary::crc(page,252),4);
  if (!flash_.eraseSector(AirbrakeAddress) || writeVerifiedPage(flash_,AirbrakeAddress,page,check)!=PageWriteError::None) return fail(ConfigError::Io);
  error_=ConfigError::None; return true;
}
bool ConfigurationStore::loadStateMachine(recovery::StateMachineSettings &settings) {
  uint8_t page[FlashLayout::PageBytes];
  if (!flash_.read(MachineAddress,page,sizeof(page))) return fail(ConfigError::Io);
  bool blank=true; for (auto byte:page) blank &= byte==255;
  if (blank) { settings={}; error_=ConfigError::None; return true; }
  if (binary::get32(page,4)!=0x4d534650 || page[4]!=1 || page[5]>1 || binary::get32(page+252,4)!=binary::crc(page,252)) return fail(ConfigError::Corrupt);
  recovery::StateMachineSettings decoded; decoded.bistage=page[5];
  uint32_t bits=binary::get32(page+8,4); memcpy(&decoded.minimumCoastVelocity,&bits,4);
  if (!recovery::validStateMachineSettings(decoded)) return fail(ConfigError::Corrupt);
  settings=decoded; error_=ConfigError::None; return true;
}
bool ConfigurationStore::saveStateMachine(const recovery::StateMachineSettings &settings,bool disarmed) {
  if (!disarmed) return fail(ConfigError::Armed);
  if (!recovery::validStateMachineSettings(settings)) return fail(ConfigError::InvalidConfiguration);
  uint8_t page[FlashLayout::PageBytes],check[FlashLayout::PageBytes]; memset(page,255,sizeof(page));
  binary::put(page,0x4d534650,4); page[4]=1; page[5]=settings.bistage;
  uint32_t bits; memcpy(&bits,&settings.minimumCoastVelocity,4); binary::put(page+8,bits,4);
  binary::put(page+252,binary::crc(page,252),4);
  if (!flash_.eraseSector(MachineAddress) || writeVerifiedPage(flash_,MachineAddress,page,check)!=PageWriteError::None) return fail(ConfigError::Io);
  error_=ConfigError::None; return true;
}
bool ConfigurationStore::loadOrientation(uint8_t &axis) {
  uint8_t page[FlashLayout::PageBytes];
  if (!flash_.read(OrientationAddress,page,sizeof(page))) return fail(ConfigError::Io);
  bool blank=true; for (auto byte:page) blank &= byte==255;
  if (blank) return fail(ConfigError::Missing);
  if (binary::get32(page,4)!=0x544e524f || page[4]!=1 || page[5]>=6 ||
      binary::get32(page+252,4)!=binary::crc(page,252)) return fail(ConfigError::Corrupt);
  axis=page[5]; error_=ConfigError::None; return true;
}
bool ConfigurationStore::saveOrientation(uint8_t axis, bool disarmed) {
  if (!disarmed) return fail(ConfigError::Armed);
  if (axis>=6) return fail(ConfigError::InvalidConfiguration);
  uint8_t page[FlashLayout::PageBytes], check[FlashLayout::PageBytes];
  memset(page,255,sizeof(page)); binary::put(page,0x544e524f,4); page[4]=1; page[5]=axis;
  binary::put(page+252,binary::crc(page,252),4);
  if (!flash_.eraseSector(OrientationAddress) || writeVerifiedPage(flash_,OrientationAddress,page,check)!=PageWriteError::None)
    return fail(ConfigError::Io);
  error_=ConfigError::None; return true;
}

bool ConfigurationStore::clear(bool disarmed) {
  if (!disarmed) return fail(ConfigError::Armed);
  uint8_t page[FlashLayout::PageBytes];
  for (const auto address:{Begin,OrientationAddress,MachineAddress,AirbrakeAddress}) {
    if (!flash_.eraseSector(address)) return fail(ConfigError::Io);
    for (uint32_t offset=0;offset<FlashLayout::SectorBytes;offset+=sizeof(page)) {
      if (!flash_.read(address+offset,page,sizeof(page))) return fail(ConfigError::Io);
      for (auto byte:page) if (byte!=255) return fail(ConfigError::Io);
    }
  }
  error_=ConfigError::None; return true;
}

namespace {
using binary::put;
using binary::get32;
using binary::crc;
constexpr uint32_t Magic = 0x47464350, Commit = 0x54494D43, Page = FlashLayout::PageBytes, PayloadOffset = 512;
constexpr uint16_t Version = ConfigurationStore::FormatVersion;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "Requires IEEE-754 binary32");
static_assert(39 + recovery::MaxConditions * 19 + recovery::MaxRules * (58 + recovery::MaxConditions * 19 + recovery::MaxAssignments * 6)
              <= FlashLayout::SectorBytes - PayloadOffset, "Supported configuration must fit one sector");
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
  uint8_t header[Page], marker[Page];
  if (!flash_.read(Begin, header, Page) || !flash_.read(Begin + Page, marker, Page)) return fail(ConfigError::Io);
  bool erased = true; for (auto byte : header) erased &= byte == 255;
  if (erased) return fail(ConfigError::Missing);
  if (get32(header, 4) != Magic || get32(header + 252, 4) != crc(header, 252)) return fail(ConfigError::Corrupt);
  const auto version = get32(header + 4, 2);
  if (version != Version) return fail(ConfigError::UnsupportedVersion);
  const uint32_t length = get32(header + 8, 4), checksum = get32(header + 12, 4);
  if (length == 0 || length > MaxPayload || length > Bytes - PayloadOffset ||
      get32(marker, 4) != Commit || get32(marker + 4, 4) != get32(header + 252, 4) ||
      get32(marker + 8, 4) != checksum || get32(marker + 12, 4) != length) return fail(ConfigError::Corrupt);
  if (!flash_.read(Begin + PayloadOffset, scratch_, length)) return fail(ConfigError::Io);
  if (crc(scratch_, length) != checksum) return fail(ConfigError::Corrupt);
  if (!decode(scratch_, length, decoded_)) return fail(ConfigError::Corrupt);
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
  if (!encode(config, scratch_, sizeof(scratch_), length)) return fail(ConfigError::InvalidConfiguration);
  if (length > FlashLayout::SectorBytes - PayloadOffset) return fail(ConfigError::InvalidConfiguration);
  const uint32_t checksum = crc(scratch_, length);
  uint8_t header[Page], marker[Page], check[Page];
  memset(header, 255, Page); memset(marker, 255, Page);
  put(header, Magic, 4); put(header + 4, Version, 2); put(header + 8, uint32_t(length), 4); put(header + 12, checksum, 4);
  const uint32_t headerCrc = crc(header, 252); put(header + 252, headerCrc, 4);
  put(marker, Commit, 4); put(marker + 4, headerCrc, 4); put(marker + 8, checksum, 4); put(marker + 12, uint32_t(length), 4);
  // All supported settings fit the first sector. The remaining reservation stays untouched.
  if (!flash_.eraseSector(Begin)) return fail(ConfigError::Io);
  if (writeVerifiedPage(flash_, Begin, header, check) != PageWriteError::None) return fail(ConfigError::Io);
  for (std::size_t offset = 0; offset < length; offset += Page) {
    uint8_t page[Page]; memset(page, 255, Page);
    const auto count = length - offset < Page ? length - offset : Page;
    memcpy(page, scratch_ + offset, count);
    const auto address = Begin + PayloadOffset + uint32_t(offset);
    if (writeVerifiedPage(flash_, address, page, check) != PageWriteError::None) return fail(ConfigError::Io);
  }
  if (writeVerifiedPage(flash_, Begin + Page, marker, check) != PageWriteError::None) return fail(ConfigError::Io);
  // Verify via the same boot load path, including semantic validation.
  return load(decoded_, limits);
}
}
