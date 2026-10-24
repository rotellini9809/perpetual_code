#include "storage/FlightHeaderStore.h"
#include "storage/BinaryEncoding.h"
#include "storage/VerifiedPage.h"
#include <cstring>
#include <limits>

namespace storage {
namespace {
using binary::put;
using binary::get32;
using binary::crc;
using binary::extendCrc;
using binary::CrcInitial;
constexpr uint32_t Magic = 0x52444850, HeaderCommit = 0x43444850, StartMagic = 0x54534850, StartCommit = 0x43534850;
constexpr uint32_t ConfigOffset = 512, Page = FlashLayout::PageBytes;
constexpr uint16_t HeaderVersion = 2, StartVersion = 1;
static_assert(recovery::MaxAssignments >= 2 * recovery::ChannelCount, "Header must hold every physical output");
static_assert(64 + recovery::MaxAssignments * 8 <= 252, "Mapping table exceeds descriptor page");
uint32_t aligned(uint32_t n, uint32_t a) { return (n + a - 1) / a * a; }
bool erased(const uint8_t *p, std::size_t n) { for (std::size_t i = 0; i < n; ++i) if (p[i] != 255) return false; return true; }
void marker(uint8_t *p, uint32_t magic, uint32_t checksum) {
  memset(p, 255, Page); put(p, magic, 4); put(p + 4, checksum, 4); put(p + 8, ~checksum, 4);
}
bool committed(const uint8_t *p, uint32_t magic, uint32_t checksum) {
  return get32(p, 4) == magic && get32(p + 4, 4) == checksum && get32(p + 8, 4) == ~checksum && erased(p + 12, Page - 12);
}
bool modes(SensorMode sensor, OutputMode output, sensors::FrameConvention frame) {
  return (sensor == SensorMode::Hardware || sensor == SensorMode::Simulation) &&
      (output == OutputMode::Physical || output == OutputMode::Virtual) &&
      (frame == sensors::FrameConvention::SensorAxes || frame == sensors::FrameConvention::LoggedCalibratedBody);
}
uint8_t mappings(const recovery::Configuration &config, std::array<OutputStartSlot, recovery::MaxAssignments> &slots) {
  slots = {}; uint8_t count = 0;
  for (std::size_t i = 0; i < config.ruleCount; ++i)
    for (std::size_t j = 0; j < config.rules[i].assignmentCount; ++j) {
      const auto &a = config.rules[i].assignments[j];
      auto &s = slots[count++]; s.rule = config.rules[i].id; s.kind = a.kind; s.channel = a.channel;
    }
  return count;
}
}

const char *FlightHeaderStore::errorName(HeaderError e) {
  switch (e) {
#define HEADER_ERROR(x) case HeaderError::x: return #x
    HEADER_ERROR(None); HEADER_ERROR(Bounds); HEADER_ERROR(Active); HEADER_ERROR(InvalidConfiguration);
    HEADER_ERROR(InvalidArgument); HEADER_ERROR(NotErased); HEADER_ERROR(Io); HEADER_ERROR(Empty);
    HEADER_ERROR(Incomplete); HEADER_ERROR(Corrupt); HEADER_ERROR(UnsupportedVersion); HEADER_ERROR(NotLoaded);
    HEADER_ERROR(UnknownOutput); HEADER_ERROR(Conflict); HEADER_ERROR(Closed); HEADER_ERROR(ProducerActive); HEADER_ERROR(Persistence); HEADER_ERROR(Verify);
#undef HEADER_ERROR
  }
  return "Unknown";
}
bool FlightHeaderStore::plan(const recovery::Configuration &config, const recovery::ValidationLimits &limits, FlightHeaderLayout &layout) {
  if (!recovery::validate(config, limits).ok()) return fail(HeaderError::InvalidConfiguration);
  std::size_t length;
  if (!ConfigurationStore::encode(config, payload_, sizeof(payload_), length)) return fail(HeaderError::InvalidConfiguration);
  FlightHeaderLayout result; result.configBytes = uint32_t(length);
  for (std::size_t i = 0; i < config.ruleCount; ++i) result.slotCount += uint8_t(config.rules[i].assignmentCount);
  result.tableOffset = ConfigOffset + aligned(result.configBytes, Page);
  const auto end = result.tableOffset + result.slotCount * SlotBytes;
  result.summaryOffset = end;
  result.headerBytes = aligned(end + SlotBytes, FlashLayout::SectorBytes);
  layout = result; error_ = HeaderError::None; return true;
}
bool FlightHeaderStore::programPage(uint32_t address, const uint8_t *bytes) {
  const auto error = writeVerifiedPage(flash_, address, bytes, check_);
  if (error != PageWriteError::None) return fail(error == PageWriteError::Io ? HeaderError::Io : HeaderError::Verify);
  return true;
}
bool FlightHeaderStore::create(uint32_t address, const recovery::Configuration &config, const recovery::ValidationLimits &limits,
                               SensorMode sensor, OutputMode output, sensors::FrameConvention frame) {
  if (loaded_ && !finished_) return fail(HeaderError::Active);
  loaded_ = accepting_ = false;
  if (!modes(sensor, output, frame)) return fail(HeaderError::InvalidArgument);
  FlightHeaderLayout layout;
  if (!plan(config, limits, layout)) return false;
  if (address % FlashLayout::SectorBytes || !FlashLayout::dataRange(address, layout.headerBytes)) return fail(HeaderError::Bounds);
  for (uint32_t i = 0; i < layout.headerBytes; i += Page) {
    if (!flash_.read(address + i, check_, Page)) return fail(HeaderError::Io);
    if (!erased(check_, Page)) return fail(HeaderError::NotErased);
  }
  header_.layout = layout; header_.sensorMode = sensor; header_.outputMode = output; header_.frame = frame;
  mappings(config, header_.starts);
  memset(payload_ + layout.configBytes, 255, aligned(layout.configBytes, Page) - layout.configBytes);
  memset(page_, 255, Page); put(page_, Magic, 4); put(page_ + 4, HeaderVersion, 2); put(page_ + 6, Page, 2);
  put(page_ + 8, layout.headerBytes, 4); put(page_ + 12, ConfigOffset, 4); put(page_ + 16, layout.configBytes, 4);
  put(page_ + 20, ConfigurationStore::FormatVersion, 2); page_[22] = uint8_t(sensor); page_[23] = uint8_t(output); page_[24] = uint8_t(frame);
  page_[25] = 1; page_[26] = layout.slotCount;
  put(page_ + 28, layout.tableOffset, 4); put(page_ + 32, SlotBytes, 2); put(page_ + 36, address, 4); put(page_ + 40, layout.headerBytes, 4);
  put(page_ + 44, layout.summaryOffset, 4);
  for (unsigned i = 0; i < layout.slotCount; ++i) {
    auto *p = page_ + 64 + i*8; const auto &s = header_.starts[i];
    put(p, s.rule, 4); p[4] = uint8_t(s.kind); p[5] = s.channel;
  }
  const auto checksum = extendCrc(extendCrc(CrcInitial, page_, 252), payload_, aligned(layout.configBytes, Page)) ^ CrcInitial;
  put(page_ + 252, checksum, 4);
  // Each immutable payload page is programmed once, descriptor next, commit last.
  for (uint32_t i = 0; i < aligned(layout.configBytes, Page); i += Page)
    if (!programPage(address + ConfigOffset + i, payload_ + i)) return false;
  if (!programPage(address, page_)) return false;
  marker(page_, HeaderCommit, checksum);
  if (!programPage(address + Page, page_)) return false;
  if (!load(address)) return false;
  accepting_ = true; finished_ = false; faulted_ = false;
  error_ = HeaderError::None; return true;
}
bool FlightHeaderStore::load(uint32_t address) {
  if (loaded_ && !finished_) return fail(HeaderError::Active);
  loaded_ = accepting_ = false;
  if (address % FlashLayout::SectorBytes || !FlashLayout::dataRange(address, ConfigOffset)) return fail(HeaderError::Bounds);
  if (!flash_.read(address, page_, Page) || !flash_.read(address + Page, check_, Page)) return fail(HeaderError::Io);
  if (erased(page_, Page) && erased(check_, Page)) return fail(HeaderError::Empty);
  const auto checksum = get32(page_ + 252, 4);
  if (!committed(check_, HeaderCommit, checksum)) return fail(HeaderError::Incomplete);
  if (get32(page_, 4) != Magic || get32(page_ + 6, 2) != Page) return fail(HeaderError::Corrupt);
  if (get32(page_ + 4, 2) != HeaderVersion || get32(page_ + 20, 2) != ConfigurationStore::FormatVersion || page_[25] != 1) return fail(HeaderError::UnsupportedVersion);
  FlightHeaderLayout layout;
  layout.headerBytes = get32(page_ + 8, 4); layout.configBytes = get32(page_ + 16, 4);
  layout.tableOffset = get32(page_ + 28, 4); layout.slotCount = page_[26];
  layout.summaryOffset = get32(page_ + 44, 4);
  if (!layout.configBytes || layout.configBytes > sizeof(payload_) || layout.slotCount > recovery::MaxAssignments ||
      get32(page_ + 12, 4) != ConfigOffset || get32(page_ + 32, 2) != SlotBytes || get32(page_ + 36, 4) != address ||
      get32(page_ + 40, 4) != layout.headerBytes || layout.tableOffset != ConfigOffset + aligned(layout.configBytes, Page) ||
      layout.summaryOffset != layout.tableOffset + layout.slotCount*SlotBytes ||
      layout.headerBytes != aligned(layout.summaryOffset + SlotBytes, FlashLayout::SectorBytes) || !FlashLayout::dataRange(address, layout.headerBytes))
    return fail(HeaderError::Corrupt);
  if (!flash_.read(address + ConfigOffset, payload_, aligned(layout.configBytes, Page))) return fail(HeaderError::Io);
  if ((extendCrc(extendCrc(CrcInitial, page_, 252), payload_, aligned(layout.configBytes, Page)) ^ CrcInitial) != checksum ||
      !erased(payload_ + layout.configBytes, aligned(layout.configBytes, Page) - layout.configBytes) || page_[27] != 255 ||
      !erased(page_ + 34, 2) || !erased(page_ + 48, 16) || !erased(page_ + 64 + layout.slotCount*8, 252 - 64 - layout.slotCount*8))
    return fail(HeaderError::Corrupt);
  const recovery::ValidationLimits historical{1, 65535, UINT32_MAX, UINT32_MAX};
  if (!ConfigurationStore::decode(payload_, layout.configBytes, header_.configuration) || !recovery::validate(header_.configuration, historical).ok())
    return fail(HeaderError::Corrupt);
  if (mappings(header_.configuration, header_.starts) != layout.slotCount) return fail(HeaderError::Corrupt);
  for (unsigned i = 0; i < layout.slotCount; ++i) {
    const auto *p = page_ + 64 + i*8; const auto &s = header_.starts[i];
    if (get32(p, 4) != s.rule || p[4] != uint8_t(s.kind) || p[5] != s.channel || !erased(p + 6, 2)) return fail(HeaderError::Corrupt);
  }
  header_.sensorMode = static_cast<SensorMode>(page_[22]); header_.outputMode = static_cast<OutputMode>(page_[23]);
  header_.frame = static_cast<sensors::FrameConvention>(page_[24]);
  if (!modes(header_.sensorMode, header_.outputMode, header_.frame)) return fail(HeaderError::Corrupt);
  header_.layout = layout; address_ = address; headerCrc_ = checksum;
  for (unsigned i = 0; i < layout.slotCount; ++i) if (!readSlot(i)) return false;
  loaded_ = true; finished_ = true; faulted_ = false; error_ = HeaderError::None; return true;
}
bool FlightHeaderStore::readSlot(unsigned index) {
  const uint32_t address = address_ + header_.layout.tableOffset + index*SlotBytes;
  if (!flash_.read(address, page_, Page) || !flash_.read(address + Page, check_, Page)) return fail(HeaderError::Io);
  auto &s = header_.starts[index]; s.state = StartState::Invalid; s.timeMs = 0;
  if (erased(page_, Page) && erased(check_, Page)) { s.state = StartState::Empty; return true; }
  const auto checksum = get32(page_ + 252, 4);
  if (!committed(check_, StartCommit, checksum) || crc(page_, 252) != checksum || get32(page_, 4) != StartMagic ||
      get32(page_ + 4, 2) != StartVersion || get32(page_ + 6, 2) != index || get32(page_ + 8, 4) != address_ || get32(page_ + 12, 4) != headerCrc_ ||
      get32(page_ + 16, 4) != s.rule || page_[20] != uint8_t(s.kind) || page_[21] != s.channel ||
      !erased(page_ + 22, 2) || !erased(page_ + 28, 224)) return true;
  s.timeMs = get32(page_ + 24, 4); s.state = StartState::Recorded; return true;
}
bool FlightHeaderStore::recordStart(recovery::RuleId rule, recovery::OutputKind kind, uint32_t channel, uint32_t timeMs) {
  if (!loaded_) return fail(HeaderError::NotLoaded);
  if (!accepting_) return fail(HeaderError::Closed);
  if (faulted_) return fail(HeaderError::Persistence);
  for (unsigned i = 0; i < header_.layout.slotCount; ++i) {
    auto &s = header_.starts[i]; if (s.rule != rule || s.kind != kind || s.channel != channel) continue;
    if (s.state == StartState::Invalid) return fail(HeaderError::Persistence);
    if (s.state != StartState::Empty && s.timeMs != timeMs) return fail(HeaderError::Conflict);
    if (s.state == StartState::Empty) { s.timeMs = timeMs; s.state = StartState::Pending; }
    error_ = HeaderError::None; return true;
  }
  return fail(HeaderError::UnknownOutput);
}
bool FlightHeaderStore::flushStarts() {
  if (!loaded_) return fail(HeaderError::NotLoaded);
  if (faulted_) return fail(HeaderError::Persistence);
  for (unsigned i = 0; i < header_.layout.slotCount; ++i) {
    auto &s = header_.starts[i]; if (s.state != StartState::Pending) continue;
    const uint32_t expectedTime = s.timeMs;
    const uint32_t address = address_ + header_.layout.tableOffset + i*SlotBytes;
    memset(page_, 255, Page); put(page_, StartMagic, 4); put(page_ + 4, StartVersion, 2); put(page_ + 6, i, 2);
    put(page_ + 8, address_, 4); put(page_ + 12, headerCrc_, 4); put(page_ + 16, s.rule, 4);
    page_[20] = uint8_t(s.kind); page_[21] = s.channel; put(page_ + 24, s.timeMs, 4);
    const auto checksum = crc(page_, 252); put(page_ + 252, checksum, 4);
    // Any reported mutation failure permanently blocks retries of this header session.
    faulted_ = true;
    if (!programPage(address, page_)) { s.state = StartState::Invalid; return false; }
    marker(page_, StartCommit, checksum);
    if (!programPage(address + Page, page_)) { s.state = StartState::Invalid; return false; }
    if (!readSlot(i)) { s.state = StartState::Invalid; s.timeMs = expectedTime; return false; }
    if (s.state != StartState::Recorded || s.timeMs != expectedTime) {
      s.state = StartState::Invalid; s.timeMs = expectedTime; return fail(HeaderError::Corrupt);
    }
    faulted_ = false;
  }
  error_ = HeaderError::None; return true;
}
bool FlightHeaderStore::finish(bool producerClosed) {
  if (!loaded_) return fail(HeaderError::NotLoaded);
  if (!producerClosed) return fail(HeaderError::ProducerActive);
  accepting_ = false;
  if (!flushStarts()) return false;
  finished_ = true; return true;
}
}
