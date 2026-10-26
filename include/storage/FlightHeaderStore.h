#pragma once
#include "storage/RecordingTypes.h"
#include "storage/FlashLayout.h"
#include "storage/ConfigurationStore.h"
#include "hardware/SensorSource.h"

namespace storage {
struct FlightHeaderLayout {
  uint32_t headerBytes = 0, configBytes = 0, tableOffset = 0, summaryOffset = 0;
  uint8_t slotCount = 0;
};
struct FlightHeader {
  FlightHeaderLayout layout;
  recovery::Configuration configuration;
  FlightCalibration calibration;
  SensorMode sensorMode = SensorMode::Simulation;
  OutputMode outputMode = OutputMode::Virtual;
  sensors::FrameConvention frame = sensors::FrameConvention::LoggedCalibratedBody;
  std::array<OutputStartSlot, recovery::MaxAssignments> starts{};
};
enum class HeaderError : uint8_t {
  None, Bounds, Active, InvalidConfiguration, InvalidArgument, NotErased,
  Io, Empty, Incomplete, Corrupt, UnsupportedVersion, NotLoaded, UnknownOutput,
  Conflict, Closed, ProducerActive, Persistence, Verify
};
// One storage owner. No directory mutation, erases, actuator API, or auto-resume.
class FlightHeaderStore {
public:
  static constexpr uint32_t SlotBytes = 512;
  explicit FlightHeaderStore(FlashDevice &flash) : flash_(flash) {}
  bool plan(const recovery::Configuration &config, const recovery::ValidationLimits &limits, FlightHeaderLayout &layout);
  bool create(uint32_t address, const recovery::Configuration &config, const recovery::ValidationLimits &limits,
              SensorMode sensorMode, OutputMode outputMode, sensors::FrameConvention frame, const FlightCalibration &calibration = FlightCalibration{});
  bool load(uint32_t address); // Read-only; never enables start notifications after reboot.
  bool recordStart(recovery::RuleId rule, recovery::OutputKind kind, uint32_t channel, uint32_t timeMs); // RAM only.
  bool flushStarts(); // Storage worker only: each slot page is programmed once.
  // Caller must first establish that no future starts remain scheduled.
  bool finish(bool producerClosed);
  const FlightHeader *view() const { return loaded_ ? &header_ : nullptr; }
  HeaderError error() const { return error_; }
  uint32_t checksum() const { return headerCrc_; }
  static const char *errorName(HeaderError error);
private:
  friend class FlightRecorder;
  void abandon() { loaded_ = accepting_ = false; finished_ = true; faulted_ = false; }
  bool fail(HeaderError error) { error_ = error; return false; }
  bool readSlot(unsigned index);
  bool programPage(uint32_t address, const uint8_t *bytes);
  FlashDevice &flash_;
  uint32_t address_ = 0, headerCrc_ = 0;
  FlightHeader header_;
  uint8_t payload_[ConfigurationStore::MaxPayload] = {}, page_[256] = {}, check_[256] = {};
  bool loaded_ = false, accepting_ = false, finished_ = true, faulted_ = false;
  HeaderError error_ = HeaderError::None;
};
}
