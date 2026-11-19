#pragma once
#include "storage/RecordingSink.h"
#include "storage/FlightDirectory.h"
#include "storage/FlightHeaderStore.h"
#include "storage/FlightSampleStore.h"
#include "storage/FlightSummaryStore.h"
#include "storage/PadHistory.h"

namespace storage {
struct RecordingInfo {
  FlightEntry entry;
  uint32_t headerBytes = 0, samples = 0;
  SummaryState summaryState = SummaryState::Missing;
  RecordingSummary summary;
};
// Synchronous storage-owner API. No USB hooks, automatic format or reclamation.
class FlightRecorder : public RecordingSink {
public:
  explicit FlightRecorder(FlashDevice &flash)
      : flash_(flash), directory_(flash), header_(flash), samples_(flash), summary_(flash) {}
  static const char *errorName(RecorderError error);
  // Sample limit meaning "the whole ring": the flight grows while recording.
  static constexpr uint32_t Unlimited = 0xFFFFF000;
  static constexpr uint32_t BaseBlocks = 8, GrowBlocks = 8; // 512 KiB at arming, then 512 KiB steps.
  static constexpr uint32_t GrowAheadBytes = 2 * FlashLayout::BlockBytes; // About 8 s at 476 Hz.
  static constexpr uint32_t FlightUpdates = 65; // Worst-case directory updates of one flight.
  static constexpr uint64_t LandedSpacingUs = 100000; // LANDED is logged at 10 Hz.
  bool format(); // Explicit logical format; preserves settings and calibration.
  bool initializeBlank(); // Explicit request; refuses any non-erased/unknown directory.
  // Boot: mount existing storage; prepare only a never-used (missing and fully erased) directory.
  // Damaged or unknown data is never formatted: the failure stays visible.
  bool prepareAtBoot();
  void abort(); // Drop RAM, preserve Recording metadata for recovery; no flash writes.
  bool initialize(); // Mount; reconcile Recording into Interrupted/Corrupt, never resume.
  bool begin(const recovery::Configuration &config, const recovery::ValidationLimits &limits, uint32_t sampleBytes,
             SensorMode sensorMode, OutputMode outputMode, sensors::FrameConvention frame, const FlightCalibration &calibration = FlightCalibration{});
  // PAD samples stay in the RAM pre-launch history; the first non-PAD sample writes
  // that history, then samples are encoded and full blocks written (no directory writes).
  bool append(const FlightSample &sample);
  bool recordStart(recovery::RuleId rule, recovery::OutputKind kind, uint32_t channel, uint32_t timeMs); // RAM only.
  bool service(); // Worker: persist pending output starts.
  bool finish(bool producerClosed, const RecordingSummary &summary = RecordingSummary{},
              RecordingRetention retention = RecordingRetention::Keep) override;
  bool remove(uint64_t sequence); // Durable logical deletion; erase only on later reuse.
  bool inspect(uint64_t sequence, RecordingInfo &output); // Detected corruption persists a directory flag.
  bool readBlock(uint64_t sequence, uint32_t index, RawBlock &output);
  const FlightHeader *metadata() const { return metadataValid_ ? header_.view() : nullptr; }
  const FlightDirectory &directory() const { return directory_; }
  const FlightEntry *current() const { return current_.sequence ? &current_ : nullptr; }
  bool active() const { return active_; }
  bool initialized() const { return initialized_; }
  bool samplesFull() const { return samplesFull_; }
  uint32_t committedSamples() const { return samples_.committedSamples(); }
  RecorderError error() const { return error_; }
  RecordingFailure failure() const override { return failure_; }
private:
  bool fail(RecorderError error, const char *component = nullptr, const char *cause = nullptr);
  bool fault(RecorderError error);
  uint32_t nextStartBlock() const;
  bool makeRoom(uint32_t first, uint32_t blocks, bool newFlight);
  bool grow();
  const FlightEntry *resolve(uint64_t sequence) const;
  bool openMetadata(const FlightEntry &entry);
  bool scan(const FlightEntry &entry, bool recover, RecordingInfo &info);
  bool corruptionDetected() const;
  bool markCorrupt(const FlightEntry &entry);
  bool rejectRead(const FlightEntry &entry); // Retain the decoding error while persisting quarantine.
  bool persistProgress(FlightStatus status);
  bool store(const FlightSample &sample, bool history, bool deliberate);
  bool writeHistory();
  FlashDevice &flash_;
  FlightDirectory directory_;
  FlightHeaderStore header_;
  FlightSampleStore samples_;
  FlightSummaryStore summary_;
  RawBlock block_;
  PadHistory pad_;
  bool launched_ = false; // A non-PAD sample was seen; history written.
  bool ringFull_ = false, haveLanded_ = false, landedBaroPending_ = false;
  uint64_t lastLandedUs_ = 0, landedBaroUs_ = 0;
  float landedPressure_ = 0, landedTemperature_ = 0;
  FlightEntry current_;
  uint32_t headerBytes_ = 0, sampleBytes_ = 0;
  bool initialized_ = false, active_ = false, samplesFull_ = false, metadataValid_ = false;
  RecorderError error_ = RecorderError::None;
  RecordingFailure failure_;
  bool faultLatched_ = false;
};
}
