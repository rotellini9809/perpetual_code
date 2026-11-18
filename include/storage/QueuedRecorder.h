#pragma once
#include "storage/FlightRecorder.h"
#include "storage/RecordingQueue.h"
#include "shared/Timing.h"

namespace storage {
// beginAsync hands preparation and exclusive flash ownership to core 1.
// Only poll() accesses the underlying writer until completion/fault acknowledgment.
class QueuedRecorder : public RecordingSink {
public:
  static constexpr uint32_t SampleCapacity = 128, StartCapacity = 8; // ~270 ms at 476 Hz.
  using Notify = void (*)();
  QueuedRecorder(FlightRecorder &writer, timing::Clock clock = nullptr, Notify notify = nullptr)
      : writer_(writer), clock_(clock), notify_(notify) {}
  void workerStarted() { online_.store(true, std::memory_order_release); }
  bool online() const { return online_.load(std::memory_order_acquire); }
  bool busy() const override;
  bool poll(); // Consumer/core 1 only; may block on flash. False means no work.
  bool begin(const recovery::Configuration &, const recovery::ValidationLimits &, uint32_t,
             SensorMode, OutputMode, sensors::FrameConvention, const FlightCalibration & = FlightCalibration{}) override;
  bool beginAsync(const recovery::Configuration &, const recovery::ValidationLimits &, uint32_t,
                  SensorMode, OutputMode, sensors::FrameConvention, const FlightCalibration &) override;
  bool preparing() const override { return state_.load(std::memory_order_acquire)==State::Preparing; }
  bool activatePrepared() override;
  bool append(const FlightSample &) override;
  bool recordStart(recovery::RuleId, recovery::OutputKind, uint32_t, uint32_t) override;
  bool service() override; // Producer/core 0: health check only; never touches flash.
  bool finish(bool producerClosed, const RecordingSummary &summary = RecordingSummary{},
              RecordingRetention retention = RecordingRetention::Keep) override;
  void abort() override; // Request abandonment; ownership ends only when busy() becomes false.
  bool initialized() const override;
  bool active() const override;
  bool samplesFull() const override { return full_.load(std::memory_order_relaxed); }
  uint32_t committedSamples() const override { return saved_.load(std::memory_order_relaxed); }
  const FlightEntry *current() const override;
  RecorderError error() const override;
  RecordingFailure failure() const override;
  bool queued() const override { return true; }
  uint32_t pendingSamples() const override { return busy() ? samples_.size() : 0; }
  uint32_t sampleQueueCapacity() const override { return SampleCapacity; }
  uint32_t backgroundDroppedSamples() const override { return dropped_.load(std::memory_order_relaxed); }
  uint32_t peakSamples() const { return peak_; } // Producer-owned.
  std::size_t directoryCount() const { return directoryCount_; } // Frozen while busy.
  const timing::Statistics &workerTiming() const { return workerTiming_; } // Idle/acknowledged only.
private:
  enum class State : uint8_t { Idle, Preparing, Prepared, Running, Closing, Aborting, Closed, Fault };
  recovery::Configuration preparationConfig_;
  recovery::ValidationLimits preparationLimits_;
  uint32_t preparationBytes_=0;
  SensorMode preparationSensor_=SensorMode::Hardware;
  OutputMode preparationOutput_=OutputMode::Physical;
  sensors::FrameConvention preparationFrame_=sensors::FrameConvention::SensorAxes;
  FlightCalibration preparationCalibration_;
  bool preparationOnly_=false; // Published with state_; true until producer activation.
  struct Start { recovery::RuleId rule = 0; recovery::OutputKind kind = recovery::OutputKind::Pyro; uint32_t channel = 0, timeMs = 0; };
  bool fail(RecorderError error) { frontError_ = error; return false; }
  void publish();
  void fault(RecorderError error, uint32_t start);
  void close(uint32_t start);
  uint32_t now() const { return clock_ ? clock_() : 0; }
  FlightRecorder &writer_;
  timing::Clock clock_;
  Notify notify_;
  RecordingQueue<FlightSample, SampleCapacity> samples_;
  RecordingQueue<Start, StartCapacity> starts_;
  std::atomic<State> state_{State::Idle};
  std::atomic<bool> online_{false}, ready_{false}, full_{false};
  std::atomic<uint32_t> saved_{0}, dropped_{0};
  std::atomic<RecorderError> workerError_{RecorderError::None};
  RecorderError frontError_ = RecorderError::None;
  FlightEntry current_;
  std::array<OutputStartSlot, recovery::MaxAssignments> assignments_{};
  uint8_t assignmentCount_ = 0;
  uint32_t peak_ = 0;
  std::size_t directoryCount_ = 0;
  timing::Statistics workerTiming_;
  RecordingSummary closingSummary_;
  RecordingRetention closingRetention_ = RecordingRetention::Keep;
};
}
