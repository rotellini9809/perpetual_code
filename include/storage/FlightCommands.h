#pragma once
#include "storage/FlightRecorder.h"
#include "storage/ProfiledFlashDevice.h"
#include "storage/QueuedRecorder.h"
#include "configuration/ConfigurationService.h"
#include "flight/RunController.h"
#include "communication/CommandService.h"

class FlightCommands : public CommandExtension {
public:
  FlightCommands(storage::FlightRecorder &recorder, ConfigurationService &config, flight::RunController &run,
                 storage::ProfiledFlashDevice *profile = nullptr, storage::QueuedRecorder *worker = nullptr, timing::Clock clock = nullptr)
      : recorder_(recorder), config_(config), run_(run), profile_(profile), worker_(worker), clock_(clock) {}
  bool dispatch(const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity) override;
  bool storageBusy() const { return worker_ && worker_->busy(); }
  bool transferActive() const { return transferring_ && (!clock_ || uint32_t(clock_()-lastTransferUs_)<5000000); }
private:
  bool select(uint64_t sequence);
  storage::FlightRecorder &recorder_;
  ConfigurationService &config_;
  flight::RunController &run_;
  storage::ProfiledFlashDevice *profile_;
  storage::QueuedRecorder *worker_;
  storage::RecordingInfo selected_;
  storage::FlightHeaderLayout headerLayout_;
  storage::FlightCalibration calibration_;
  std::array<storage::OutputStartSlot, recovery::MaxAssignments> starts_{};
  storage::SensorMode sensorMode_ = storage::SensorMode::Simulation;
  storage::OutputMode outputMode_ = storage::OutputMode::Virtual;
  sensors::FrameConvention frame_ = sensors::FrameConvention::LoggedCalibratedBody;
  uint8_t configuration_[storage::ConfigurationStore::MaxPayload] = {};
  std::size_t configBytes_ = 0;
  storage::SampleBlock block_;
  bool selectedValid_ = false, blockValid_ = false;
  timing::Clock clock_;
  uint32_t lastTransferUs_=0;
  bool transferring_=false; // Expiring download lease; not a flash-driver timeout/recovery policy.
};
