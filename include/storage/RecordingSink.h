#pragma once
#include "storage/RecordingTypes.h"
#include "storage/RecordingFailure.h"

namespace storage {
// Producer contract shared by the synchronous writer and queued storage worker.
// Successful append means accepted in RAM; committedSamples reports durable work.
class RecordingSink {
public:
  virtual ~RecordingSink() = default;
  virtual bool begin(const recovery::Configuration &, const recovery::ValidationLimits &, uint32_t,
                     SensorMode, OutputMode, sensors::FrameConvention, const FlightCalibration & = FlightCalibration{}) = 0;
  virtual bool append(const FlightSample &) = 0;
  virtual bool recordStart(recovery::RuleId, recovery::OutputKind, uint32_t, uint32_t) = 0;
  virtual bool service() = 0;
  // Requests close. For a queued sink, success means accepted, not yet durable.
  // busy() stays true until the worker acknowledges close/abort or a fault.
  // Discard durably invalidates this recording, without flushing its remaining RAM.
  virtual bool finish(bool producerClosed, const RecordingSummary &summary = RecordingSummary{},
                      RecordingRetention retention = RecordingRetention::Keep) = 0;
  virtual void abort() = 0;
  virtual bool busy() const { return active(); }
  virtual bool initialized() const = 0;
  virtual bool active() const = 0;
  virtual bool samplesFull() const = 0;
  virtual uint32_t committedSamples() const = 0;
  virtual const FlightEntry *current() const = 0;
  virtual RecorderError error() const = 0;
  virtual RecordingFailure failure() const = 0; // Detailed snapshot after worker acknowledgment.
  virtual bool queued() const { return false; }
  virtual uint32_t pendingSamples() const { return 0; }
  virtual uint32_t sampleQueueCapacity() const { return 0; }
  virtual uint32_t backgroundDroppedSamples() const { return 0; }
};
}
