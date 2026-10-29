#pragma once
#include "communication/CommandProtocol.h"
#include "storage/RecordingFailure.h"
#include "flight/FlightRuntime.h"
namespace protocol {
// Small fragments preserve each command's existing fields and field order.
inline void appendRecordingBusy(Json &data, const flight::RuntimeStatus &s) {
  data.add(R"(,"recording_busy":%s)",s.recordingBusy ? "true" : "false");
}
inline void appendRecordingCounts(Json &data, const flight::RuntimeStatus &s) {
  data.add(R"(,"recording_sequence":"%llu","saved_samples":%lu,"dropped_samples":%lu)",
      static_cast<unsigned long long>(s.recordingSequence),static_cast<unsigned long>(s.savedSamples),static_cast<unsigned long>(s.droppedSamples));
}
inline void appendRecordingError(Json &data, const flight::RuntimeStatus &s) {
  data.add(R"(,"recording_error":"%s")",storage::recorderErrorName(s.recordingError));
}
inline void appendRecordingPending(Json &data, const flight::RuntimeStatus &s) {
  data.add(R"(,"pending_samples":%lu)",static_cast<unsigned long>(s.pendingSamples));
}
inline void appendRecordingFailure(Json &data, const storage::RecordingFailure &f) {
  data.add(",\"failure\":{\"error\":\"%s\",\"component\":\"%s\",\"cause\":\"%s\",\"flash\":\"%s\",\"operation\":%u,\"address\":%lu,\"bytes\":%llu}",
           storage::recorderErrorName(f.error),f.component,f.cause,storage::flashErrorName(f.flash.cause),unsigned(f.flash.operation),
           static_cast<unsigned long>(f.flash.address),static_cast<unsigned long long>(f.flash.bytes));
}
inline void appendRecordingSummary(Json &data, storage::SummaryState state, const storage::RecordingSummary &s) {
  data.add(",\"summary\":{\"state\":\"%s\"",storage::summaryStateName(state));
  if (state==storage::SummaryState::Valid)
    data.add(",\"version\":1,\"saved_samples\":%lu,\"dropped_samples\":%lu,\"queue_peak\":%lu,\"queue_capacity\":%lu,\"reason\":\"%s\"",
             static_cast<unsigned long>(s.savedSamples),static_cast<unsigned long>(s.droppedSamples),static_cast<unsigned long>(s.queuePeak),
             static_cast<unsigned long>(s.queueCapacity),storage::recordingEndName(s.reason));
  data.add("}");
}
}
