#pragma once
#include "communication/CommandProtocol.h"
#include "storage/RecordingFailure.h"
namespace protocol {
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
