#include "storage/FlightCommands.h"
#include "communication/CommandProtocol.h"
#include "communication/TimingReply.h"
#include "communication/RecordingReply.h"

bool FlightCommands::select(uint64_t sequence) {
  if (selectedValid_ && selected_.entry.sequence == sequence) return true;
  selectedValid_ = blockValid_ = false;
  if (!recorder_.inspect(sequence, selected_)) return false;
  const auto &header = *recorder_.metadata();
  if (!storage::ConfigurationStore::encode(header.configuration, configuration_, sizeof(configuration_), configBytes_)) return false;
  headerLayout_ = header.layout; starts_ = header.starts;
  sensorMode_ = header.sensorMode; outputMode_ = header.outputMode; frame_ = header.frame;
  selectedValid_ = true; return true;
}
bool FlightCommands::dispatch(const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity) {
  if (strcmp(command, "flights") != 0) return false;
  using namespace storage;
  protocol::Tokens t(arguments); protocol::Json data; const char *error = nullptr;
  auto finish = [&]() {
    if (error && (!worker_ || !worker_->busy()) && strcmp(error,FlightRecorder::errorName(recorder_.error()))==0 && recorder_.error()!=RecorderError::None)
      protocol::appendRecordingFailure(data,recorder_.failure());
    return protocol::finish(command, data, error, hasId, id, output, capacity, true);
  };
  if (!t.valid) { error = "INVALID_ARGUMENT"; return finish(); }
  // Detailed native state/profile buffers are owned by core 1 until its release acknowledgment.
  if (worker_ && worker_->busy()) {
    if (t.count == 1 && t.is(0, "status")) {
      data.add(",\"ready\":%s,\"active\":true,\"capacity_bytes\":%lu,\"identities\":%u,\"error\":\"%s\",\"directory_error\":\"None\",\"pending_samples\":%lu,\"queue_peak\":%lu",
               worker_->initialized() ? "true" : "false", static_cast<unsigned long>(FlashLayout::Capacity), unsigned(worker_->directoryCount()),
               FlightRecorder::errorName(worker_->error()), static_cast<unsigned long>(worker_->pendingSamples()), static_cast<unsigned long>(worker_->peakSamples()));
    } else error = "BUSY";
    return finish();
  }
  if (t.count == 1 && t.is(0, "status")) {
    data.add(",\"ready\":%s,\"active\":%s,\"capacity_bytes\":%lu,\"identities\":%u,\"error\":\"%s\",\"directory_error\":\"%s\",\"corrupt_flights\":%u",
             recorder_.initialized() ? "true" : "false", recorder_.active() ? "true" : "false",
             static_cast<unsigned long>(FlashLayout::Capacity), unsigned(recorder_.directory().count()),
             FlightRecorder::errorName(recorder_.error()), FlightDirectory::errorName(recorder_.directory().error()), unsigned(recorder_.directory().corruptCount()));
    return finish();
  }
  if (t.count==1 && t.is(0,"failure")) { protocol::appendRecordingFailure(data,recorder_.failure()); return finish(); }
  if (t.count == 1 && t.is(0, "timing")) {
    if (!profile_) { error = "TIMING_UNAVAILABLE"; return finish(); }
    const auto &s = profile_->timing();
    data.add(",\"timing_valid\":%s", profile_->timingValid() ? "true" : "false");
    protocol::appendTiming(data, "read", s.read.time); protocol::appendTiming(data, "program", s.program.time);
    protocol::appendTiming(data, "erase", s.erase.time);
    data.add(",\"failures\":[%lu,%lu,%lu],\"bytes\":[%llu,%llu,%llu]",
             static_cast<unsigned long>(s.read.failures), static_cast<unsigned long>(s.program.failures), static_cast<unsigned long>(s.erase.failures),
             static_cast<unsigned long long>(s.read.bytes), static_cast<unsigned long long>(s.program.bytes), static_cast<unsigned long long>(s.erase.bytes));
    if (worker_) {
      protocol::appendTiming(data, "worker", worker_->workerTiming());
      data.add(",\"queue_peak\":%lu,\"queue_capacity\":%lu", static_cast<unsigned long>(worker_->peakSamples()), static_cast<unsigned long>(worker_->sampleQueueCapacity()));
    }
    return finish();
  }
  if (run_.active() || config_.runLocked() || recorder_.active()) { error = "BUSY"; return finish(); }
  if (!config_.disarmed()) { error = "ARMED"; return finish(); }
  if (t.count == 2 && t.is(0, "timing") && t.is(1, "reset")) {
    if (!profile_) { error = "TIMING_UNAVAILABLE"; return finish(); }
    profile_->resetTiming(); data.add(",\"reset\":true"); return finish();
  }
  if (t.count == 1 && (t.is(0, "init") || t.is(0, "recover"))) {
    selectedValid_ = blockValid_ = false;
    if (!(t.is(0, "init") ? recorder_.initializeBlank() : recorder_.initialize())) error = FlightRecorder::errorName(recorder_.error());
    else data.add(",\"ready\":true,\"corrupt_flights\":%u",unsigned(recorder_.directory().corruptCount()));
    return finish();
  }
  if (!recorder_.initialized()) { error = "STORAGE_NOT_READY"; return finish(); }
  uint32_t index = 0, offset = 0; uint64_t sequence = 0;
  if (t.count==2 && t.is(0,"delete") && protocol::unsignedNumber64(t[1],sequence) && sequence) {
    selectedValid_=blockValid_=false;
    if (!recorder_.remove(sequence)) error=FlightRecorder::errorName(recorder_.error());
    else data.add(",\"deleted\":true,\"sequence\":\"%llu\"",static_cast<unsigned long long>(sequence));
    return finish();
  }
  if (t.count == 2 && t.is(0, "list") && protocol::unsignedNumber(t[1], index)) {
    unsigned retained = 0;
    // Directory identities are appended in increasing sequence order; updates
    // and compaction preserve that order. List the newest retained flight first.
    for (std::size_t i = recorder_.directory().count(); i > 0; --i) {
      const auto &entry = recorder_.directory().entries()[i-1];
      if (entry.status == FlightStatus::Invalidated) continue;
      if (retained++ != index) continue;
      data.add(",\"available\":true,\"sequence\":\"%llu\",\"status\":%u,\"corrupt\":%s,\"committed_bytes\":%lu,\"allocated_bytes\":%lu,\"start_address\":%lu",
               static_cast<unsigned long long>(entry.sequence), unsigned(entry.status), entry.status==FlightStatus::Corrupt ? "true" : "false", static_cast<unsigned long>(entry.committedBytes), static_cast<unsigned long>(entry.allocatedBytes),static_cast<unsigned long>(entry.start));
      return finish();
    }
    data.add(",\"available\":false"); return finish();
  }
  if (t.count >= 2 && protocol::unsignedNumber64(t[1], sequence)) {
    const bool info = t.count == 2 && t.is(0, "info");
    const bool summary = t.count == 2 && t.is(0, "summary");
    const bool config = t.count == 3 && t.is(0, "config") && protocol::unsignedNumber(t[2], offset);
    const bool read = t.count == 4 && t.is(0, "read") && protocol::unsignedNumber(t[2], index) && protocol::unsignedNumber(t[3], offset);
    if (!info && !summary && !config && !read) { error = "INVALID_ARGUMENT"; return finish(); }
    if (!select(sequence)) {
      error = recorder_.error() == RecorderError::None ? "Corrupt" : FlightRecorder::errorName(recorder_.error());
      return finish();
    }
    data.add(",\"sequence\":\"%llu\"", static_cast<unsigned long long>(sequence));
    if (summary) { protocol::appendRecordingSummary(data,selected_.summaryState,selected_.summary); return finish(); }
    const auto blocks = (selected_.entry.committedBytes - selected_.headerBytes) / FlightSampleStore::BlockBytes;
    if (info) {
      data.add(",\"status\":%u,\"samples\":%lu,\"blocks\":%lu,\"sensor_mode\":%u,\"output_mode\":%u,\"frame\":%u,\"config_version\":%u,\"config_bytes\":%u,\"starts\":[",
               unsigned(selected_.entry.status), static_cast<unsigned long>(selected_.samples), static_cast<unsigned long>(blocks),
               unsigned(sensorMode_), unsigned(outputMode_), unsigned(frame_), unsigned(storage::ConfigurationStore::FormatVersion), unsigned(configBytes_));
      for (unsigned i = 0; i < headerLayout_.slotCount; ++i) {
        const auto &s = starts_[i]; data.add("%s[%lu,%u,%u,%u,", i ? "," : "", static_cast<unsigned long>(s.rule), unsigned(s.kind), unsigned(s.channel), unsigned(s.state));
        if (s.state == StartState::Recorded) data.add("%lu", static_cast<unsigned long>(s.timeMs)); else data.add("null");
        data.add("]");
      }
      data.add("]"); return finish();
    }
    if (config) {
      if (offset >= configBytes_) { error = "Bounds"; return finish(); }
      const auto n = configBytes_ - offset < 128 ? configBytes_ - offset : 128;
      transferring_=true; lastTransferUs_=clock_ ? clock_() : 0;
      data.add(",\"offset\":%lu,\"hex\":\"", static_cast<unsigned long>(offset));
      for (std::size_t i = 0; i < n; ++i) data.add("%02x", unsigned(configuration_[offset+i]));
      data.add("\""); return finish();
    }
    if (index >= blocks) { error = "Bounds"; return finish(); }
    if (!blockValid_ || block_.index != index) {
      if (!recorder_.readBlock(sequence, index, block_)) { selectedValid_ = blockValid_ = false; error = FlightRecorder::errorName(recorder_.error()); return finish(); }
      blockValid_ = true;
    }
    if (offset >= block_.count) { error = "Bounds"; return finish(); }
    const unsigned count = block_.count - offset < 2 ? block_.count - offset : 2;
    transferring_=true; lastTransferUs_=clock_ ? clock_() : 0;
    data.add(",\"block\":%lu,\"offset\":%lu,\"first\":%lu,\"block_count\":%u,\"rows\":[",
             static_cast<unsigned long>(index), static_cast<unsigned long>(offset), static_cast<unsigned long>(block_.firstSample+offset), unsigned(block_.count));
    for (unsigned i = 0; i < count; ++i) {
      const auto &sample = block_.samples[offset+i]; data.add("%s[%lu", i ? "," : "", static_cast<unsigned long>(sample.timeMs));
      for (auto value : sample.values) { data.add(","); data.value(value); }
      data.add(",%u,", unsigned(sample.phase));
      if (sample.batteryMv==UINT16_MAX) data.add("null]");
      else data.add("%.3f]",double(sample.batteryMv)/1000.0);
    }
    data.add("]"); return finish();
  }
  error = "INVALID_ARGUMENT"; return finish();
}
