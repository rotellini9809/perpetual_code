#pragma once
#include "storage/FlashDevice.h"
#include "storage/FlashLayout.h"
#include "shared/Timing.h"

namespace storage {
struct FlashOperationTiming {
  timing::Statistics time;
  uint64_t bytes = 0;
  uint32_t failures = 0;
};
struct FlashTiming {
  FlashOperationTiming read, program, erase;
};
// One owner, RAM statistics only. Driver program/erase timings include its
// busy waits and verification; recorder-level readback is measured as read.
class ProfiledFlashDevice : public FlashDevice {
public:
  ProfiledFlashDevice(FlashDevice &device, timing::Clock clock = nullptr) : device_(device), clock_(clock) {}
  bool read(uint32_t address, void *data, std::size_t length) override {
    const auto start = now(); const bool ok = device_.read(address, data, length);
    record(stats_.read, start, length, ok); failure(ok, FlashOperation::Read, address, length); return ok;
  }
  bool program(uint32_t address, const void *data, std::size_t length) override {
    const auto start = now(); const bool ok = device_.program(address, data, length);
    record(stats_.program, start, length, ok); failure(ok, FlashOperation::Program, address, length); return ok;
  }
  bool programErased(uint32_t address, const void *data, std::size_t length) override {
    const auto start = now(); const bool ok = device_.programErased(address, data, length);
    record(stats_.program, start, length, ok); failure(ok, FlashOperation::Program, address, length); return ok;
  }
  bool eraseSector(uint32_t address) override {
    const auto start = now(); const bool ok = device_.eraseSector(address);
    record(stats_.erase, start, FlashLayout::SectorBytes, ok); failure(ok, FlashOperation::Erase, address, FlashLayout::SectorBytes); return ok;
  }
  bool eraseRange(uint32_t address, uint32_t bytes) override {
    const auto start = now(); const bool ok = device_.eraseRange(address, bytes);
    record(stats_.erase, start, bytes, ok); failure(ok, FlashOperation::Erase, address, bytes); return ok;
  }
  bool timingValid() const { return clock_ != nullptr; }
  const FlashTiming &timing() const { return stats_; }
  void resetTiming() { stats_ = {}; } // Caller must exclude active producers.
  FlashFailure lastFailure() const override { return failure_; }
private:
  void failure(bool ok, FlashOperation operation, uint32_t address, std::size_t bytes) {
    failure_ = {}; if (ok) return;
    failure_ = device_.lastFailure(); failure_.operation = operation;
    failure_.address = address; failure_.bytes = bytes;
    if (failure_.cause == FlashError::None) failure_.cause = FlashError::Unknown;
  }
  uint32_t now() const { return clock_ ? clock_() : 0; }
  void record(FlashOperationTiming &operation, uint32_t start, std::size_t bytes, bool ok) {
    const auto elapsed = uint32_t(now() - start);
    operation.time.add(elapsed); operation.bytes += bytes;
    if (!ok) ++operation.failures;
  }
  FlashDevice &device_;
  timing::Clock clock_;
  FlashTiming stats_;
  FlashFailure failure_;
};
}
