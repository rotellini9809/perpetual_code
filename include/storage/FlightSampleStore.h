#pragma once
#include "storage/RecordingTypes.h"
#include "storage/FlashLayout.h"
#include "storage/FlashDevice.h"
#include "shared/FlightTypes.h"
#include <array>
#include <limits>

namespace storage {
enum class SampleError : uint8_t {
  None, Bounds, InvalidArgument, Active, Inactive, NotErased, BufferFull,
  Full, Timestamp, Io, Empty, Incomplete, Corrupt, Identity, Verify
};
struct SampleBlock {
  uint64_t sequence = 0;
  uint32_t index = 0, firstSample = 0;
  uint16_t count = 0;
  std::array<FlightSample, 9> samples;
};
// Low-level synchronous storage-worker component; never call flush from navigation.
// Caller owns a contiguous, preallocated, erased extent. No erase or directory writes.
class FlightSampleStore {
public:
  static constexpr uint32_t RecordBytes = 76, BodyBytes = 768, BlockBytes = 1024;
  static constexpr uint16_t SamplesPerBlock = 9;
  explicit FlightSampleStore(FlashDevice &flash) : flash_(flash) {}
  bool begin(uint64_t sequence, uint32_t start, uint32_t allocatedBytes);
  bool append(const FlightSample &sample); // RAM only; full buffer requires worker flush.
  bool flush(); // Writes 3 body pages, verifies, then writes a separate commit page.
  bool finish(); // Flush pending samples, then close; no footer/directory operation.
  bool readBlock(uint32_t address, uint64_t sequence, uint32_t index, SampleBlock &output);
  bool active() const { return active_; }
  uint32_t committedSamples() const { return committedSamples_; }
  uint32_t committedBytes() const { return blocks_ * BlockBytes; }
  uint16_t bufferedSamples() const { return buffered_; }
  SampleError error() const { return error_; }
  static const char *errorName(SampleError error);
  static void encodeSample(const FlightSample &sample, uint8_t *record);
private:
  friend class FlightRecorder;
  void abandon() { active_ = false; buffered_ = 0; }
  bool fail(SampleError error) { error_ = error; return false; }
  FlashDevice &flash_;
  uint32_t start_ = 0, allocatedBytes_ = 0, blocks_ = 0, committedSamples_ = 0, lastTime_ = 0;
  uint64_t sequence_ = 0;
  bool active_ = false, haveTime_ = false;
  uint16_t buffered_ = 0;
  std::array<FlightSample, SamplesPerBlock> pending_;
  uint8_t body_[BodyBytes] = {}, page_[256] = {};
  SampleError error_ = SampleError::None;
};
}
