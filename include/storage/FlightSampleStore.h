#pragma once
#include "storage/RecordingTypes.h"
#include "storage/FlashLayout.h"
#include "storage/FlashDevice.h"
#include "storage/FlightLogCodec.h"
#include "shared/FlightTypes.h"
#include <array>
#include <limits>

namespace storage {
enum class SampleError : uint8_t {
  None, Bounds, InvalidArgument, Active, Inactive, NotErased, BufferFull,
  Full, Timestamp, Io, Empty, Incomplete, Corrupt, Identity, Verify
};
// One committed v4 block as stored: validated header fields plus the raw body.
// Records are decoded only on the PC (or by FlightLogDecoder.h in host tests).
struct RawBlock {
  uint64_t sequence = 0, firstTimeUs = 0;
  uint32_t index = 0, firstSequence = 0;
  uint16_t frames = 0;
  uint8_t body[flightlog::BodyBytes] = {};
};
// Low-level synchronous storage-worker component. Caller owns a contiguous,
// preallocated, erased extent. No erase or directory writes.
class FlightSampleStore {
public:
  static constexpr uint32_t BlockBytes = flightlog::BlockBytes, BodyBytes = flightlog::BodyBytes;
  explicit FlightSampleStore(FlashDevice &flash) : flash_(flash) {}
  bool begin(uint64_t sequence, uint32_t start, uint32_t allocatedBytes);
  // Encodes one sample group; when the RAM block is full it is first sealed and
  // written (7 body pages, then the commit page). Full means the extent is used up.
  bool append(const FlightSample &sample, bool history = false);
  bool flush(); // Writes the partially filled RAM block, if any.
  bool finish(); // Flush pending samples, then close; no footer/directory operation.
  bool readBlock(uint32_t address, uint64_t sequence, uint32_t index, RawBlock &output);
  bool active() const { return active_; }
  uint32_t committedSamples() const { return committedSamples_; }
  uint32_t committedBytes() const { return blocks_ * BlockBytes; }
  uint16_t bufferedSamples() const { return encoder_.frames(); }
  SampleError error() const { return error_; }
  static const char *errorName(SampleError error);
private:
  friend class FlightRecorder;
  void abandon() { active_ = false; encoder_.reset(); }
  bool fail(SampleError error) { error_ = error; return false; }
  uint32_t capacity() const { return allocatedBytes_ / BlockBytes; }
  FlashDevice &flash_;
  flightlog::Encoder encoder_;
  uint32_t start_ = 0, allocatedBytes_ = 0, blocks_ = 0, committedSamples_ = 0;
  uint64_t sequence_ = 0;
  bool active_ = false;
  uint8_t body_[BodyBytes] = {}, page_[256] = {}, check_[256] = {};
  SampleError error_ = SampleError::None;
};
}
