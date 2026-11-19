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
// Low-level synchronous storage-worker component for one flight on the ring.
// Sample block i lives at flight offset headerBytes + i * BlockBytes, mapped onto the ring.
// The flight owns 64 KiB ring blocks given by its caller (directory ownership first); blocks
// added by extend() are erased asynchronously by service(), while sealed blocks wait in a RAM
// ring of PendingBlocks. No directory writes.
class FlightSampleStore {
public:
  static constexpr uint32_t BlockBytes = flightlog::BlockBytes, BodyBytes = flightlog::BodyBytes;
  static constexpr unsigned PendingBlocks = 16; // About 2 s at 476 Hz: a worst-case 64 KiB erase.
  explicit FlightSampleStore(FlashDevice &flash) : flash_(flash) {}
  // ownedBytes (flight bytes from the start, including the header) are already erased.
  bool begin(uint64_t sequence, uint32_t flightStart, uint32_t headerBytes, uint32_t limitBytes, uint32_t ownedBytes);
  // Adds owned 64 KiB blocks after the current ones; service() erases them in order.
  bool extend(uint32_t blocks);
  // Advances an erase in progress, starts the next one and writes pending blocks.
  bool service();
  // Encodes one sample group; a full RAM block is sealed and written or queued.
  // Full means the sample limit or the owned extent is used up.
  bool append(const FlightSample &sample, bool history = false, bool deliberate = false);
  bool flush(); // Seals the partially filled RAM block and writes everything pending.
  bool finish(); // Flush, finish any erase, then close; no footer/directory operation.
  bool readBlock(uint32_t address, uint64_t sequence, uint32_t index, RawBlock &output);
  uint32_t address(uint32_t index) const { return FlashLayout::ringAddress(flightStart_, headerBytes_ + index * BlockBytes); }
  bool active() const { return active_; }
  uint32_t committedSamples() const { return committedSamples_; }
  uint32_t committedBytes() const { return blocks_ * BlockBytes; }
  uint16_t bufferedSamples() const { return encoder_.frames(); }
  uint32_t ownedBytes() const { return ownedBytes_; }
  bool erasing() const { return eraseBusy_ || erasedBytes_ < ownedBytes_; }
  // Erased sample space not yet holding a written or pending block.
  uint32_t erasedAheadBytes() const;
  bool limitCovered() const { return ownedBytes_ - headerBytes_ >= limitBytes_; }
  SampleError error() const { return error_; }
  static const char *errorName(SampleError error);
private:
  friend class FlightRecorder;
  void abandon() { active_ = false; encoder_.reset(); pendingCount_ = 0; }
  bool fail(SampleError error) { error_ = error; return false; }
  uint32_t maxBlocks() const;
  bool seal();
  bool drain(bool wait);
  bool waitErase();
  bool writeBlock(const uint8_t *body);
  FlashDevice &flash_;
  flightlog::Encoder encoder_;
  uint32_t flightStart_ = 0, headerBytes_ = 0, limitBytes_ = 0, ownedBytes_ = 0, erasedBytes_ = 0;
  uint32_t blocks_ = 0, committedSamples_ = 0;
  uint64_t sequence_ = 0;
  bool active_ = false, eraseBusy_ = false;
  uint8_t pending_[PendingBlocks][BodyBytes] = {};
  uint16_t pendingFrames_[PendingBlocks] = {};
  unsigned pendingHead_ = 0, pendingCount_ = 0;
  uint8_t body_[BodyBytes] = {}, page_[256] = {}, check_[256] = {};
  SampleError error_ = SampleError::None;
};
}
