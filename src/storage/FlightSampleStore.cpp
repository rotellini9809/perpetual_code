#include "storage/FlightSampleStore.h"
#include "storage/BinaryEncoding.h"
#include <cstring>

namespace storage {
namespace {
using binary::put;
using binary::get64;
using binary::crc;
bool erased(const uint8_t *p, std::size_t n) { for (std::size_t i = 0; i < n; ++i) if (p[i] != 255) return false; return true; }
}

const char *FlightSampleStore::errorName(SampleError e) {
  switch (e) {
#define SAMPLE_ERROR(x) case SampleError::x: return #x
    SAMPLE_ERROR(None); SAMPLE_ERROR(Bounds); SAMPLE_ERROR(InvalidArgument); SAMPLE_ERROR(Active);
    SAMPLE_ERROR(Inactive); SAMPLE_ERROR(NotErased); SAMPLE_ERROR(BufferFull); SAMPLE_ERROR(Full);
    SAMPLE_ERROR(Timestamp); SAMPLE_ERROR(Io); SAMPLE_ERROR(Empty); SAMPLE_ERROR(Incomplete);
    SAMPLE_ERROR(Corrupt); SAMPLE_ERROR(Identity); SAMPLE_ERROR(Verify);
#undef SAMPLE_ERROR
  }
  return "Unknown";
}
bool FlightSampleStore::begin(uint64_t sequence, uint32_t flightStart, uint32_t headerBytes, uint32_t limitBytes, uint32_t ownedBytes) {
  if (active_) return fail(SampleError::Active);
  if (!sequence || !limitBytes || !ownedBytes) return fail(SampleError::InvalidArgument);
  if (flightStart < FlashLayout::RingBegin || flightStart >= FlashLayout::Capacity || (flightStart - FlashLayout::RingBegin) % FlashLayout::BlockBytes ||
      headerBytes % FlashLayout::SectorBytes || headerBytes >= ownedBytes || ownedBytes % FlashLayout::BlockBytes ||
      ownedBytes > FlashLayout::RingBytes || limitBytes % FlashLayout::SectorBytes) return fail(SampleError::Bounds);
  // Read-only preflight of the erased base: cannot overwrite stale/partially programmed data.
  for (uint32_t offset = headerBytes; offset < ownedBytes; offset += sizeof(page_)) {
    if (!flash_.read(FlashLayout::ringAddress(flightStart, offset), page_, sizeof(page_))) return fail(SampleError::Io);
    if (!erased(page_, sizeof(page_))) return fail(SampleError::NotErased);
  }
  sequence_ = sequence; flightStart_ = flightStart; headerBytes_ = headerBytes; limitBytes_ = limitBytes;
  ownedBytes_ = erasedBytes_ = ownedBytes; eraseBusy_ = false;
  blocks_ = committedSamples_ = 0; pendingHead_ = pendingCount_ = 0; encoder_.reset(); active_ = true;
  error_ = SampleError::None; return true;
}
uint32_t FlightSampleStore::maxBlocks() const {
  const uint32_t space = ownedBytes_ - headerBytes_;
  return (space < limitBytes_ ? space : limitBytes_) / BlockBytes;
}
uint32_t FlightSampleStore::erasedAheadBytes() const {
  const uint32_t space = erasedBytes_ - headerBytes_, usable = space < limitBytes_ ? space : limitBytes_;
  const uint32_t sealed = (blocks_ + pendingCount_) * BlockBytes;
  return usable > sealed ? usable - sealed : 0;
}
bool FlightSampleStore::extend(uint32_t blocks) {
  if (!active_) return fail(SampleError::Inactive);
  if (!blocks || uint64_t(ownedBytes_) + uint64_t(blocks) * FlashLayout::BlockBytes > FlashLayout::RingBytes) return fail(SampleError::Bounds);
  ownedBytes_ += blocks * FlashLayout::BlockBytes;
  error_ = SampleError::None; return true;
}
bool FlightSampleStore::service() {
  if (!active_) return true;
  if (eraseBusy_) {
    bool done = false;
    if (!flash_.eraseDone(done)) { eraseBusy_ = false; active_ = false; return fail(SampleError::Io); }
    if (!done) return true; // The flash is busy: nothing else may touch it.
    eraseBusy_ = false; erasedBytes_ += FlashLayout::BlockBytes;
  }
  if (!drain(false)) return false; // Pending samples first, then the next erase.
  if (erasedBytes_ < ownedBytes_) {
    if (!flash_.startBlockErase(FlashLayout::ringAddress(flightStart_, erasedBytes_))) { active_ = false; return fail(SampleError::Io); }
    eraseBusy_ = true;
  }
  error_ = SampleError::None; return true;
}
bool FlightSampleStore::waitErase() {
  // Pending RAM is full or the stream is closing: block until the erase completes.
  while (eraseBusy_) {
    bool done = false;
    if (!flash_.eraseDone(done)) { eraseBusy_ = false; active_ = false; return fail(SampleError::Io); }
    if (done) { eraseBusy_ = false; erasedBytes_ += FlashLayout::BlockBytes; }
  }
  return true;
}
bool FlightSampleStore::drain(bool wait) {
  while (pendingCount_) {
    const uint32_t space = erasedBytes_ - headerBytes_;
    const uint32_t writable = (space < limitBytes_ ? space : limitBytes_) / BlockBytes;
    if (eraseBusy_ || blocks_ >= writable) {
      if (!wait) return true;
      if (eraseBusy_) { if (!waitErase()) return false; continue; }
      if (erasedBytes_ >= ownedBytes_) return fail(SampleError::Full);
      if (!flash_.startBlockErase(FlashLayout::ringAddress(flightStart_, erasedBytes_))) { active_ = false; return fail(SampleError::Io); }
      eraseBusy_ = true; continue;
    }
    if (!writeBlock(pending_[pendingHead_])) return false;
    committedSamples_ += pendingFrames_[pendingHead_]; ++blocks_;
    pendingHead_ = (pendingHead_ + 1) % PendingBlocks; --pendingCount_;
  }
  return true;
}
bool FlightSampleStore::seal() {
  if (encoder_.empty()) return true;
  if (pendingCount_ == PendingBlocks && !drain(true)) return false;
  if (pendingCount_ == PendingBlocks) return fail(SampleError::BufferFull);
  const unsigned slot = (pendingHead_ + pendingCount_) % PendingBlocks;
  const uint32_t index = blocks_ + pendingCount_;
  pendingFrames_[slot] = encoder_.frames();
  encoder_.seal(pending_[slot], sequence_, index, address(index));
  ++pendingCount_;
  return drain(false);
}
bool FlightSampleStore::append(const FlightSample &sample, bool history, bool deliberate) {
  if (!active_) return fail(SampleError::Inactive);
  if (blocks_ + pendingCount_ >= maxBlocks()) return fail(SampleError::Full);
  auto result = encoder_.add(sample, history, deliberate);
  if (result == flightlog::Encoder::Result::Full) {
    if (!seal()) return false;
    if (blocks_ + pendingCount_ >= maxBlocks()) return fail(SampleError::Full);
    result = encoder_.add(sample, history, deliberate);
  }
  if (result != flightlog::Encoder::Result::Ok) return fail(SampleError::Timestamp); // Non-increasing time/sequence or invalid state.
  error_ = SampleError::None; return true;
}
bool FlightSampleStore::writeBlock(const uint8_t *body) {
  const uint32_t address = this->address(blocks_);
  // Any attempted mutation can have completed despite an I/O failure: never retry.
  // The extent is erased (verified at begin or erased by service); the driver verifies
  // every page it programs.
  active_ = false;
  for (uint32_t offset = 0; offset < BodyBytes; offset += FlashLayout::PageBytes)
    if (!flash_.programErased(address + offset, body + offset, FlashLayout::PageBytes)) return fail(SampleError::Io);
  memset(page_, 255, sizeof(page_));
  const uint32_t checksum = uint32_t(get64(body + flightlog::CrcOffset, 4));
  put(page_, flightlog::CommitMagic, 4); put(page_ + 4, checksum, 4); put(page_ + 8, ~checksum, 4);
  if (!flash_.programErased(address + BodyBytes, page_, sizeof(page_))) return fail(SampleError::Io);
  // One end-to-end read-back of exactly what a download will read (about 2 ms per
  // block at 18.75 MHz, a few blocks per second), independent of the driver's check.
  for (uint32_t offset = 0; offset <= BodyBytes; offset += FlashLayout::PageBytes) {
    if (!flash_.read(address + offset, check_, sizeof(check_))) return fail(SampleError::Io);
    if (memcmp(check_, offset < BodyBytes ? body + offset : page_, sizeof(check_))) return fail(SampleError::Verify);
  }
  active_ = true; error_ = SampleError::None; return true;
}
bool FlightSampleStore::flush() {
  if (!active_) return fail(SampleError::Inactive);
  if (!seal() || !drain(true)) return false;
  error_ = SampleError::None; return true;
}
bool FlightSampleStore::finish() {
  if (!flush() || !waitErase()) return false; // Leave the flash idle.
  active_ = false; return true;
}
bool FlightSampleStore::readBlock(uint32_t address, uint64_t sequence, uint32_t index, RawBlock &output) {
  using namespace flightlog;
  if (!sequence || address % BlockBytes || !FlashLayout::dataRange(address, BlockBytes)) return fail(SampleError::Bounds);
  if (!flash_.read(address, body_, sizeof(body_)) || !flash_.read(address + BodyBytes, page_, sizeof(page_))) return fail(SampleError::Io);
  if (erased(body_, sizeof(body_)) && erased(page_, sizeof(page_))) return fail(SampleError::Empty);
  const uint32_t checksum = uint32_t(get64(body_ + CrcOffset, 4));
  if (get64(page_, 4) != CommitMagic || get64(page_ + 4, 4) != checksum || get64(page_ + 8, 4) != uint32_t(~checksum) || !erased(page_ + 12, 244))
    return fail(SampleError::Incomplete);
  if (get64(body_, 4) != Magic || get64(body_ + 4, 2) != Version || get64(body_ + 6, 2) != BodyBytes || crc(body_, CrcOffset) != checksum)
    return fail(SampleError::Corrupt);
  if (get64(body_ + 8, 8) != sequence || get64(body_ + 16, 4) != index || get64(body_ + 20, 4) != address) return fail(SampleError::Identity);
  const auto frames = uint16_t(get64(body_ + 36, 2)), used = uint16_t(get64(body_ + 38, 2));
  if (!frames || frames > MaxFrames || !used || used > PayloadBytes || used < frames * MinFrameBytes ||
      !erased(body_ + 56, PayloadOffset - 56) || !erased(body_ + PayloadOffset + used, PayloadBytes - used))
    return fail(SampleError::Corrupt);
  output.sequence = sequence; output.index = index; output.frames = frames;
  output.firstTimeUs = get64(body_ + 24, 8); output.firstSequence = uint32_t(get64(body_ + 32, 4));
  memcpy(output.body, body_, sizeof(body_));
  error_ = SampleError::None; return true;
}
}
