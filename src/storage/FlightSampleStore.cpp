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
bool FlightSampleStore::begin(uint64_t sequence, uint32_t start, uint32_t allocatedBytes) {
  if (active_) return fail(SampleError::Active);
  if (!sequence || !allocatedBytes) return fail(SampleError::InvalidArgument);
  if (start % FlashLayout::SectorBytes || allocatedBytes % FlashLayout::SectorBytes || !FlashLayout::dataRange(start, allocatedBytes)) return fail(SampleError::Bounds);
  // Read-only preflight: cannot overwrite stale/partially programmed data.
  for (uint32_t offset = 0; offset < allocatedBytes; offset += sizeof(page_)) {
    if (!flash_.read(start + offset, page_, sizeof(page_))) return fail(SampleError::Io);
    if (!erased(page_, sizeof(page_))) return fail(SampleError::NotErased);
  }
  sequence_ = sequence; start_ = start; allocatedBytes_ = allocatedBytes;
  blocks_ = committedSamples_ = 0; encoder_.reset(); active_ = true;
  error_ = SampleError::None; return true;
}
bool FlightSampleStore::append(const FlightSample &sample, bool history) {
  if (!active_) return fail(SampleError::Inactive);
  if (blocks_ == capacity()) return fail(SampleError::Full);
  auto result = encoder_.add(sample, history);
  if (result == flightlog::Encoder::Result::Full) {
    if (!flush()) return false;
    if (blocks_ == capacity()) return fail(SampleError::Full);
    result = encoder_.add(sample, history);
  }
  if (result != flightlog::Encoder::Result::Ok) return fail(SampleError::Timestamp); // Non-increasing time/sequence or invalid state.
  error_ = SampleError::None; return true;
}
bool FlightSampleStore::flush() {
  if (!active_) return fail(SampleError::Inactive);
  if (encoder_.empty()) { error_ = SampleError::None; return true; }
  if (blocks_ == capacity()) return fail(SampleError::Full);
  const uint32_t address = start_ + blocks_ * BlockBytes;
  const uint16_t frames = encoder_.frames();
  encoder_.seal(body_, sequence_, blocks_, address);
  // Any attempted mutation can have completed despite an I/O failure: never retry.
  // The extent was verified erased at begin; the driver verifies every page it programs.
  active_ = false;
  for (uint32_t offset = 0; offset < BodyBytes; offset += FlashLayout::PageBytes)
    if (!flash_.programErased(address + offset, body_ + offset, FlashLayout::PageBytes)) return fail(SampleError::Io);
  memset(page_, 255, sizeof(page_));
  const uint32_t checksum = uint32_t(get64(body_ + flightlog::CrcOffset, 4));
  put(page_, flightlog::CommitMagic, 4); put(page_ + 4, checksum, 4); put(page_ + 8, ~checksum, 4);
  if (!flash_.programErased(address + BodyBytes, page_, sizeof(page_))) return fail(SampleError::Io);
  // One end-to-end read-back of exactly what a download will read (about 2 ms per
  // block at 18.75 MHz, a few blocks per second), independent of the driver's check.
  for (uint32_t offset = 0; offset <= BodyBytes; offset += FlashLayout::PageBytes) {
    if (!flash_.read(address + offset, check_, sizeof(check_))) return fail(SampleError::Io);
    if (memcmp(check_, offset < BodyBytes ? body_ + offset : page_, sizeof(check_))) return fail(SampleError::Verify);
  }
  committedSamples_ += frames; ++blocks_; active_ = true;
  error_ = SampleError::None; return true;
}
bool FlightSampleStore::finish() {
  if (!flush()) return false;
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
