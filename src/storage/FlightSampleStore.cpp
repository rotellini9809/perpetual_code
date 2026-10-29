#include "storage/FlightSampleStore.h"
#include "storage/BinaryEncoding.h"
#include "storage/VerifiedPage.h"
#include <cstring>
#include <cmath>

namespace storage {
namespace {
using binary::put;
using binary::get64;
using binary::crc;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "IEEE-754 binary32 required");
constexpr uint32_t Magic = 0x4B425350, Commit = 0x4D435350;
bool erased(const uint8_t *p, std::size_t n) { for (std::size_t i = 0; i < n; ++i) if (p[i] != 255) return false; return true; }
void encode(const FlightSample &s, uint8_t *p) {
  put(p, s.timeMs, 4);
  for (unsigned i = 0; i < 14; ++i) {
    uint32_t bits = 0x7FC00000u; // Canonical NaN for invalid/missing measurements.
    if (std::isfinite(s.values[i])) memcpy(&bits, &s.values[i], 4);
    put(p + 4 + i*4, bits, 4);
  }
  p[60] = uint8_t(s.phase); p[63]=uint8_t(s.abortReason);
  put(p + 61, s.batteryMv, 2);
  for (unsigned i=0;i<4;++i) put(p+64+2*i,s.airbrakes.opening[i],2);
  put(p+72,s.airbrakes.flags,2); p[74]=s.airbrakes.failedConditions; p[75]=s.airbrakes.channelMask;
}
bool decode(const uint8_t *p, FlightSample &s) {
  if (p[60] > uint8_t(recovery::FlightPhase::Abort) || p[63] > uint8_t(recovery::AbortReason::Tilt)) return false;
  s.airbrakes.flags=uint16_t(get64(p+72,2)); s.airbrakes.failedConditions=p[74]; s.airbrakes.channelMask=p[75];
  if (s.airbrakes.channelMask>15 || (s.airbrakes.flags & ~511u)) return false;
  for (unsigned i=0;i<4;++i) {
    const auto value=uint16_t(get64(p+64+2*i,2)); s.airbrakes.opening[i]=value;
    if ((s.airbrakes.channelMask&(1u<<i)) ? value>10000 : value!=65535) return false;
  }
  s.abortReason=static_cast<recovery::AbortReason>(p[63]);
  s.batteryMv = uint16_t(get64(p + 61, 2));
  s.timeMs = uint32_t(get64(p, 4)); s.phase = static_cast<recovery::FlightPhase>(p[60]);
  for (unsigned i = 0; i < 14; ++i) {
    const uint32_t bits = uint32_t(get64(p + 4 + i*4, 4)); memcpy(&s.values[i], &bits, 4);
    if (!std::isfinite(s.values[i]) && bits != 0x7FC00000u) return false;
  }
  return true;
}
}

void FlightSampleStore::encodeSample(const FlightSample &sample, uint8_t *record) { encode(sample,record); }

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
  blocks_ = committedSamples_ = lastTime_ = 0; buffered_ = 0; haveTime_ = false; active_ = true;
  error_ = SampleError::None; return true;
}
bool FlightSampleStore::append(const FlightSample &sample) {
  if (!active_) return fail(SampleError::Inactive);
  if (blocks_ == allocatedBytes_ / BlockBytes) return fail(SampleError::Full);
  if (buffered_ == SamplesPerBlock) return fail(SampleError::BufferFull);
  if (uint8_t(sample.abortReason)>uint8_t(recovery::AbortReason::Tilt)) return fail(SampleError::InvalidArgument);
  if (uint8_t(sample.phase) > uint8_t(recovery::FlightPhase::Abort)) return fail(SampleError::InvalidArgument);
  if (haveTime_ && sample.timeMs <= lastTime_) return fail(SampleError::Timestamp);
  pending_[buffered_++] = sample; haveTime_ = true; lastTime_ = sample.timeMs;
  error_ = SampleError::None; return true;
}
bool FlightSampleStore::flush() {
  if (!active_) return fail(SampleError::Inactive);
  if (!buffered_) { error_ = SampleError::None; return true; }
  const uint32_t address = start_ + blocks_ * BlockBytes;
  memset(body_, 255, sizeof(body_));
  put(body_, Magic, 4); put(body_ + 4, 3, 2); put(body_ + 6, RecordBytes, 2);
  put(body_ + 8, sequence_, 8); put(body_ + 16, blocks_, 4); put(body_ + 20, buffered_, 2);
  put(body_ + 24, committedSamples_, 4); put(body_ + 28, address, 4);
  for (unsigned i = 0; i < buffered_; ++i) encode(pending_[i], body_ + 32 + i*RecordBytes);
  const auto checksum = crc(body_, BodyBytes - 4); put(body_ + BodyBytes - 4, checksum, 4);
  // Any attempted mutation can have completed despite an I/O failure: never retry.
  active_ = false;
  for (unsigned i = 0; i < BodyBytes; i += sizeof(page_)) {
    const auto error = writeVerifiedPage(flash_, address + i, body_ + i, page_);
    if (error != PageWriteError::None) return fail(error == PageWriteError::Io ? SampleError::Io : SampleError::Verify);
  }
  memset(page_, 255, sizeof(page_)); put(page_, Commit, 4); put(page_ + 4, checksum, 4); put(page_ + 8, ~checksum, 4);
  if (!flash_.program(address + BodyBytes, page_, sizeof(page_))) return fail(SampleError::Io);
  SampleBlock verified;
  if (!readBlock(address, sequence_, blocks_, verified)) return error_==SampleError::Io ? false : fail(SampleError::Verify);
  if (verified.count != buffered_ || verified.firstSample != committedSamples_) return fail(SampleError::Verify);
  committedSamples_ += buffered_; ++blocks_; buffered_ = 0; active_ = true;
  error_ = SampleError::None; return true;
}
bool FlightSampleStore::finish() {
  if (!flush()) return false;
  active_ = false; return true;
}
bool FlightSampleStore::readBlock(uint32_t address, uint64_t sequence, uint32_t index, SampleBlock &output) {
  if (!sequence || address % BlockBytes || !FlashLayout::dataRange(address, BlockBytes)) return fail(SampleError::Bounds);
  if (!flash_.read(address, body_, sizeof(body_)) || !flash_.read(address + BodyBytes, page_, sizeof(page_))) return fail(SampleError::Io);
  if (erased(body_, sizeof(body_)) && erased(page_, sizeof(page_))) return fail(SampleError::Empty);
  const uint32_t checksum = uint32_t(get64(body_ + BodyBytes - 4, 4));
  if (get64(page_, 4) != Commit || get64(page_ + 4, 4) != checksum || get64(page_ + 8, 4) != uint32_t(~checksum) || !erased(page_ + 12, 244))
    return fail(SampleError::Incomplete);
  if (get64(body_, 4) != Magic || get64(body_ + 4, 2) != 3 || get64(body_ + 6, 2) != RecordBytes || crc(body_, BodyBytes - 4) != checksum)
    return fail(SampleError::Corrupt);
  if (get64(body_ + 8, 8) != sequence || get64(body_ + 16, 4) != index || get64(body_ + 28, 4) != address) return fail(SampleError::Identity);
  const auto count = uint16_t(get64(body_ + 20, 2));
  if (!count || count > SamplesPerBlock || !erased(body_ + 22, 2) || !erased(body_ + 32 + count*RecordBytes, BodyBytes - 4 - 32 - count*RecordBytes))
    return fail(SampleError::Corrupt);
  SampleBlock decoded; decoded.sequence = sequence; decoded.index = index;
  decoded.firstSample = uint32_t(get64(body_ + 24, 4)); decoded.count = count;
  if (decoded.firstSample > UINT32_MAX - count) return fail(SampleError::Corrupt);
  for (unsigned i = 0; i < count; ++i) {
    if (!decode(body_ + 32 + i*RecordBytes, decoded.samples[i]) || (i && decoded.samples[i].timeMs <= decoded.samples[i-1].timeMs))
      return fail(SampleError::Corrupt);
  }
  output = decoded; error_ = SampleError::None; return true;
}
}
