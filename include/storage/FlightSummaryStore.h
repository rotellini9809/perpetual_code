#pragma once
#include "storage/RecordingTypes.h"
#include "storage/FlashDevice.h"
namespace storage {
enum class SummaryError : uint8_t { None, Bounds, InvalidArgument, NotErased, Io, Verify };
// One immutable 512-byte summary slot: descriptor + independent commit page.
class FlightSummaryStore {
public:
  explicit FlightSummaryStore(FlashDevice &flash) : flash_(flash) {}
  bool write(uint32_t address, uint64_t sequence, uint32_t headerAddress, uint32_t headerCrc, const RecordingSummary &summary);
  bool read(uint32_t address, uint64_t sequence, uint32_t headerAddress, uint32_t headerCrc, SummaryState &state, RecordingSummary &summary);
  SummaryError error() const { return error_; }
  static const char *errorName(SummaryError error);
private:
  bool fail(SummaryError error) { error_ = error; return false; }
  bool programPage(uint32_t address);
  FlashDevice &flash_;
  SummaryError error_ = SummaryError::None;
  uint8_t page_[256] = {}, check_[256] = {};
};
}
