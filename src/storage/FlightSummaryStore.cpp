#include "storage/FlightSummaryStore.h"
#include "storage/FlashLayout.h"
#include "storage/BinaryEncoding.h"
#include "storage/VerifiedPage.h"
#include <cstring>
namespace storage {
namespace {
using binary::put; using binary::get32; using binary::get64; using binary::crc;
constexpr uint32_t Magic = 0x4D535250, Commit = 0x43535250, Page = 256;
bool erased(const uint8_t *p, unsigned n) { for (unsigned i=0; i<n; ++i) if (p[i]!=255) return false; return true; }
bool valid(const RecordingSummary &s) {
  return unsigned(s.reason)>=1 && unsigned(s.reason)<=4 && s.queuePeak<=s.queueCapacity;
}
}
const char *FlightSummaryStore::errorName(SummaryError e) {
  switch (e) {
#define SUMMARY_ERROR(x) case SummaryError::x: return #x
    SUMMARY_ERROR(None); SUMMARY_ERROR(Bounds); SUMMARY_ERROR(InvalidArgument);
    SUMMARY_ERROR(NotErased); SUMMARY_ERROR(Io); SUMMARY_ERROR(Verify);
#undef SUMMARY_ERROR
  }
  return "Unknown";
}
bool FlightSummaryStore::programPage(uint32_t address) {
  const auto error = writeVerifiedPage(flash_, address, page_, check_);
  return error == PageWriteError::None || fail(error == PageWriteError::Io ? SummaryError::Io : SummaryError::Verify);
}
bool FlightSummaryStore::write(uint32_t address, uint64_t sequence, uint32_t headerAddress, uint32_t headerCrc, const RecordingSummary &s) {
  if (!sequence || !valid(s)) return fail(SummaryError::InvalidArgument);
  if (address%Page || !FlashLayout::dataRange(address,512)) return fail(SummaryError::Bounds);
  for (unsigned i=0; i<2; ++i) {
    if (!flash_.read(address+i*Page,check_,Page)) return fail(SummaryError::Io);
    if (!erased(check_,Page)) return fail(SummaryError::NotErased);
  }
  memset(page_,255,Page); put(page_,Magic,4); put(page_+4,1,2); put(page_+6,512,2);
  put(page_+8,sequence,8); put(page_+16,headerAddress,4); put(page_+20,headerCrc,4);
  put(page_+24,s.savedSamples,4); put(page_+28,s.droppedSamples,4);
  put(page_+32,s.queuePeak,4); put(page_+36,s.queueCapacity,4); page_[40]=uint8_t(s.reason);
  const auto checksum=crc(page_,252); put(page_+252,checksum,4);
  if (!programPage(address)) return false;
  memset(page_,255,Page); put(page_,Commit,4); put(page_+4,checksum,4); put(page_+8,~checksum,4);
  if (!programPage(address+Page)) return false;
  error_=SummaryError::None; return true;
}
bool FlightSummaryStore::read(uint32_t address, uint64_t sequence, uint32_t headerAddress, uint32_t headerCrc, SummaryState &state, RecordingSummary &summary) {
  if (address%Page || !FlashLayout::dataRange(address,512)) return fail(SummaryError::Bounds);
  if (!flash_.read(address,page_,Page) || !flash_.read(address+Page,check_,Page)) return fail(SummaryError::Io);
  SummaryState result=SummaryState::Incomplete; RecordingSummary s;
  const auto checksum=get32(page_+252,4);
  if (erased(page_,Page) && erased(check_,Page)) result=SummaryState::Missing;
  else if (get32(check_,4)==Commit && get32(check_+4,4)==checksum && get32(check_+8,4)==~checksum && erased(check_+12,244)) {
    result=SummaryState::Corrupt;
    if (get32(page_,4)==Magic && crc(page_,252)==checksum && get32(page_+6,2)==512 &&
        get64(page_+8,8)==sequence && get32(page_+16,4)==headerAddress && get32(page_+20,4)==headerCrc) {
      if (get32(page_+4,2)!=1) result=SummaryState::Unsupported;
      else {
        s.savedSamples=get32(page_+24,4); s.droppedSamples=get32(page_+28,4);
        s.queuePeak=get32(page_+32,4); s.queueCapacity=get32(page_+36,4); s.reason=static_cast<RecordingEnd>(page_[40]);
        if (valid(s) && erased(page_+41,211)) result=SummaryState::Valid;
      }
    }
  }
  state=result; summary=result==SummaryState::Valid ? s : RecordingSummary{};
  error_=SummaryError::None; return true;
}
}
