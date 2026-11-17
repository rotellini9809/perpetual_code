#include "storage/ImuCalibrationStore.h"
#include "storage/FlashLayout.h"
#include "storage/BinaryEncoding.h"
#include <cmath>
#include <cstring>

namespace storage {
namespace {
constexpr uint32_t Magic=0x554d4950;
bool decode(const uint8_t *p, ImuCalibrationRecord &r) {
  if (binary::get32(p,4)!=Magic || p[4]!=1 || p[5]>2 ||
      binary::get32(p+252,4)!=binary::crc(p,252)) return false;
  r.revision=binary::get32(p+8,4); r.approval=ImuApproval(p[5]);
  for (unsigned i=0;i<3;++i) {
    uint32_t b=binary::get32(p+12+4*i,4),s=binary::get32(p+24+4*i,4);
    std::memcpy(&r.bias[i],&b,4); std::memcpy(&r.scale[i],&s,4);
    if (!std::isfinite(r.bias[i]) || std::fabs(r.bias[i])>2 ||
        !std::isfinite(r.scale[i]) || r.scale[i]<.8f || r.scale[i]>1.2f) return false;
  }
  return r.revision!=0;
}
}
bool ImuCalibrationStore::load() {
  present_=false; next_=0; record_={};
  bool damaged=false;
  for (unsigned n=0;n<FlashLayout::CalibrationBytes/256;++n) {
    uint8_t page[256];
    if (!flash_.read(FlashLayout::CalibrationBegin+n*256,page,sizeof(page))) { present_=false; return false; }
    bool empty=true; for (auto b:page) if (b!=255) empty=false;
    if (empty) continue;
    next_=n+1; ImuCalibrationRecord candidate;
    if (!decode(page,candidate) || (present_ && candidate.revision!=record_.revision)) { damaged=true; continue; }
    record_=candidate; present_=true; damaged=false;
  }
  // A torn approval write must never fall back to an older Verified record.
  if (damaged && present_) record_.approval=ImuApproval::Required;
  return present_;
}
bool ImuCalibrationStore::write(unsigned n,const ImuCalibrationRecord &r) {
  uint8_t page[256]; std::memset(page,255,sizeof(page));
  binary::put(page,Magic,4); page[4]=1; page[5]=uint8_t(r.approval);
  binary::put(page+8,r.revision,4);
  for (unsigned i=0;i<3;++i) {
    uint32_t b,s; std::memcpy(&b,&r.bias[i],4); std::memcpy(&s,&r.scale[i],4);
    binary::put(page+12+4*i,b,4); binary::put(page+24+4*i,s,4);
  }
  binary::put(page+252,binary::crc(page,252),4);
  ImuCalibrationRecord check;
  if (!decode(page,check)) return false;
  const auto address=FlashLayout::CalibrationBegin+n*256;
  uint8_t readback[256];
  return flash_.program(address,page,256) && flash_.read(address,readback,256) && !std::memcmp(page,readback,256);
}
bool ImuCalibrationStore::save(const ImuCalibrationRecord &r) {
  if (!r.revision || unsigned(r.approval)>2) return false;
  for (unsigned i=0;i<3;++i)
    if (!std::isfinite(r.bias[i]) || std::fabs(r.bias[i])>2 || !std::isfinite(r.scale[i]) || r.scale[i]<.8f || r.scale[i]>1.2f) return false;
  // Explicit idle operation. Loss of power during erase leaves calibration unavailable.
  present_=false;
  if (!flash_.eraseSector(FlashLayout::CalibrationBegin)) return false;
  next_=1;
  if (!write(0,r)) return false;
  record_=r; present_=true; return true;
}
bool ImuCalibrationStore::approve(ImuApproval approval) {
  if (!present_) return false;
  auto r=record_; r.approval=approval;
  if (next_>=FlashLayout::CalibrationBytes/256) {
    // Compaction is only called while idle, never in flight.
    return save(r);
  }
  const unsigned page=next_++;
  if (!write(page,r)) { record_.approval=ImuApproval::Required; return false; }
  record_=r; return true;
}
void ImuCalibrationStore::apply(navigation::SensorCalibration &c) const {
  if (!present_) return;
  c.accelCalibrationRevision=record_.revision;
  for (unsigned i=0;i<3;++i) { c.accelBias[i]=record_.bias[i]; c.accelScale[i]=record_.scale[i]; }
}
}
