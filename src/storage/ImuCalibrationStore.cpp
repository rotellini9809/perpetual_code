#include "storage/ImuCalibrationStore.h"
#include "storage/FlashLayout.h"
#include "storage/BinaryEncoding.h"
#include <cmath>
#include <cstring>

namespace storage {
namespace {
// Journal type 1 payload: approval, identity marker, revision, accel bias XYZ, accel scale XYZ.
constexpr std::size_t RecordBytes = 30;
bool validCoefficients(const ImuCalibrationRecord &r) {
  for (unsigned i=0;i<3;++i)
    if (!std::isfinite(r.bias[i]) || std::fabs(r.bias[i])>2 || !std::isfinite(r.scale[i]) || r.scale[i]<.8f || r.scale[i]>1.2f) return false;
  return r.revision!=0;
}
void encode(const ImuCalibrationRecord &r, uint8_t *p) {
  p[0]=uint8_t(r.approval); p[1]=r.identity ? 0x49 : 0xFF; binary::put(p+2,r.revision,4);
  for (unsigned i=0;i<3;++i) {
    uint32_t b,s; std::memcpy(&b,&r.bias[i],4); std::memcpy(&s,&r.scale[i],4);
    binary::put(p+6+4*i,b,4); binary::put(p+18+4*i,s,4);
  }
}
bool decode(const uint8_t *p, ImuCalibrationRecord &r) {
  if (p[0]>3 || (p[1]!=0xFF && p[1]!=0x49)) return false;
  r.approval=ImuApproval(p[0]); r.identity=p[1]==0x49; r.revision=binary::get32(p+2,4);
  for (unsigned i=0;i<3;++i) {
    uint32_t b=binary::get32(p+6+4*i,4),s=binary::get32(p+18+4*i,4);
    std::memcpy(&r.bias[i],&b,4); std::memcpy(&r.scale[i],&s,4);
  }
  return validCoefficients(r);
}
}
bool ImuCalibrationStore::load() {
  present_=false; identityAllowed_=policyReadable_=false; record_={};
  if (!loadIdentityPolicy()) { if (!policyReadable_) return false; }
  DeviceJournal::Found found;
  if (!journal_.read(DeviceJournal::ImuCalibration,buffer_,sizeof(buffer_),found)) return false;
  // A torn or unreadable newer record never falls back to an older approval.
  const bool damaged=found.damagedAfter;
  if (found.present && found.length==RecordBytes && decode(buffer_,record_)) present_=true;
  else record_={};
  if (damaged && present_) record_.approval=ImuApproval::Invalid;
  // Only a persisted Invalid record comes from invalidate(): finish a revocation interrupted by a reset.
  // A torn write (damaged) is not a failed verification and never revokes the identity.
  if (present_ && !damaged && record_.approval==ImuApproval::Invalid && identityAllowed_) revokeIdentity();
  return present_;
}
bool ImuCalibrationStore::write(const ImuCalibrationRecord &r) {
  uint8_t payload[RecordBytes]; encode(r,payload);
  ImuCalibrationRecord check;
  if (!decode(payload,check)) return false;
  return journal_.write(DeviceJournal::ImuCalibration,payload,sizeof(payload));
}
bool ImuCalibrationStore::save(const ImuCalibrationRecord &r) {
  if (!r.revision || unsigned(r.approval)>3 || (r.identity && !identityAllowed_) || !validCoefficients(r)) return false;
  if (!policyReadable_ && !loadIdentityPolicy() && !policyReadable_) return false; // Unknown policy: do not write.
  // Written beside the previous calibration: a power cut keeps the old one.
  if (!write(r)) { record_.approval=ImuApproval::Invalid; return false; }
  record_=r; present_=true; return true;
}
bool ImuCalibrationStore::approve(ImuApproval approval) {
  if (!present_ || (approval!=ImuApproval::Invalid && needsCalibration())) return false;
  auto r=record_; r.approval=approval;
  if (!write(r)) { record_.approval=ImuApproval::Invalid; return false; }
  record_=r; return true;
}
bool ImuCalibrationStore::loadIdentityPolicy() {
  DeviceJournal::Found found;
  if (!journal_.read(DeviceJournal::IdentityRevoked,buffer_,sizeof(buffer_),found)) return false;
  policyReadable_=true;
  identityAllowed_=!found.present; // The revocation record is copied by every compaction; only ERASE ALL removes it.
  return identityAllowed_;
}
bool ImuCalibrationStore::revokeIdentity() {
  identityAllowed_=false;
  DeviceJournal::Found found;
  if (journal_.read(DeviceJournal::IdentityRevoked,buffer_,sizeof(buffer_),found) && found.present) return true;
  return journal_.write(DeviceJournal::IdentityRevoked,nullptr,0);
}
bool ImuCalibrationStore::discard() {
  if (!present_) return true;
  if (!identityAllowed_) return approve(ImuApproval::Invalid); // Nothing to fall back to: recalibrate.
  ImuCalibrationRecord identity; identity.identity=true; identity.approval=ImuApproval::Verified;
  identity.revision=record_.revision==UINT32_MAX ? record_.revision : record_.revision+1;
  return save(identity);
}
bool ImuCalibrationStore::invalidate() {
  const bool saved=approve(ImuApproval::Invalid);
  if (present_) record_.approval=ImuApproval::Invalid;
  const bool revoked=revokeIdentity();
  return saved && revoked;
}
void ImuCalibrationStore::apply(navigation::SensorCalibration &c) const {
  if (!present_) return;
  c.accelCalibrationRevision=record_.revision;
  for (unsigned i=0;i<3;++i) { c.accelBias[i]=record_.bias[i]; c.accelScale[i]=record_.scale[i]; }
}
}
