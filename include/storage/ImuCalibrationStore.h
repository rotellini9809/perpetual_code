#pragma once
#include "storage/FlashDevice.h"
#include "navigation/SensorPipeline.h"

namespace storage {
enum class ImuApproval : uint8_t { Required, Verified, InUse, Invalid };
struct ImuCalibrationRecord {
  uint32_t revision = 0;
  float bias[3] = {}, scale[3] = {1,1,1};
  ImuApproval approval = ImuApproval::Required;
  bool identity = false;
};
// Dedicated sector: full coefficient record followed by append-only approval records.
// InUse is written before arming, so resets cannot resurrect a consumed approval.
class ImuCalibrationStore {
public:
  explicit ImuCalibrationStore(FlashDevice &flash) : flash_(flash) {}
  bool load();
  bool save(const ImuCalibrationRecord &record);
  bool approve(ImuApproval approval);
  // After use (a flight or a reset while armed): invalidate and revoke the factory identity for good.
  bool invalidate();
  // A custom calibration that fails before any use: drop only it; identity returns if still allowed.
  bool discard();
  bool identityAllowed() const { return identityAllowed_; }
  bool needsCalibration() const { return present_ && (record_.approval==ImuApproval::Invalid || (record_.identity && !identityAllowed_)); }
  bool present() const { return present_; }
  bool verified() const { return present_ && record_.approval==ImuApproval::Verified; }
  const ImuCalibrationRecord &record() const { return record_; }
  void apply(navigation::SensorCalibration &calibration) const;
private:
  bool write(unsigned page, const ImuCalibrationRecord &record);
  FlashDevice &flash_;
  ImuCalibrationRecord record_;
  // One byte in its own sector, never erased by the application: erased (0xFF) as delivered means
  // the factory identity is allowed; written to 0x00 by a failed post-Boost verification, forever.
  bool loadIdentityPolicy();
  bool revokeIdentity();
  bool identityAllowed_ = false, policyReadable_ = false;
  bool present_ = false;
  unsigned next_ = 0;
};
}
