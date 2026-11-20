#pragma once
#include "storage/FlashDevice.h"
#include "storage/DeviceJournal.h"
#include "navigation/SensorPipeline.h"

namespace storage {
enum class ImuApproval : uint8_t { Required, Verified, InUse, Invalid };
struct ImuCalibrationRecord {
  uint32_t revision = 0;
  float bias[3] = {}, scale[3] = {1,1,1};
  ImuApproval approval = ImuApproval::Required;
  bool identity = false;
};
// Device journal type 1: every approval change is a new complete record (coefficients + state).
// InUse is written before arming, so resets cannot resurrect a consumed approval.
class ImuCalibrationStore {
public:
  explicit ImuCalibrationStore(FlashDevice &flash) : journal_(flash) {}
  bool load();
  bool save(const ImuCalibrationRecord &record);
  bool approve(ImuApproval approval);
  // After use (a flight or a reset while armed): invalidate and revoke the factory identity for good.
  bool invalidate();
  // A custom calibration that fails before any use: drop only it; identity returns if still allowed.
  bool discard();
  // Testing only (`imu identity`, IMU_IDENTITY_TEST builds): forget the calibration and the identity
  // revocation, as delivered; the next physical ARM stores the identity again.
  bool restoreIdentityForTesting();
  bool identityAllowed() const { return identityAllowed_; }
  bool needsCalibration() const { return present_ && (record_.approval==ImuApproval::Invalid || (record_.identity && !identityAllowed_)); }
  bool present() const { return present_; }
  bool verified() const { return present_ && record_.approval==ImuApproval::Verified; }
  const ImuCalibrationRecord &record() const { return record_; }
  void apply(navigation::SensorCalibration &calibration) const;
private:
  bool write(const ImuCalibrationRecord &record);
  DeviceJournal journal_;
  uint8_t buffer_[32] = {};
  ImuCalibrationRecord record_;
  // Journal type 4: absent as delivered (factory identity allowed); written by a failed
  // post-Boost verification and carried by every compaction, forever (only ERASE ALL resets it).
  bool loadIdentityPolicy();
  bool revokeIdentity();
  bool identityAllowed_ = false, policyReadable_ = false;
  bool present_ = false;
};
}
