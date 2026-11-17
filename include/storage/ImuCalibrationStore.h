#pragma once
#include "storage/FlashDevice.h"
#include "navigation/SensorPipeline.h"

namespace storage {
enum class ImuApproval : uint8_t { Required, Verified, InUse };
struct ImuCalibrationRecord {
  uint32_t revision = 0;
  float bias[3] = {}, scale[3] = {1,1,1};
  ImuApproval approval = ImuApproval::Required;
};
// Dedicated sector: full coefficient record followed by append-only approval records.
// InUse is written before arming, so resets cannot resurrect a consumed approval.
class ImuCalibrationStore {
public:
  explicit ImuCalibrationStore(FlashDevice &flash) : flash_(flash) {}
  bool load();
  bool save(const ImuCalibrationRecord &record);
  bool approve(ImuApproval approval);
  bool present() const { return present_; }
  bool verified() const { return present_ && record_.approval==ImuApproval::Verified; }
  const ImuCalibrationRecord &record() const { return record_; }
  void apply(navigation::SensorCalibration &calibration) const;
private:
  bool write(unsigned page, const ImuCalibrationRecord &record);
  FlashDevice &flash_;
  ImuCalibrationRecord record_;
  bool present_ = false;
  unsigned next_ = 0;
};
}
