#include "flight/ImuCalibration.h"
#include <cmath>
#include <algorithm>

namespace flight {
namespace { constexpr float StandardGravity=9.80665f; }
void ImuCalibration::resetWindow() {
  samples_=0; windowStartUs_=0;
  for (unsigned i=0;i<3;++i) sum_[i]=sumSquares_[i]=0;
}
void ImuCalibration::begin(bool verify,const storage::ImuCalibrationRecord &record) {
  *this=ImuCalibration{}; active_=true; verification_=verify; record_=record;
}
bool ImuCalibration::capture(unsigned face) {
  if (!active_ || collecting_ || failed_ || face>=6) return false;
  face_=face; collecting_=true; haveSample_=false; error_="None"; resetWindow(); return true;
}
void ImuCalibration::add(const sensors::Reading &reading) {
  if (!collecting_) return;
  if (haveSample_ && (reading.timeUs<=lastImuTimeUs_ || reading.imuSequence<=lastImuSequence_)) return;
  if (haveSample_ && reading.timeUs-lastImuTimeUs_>100000) resetWindow(); // A data gap is not the user's doing: restart silently.
  if (!haveSample_) settleStartUs_=reading.timeUs;
  haveSample_=true; lastImuTimeUs_=reading.timeUs; lastImuSequence_=reading.imuSequence;
  bool valid=reading.imuValid; float accel[3],gyroNormSquared=0,accelNormSquared=0;
  for (unsigned i=0;i<3;++i) {
    accel[i]=verification_ ? (reading.accelMps2[i]-record_.bias[i])*record_.scale[i] : reading.accelMps2[i];
    valid=valid && std::isfinite(accel[i]) && std::isfinite(reading.gyroRadps[i]);
    accelNormSquared+=accel[i]*accel[i]; gyroNormSquared+=reading.gyroRadps[i]*reading.gyroRadps[i];
  }
  const unsigned axis=face_/2; const float sign=face_%2 ? -1.f : 1.f;
  // Provisional board qualification limits. Motion restarts the continuous window.
  const float magnitude=std::sqrt(accelNormSquared);
  // Verification collects a stationary, correctly oriented window before judging
  // the coefficients. A stable wrong magnitude must fail, not retry forever.
  const float orientationMagnitude=verification_ ? magnitude : StandardGravity;
  valid=valid && magnitude>1.f && (verification_ || std::fabs(magnitude-StandardGravity)<2.f) &&
      gyroNormSquared<.01f && sign*accel[axis]>orientationMagnitude*.94f;
  if (verification_) { // Any tilt up to 10 degrees from the face, in any direction.
    float transverse=0;
    for (unsigned i=0;i<3;++i) if (i!=axis) transverse+=accel[i]*accel[i];
    if (!(std::sqrt(transverse)<=magnitude*VerificationTiltSin)) valid=false;
  } else for (unsigned i=0;i<3;++i) if (i!=axis && std::fabs(accel[i])>orientationMagnitude*.15f) valid=false;
  if (!valid) {
    resetWindow();
    if (reading.timeUs-settleStartUs_>=SettleUs) { collecting_=false; error_="POSITION_OR_MOTION"; } // Capture again.
    return;
  }
  if (!samples_) windowStartUs_=reading.timeUs;
  for (unsigned i=0;i<3;++i) { sum_[i]+=accel[i]; sumSquares_[i]+=double(accel[i])*accel[i]; }
  ++samples_; error_="None";
  if (samples_<TargetSamples || reading.timeUs-windowStartUs_<2000000) return;
  norm_=noise_=0;
  for (unsigned i=0;i<3;++i) {
    const double mean=sum_[i]/samples_;
    faceMeans_[face_][i]=float(mean); norm_+=float(mean*mean);
    noise_=std::max(noise_,float(std::sqrt(std::max(0.,sumSquares_[i]/samples_-mean*mean))));
  }
  norm_=std::sqrt(norm_);
  if (noise_>.12f) { resetWindow(); collecting_=false; error_="NOISY"; return; } // Capture again.
  // Verification judges only the corrected magnitude, which does not depend on how well the face
  // is aligned (the capture gate above already bounds that to about 8.6 degrees). The upward axis
  // carries almost all of gravity, so each face checks that axis' bias and scale.
  if (verification_) residual_[axis]=norm_-StandardGravity;
  if (verification_ && !(std::fabs(norm_-StandardGravity)<.35f)) { failed_=true; collecting_=false; error_="IMU_VERIFICATION_FAILED"; return; }
  capturedFaces_|=1u<<face_; collecting_=false;
}
bool ImuCalibration::result(storage::ImuCalibrationRecord &record) const {
  if (!complete()) return false;
  record=record_;
  if (verification_) { record.approval=storage::ImuApproval::Verified; return true; }
  if (record.revision==UINT32_MAX) return false;
  ++record.revision; record.approval=storage::ImuApproval::Required; record.identity=false;
  for (unsigned i=0;i<3;++i) record.residual[i]=NAN; // New coefficients: not verified yet.
  for (unsigned i=0;i<3;++i) {
    const float plus=faceMeans_[2*i][i],minus=faceMeans_[2*i+1][i];
    record.bias[i]=(plus+minus)*.5f; record.scale[i]=2*StandardGravity/(plus-minus);
    if (!std::isfinite(record.scale[i]) || record.scale[i]<.8f || record.scale[i]>1.2f || std::fabs(record.bias[i])>2) return false;
  }
  // Calibration is strict: every component, transverse included, within 0.4 m/s^2 of the ideal
  // face, so each face must be aligned within about 2.3 degrees (a stand helps).
  for (unsigned faceIndex=0;faceIndex<6;++faceIndex) for (unsigned i=0;i<3;++i) {
    const float expected=i==faceIndex/2 ? (faceIndex%2 ? -StandardGravity:StandardGravity):0;
    if (std::fabs((faceMeans_[faceIndex][i]-record.bias[i])*record.scale[i]-expected)>.4f) return false;
  }
  return true;
}
}
