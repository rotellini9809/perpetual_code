#include "flight/ImuCalibration.h"
#include <cmath>
#include <algorithm>

namespace flight {
namespace { constexpr float G=9.80665f; }
void ImuCalibration::resetWindow() {
  samples_=0; startTime_=0;
  for (unsigned i=0;i<3;++i) sum_[i]=square_[i]=0;
}
void ImuCalibration::begin(bool verify,const storage::ImuCalibrationRecord &r) {
  *this=ImuCalibration{}; active_=true; verification_=verify; record_=r;
}
bool ImuCalibration::capture(unsigned face) {
  if (!active_ || collecting_ || face>=6) return false;
  face_=face; collecting_=true; have_=false; error_="None"; resetWindow(); return true;
}
void ImuCalibration::add(const sensors::Reading &r) {
  if (!collecting_) return;
  if (have_ && (r.timeUs<=lastTime_ || r.imuSequence<=lastSequence_)) return;
  if (have_ && r.timeUs-lastTime_>100000) resetWindow();
  have_=true; lastTime_=r.timeUs; lastSequence_=r.imuSequence;
  bool valid=r.imuValid; float a[3],g2=0,n2=0;
  for (unsigned i=0;i<3;++i) {
    a[i]=verification_ ? (r.accelMps2[i]-record_.bias[i])*record_.scale[i] : r.accelMps2[i];
    valid=valid && std::isfinite(a[i]) && std::isfinite(r.gyroRadps[i]);
    n2+=a[i]*a[i]; g2+=r.gyroRadps[i]*r.gyroRadps[i];
  }
  const unsigned axis=face_/2; const float sign=face_%2 ? -1.f : 1.f;
  // Provisional board qualification limits. Motion restarts the continuous window.
  valid=valid && std::fabs(std::sqrt(n2)-G)<(verification_ ?.35f:2.f) &&
      g2<.01f && sign*a[axis]>G*.94f;
  for (unsigned i=0;i<3;++i) if (i!=axis && std::fabs(a[i])>G*.15f) valid=false;
  if (!valid) { resetWindow(); error_="POSITION_OR_MOTION"; return; }
  if (!samples_) startTime_=r.timeUs;
  for (unsigned i=0;i<3;++i) { sum_[i]+=a[i]; square_[i]+=double(a[i])*a[i]; }
  ++samples_; error_="None";
  if (samples_<TargetSamples || r.timeUs-startTime_<2000000) return;
  norm_=noise_=0;
  for (unsigned i=0;i<3;++i) {
    const double mean=sum_[i]/samples_;
    means_[face_][i]=float(mean); norm_+=float(mean*mean);
    noise_=std::max(noise_,float(std::sqrt(std::max(0.,square_[i]/samples_-mean*mean))));
  }
  norm_=std::sqrt(norm_);
  if (noise_>.12f) { resetWindow(); error_="NOISY"; return; }
  mask_|=1u<<face_; collecting_=false;
}
bool ImuCalibration::result(storage::ImuCalibrationRecord &r) const {
  if (!complete()) return false;
  r=record_;
  if (verification_) { r.approval=storage::ImuApproval::Verified; return true; }
  if (r.revision==UINT32_MAX) return false;
  ++r.revision; r.approval=storage::ImuApproval::Required;
  for (unsigned i=0;i<3;++i) {
    const float plus=means_[2*i][i],minus=means_[2*i+1][i];
    r.bias[i]=(plus+minus)*.5f; r.scale[i]=2*G/(plus-minus);
    if (!std::isfinite(r.scale[i]) || r.scale[i]<.8f || r.scale[i]>1.2f || std::fabs(r.bias[i])>2) return false;
  }
  // Check residuals on every component, including cross-axis offsets.
  for (unsigned f=0;f<6;++f) for (unsigned i=0;i<3;++i) {
    const float expected=i==f/2 ? (f%2 ? -G:G):0;
    if (std::fabs((means_[f][i]-r.bias[i])*r.scale[i]-expected)>.4f) return false;
  }
  return true;
}
}
