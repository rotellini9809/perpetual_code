#pragma once
#include "storage/ImuCalibrationStore.h"
#include <limits>

namespace flight {
// One explicit stationary acquisition at a time; no barometer dependency.
class ImuCalibration {
public:
  static constexpr unsigned TargetSamples=240;
  static constexpr float VerificationTiltSin=0.17364818f; // sin(10 deg): verification face tolerance.
  // After a capture request the board may still be settling (the click, the hand leaving): until then a
  // wrong sample only restarts the window. Later, a wrong sample or a noisy window stops the capture and
  // the user captures that face again.
  static constexpr uint64_t SettleUs=500000;
  // Faces 0..5 are +X, -X, +Y, -Y, +Z, -Z (the axis pointing up); bit n of mask() is face n.
  static constexpr unsigned AllFaces=0x3F, PositiveFaces=0x15; // PositiveFaces: +X, +Y, +Z.
  void begin(bool verification, const storage::ImuCalibrationRecord &record);
  bool capture(unsigned face);
  void cancel() { active_=collecting_=false; }
  void add(const sensors::Reading &reading);
  bool active() const { return active_; }
  bool collecting() const { return collecting_; }
  // Verification: one face per axis (+X, +Y, +Z), since a still reading tests mainly the upward axis.
  bool complete() const { return verification_ ? (capturedFaces_ & PositiveFaces)==PositiveFaces : capturedFaces_==AllFaces; }
  bool verification() const { return verification_; }
  bool failed() const { return failed_; }
  unsigned mask() const { return capturedFaces_; }
  unsigned face() const { return face_; }
  unsigned samples() const { return samples_; }
  const char *error() const { return error_; }
  float norm() const { return norm_; }
  // Verification: corrected |a| minus g for +X, +Y, +Z (NaN for a face not measured yet).
  const float *residuals() const { return residual_; }
  float noise() const { return noise_; }
  bool result(storage::ImuCalibrationRecord &record) const;
private:
  void resetWindow();
  bool active_=false,collecting_=false,verification_=false,haveSample_=false,failed_=false;
  unsigned capturedFaces_=0,face_=0,samples_=0;
  uint64_t lastImuTimeUs_=0,lastImuSequence_=0,windowStartUs_=0,settleStartUs_=0;
  double sum_[3]={},sumSquares_[3]={}; // Current window, for its mean and standard deviation.
  float faceMeans_[6][3]={},norm_=0,noise_=0; // Mean acceleration per captured face; last window stats.
  float residual_[3]={std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN()};
  storage::ImuCalibrationRecord record_;
  const char *error_="None";
};
}
