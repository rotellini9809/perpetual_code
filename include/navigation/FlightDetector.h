#pragma once
#include "navigation/VerticalKalman.h"
#include <array>

namespace navigation {
class FlightDetector {
public:
  void reset(const recovery::StateMachineSettings &settings = recovery::StateMachineSettings{});
  void update(float dt, float accelerationMagnitude, float bodyX, float gyroMagnitude,
              float barometerAltitude, float previousVerticalVelocity, float tiltRadians = 0);
  void confirmCoastVelocity(float velocity);
  // Confirmations are durations, independent of the IMU rate. Each is the time elapsed since the
  // first qualifying sample, so the interval before it (or a gap) never counts.
  // Launch and second-stage ignition: a sample above 3 g, then acceleration held above 1.5 g for 200 ms. The Hoppino flight
  // log peaks above 3 g for only 92 ms but holds about 2 g for its whole burn; hand shakes on the
  // bench stayed above 3 g for 34-57 ms and above 1.5 g for at most 90 ms.
  static constexpr float BoostTriggerMps2 = 3.0f * 9.81f, BoostHoldMps2 = 1.5f * 9.81f;
  static constexpr float BoostConfirmSeconds = 0.2f, CoastConfirmSeconds = 0.03f;
  static constexpr float ConfirmToleranceSeconds = 1e-4f; // Float sums of frame intervals.
  void interruptConfirmation() { boostTime_=coastTime_=-1; }
  // An ignition being confirmed (from PAD, or from COAST in the bistage machine). Navigation
  // already runs it as BOOST/BOOST 2, so the confirmation delay loses no velocity or attitude
  // weighting; if it is rejected, PAD pins the velocity again.
  bool launchPending() const { return phase_==Phase::Pad && boostTime_>=0; }
  Phase navigationPhase() const {
    if (boostTime_>=0 && phase_==Phase::Pad) return Phase::Boost;
    if (boostTime_>=0 && phase_==Phase::Coast) return Phase::Boost2;
    return phase_;
  }
  bool startsAllowed() const { return phase_!=Phase::Abort && !(settings_.bistage && phase_==Phase::Coast && samples_<8); }
  bool bistage() const { return settings_.bistage; }
  float velocityMedian() const { return median_; }
  recovery::AbortReason abortReason() const { return abortReason_; }
  Phase phase() const { return phase_; }
private:
  recovery::StateMachineSettings settings_;
  std::array<float,8> velocities_{};
  unsigned samples_ = 0;
  float boostTime_ = -1, coastTime_ = -1; // -1: no qualifying sample yet.
  bool ignition(float magnitude, float dt);
  // Adds this sample to a confirmation; true once it has lasted `seconds`.
  static bool confirm(float &elapsed, float dt, float seconds) {
    elapsed = elapsed < 0 ? 0 : elapsed + dt;
    if (elapsed < seconds - ConfirmToleranceSeconds) return false;
    elapsed = -1; return true;
  }
  float median_ = 0;
  recovery::AbortReason abortReason_ = recovery::AbortReason::None;
  Phase phase_ = Phase::Pad;
  float stable_ = 0, minimum_ = 0, maximum_ = 0;
  bool window_ = false;
};
}
