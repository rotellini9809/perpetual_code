#pragma once
#include "hardware/SensorSource.h"
#include "shared/Timing.h"

namespace sensors {
// Onboard simulation sources, scheduled by the board clock:
// - Hoppino: the official BODY sensor log in firmware flash, at its logged timestamps (200 Hz).
// - Bistage: a synthetic two-burn profile at 200 Hz.
// - Hoppino480: a stress profile derived from the same log at the flight IMU rate. The log is
//   linearly interpolated at 480 Hz with a fresh barometer every 10 ms; PAD is extended by 12 s
//   (a full pre-launch history) and the descent is played at half speed (a longer recording that
//   grows the flight in the ring). It is generated on the fly: no extra table in flash.
class SimulatedSensorSource : public SensorSource {
public:
  enum class Profile : uint8_t { Hoppino, Bistage, Hoppino480 };
  // Hoppino480 time mapping, in microseconds of output time and of log time.
  static constexpr uint32_t StressRateHz = 480, StressBaroPeriodUs = 10000;
  static constexpr uint32_t StressPadHoldUs = 12000000, StressPadLoopUs = 6000000;
  static constexpr uint32_t StressStretchFromUs = 14000000, StressStretchToUs = 30000000, StressStretch = 2;
  static uint32_t periodUs(); // Logged Hoppino period.
  static uint32_t durationMs();
  static uint32_t sampleCount();
  static float nominalHz();
  static float referenceAltitude();
  explicit SimulatedSensorSource(timing::Clock clock) : clock_(clock) {}
  bool selectProfile(Profile profile) { if (running_) return false; profile_=profile; return true; }
  bool selectBistage(bool enabled) { return selectProfile(enabled ? Profile::Bistage : Profile::Hoppino); }
  Profile profile() const { return profile_; }
  bool bistage() const { return profile_==Profile::Bistage; }
  const char *profileName() const;
  uint32_t activeDurationMs() const;
  uint32_t activeSampleCount() const;
  uint32_t activePeriodUs() const;
  float activeNominalHz() const;
  float activeReferenceAltitude() const { return bistage() ? 0 : referenceAltitude(); }
  void start() override;
  void stop() override { running_ = false; }
  bool poll(Reading &reading) override;
  FrameConvention convention() const override { return FrameConvention::LoggedCalibratedBody; }
  bool finished() const { return finished_; }
  uint32_t missed() const { return missed_; }
  uint32_t maxLatenessUs() const { return maxLate_; }
  // Log time (us) played at a Hoppino480 output time (us); exposed for host checks.
  static uint32_t stressSourceTimeUs(uint64_t outputUs);
  static uint64_t stressTimeUs(uint32_t index) { return uint64_t(index)*1000000u/StressRateHz; }
private:
  bool pollBistage(uint32_t elapsed, Reading &reading);
  bool pollLogged(uint32_t elapsed, Reading &reading);
  bool pollStress(uint32_t elapsed, Reading &reading);
  timing::Clock clock_;
  Profile profile_ = Profile::Hoppino;
  bool running_ = false, finished_ = false;
  uint32_t origin_ = 0, next_ = 0, missed_ = 0, maxLate_ = 0, baroSequence_ = 0;
  uint32_t cursor_ = 0, baroSlot_ = 0; // Hoppino480: log row at or before the played time; last barometer slot.
};
}
