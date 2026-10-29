#pragma once
#include "configuration/RecoveryRuleEngine.h"
#include "shared/Timing.h"

namespace flight {
// GPIO/PWM adapter only. The executor owns pulse timing and per-run state.
class OutputHardware {
public:
  virtual ~OutputHardware() = default;
  virtual bool continuity(uint8_t channel) = 0;
  virtual void pyros(bool enabled, uint8_t mask) = 0;
  virtual bool servo(uint8_t channel, uint16_t pulseUs) = 0;
  virtual void releaseServo(uint8_t channel) = 0;
};

class OutputExecutor {
public:
  OutputExecutor(OutputHardware &hardware, timing::Clock clock) : hardware_(hardware), clock_(clock) {}
  bool timingAvailable() const { return clock_ != nullptr; }
  uint8_t connectedPyros(uint8_t mask);
  const char *prepare(const recovery::Configuration &config, uint8_t mask);
  uint8_t preparedServos(const recovery::Configuration &config) const;
  bool hasPreparation() const { return preparedMask_ != 0; }
  bool hasPreview() const { return previewMask_ != 0; }
  const char *preview(uint8_t channel, uint16_t pulseUs);
  void releasePreview(uint8_t channel = 0);
  void releasePrepared();
  void reconcile(const recovery::Configuration *config);
  const char *begin(const recovery::Configuration &config, uint64_t owner, bool requirePrepared = false, bool enableAirbrakes = true);
  bool submit(const recovery::VirtualStart &start);
  bool airbrakes(uint64_t owner, const std::array<float,4> &opening);
  void inhibit(uint64_t owner);
  void poll(bool allowNewStarts = true); // Called every cooperative loop, also when no sensor frame is due.
  std::size_t drain(recovery::VirtualStart *starts, std::size_t capacity);
  unsigned stop(uint64_t owner);
  uint64_t owner() const { return owner_; }
  bool active() const { return owner_ && (servoMask_ || pendingPyros_ || highPyros_); }
  bool pulsing() const { return pendingPyros_ || highPyros_; }
private:
  OutputHardware &hardware_;
  timing::Clock clock_;
  uint64_t owner_ = 0;
  uint32_t epochUs_ = 0, enableUs_ = 0;
  uint8_t airbrakeMask_ = 0;
  uint8_t servoMask_ = 0, allowedPyros_ = 0, requestedPyros_ = 0, requestedServos_ = 0;
  uint8_t pendingPyros_ = 0, highPyros_ = 0;
  bool enabled_ = false;
  uint8_t preparedMask_ = 0;
  uint8_t previewMask_ = 0; // Held PWM for positioning, never valid flight preparation.
  std::array<uint16_t, recovery::ChannelCount> preparedInitial_{}, preparedFinal_{};
  std::array<uint16_t, recovery::ChannelCount> initialUs_{}, finalUs_{};
  std::array<uint32_t, recovery::ChannelCount> pulseUs_{};
  std::array<recovery::VirtualStart, recovery::ChannelCount> pyroStarts_{};
  std::array<recovery::VirtualStart, recovery::MaxAssignments> started_{};
  std::size_t startCount_ = 0;
  void report(recovery::VirtualStart start, uint32_t now);
};
}
