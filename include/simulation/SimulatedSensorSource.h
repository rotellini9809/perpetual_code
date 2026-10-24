#pragma once
#include "hardware/SensorSource.h"
#include "shared/Timing.h"

namespace sensors {
// Official Hoppino BODY sensor log in firmware flash, scheduled by its timestamps.
class SimulatedSensorSource : public SensorSource {
public:
  static uint32_t periodUs();
  static uint32_t durationMs();
  static uint32_t sampleCount();
  static float nominalHz();
  static float referenceAltitude();
  explicit SimulatedSensorSource(timing::Clock clock) : clock_(clock) {}
  void start() override;
  void stop() override { running_ = false; }
  bool poll(Reading &reading) override;
  FrameConvention convention() const override { return FrameConvention::LoggedCalibratedBody; }
  bool finished() const { return finished_; }
  uint32_t missed() const { return missed_; }
  uint32_t maxLatenessUs() const { return maxLate_; }
private:
  timing::Clock clock_;
  bool running_ = false, finished_ = false;
  uint32_t origin_ = 0, next_ = 0, missed_ = 0, maxLate_ = 0, baroSequence_ = 0;
};
}
