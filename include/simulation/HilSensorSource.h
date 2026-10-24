#pragma once
#include "hardware/SensorSource.h"

namespace sensors {
// A bounded one-frame mailbox. Caller must consume before submitting another.
class HilSensorSource : public SensorSource {
public:
  explicit HilSensorSource(FrameConvention convention = FrameConvention::SensorAxes) : convention_(convention) {}
  bool submit(const Reading &reading) {
    if (pending_) return false;
    reading_ = reading; pending_ = true; return true;
  }
  bool poll(Reading &reading) override {
    if (!pending_) return false;
    reading = reading_; pending_ = false; return true;
  }
  FrameConvention convention() const override { return convention_; }
  void clear() { pending_ = false; }
private:
  FrameConvention convention_;
  Reading reading_{};
  bool pending_ = false;
};
}
