#pragma once
#include "storage/RecordingTypes.h"
#include <array>

namespace storage {
// Pre-launch history kept in RAM while the phase is PAD: the last 0.5 s at full
// rate, and before that one sample every 100 ms for 10 s. A sample leaving the
// full-rate window is kept only on the 100 ms grid; the newest barometer reading
// seen since the previous kept sample travels with it. Nothing touches flash.
class PadHistory {
public:
  static constexpr unsigned FastCapacity = 256, SlowCapacity = 100; // 0.53 s at 480 Hz; 10 s at 10 Hz.
  static constexpr uint64_t FastWindowUs = 500000, SlowSpacingUs = 100000;
  void clear() { fastHead_ = fastCount_ = slowHead_ = slowCount_ = 0; haveSlow_ = baroPending_ = false; }
  void push(const FlightSample &sample) {
    if (fastCount_ == FastCapacity) promote();
    fast_[(fastHead_ + fastCount_) % FastCapacity] = sample; ++fastCount_;
    while (fastCount_ > 1 && sample.timeUs - fast_[fastHead_].timeUs > FastWindowUs) promote();
  }
  unsigned size() const { return slowCount_ + fastCount_; }
  // Oldest to newest; stops early when visit returns false.
  template<typename Visit> bool forEach(Visit visit) const {
    for (unsigned i = 0; i < slowCount_; ++i) if (!visit(slow_[(slowHead_ + i) % SlowCapacity])) return false;
    for (unsigned i = 0; i < fastCount_; ++i) if (!visit(fast_[(fastHead_ + i) % FastCapacity])) return false;
    return true;
  }
private:
  void promote() {
    FlightSample sample = fast_[fastHead_];
    fastHead_ = (fastHead_ + 1) % FastCapacity; --fastCount_;
    if (sample.baroFresh) {
      baroTimeUs_ = sample.timeUs - sample.baroAgeUs; baroPending_ = true;
      pressure_ = sample.values[FlightSample::PressurePa]; temperature_ = sample.values[FlightSample::TemperatureC];
    }
    if (haveSlow_ && sample.timeUs - lastSlowUs_ < SlowSpacingUs) return;
    if (!sample.baroFresh && baroPending_ && sample.timeUs - baroTimeUs_ <= 0xFFFF) {
      sample.baroFresh = true; sample.baroAgeUs = uint16_t(sample.timeUs - baroTimeUs_);
      sample.values[FlightSample::PressurePa] = pressure_; sample.values[FlightSample::TemperatureC] = temperature_;
    }
    baroPending_ = false;
    if (slowCount_ == SlowCapacity) { slowHead_ = (slowHead_ + 1) % SlowCapacity; --slowCount_; }
    slow_[(slowHead_ + slowCount_) % SlowCapacity] = sample; ++slowCount_;
    lastSlowUs_ = sample.timeUs; haveSlow_ = true;
  }
  std::array<FlightSample, FastCapacity> fast_;
  std::array<FlightSample, SlowCapacity> slow_;
  unsigned fastHead_ = 0, fastCount_ = 0, slowHead_ = 0, slowCount_ = 0;
  uint64_t lastSlowUs_ = 0, baroTimeUs_ = 0;
  bool haveSlow_ = false, baroPending_ = false;
  float pressure_ = 0, temperature_ = 0;
};
}
