#pragma once
#include <cstdint>

// First five modes follow FlightPhase. All animation is clock-driven, without delays.
enum class LedMode : uint8_t { Pad, Boost, Coast, Descent, Landed, Disarmed, Calibrating, PowerSaving };
class StatusLed {
public:
  using Writer = void (*)(uint8_t, uint8_t, uint8_t);
  explicit StatusLed(Writer write) : write_(write) {}
  void update(uint32_t nowMs, LedMode mode) {
    const bool changed=!initialized_ || mode!=mode_;
    if (!changed && uint32_t(nowMs-lastMs_)<20) return;
    if (changed) { mode_=mode; origin_=nowMs; }
    lastMs_=nowMs;
    uint32_t color=0;
    switch (mode) {
      case LedMode::Pad: color=0xFFA500; break;
      case LedMode::Boost: color=0xFF0000; break;
      case LedMode::Coast: color=0xFF00FF; break;
      case LedMode::Descent: color=0x00FFFF; break;
      case LedMode::Landed: color=0x00FF00; break;
      case LedMode::Disarmed: color=0x000050; break;
      case LedMode::Calibrating: {
        const auto step=uint32_t(nowMs-origin_)%1600;
        color=uint32_t(40+(215*(step<800 ? 800-step : step-800))/800); break;
      }
      case LedMode::PowerSaving: {
        auto hue=uint32_t(nowMs-origin_)/40%768;
        if (hue<256) color=((255-hue)<<16)|hue;
        else if (hue<512) { hue-=256; color=(hue<<8)|(255-hue); }
        else { hue-=512; color=(hue<<16)|((255-hue)<<8); }
        break;
      }
    }
    if (!initialized_ || color!=color_) write_(uint8_t(color>>16),uint8_t(color>>8),uint8_t(color));
    color_=color; initialized_=true;
  }
private:
  Writer write_;
  LedMode mode_=LedMode::Disarmed;
  uint32_t lastMs_=0, origin_=0, color_=0;
  bool initialized_=false;
};
