#pragma once
#include <cstdint>

// Board telemetry, independent of physical/HIL navigation sensor sources.
class BatteryMonitor {
public:
  static constexpr uint32_t IntervalMs = 200;
  static constexpr uint16_t Missing = UINT16_MAX;
  using Reader = uint16_t (*)(); // Averaged 12-bit ADC count; >=4095 means invalid/saturated.
  static uint16_t fromAdc(uint16_t raw) {
    // Nominal 3.3 V ADC reference, 100k battery->ADC and 24k ADC->GND: ratio 31/6.
    if (raw >= 4095) return Missing;
    return uint16_t((uint32_t(raw)*3300u*31u + 4095u*3u)/(4095u*6u));
  }
  void poll(uint32_t nowMs, Reader read) {
    if (sampled_ && uint32_t(nowMs-lastMs_) < IntervalMs) return;
    lastMs_=nowMs; sampled_=true; millivolts_=fromAdc(read());
  }
  uint16_t millivolts() const { return millivolts_; }
  void clear() { *this=BatteryMonitor{}; }
private:
  uint32_t lastMs_=0;
  uint16_t millivolts_=Missing;
  bool sampled_=false;
};
