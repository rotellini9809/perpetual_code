#pragma once

#include <Arduino.h>

class PyroController {
public:
  enum class ContinuityState : uint8_t {
    Open,
    Connected,
    Uncertain,
    BatteryNotConnected
  };

  struct ContinuityReading {
    uint16_t raw;
    ContinuityState state;
  };

  void begin();
  void drive(bool enabled, uint8_t mask); // Exclusive owner: flight::OutputExecutor.
  ContinuityReading checkContinuity(uint8_t channel);

private:
  static constexpr uint8_t CHANNEL_COUNT = 4;
  static constexpr unsigned int ADC_SETTLE_US = 250;
  static constexpr uint8_t ADC_SAMPLE_COUNT = 16;
  // GP29 presence threshold (~0.5 V at the battery), not a low-voltage cutoff.
  static constexpr uint16_t BATTERY_CONNECTED_MIN_RAW = 120;
  static constexpr uint16_t CONTINUITY_CONNECTED_MIN_RAW = 1000;

  bool enabled_ = false;
  uint8_t mask_ = 0;

  bool isValidChannel(uint8_t channel) const;
  uint8_t channelToIndex(uint8_t channel) const;
  void selectContinuityChannel(uint8_t channel);
  ContinuityState classifyContinuity(uint16_t raw, uint16_t batteryRaw) const;
};
