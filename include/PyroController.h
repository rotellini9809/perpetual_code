#pragma once

#include <Arduino.h>

class PyroController {
public:
  enum class ContinuityState : uint8_t {
    Open,
    Connected,
    Uncertain
  };

  struct ContinuityReading {
    uint16_t raw;
    ContinuityState state;
  };

  void begin();
  void enablePyros();
  void disablePyros();
  bool firePyro(uint8_t channel);
  void update();
  ContinuityReading checkContinuity(uint8_t channel);

private:
  enum class FiringPhase : uint8_t {
    Idle,
    Start,
    EnableSettling,
    Pulse
  };

  static constexpr uint8_t CHANNEL_COUNT = 4;
  static constexpr uint8_t NO_CHANNEL = 0;

  static constexpr unsigned long ENABLE_SETTLE_MS = 10;
  static constexpr unsigned long PULSE_DURATION_MS = 1000;

  static constexpr unsigned int ADC_SETTLE_US = 250;
  static constexpr uint8_t ADC_SAMPLE_COUNT = 16;
  static constexpr uint16_t CONTINUITY_OPEN_MAX_RAW = 300;
  static constexpr uint16_t CONTINUITY_CONNECTED_MIN_RAW = 3000;

  bool softwareEnabled = false;
  uint8_t currentChannel = NO_CHANNEL;
  FiringPhase phase = FiringPhase::Idle;
  unsigned long phaseStartMs = 0;
  bool fired[CHANNEL_COUNT] = {};

  bool isValidChannel(uint8_t channel) const;
  uint8_t channelToIndex(uint8_t channel) const;
  uint8_t pyroPinForChannel(uint8_t channel) const;
  void setAllPyroOutputsLow();
  void selectContinuityChannel(uint8_t channel);
  ContinuityState classifyContinuity(uint16_t raw) const;
};
