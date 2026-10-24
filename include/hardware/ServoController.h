#pragma once

#include <Arduino.h>
#include <Servo.h>

struct ServoConfig {
  uint8_t pin;
  uint16_t minPulse;
  uint16_t centerPulse;
  uint16_t maxPulse;
  float minAngle;
  float maxAngle;
};

class ServoController {
public:
  ServoController();

  void begin();
  bool setAngle(uint8_t channel, float angle);
  bool setPulse(uint8_t channel, uint16_t microseconds);
  bool center(uint8_t channel);
  bool stop(uint8_t channel);

private:
  static constexpr uint8_t CHANNEL_COUNT = 4;

  ServoConfig configs[CHANNEL_COUNT];
  Servo servoOutputs[CHANNEL_COUNT];
  bool active[CHANNEL_COUNT] = {};

  bool isValidChannel(uint8_t channel) const;
  uint8_t channelToIndex(uint8_t channel) const;
  uint16_t clampPulse(uint8_t index, uint16_t microseconds) const;
  bool attachChannel(uint8_t index, uint16_t initialPulse);
};
