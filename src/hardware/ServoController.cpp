#include "hardware/ServoController.h"

#include "hardware/board_pins.h"

namespace {
constexpr uint16_t DEFAULT_MIN_PULSE_US = 1000;
constexpr uint16_t DEFAULT_CENTER_PULSE_US = 1500;
constexpr uint16_t DEFAULT_MAX_PULSE_US = 2000;
constexpr float DEFAULT_MIN_ANGLE = 0.0f;
constexpr float DEFAULT_MAX_ANGLE = 180.0f;

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }

  if (value > maximum) {
    return maximum;
  }

  return value;
}
}

ServoController::ServoController()
    : configs{
          {SERVO_PWM_1, DEFAULT_MIN_PULSE_US, DEFAULT_CENTER_PULSE_US, DEFAULT_MAX_PULSE_US, DEFAULT_MIN_ANGLE,
           DEFAULT_MAX_ANGLE},
          {SERVO_PWM_2, DEFAULT_MIN_PULSE_US, DEFAULT_CENTER_PULSE_US, DEFAULT_MAX_PULSE_US, DEFAULT_MIN_ANGLE,
           DEFAULT_MAX_ANGLE},
          {SERVO_PWM_3, DEFAULT_MIN_PULSE_US, DEFAULT_CENTER_PULSE_US, DEFAULT_MAX_PULSE_US, DEFAULT_MIN_ANGLE,
           DEFAULT_MAX_ANGLE},
          {SERVO_PWM_4, DEFAULT_MIN_PULSE_US, DEFAULT_CENTER_PULSE_US, DEFAULT_MAX_PULSE_US, DEFAULT_MIN_ANGLE,
           DEFAULT_MAX_ANGLE},
      } {}

void ServoController::begin() {
  for (const ServoConfig &config : configs) {
    digitalWrite(config.pin, LOW);
    pinMode(config.pin, OUTPUT);
  }

  // Remain detached at boot; the selected physical run commands assigned initial positions.
}

bool ServoController::setAngle(uint8_t channel, float angle) {
  if (!isValidChannel(channel)) {
    return false;
  }

  const uint8_t index = channelToIndex(channel);
  const ServoConfig &config = configs[index];
  const float limitedAngle = clampFloat(angle, config.minAngle, config.maxAngle);
  const float angleRange = config.maxAngle - config.minAngle;
  if (angleRange <= 0.0f) {
    return setPulse(channel, config.centerPulse);
  }

  const float normalized = (limitedAngle - config.minAngle) / angleRange;
  const float pulseRange = static_cast<float>(config.maxPulse - config.minPulse);
  const uint16_t pulse = static_cast<uint16_t>(config.minPulse + (normalized * pulseRange) + 0.5f);
  return setPulse(channel, pulse);
}

bool ServoController::setPulse(uint8_t channel, uint16_t microseconds) {
  if (!isValidChannel(channel)) {
    return false;
  }

  const uint8_t index = channelToIndex(channel);
  const uint16_t pulse = clampPulse(index, microseconds);
  if (!attachChannel(index,pulse)) {
    return false;
  }

  servoOutputs[index].writeMicroseconds(pulse);
  active[index] = true;
  return true;
}

bool ServoController::center(uint8_t channel) {
  if (!isValidChannel(channel)) {
    return false;
  }

  return setPulse(channel, configs[channelToIndex(channel)].centerPulse);
}

bool ServoController::stop(uint8_t channel) {
  if (!isValidChannel(channel)) {
    return false;
  }

  const uint8_t index = channelToIndex(channel);
  const uint8_t pin = configs[index].pin;

  if (servoOutputs[index].attached()) {
    servoOutputs[index].detach();
  }

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  active[index] = false;
  return true;
}

bool ServoController::isValidChannel(uint8_t channel) const {
  return channel >= 1 && channel <= CHANNEL_COUNT;
}

uint8_t ServoController::channelToIndex(uint8_t channel) const {
  return channel - 1;
}

uint16_t ServoController::clampPulse(uint8_t index, uint16_t microseconds) const {
  const ServoConfig &config = configs[index];
  if (microseconds < config.minPulse) {
    return config.minPulse;
  }

  if (microseconds > config.maxPulse) {
    return config.maxPulse;
  }

  return microseconds;
}

bool ServoController::attachChannel(uint8_t index, uint16_t initialPulse) {
  if (servoOutputs[index].attached()) {
    return true;
  }

  const ServoConfig &config = configs[index];
  const int result = servoOutputs[index].attach(config.pin, config.minPulse, config.maxPulse, initialPulse);
  return result >= 0;
}
