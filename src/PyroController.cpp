#include "PyroController.h"

#include <board_pins.h>

namespace {
constexpr uint8_t PYRO_OUTPUT_PINS[] = {PYRO_1, PYRO_2, PYRO_3, PYRO_4};

bool hasElapsed(unsigned long now, unsigned long start, unsigned long duration) {
  return now - start >= duration;
}
}

void PyroController::begin() {
  for (uint8_t pin : PYRO_OUTPUT_PINS) {
    pinMode(pin, INPUT_PULLDOWN);
    digitalWrite(pin, LOW);
  }

  for (uint8_t pin : PYRO_OUTPUT_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  digitalWrite(PYRO_ENABLE, LOW);
  pinMode(PYRO_ENABLE, OUTPUT);

  digitalWrite(CONTINUITY_SELECT_0, LOW);
  digitalWrite(CONTINUITY_SELECT_1, LOW);
  pinMode(CONTINUITY_SELECT_0, OUTPUT);
  pinMode(CONTINUITY_SELECT_1, OUTPUT);

  pinMode(CONTINUITY_SENSE, INPUT);
  analogReadResolution(12);

  softwareEnabled = false;
  currentChannel = NO_CHANNEL;
  phase = FiringPhase::Idle;
  phaseStartMs = 0;
  for (bool &channelFired : fired) {
    channelFired = false;
  }
}

void PyroController::enablePyros() {
  setAllPyroOutputsLow();
  digitalWrite(PYRO_ENABLE, LOW);
  softwareEnabled = true;
}

void PyroController::disablePyros() {
  setAllPyroOutputsLow();
  digitalWrite(PYRO_ENABLE, LOW);
  softwareEnabled = false;
  currentChannel = NO_CHANNEL;
  phase = FiringPhase::Idle;
  phaseStartMs = 0;
}

bool PyroController::firePyro(uint8_t channel) {
  if (!softwareEnabled || phase != FiringPhase::Idle || !isValidChannel(channel)) {
    return false;
  }

  if (fired[channelToIndex(channel)]) {
    return false;
  }

  currentChannel = channel;
  phase = FiringPhase::Start;
  phaseStartMs = millis();
  return true;
}

void PyroController::update() {
  const unsigned long now = millis();

  switch (phase) {
    case FiringPhase::Idle:
      return;

    case FiringPhase::Start:
      setAllPyroOutputsLow();
      digitalWrite(PYRO_ENABLE, HIGH);
      phaseStartMs = now;
      phase = FiringPhase::EnableSettling;
      return;

    case FiringPhase::EnableSettling:
      if (hasElapsed(now, phaseStartMs, ENABLE_SETTLE_MS)) {
        digitalWrite(pyroPinForChannel(currentChannel), HIGH);
        phaseStartMs = now;
        phase = FiringPhase::Pulse;
      }
      return;

    case FiringPhase::Pulse:
      if (hasElapsed(now, phaseStartMs, PULSE_DURATION_MS)) {
        digitalWrite(pyroPinForChannel(currentChannel), LOW);
        digitalWrite(PYRO_ENABLE, LOW);
        fired[channelToIndex(currentChannel)] = true;
        currentChannel = NO_CHANNEL;
        phase = FiringPhase::Idle;
      }
      return;
  }
}

PyroController::ContinuityReading PyroController::checkContinuity(uint8_t channel) {
  if (!isValidChannel(channel)) {
    return {0, ContinuityState::Uncertain};
  }

  selectContinuityChannel(channel);
  delayMicroseconds(ADC_SETTLE_US);
  (void)analogRead(CONTINUITY_SENSE);

  uint32_t total = 0;
  for (uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++) {
    total += analogRead(CONTINUITY_SENSE);
    delayMicroseconds(ADC_SETTLE_US);
  }

  const uint16_t raw = static_cast<uint16_t>(total / ADC_SAMPLE_COUNT);
  return {raw, classifyContinuity(raw)};
}

bool PyroController::isValidChannel(uint8_t channel) const {
  return channel >= 1 && channel <= CHANNEL_COUNT;
}

uint8_t PyroController::channelToIndex(uint8_t channel) const {
  return channel - 1;
}

uint8_t PyroController::pyroPinForChannel(uint8_t channel) const {
  return PYRO_OUTPUT_PINS[channelToIndex(channel)];
}

void PyroController::setAllPyroOutputsLow() {
  for (uint8_t pin : PYRO_OUTPUT_PINS) {
    digitalWrite(pin, LOW);
  }
}

void PyroController::selectContinuityChannel(uint8_t channel) {
  const uint8_t index = channelToIndex(channel);
  digitalWrite(CONTINUITY_SELECT_0, (index & 0x01) ? HIGH : LOW);
  digitalWrite(CONTINUITY_SELECT_1, (index & 0x02) ? HIGH : LOW);
}

PyroController::ContinuityState PyroController::classifyContinuity(uint16_t raw) const {
  if (raw <= CONTINUITY_OPEN_MAX_RAW) {
    return ContinuityState::Open;
  }

  if (raw >= CONTINUITY_CONNECTED_MIN_RAW) {
    return ContinuityState::Connected;
  }

  return ContinuityState::Uncertain;
}
