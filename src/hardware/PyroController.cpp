#include "hardware/PyroController.h"

#include "hardware/board_pins.h"

namespace {
constexpr uint8_t PYRO_OUTPUT_PINS[] = {PYRO_1, PYRO_2, PYRO_3, PYRO_4};

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
  pinMode(BATT_SENSE, INPUT);
  analogReadResolution(12);

  enabled_=false; firingMask_=0;
}

void PyroController::drive(bool enabled, uint8_t mask) {
  mask = enabled ? uint8_t(mask & 0x0f) : 0;
  if (enabled==enabled_ && mask==firingMask_) return;
  if (!enabled) digitalWrite(PYRO_ENABLE,LOW);
  for (uint8_t i=0;i<CHANNEL_COUNT;++i) digitalWrite(PYRO_OUTPUT_PINS[i],(mask & (1u<<i)) ? HIGH : LOW);
  if (enabled) digitalWrite(PYRO_ENABLE,HIGH);
  enabled_=enabled; firingMask_=mask;
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
  // Read battery presence independently of the selected continuity channel.
  // Discard the first conversion after switching ADC inputs, as in MAIN.
  (void)analogRead(BATT_SENSE);
  uint32_t batteryTotal = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    const auto batterySample = uint16_t(analogRead(BATT_SENSE));
    if (batterySample >= 4095) return {raw, ContinuityState::Uncertain};
    batteryTotal += batterySample;
  }
  const auto batteryRaw = uint16_t((batteryTotal + 4) / 8);
  return {raw, classifyContinuity(raw, batteryRaw)};
}

bool PyroController::isValidChannel(uint8_t channel) const {
  return channel >= 1 && channel <= CHANNEL_COUNT;
}

uint8_t PyroController::channelToIndex(uint8_t channel) const {
  return channel - 1;
}

void PyroController::selectContinuityChannel(uint8_t channel) {
  const uint8_t index = channelToIndex(channel);
  digitalWrite(CONTINUITY_SELECT_0, (index & 0x01) ? HIGH : LOW);
  digitalWrite(CONTINUITY_SELECT_1, (index & 0x02) ? HIGH : LOW);
}

PyroController::ContinuityState PyroController::classifyContinuity(uint16_t raw, uint16_t batteryRaw) const {
  if (batteryRaw < BATTERY_CONNECTED_MIN_RAW) {
    return ContinuityState::BatteryNotConnected;
  }

  if (raw >= CONTINUITY_CONNECTED_MIN_RAW) {
    return ContinuityState::Connected;
  }

  return ContinuityState::Open;
}
