#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <board_pins.h>
#include "PyroController.h"

Adafruit_NeoPixel statusLed(1, RGB_LED, NEO_RGB + NEO_KHZ800);
PyroController pyros;

constexpr unsigned long CONTINUITY_LOG_INTERVAL_MS = 1000;
unsigned long lastContinuityLogMs = 0;

void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  statusLed.setPixelColor(0, statusLed.Color(r, g, b));
  statusLed.show();
}

const char *continuityStateName(PyroController::ContinuityState state) {
  switch (state) {
    case PyroController::ContinuityState::Open:
      return "OPEN";
    case PyroController::ContinuityState::Connected:
      return "CONNECTED";
    case PyroController::ContinuityState::Uncertain:
      return "UNCERTAIN";
  }

  return "UNCERTAIN";
}

void setup() {
  Serial.begin(115200);

  statusLed.begin();
  statusLed.setBrightness(40);
  setRgb(0, 0, 255);

  pyros.begin();
  Serial.println("PyroController ready. Pyros are disabled.");
}

void loop() {
  pyros.update();

  const unsigned long now = millis();
  if (now - lastContinuityLogMs >= CONTINUITY_LOG_INTERVAL_MS) {
    lastContinuityLogMs = now;

    const PyroController::ContinuityReading reading = pyros.checkContinuity(1);
    Serial.print("CH1 continuity=");
    Serial.print(reading.raw);
    Serial.print("/");
    Serial.print(continuityStateName(reading.state));
    Serial.println();
  }
}
