#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <board_pins.h>
#include "PyroController.h"

Adafruit_NeoPixel statusLed(1, RGB_LED, NEO_RGB + NEO_KHZ800);
PyroController pyros;
SPIClassRP2040 memorySpi(FLASH_SPI_PORT, MISO_FLASH, CS_FLASH1, SCK_FLASH, MOSI_FLASH);

constexpr unsigned long CONTINUITY_LOG_INTERVAL_MS = 1000;
// TEMP BENCH TEST ONLY: use only with no igniters or live pyro charges connected.
constexpr bool PYRO_ENABLE_LED_TEST = true;
constexpr uint32_t MEMORY_SPI_HZ = 1000000;
constexpr uint32_t EXPECTED_W25Q64JV_JEDEC = 0xEF4017;
constexpr uint8_t CMD_FLASH_RELEASE_POWER_DOWN = 0xAB;
constexpr uint8_t CMD_FLASH_READ_JEDEC = 0x9F;
unsigned long lastContinuityLogMs = 0;

void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  statusLed.setPixelColor(0, statusLed.Color(r, g, b));
  statusLed.show();
}

void setPyroEnableLedTest(bool enabled) {
  digitalWrite(PYRO_1, LOW);
  digitalWrite(PYRO_2, LOW);
  digitalWrite(PYRO_3, LOW);
  digitalWrite(PYRO_4, LOW);
  digitalWrite(PYRO_ENABLE, enabled ? HIGH : LOW);
}

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print("0");
  }
  Serial.print(value, HEX);
}

void printHex24(uint32_t value) {
  printHexByte(static_cast<uint8_t>((value >> 16) & 0xFF));
  printHexByte(static_cast<uint8_t>((value >> 8) & 0xFF));
  printHexByte(static_cast<uint8_t>(value & 0xFF));
}

void flashSelect() {
  digitalWrite(CS_FLASH1, LOW);
}

void flashDeselect() {
  digitalWrite(CS_FLASH1, HIGH);
}

void releaseFlashPowerDown() {
  memorySpi.beginTransaction(SPISettings(MEMORY_SPI_HZ, MSBFIRST, SPI_MODE0));
  flashSelect();
  memorySpi.transfer(CMD_FLASH_RELEASE_POWER_DOWN);
  flashDeselect();
  memorySpi.endTransaction();
  delayMicroseconds(50);
}

uint32_t readMemoryJedec() {
  memorySpi.beginTransaction(SPISettings(MEMORY_SPI_HZ, MSBFIRST, SPI_MODE0));
  flashSelect();
  memorySpi.transfer(CMD_FLASH_READ_JEDEC);
  const uint8_t manufacturer = memorySpi.transfer(0);
  const uint8_t memoryType = memorySpi.transfer(0);
  const uint8_t capacity = memorySpi.transfer(0);
  flashDeselect();
  memorySpi.endTransaction();

  return (static_cast<uint32_t>(manufacturer) << 16) | (static_cast<uint32_t>(memoryType) << 8) | capacity;
}

void beginMemoryProbe() {
  digitalWrite(CS_FLASH1, HIGH);
  pinMode(CS_FLASH1, OUTPUT);
  memorySpi.begin();
  releaseFlashPowerDown();

  const uint32_t jedec = readMemoryJedec();
  Serial.print("Memory JEDEC=0x");
  printHex24(jedec);
  Serial.print(" expected W25Q64JVSSIQ=0x");
  printHex24(EXPECTED_W25Q64JV_JEDEC);
  Serial.println(jedec == EXPECTED_W25Q64JV_JEDEC ? " MATCH" : " MISMATCH");
}

const char *continuityStateName(PyroController::ContinuityState state) {
  switch (state) {
    case PyroController::ContinuityState::Open:
      return "UNCONNECTED";
    case PyroController::ContinuityState::Connected:
      return "CONNECTED";
    case PyroController::ContinuityState::Uncertain:
      return "UNCERTAIN";
    case PyroController::ContinuityState::BatteryNotConnected:
      return "BATTERY NOT CONNECTED";
  }

  return "UNCERTAIN";
}

void setup() {
  Serial.begin(115200);
  delay(5000);

  statusLed.begin();
  statusLed.setBrightness(40);
  setRgb(0, 0, 255);

  pyros.begin();
  Serial.println("PyroController ready. Pyros are disabled.");
  if (PYRO_ENABLE_LED_TEST) {
    setPyroEnableLedTest(true);
    Serial.println("TEMP TEST: PYRO_ENABLE held HIGH for LED test. Fire outputs held LOW.");
  }

  beginMemoryProbe();
  setRgb(0, 80, 0);
}

void loop() {
  pyros.update();
  if (PYRO_ENABLE_LED_TEST) {
    setPyroEnableLedTest(true);
  }

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
