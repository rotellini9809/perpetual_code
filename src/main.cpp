#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <board_pins.h>
#include "PyroController.h"
#include "Sensors.h"

Adafruit_NeoPixel statusLed(1, RGB_LED, NEO_RGB + NEO_KHZ800);
PyroController pyros;
Sensors sensors;
SPIClassRP2040 memorySpi(FLASH_SPI_PORT, MISO_FLASH, CS_FLASH1, SCK_FLASH, MOSI_FLASH);

constexpr unsigned long CONTINUITY_LOG_INTERVAL_MS = 1000;
constexpr unsigned long IMU_STREAM_INTERVAL_MS = 1000;
constexpr unsigned long IMU_ERROR_LOG_INTERVAL_MS = 1000;
constexpr uint32_t MEMORY_SPI_HZ = 1000000;
constexpr uint32_t EXPECTED_W25Q64JV_JEDEC = 0xEF4017;
constexpr uint8_t CMD_FLASH_RELEASE_POWER_DOWN = 0xAB;
constexpr uint8_t CMD_FLASH_READ_JEDEC = 0x9F;
unsigned long lastContinuityLogMs = 0;
unsigned long lastImuStreamMs = 0;
unsigned long lastImuErrorLogMs = 0;

void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  statusLed.setPixelColor(0, statusLed.Color(r, g, b));
  statusLed.show();
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

void handlePyroKeyboardInput() {
  while (Serial.available() > 0) {
    const char input = static_cast<char>(Serial.read());
    if (input != 'f' && input != 'F') {
      continue;
    }

    pyros.enablePyros();
    if (pyros.firePyro(1)) {
      Serial.println("Firing pyro channel 1.");
    } else {
      pyros.disablePyros();
      Serial.println("Pyro channel 1 fire request rejected (busy or already fired).");
    }
  }
}

void streamImu(unsigned long now) {
  if (!sensors.imuOk() || now - lastImuStreamMs < IMU_STREAM_INTERVAL_MS) {
    return;
  }
  lastImuStreamMs = now;

  // The combined read can fail for the barometer while the IMU is valid.
  sensors.read();
  const SensorSample &sample = sensors.sample();
  if (!sample.imuValid) {
    if (now - lastImuErrorLogMs >= IMU_ERROR_LOG_INTERVAL_MS) {
      lastImuErrorLogMs = now;
      Serial.print("IMU read failed, error=");
      Serial.println(sensors.lastImuError());
    }
    return;
  }

  Serial.print("t_ms:");
  Serial.print(now);
  const char *accelLabels[] = {",ax_mps2:", ",ay_mps2:", ",az_mps2:"};
  const char *gyroLabels[] = {",gx_radps:", ",gy_radps:", ",gz_radps:"};
  for (uint8_t axis = 0; axis < 3; axis++) {
    Serial.print(accelLabels[axis]);
    Serial.print(sample.accelMps2[axis], 4);
  }
  for (uint8_t axis = 0; axis < 3; axis++) {
    Serial.print(gyroLabels[axis]);
    Serial.print(sample.gyroRadps[axis], 4);
  }
  Serial.print(",temp_C:");
  Serial.println(sample.imuTempC, 2);
}

void setup() {
  Serial.begin(115200);
  delay(5000);

  statusLed.begin();
  statusLed.setBrightness(40);
  setRgb(0, 0, 255);

  pyros.begin();
  Serial.println("PyroController ready. Pyros are disabled.");
  Serial.println("Press F in the Serial Monitor to fire pyro channel 1 once.");

  beginMemoryProbe();
  sensors.begin();
  if (sensors.imuOk()) {
    Serial.println("IMU ready. Sampling and streaming once per second: acceleration in m/s^2, gyro in rad/s, temperature in C.");
    setRgb(0, 80, 0);
  } else {
    Serial.print("IMU initialization failed. WHO_AM_I=0x");
    printHexByte(sensors.imuWhoAmI());
    Serial.print(" error=");
    Serial.println(sensors.lastImuError());
    setRgb(80, 0, 0);
  }
}

void loop() {
  pyros.update();
  handlePyroKeyboardInput();

  const unsigned long now = millis();
  streamImu(now);
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
