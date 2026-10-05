#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <board_pins.h>
#include "PyroController.h"
#include "Sensors.h"

namespace {
Adafruit_NeoPixel statusLed(1, RGB_LED, NEO_RGB + NEO_KHZ800);
PyroController pyros;
Sensors sensors;
SPIClassRP2040 memorySpi(FLASH_SPI_PORT, MISO_FLASH, CS_FLASH1, SCK_FLASH, MOSI_FLASH);
unsigned long lastLogMs = 0;
bool memoryOk = false;
bool wasFiring = false;

void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  statusLed.setPixelColor(0, statusLed.Color(r, g, b));
  statusLed.show();
}

void printHex(uint32_t value, uint8_t digits) {
  Serial.print("0x");
  for (int shift = (digits - 1) * 4; shift >= 0; shift -= 4) {
    Serial.print((value >> shift) & 0xF, HEX);
  }
}

bool probeMemory() {
  digitalWrite(CS_FLASH1, HIGH);
  pinMode(CS_FLASH1, OUTPUT);
  memorySpi.begin();
  memorySpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_FLASH1, LOW);
  memorySpi.transfer(0xAB); // Release deep power-down before reading the ID.
  digitalWrite(CS_FLASH1, HIGH);
  memorySpi.endTransaction();
  delayMicroseconds(50);

  memorySpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_FLASH1, LOW);
  memorySpi.transfer(0x9F);
  uint32_t jedec = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    jedec = (jedec << 8) | memorySpi.transfer(0);
  }
  digitalWrite(CS_FLASH1, HIGH);
  memorySpi.endTransaction();

  Serial.print("FLASH CS=GP");
  Serial.print(CS_FLASH1);
  Serial.print(" JEDEC=");
  printHex(jedec, 6);
  switch (jedec) {
    case 0xEF4017:
      Serial.println(" PASS: W25Q64JV-compatible, 64 Mbit / 8 MiB");
      return true;
    case 0xEF4018:
      Serial.println(" PASS: W25Q128JV-compatible, 128 Mbit / 16 MiB");
      return true;
    default:
      Serial.println(" FAIL: absent or unrecognized (expected 0xEF4017 or 0xEF4018)");
      return false;
  }
}

const char *continuityName(PyroController::ContinuityState state) {
  switch (state) {
    case PyroController::ContinuityState::Open: return "UNCONNECTED";
    case PyroController::ContinuityState::Connected: return "CONNECTED";
    case PyroController::ContinuityState::BatteryNotConnected: return "BATTERY NOT CONNECTED";
    default: return "UNCERTAIN";
  }
}

void reportSensors() {
  Serial.print("IMU LSM6DSV32X CS=GP");
  Serial.print(CS_IMU);
  Serial.print(" WHO_AM_I register=");
  printHex(LSM6DSV32X_WHO_AM_I, 2);
  Serial.print(" read=");
  printHex(sensors.imuWhoAmI(), 2);
  Serial.print(" expected=");
  printHex(LSM6DSV32X_ID, 2);
  Serial.print(sensors.imuOk() ? " PASS" : " FAIL");
  Serial.print(" init_error=");
  Serial.println(sensors.lastImuError());

  Serial.print("BARO BMP5 CS=GP");
  Serial.print(CS_BARO);
  Serial.print(" CHIP_ID register=");
  printHex(BMP5_REG_CHIP_ID, 2);
  Serial.print(" raw=");
  printHex(sensors.baroRawChipId(), 2);
  Serial.print(" driver_id=");
  printHex(sensors.baroChipId(), 2);
  Serial.print(" expected=");
  printHex(BMP5_CHIP_ID_PRIM, 2);
  Serial.print(" or ");
  printHex(BMP5_CHIP_ID_SEC, 2);
  Serial.print(sensors.baroOk() ? " PASS" : " FAIL");
  Serial.print(" init_error=");
  Serial.println(sensors.lastBaroError());
}

void handleKeyboard() {
  if (!Serial.available()) return;
  const char key = static_cast<char>(Serial.read());
  if ((key == 'd' || key == 'D') && !pyros.isFiring()) {
    sensors.troubleshoot(Serial);
    return;
  }
  if (key != 'f' && key != 'F') return;
  if (pyros.isFiring()) return;

  pyros.enablePyros();
  if (pyros.firePyro(1)) {
    wasFiring = true;
    // Keep serial output and sensor reads out of the firing interval.
  } else {
    pyros.disablePyros();
    Serial.println("CH1 rejected: already fired. Reset to test again.");
  }
}

void logReadings() {
  sensors.read();
  const SensorSample &sample = sensors.sample();
  Serial.print("t_ms=");
  Serial.println(millis());
  if (sample.imuValid) {
    Serial.print("IMU accel_mps2=");
    for (uint8_t i = 0; i < 3; ++i) {
      if (i) Serial.print(',');
      Serial.print(sample.accelMps2[i], 4);
    }
    Serial.print(" gyro_radps=");
    for (uint8_t i = 0; i < 3; ++i) {
      if (i) Serial.print(',');
      Serial.print(sample.gyroRadps[i], 4);
    }
    Serial.print(" temp_C=");
    Serial.println(sample.imuTempC, 2);
  } else {
    Serial.print("IMU unavailable/read failed, error=");
    Serial.println(sensors.lastImuError());
  }
  if (sample.baroValid) {
    Serial.print("BARO pressure_Pa=");
    Serial.print(sample.pressurePa, 2);
    Serial.print(" temp_C=");
    Serial.println(sample.baroTempC, 2);
  } else {
    Serial.print("BARO unavailable/read failed, error=");
    Serial.println(sensors.lastBaroError());
  }
  for (uint8_t channel = 1; channel <= 4; ++channel) {
    const auto reading = pyros.checkContinuity(channel);
    Serial.print("CH");
    Serial.print(channel);
    Serial.print(" continuity=");
    Serial.print(reading.raw);
    Serial.print('/');
    Serial.println(continuityName(reading.state));
  }
  const bool healthy = memoryOk && sensors.ok() && sample.imuValid && sample.baroValid;
  setRgb(healthy ? 0 : 80, healthy ? 80 : 0, 0);
}
}

void setup() {
  // Both sensor chip selects are active-low; keep them inactive during the serial wait.
  pinMode(CS_LSM6DSV32X, INPUT_PULLUP);
  pinMode(CS_BMP581, INPUT_PULLUP);
  // Drive outputs inactive even while waiting indefinitely for USB serial.
  pyros.begin();
  statusLed.begin();
  statusLed.setBrightness(40);
  setRgb(0, 0, 255);
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("\nBoard smoke_test: pyros disabled");
  Serial.println("Sensors use SPI; checking identity registers, not I2C addresses.");
  sensors.begin(&Serial);
  reportSensors();
  sensors.troubleshoot(Serial);
  memoryOk = probeMemory();
  const bool healthy = sensors.ok() && memoryOk;
  Serial.println(healthy ? "Startup checks PASS" : "Startup checks FAIL: see device results above");
  Serial.println("Flash identification only; no erase/write test performed.");
  Serial.println("F/f fires CH1 once per boot (1 second pulse). Continuity CH1-4 follows.");
  Serial.println("D/d repeats sensor diagnostics without resetting the sensors; ignored during firing.");
  setRgb(healthy ? 0 : 80, healthy ? 80 : 0, 0);
}

void loop() {
  pyros.update();
  if (wasFiring && !pyros.isFiring()) {
    pyros.disablePyros();
    wasFiring = false;
    Serial.println("CH1 pulse complete; pyros disabled.");
  }
  handleKeyboard();
  if (pyros.isFiring() || !Serial) return;
  const unsigned long now = millis();
  if (now - lastLogMs >= 1000) {
    lastLogMs = now;
    logReadings();
  }
}
