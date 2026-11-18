#include "diagnostics/BuildMode.h"
#ifdef SENSOR_TEST
#include "hardware/Sensors.h"
#include "hardware/board_pins.h"
#include <hardware/gpio.h>

namespace {
// Test-only direct register access and the normal driver use hardware SPI1
// sequentially on this core. Neither wrapper ends the shared peripheral.
SPIClassRP2040 probeSpi(SENSOR_SPI_PORT, MISO_SENSOR, CS_IMU, SCK_SENSOR, MOSI_SENSOR);
Sensors device;
struct Probe {
  uint32_t hz = 100000;
  uint8_t mode = SPI_MODE0;
  bool driven = true;
  unsigned matches = 0;
};
bool connected = false, streaming = false, initialized = false;
uint32_t lastRead = 0, lastReport = 0, imuAt = 0, baroAt = 0;
unsigned imuFresh = 0, baroFresh = 0;
bool imuSeen = false, baroSeen = false;

void inactiveOutputs() {
  for (uint8_t pin : {PYRO_ENABLE, PYRO_1, PYRO_2, PYRO_3, PYRO_4,
                      SERVO_PWM_1, SERVO_PWM_2, SERVO_PWM_3, SERVO_PWM_4}) {
    digitalWrite(pin, LOW); pinMode(pin, OUTPUT);
  }
  digitalWrite(CS_FLASH1, HIGH); pinMode(CS_FLASH1, OUTPUT);
}
void deselect(bool driven) {
  for (uint8_t pin : {CS_IMU, CS_BARO}) {
    if (driven) { digitalWrite(pin, HIGH); pinMode(pin, OUTPUT); }
    else pinMode(pin, INPUT_PULLUP); // Match the current production driver.
  }
}
uint8_t readRegister(uint8_t cs, uint8_t address, const Probe &p) {
  probeSpi.beginTransaction(SPISettings(p.hz, MSBFIRST, p.mode));
  deselect(p.driven);
  digitalWrite(cs, LOW); pinMode(cs, OUTPUT); delayMicroseconds(2);
  probeSpi.transfer(address | 0x80);
  const uint8_t value = probeSpi.transfer(0);
  deselect(p.driven); probeSpi.endTransaction(); delayMicroseconds(5);
  return value;
}
void hexByte(uint8_t value) {
  if (value < 16) Serial.print('0');
  Serial.print(value, HEX);
}
bool validId(uint8_t cs, uint8_t id) {
  return cs == CS_IMU ? id == LSM6DSV32X_ID :
      (id == BMP5_CHIP_ID_PRIM || id == BMP5_CHIP_ID_SEC);
}
void snapshot(const Probe &p, uint8_t cs) {
  Serial.print(cs == CS_BARO ? "BARO best raw snapshot: " : "IMU best raw snapshot: ");
  const uint8_t baroRegs[] = {BMP5_REG_CHIP_ID, BMP5_REG_REV_ID, BMP5_REG_CHIP_STATUS,
      BMP5_REG_STATUS, BMP5_REG_OSR_CONFIG, BMP5_REG_ODR_CONFIG};
  const uint8_t imuRegs[] = {LSM6DSV32X_WHO_AM_I, LSM6DSV32X_CTRL1, LSM6DSV32X_CTRL2,
      LSM6DSV32X_CTRL3, LSM6DSV32X_STATUS_REG};
  const auto *regs = cs == CS_BARO ? baroRegs : imuRegs;
  const unsigned count = cs == CS_BARO ? sizeof(baroRegs) : sizeof(imuRegs);
  for (unsigned i = 0; i < count; ++i) {
    Serial.print("reg[0x"); hexByte(regs[i]); Serial.print("]=0x");
    hexByte(readRegister(cs, regs[i], p)); Serial.print(' ');
  }
  Serial.println();
  if (cs == CS_BARO) {
    const uint8_t status = readRegister(cs, BMP5_REG_STATUS, p);
    Serial.printf("BARO STATUS=0x%02X NVM_RDY=%u NVM_ERR=%u NVM_CMD_ERR=%u (meaningful only with stable valid ID)\n",
        status, !!(status & BMP5_INT_NVM_RDY), !!(status & BMP5_INT_NVM_ERR), !!(status & BMP5_INT_NVM_CMD_ERR));
    const auto odr = readRegister(cs, BMP5_REG_ODR_CONFIG, p);
    Serial.printf("BARO ODR_CONFIG=0x%02X pwr_mode=%u (0=standby, 1=normal, 2=forced, 3=continuous). NVM_RDY=0 during measurement is not proof of damaged NVM.\n",
        odr, odr & BMP5_POWERMODE_MSK);
  }
  // Only MCU weak pulls are changed; sensor registers are read, never written.
  Serial.print("ID with MISO pulls none/up/down (3 reads each): ");
  for (unsigned pull = 0; pull < 3; ++pull) {
    gpio_set_pulls(MISO_SENSOR, pull == 1, pull == 2); delayMicroseconds(50);
    for (unsigned i = 0; i < 3; ++i) {
      hexByte(readRegister(cs, cs == CS_BARO ? BMP5_REG_CHIP_ID : LSM6DSV32X_WHO_AM_I, p)); Serial.print(' ');
    }
    Serial.print(" / ");
  }
  gpio_disable_pulls(MISO_SENSOR); Serial.println();
}
const char *baroErrorName(int code) {
  switch (code) {
    case BMP5_OK: return "OK (SPI transfer completion alone does not prove a device answered)";
    case BMP5_E_NULL_PTR: return "NULL_PTR";
    case BMP5_E_COM_FAIL: return "COMMUNICATION_FAILURE";
    case BMP5_E_DEV_NOT_FOUND: return "DEVICE_NOT_FOUND";
    case BMP5_E_INVALID_CHIP_ID: return "INVALID_CHIP_ID";
    case BMP5_E_NVM_NOT_READY: return "NVM_NOT_READY (STATUS not ready or NVM error; invalid bus data can also cause this)";
    case BMP5_E_POR_SOFTRESET: return "POR_SOFTRESET_FAILURE";
    case BMP5_E_INVALID_POWERMODE: return "INVALID_POWER_MODE";
    default: return "See BMP5 SensorAPI error code";
  }
}
void driverStatus() {
  Serial.printf("Driver IMU: %s ID=0x%02X error=%ld | BARO: %s ID=0x%02X last_probe_ID=0x%02X error=%d\n",
      device.imuOk() ? "READY" : "UNAVAILABLE", device.imuWhoAmI(), long(device.lastImuError()),
      device.baroOk() ? "READY" : "UNAVAILABLE", device.baroChipId(), device.baroRawChipId(), int(device.lastBaroError()));
  Serial.print("BARO error meaning: "); Serial.println(baroErrorName(device.lastBaroError()));
  Serial.printf("Driver BARO selected mode=%u Hz=%lu\n", device.baroSpiMode() == SPI_MODE3 ? 3 : 0,
      (unsigned long)device.baroSpiClockHz());
  Serial.println("Driver BARO ID can remain 0 if init fails before chip_id is stored; compare RAW IDs above.");
  Serial.println("IMU error=0 with a wrong ID means the SPI callback completed, not that identification succeeded.");
}
void initDriver() {
  streaming = false;
  Serial.println("--- Normal Sensors::begin(): volatile sensor setup / IMU software reset; no flash or sensor NVM writes ---");
  device.begin(); initialized = true;
  imuSeen = baroSeen = false; imuFresh = baroFresh = 0;
  lastRead = lastReport = millis();
  driverStatus();
}
void baroChecks() {
  streaming = false;
  if (!device.baroOk()) {
    Serial.println("BARO checks skipped: run i first; barometer must be READY.");
    return;
  }
  Serial.println("=== BARO read-only checks: 20 rounds, interleaved speeds, production CS=PULLUP ===");
  Serial.println("Raw six-byte bursts use Bosch conversion P=raw/64 Pa. Reads are not synchronized to data-ready; repeats/changes alone are not errors.");
  Probe p{100000, device.baroSpiMode(), false, 0};
  const uint8_t baselineOsr = readRegister(CS_BARO, BMP5_REG_OSR_CONFIG, p);
  const uint8_t baselineOdr = readRegister(CS_BARO, BMP5_REG_ODR_CONFIG, p);
  const uint32_t speeds[] = {100000, 500000, 1000000};
  unsigned badIds[3] = {}, configDiffs[3] = {};
  float minimum[3] = {}, maximum[3] = {};
  Serial.printf("Baseline OSR=0x%02X ODR=0x%02X (comparison baseline, not proof of correct configuration)\n", baselineOsr, baselineOdr);
  for (unsigned round = 0; round < 20; ++round) {
    for (unsigned step = 0; step < 3; ++step) {
      const unsigned index = round % 2 ? 2-step : step;
      p.hz = speeds[index];
      const uint8_t id = readRegister(CS_BARO, BMP5_REG_CHIP_ID, p);
      const uint8_t osr = readRegister(CS_BARO, BMP5_REG_OSR_CONFIG, p);
      const uint8_t odr = readRegister(CS_BARO, BMP5_REG_ODR_CONFIG, p);
      uint8_t bytes[6];
      probeSpi.beginTransaction(SPISettings(p.hz, MSBFIRST, p.mode));
      deselect(false);
      digitalWrite(CS_BARO, LOW); pinMode(CS_BARO, OUTPUT); delayMicroseconds(2);
      probeSpi.transfer(BMP5_REG_TEMP_DATA_XLSB | 0x80);
      for (auto &byte : bytes) byte = probeSpi.transfer(0);
      deselect(false); probeSpi.endTransaction();
      const uint32_t rawP = uint32_t(bytes[3]) | (uint32_t(bytes[4]) << 8) | (uint32_t(bytes[5]) << 16);
      const float pressure = rawP / 64.0f;
      const bool idOk = validId(CS_BARO, id);
      const bool configOk = osr == baselineOsr && odr == baselineOdr;
      badIds[index] += !idOk; configDiffs[index] += !configOk;
      if (!round || pressure < minimum[index]) minimum[index] = pressure;
      if (!round || pressure > maximum[index]) maximum[index] = pressure;
      Serial.printf("BARO CHECK round=%u Hz=%lu ID=%02X OSR=%02X ODR=%02X bytes=", round+1, (unsigned long)p.hz, id, osr, odr);
      for (auto byte : bytes) { hexByte(byte); Serial.print(' '); }
      Serial.printf("rawP=%lu P=%.3fPa ID_OK=%u CFG_SAME=%u\n", (unsigned long)rawP, pressure, idOk, configOk);
    }
    delay(25);
  }
  for (unsigned index = 0; index < 3; ++index)
    Serial.printf("BARO SUMMARY Hz=%lu bad_ID=%u/20 config_diff=%u/20 P_min=%.3f P_max=%.3f span=%.3fPa\n",
        (unsigned long)speeds[index], badIds[index], configDiffs[index], minimum[index], maximum[index], maximum[index]-minimum[index]);
  Serial.println("Ranges include all raw reads, even invalid IDs/configurations. Stable wrong pressure does not distinguish sensor damage, obstruction or electrical faults.");
  imuSeen = baroSeen = false; imuFresh = baroFresh = 0;
  lastRead = lastReport = millis();
}
void readBurst(uint8_t cs, uint8_t address, uint8_t *out, unsigned count, const Probe &p) {
  probeSpi.beginTransaction(SPISettings(p.hz, MSBFIRST, p.mode));
  deselect(p.driven);
  digitalWrite(cs, LOW); pinMode(cs, OUTPUT);
  probeSpi.transfer(address | 0x80);
  for (unsigned i = 0; i < count; ++i) out[i] = probeSpi.transfer(0);
  deselect(p.driven); probeSpi.endTransaction();
}
// Read-only clock sweep with production chip-select handling (release to pull-up).
// Each speed: ID reads, config bursts compared with a 1 MHz reference, and
// plausibility of live data bursts (|a| near 1 g at rest, pressure in range).
void speedSweep() {
  streaming = false;
  if (!device.imuOk() || !device.baroOk()) { Serial.println("SWEEP skipped: run i first; IMU and BARO must be READY."); return; }
  constexpr unsigned Rounds = 2000;
  const Probe imuRef{1000000, SPI_MODE3, false, 0}, baroRef{1000000, device.baroSpiMode(), false, 0};
  uint8_t imuCfg[10], baroCfg[2];
  readBurst(CS_IMU, LSM6DSV32X_CTRL1, imuCfg, sizeof(imuCfg), imuRef);
  readBurst(CS_BARO, BMP5_REG_OSR_CONFIG, baroCfg, sizeof(baroCfg), baroRef);
  Serial.printf("=== SPI SPEED SWEEP: %u rounds/speed, CS=PULLUP, IMU mode3, BARO mode%u ===\n", Rounds, device.baroSpiMode() == SPI_MODE3 ? 3 : 0);
  const uint32_t speeds[] = {1000000, 2000000, 4000000, 6000000, 8000000, 10000000, 12500000,
                             15000000, 18750000, 25000000, 37500000};
  for (const auto hz : speeds) {
    Probe imu{hz, SPI_MODE3, false, 0}, baro{hz, device.baroSpiMode(), false, 0};
    probeSpi.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE3));
    const uint32_t actual = spi_get_baudrate(SENSOR_SPI_PORT);
    probeSpi.endTransaction();
    unsigned imuId = 0, imuConfig = 0, imuData = 0, baroId = 0, baroConfig = 0, baroData = 0;
    uint32_t imuBurstUs = 0;
    for (unsigned round = 0; round < Rounds; ++round) {
      imuId += readRegister(CS_IMU, LSM6DSV32X_WHO_AM_I, imu) != LSM6DSV32X_ID;
      uint8_t cfg[10]; readBurst(CS_IMU, LSM6DSV32X_CTRL1, cfg, sizeof(cfg), imu);
      imuConfig += memcmp(cfg, imuCfg, sizeof(cfg)) != 0;
      uint8_t data[16];
      const auto t0 = micros();
      readBurst(CS_IMU, LSM6DSV32X_STATUS_REG, data, sizeof(data), imu);
      imuBurstUs += micros() - t0;
      float norm = 0;
      for (unsigned axis = 0; axis < 3; ++axis) {
        const float g = lsm6dsv32x_from_fs32_to_mg(int16_t(data[10+2*axis] | (data[11+2*axis] << 8))) / 1000.0f;
        norm += g*g;
      }
      imuData += norm < 0.7f*0.7f || norm > 1.3f*1.3f;
      baroId += !validId(CS_BARO, readRegister(CS_BARO, BMP5_REG_CHIP_ID, baro));
      uint8_t bcfg[2]; readBurst(CS_BARO, BMP5_REG_OSR_CONFIG, bcfg, sizeof(bcfg), baro);
      baroConfig += memcmp(bcfg, baroCfg, sizeof(bcfg)) != 0;
      uint8_t bytes[6]; readBurst(CS_BARO, BMP5_REG_TEMP_DATA_XLSB, bytes, sizeof(bytes), baro);
      const float pressure = (uint32_t(bytes[3]) | (uint32_t(bytes[4]) << 8) | (uint32_t(bytes[5]) << 16)) / 64.0f;
      baroData += pressure < 30000.0f || pressure > 110000.0f;
    }
    const bool pass = !(imuId || imuConfig || imuData || baroId || baroConfig || baroData);
    Serial.printf("SWEEP req=%lu actual=%lu IMU bad id/cfg/data=%u/%u/%u BARO bad id/cfg/data=%u/%u/%u imu16_us=%.1f %s\n",
        (unsigned long)hz, (unsigned long)actual, imuId, imuConfig, imuData, baroId, baroConfig, baroData,
        double(imuBurstUs)/Rounds, pass ? "PASS" : "FAIL");
  }
  Serial.println("=== END SWEEP (read-only; IMU data check assumes the board is at rest) ===");
  imuSeen = baroSeen = false; imuFresh = baroFresh = 0;
  lastRead = lastReport = millis();
}
// True IMU output rate: 10 s tight polling with the production driver (no other work),
// plus the factory oscillator trim INTERNAL_FREQ_FINE (0x4F, signed).
void odrCheck() {
  streaming = false;
  if (!device.imuOk()) { Serial.println("ODR check skipped: run i first; IMU must be READY."); return; }
  const Probe p{1000000, SPI_MODE3, false, 0};
  const auto fine = int8_t(readRegister(CS_IMU, LSM6DSV32X_INTERNAL_FREQ, p));
  unsigned frames = 0; uint32_t maxGap = 0, last = 0;
  const uint32_t start = micros();
  while (uint32_t(micros() - start) < 10000000u) {
    device.read();
    if (device.sample().imuFresh) {
      const uint32_t now = micros();
      if (frames && now - last > maxGap) maxGap = now - last;
      last = now; ++frames;
    }
  }
  Serial.printf("ODR CHECK frames=%u rate=%.2f Hz max_gap_us=%lu FREQ_FINE=%d (~%.2f%% per datasheet 0.13%%/LSB)\n",
      frames, frames / 10.0, (unsigned long)maxGap, fine, fine * 0.13);
  imuSeen = baroSeen = false; imuFresh = baroFresh = 0;
  lastRead = lastReport = millis();
}
void diagnostics() {
  streaming = false;
  Serial.println("\n=== SENSOR_TEST: hardware SPI1 diagnostics ===");
  Serial.printf("MISO=GP%u MOSI=GP%u CLK=GP%u CS_IMU=GP%u CS_BARO=GP%u\n",
      MISO_SENSOR, MOSI_SENSOR, SCK_SENSOR, CS_IMU, CS_BARO);
  Serial.println("Expected: IMU LSM6DSV32X=0x70; BARO BMP5=0x50/0x51 (this board previously 0x50).");
  Serial.println("Raw scan is read-only. Outputs low; flash deselected; Bluetooth/flight runtime inactive.");
  Serial.println("CS=HIGH drives unselected pins high; CS=PULLUP matches production driver. Five independent ID reads per row.");
  Probe bestBaro, bestImu;
  deselect(true); probeSpi.begin(false); delay(10);
  for (unsigned pass = 0; pass < 2; ++pass) {
  Serial.println(pass == 0 ? "CS ORDER forward: HIGH then PULLUP" : "CS ORDER reverse: PULLUP then HIGH (no reset between passes)");
  for (unsigned selection = 0; selection < 2; ++selection) {
    const bool driven = pass == 0 ? selection == 0 : selection == 1;
    for (uint32_t hz : {100000u, 500000u, 1000000u}) {
      for (uint8_t mode : {uint8_t(SPI_MODE0), uint8_t(SPI_MODE3)}) {
        for (uint8_t cs : {CS_BARO, CS_IMU}) {
          Probe p{hz, mode, driven, 0};
          const auto reg = cs == CS_BARO ? BMP5_REG_CHIP_ID : LSM6DSV32X_WHO_AM_I;
          // BMP5 first read may only switch the host interface to SPI. Show it,
          // then count subsequent independent transactions; no extra dummy byte.
          const uint8_t first = readRegister(cs, reg, p);
          Serial.printf("%s CS=%s mode=%u Hz=%lu first=%02X IDs: ", cs == CS_BARO ? "BARO" : "IMU ",
              driven ? "HIGH" : "PULLUP", mode == SPI_MODE3 ? 3 : 0, (unsigned long)hz, first);
          for (unsigned repeat = 0; repeat < 5; ++repeat) {
            const uint8_t id = readRegister(cs, reg, p);
            hexByte(id); Serial.print(' '); if (validId(cs, id)) ++p.matches;
            delay(2);
          }
          Serial.printf("match=%u/5\n", p.matches);
          auto &best = cs == CS_BARO ? bestBaro : bestImu;
          if (p.matches > best.matches) best = p;
        }
      }
    }
  }
  }
  snapshot(bestBaro, CS_BARO); snapshot(bestImu, CS_IMU);
  Serial.printf("Best valid IDs per setting: BARO=%u/5 IMU=%u/5\n", bestBaro.matches, bestImu.matches);
  Serial.println("00/FF alone cannot identify the fault. Pull-dependent IDs suggest undriven MISO/selection issues.");
  Serial.println("Mode/speed/CS-dependent results suggest timing, wiring or selection sensitivity; a valid ID alone is not a sensor self-test.");
  Serial.println("If both fail, inspect shared supply/SPI wiring and a possible MISO bus conflict; if only one fails, inspect its CS/solder joints/supply.");
  initDriver();
  Serial.println("=== END: r=repeat both scan orders+init | i=retry driver init | s=status | v=live | b=BARO speed/config/raw checks | h=help (then Enter) ===");
}
void liveReport() {
  const auto &s = device.sample();
  Serial.printf("LIVE IMU ready=%u valid=%u fresh_count=%u error=%ld age_ms=", device.imuOk(), s.imuValid, imuFresh, long(device.lastImuError()));
  if (imuSeen) Serial.print(millis()-imuAt); else Serial.print("never");
  if (s.imuValid) Serial.printf(" a[m/s2]=%.3f,%.3f,%.3f w[rad/s]=%.4f,%.4f,%.4f T=%.2fC",
      s.accelMps2[0], s.accelMps2[1], s.accelMps2[2], s.gyroRadps[0], s.gyroRadps[1], s.gyroRadps[2], s.imuTempC);
  Serial.println();
  Serial.printf("LIVE BARO ready=%u valid=%u fresh_count=%u error=%d age_ms=", device.baroOk(), s.baroValid, baroFresh, int(device.lastBaroError()));
  if (baroSeen) Serial.print(millis()-baroAt); else Serial.print("never");
  if (s.baroValid) Serial.printf(" P=%.3fPa T=%.3fC", s.pressurePa, s.baroTempC);
  Serial.println(); imuFresh = baroFresh = 0;
}
}

void setup() { inactiveOutputs(); deselect(true); Serial.begin(115200); }
void loop() {
  if (!Serial) { connected = false; streaming = false; return; }
  if (!connected) { connected = true; diagnostics(); }
  if (Serial.available()) {
    switch (Serial.read()) {
      case 'r': diagnostics(); break;
      case 'i': initDriver(); break;
      case 's': driverStatus(); break;
      case 'b': baroChecks(); break;
      case 'f': speedSweep(); break;
      case 'o': odrCheck(); break;
      case 'v': streaming = !streaming; imuFresh = baroFresh = 0; lastReport = millis();
        Serial.println(streaming ? "Live samples ON (2 reports/s)" : "Live samples OFF"); break;
      case 'h': Serial.println("r both scan orders+init | i init | s status | v live on/off | b BARO speed/config/raw checks (stops live) | f SPI speed sweep 1-37.5 MHz | o IMU output-rate check; use LF/CRLF. No arming/output commands."); break;
      default: break;
    }
  }
  const uint32_t now = millis();
  if (initialized && uint32_t(now-lastRead) >= 5) {
    lastRead = now; device.read(); const auto &s = device.sample();
    if (s.imuFresh) { imuSeen = true; imuAt = now; ++imuFresh; }
    if (s.baroFresh) { baroSeen = true; baroAt = now; ++baroFresh; }
  }
  if (streaming && uint32_t(now-lastReport) >= 500) { lastReport = now; liveReport(); }
}
#endif
