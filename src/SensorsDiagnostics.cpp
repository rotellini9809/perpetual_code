#include "Sensors.h"
#include <board_pins.h>

namespace {
void hexByte(Print &out, uint8_t value) {
  out.print("0x");
  if (value < 0x10) out.print('0');
  out.print(value, HEX);
}

void printSettings(Print &out, const SensorSpiDevice &device) {
  out.print(" CS=GP");
  out.print(device.cs);
  out.print(" mode=");
  out.print(device.mode == SPI_MODE3 ? 3 : 0);
  out.print(" Hz=");
  out.print(device.clockHz);
}

// Read individual registers only; avoid interrupt/FIFO registers with read side effects.
uint8_t readRegister(SensorSpiDevice &device, uint8_t reg) {
  pinMode(CS_IMU, INPUT_PULLUP);
  pinMode(CS_BARO, INPUT_PULLUP);
  device.bus->beginTransaction(SPISettings(device.clockHz, MSBFIRST, device.mode));
  digitalWrite(device.cs, LOW);
  pinMode(device.cs, OUTPUT);
  device.bus->transfer(reg | 0x80);
  const uint8_t value = device.bus->transfer(0);
  pinMode(device.cs, INPUT_PULLUP);
  device.bus->endTransaction();
  return value;
}

void sweepIdentity(Print &out, const char *name, SensorSpiDevice device,
                   uint8_t reg, uint8_t expected) {
  const uint8_t modes[] = {SPI_MODE0, SPI_MODE3};
  const uint32_t clocks[] = {1000000, 100000};
  out.print(name);
  out.print(" identity sweep: register=");
  hexByte(out, reg);
  out.print(" expected=");
  hexByte(out, expected);
  out.println();
  for (uint32_t hz : clocks) {
    for (uint8_t mode : modes) {
      device.clockHz = hz;
      device.mode = mode;
      uint8_t ids[3];
      for (uint8_t &id : ids) {
        id = readRegister(device, reg);
        delay(1);
      }
      printSettings(out, device);
      out.print(" IDs=");
      for (uint8_t id : ids) {
        hexByte(out, id);
        out.print(' ');
      }
      const bool stable = ids[0] == ids[1] && ids[1] == ids[2];
      out.println(!stable ? "UNSTABLE" : ids[0] == expected ? "MATCH" : "MISMATCH");
    }
  }
}
}

void Sensors::traceStep(const char *step, int32_t result) {
  if (!debug) return;
  debug->print("[sensor init] ");
  debug->print(step);
  debug->print(" result=");
  debug->println(result);
  if (result == BMP5_E_NVM_NOT_READY && step[0] == 'B') {
    debug->println("  BMP5_E_NVM_NOT_READY: NVM ready/error status check failed; raw SPI data may also be invalid.");
  }
}

void Sensors::traceProbe(const char *name, const SensorSpiDevice &device, uint8_t id) {
  if (!debug) return;
  debug->print("[sensor probe] ");
  debug->print(name);
  printSettings(*debug, device);
  debug->print(" ID=");
  hexByte(*debug, id);
  debug->println();
}

void Sensors::troubleshoot(Print &out) {
  out.println("\n=== SENSOR DIAGNOSTICS BEGIN ===");
  if (!imuSpi.bus || !baroSpi.bus) {
    out.println("Call Sensors::begin() first.");
    out.println("=== SENSOR DIAGNOSTICS END ===");
    return;
  }
  out.print("SPI1 SCK=GP"); out.print(SCK_SENSOR);
  out.print(" MOSI=GP"); out.print(MOSI_SENSOR);
  out.print(" MISO=GP"); out.println(MISO_SENSOR);
  initChipSelects();
  delayMicroseconds(10);
  out.print("Inactive CS levels (expected 1,1): IMU=");
  out.print(digitalRead(CS_IMU));
  out.print(" BARO="); out.println(digitalRead(CS_BARO));
  out.print("Initialization: IMU="); out.print(imuReady ? "PASS" : "FAIL");
  out.print(" error="); out.print(imuError);
  out.print(" BARO="); out.print(baroReady ? "PASS" : "FAIL");
  out.print(" error="); out.println(baroError);

  // Snapshot the failing configuration before the optional mode/speed sweep.
  out.print("BARO current settings:"); printSettings(out, baroSpi); out.println();
  const uint8_t regs[] = {BMP5_REG_CHIP_ID, BMP5_REG_REV_ID, BMP5_REG_CHIP_STATUS,
                          BMP5_REG_STATUS, BMP5_REG_OSR_CONFIG, BMP5_REG_ODR_CONFIG};
  for (uint8_t reg : regs) {
    const uint8_t value = readRegister(baroSpi, reg);
    out.print("  register "); hexByte(out, reg);
    out.print(" = "); hexByte(out, value);
    if (reg == BMP5_REG_STATUS) {
      out.print(" NVM_RDY="); out.print(bool(value & BMP5_INT_NVM_RDY));
      out.print(" NVM_ERR="); out.print(bool(value & BMP5_INT_NVM_ERR));
      out.print(" NVM_CMD_ERR="); out.print(bool(value & BMP5_INT_NVM_CMD_ERR));
    }
    out.println();
  }
  sweepIdentity(out, "LSM6DSV32XTR", imuSpi, LSM6DSV32X_WHO_AM_I, LSM6DSV32X_ID);
  sweepIdentity(out, "BMP581", baroSpi, BMP5_REG_CHIP_ID, BMP5_CHIP_ID_PRIM);
  out.println("All 0x00/0xFF: check power, ground, MISO and CS wiring; not proof of a specific fault.");
  out.println("Changing IDs: inspect signal integrity, soldering, CS isolation and SPI timing.");
  out.println("Only 100 kHz matches: inspect clock/signal integrity. Sweep does not change driver settings.");
  out.println("Stable expected IDs but init fails: inspect the init stage and BARO status above.");
  out.println("Status bits are meaningful only when SPI reads are trustworthy.");
  out.println("=== SENSOR DIAGNOSTICS END ===\n");
}
