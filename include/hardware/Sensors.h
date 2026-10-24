#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <bmp5.h>
#include <lsm6dsv32x_reg.h>
#include <math.h>
#include "hardware/SensorDevice.h"

struct SensorSpiDevice {
  SPIClassRP2040 *bus = nullptr;
  uint8_t cs = 0xFF;
  uint32_t clockHz = 1000000;
  uint8_t mode = SPI_MODE0;
};

class Sensors : public SensorDevice {
public:
  bool begin() override;
  bool read() override;
  bool suspend(); // Stop only detected devices; wake uses begin() to restore the full profile.

  bool imuOk() const override { return imuReady; }
  bool baroOk() const override { return baroReady; }
  bool ok() const { return imuReady && baroReady; }

  uint8_t imuWhoAmI() const { return imuId; }
  uint8_t baroChipId() const { return baroId; }
  uint8_t baroRawChipId() const { return baroRawId; }
  uint8_t baroSpiMode() const { return baroSpi.mode; }
  uint32_t baroSpiClockHz() const { return baroSpi.clockHz; }
  int32_t lastImuError() const { return imuError; }
  int8_t lastBaroError() const { return baroError; }

  const SensorSample &sample() const override { return latest; }

private:
  bool beginImu();
  bool beginBaro();
  bool readImu();
  bool readBaro();
  bool probeImuSpiMode(uint8_t spiMode);
  bool probeBaroSpiSettings();
  bool isValidBaroChipId(uint8_t chipId) const;
  uint8_t readBaroChipIdRaw();
  void initChipSelects();

  SensorSpiDevice imuSpi;
  SensorSpiDevice baroSpi;
  bmp5_dev bmpDev = {};
  bmp5_osr_odr_press_config bmpConfig = {};
  stmdev_ctx_t imuCtx = {};

  bool imuReady = false;
  bool baroReady = false;
  uint8_t imuId = 0;
  uint8_t baroId = 0;
  uint8_t baroRawId = 0;
  int32_t imuError = 0;
  int8_t baroError = 0;
  SensorSample latest;
};
