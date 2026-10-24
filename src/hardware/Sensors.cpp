#include "hardware/Sensors.h"

#include "hardware/board_pins.h"

namespace {
constexpr uint32_t BMP581_SPI_HZ = 1000000;
constexpr uint32_t BMP581_PROBE_SPI_HZ = 100000;
constexpr uint32_t LSM6DSV32X_SPI_HZ = 1000000;
constexpr float STANDARD_GRAVITY = 9.80665f;
constexpr float DEG_TO_RAD_F = 0.017453292519943295f;

SPIClassRP2040 sensorSpi(SENSOR_SPI_PORT, MISO_SENSOR, CS_IMU, SCK_SENSOR, MOSI_SENSOR);

void deselectAllSensors() {
  pinMode(CS_LSM6DSV32X, INPUT_PULLUP);
  pinMode(CS_BMP581, INPUT_PULLUP);
}

void selectSensor(uint8_t cs) {
  deselectAllSensors();
  digitalWrite(cs, LOW);
  pinMode(cs, OUTPUT);
}
}

namespace {
BMP5_INTF_RET_TYPE bmp581Read(uint8_t reg, uint8_t *data, uint32_t len, void *intfPtr) {
  SensorSpiDevice *device = static_cast<SensorSpiDevice *>(intfPtr);
  if (device == nullptr || device->bus == nullptr || data == nullptr) {
    return BMP5_E_COM_FAIL;
  }

  device->bus->beginTransaction(SPISettings(device->clockHz, MSBFIRST, device->mode));
  selectSensor(device->cs);
  device->bus->transfer(reg);
  for (uint32_t i = 0; i < len; i++) {
    data[i] = device->bus->transfer(0x00);
  }
  deselectAllSensors();
  device->bus->endTransaction();

  return BMP5_INTF_RET_SUCCESS;
}

BMP5_INTF_RET_TYPE bmp581Write(uint8_t reg, const uint8_t *data, uint32_t len, void *intfPtr) {
  SensorSpiDevice *device = static_cast<SensorSpiDevice *>(intfPtr);
  if (device == nullptr || device->bus == nullptr || data == nullptr) {
    return BMP5_E_COM_FAIL;
  }

  device->bus->beginTransaction(SPISettings(device->clockHz, MSBFIRST, device->mode));
  selectSensor(device->cs);
  device->bus->transfer(reg);
  for (uint32_t i = 0; i < len; i++) {
    device->bus->transfer(data[i]);
  }
  deselectAllSensors();
  device->bus->endTransaction();

  return BMP5_INTF_RET_SUCCESS;
}

void bmp581DelayUs(uint32_t period, void *intfPtr) {
  (void)intfPtr;
  delayMicroseconds(period);
}

int32_t lsm6dsv32xRead(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
  SensorSpiDevice *device = static_cast<SensorSpiDevice *>(handle);
  if (device == nullptr || device->bus == nullptr || data == nullptr) {
    return -1;
  }

  device->bus->beginTransaction(SPISettings(device->clockHz, MSBFIRST, device->mode));
  selectSensor(device->cs);
  device->bus->transfer(reg | 0x80);
  for (uint16_t i = 0; i < len; i++) {
    data[i] = device->bus->transfer(0x00);
  }
  deselectAllSensors();
  device->bus->endTransaction();

  return 0;
}

int32_t lsm6dsv32xWrite(void *handle, uint8_t reg, const uint8_t *data, uint16_t len) {
  SensorSpiDevice *device = static_cast<SensorSpiDevice *>(handle);
  if (device == nullptr || device->bus == nullptr || data == nullptr) {
    return -1;
  }

  device->bus->beginTransaction(SPISettings(device->clockHz, MSBFIRST, device->mode));
  selectSensor(device->cs);
  device->bus->transfer(reg & 0x7F);
  for (uint16_t i = 0; i < len; i++) {
    device->bus->transfer(data[i]);
  }
  deselectAllSensors();
  device->bus->endTransaction();

  return 0;
}

void lsm6dsv32xDelayMs(uint32_t periodMs) {
  delay(periodMs);
}
}

bool Sensors::begin() {
  latest = {}; imuReady = baroReady = false;
  initChipSelects();
  sensorSpi.begin(false);
  delay(5);

  baroReady = beginBaro();
  imuReady = beginImu();

  return ok();
}

bool Sensors::read() {
  latest.imuFresh = latest.baroFresh = false;
  const bool imuReadOk = readImu();
  const bool baroReadOk = readBaro();
  return imuReadOk && baroReadOk;
}

bool Sensors::suspend() {
  bool success=true;
  // BRING-UP: the unsoldered/unavailable IMU has no valid driver context. It must
  // not prevent suspending a working barometer or be replaced by fake readiness.
  if (imuReady) {
    imuError=lsm6dsv32x_xl_data_rate_set(&imuCtx,LSM6DSV32X_ODR_OFF);
    const auto gyroError=lsm6dsv32x_gy_data_rate_set(&imuCtx,LSM6DSV32X_ODR_OFF);
    if (!imuError) imuError=gyroError;
    success=imuError==0;
  }
  if (baroReady) {
    baroError=bmp5_set_power_mode(BMP5_POWERMODE_DEEP_STANDBY,&bmpDev);
    success=success && baroError==BMP5_OK;
  }
  imuReady=baroReady=false; latest={};
  deselectAllSensors();
  if (success) sensorSpi.end();
  return success;
}

bool Sensors::beginBaro() {
  baroSpi.bus = &sensorSpi;
  baroSpi.cs = CS_BMP581;
  baroSpi.clockHz = BMP581_SPI_HZ;
  baroSpi.mode = SPI_MODE0;

  bmpDev = {};
  bmpConfig = {};

  bmpDev.intf = BMP5_SPI_INTF;
  bmpDev.intf_ptr = &baroSpi;
  bmpDev.read = bmp581Read;
  bmpDev.write = bmp581Write;
  bmpDev.delay_us = bmp581DelayUs;

  baroId = 0;
  if (!probeBaroSpiSettings()) {
    baroError = BMP5_E_DEV_NOT_FOUND;
    return false;
  }

  // MCU reset does not reset the powered barometer. It may still be measuring,
  // where NVM_RDY is low; bmp5_init() assumes the post-power-up standby state.
  // Identify first, then stop measurements before that check. The Bosch API
  // handles the standby transition delay; no NVM programming or retry loop.
  baroError = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &bmpDev);
  if (baroError != BMP5_OK) {
    return false;
  }

  baroError = bmp5_init(&bmpDev);
  if (baroError != BMP5_OK) {
    baroId = bmpDev.chip_id;
    return false;
  }

  baroId = bmpDev.chip_id;

  baroError = bmp5_get_osr_odr_press_config(&bmpConfig, &bmpDev);
  if (baroError != BMP5_OK) {
    return false;
  }

  bmpConfig.odr = BMP5_ODR_50_HZ;
  bmpConfig.press_en = BMP5_ENABLE;
  bmpConfig.osr_t = BMP5_OVERSAMPLING_4X;
  bmpConfig.osr_p = BMP5_OVERSAMPLING_4X;

  baroError = bmp5_set_osr_odr_press_config(&bmpConfig, &bmpDev);
  if (baroError != BMP5_OK) {
    return false;
  }

  struct bmp5_int_source_select interruptSources = {};
  interruptSources.drdy_en = BMP5_ENABLE;
  baroError = bmp5_int_source_select(&interruptSources, &bmpDev);
  if (baroError != BMP5_OK) return false;

  baroError = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &bmpDev);
  if (baroError != BMP5_OK) {
    return false;
  }

  return true;
}

bool Sensors::beginImu() {
  imuSpi.bus = &sensorSpi;
  imuSpi.cs = CS_LSM6DSV32X;
  imuSpi.clockHz = LSM6DSV32X_SPI_HZ;
  imuSpi.mode = SPI_MODE3;

  imuCtx = {};
  imuCtx.write_reg = lsm6dsv32xWrite;
  imuCtx.read_reg = lsm6dsv32xRead;
  imuCtx.mdelay = lsm6dsv32xDelayMs;
  imuCtx.handle = &imuSpi;

  if (!probeImuSpiMode(SPI_MODE3) && !probeImuSpiMode(SPI_MODE0)) {
    return false;
  }

  imuError = lsm6dsv32x_sw_reset(&imuCtx);
  if (imuError != 0) {
    return false;
  }

  delay(10);

  imuError = lsm6dsv32x_device_id_get(&imuCtx, &imuId);
  if (imuError != 0 || imuId != LSM6DSV32X_ID) {
    return false;
  }

  imuError = lsm6dsv32x_auto_increment_set(&imuCtx, PROPERTY_ENABLE);
  imuError += lsm6dsv32x_block_data_update_set(&imuCtx, PROPERTY_ENABLE);
  imuError += lsm6dsv32x_ui_i2c_i3c_mode_set(&imuCtx, LSM6DSV32X_I2C_I3C_DISABLE);
  imuError += lsm6dsv32x_spi_mode_set(&imuCtx, LSM6DSV32X_SPI_4_WIRE);
  imuError += lsm6dsv32x_xl_full_scale_set(&imuCtx, LSM6DSV32X_32g);
  imuError += lsm6dsv32x_gy_full_scale_set(&imuCtx, LSM6DSV32X_2000dps);
  imuError += lsm6dsv32x_xl_data_rate_set(&imuCtx, LSM6DSV32X_ODR_AT_120Hz);
  imuError += lsm6dsv32x_gy_data_rate_set(&imuCtx, LSM6DSV32X_ODR_AT_120Hz);

  return imuError == 0;
}

bool Sensors::readBaro() {
  if (!baroReady) {
    latest.baroValid = false;
    return false;
  }

  uint8_t status = 0;
  baroError = bmp5_get_interrupt_status(&status, &bmpDev);
  if (baroError != BMP5_OK) { latest.baroValid = false; return false; }
  if (!(status & BMP5_INT_ASSERTED_DRDY)) return true;
  bmp5_sensor_data sensorData = {};
  baroError = bmp5_get_sensor_data(&sensorData, &bmpConfig, &bmpDev);
  if (baroError != BMP5_OK) {
    latest.baroValid = false;
    return false;
  }

  latest.pressurePa = sensorData.pressure;
  latest.baroTempC = sensorData.temperature;
  latest.baroValid = true;
  latest.baroFresh = true;
  return true;
}

bool Sensors::readImu() {
  if (!imuReady) {
    latest.imuValid = false;
    return false;
  }

  lsm6dsv32x_data_ready_t ready = {};
  imuError = lsm6dsv32x_flag_data_ready_get(&imuCtx, &ready);
  if (imuError != 0) { latest.imuValid = false; return false; }
  if (!ready.drdy_xl || !ready.drdy_gy) return true;
  int16_t accelRaw[3] = {};
  int16_t gyroRaw[3] = {};
  int16_t tempRaw = 0;

  int32_t result = lsm6dsv32x_acceleration_raw_get(&imuCtx, accelRaw);
  if (result == 0) {
    result = lsm6dsv32x_angular_rate_raw_get(&imuCtx, gyroRaw);
  }
  if (result == 0) {
    result = lsm6dsv32x_temperature_raw_get(&imuCtx, &tempRaw);
  }

  imuError = result;
  if (result != 0) {
    latest.imuValid = false;
    return false;
  }

  for (uint8_t axis = 0; axis < 3; axis++) {
    const float accelMg = lsm6dsv32x_from_fs32_to_mg(accelRaw[axis]);
    const float gyroMdps = lsm6dsv32x_from_fs2000_to_mdps(gyroRaw[axis]);
    latest.accelMps2[axis] = accelMg * STANDARD_GRAVITY / 1000.0f;
    latest.gyroRadps[axis] = gyroMdps * DEG_TO_RAD_F / 1000.0f;
  }

  latest.imuTempC = lsm6dsv32x_from_lsb_to_celsius(tempRaw);
  latest.imuValid = true;
  latest.imuFresh = true;
  return true;
}

bool Sensors::probeImuSpiMode(uint8_t spiMode) {
  if (imuSpi.bus == nullptr) {
    return false;
  }

  imuSpi.mode = spiMode;

  uint8_t whoAmI = 0;
  imuError = lsm6dsv32x_device_id_get(&imuCtx, &whoAmI);
  imuId = whoAmI;
  return imuError == 0 && whoAmI == LSM6DSV32X_ID;
}

bool Sensors::probeBaroSpiSettings() {
  const uint8_t modes[] = {SPI_MODE0, SPI_MODE3};
  const uint32_t clocks[] = {BMP581_SPI_HZ, BMP581_PROBE_SPI_HZ};

  for (uint8_t clockIndex = 0; clockIndex < sizeof(clocks) / sizeof(clocks[0]); clockIndex++) {
    for (uint8_t modeIndex = 0; modeIndex < sizeof(modes) / sizeof(modes[0]); modeIndex++) {
      baroSpi.clockHz = clocks[clockIndex];
      baroSpi.mode = modes[modeIndex];
      delay(1);

      baroRawId = readBaroChipIdRaw();
      if (isValidBaroChipId(baroRawId)) {
        return true;
      }
    }
  }

  baroSpi.clockHz = BMP581_SPI_HZ;
  baroSpi.mode = SPI_MODE0;
  return false;
}

bool Sensors::isValidBaroChipId(uint8_t chipId) const {
  return chipId == BMP5_CHIP_ID_PRIM || chipId == BMP5_CHIP_ID_SEC;
}

uint8_t Sensors::readBaroChipIdRaw() {
  uint8_t chipId = 0;
  const BMP5_INTF_RET_TYPE result = bmp581Read(BMP5_REG_CHIP_ID | BMP5_SPI_RD_MASK, &chipId, 1, &baroSpi);
  if (result != BMP5_INTF_RET_SUCCESS) {
    return 0;
  }

  return chipId;
}

void Sensors::initChipSelects() {
  pinMode(CS_LSM6DSV32X, INPUT_PULLUP);
  pinMode(CS_BMP581, INPUT_PULLUP);
}
