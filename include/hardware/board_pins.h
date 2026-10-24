#pragma once

#include <Arduino.h>
#include "hardware/spi.h"

// Board pin map for the Waveshare RP2350 Tiny port.
// Values are RP GPIO numbers, not physical header/schematic pin numbers.

// ===================== SPI flash / memory =====================
#define FLASH_SPI_PORT spi0

constexpr uint8_t PIN_MEM_MISO = 0;  // MISO_memo
constexpr uint8_t PIN_MEM_CS1  = 1;  // CS_memo1
constexpr uint8_t PIN_MEM_SCK  = 2;  // CLK_memo
constexpr uint8_t PIN_MEM_MOSI = 3;  // MOSI_memo
// 1 MHz cannot sustain verified 200 Hz recording; keep board policy explicit.
constexpr uint32_t FLASH_CLOCK_HZ = 4000000;

// Compatibility with recovery_system_avionics names.
constexpr uint8_t MISO_FLASH = PIN_MEM_MISO;
constexpr uint8_t CS_FLASH1  = PIN_MEM_CS1;
constexpr uint8_t SCK_FLASH  = PIN_MEM_SCK;
constexpr uint8_t MOSI_FLASH = PIN_MEM_MOSI;

// ===================== Bluetooth =====================
// Directions are from the MCU: module TXD -> GP5; module RXD <- GP4.
constexpr uint8_t RX_BLE   = 5;
constexpr uint8_t TX_BLE   = 4;
constexpr uint8_t PWRC_BLE = 17;

// ===================== Pyro outputs =====================
constexpr uint8_t PYRO_1      = 9;
constexpr uint8_t PYRO_2      = 13;
constexpr uint8_t PYRO_3      = 6;
constexpr uint8_t PYRO_4      = 7;
constexpr uint8_t PYRO_ENABLE = 8;

// ===================== Continuity measurement =====================
constexpr uint8_t CONTINUITY_SELECT_0 = 11;
constexpr uint8_t CONTINUITY_SELECT_1 = 10;
constexpr uint8_t CONTINUITY_SENSE    = 28;  // GP28 / ADC2
constexpr uint8_t CONTINUITY_SENSE_ADC = 2;

// ===================== Sensors SPI =====================
#define SENSOR_SPI_PORT spi1

constexpr uint8_t SCK_SENSOR  = 14;
constexpr uint8_t MOSI_SENSOR = 15;
constexpr uint8_t MISO_SENSOR = 12;
constexpr uint8_t CS_IMU      = 27;  // GP27 / ADC1
constexpr uint8_t CS_BARO     = 26;  // GP26 / ADC0

constexpr uint8_t CS_LSM6DSV32X = CS_IMU;
constexpr uint8_t CS_BMP581     = CS_BARO;

// Compatibility with recovery_system_avionics sensor names.
constexpr uint8_t CS_LSM6    = CS_IMU;
constexpr uint8_t CS_BMP     = CS_BARO;

// ===================== Analog telemetry =====================
constexpr uint8_t BATT_SENSE = 29;  // GP29 / ADC3
constexpr uint8_t BATT_SENSE_ADC = 3;

// ===================== Telemetry / external I2C header =====================
constexpr uint8_t TX_TELEMETRY  = 18;  // TX_TELEMETRY / SDA
constexpr uint8_t RX_TELEMETRY  = 19;  // RX_TELEMETRY / SCL
constexpr uint8_t SDA_TELEMETRY = TX_TELEMETRY;
constexpr uint8_t SCL_TELEMETRY = RX_TELEMETRY;

// ===================== Status LED =====================
constexpr uint8_t RGB_LED = 16;

// ===================== Servo PWM outputs =====================
constexpr uint8_t SERVO_PWM_1 = 21;
constexpr uint8_t SERVO_PWM_2 = 20;
constexpr uint8_t SERVO_PWM_3 = 23;
constexpr uint8_t SERVO_PWM_4 = 22;

// Primary parachute servo alias used by the legacy ServoController.
constexpr uint8_t SERVO_PIN = SERVO_PWM_1;
