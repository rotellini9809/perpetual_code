#include "diagnostics/BuildMode.h"
#ifdef FLASH_DIRECTORY_TEST
#include "storage/FlightDirectory.h"
#include "storage/FlashLoggerAdapter.h"
#include "hardware/board_pins.h"

namespace {
SPIClassRP2040 directorySpi(FLASH_SPI_PORT, MISO_FLASH, CS_FLASH1, SCK_FLASH, MOSI_FLASH);
FlashLogger directoryFlash(directorySpi, CS_FLASH1);
storage::FlashLoggerAdapter directoryDevice(directoryFlash);
constexpr uint32_t BackupBegin = 0x5000;
constexpr uint32_t BackupBytes = 65536;
uint8_t directoryBackup[BackupBytes];
bool backupAtRisk = false;
bool connected = false;
bool verifyBytes(uint32_t address, const uint8_t *expected, uint32_t length) {
  uint8_t bytes[256];
  for (uint32_t offset = 0; offset < length; offset += sizeof(bytes))
    if (!directoryFlash.read(address + offset, bytes, sizeof(bytes)) ||
        memcmp(bytes, expected + offset, sizeof(bytes))) return false;
  return true;
}
void testDirectory() {
  if (backupAtRisk) { Serial.println("Blocked: previous restore failed; preserve RAM, do not reboot."); return; }
  if (!directoryFlash.initialize(FLASH_CLOCK_HZ)) { Serial.println("Flash initialization failed; no changes."); return; }
  if (!directoryFlash.read(BackupBegin, directoryBackup, BackupBytes) ||
      !verifyBytes(BackupBegin, directoryBackup, BackupBytes)) {
    Serial.println("Backup failed; no changes."); return;
  }
  Serial.println("Backup of 0x005000..0x014FFF verified. Testing directory A/B only.");
  backupAtRisk = true;
  storage::FlightDirectory directory(directoryDevice);
  bool passed = directory.format();
  storage::FlightEntry entry;
  entry.sequence = 1; entry.start = 0x15000; entry.allocatedBytes = 4096;
  passed = passed && directory.update(entry);
  for (unsigned i = 1; passed && i <= 80; ++i) {
    entry.committedBytes = i * 10; passed = directory.update(entry);
  }
  entry.status = storage::FlightStatus::Completed;
  passed = passed && directory.update(entry) && directory.compact();
  storage::FlightDirectory reboot(directoryDevice);
  passed = passed && reboot.mount() && reboot.count() == 1 && reboot.highWater() == 1 &&
      reboot.entries()[0].committedBytes == 800 &&
      reboot.entries()[0].status == storage::FlightStatus::Completed && reboot.generation() >= 3;
  entry.status = storage::FlightStatus::Invalidated;
  passed = passed && reboot.update(entry) && reboot.compact();
  storage::FlightDirectory recovered(directoryDevice);
  passed = passed && recovered.mount() && recovered.count() == 0 && recovered.highWater() == 1;
  Serial.println(passed ? "Directory append/compaction/remount/invalidation: PASS" : "Directory test: FAIL");
  if (!passed) {
    Serial.print("Directory error: "); Serial.println(storage::FlightDirectory::errorName(directory.error()));
    Serial.print("Flash error: "); Serial.println(FlashLogger::errorName(directoryFlash.lastError()));
  }
  bool restored = true;
  for (uint32_t offset = 0; restored && offset < BackupBytes; offset += FlashLogger::SectorSize)
    restored = directoryFlash.eraseSector(BackupBegin + offset) &&
        directoryFlash.program(BackupBegin + offset, directoryBackup + offset, FlashLogger::SectorSize);
  restored = restored && verifyBytes(BackupBegin, directoryBackup, BackupBytes);
  backupAtRisk = !restored;
  Serial.println(restored ? "Original directory regions RESTORED and VERIFIED." : "RESTORE FAILED; do not reboot: backup is in RAM.");
  Serial.println(passed && restored ? "=== DIRECTORY TEST PASSED ===" : "=== DIRECTORY TEST FAILED ===");
}
}
void setup() {
  for (uint8_t pin : {PYRO_ENABLE, PYRO_1, PYRO_2, PYRO_3, PYRO_4,
                      SERVO_PWM_1, SERVO_PWM_2, SERVO_PWM_3, SERVO_PWM_4}) {
    digitalWrite(pin, LOW); pinMode(pin, OUTPUT);
  }
  digitalWrite(CS_IMU, HIGH); pinMode(CS_IMU, OUTPUT);
  digitalWrite(CS_BARO, HIGH); pinMode(CS_BARO, OUTPUT);
  Serial.begin(115200); delay(100);
}
void loop() {
  if (!Serial) { connected = false; return; }
  if (!connected) {
    connected = true; delay(300);
    Serial.println("\n=== FLASH_DIRECTORY_TEST ===");
    Serial.println("Send d + Enter: test A/B banks 0x005000..0x014FFF (64KiB), then restore RAM backup.");
    Serial.println("Power loss during test can destroy those regions. Calibration/settings/flight data are untouched.");
  }
  if (Serial.available()) {
    const int c = Serial.read(); if (c == 'd' || c == 'D') testDirectory();
  }
}

#endif // FLASH_DIRECTORY_TEST
