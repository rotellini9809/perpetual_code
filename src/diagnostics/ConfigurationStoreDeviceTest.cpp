#include "diagnostics/BuildMode.h"
#ifdef CONFIG_STORE_TEST
#include "storage/ConfigurationStore.h"
#include "storage/FlashLoggerAdapter.h"
#include "hardware/board_pins.h"

namespace {
SPIClassRP2040 configSpi(FLASH_SPI_PORT, MISO_FLASH, CS_FLASH1, SCK_FLASH, MOSI_FLASH);
FlashLogger configFlash(configSpi, CS_FLASH1);
storage::FlashLoggerAdapter device(configFlash);
storage::ConfigurationStore store(device);
const recovery::ValidationLimits limits{1000, 2000, 1000, 60000}; // Test policy only.
uint8_t backup[2 * storage::FlashLayout::JournalBankBytes]; // The whole device journal (block 0).
uint8_t encodedBefore[storage::ConfigurationStore::MaxPayload];
uint8_t encodedAfter[storage::ConfigurationStore::MaxPayload];
recovery::Configuration fixture, loaded;
bool backupAtRisk = false;
bool connected = false;
bool verifyBytes(uint32_t address, const uint8_t *expected, uint32_t count) {
  uint8_t page[256];
  for (uint32_t offset = 0; offset < count; offset += sizeof(page))
    if (!configFlash.read(address + offset, page, sizeof(page)) || memcmp(page, expected + offset, sizeof(page))) return false;
  return true;
}
bool samePlan() {
  std::size_t before, after;
  return storage::ConfigurationStore::encode(fixture, encodedBefore, sizeof(encodedBefore), before) &&
      storage::ConfigurationStore::encode(loaded, encodedAfter, sizeof(encodedAfter), after) &&
      before == after && memcmp(encodedBefore, encodedAfter, before) == 0;
}
void runConfigTest() {
  if (backupAtRisk) { Serial.println("Blocked: previous restore failed; preserve RAM, do not reboot."); return; }
  if (!configFlash.initialize(FLASH_CLOCK_HZ)) { Serial.println("Flash initialization failed; no changes."); return; }
  constexpr uint32_t begin = storage::FlashLayout::JournalA;
  if (!configFlash.read(begin, backup, sizeof(backup)) || !verifyBytes(begin, backup, sizeof(backup))) {
    Serial.println("Backup failed; no changes."); return;
  }
  Serial.println("Backup of the device journal 0x000000..0x00FFFF verified. No outputs will move.");
  fixture = {}; fixture.ruleCount = 1; fixture.pyros[0].pulseMs = 100;
  auto &rule = fixture.rules[0]; rule.id = 42; strcpy(rule.name.data(), "Storage test only");
  rule.conditionCount = rule.assignmentCount = 1; rule.assignments[0].channel = 1;
  bool passed = !store.save(fixture, limits, false) && store.error() == storage::ConfigError::Armed;
  const auto validCount = fixture.ruleCount; fixture.ruleCount = 0;
  passed = passed && !store.save(fixture, limits, true) && store.error() == storage::ConfigError::InvalidConfiguration;
  fixture.ruleCount = validCount;
  passed = passed && verifyBytes(begin, backup, sizeof(backup));
  if (!passed) { Serial.println("Pre-erase validation checks FAILED; no test save attempted."); return; }
  Serial.println("Armed/invalid-plan rejection without writes: PASS");
  backupAtRisk = true;
  passed = store.save(fixture, limits, true) && store.load(loaded, limits) && samePlan();
  fixture.pyros[0].pulseMs = 200;
  passed = passed && store.save(fixture, limits, true) && store.load(loaded, limits) && samePlan();
  // Fresh store instance uses only persisted data, not an in-memory applied plan.
  static storage::ConfigurationStore reboot(device);
  passed = passed && reboot.load(loaded, limits) && samePlan();
  Serial.println(passed ? "Journal save/load/replacement/reload: PASS" : "Configuration test: FAIL");
  if (!passed) {
    Serial.print("Store result: "); Serial.println(storage::ConfigurationStore::errorName(store.error()));
    Serial.print("Flash result: "); Serial.println(FlashLogger::errorName(configFlash.lastError()));
  }
  bool restored = true;
  restored = configFlash.eraseRange(begin, sizeof(backup)) && configFlash.program(begin, backup, sizeof(backup));
  restored = restored && verifyBytes(begin, backup, sizeof(backup));
  backupAtRisk = !restored;
  Serial.println(restored ? "Original device journal RESTORED and VERIFIED." : "RESTORE FAILED; preserve RAM, do not reboot.");
  Serial.println(passed && restored ? "=== CONFIGURATION STORE TEST PASSED ===" : "=== CONFIGURATION STORE TEST FAILED ===");
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
    Serial.println("\n=== CONFIG_STORE_TEST ===");
    Serial.println("Send c + Enter: test the device journal 0x000000..0x00FFFF (rules/settings), then restore RAM backup.");
    Serial.println("Power loss during test can destroy settings and calibration. Directory/flight data are untouched.");
  }
  if (Serial.available()) { const int c = Serial.read(); if (c == 'c' || c == 'C') runConfigTest(); }
}
#endif
