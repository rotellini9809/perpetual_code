#include "diagnostics/BuildMode.h"
#include "storage/FlashLogger.h"
#ifdef FLASH_TEST
#include "hardware/board_pins.h"

namespace {
SPIClassRP2040 flashSpi(FLASH_SPI_PORT, MISO_FLASH, CS_FLASH1, SCK_FLASH, MOSI_FLASH);
FlashLogger logger(flashSpi, CS_FLASH1);

void hexByte(uint8_t value) {
  if (value < 16) Serial.print('0');
  Serial.print(value, HEX);
}
void printBytes(const uint8_t *bytes, std::size_t count) {
  for (std::size_t i = 0; i < count; ++i) { hexByte(bytes[i]); Serial.print(' '); }
}
void inactiveOutputs() {
  for (uint8_t pin : {PYRO_ENABLE, PYRO_1, PYRO_2, PYRO_3, PYRO_4,
                      SERVO_PWM_1, SERVO_PWM_2, SERVO_PWM_3, SERVO_PWM_4}) {
    digitalWrite(pin, LOW); pinMode(pin, OUTPUT);
  }
  digitalWrite(CS_IMU, HIGH); pinMode(CS_IMU, OUTPUT);
  digitalWrite(CS_BARO, HIGH); pinMode(CS_BARO, OUTPUT);
}
// Independent mode-0 bit-bang reference; test-only, not part of production driver.
uint8_t softwareByte(uint8_t tx) {
  uint8_t rx = 0;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    digitalWrite(MOSI_FLASH, (tx & 0x80) ? HIGH : LOW);
    delayMicroseconds(10);
    digitalWrite(SCK_FLASH, HIGH);
    delayMicroseconds(10);
    rx = uint8_t((rx << 1) | (digitalRead(MISO_FLASH) ? 1 : 0));
    digitalWrite(SCK_FLASH, LOW);
    tx <<= 1;
  }
  return rx;
}
void softwareProbe() {
  logger.end();
  pinMode(CS_FLASH1, OUTPUT); digitalWrite(CS_FLASH1, HIGH);
  pinMode(MISO_FLASH, INPUT);
  digitalWrite(MOSI_FLASH, LOW); pinMode(MOSI_FLASH, OUTPUT);
  digitalWrite(SCK_FLASH, LOW); pinMode(SCK_FLASH, OUTPUT);
  digitalWrite(CS_FLASH1, LOW); softwareByte(0xAB); digitalWrite(CS_FLASH1, HIGH);
  delayMicroseconds(100);
  for (uint8_t repeat = 0; repeat < 3; ++repeat) {
    uint8_t bytes[8];
    digitalWrite(CS_FLASH1, LOW); softwareByte(0x9F);
    for (auto &byte : bytes) byte = softwareByte(0);
    digitalWrite(CS_FLASH1, HIGH);
    Serial.print("SOFTWARE mode0 ~50kHz JEDEC raw: "); printBytes(bytes, sizeof(bytes)); Serial.println();
    delay(5);
  }
}
void runDiagnostics() {
  Serial.println("\n=== FLASH_TEST: read-only W25Q128 diagnostics ===");
  Serial.println("Pins: MISO=GP0 CS=GP1 CLK=GP2 MOSI=GP3");
  Serial.println("Expected JEDEC: EF 40 18 or EF 70 18. Legacy ID: EF 17.");
  Serial.println("No write-enable, program, erase or reset commands. Outputs inactive.");
  unsigned matches = 0, reads = 0;
  constexpr uint32_t frequencies[] = {100000, 500000, 1000000, FLASH_CLOCK_HZ};
  for (uint8_t mode : {uint8_t(SPI_MODE0), uint8_t(SPI_MODE3)}) {
    for (uint32_t hz : frequencies) {
      logger.begin(hz, mode); logger.releasePowerDown();
      for (uint8_t repeat = 0; repeat < 3; ++repeat) {
        uint8_t bytes[8]; logger.readJedecBytes(bytes);
        const uint32_t id = (uint32_t(bytes[0]) << 16) | (uint32_t(bytes[1]) << 8) | bytes[2];
        if (FlashLogger::isW25Q128(id)) ++matches;
        ++reads;
        Serial.print("HW mode="); Serial.print(mode == SPI_MODE0 ? 0 : 3);
        Serial.print(" Hz="); Serial.print(hz); Serial.print(" #"); Serial.print(repeat + 1);
        Serial.print(" JEDEC raw: "); printBytes(bytes, sizeof(bytes));
        Serial.print(FlashLogger::isW25Q128(id) ? " MATCH" : " MISMATCH");
        Serial.print(" SR1=0x"); hexByte(logger.readStatus1());
        const uint16_t legacy = logger.readLegacyId();
        Serial.print(" legacy=0x"); hexByte(uint8_t(legacy >> 8)); hexByte(uint8_t(legacy));
        Serial.println(); delay(5);
      }
      logger.end();
    }
  }
  softwareProbe();
  Serial.print("Hardware matching reads: "); Serial.print(matches); Serial.print('/'); Serial.println(reads);
  Serial.println("An ID match verifies identification only, not memory integrity.");
  Serial.println("=== END: r + Enter repeats ID checks; w + Enter tests last 4KiB sector ===");
  Serial.println("w saves/restores that sector in RAM; power loss can destroy its original contents.");
  Serial.println("ERASE ALL + Enter erases and verifies the entire 16MiB device, including settings and flights.");
}
void eraseAll() {
  inactiveOutputs();
  if (!logger.initialize(FLASH_CLOCK_HZ)) {
    Serial.print("ERASE FAILED: "); Serial.println(FlashLogger::errorName(logger.lastError())); return;
  }
  Serial.println("=== FULL FLASH ERASE START: 16777216 bytes ===");
  for (uint32_t address=0; address<FlashLogger::Capacity; address+=FlashLogger::SectorSize) {
    if (!logger.eraseSector(address)) {
      Serial.print("ERASE FAILED at "); Serial.print(address); Serial.print(": ");
      Serial.println(FlashLogger::errorName(logger.lastError())); return;
    }
    const auto done=address+FlashLogger::SectorSize;
    if (done%(64*FlashLogger::SectorSize)==0) { Serial.print("ERASE verified bytes: "); Serial.println(done); }
    yield();
  }
  Serial.println("=== FULL FLASH ERASE VERIFIED: 16777216 bytes ===");
}
void runStorageTest() {
  constexpr uint32_t address = FlashLogger::Capacity - FlashLogger::SectorSize;
  static uint8_t backup[FlashLogger::SectorSize];
  static uint8_t expected[FlashLogger::SectorSize];
  static bool backupAtRisk = false;
  uint8_t verify[FlashLogger::PageSize];
  if (backupAtRisk) {
    Serial.println("Previous restore failed. Test blocked to preserve the RAM backup; do not reboot.");
    return;
  }
  Serial.println("\n=== FLASH storage test: ONLY 0xFFF000..0xFFFFFF ===");
  if (!logger.initialize(FLASH_CLOCK_HZ)) {
    Serial.print("INIT FAILED: "); Serial.println(FlashLogger::errorName(logger.lastError())); return;
  }
  if (!logger.read(address, backup, sizeof(backup))) {
    Serial.println("Backup read failed; no erase attempted."); return;
  }
  for (uint32_t offset = 0; offset < sizeof(backup); offset += sizeof(verify)) {
    if (!logger.read(address + offset, verify, sizeof(verify)) ||
        memcmp(verify, backup + offset, sizeof(verify)) != 0) {
      Serial.println("Backup unstable; no erase attempted."); return;
    }
  }
  // These rejected operations must never send an erase/program instruction.
  bool boundsOk = !logger.read(FlashLogger::Capacity, verify, 1) &&
                  logger.lastError() == FlashLogger::Error::Bounds;
  boundsOk = boundsOk && !logger.program(0xFFFFFFFFu, verify, 1) &&
             logger.lastError() == FlashLogger::Error::Bounds;
  boundsOk = boundsOk && !logger.eraseSector(address + 1) &&
             logger.lastError() == FlashLogger::Error::Bounds;
  boundsOk = boundsOk && !logger.eraseSector(address - 1) &&
             logger.lastError() == FlashLogger::Error::Alignment;
  if (!boundsOk) { Serial.println("Bounds check FAILED; test stopped before erase."); return; }
  Serial.println("Bounds/alignment checks PASS. Backup captured and double-read verified.");
  memset(expected, 0xFF, sizeof(expected));
  uint32_t random = 0x2350128u;
  for (uint32_t i = 3; i < sizeof(expected) - 3; ++i) {
    random ^= random << 13; random ^= random >> 17; random ^= random << 5;
    expected[i] = uint8_t(random);
  }
  backupAtRisk = true;
  bool passed = logger.eraseSector(address);
  if (passed) passed = logger.program(address + 3, expected + 3, sizeof(expected) - 6);
  if (passed) {
    for (uint32_t offset = 0; offset < sizeof(expected); offset += sizeof(verify)) {
      if (!logger.read(address + offset, verify, sizeof(verify)) ||
          memcmp(verify, expected + offset, sizeof(verify)) != 0) { passed = false; break; }
    }
  }
  if (passed) {
    // Force a known 0 bit, then ensure a 0->1 attempt is rejected.
    const uint8_t zero = 0, erased = 0xFF;
    passed = logger.program(address, &zero, 1) &&
             !logger.program(address, &erased, 1) &&
             logger.lastError() == FlashLogger::Error::NeedsErase;
  }
  const auto testError = logger.lastError();
  Serial.print("Erase / unaligned multi-page program / full readback / 0->1 rejection: ");
  Serial.println(passed ? "PASS" : "FAIL");
  if (!passed) { Serial.print("Driver result: "); Serial.println(FlashLogger::errorName(testError)); }
  // Always attempt restoration after the first erase, even when testing failed.
  bool restored = logger.eraseSector(address) && logger.program(address, backup, sizeof(backup));
  if (restored) {
    for (uint32_t offset = 0; offset < sizeof(backup); offset += sizeof(verify)) {
      if (!logger.read(address + offset, verify, sizeof(verify)) ||
          memcmp(verify, backup + offset, sizeof(verify)) != 0) { restored = false; break; }
    }
  }
  Serial.println(restored ? "Original sector RESTORED and VERIFIED." :
                           "RESTORE FAILED: sector contents may be lost; backup exists only in RAM.");
  backupAtRisk = !restored;
  Serial.println(passed && restored ? "=== STORAGE TEST PASSED (one sector only) ===" : "=== STORAGE TEST FAILED ===");
}
bool connected = false;
}
void setup() {
  inactiveOutputs();
  Serial.begin(115200);
  delay(100);
}
void loop() {
  if (!Serial) { connected = false; return; }
  if (!connected) {
    connected = true;
    delay(300);
    runDiagnostics();
  }
  static char command[16]; static uint8_t used=0; static bool overflow=false;
  while (Serial.available()) {
    const char c=char(Serial.read());
    if (c=='\r') continue;
    if (c=='\n') {
      command[used]=0;
      if (!overflow) {
        if (!strcmp(command,"r") || !strcmp(command,"R")) runDiagnostics();
        else if (!strcmp(command,"w") || !strcmp(command,"W")) runStorageTest();
        else if (!strcmp(command,"ERASE ALL")) eraseAll();
      }
      used=0; overflow=false;
    } else if (used<sizeof(command)-1) command[used++]=c;
    else overflow=true;
  }
}
#endif // FLASH_TEST
