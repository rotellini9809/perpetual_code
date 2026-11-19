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
  Serial.println("=== END: r + Enter repeats ID checks; w + Enter tests the last flight-data sector; c + Enter sweeps SPI clocks 4-75 MHz ===");
  Serial.println("w saves/restores that sector in RAM; power loss can destroy its original contents.");
  Serial.println("w uses the last flight-ring sector only, never the device journal or directory (blocks 0..2).");
  Serial.println("ERASE ALL + Enter erases the entire 16MiB device: journal (calibration, settings, rules, identity revocation), directory and flights.");
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
// Last sector of the flight ring; device metadata (blocks 0..2) is never touched here.
constexpr uint32_t TestSector = storage::FlashLayout::Capacity - FlashLogger::SectorSize;
static_assert(TestSector >= storage::FlashLayout::RingBegin, "Test sector overlaps device metadata");
uint8_t backup[FlashLogger::SectorSize];
bool backupAtRisk = false; // Shared by w and c: a failed restore blocks both until reboot.
uint32_t crc32(const uint8_t *p, std::size_t n, uint32_t crc) {
  for (std::size_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (unsigned bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return crc;
}
// Clock sweep. Read-only part: JEDEC repeats and CRC of 64 KiB from the start of
// flight data, with 0x03 and 0x0B, against a 4 MHz reference. Write part: only
// TestSector (backup in RAM, restored and verified at the end, like `w`): one
// pseudo-random page per clock is programmed at that clock, then read at 4 MHz.
void clockSweep() {
  if (backupAtRisk) { Serial.println("Previous restore failed. Sweep blocked to preserve the RAM backup; do not reboot."); return; }
  inactiveOutputs();
  constexpr uint32_t Reference = 4000000, RegionBegin = storage::FlashLayout::DataBegin, RegionBytes = 65536;
  static uint8_t chunk[4096];
  uint8_t page[FlashLogger::PageSize], check[FlashLogger::PageSize];
  const auto regionCrc = [&](uint32_t &crc, unsigned &nonErased) {
    crc = 0xFFFFFFFFu; nonErased = 0;
    for (uint32_t offset = 0; offset < RegionBytes; offset += sizeof(chunk)) {
      if (!logger.read(RegionBegin + offset, chunk, sizeof(chunk))) return false;
      for (auto byte : chunk) nonErased += byte != 0xFF;
      crc = crc32(chunk, sizeof(chunk), crc);
    }
    crc = ~crc; return true;
  };
  if (!logger.initialize(Reference)) { Serial.print("INIT FAILED: "); Serial.println(FlashLogger::errorName(logger.lastError())); return; }
  const uint32_t jedec = logger.readJEDEC();
  uint32_t referenceCrc = 0; unsigned nonErased = 0;
  if (!regionCrc(referenceCrc, nonErased)) { Serial.println("Reference read failed."); return; }
  if (!logger.read(TestSector, backup, sizeof(backup))) { Serial.println("Backup read failed; no erase attempted."); return; }
  for (uint32_t offset = 0; offset < sizeof(backup); offset += sizeof(check))
    if (!logger.read(TestSector + offset, check, sizeof(check)) || memcmp(check, backup + offset, sizeof(check))) {
      Serial.println("Backup unstable; no erase attempted."); return;
    }
  Serial.printf("=== FLASH CLOCK SWEEP: JEDEC=%06lX, region 0x%06lX+64KiB CRC=%08lX (%u non-FF bytes), test sector 0x%06lX ===\n",
      (unsigned long)jedec, (unsigned long)RegionBegin, (unsigned long)referenceCrc, nonErased, (unsigned long)TestSector);
  backupAtRisk = true;
  bool erased = logger.eraseSector(TestSector);
  if (!erased) Serial.println("Test sector erase FAILED; write checks skipped.");
  const uint32_t speeds[] = {4000000, 8000000, 12500000, 18750000, 25000000, 30000000, 37500000, 50000000, 75000000};
  for (unsigned index = 0; index < sizeof(speeds) / sizeof(speeds[0]); ++index) {
    const uint32_t hz = speeds[index];
    const bool init = logger.initialize(hz);
    flashSpi.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
    const uint32_t actual = spi_get_baudrate(FLASH_SPI_PORT);
    flashSpi.endTransaction();
    unsigned jedecBad = 0, slowBad = 0, fastBad = 0; uint32_t slowUs = 0, fastUs = 0, programUs = 0;
    bool written = false;
    if (init) {
      for (unsigned i = 0; i < 200; ++i) jedecBad += logger.readJEDEC() != jedec;
      for (unsigned fast = 0; fast < 2; ++fast) {
        logger.setFastRead(fast);
        for (unsigned repeat = 0; repeat < 4; ++repeat) {
          uint32_t crc = 0; unsigned ignored = 0; const uint32_t start = micros();
          const bool ok = regionCrc(crc, ignored) && crc == referenceCrc;
          (fast ? fastUs : slowUs) += micros() - start;
          (fast ? fastBad : slowBad) += !ok;
        }
      }
      if (erased) {
        uint32_t random = 0x9E3779B9u ^ hz;
        for (auto &byte : page) { random ^= random << 13; random ^= random >> 17; random ^= random << 5; byte = uint8_t(random); }
        logger.setFastRead(hz > 50000000);
        const uint32_t start = micros();
        logger.programErased(TestSector + index * FlashLogger::PageSize, page, sizeof(page)); // Verifies at this clock.
        programUs = micros() - start;
        logger.setFastRead(false);
        written = logger.initialize(Reference) && logger.read(TestSector + index * FlashLogger::PageSize, check, sizeof(check)) &&
                  !memcmp(check, page, sizeof(page));
      }
    }
    logger.setFastRead(false);
    const bool pass = init && !jedecBad && !slowBad && !fastBad && written;
    Serial.printf("FLASH SWEEP req=%lu actual=%lu init=%u JEDEC_bad=%u/200 read03_bad=%u/4 read0B_bad=%u/4 64KiB_us 03=%lu 0B=%lu page_program_us=%lu write@4MHz=%s %s\n",
        (unsigned long)hz, (unsigned long)actual, init, jedecBad, slowBad, fastBad, (unsigned long)(slowUs / 4), (unsigned long)(fastUs / 4),
        (unsigned long)programUs, erased ? (written ? "OK" : "BAD") : "SKIPPED", pass ? "PASS" : "FAIL");
  }
  bool restored = logger.initialize(Reference) && logger.eraseSector(TestSector) && logger.program(TestSector, backup, sizeof(backup));
  for (uint32_t offset = 0; restored && offset < sizeof(backup); offset += sizeof(check))
    restored = logger.read(TestSector + offset, check, sizeof(check)) && !memcmp(check, backup + offset, sizeof(check));
  backupAtRisk = !restored;
  Serial.println(restored ? "Test sector RESTORED and VERIFIED." : "RESTORE FAILED: test sector backup exists only in RAM; do not reboot.");
  Serial.println("=== END FLASH SWEEP (0x03 above 50 MHz is outside the datasheet; reported for margin only) ===");
}
void runStorageTest() {
  constexpr uint32_t address = TestSector;
  static uint8_t expected[FlashLogger::SectorSize];
  uint8_t verify[FlashLogger::PageSize];
  if (backupAtRisk) {
    Serial.println("Previous restore failed. Test blocked to preserve the RAM backup; do not reboot.");
    return;
  }
  Serial.println("\n=== FLASH storage test: ONLY 0xFFE000..0xFFEFFF (last flight-data sector) ===");
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
  boundsOk = boundsOk && !logger.eraseSector(FlashLogger::Capacity) &&
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
        else if (!strcmp(command,"c") || !strcmp(command,"C")) clockSweep();
        else if (!strcmp(command,"ERASE ALL")) eraseAll();
      }
      used=0; overflow=false;
    } else if (used<sizeof(command)-1) command[used++]=c;
    else overflow=true;
  }
}
#endif // FLASH_TEST
