#include "storage/FlashLogger.h"


void FlashLogger::begin(uint32_t hz, uint8_t mode) {
  ready_ = false;
  digitalWrite(cs_, HIGH);
  pinMode(cs_, OUTPUT);
  hz_ = hz; mode_ = mode;
  bus_.begin(false);
  begun_ = true;
}
void FlashLogger::end() {
  ready_ = false;
  if (begun_) bus_.end();
  digitalWrite(cs_, HIGH);
  begun_ = false;
}
void FlashLogger::exchange(const uint8_t *tx, uint8_t *rx, std::size_t count) {
  if (!begun_) { if (rx) memset(rx, 0, count); return; }
  bus_.beginTransaction(SPISettings(hz_, MSBFIRST, mode_));
  digitalWrite(cs_, LOW);
  delayMicroseconds(2);
  bus_.transfer(tx, rx, count);
  delayMicroseconds(2);
  digitalWrite(cs_, HIGH);
  bus_.endTransaction();
  delayMicroseconds(5);
}
void FlashLogger::releasePowerDown() {
  const uint8_t tx[] = {0xAB}; uint8_t rx[1];
  exchange(tx, rx, sizeof(tx));
  delayMicroseconds(100);
}
bool FlashLogger::powerDown() {
  if (!ready_) return fail(Error::NotInitialized);
  if (!waitReady(2000)) return false;
  const uint8_t command=0xB9; uint8_t ignored=0;
  exchange(&command,&ignored,1); delayMicroseconds(100);
  end(); error_=Error::None; return true;
}
void FlashLogger::readJedecBytes(uint8_t (&bytes)[8]) {
  const uint8_t tx[9] = {0x9F}; uint8_t rx[9] = {};
  exchange(tx, rx, sizeof(tx));
  memcpy(bytes, rx + 1, sizeof(bytes));
}
uint32_t FlashLogger::readJEDEC() {
  uint8_t bytes[8]; readJedecBytes(bytes);
  return (uint32_t(bytes[0]) << 16) | (uint32_t(bytes[1]) << 8) | bytes[2];
}
uint8_t FlashLogger::readStatus1() {
  const uint8_t tx[] = {0x05, 0}; uint8_t rx[2] = {};
  exchange(tx, rx, sizeof(tx)); return rx[1];
}
uint16_t FlashLogger::readLegacyId() {
  const uint8_t tx[] = {0x90, 0, 0, 0, 0, 0}; uint8_t rx[6] = {};
  exchange(tx, rx, sizeof(tx)); return (uint16_t(rx[4]) << 8) | rx[5];
}

const char *FlashLogger::errorName(Error error) {
  switch (error) {
#define FLASH_ERROR(e) case Error::e: return #e
    FLASH_ERROR(None); FLASH_ERROR(NotInitialized); FLASH_ERROR(Identification);
    FLASH_ERROR(Bounds); FLASH_ERROR(Alignment); FLASH_ERROR(InvalidArgument);
    FLASH_ERROR(Timeout); FLASH_ERROR(WriteEnable); FLASH_ERROR(NeedsErase); FLASH_ERROR(Verify);
#undef FLASH_ERROR
  }
  return "Unknown";
}
void FlashLogger::select() {
  bus_.beginTransaction(SPISettings(hz_, MSBFIRST, mode_));
  digitalWrite(cs_, LOW); delayMicroseconds(2);
}
void FlashLogger::deselect() {
  delayMicroseconds(2); digitalWrite(cs_, HIGH);
  bus_.endTransaction(); delayMicroseconds(5);
}
void FlashLogger::sendAddress(uint8_t opcode, uint32_t address) {
  const uint8_t header[] = {opcode, uint8_t(address >> 16), uint8_t(address >> 8), uint8_t(address)};
  bus_.transfer(header, nullptr, sizeof(header));
}
bool FlashLogger::initialize(uint32_t hz, uint8_t mode) {
  begin(hz, mode); releasePowerDown();
  const uint32_t id = readJEDEC();
  if (!isW25Q128(id) || readJEDEC() != id || readJEDEC() != id) return fail(Error::Identification);
  if (!waitReady(2000)) return false;
  ready_ = true; error_ = Error::None; return true;
}
bool FlashLogger::waitReady(uint32_t timeoutMs) {
  if (!begun_) return fail(Error::NotInitialized);
  const uint32_t start = millis();
  do {
    if (!(readStatus1() & 1)) { error_ = Error::None; return true; }
    if (uint32_t(millis() - start) >= timeoutMs) {
      ready_ = false; return fail(Error::Timeout);
    }
    delay(1);
  } while (true);
}
bool FlashLogger::checkRange(uint32_t address, const void *data, std::size_t length) {
  if (!ready_) return fail(Error::NotInitialized);
  if (address >= Capacity || length > Capacity - address) return fail(Error::Bounds);
  if (length && !data) return fail(Error::InvalidArgument);
  return true;
}
bool FlashLogger::read(uint32_t address, void *data, std::size_t length) {
  if (!checkRange(address, data, length) || !waitReady(2000)) return false;
  if (length) {
    select(); sendAddress(0x03, address); bus_.transfer(nullptr, data, length); deselect();
  }
  error_ = Error::None; return true;
}
bool FlashLogger::writeEnable() {
  const uint8_t opcode = 0x06; uint8_t rx;
  exchange(&opcode, &rx, 1);
  if (!(readStatus1() & 2)) return fail(Error::WriteEnable);
  return true;
}
bool FlashLogger::program(uint32_t address, const void *data, std::size_t length) {
  if (!checkRange(address, data, length)) return false;
  const auto *source = static_cast<const uint8_t *>(data);
  uint8_t buffer[PageSize];
  // Preflight the complete range before making any changes.
  for (std::size_t offset = 0; offset < length;) {
    const std::size_t count = (length - offset < PageSize) ? length - offset : PageSize;
    if (!read(address + offset, buffer, count)) return false;
    for (std::size_t i = 0; i < count; ++i)
      if ((buffer[i] & source[offset + i]) != source[offset + i]) return fail(Error::NeedsErase);
    offset += count;
  }
  for (std::size_t offset = 0; offset < length;) {
    const uint32_t current = address + offset;
    const std::size_t room = PageSize - current % PageSize;
    const std::size_t count = (length - offset < room) ? length - offset : room;
    if (!waitReady(2000) || !writeEnable()) return false;
    select(); sendAddress(0x02, current);
    bus_.transfer(source + offset, nullptr, count); deselect();
    if (!waitReady(20) || !read(current, buffer, count)) return false;
    if (memcmp(buffer, source + offset, count) != 0) return fail(Error::Verify);
    offset += count;
  }
  error_ = Error::None; return true;
}
bool FlashLogger::eraseSector(uint32_t address) {
  uint8_t buffer[PageSize];
  if (!checkRange(address, buffer, SectorSize)) return false;
  if (address % SectorSize) return fail(Error::Alignment);
  if (!waitReady(2000) || !writeEnable()) return false;
  select(); sendAddress(0x20, address); deselect();
  if (!waitReady(2000)) return false;
  for (uint32_t offset = 0; offset < SectorSize; offset += PageSize) {
    if (!read(address + offset, buffer, sizeof(buffer))) return false;
    for (auto byte : buffer) if (byte != 0xFF) return fail(Error::Verify);
  }
  error_ = Error::None; return true;
}
