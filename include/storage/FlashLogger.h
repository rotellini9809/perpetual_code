#pragma once
#include "storage/FlashLayout.h"
#include <Arduino.h>
#include <SPI.h>

// Device access foundation. Call from a storage worker, never a flight loop/ISR.
// Flight layout and persistence belong to the storage components above this driver.
class FlashLogger {
public:
  FlashLogger(SPIClassRP2040 &bus, uint8_t cs) : bus_(bus), cs_(cs) {}
  void begin(uint32_t hz = 1000000, uint8_t mode = SPI_MODE0);
  void end();
  void releasePowerDown();
  bool powerDown(); // Idle owner only; initialize() releases power-down and verifies identity.
  uint32_t readJEDEC();
  void readJedecBytes(uint8_t (&bytes)[8]);
  uint8_t readStatus1();
  uint16_t readLegacyId();
  static bool isW25Q128(uint32_t id) { return id == 0xEF4018 || id == 0xEF7018; }
  static constexpr uint32_t Capacity = storage::FlashLayout::Capacity;
  static constexpr uint32_t SectorSize = storage::FlashLayout::SectorBytes;
  static constexpr uint32_t PageSize = storage::FlashLayout::PageBytes;
  enum class Error : uint8_t {
    None, NotInitialized, Identification, Bounds, Alignment, InvalidArgument,
    Timeout, WriteEnable, NeedsErase, Verify
  };
  // Validate repeated identification before enabling data operations.
  bool initialize(uint32_t hz = 1000000, uint8_t mode = SPI_MODE0);
  bool read(uint32_t address, void *data, std::size_t length);
  // Splits at page boundaries, rejects attempted 0->1 changes, verifies each page.
  bool program(uint32_t address, const void *data, std::size_t length);
  bool eraseSector(uint32_t address); // Must be aligned; verifies all 4096 bytes.
  bool waitReady(uint32_t timeoutMs);
  Error lastError() const { return error_; }
  static const char *errorName(Error error);
private:
  bool fail(Error error) { error_ = error; return false; }
  bool checkRange(uint32_t address, const void *data, std::size_t length);
  bool writeEnable();
  void select();
  void deselect();
  void sendAddress(uint8_t opcode, uint32_t address);
  void exchange(const uint8_t *tx, uint8_t *rx, std::size_t count);
  SPIClassRP2040 &bus_;
  uint8_t cs_;
  uint32_t hz_ = 1000000;
  uint8_t mode_ = SPI_MODE0;
  bool begun_ = false;
  bool ready_ = false;
  Error error_ = Error::None;
};
