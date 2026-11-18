#pragma once
#include <cstddef>
#include <cstdint>
#include "storage/FlashLayout.h"
namespace storage {
enum class FlashError : uint8_t { None, Unknown, NotInitialized, Identification, Bounds, Alignment, InvalidArgument, Timeout, WriteEnable, NeedsErase, Verify };
enum class FlashOperation : uint8_t { None, Read, Program, Erase };
struct FlashFailure {
  FlashError cause = FlashError::None;
  FlashOperation operation = FlashOperation::None;
  uint32_t address = 0;
  std::size_t bytes = 0;
};
inline const char *flashErrorName(FlashError e) {
  switch (e) {
#define FLASH_CAUSE(x) case FlashError::x: return #x
    FLASH_CAUSE(None); FLASH_CAUSE(Unknown); FLASH_CAUSE(NotInitialized); FLASH_CAUSE(Identification);
    FLASH_CAUSE(Bounds); FLASH_CAUSE(Alignment); FLASH_CAUSE(InvalidArgument); FLASH_CAUSE(Timeout);
    FLASH_CAUSE(WriteEnable); FLASH_CAUSE(NeedsErase); FLASH_CAUSE(Verify);
#undef FLASH_CAUSE
  }
  return "Unknown";
}
// One owner performs all flash access; implementations enforce device bounds.
class FlashDevice {
public:
  virtual ~FlashDevice() = default;
  virtual bool read(uint32_t address, void *data, std::size_t length) = 0;
  virtual bool program(uint32_t address, const void *data, std::size_t length) = 0;
  // Owner-guaranteed erased extent: may skip the pre-write erased check, never verification.
  virtual bool programErased(uint32_t address, const void *data, std::size_t length) { return program(address, data, length); }
  virtual bool eraseSector(uint32_t address) = 0;
  // Sector-aligned bulk erase. Implementations may use larger erase blocks and skip
  // read-back; callers must verify the erased state before relying on it.
  virtual bool eraseRange(uint32_t address, uint32_t bytes) {
    for (uint32_t offset = 0; offset < bytes; offset += FlashLayout::SectorBytes)
      if (!eraseSector(address + offset)) return false;
    return true;
  }
  virtual FlashFailure lastFailure() const { return {}; }
};
}
