#pragma once
#include <cstddef>
#include <cstdint>
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
  virtual bool eraseSector(uint32_t address) = 0;
  virtual FlashFailure lastFailure() const { return {}; }
};
}
