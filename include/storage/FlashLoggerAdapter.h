#pragma once
#include "storage/FlashDevice.h"
#include "storage/FlashLogger.h"

namespace storage {
class FlashLoggerAdapter : public FlashDevice {
public:
  explicit FlashLoggerAdapter(FlashLogger &flash) : flash_(flash) {}
  bool read(uint32_t a, void *p, std::size_t n) override { return result(flash_.read(a, p, n), FlashOperation::Read, a, n); }
  bool program(uint32_t a, const void *p, std::size_t n) override { return result(flash_.program(a, p, n), FlashOperation::Program, a, n); }
  bool eraseSector(uint32_t a) override { return result(flash_.eraseSector(a), FlashOperation::Erase, a, FlashLogger::SectorSize); }
  FlashFailure lastFailure() const override { return failure_; }
private:
  bool result(bool ok, FlashOperation operation, uint32_t address, std::size_t bytes) {
    failure_ = {}; if (ok) return true;
    failure_.operation = operation; failure_.address = address; failure_.bytes = bytes;
    switch (flash_.lastError()) {
#define MAP_FLASH(x) case FlashLogger::Error::x: failure_.cause = FlashError::x; break
      MAP_FLASH(None); MAP_FLASH(NotInitialized); MAP_FLASH(Identification); MAP_FLASH(Bounds);
      MAP_FLASH(Alignment); MAP_FLASH(InvalidArgument); MAP_FLASH(Timeout); MAP_FLASH(WriteEnable);
      MAP_FLASH(NeedsErase); MAP_FLASH(Verify);
#undef MAP_FLASH
    }
    if (failure_.cause == FlashError::None) failure_.cause = FlashError::Unknown;
    return false;
  }
  FlashLogger &flash_;
  FlashFailure failure_;
};
}
