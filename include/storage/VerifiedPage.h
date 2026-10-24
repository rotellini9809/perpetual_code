#pragma once
#include "storage/FlashDevice.h"
#include "storage/FlashLayout.h"
#include <cstring>

namespace storage {
enum class PageWriteError { None, Io, Verify };
// One program followed by readback. No retries, erases or extra I/O after failure.
// Source and scratch must be distinct; the caller owns alignment and commit order.
inline PageWriteError writeVerifiedPage(FlashDevice &flash, uint32_t address, const uint8_t *source,
                                       uint8_t (&scratch)[FlashLayout::PageBytes]) {
  if (!flash.program(address, source, FlashLayout::PageBytes) ||
      !flash.read(address, scratch, FlashLayout::PageBytes)) return PageWriteError::Io;
  return std::memcmp(source, scratch, FlashLayout::PageBytes) ? PageWriteError::Verify : PageWriteError::None;
}
}
