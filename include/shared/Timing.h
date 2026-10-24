#pragma once
#include <cstdint>

namespace timing {
using Clock = uint32_t (*)();
struct Statistics {
  uint32_t count = 0, maxUs = 0;
  uint64_t totalUs = 0;
  void add(uint32_t elapsedUs) {
    ++count; totalUs += elapsedUs;
    if (elapsedUs > maxUs) maxUs = elapsedUs;
  }
};
}
