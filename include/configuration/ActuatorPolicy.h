#pragma once
#include <cstdint>

namespace recovery {
// Current fixed board policy; no retry sequence is implemented.
constexpr uint32_t PyroPulseMs = 1000;
constexpr uint32_t PyroEnableSettleMs = 10;
}
