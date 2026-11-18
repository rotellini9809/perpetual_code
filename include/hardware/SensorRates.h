#pragma once
#include <cstdint>

namespace sensors {
// Keep the driver ODR selection and physical runtime timing in agreement.
constexpr uint16_t ImuRateHz = 240;
}
