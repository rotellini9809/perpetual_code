#pragma once
#include <array>
#include <cstdint>

namespace control {
enum AirbrakeFlag : uint16_t {
  Configured=1, Enabled=2, RuleFalse=4, InvalidEstimate=8, Aborted=16,
  InvalidRequest=32, OutputFault=64, StaleEstimate=128, NavigationBlocked=256
};
struct AirbrakeStatus {
  uint16_t flags=0;
  uint8_t failedConditions=0, channelMask=0;
  std::array<uint16_t,4> opening{{65535,65535,65535,65535}}; // 0..10000; FFFF unassigned.
};
}
