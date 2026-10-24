#pragma once
#include "flight/OutputExecutor.h"
#include <utility>
#include <vector>

struct FakeOutputs : flight::OutputHardware {
  bool enabled=false, failServo=false;
  uint8_t mask=0, connected=15, checkedMask=0;
  uint8_t failChannel=0;
  unsigned writes=0;
  std::array<uint16_t,4> positions{};
  std::vector<std::pair<uint8_t,uint16_t>> servoCommands;
  bool continuity(uint8_t channel) override { checkedMask |= uint8_t(1u<<(channel-1)); return connected & (1u<<(channel-1)); }
  void pyros(bool enable,uint8_t value) override { enabled=enable; mask=value; ++writes; }
  bool servo(uint8_t channel,uint16_t pulse) override {
    ++writes; if (failServo || channel==failChannel) return false;
    positions[channel-1]=pulse; servoCommands.emplace_back(channel,pulse); return true;
  }
  void releaseServo(uint8_t channel) override { ++writes; positions[channel-1]=0; }
};
