#pragma once
#include "storage/ConfigurationStore.h"
#include <cassert>
#include <cstring>
#include <vector>
namespace replay_test {
class MemoryFlash : public storage::FlashDevice {
public:
  std::vector<uint8_t> bytes = std::vector<uint8_t>(0x5000, 255);
  unsigned mutations = 0;
  bool read(uint32_t a, void *p, std::size_t n) override {
    if (a > bytes.size() || n > bytes.size() - a) return false;
    memcpy(p, bytes.data() + a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    assert(a >= 0x1000 && a + n <= 0x5000); ++mutations;
    const auto *source = static_cast<const uint8_t *>(p);
    for (std::size_t i = 0; i < n; ++i) bytes[a + i] &= source[i];
    return true;
  }
  bool eraseSector(uint32_t a) override { assert(a >= 0x1000 && a < 0x5000); ++mutations; memset(bytes.data() + a, 255, 4096); return true; }
};
const recovery::ValidationLimits limits{1000, 2000, 1000, 3600000};
inline recovery::Configuration fixture() {
  using namespace recovery;
  Configuration c; c.ruleCount = 2;
  auto &a = c.rules[0]; a.id = 10; strcpy(a.name.data(), "Example apogee");
  a.conditionCount = 1; a.assignmentCount = 2; a.assignments[0].channel = 1;
  a.assignments[1].kind = OutputKind::Servo; a.assignments[1].channel = 1;
  c.pyros[0].pulseMs = 100; c.pyros[1].pulseMs = 100;
  auto &s = c.servos[0]; s.calibrated = true; s.initialUs = 1000; s.finalUs = 2000;
  auto &b = c.rules[1]; b.id = 20; strcpy(b.name.data(), "Example low descent");
  b.conditionCount = 2; b.assignmentCount = 1; b.delayMs = 100; b.assignments[0].channel = 2;
  b.conditions[0].kind = ConditionKind::Phase; b.conditions[0].phases = DescentMask;
  b.conditions[1].kind = ConditionKind::AltitudeComparison; b.conditions[1].value = 30;
  return c;
}
}
