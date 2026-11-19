#pragma once
#include "shared/Timing.h"
#include <cstdint>

// Board timing/stack evidence for `diag`. MAIN fills it; host tests may fill it by hand.
// Loop and run figures restart at each run begin (arming, simulation or HIL), so after a
// run they describe that run; stack high-water marks are since boot.
struct StackUse {
  uint32_t capacityBytes = 0, usedBytes = 0;
  bool exhausted = false; // The lowest painted word was overwritten: usage reached the limit.
  uint32_t intactBytes = 0, intactLowOffset = 0, intactHighOffset = 0; // Painted words still intact, and where (from the bottom).
};
struct BoardDiagnostics {
  StackUse stack[2]; // Core 0 (main loop, navigation, commands), core 1 (recording worker).
  timing::Statistics loop; // Core 0 loop() period, entry to entry.
  uint32_t loopPeriodUs = 0, loopOverPeriod = 0; // Loop periods longer than one IMU period.
  const char *run = "none"; // Last run: physical, simulation, hil or none.
  bool runActive = false;
  timing::Statistics step, compute, storage;
  uint32_t overBudget = 0, frameDtMinUs = 0, frameDtMaxUs = 0, sequenceGaps = 0;
  uint32_t fifoOverruns = 0; // IMU FIFO overruns since the run began.
  const char *resetReason = "unknown"; // Cause of the last chip reset, read at boot.
  uint32_t resetChipBits = 0, resetWatchdogBits = 0; // Raw POWMAN CHIP_RESET and WATCHDOG REASON.
  // A fault before the last reset (stack limit or any HardFault), kept across the reboot it caused.
  bool lastFault = false, lastFaultStackOverflow = false;
  uint32_t lastFaultCore = 0, lastFaultCfsr = 0, lastFaultHfsr = 0, lastFaultSp = 0;
  void (*refresh)(BoardDiagnostics &) = nullptr; // Called just before a reply (stack scan).
};
