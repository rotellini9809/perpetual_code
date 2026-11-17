#pragma once
#include <cstdint>

namespace storage {
// Fixed board: external Winbond W25Q128JV, 128 Mbit / 16 MiB.
// This is the single product layout, shared by firmware and host tests.
struct FlashLayout {
  static constexpr uint32_t Capacity = 16u * 1024u * 1024u;
  static constexpr uint32_t SectorBytes = 4096, PageBytes = 256;
  // Accelerometer bias/scale and append-only validation journal. Gyro stays in RAM.
  static constexpr uint32_t CalibrationBegin = 0, CalibrationBytes = SectorBytes;
  static constexpr uint32_t ConfigurationBegin = 0x001000, ConfigurationBytes = 0x004000;
  static constexpr uint32_t DirectoryA = 0x005000, DirectoryB = 0x00D000, DirectoryBankBytes = 0x008000;
  // One-time identity eligibility; never erased by application maintenance.
  static constexpr uint32_t IdentityPolicyBegin = Capacity - SectorBytes;
  static constexpr uint32_t DataBegin = 0x015000, DataEnd = IdentityPolicyBegin;
  static constexpr bool dataRange(uint32_t address, uint32_t bytes) {
    return address >= DataBegin && address <= DataEnd && bytes <= DataEnd - address;
  }
};
static_assert(FlashLayout::CalibrationBegin + FlashLayout::CalibrationBytes == FlashLayout::ConfigurationBegin,
              "Calibration/settings overlap or gap");
static_assert(FlashLayout::ConfigurationBegin + FlashLayout::ConfigurationBytes == FlashLayout::DirectoryA &&
              FlashLayout::DirectoryA + FlashLayout::DirectoryBankBytes == FlashLayout::DirectoryB &&
              FlashLayout::DirectoryB + FlashLayout::DirectoryBankBytes == FlashLayout::DataBegin,
              "Storage regions must be contiguous and disjoint");
static_assert(FlashLayout::ConfigurationBegin % FlashLayout::SectorBytes == 0 &&
              FlashLayout::DirectoryA % FlashLayout::SectorBytes == 0 &&
              FlashLayout::DirectoryBankBytes % FlashLayout::SectorBytes == 0 &&
              FlashLayout::DataEnd % FlashLayout::SectorBytes == 0 && FlashLayout::DataBegin < FlashLayout::DataEnd &&
              FlashLayout::SectorBytes % FlashLayout::PageBytes == 0, "Invalid flash geometry/alignment");
}
