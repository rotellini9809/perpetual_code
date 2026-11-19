#pragma once
#include <cstdint>

namespace storage {
// Fixed board: external Winbond W25Q128JV, 128 Mbit / 16 MiB.
// This is the single product layout (STORAGE_FORMAT.md, "Layout v2"), shared by firmware and
// host tests. Every large region is 64 KiB aligned so each erase is a single block command.
struct FlashLayout {
  static constexpr uint32_t Capacity = 16u * 1024u * 1024u;
  static constexpr uint32_t SectorBytes = 4096, PageBytes = 256, BlockBytes = 65536;
  // Block 0: typed device journal (IMU calibration, settings, rules, identity revocation), A/B.
  static constexpr uint32_t JournalA = 0x000000, JournalB = 0x008000, JournalBankBytes = 0x008000;
  // Blocks 1..2: flight directory, A/B.
  static constexpr uint32_t DirectoryA = 0x010000, DirectoryB = 0x020000, DirectoryBankBytes = 0x010000;
  // Blocks 3..255: flight ring.
  static constexpr uint32_t RingBegin = 0x030000, RingBlocks = (Capacity - RingBegin) / BlockBytes;
  static constexpr uint32_t DataBegin = RingBegin, DataEnd = Capacity;
  static constexpr bool dataRange(uint32_t address, uint32_t bytes) {
    return address >= DataBegin && address <= DataEnd && bytes <= DataEnd - address;
  }
  static constexpr uint32_t blockAddress(uint32_t block) { return RingBegin + block * BlockBytes; }
  static constexpr uint32_t RingBytes = RingBlocks * BlockBytes;
  // Physical address of byte `offset` of a flight that starts at ring address `start` (wrapping).
  static constexpr uint32_t ringAddress(uint32_t start, uint32_t offset) {
    return RingBegin + uint32_t((uint64_t(start - RingBegin) + offset) % RingBytes);
  }
};
static_assert(FlashLayout::JournalA + FlashLayout::JournalBankBytes == FlashLayout::JournalB &&
              FlashLayout::JournalB + FlashLayout::JournalBankBytes == FlashLayout::DirectoryA &&
              FlashLayout::DirectoryA + FlashLayout::DirectoryBankBytes == FlashLayout::DirectoryB &&
              FlashLayout::DirectoryB + FlashLayout::DirectoryBankBytes == FlashLayout::RingBegin,
              "Storage regions must be contiguous and disjoint");
static_assert(FlashLayout::DirectoryA % FlashLayout::BlockBytes == 0 && FlashLayout::RingBegin % FlashLayout::BlockBytes == 0 &&
              (FlashLayout::Capacity - FlashLayout::RingBegin) % FlashLayout::BlockBytes == 0 && FlashLayout::RingBlocks == 253 &&
              FlashLayout::SectorBytes % FlashLayout::PageBytes == 0, "Invalid flash geometry/alignment");
}
