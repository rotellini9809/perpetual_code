#pragma once
#include "storage/FlashDevice.h"
#include "storage/FlashLayout.h"
#include <cstddef>
#include <cstdint>

namespace storage {
enum class JournalError : uint8_t { None, Io, Corrupt, Full, InvalidArgument };
// Typed append-only device journal in block 0 (STORAGE_FORMAT.md, "Device journal v1").
// Two 32 KiB banks; the valid bank header with the highest generation is active. For each
// type the newest committed record wins; a new copy is always written beside the old one,
// and compaction copies the newest record of every type before the old bank may be erased.
// Every operation rescans flash, so several users of the same journal never disagree.
class DeviceJournal {
public:
  // Each setting is its own type, so damage to one never touches another.
  enum Type : uint8_t { ImuCalibration = 1, Orientation = 2, Rules = 3, IdentityRevoked = 4, StateMachine = 5, Airbrakes = 6 };
  static constexpr unsigned TypeCount = 6, BankPages = FlashLayout::JournalBankBytes / FlashLayout::PageBytes;
  static constexpr std::size_t MaxPayload = 4096;
  struct Found {
    bool present = false;
    bool damagedAfter = false; // A torn, damaged or unreadable record follows the newest valid one of this type.
    bool corruptAfter = false; // A committed record of this type, newer than the valid one, was damaged later.
    uint32_t length = 0;
  };
  explicit DeviceJournal(FlashDevice &flash) : flash_(flash) {}
  // Copies the newest committed record of `type`; found.present is false when there is none.
  bool read(Type type, uint8_t *payload, std::size_t capacity, Found &found);
  bool write(Type type, const uint8_t *payload, std::size_t length);
  // Explicit reset: a compaction that keeps only the types set in keepMask (bit 1 << type).
  bool reset(uint8_t keepMask);
  JournalError error() const { return error_; }
  static const char *errorName(JournalError error);
private:
  struct Latest { bool valid = false, damagedAfter = false, corruptAfter = false; uint16_t page = 0; uint32_t length = 0; uint64_t sequence = 0; };
  bool scan();
  bool initialize(); // Only on a completely erased block 0.
  bool compact(uint8_t keepMask);
  // flags: damage inherited by a compaction copy (bit 0 damaged, bit 1 corrupt); 0 for new writes.
  bool append(uint32_t bank, uint32_t page, Type type, uint64_t sequence, const uint8_t *payload, std::size_t length, uint8_t flags = 0);
  bool readPayload(uint32_t bank, const Latest &latest, uint8_t *payload, std::size_t capacity);
  bool commitBank(uint32_t bank, uint64_t generation, uint64_t nextSequence);
  bool fail(JournalError error) { error_ = error; return false; }
  FlashDevice &flash_;
  Latest latest_[TypeCount + 1];
  bool haveBank_ = false, erased_ = false;
  uint32_t bank_ = 0, appendPage_ = 2;
  uint64_t generation_ = 0, nextSequence_ = 1;
  uint8_t page_[FlashLayout::PageBytes] = {}, check_[FlashLayout::PageBytes] = {};
  uint8_t copy_[MaxPayload] = {};
  JournalError error_ = JournalError::None;
};
}
