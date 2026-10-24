#pragma once
#include "storage/RecordingTypes.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include "storage/FlashDevice.h"
#include "storage/FlashLayout.h"

namespace storage {
enum class DirectoryError : uint8_t {
  None, Io, Missing, Corrupt, NotMounted, InvalidEntry,
  InvalidTransition, Full, SequenceExhausted, Verify
};
class FlightDirectory {
public:
  static constexpr std::size_t MaxEntries = 48;
  static constexpr uint32_t SlotBytes = 512; // Data page + separate commit page.
  explicit FlightDirectory(FlashDevice &flash) : flash_(flash) {}
  bool mount(); // Read-only, never formats unknown contents.
  bool format(); // Explicit destructive operation: directory banks only.
  bool update(const FlightEntry &entry);
  bool compact();
  const std::array<FlightEntry, MaxEntries> &entries() const { return entries_; }
  std::size_t count() const { return count_; }
  std::size_t retainedCount() const {
    std::size_t count=0; for (std::size_t i=0; i<count_; ++i) if (entries_[i].status!=FlightStatus::Invalidated) ++count;
    return count;
  }
  std::size_t corruptCount() const {
    std::size_t count=0; for (std::size_t i=0; i<count_; ++i) if (entries_[i].status==FlightStatus::Corrupt) ++count;
    return count;
  }
  uint64_t highWater() const { return highWater_; }
  uint64_t generation() const { return generation_; }
  DirectoryError error() const { return error_; }
  static const char *errorName(DirectoryError error);
private:
  bool fail(DirectoryError error) { error_ = error; return false; }
  bool entryValid(const FlightEntry &entry) const;
  bool apply(const FlightEntry &entry);
  bool writeSlot(uint32_t bank, uint32_t slot, uint64_t generation,
                 const FlightEntry *entry, uint32_t baseline, uint64_t highWater);
  bool readSlot(uint32_t bank, uint32_t slot, uint8_t *body, bool &erased, bool &committed);
  bool eraseBank(uint32_t bank);
  FlashDevice &flash_;
  std::array<FlightEntry, MaxEntries> entries_{};
  std::size_t count_ = 0;
  uint64_t generation_ = 0;
  uint64_t highWater_ = 0;
  uint32_t activeBank_ = 0;
  uint32_t nextSlot_ = 1;
  bool mounted_ = false;
  DirectoryError error_ = DirectoryError::None;
};
} // namespace storage
