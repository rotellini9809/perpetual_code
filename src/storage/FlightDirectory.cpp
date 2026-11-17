#include "storage/FlightDirectory.h"
#include "storage/BinaryEncoding.h"
#include "storage/VerifiedPage.h"
#include <cstring>
#include <limits>

namespace storage {
namespace {
using binary::put;
using binary::get64;
using binary::crc;
static_assert(FlashLayout::DirectoryBankBytes >= (FlightDirectory::MaxEntries + 4) * FlightDirectory::SlotBytes, "Directory bank too small");
constexpr uint32_t Sector = FlashLayout::SectorBytes, Page = FlashLayout::PageBytes, Magic = 0x52494450, Commit = 0x54494D43;
bool allFF(const uint8_t *p) { for (unsigned i = 0; i < Page; ++i) if (p[i] != 255) return false; return true; }
bool bodyValid(const uint8_t *p, uint32_t slot, uint64_t generation) {
  return get64(p, 4) == Magic && get64(p + 4, 2) == 1 && get64(p + 8, 8) == generation &&
         get64(p + 16, 4) == slot && get64(p + 252, 4) == crc(p, 252);
}
FlightEntry decode(const uint8_t *p) {
  FlightEntry e;
  e.sequence = get64(p + 24, 8); e.start = uint32_t(get64(p + 32, 4));
  e.committedBytes = uint32_t(get64(p + 36, 4)); e.allocatedBytes = uint32_t(get64(p + 40, 4));
  e.status = static_cast<FlightStatus>(p[44]); return e;
}
}
const char *FlightDirectory::errorName(DirectoryError e) {
  switch (e) {
#define DIR_ERROR(x) case DirectoryError::x: return #x
    DIR_ERROR(None); DIR_ERROR(Io); DIR_ERROR(Missing); DIR_ERROR(Corrupt);
    DIR_ERROR(NotMounted); DIR_ERROR(InvalidEntry); DIR_ERROR(InvalidTransition); DIR_ERROR(Full); DIR_ERROR(SequenceExhausted); DIR_ERROR(Verify);
#undef DIR_ERROR
  }
  return "Unknown";
}
bool FlightDirectory::entryValid(const FlightEntry &e) const {
  const auto status = static_cast<uint8_t>(e.status);
  return e.sequence != 0 && e.start >= FlashLayout::DataBegin && e.start < FlashLayout::DataEnd &&
      e.start % Sector == 0 && e.allocatedBytes != 0 && e.allocatedBytes % Sector == 0 &&
      e.allocatedBytes <= FlashLayout::DataEnd - e.start && e.committedBytes <= e.allocatedBytes &&
      status >= 1 && status <= 6;
}
bool FlightDirectory::readSlot(uint32_t bank, uint32_t slot, uint8_t *body, bool &erased, bool &committed) {
  uint8_t marker[Page];
  if (!flash_.read(bank + slot * SlotBytes, body, Page) ||
      !flash_.read(bank + slot * SlotBytes + Page, marker, Page)) return fail(DirectoryError::Io);
  erased = allFF(body) && allFF(marker);
  committed = get64(marker, 4) == Commit && get64(marker + 4, 4) == get64(body + 252, 4) &&
              get64(marker + 8, 4) == (uint32_t(get64(marker + 4, 4)) ^ 0xFFFFFFFFu);
  return true;
}
bool FlightDirectory::writeSlot(uint32_t bank, uint32_t slot, uint64_t generation,
                                const FlightEntry *entry, uint32_t baseline, uint64_t highWater) {
  uint8_t body[Page], marker[Page], check[Page];
  memset(body, 255, Page); memset(marker, 255, Page);
  put(body, Magic, 4); put(body + 4, 1, 2); body[6] = entry ? 1 : 0;
  put(body + 8, generation, 8); put(body + 16, slot, 4);
  if (entry) {
    put(body + 24, entry->sequence, 8); put(body + 32, entry->start, 4);
    put(body + 36, entry->committedBytes, 4); put(body + 40, entry->allocatedBytes, 4);
    body[44] = static_cast<uint8_t>(entry->status);
  } else {
    put(body + 24, highWater, 8); put(body + 32, baseline, 4);
    put(body + 40, FlashLayout::DirectoryA, 4); put(body + 44, FlashLayout::DirectoryB, 4);
    put(body + 48, FlashLayout::DirectoryBankBytes, 4); put(body + 52, FlashLayout::DataBegin, 4); put(body + 56, FlashLayout::DataEnd, 4);
  }
  const uint32_t checksum = crc(body, 252);
  put(body + 252, checksum, 4); put(marker, Commit, 4); put(marker + 4, checksum, 4);
  put(marker + 8, checksum ^ 0xFFFFFFFFu, 4);
  const uint32_t address = bank + slot * SlotBytes;
  auto verified = [&](uint32_t target,const uint8_t *source) {
    const auto error = writeVerifiedPage(flash_, target, source, check);
    if (error != PageWriteError::None) { mounted_=false; return fail(error == PageWriteError::Io ? DirectoryError::Io : DirectoryError::Verify); }
    return true;
  };
  if (!verified(address,body) || !verified(address+Page,marker)) return false;
  return true;
}
bool FlightDirectory::eraseBank(uint32_t bank) {
  for (uint32_t offset = 0; offset < FlashLayout::DirectoryBankBytes; offset += Sector)
    if (!flash_.eraseSector(bank + offset)) { mounted_ = false; return fail(DirectoryError::Io); }
  return true;
}
bool FlightDirectory::apply(const FlightEntry &e) {
  if (!entryValid(e)) return fail(DirectoryError::InvalidEntry);
  if (e.status != FlightStatus::Invalidated) {
    for (std::size_t i = 0; i < count_; ++i) {
      const auto &other = entries_[i];
      if (other.sequence == e.sequence || other.status == FlightStatus::Invalidated) continue;
      if (other.status == FlightStatus::Recording && e.status == FlightStatus::Recording)
        return fail(DirectoryError::InvalidEntry);
      if (e.start < other.start + other.allocatedBytes && other.start < e.start + e.allocatedBytes)
        return fail(DirectoryError::InvalidEntry);
    }
  }
  for (std::size_t i = 0; i < count_; ++i) {
    if (entries_[i].sequence != e.sequence) continue;
    const auto &old = entries_[i];
    // Quarantine preserves the complete owned extent and the published length.
    // It neither repairs data nor releases space, and cannot be reversed.
    if (e.status==FlightStatus::Corrupt) {
      if (old.status==FlightStatus::Invalidated || old.status==FlightStatus::Corrupt ||
          e.start!=old.start || e.allocatedBytes!=old.allocatedBytes || e.committedBytes!=old.committedBytes)
        return fail(DirectoryError::InvalidTransition);
      entries_[i]=e; return true;
    }
    if (old.start != e.start || e.allocatedBytes < old.allocatedBytes || e.committedBytes < old.committedBytes ||
        (old.status != FlightStatus::Recording && e.status != FlightStatus::Invalidated) ||
        (old.status == FlightStatus::Invalidated) ||
        ((old.status != FlightStatus::Recording || e.status == FlightStatus::Invalidated) &&
         (e.committedBytes != old.committedBytes || e.allocatedBytes != old.allocatedBytes)))
      return fail(DirectoryError::InvalidTransition);
    entries_[i] = e; return true;
  }
  if (e.sequence <= highWater_) return fail(DirectoryError::InvalidTransition);
  if (count_ == MaxEntries) {
    std::size_t dead=0; while (dead<count_ && entries_[dead].status!=FlightStatus::Invalidated) ++dead;
    if (dead==count_) return fail(DirectoryError::Full);
    for (std::size_t i=dead+1; i<count_; ++i) entries_[i-1]=entries_[i];
    --count_;
  }
  entries_[count_++] = e; highWater_ = e.sequence; return true;
}
bool FlightDirectory::mount() {
  mounted_ = false; count_ = 0; highWater_ = 0;
  uint8_t headers[2][Page]; bool valid[2] = {};
  const uint32_t banks[] = {FlashLayout::DirectoryA, FlashLayout::DirectoryB};
  bool anyBytes = false;
  for (unsigned i = 0; i < 2; ++i) {
    bool erased, committed;
    if (!readSlot(banks[i], 0, headers[i], erased, committed)) return false;
    anyBytes |= !erased;
    valid[i] = committed && headers[i][6] == 0 && get64(headers[i] + 8, 8) != 0 &&
        bodyValid(headers[i], 0, get64(headers[i] + 8, 8)) && get64(headers[i] + 32, 4) <= MaxEntries &&
        get64(headers[i] + 40, 4) == FlashLayout::DirectoryA && get64(headers[i] + 44, 4) == FlashLayout::DirectoryB &&
        get64(headers[i] + 48, 4) == FlashLayout::DirectoryBankBytes && get64(headers[i] + 52, 4) == FlashLayout::DataBegin &&
        (get64(headers[i] + 56, 4) == FlashLayout::DataEnd ||
         get64(headers[i] + 56, 4) == FlashLayout::Capacity); // Previous boundary: entries still checked against the new end.
  }
  if (!valid[0] && !valid[1]) return fail(anyBytes ? DirectoryError::Corrupt : DirectoryError::Missing);
  if (valid[0] && valid[1] && get64(headers[0] + 8, 8) == get64(headers[1] + 8, 8)) return fail(DirectoryError::Corrupt);
  const unsigned chosen = !valid[0] ? 1 : (!valid[1] ? 0 : (get64(headers[1] + 8, 8) > get64(headers[0] + 8, 8)));
  activeBank_ = banks[chosen]; generation_ = get64(headers[chosen] + 8, 8);
  const uint64_t savedHighWater = get64(headers[chosen] + 24, 8);
  const uint32_t baseline = uint32_t(get64(headers[chosen] + 32, 4));
  nextSlot_ = 1;
  if (!baseline) highWater_=savedHighWater;
  for (uint32_t slot = 1; slot < FlashLayout::DirectoryBankBytes / SlotBytes; ++slot) {
    uint8_t body[Page]; bool erased, committed;
    if (!readSlot(activeBank_, slot, body, erased, committed)) return false;
    if (!erased) nextSlot_ = slot + 1; // Never program a torn slot again.
    if (slot <= baseline && !committed) return fail(DirectoryError::Corrupt);
    if (!committed) continue;
    if (!bodyValid(body, slot, generation_) || body[6] != 1 || !apply(decode(body))) return fail(DirectoryError::Corrupt);
    if (slot == baseline && highWater_ > savedHighWater) return fail(DirectoryError::Corrupt);
    if (slot == baseline) highWater_=savedHighWater;
  }
  if (highWater_ < savedHighWater) highWater_ = savedHighWater;
  mounted_ = true; error_ = DirectoryError::None; return true;
}
bool FlightDirectory::format() {
  mounted_ = false;
  if (!eraseBank(FlashLayout::DirectoryA) || !eraseBank(FlashLayout::DirectoryB)) return false;
  if (!writeSlot(FlashLayout::DirectoryA, 0, 1, nullptr, 0, 0)) return false;
  return mount();
}
bool FlightDirectory::compact() {
  if (!mounted_) return fail(DirectoryError::NotMounted);
  if (generation_ == std::numeric_limits<uint64_t>::max()) return fail(DirectoryError::SequenceExhausted);
  const uint32_t target = activeBank_ == FlashLayout::DirectoryA ? FlashLayout::DirectoryB : FlashLayout::DirectoryA;
  if (!eraseBank(target)) return false;
  // Only explicitly invalidated identities disappear; highWater prevents reuse.
  uint32_t retained=0;
  for (std::size_t i = 0; i < count_; ++i)
    if (entries_[i].status!=FlightStatus::Invalidated && !writeSlot(target, ++retained, generation_ + 1, &entries_[i], 0, 0)) return false;
  if (!writeSlot(target, 0, generation_ + 1, nullptr, retained, highWater_)) return false;
  return mount(); // Old bank remains intact until a later compaction reuses it.
}
bool FlightDirectory::update(const FlightEntry &e) {
  if (!mounted_) return fail(DirectoryError::NotMounted);
  bool exists = false;
  for (std::size_t i = 0; i < count_; ++i) exists |= entries_[i].sequence == e.sequence;
  if (!exists && e.status != FlightStatus::Recording) return fail(DirectoryError::InvalidTransition);
  // Validate against a small RAM snapshot before touching persistent bytes.
  const auto oldEntries = entries_; const auto oldCount = count_; const auto oldHighWater = highWater_;
  const bool valid = apply(e);
  entries_ = oldEntries; count_ = oldCount; highWater_ = oldHighWater;
  if (!valid) return false;
  // Keep two slots in reserve; compact before normal appends consume them.
  if (nextSlot_ + 2 >= FlashLayout::DirectoryBankBytes / SlotBytes && !compact()) return false;
  if (!writeSlot(activeBank_, nextSlot_, generation_, &e, 0, 0)) return false;
  ++nextSlot_;
  if (!apply(e)) { mounted_ = false; return fail(DirectoryError::Corrupt); }
  error_ = DirectoryError::None; return true;
}
} // namespace storage
