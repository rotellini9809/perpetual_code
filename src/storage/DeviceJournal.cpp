#include "storage/DeviceJournal.h"
#include "storage/BinaryEncoding.h"
#include "storage/VerifiedPage.h"
#include <cstring>

namespace storage {
namespace {
using binary::put;
using binary::get64;
using binary::crc;
constexpr uint32_t Page = FlashLayout::PageBytes;
constexpr uint32_t BankMagic = 0x4A564450, RecordMagic = 0x52564450, CommitMagic = 0x43564450;
constexpr uint16_t Version = 1;
constexpr uint8_t TypeVersion = 1;
bool erased(const uint8_t *p) { for (uint32_t i = 0; i < Page; ++i) if (p[i] != 0xFF) return false; return true; }
uint32_t pagesFor(uint32_t length) { return 2 + (length + Page - 1) / Page; } // Header + payload + commit.
void commitPage(uint8_t *p, uint32_t checksum) {
  memset(p, 0xFF, Page); put(p, CommitMagic, 4); put(p + 4, checksum, 4); put(p + 8, ~checksum, 4);
}
bool committed(const uint8_t *p, uint32_t checksum) {
  if (get64(p, 4) != CommitMagic || get64(p + 4, 4) != checksum || get64(p + 8, 4) != uint32_t(~checksum)) return false;
  for (uint32_t i = 12; i < Page; ++i) if (p[i] != 0xFF) return false;
  return true;
}
}

const char *DeviceJournal::errorName(JournalError e) {
  switch (e) {
    case JournalError::None: return "None";
    case JournalError::Io: return "Io";
    case JournalError::Corrupt: return "Corrupt";
    case JournalError::Full: return "Full";
    case JournalError::InvalidArgument: return "InvalidArgument";
  }
  return "Unknown";
}

bool DeviceJournal::scan() {
  haveBank_ = false; appendPage_ = 2; generation_ = 0; nextSequence_ = 1;
  for (auto &latest : latest_) latest = Latest{};
  const uint32_t banks[] = {FlashLayout::JournalA, FlashLayout::JournalB};
  uint64_t generations[2] = {}, sequences[2] = {};
  for (unsigned i = 0; i < 2; ++i) {
    if (!flash_.read(banks[i], page_, Page) || !flash_.read(banks[i] + Page, check_, Page)) return fail(JournalError::Io);
    const uint32_t checksum = uint32_t(get64(page_ + 252, 4));
    if (get64(page_, 4) == BankMagic && get64(page_ + 4, 2) == Version && crc(page_, 252) == checksum &&
        committed(check_, checksum) && get64(page_ + 8, 8)) {
      generations[i] = get64(page_ + 8, 8); sequences[i] = get64(page_ + 16, 8);
    }
  }
  if (!generations[0] && !generations[1]) { error_ = JournalError::None; return true; } // Never written.
  if (generations[0] == generations[1]) return fail(JournalError::Corrupt);
  const unsigned active = generations[1] > generations[0] ? 1 : 0;
  haveBank_ = true; bank_ = banks[active]; generation_ = generations[active];
  nextSequence_ = sequences[active] ? sequences[active] : 1;
  uint32_t floor = 2, lastWritten = 1;
  for (uint32_t p = 2; p < BankPages;) {
    if (!flash_.read(bank_ + p * Page, page_, Page)) return fail(JournalError::Io);
    if (erased(page_)) { ++p; continue; }
    lastWritten = p;
    const uint32_t headerCrc = uint32_t(get64(page_ + 252, 4));
    const uint8_t type = page_[4];
    const uint32_t length = uint32_t(get64(page_ + 16, 4)), pages = pagesFor(length);
    if (get64(page_, 4) != RecordMagic || crc(page_, 252) != headerCrc || length > MaxPayload || p + pages > BankPages) {
      for (auto &latest : latest_) latest.damagedAfter = true; // Unknown page: be conservative for every type.
      ++p; continue;
    }
    const uint64_t sequence = get64(page_ + 8, 8);
    const uint32_t payloadCrc = uint32_t(get64(page_ + 20, 4));
    // Verify payload and commit; a torn record still owns its declared extent.
    uint32_t state = binary::CrcInitial; bool readable = true;
    for (uint32_t i = 0; i < length; i += Page) {
      if (!flash_.read(bank_ + (p + 1) * Page + i, check_, Page)) return fail(JournalError::Io);
      state = binary::extendCrc(state, check_, length - i < Page ? length - i : Page);
    }
    const bool payloadOk = (state ^ binary::CrcInitial) == payloadCrc;
    if (!flash_.read(bank_ + (p + pages - 1) * Page, check_, Page)) return fail(JournalError::Io);
    const bool commit = committed(check_, headerCrc);
    // Without its commit the write never completed (fall back); with it, the data was damaged later.
    if (!payloadOk || !commit || page_[5] != TypeVersion) readable = false;
    const bool corrupt = commit && !readable;
    if (type >= 1 && type <= TypeCount) {
      auto &latest = latest_[type];
      if (readable && (page_[6] & 4)) {
        // Placeholder copied by a compaction: no data, only the damage that had no valid copy.
        if (!latest.valid) { latest.damagedAfter = latest.damagedAfter || (page_[6] & 1); latest.corruptAfter = latest.corruptAfter || (page_[6] & 2); }
      } else if (readable && (!latest.valid || sequence > latest.sequence)) {
        // A compaction copy carries the damage that followed its original (byte 6).
        latest.valid = true; latest.damagedAfter = page_[6] & 1; latest.corruptAfter = page_[6] & 2;
        latest.page = uint16_t(p); latest.length = length; latest.sequence = sequence;
      } else if (!readable) { latest.damagedAfter = true; latest.corruptAfter = latest.corruptAfter || corrupt; }
    }
    if (sequence >= nextSequence_) nextSequence_ = sequence + 1;
    lastWritten = p + pages - 1; floor = p + pages; p += pages;
  }
  appendPage_ = lastWritten + 1 > floor ? lastWritten + 1 : floor;
  error_ = JournalError::None; return true;
}

bool DeviceJournal::readPayload(uint32_t bank, const Latest &latest, uint8_t *payload, std::size_t capacity) {
  if (latest.length > capacity) return fail(JournalError::InvalidArgument);
  if (latest.length && !flash_.read(bank + (latest.page + 1u) * Page, payload, latest.length)) return fail(JournalError::Io);
  return true;
}

bool DeviceJournal::read(Type type, uint8_t *payload, std::size_t capacity, Found &found) {
  found = Found{};
  if (type < 1 || type > TypeCount) return fail(JournalError::InvalidArgument);
  if (!scan()) return false;
  const auto &latest = latest_[type];
  found.damagedAfter = latest.damagedAfter; found.corruptAfter = latest.corruptAfter;
  if (!haveBank_ || !latest.valid) { error_ = JournalError::None; return true; }
  if (!readPayload(bank_, latest, payload, capacity)) return false;
  found.present = true; found.length = latest.length;
  error_ = JournalError::None; return true;
}

bool DeviceJournal::append(uint32_t bank, uint32_t page, Type type, uint64_t sequence, const uint8_t *payload, std::size_t length, uint8_t flags) {
  uint8_t header[Page];
  memset(header, 0xFF, Page);
  put(header, RecordMagic, 4); header[4] = type; header[5] = TypeVersion; header[6] = flags; put(header + 8, sequence, 8);
  put(header + 16, uint32_t(length), 4); put(header + 20, crc(payload, length), 4);
  const uint32_t headerCrc = crc(header, 252); put(header + 252, headerCrc, 4);
  // Header, payload, commit: each page programmed once and read back.
  if (writeVerifiedPage(flash_, bank + page * Page, header, check_) != PageWriteError::None) return fail(JournalError::Io);
  for (std::size_t offset = 0; offset < length; offset += Page) {
    memset(page_, 0xFF, Page);
    memcpy(page_, payload + offset, length - offset < Page ? length - offset : Page);
    if (writeVerifiedPage(flash_, bank + (page + 1 + offset / Page) * Page, page_, check_) != PageWriteError::None) return fail(JournalError::Io);
  }
  commitPage(page_, headerCrc);
  if (writeVerifiedPage(flash_, bank + (page + pagesFor(uint32_t(length)) - 1) * Page, page_, check_) != PageWriteError::None)
    return fail(JournalError::Io);
  return true;
}

bool DeviceJournal::commitBank(uint32_t bank, uint64_t generation, uint64_t nextSequence) {
  uint8_t header[Page];
  memset(header, 0xFF, Page);
  put(header, BankMagic, 4); put(header + 4, Version, 2); put(header + 8, generation, 8); put(header + 16, nextSequence, 8);
  const uint32_t checksum = crc(header, 252); put(header + 252, checksum, 4);
  if (writeVerifiedPage(flash_, bank, header, check_) != PageWriteError::None) return fail(JournalError::Io);
  commitPage(page_, checksum);
  if (writeVerifiedPage(flash_, bank + Page, page_, check_) != PageWriteError::None) return fail(JournalError::Io);
  return true;
}

bool DeviceJournal::initialize() {
  // No valid bank: blank device (or unreadable metadata, which cannot be recovered anyway).
  if (!flash_.eraseRange(FlashLayout::JournalA, FlashLayout::JournalBankBytes)) return fail(JournalError::Io);
  if (!commitBank(FlashLayout::JournalA, 1, nextSequence_)) return false;
  return scan();
}

bool DeviceJournal::compact(uint8_t keepMask) {
  const uint32_t source = bank_;
  const uint32_t target = source == FlashLayout::JournalA ? FlashLayout::JournalB : FlashLayout::JournalA;
  if (!flash_.eraseRange(target, FlashLayout::JournalBankBytes)) return fail(JournalError::Io);
  uint32_t page = 2;
  for (unsigned type = 1; type <= TypeCount; ++type) {
    const Latest latest = latest_[type];
    if (!(keepMask & (1u << type))) continue;
    if (!latest.valid) {
      // Damage without any valid copy must survive too: keep it as a placeholder (flag bit 2).
      if (!latest.damagedAfter && !latest.corruptAfter) continue;
      const uint8_t flags = uint8_t(4 | (latest.damagedAfter ? 1 : 0) | (latest.corruptAfter ? 2 : 0));
      if (!append(target, page, Type(type), 0, nullptr, 0, flags)) return false;
      page += pagesFor(0); continue;
    }
    if (!readPayload(source, latest, copy_, sizeof(copy_))) return false;
    const uint8_t flags = uint8_t((latest.damagedAfter ? 1 : 0) | (latest.corruptAfter ? 2 : 0));
    if (!append(target, page, Type(type), latest.sequence, copy_, latest.length, flags)) return false;
    page += pagesFor(latest.length);
  }
  // The new bank becomes active only with this commit; until then the old one stays valid.
  if (!commitBank(target, generation_ + 1, nextSequence_)) return false;
  return scan();
}

bool DeviceJournal::write(Type type, const uint8_t *payload, std::size_t length) {
  if (type < 1 || type > TypeCount || length > MaxPayload || (length && !payload)) return fail(JournalError::InvalidArgument);
  if (!scan()) return false;
  if (!haveBank_ && !initialize()) return false;
  const uint32_t pages = pagesFor(uint32_t(length));
  if (appendPage_ + pages > BankPages) {
    if (!compact(0xFF)) return false;
    if (appendPage_ + pages > BankPages) return fail(JournalError::Full);
  }
  const uint64_t sequence = nextSequence_;
  if (!append(bank_, appendPage_, type, sequence, payload, length)) return false;
  // Read back through the normal path: the new record must now be the newest of its type.
  if (!scan()) return false;
  if (!latest_[type].valid || latest_[type].sequence != sequence) return fail(JournalError::Corrupt);
  return true;
}

bool DeviceJournal::reset(uint8_t keepMask) {
  if (!scan()) return false;
  if (!haveBank_) return initialize();
  return compact(keepMask);
}
}
