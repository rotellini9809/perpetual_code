#pragma once
#include <cstddef>
#include <cstdint>

namespace storage {
namespace binary {
// Caller validates buffer bounds. Width is 0..8 bytes (0..4 for get32).
// No native struct layout, alignment or host-endian assumptions.
inline void put(uint8_t *bytes, uint64_t value, unsigned width) {
  for (unsigned i = 0; i < width; ++i) { bytes[i] = uint8_t(value); value >>= 8; }
}
inline uint64_t get64(const uint8_t *bytes, unsigned width) {
  uint64_t value = 0;
  for (unsigned i = 0; i < width; ++i) value |= uint64_t(bytes[i]) << (8*i);
  return value;
}
inline uint32_t get32(const uint8_t *bytes, unsigned width) {
  return uint32_t(get64(bytes, width));
}
// CRC-32/ISO-HDLC: reflected polynomial EDB88320, initial/final XOR FFFFFFFF.
// extendCrc consumes/returns the unfinalized state, allowing disjoint regions.
constexpr uint32_t CrcInitial = 0xFFFFFFFFu;
inline uint32_t extendCrc(uint32_t state, const uint8_t *bytes, std::size_t length) {
  for (std::size_t i = 0; i < length; ++i) {
    state ^= bytes[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      state = (state >> 1) ^ ((state & 1) ? 0xEDB88320u : 0);
  }
  return state;
}
inline uint32_t crc(const uint8_t *bytes, std::size_t length) {
  return extendCrc(CrcInitial, bytes, length) ^ CrcInitial;
}
}
}
