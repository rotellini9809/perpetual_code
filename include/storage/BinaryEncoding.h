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
// Half-byte table: two lookups per byte (~4x the bit loop), constant data shared by
// both cores without runtime initialization.
inline uint32_t extendCrc(uint32_t state, const uint8_t *bytes, std::size_t length) {
  static const uint32_t nibble[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu, 0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu, 0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu};
  for (std::size_t i = 0; i < length; ++i) {
    state ^= bytes[i];
    state = (state >> 4) ^ nibble[state & 15u];
    state = (state >> 4) ^ nibble[state & 15u];
  }
  return state;
}
inline uint32_t crc(const uint8_t *bytes, std::size_t length) {
  return extendCrc(CrcInitial, bytes, length) ^ CrcInitial;
}
}
}
