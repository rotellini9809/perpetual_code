#pragma once
#include "storage/FlightLogCodec.h"
#include "storage/BinaryEncoding.h"
#include <array>

// Reference decoder for flight log v4 bodies. Host tests use it to check the
// encoder and to produce cross-check vectors for the PC decoder; firmware never
// decodes records (recovery uses only the block header).
namespace storage {
namespace flightlog {
struct DecodedFrame {
  uint64_t timeUs = 0;
  uint32_t sequence = 0, gapBefore = 0;
  bool deliberateGap = false, history = false, imuValid = true, estimates = false, baroFresh = false;
  int16_t imu[6] = {};
  int32_t altitudeCm = 0;
  int16_t velocity = 0, verticalAccel = 0, angles[3] = {};
  uint16_t baroAgeUs = 0;
  uint32_t pressure64 = 0; // 1/64 Pa
  int16_t temperatureCenti = 0;
  LogState state;
};
struct DecodedBlock {
  uint64_t sequence = 0, firstTimeUs = 0;
  uint32_t index = 0, address = 0, firstSequence = 0;
  uint16_t frames = 0, used = 0;
  LogState start;
  std::array<DecodedFrame, MaxFrames> frame;
};

// Validates the complete body (header, CRC, record syntax, FF padding) and decodes it.
inline bool decodeBody(const uint8_t *body, DecodedBlock &out) {
  using binary::get64;
  const auto u = [&](unsigned at, unsigned width) { return get64(body + at, width); };
  if (u(0, 4) != Magic || u(4, 2) != Version || u(6, 2) != BodyBytes) return false;
  if (u(CrcOffset, 4) != binary::crc(body, CrcOffset)) return false;
  DecodedBlock b;
  b.sequence = u(8, 8); b.index = uint32_t(u(16, 4)); b.address = uint32_t(u(20, 4));
  b.firstTimeUs = u(24, 8); b.firstSequence = uint32_t(u(32, 4));
  const auto frames = uint16_t(u(36, 2)); b.used = uint16_t(u(38, 2));
  if (!frames || frames > MaxFrames || !b.used || b.used > PayloadBytes) return false;
  for (unsigned i = 56; i < PayloadOffset; ++i) if (body[i] != 0xFF) return false;
  for (unsigned i = PayloadOffset + b.used; i < CrcOffset; ++i) if (body[i] != 0xFF) return false;
  b.start.phase = body[40]; b.start.abortReason = body[41]; b.start.batteryMv = uint16_t(u(42, 2));
  b.start.airbrakes.flags = uint16_t(u(44, 2)); b.start.airbrakes.failedConditions = body[46];
  b.start.airbrakes.channelMask = body[47];
  for (unsigned i = 0; i < 4; ++i) b.start.airbrakes.opening[i] = uint16_t(u(48 + 2 * i, 2));
  if (b.start.phase > 7 || b.start.abortReason > 2 || b.start.airbrakes.channelMask > 15) return false;

  LogState state = b.start;
  DecodedFrame pending; bool haveTime = false; uint64_t timeDelta = 0;
  uint64_t previousTime = 0; uint32_t previousSequence = 0;
  const uint8_t *p = body + PayloadOffset, *end = p + b.used;
  const auto need = [&](unsigned n) { return unsigned(end - p) >= n; };
  bool dangling = false; // Auxiliary records seen since the last FRAME.
  while (p < end) {
    const uint8_t tag = *p++, type = tag >> 5, flags = tag & 31;
    dangling = type != Frame;
    switch (type) {
      case Status:
        if (flags || !need(2) || p[0] > 7 || p[1] > 2) return false;
        state.phase = p[0]; state.abortReason = p[1]; p += 2; break;
      case Battery:
        if (flags || !need(2)) return false;
        state.batteryMv = uint16_t(get64(p, 2)); p += 2; break;
      case Airbrake: {
        if (flags || !need(4)) return false;
        control::AirbrakeStatus a; a.flags = uint16_t(get64(p, 2)); a.failedConditions = p[2]; a.channelMask = p[3]; p += 4;
        if (a.channelMask > 15 || (a.flags & ~511u)) return false;
        for (unsigned i = 0; i < 4; ++i)
          if (a.channelMask & (1u << i)) { if (!need(2)) return false; a.opening[i] = uint16_t(get64(p, 2)); p += 2; }
        state.airbrakes = a; break;
      }
      case Gap:
        if ((flags & ~GapDeliberate) || !need(2)) return false;
        pending.gapBefore += uint32_t(get64(p, 2)); pending.deliberateGap = flags & GapDeliberate; p += 2; break;
      case Time:
        if (flags || !need(4) || haveTime) return false;
        timeDelta = get64(p, 4); haveTime = true; p += 4; break;
      case Baro:
        if (flags || !need(7) || pending.baroFresh) return false;
        pending.baroFresh = true; pending.baroAgeUs = uint16_t(get64(p, 2));
        pending.pressure64 = uint32_t(get64(p + 2, 3)); pending.temperatureCenti = int16_t(get64(p + 5, 2)); p += 7; break;
      case Frame: {
        if ((flags & ~(FrameEstimates | FrameHistory | FrameImuInvalid)) || !need(14)) return false;
        DecodedFrame f = pending; pending = DecodedFrame{};
        f.estimates = flags & FrameEstimates; f.history = flags & FrameHistory; f.imuValid = !(flags & FrameImuInvalid);
        const auto dt = uint16_t(get64(p, 2)); p += 2;
        for (unsigned i = 0; i < 6; ++i) { f.imu[i] = int16_t(get64(p, 2)); p += 2; }
        if (f.estimates) {
          if (!need(13)) return false;
          const uint32_t raw = uint32_t(get64(p, 3)); f.altitudeCm = int32_t(raw << 8) >> 8; p += 3;
          f.velocity = int16_t(get64(p, 2)); f.verticalAccel = int16_t(get64(p + 2, 2)); p += 4;
          for (auto &angle : f.angles) { angle = int16_t(get64(p, 2)); p += 2; }
        }
        if (b.frames == 0) {
          if (dt || haveTime) return false;
          f.timeUs = b.firstTimeUs; f.sequence = b.firstSequence;
        } else {
          if ((dt == DtFromTime) != haveTime) return false;
          const uint64_t delta = haveTime ? timeDelta : dt;
          if (!delta) return false;
          f.timeUs = previousTime + delta; f.sequence = previousSequence + 1 + f.gapBefore;
        }
        haveTime = false; f.state = state;
        previousTime = f.timeUs; previousSequence = f.sequence;
        if (b.frames == MaxFrames) return false;
        b.frame[b.frames++] = f;
        break;
      }
      default: return false;
    }
  }
  // Records after the last FRAME would belong to no sample.
  if (b.frames != frames || dangling) return false;
  out = b; return true;
}
}
}
