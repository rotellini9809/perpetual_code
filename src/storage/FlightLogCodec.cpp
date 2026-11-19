#include "storage/FlightLogCodec.h"
#include "storage/BinaryEncoding.h"
#include <cmath>
#include <cstring>

namespace storage {
namespace flightlog {
namespace {
using binary::put;
int64_t quantize(double value, double unit, int64_t low, int64_t high) {
  const double scaled = std::round(value / unit);
  return scaled < double(low) ? low : scaled > double(high) ? high : int64_t(scaled);
}
// Estimate fields reserve their minimum value for "invalid".
int64_t estimate(float value, double unit, unsigned bits) {
  const int64_t high = (int64_t(1) << (bits - 1)) - 1;
  return std::isfinite(value) ? quantize(value, unit, -high, high) : -high - 1;
}
}

void Encoder::reset() {
  used_ = frames_ = 0; firstTimeUs_ = previousTimeUs_ = 0; firstSequence_ = previousSequence_ = 0;
  havePrevious_ = false; blockStart_ = current_ = LogState{};
}

unsigned Encoder::group(const FlightSample &s, bool history, bool deliberate, uint8_t *out) const {
  unsigned n = 0;
  const auto phase = uint8_t(s.phase), abort = uint8_t(s.abortReason);
  if (phase != current_.phase || abort != current_.abortReason) {
    out[n++] = uint8_t(Status << 5); out[n++] = phase; out[n++] = abort;
  }
  if (s.batteryMv != current_.batteryMv) { out[n++] = uint8_t(Battery << 5); put(out + n, s.batteryMv, 2); n += 2; }
  if (!sameAirbrakes(s.airbrakes, current_.airbrakes)) {
    out[n++] = uint8_t(Airbrake << 5); put(out + n, s.airbrakes.flags, 2); n += 2;
    out[n++] = s.airbrakes.failedConditions; out[n++] = s.airbrakes.channelMask;
    for (unsigned i = 0; i < 4; ++i)
      if (s.airbrakes.channelMask & (1u << i)) { put(out + n, s.airbrakes.opening[i], 2); n += 2; }
  }
  if (havePrevious_ && s.imuSequence != previousSequence_ + 1) {
    const uint32_t lost = s.imuSequence - previousSequence_ - 1;
    out[n++] = uint8_t(Gap << 5 | (deliberate ? GapDeliberate : 0)); put(out + n, lost > 0xFFFF ? 0xFFFF : lost, 2); n += 2;
  }
  uint16_t dt = 0;
  if (frames_) {
    const uint64_t delta = s.timeUs - previousTimeUs_;
    if (delta >= DtFromTime) { out[n++] = TimeTag; put(out + n, delta, 4); n += 4; dt = DtFromTime; }
    else dt = uint16_t(delta);
  }
  const float pressure = s.values[FlightSample::PressurePa];
  if (s.baroFresh && std::isfinite(pressure) && pressure > 0) {
    out[n++] = uint8_t(Baro << 5); put(out + n, s.baroAgeUs, 2); n += 2;
    put(out + n, uint64_t(quantize(pressure, 1.0 / 64, 0, 0xFFFFFE)), 3); n += 3;
    put(out + n, uint16_t(int16_t(estimate(s.values[FlightSample::TemperatureC], 0.01, 16))), 2); n += 2;
  }
  bool estimates = false;
  for (auto field : {FlightSample::AltitudeM, FlightSample::VelocityMps, FlightSample::VerticalAccelMps2,
                     FlightSample::RollDegrees, FlightSample::PitchDegrees, FlightSample::YawDegrees})
    estimates = estimates || std::isfinite(s.values[field]);
  bool imuValid = s.imuValid;
  for (unsigned i = 0; i < 6; ++i) imuValid = imuValid && std::isfinite(s.values[FlightSample::AccelX + i]);
  out[n++] = uint8_t(Frame << 5 | (estimates ? FrameEstimates : 0) | (history ? FrameHistory : 0) | (imuValid ? 0 : FrameImuInvalid));
  put(out + n, dt, 2); n += 2;
  for (unsigned i = 0; i < 6; ++i) {
    const double unit = i < 3 ? AccelMps2PerCode : GyroRadpsPerCode;
    const int64_t code = imuValid ? quantize(s.values[FlightSample::AccelX + i], unit, -32768, 32767) : 0;
    put(out + n, uint16_t(int16_t(code)), 2); n += 2;
  }
  if (estimates) {
    put(out + n, uint32_t(int32_t(estimate(s.values[FlightSample::AltitudeM], 0.01, 24))), 3); n += 3;
    put(out + n, uint16_t(int16_t(estimate(s.values[FlightSample::VelocityMps], 0.02, 16))), 2); n += 2;
    put(out + n, uint16_t(int16_t(estimate(s.values[FlightSample::VerticalAccelMps2], 0.01, 16))), 2); n += 2;
    for (auto field : {FlightSample::RollDegrees, FlightSample::PitchDegrees, FlightSample::YawDegrees}) {
      put(out + n, uint16_t(int16_t(estimate(s.values[field], 0.01, 16))), 2); n += 2;
    }
  }
  return n;
}

Encoder::Result Encoder::add(const FlightSample &s, bool history, bool deliberate) {
  if (havePrevious_ && (s.imuSequence <= previousSequence_ || s.timeUs <= previousTimeUs_)) return Result::Invalid;
  if (uint8_t(s.phase) > uint8_t(recovery::FlightPhase::Abort) || uint8_t(s.abortReason) > uint8_t(recovery::AbortReason::Tilt) ||
      s.airbrakes.channelMask > 15 || (s.airbrakes.flags & ~511u)) return Result::Invalid;
  // Mid-block references must fit the 16-bit GAP and 32-bit TIME fields; a new
  // block starts from absolute values in its header instead.
  if (frames_ && (s.imuSequence - previousSequence_ - 1 > 0xFFFF || s.timeUs - previousTimeUs_ > 0xFFFFFFFFu)) return Result::Full;
  uint8_t bytes[MaxGroupBytes];
  const unsigned n = group(s, history, deliberate, bytes);
  if (frames_ == MaxFrames || used_ + n > PayloadBytes) return Result::Full;
  if (!frames_) { blockStart_ = current_; firstTimeUs_ = s.timeUs; firstSequence_ = s.imuSequence; }
  memcpy(payload_ + used_, bytes, n); used_ = uint16_t(used_ + n); ++frames_;
  current_.phase = uint8_t(s.phase); current_.abortReason = uint8_t(s.abortReason);
  current_.batteryMv = s.batteryMv; current_.airbrakes = s.airbrakes;
  previousTimeUs_ = s.timeUs; previousSequence_ = s.imuSequence; havePrevious_ = true;
  return Result::Ok;
}

void Encoder::seal(uint8_t (&body)[BodyBytes], uint64_t sequence, uint32_t index, uint32_t address) {
  memset(body, 0xFF, sizeof(body));
  put(body, Magic, 4); put(body + 4, Version, 2); put(body + 6, BodyBytes, 2);
  put(body + 8, sequence, 8); put(body + 16, index, 4); put(body + 20, address, 4);
  put(body + 24, firstTimeUs_, 8); put(body + 32, firstSequence_, 4); put(body + 36, frames_, 2); put(body + 38, used_, 2);
  body[40] = blockStart_.phase; body[41] = blockStart_.abortReason; put(body + 42, blockStart_.batteryMv, 2);
  put(body + 44, blockStart_.airbrakes.flags, 2); body[46] = blockStart_.airbrakes.failedConditions;
  body[47] = blockStart_.airbrakes.channelMask;
  for (unsigned i = 0; i < 4; ++i) put(body + 48 + 2 * i, blockStart_.airbrakes.opening[i], 2);
  memcpy(body + PayloadOffset, payload_, used_);
  put(body + CrcOffset, binary::crc(body, CrcOffset), 4);
  used_ = frames_ = 0;
}
}
}
