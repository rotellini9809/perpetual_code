#pragma once
#include "storage/RecordingTypes.h"
#include <cstddef>
#include <cstdint>

// Flight log v4 block codec (docs/technical/STORAGE_FORMAT.md, "Samples and blocks v4").
// The firmware only encodes; FlightLogDecoder.h is the reference decoder for tests.
namespace storage {
namespace flightlog {
constexpr uint32_t Magic = 0x4B425350, CommitMagic = 0x4D435350;
constexpr uint16_t Version = 4;
constexpr uint32_t BlockBytes = 2048, BodyBytes = 1792, PayloadOffset = 64, CrcOffset = 1788;
constexpr uint32_t PayloadBytes = CrcOffset - PayloadOffset; // 1724
constexpr unsigned MaxGroupBytes = 63, MinFrameBytes = 15;
constexpr uint16_t MaxFrames = PayloadBytes / MinFrameBytes; // 114
static_assert(BlockBytes == BodyBytes + 256, "Seven body pages and one commit page");

enum Type : uint8_t { Frame = 1, Baro = 2, Status = 3, Battery = 4, Airbrake = 5, Gap = 6, Time = 7 };
enum FrameFlag : uint8_t { FrameEstimates = 1, FrameHistory = 2, FrameImuInvalid = 4 };
enum GapFlag : uint8_t { GapDeliberate = 1 };
constexpr uint8_t EndTag = 0xFF, TimeTag = uint8_t(Time << 5);
constexpr uint16_t DtFromTime = 0xFFFF;

// Code <-> SI conversions of the LSM6DSV32X driver at +/-32 g and +/-2000 dps.
// Firmware floats are code x constant, so rounding back recovers the exact code.
constexpr double AccelMps2PerCode = 0.976 * 9.80665 / 1000.0;
constexpr double GyroRadpsPerCode = 70.0 / 1000.0 * 3.14159265358979323846 / 180.0;

// State carried across records and snapshotted in every block header.
struct LogState {
  uint8_t phase = 0, abortReason = 0;
  uint16_t batteryMv = UINT16_MAX;
  control::AirbrakeStatus airbrakes;
};
inline bool sameAirbrakes(const control::AirbrakeStatus &a, const control::AirbrakeStatus &b) {
  return a.flags == b.flags && a.failedConditions == b.failedConditions && a.channelMask == b.channelMask && a.opening == b.opening;
}

// Builds one block body in RAM. add() either encodes the whole sample group or
// leaves the block untouched; seal() completes the body and starts the next block
// with the carried state. Pure RAM: no flash access.
class Encoder {
public:
  enum class Result : uint8_t { Ok, Full, Invalid };
  void reset(); // New recording: no previous sample, default state.
  // history: written from the pre-launch buffer; deliberate: samples skipped on purpose since
  // the previous stored one (pre-launch history, 10 Hz LANDED), not lost.
  Result add(const FlightSample &sample, bool history, bool deliberate);
  bool empty() const { return frames_ == 0; }
  uint16_t frames() const { return frames_; }
  void seal(uint8_t (&body)[BodyBytes], uint64_t sequence, uint32_t index, uint32_t address);
private:
  unsigned group(const FlightSample &sample, bool history, bool deliberate, uint8_t *out) const;
  uint8_t payload_[PayloadBytes] = {};
  uint16_t used_ = 0, frames_ = 0;
  uint64_t firstTimeUs_ = 0, previousTimeUs_ = 0;
  uint32_t firstSequence_ = 0, previousSequence_ = 0;
  bool havePrevious_ = false;
  LogState blockStart_, current_;
};
}
}
