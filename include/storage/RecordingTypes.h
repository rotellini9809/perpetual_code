#pragma once
#include "configuration/RecoveryConfiguration.h"
#include "control/AirbrakeStatus.h"
#include "hardware/SensorSource.h"
#include "shared/FlightTypes.h"
#include <array>
#include <limits>

namespace storage {
// Persisted directory values. Corrupt retains ownership until explicit deletion.
enum class FlightStatus : uint8_t { Recording = 1, Completed, Interrupted, StoppedFull, Invalidated, Corrupt = 6 };
struct FlightEntry {
  uint64_t sequence = 0;
  uint32_t start = 0;
  uint32_t committedBytes = 0;
  uint32_t allocatedBytes = 0; // Contiguous erase-aligned extent, including padding.
  FlightStatus status = FlightStatus::Recording;
};
// Matches the legacy CSV field order without magnetometer fields. No output events.
struct FlightSample {
  uint32_t timeMs = 0;
  control::AirbrakeStatus airbrakes;
  std::array<float, 14> values; // See Field for explicit units and ordering.
  recovery::FlightPhase phase = recovery::FlightPhase::Pad;
  recovery::AbortReason abortReason = recovery::AbortReason::None;
  uint16_t batteryMv = UINT16_MAX; // Latest 5 Hz board measurement; FFFF = unavailable.
  enum Field : std::size_t {
    AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, PressureHpa, TemperatureC,
    AltitudeM, VelocityMps, RollDegrees, PitchDegrees, YawDegrees, VerticalAccelMps2
  };
  FlightSample() { values.fill(std::numeric_limits<float>::quiet_NaN()); }
};
enum class SensorMode : uint8_t { Hardware = 0, Simulation = 1 };
enum class OutputMode : uint8_t { Physical = 0, Virtual = 1 };
// Immutable per-flight snapshot, captured before recording begins.
struct FlightCalibration {
  recovery::StateMachineSettings machine;
  uint32_t airbrakeLawVersion=0;
  bool airbrakesEnabled=false;
  bool available = false, sensorAxes = false, gyroCalibrated = false;
  uint8_t upAxis = 0;
  float referenceAltitude = 0, gyroBias[3] = {}, accelBias[3] = {};
  float accelScale[3] = {1,1,1};
  uint32_t accelCalibrationRevision=0;
};

enum class StartState : uint8_t { Empty, Pending, Recorded, Invalid };
struct OutputStartSlot {
  recovery::RuleId rule = 0;
  recovery::OutputKind kind = recovery::OutputKind::Pyro;
  uint8_t channel = 0;
  StartState state = StartState::Empty;
  uint32_t timeMs = 0; // Valid only for Pending/Recorded; zero is a valid timestamp.
};
enum class RecorderError : uint8_t {
  None, NotInitialized, Busy, NotRecording, InvalidArgument, Bounds, NoSpace,
  DirectoryFull, SequenceExhausted, Directory, Header, Sample, Io, Corrupt,
  Full, Backpressure, ProducerActive, StaleSelection, EmptyRecording, NotBlank, Summary
};
inline const char *recorderErrorName(RecorderError error) {
  switch (error) {
#define RECORD_ERROR(name) case RecorderError::name: return #name
    RECORD_ERROR(None); RECORD_ERROR(NotInitialized); RECORD_ERROR(Busy); RECORD_ERROR(NotRecording);
    RECORD_ERROR(InvalidArgument); RECORD_ERROR(Bounds); RECORD_ERROR(NoSpace); RECORD_ERROR(DirectoryFull);
    RECORD_ERROR(SequenceExhausted); RECORD_ERROR(Directory); RECORD_ERROR(Header); RECORD_ERROR(Sample);
    RECORD_ERROR(Io); RECORD_ERROR(Corrupt); RECORD_ERROR(Full);
    RECORD_ERROR(Backpressure); RECORD_ERROR(ProducerActive); RECORD_ERROR(StaleSelection);
    RECORD_ERROR(EmptyRecording); RECORD_ERROR(NotBlank);
    RECORD_ERROR(Summary);
#undef RECORD_ERROR
  }
  return "Unknown";
}

enum class RecordingEnd : uint8_t { Completed = 1, Stopped, Disconnected, StorageFull };
// Close-time instruction only; never serialized into a recording header/summary.
enum class RecordingRetention : uint8_t { Keep, Discard };
enum class SummaryState : uint8_t { Missing, Incomplete, Valid, Corrupt, Unsupported };
struct RecordingSummary {
  uint32_t savedSamples = 0, droppedSamples = 0, queuePeak = 0, queueCapacity = 0;
  RecordingEnd reason = RecordingEnd::Completed;
};
inline const char *recordingEndName(RecordingEnd e) {
  switch (e) {
    case RecordingEnd::Completed: return "Completed";
    case RecordingEnd::Stopped: return "Stopped";
    case RecordingEnd::Disconnected: return "Disconnected";
    case RecordingEnd::StorageFull: return "StorageFull";
  }
  return "Unknown";
}
inline const char *summaryStateName(SummaryState s) {
  switch (s) {
    case SummaryState::Missing: return "Missing";
    case SummaryState::Incomplete: return "Incomplete"; case SummaryState::Valid: return "Valid";
    case SummaryState::Corrupt: return "Corrupt"; case SummaryState::Unsupported: return "Unsupported";
  }
  return "Unknown";
}
}
