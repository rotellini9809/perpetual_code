#include "simulation/SimulatedSensorSource.h"
#include <cmath>

namespace sensors {
namespace {
struct LoggedSample {
  uint32_t timeMs;
  float accel[3], gyro[3], pressurePa, temperatureC;
};
#include "data/HoppinoData.inc"
}
uint32_t SimulatedSensorSource::periodUs() { return NominalPeriodUs; }
uint32_t SimulatedSensorSource::durationMs() { return DurationMs; }
uint32_t SimulatedSensorSource::sampleCount() { return sizeof(HoppinoSamples)/sizeof(HoppinoSamples[0]); }
float SimulatedSensorSource::nominalHz() { return NominalHz; }
float SimulatedSensorSource::referenceAltitude() { return ReferenceAltitude; }
void SimulatedSensorSource::start() {
  origin_ = clock_ ? clock_() : 0; next_ = missed_ = maxLate_ = baroSequence_ = 0;
  running_ = clock_ != nullptr; finished_ = false;
}
bool SimulatedSensorSource::poll(Reading &reading) {
  if (!running_) return false;
  const uint32_t elapsed = uint32_t(clock_() - origin_);
  const uint32_t dueUs = HoppinoSamples[next_].timeMs * 1000;
  if (elapsed < dueUs) return false;
  const auto late = elapsed - dueUs;
  if (late > maxLate_) maxLate_ = late;
  // Skip stale frames on overrun; never feed a burst of historical IMU readings.
  uint32_t index = next_;
  while (index+1 < sampleCount() && HoppinoSamples[index+1].timeMs*1000 <= elapsed) ++index;
  missed_ += index-next_; next_ = index+1;
  const auto &sample = HoppinoSamples[index];
  reading = {}; reading.timeUs = uint64_t(sample.timeMs)*1000;
  reading.imuSequence = uint64_t(index)+1; reading.imuValid = true;
  for (unsigned i=0;i<3;++i) {
    reading.accelMps2[i]=sample.accel[i]; reading.gyroRadps[i]=sample.gyro[i];
    reading.imuValid = reading.imuValid && std::isfinite(sample.accel[i]) && std::isfinite(sample.gyro[i]);
  }
  // The log has no barometer acquisition IDs. Match sensor HIL: each finite
  // logged pressure is fresh; missing pressure never becomes zero.
  reading.pressurePa=sample.pressurePa; reading.baroValid=std::isfinite(sample.pressurePa);
  if (reading.baroValid) ++baroSequence_;
  reading.baroSequence=baroSequence_; reading.baroTimeUs=reading.timeUs;
  reading.temperatureC=sample.temperatureC;
  if (next_==sampleCount()) { running_=false; finished_=true; }
  return true;
}
}
