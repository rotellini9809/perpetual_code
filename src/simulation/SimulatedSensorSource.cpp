#include "simulation/SimulatedSensorSource.h"
#include <cmath>
#include <algorithm>

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
  if (bistage_) {
    const uint32_t dueUs=next_*5000u;
    if (elapsed<dueUs) return false;
    const uint32_t index=std::min<uint32_t>(elapsed/5000u,5000u);
    const uint32_t late=elapsed-dueUs; if (late>maxLate_) maxLate_=late;
    missed_+=index-next_; next_=index+1;
    const float seconds=float(index)*.005f;
    float height=0,velocity=0,force=9.80665f;
    if (seconds>=5) {
      const float first=std::min(seconds-5,1.0f); height=12.5f*first*first; velocity=25*first; force=34.80665f;
      if (seconds>=6) {
        const float coast=std::min(seconds-6,1.0f); height+=25*coast-4.903325f*coast*coast; velocity=25-9.80665f*coast; force=0;
      }
      if (seconds>=7) {
        const float second=std::min(seconds-7,1.0f); height+=15.19335f*second+12.5f*second*second; velocity=15.19335f+25*second; force=34.80665f;
      }
      if (seconds>=8) {
        const float coast=seconds-8; height+=40.19335f*coast-4.903325f*coast*coast; velocity=40.19335f-9.80665f*coast; force=0;
      }
      if (height<=0 && velocity<0) { height=0; force=9.80665f; }
    }
    reading={}; reading.timeUs=uint64_t(index)*5000; reading.imuSequence=uint64_t(index)+1;
    reading.imuValid=reading.baroValid=true; reading.accelMps2[0]=force;
    reading.pressurePa=101325.0f*std::pow(1.0f-height/44330.0f,1.0f/.1903f);
    reading.baroSequence=reading.imuSequence; reading.baroTimeUs=reading.timeUs; reading.temperatureC=20;
    if (index==5000) { running_=false; finished_=true; }
    return true;
  }
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
