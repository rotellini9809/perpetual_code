#include "simulation/SimulatedSensorSource.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace sensors {
namespace {
struct LoggedSample {
  uint32_t timeUs; // Relative to the first sample.
  float accel[3], gyro[3], pressurePa, temperatureC;
};
#include "data/HoppinoData.inc"
constexpr uint32_t LogCount = sizeof(HoppinoSamples)/sizeof(HoppinoSamples[0]);
uint32_t logEndUs() { return HoppinoSamples[LogCount-1].timeUs; }
constexpr uint32_t StretchedUs = (SimulatedSensorSource::StressStretchToUs-SimulatedSensorSource::StressStretchFromUs)*SimulatedSensorSource::StressStretch;
uint32_t stressCount() {
  const uint64_t duration = uint64_t(SimulatedSensorSource::StressPadHoldUs)+SimulatedSensorSource::StressStretchFromUs+StretchedUs+
                            (logEndUs()-SimulatedSensorSource::StressStretchToUs);
  return uint32_t(duration*SimulatedSensorSource::StressRateHz/1000000u)+1;
}
static_assert(SimulatedSensorSource::StressPadHoldUs%SimulatedSensorSource::StressPadLoopUs==0,"PAD loop must end where the flight starts");
float mix(float a, float b, float fraction) {
  if (!std::isfinite(a)) return b;
  if (!std::isfinite(b)) return a;
  return a+(b-a)*fraction;
}
}
uint32_t SimulatedSensorSource::periodUs() { return NominalPeriodUs; }
uint32_t SimulatedSensorSource::durationMs() { return DurationMs; }
uint32_t SimulatedSensorSource::sampleCount() { return LogCount; }
float SimulatedSensorSource::nominalHz() { return NominalHz; }
float SimulatedSensorSource::referenceAltitude() { return ReferenceAltitude; }
const char *SimulatedSensorSource::profileName() const {
  switch (profile_) {
    case Profile::Bistage: return "bistage_synthetic";
    case Profile::Hoppino480: return "hoppino480";
    default: return "hoppino";
  }
}
uint32_t SimulatedSensorSource::activeDurationMs() const {
  switch (profile_) {
    case Profile::Bistage: return 25000;
    case Profile::Hoppino480: return uint32_t(stressTimeUs(stressCount()-1)/1000);
    default: return durationMs();
  }
}
uint32_t SimulatedSensorSource::activeSampleCount() const {
  switch (profile_) {
    case Profile::Bistage: return 5001;
    case Profile::Hoppino480: return stressCount();
    default: return sampleCount();
  }
}
uint32_t SimulatedSensorSource::activePeriodUs() const {
  return profile_==Profile::Hoppino480 ? (1000000u+StressRateHz/2)/StressRateHz : profile_==Profile::Bistage ? 5000u : periodUs();
}
float SimulatedSensorSource::activeNominalHz() const {
  return profile_==Profile::Hoppino480 ? float(StressRateHz) : profile_==Profile::Bistage ? 200.0f : nominalHz();
}
uint32_t SimulatedSensorSource::stressSourceTimeUs(uint64_t t) {
  if (t < StressPadHoldUs) return uint32_t(t%StressPadLoopUs);
  const uint64_t u = t-StressPadHoldUs;
  if (u < StressStretchFromUs) return uint32_t(u);
  if (u < uint64_t(StressStretchFromUs)+StretchedUs) return StressStretchFromUs+uint32_t((u-StressStretchFromUs)/StressStretch);
  const uint64_t source = StressStretchToUs+(u-StressStretchFromUs-StretchedUs);
  return source > logEndUs() ? logEndUs() : uint32_t(source);
}
void SimulatedSensorSource::start() {
  origin_ = clock_ ? clock_() : 0; next_ = missed_ = maxLate_ = baroSequence_ = cursor_ = baroSlot_ = 0;
  running_ = clock_ != nullptr; finished_ = false;
}
bool SimulatedSensorSource::poll(Reading &reading) {
  if (!running_) return false;
  const uint32_t elapsed = uint32_t(clock_() - origin_);
  if (profile_==Profile::Bistage) return pollBistage(elapsed,reading);
  if (profile_==Profile::Hoppino480) return pollStress(elapsed,reading);
  return pollLogged(elapsed,reading);
}
bool SimulatedSensorSource::pollBistage(uint32_t elapsed, Reading &reading) {
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
bool SimulatedSensorSource::pollLogged(uint32_t elapsed, Reading &reading) {
  const uint32_t dueUs = HoppinoSamples[next_].timeUs;
  if (elapsed < dueUs) return false;
  const auto late = elapsed - dueUs;
  if (late > maxLate_) maxLate_ = late;
  // Skip stale frames on overrun; never feed a burst of historical IMU readings.
  uint32_t index = next_;
  while (index+1 < LogCount && HoppinoSamples[index+1].timeUs <= elapsed) ++index;
  missed_ += index-next_; next_ = index+1;
  const auto &sample = HoppinoSamples[index];
  reading = {}; reading.timeUs = sample.timeUs;
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
  if (next_==LogCount) { running_=false; finished_=true; }
  return true;
}
bool SimulatedSensorSource::pollStress(uint32_t elapsed, Reading &reading) {
  const uint64_t dueUs = stressTimeUs(next_);
  if (elapsed < dueUs) return false;
  const uint32_t late = uint32_t(elapsed-dueUs); if (late > maxLate_) maxLate_ = late;
  // Newest frame due by now: stressTimeUs() floors, so elapsed*rate/1e6 can be one short of a
  // frame due exactly now. Start from the frame already due and only move forward.
  uint32_t index = next_;
  while (stressTimeUs(index+1) <= elapsed) ++index;
  const uint32_t count = stressCount();
  if (index >= count) index = count-1;
  missed_ += index-next_; next_ = index+1;
  const uint64_t timeUs = stressTimeUs(index);
  const uint32_t source = stressSourceTimeUs(timeUs);
  if (source < HoppinoSamples[cursor_].timeUs) cursor_ = 0; // PAD loop restarts the log.
  while (cursor_+1 < LogCount && HoppinoSamples[cursor_+1].timeUs <= source) ++cursor_;
  const auto &a = HoppinoSamples[cursor_];
  const auto &b = HoppinoSamples[cursor_+1 < LogCount ? cursor_+1 : cursor_];
  const float fraction = b.timeUs > a.timeUs ? float(source-a.timeUs)/float(b.timeUs-a.timeUs) : 0.0f;
  reading = {}; reading.timeUs = timeUs; reading.imuSequence = uint64_t(index)+1; reading.imuValid = true;
  for (unsigned i=0;i<3;++i) {
    reading.accelMps2[i]=mix(a.accel[i],b.accel[i],fraction); reading.gyroRadps[i]=mix(a.gyro[i],b.gyro[i],fraction);
    reading.imuValid = reading.imuValid && std::isfinite(reading.accelMps2[i]) && std::isfinite(reading.gyroRadps[i]);
  }
  // A 100 Hz barometer: one fresh reading per 10 ms slot, interpolated between logged pressures.
  const uint32_t slot = uint32_t(timeUs/StressBaroPeriodUs)+1;
  if (slot != baroSlot_) {
    baroSlot_ = slot;
    uint32_t before = cursor_, after = cursor_+1;
    while (before > 0 && !std::isfinite(HoppinoSamples[before].pressurePa)) --before;
    while (after < LogCount && !std::isfinite(HoppinoSamples[after].pressurePa)) ++after;
    const auto &p = HoppinoSamples[before];
    if (after < LogCount && std::isfinite(p.pressurePa) && HoppinoSamples[after].timeUs > p.timeUs) {
      const auto &q = HoppinoSamples[after];
      const float f = source <= p.timeUs ? 0.0f : float(source-p.timeUs)/float(q.timeUs-p.timeUs);
      reading.pressurePa = mix(p.pressurePa,q.pressurePa,f); reading.temperatureC = mix(p.temperatureC,q.temperatureC,f);
    } else if (std::isfinite(p.pressurePa)) { reading.pressurePa = p.pressurePa; reading.temperatureC = p.temperatureC; }
    else if (after < LogCount) { reading.pressurePa = HoppinoSamples[after].pressurePa; reading.temperatureC = HoppinoSamples[after].temperatureC; }
    reading.baroValid = std::isfinite(reading.pressurePa);
    if (reading.baroValid) ++baroSequence_;
  }
  if (!reading.baroValid) { reading.pressurePa = reading.temperatureC = std::numeric_limits<float>::quiet_NaN(); }
  reading.baroSequence = baroSequence_; reading.baroTimeUs = reading.timeUs;
  if (next_==count) { running_=false; finished_=true; }
  return true;
}
}
