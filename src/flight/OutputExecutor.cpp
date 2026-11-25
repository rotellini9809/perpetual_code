#include "flight/OutputExecutor.h"
#include "configuration/ActuatorPolicy.h"
#include <cmath>

namespace flight {
uint8_t OutputExecutor::connectedPyros(uint8_t mask) {
  uint8_t connected=0;
  for (uint8_t i=0;i<recovery::ChannelCount;++i)
    if ((mask & (1u<<i)) && hardware_.continuity(i+1)) connected |= uint8_t(1u<<i);
  return connected;
}
uint8_t OutputExecutor::preparedServos(const recovery::Configuration &config) const {
  uint8_t matching=0;
  for (unsigned i=0;i<recovery::ChannelCount;++i) {
    const auto &servo=config.servos[i];
    if (((preparedMask_ & ~previewMask_) & (1u<<i)) && servo.calibrated && servo.initialUs==preparedInitial_[i] && servo.finalUs==preparedFinal_[i])
      matching |= uint8_t(1u<<i);
  }
  return matching;
}
void OutputExecutor::releasePrepared() {
  for (uint8_t i=0;i<recovery::ChannelCount;++i) if (preparedMask_ & (1u<<i)) hardware_.releaseServo(i+1);
  preparedMask_=previewMask_=0;
}
const char *OutputExecutor::preview(uint8_t channel, uint16_t pulseUs) {
  if (owner_) return "BUSY";
  if (!clock_) return "TIMING_UNAVAILABLE";
  if (channel<1 || channel>recovery::ChannelCount) return "INVALID_ARGUMENT";
  const uint8_t bit=uint8_t(1u<<(channel-1));
  if (!hardware_.servo(channel,pulseUs)) {
    hardware_.releaseServo(channel); preparedMask_ &= uint8_t(~bit); previewMask_ &= uint8_t(~bit);
    return "OUTPUT_ERROR";
  }
  preparedMask_ |= bit; previewMask_ |= bit;
  return nullptr;
}
void OutputExecutor::releasePreview(uint8_t channel) {
  for (uint8_t i=0;i<recovery::ChannelCount;++i) {
    const uint8_t bit=uint8_t(1u<<i);
    if ((previewMask_ & bit) && (!channel || channel==i+1)) {
      hardware_.releaseServo(i+1); preparedMask_ &= uint8_t(~bit); previewMask_ &= uint8_t(~bit);
    }
  }
}
void OutputExecutor::reconcile(const recovery::Configuration *config) {
  if (owner_) return;
  uint8_t assigned=config ? config->airbrakes.channelMask : 0;
  if (config)
    for (std::size_t i=0;i<config->ruleCount;++i)
      for (std::size_t j=0;j<config->rules[i].assignmentCount;++j) {
        const auto &assignment=config->rules[i].assignments[j];
        if (assignment.kind==recovery::OutputKind::Servo) assigned |= uint8_t(1u<<(assignment.channel-1));
      }
  const uint8_t keep=uint8_t(previewMask_ | (config ? assigned & preparedServos(*config) : 0));
  for (uint8_t i=0;i<recovery::ChannelCount;++i)
    if ((preparedMask_ & ~keep) & (1u<<i)) hardware_.releaseServo(i+1);
  preparedMask_ &= keep;
}
const char *OutputExecutor::prepare(const recovery::Configuration &config, uint8_t mask) {
  if (owner_ || previewMask_) return "BUSY";
  if (!clock_) return "TIMING_UNAVAILABLE";
  for (unsigned i=0;i<recovery::ChannelCount;++i)
    if ((mask & (1u<<i)) && !config.servos[i].calibrated) return "SERVOS_NOT_CONFIGURED";
  const auto alreadyPrepared=preparedServos(config);
  for (uint8_t i=0;i<recovery::ChannelCount;++i) {
    const uint8_t bit=uint8_t(1u<<i);
    if (!(mask & bit) || (alreadyPrepared & bit)) continue;
    if (!hardware_.servo(i+1,config.servos[i].initialUs)) {
      for (uint8_t j=0;j<recovery::ChannelCount;++j)
        if ((preparedMask_ & ~alreadyPrepared) & (1u<<j)) hardware_.releaseServo(j+1);
      hardware_.releaseServo(i+1); preparedMask_=alreadyPrepared;
      return "SERVO_PREPARATION_FAILED";
    }
    preparedInitial_[i]=config.servos[i].initialUs; preparedFinal_[i]=config.servos[i].finalUs;
    preparedMask_ |= bit;
  }
  return nullptr;
}
const char *OutputExecutor::begin(const recovery::Configuration &config, uint64_t owner, bool requirePrepared, bool enableAirbrakes) {
  if (owner_ || previewMask_) return "BUSY";
  if (!clock_) return "TIMING_UNAVAILABLE";
  if (!owner) return "INVALID_ARGUMENT";
  if (config.airbrakes.channelMask>15) return "INVALID_ARGUMENT";
  const uint8_t airbrakeMask=enableAirbrakes ? config.airbrakes.channelMask : 0;
  uint8_t servos=airbrakeMask, pyros=0;
  for (std::size_t i=0;i<config.ruleCount;++i)
    for (std::size_t j=0;j<config.rules[i].assignmentCount;++j) {
      const auto &assignment=config.rules[i].assignments[j];
      if (assignment.channel<1 || assignment.channel>recovery::ChannelCount ||
          (assignment.kind!=recovery::OutputKind::Pyro && assignment.kind!=recovery::OutputKind::Servo)) return "INVALID_ARGUMENT";
      if (assignment.kind==recovery::OutputKind::Servo && (config.airbrakes.channelMask & (1u<<(assignment.channel-1)))) return "INVALID_ARGUMENT";
      (assignment.kind==recovery::OutputKind::Pyro ? pyros : servos) |= uint8_t(1u<<(assignment.channel-1));
    }
  for (unsigned i=0;i<recovery::ChannelCount;++i)
    if ((servos & (1u<<i)) && !config.servos[i].calibrated) return "SERVOS_NOT_CONFIGURED";
  if (requirePrepared && (servos & ~preparedServos(config))) return "SERVOS_NOT_PREPARED";
  if (!requirePrepared) if (const auto error=prepare(config,servos)) return error;
  owner_=owner; airbrakeMask_=airbrakeMask; allowedPyros_=pyros; servoMask_=servos;
  preparedMask_ &= uint8_t(~servos); // Transfer held PWM to the run without detach/reposition.
  requestedPyros_=requestedServos_=pendingPyros_=firingPyros_=0; pyroEnabled_=false; reportedStartCount_=0;
  hardware_.pyros(false,0);
  for (uint8_t i=0;i<recovery::ChannelCount;++i) {
    runInitialUs_[i]=config.servos[i].initialUs; runFinalUs_[i]=config.servos[i].finalUs;
  }
  runStartUs_=clock_(); // Exclude servo preparation, as the sensor source starts next.
  return nullptr;
}
void OutputExecutor::report(recovery::VirtualStart start, uint32_t now) {
  start.timeMs=uint32_t(now-runStartUs_)/1000;
  if (reportedStartCount_<reportedStarts_.size()) reportedStarts_[reportedStartCount_++]=start;
}
bool OutputExecutor::submit(const recovery::VirtualStart &start) {
  if (!owner_ || start.channel<1 || start.channel>recovery::ChannelCount) return false;
  const auto i=start.channel-1; const uint8_t bit=uint8_t(1u<<i);
  if (start.kind==recovery::OutputKind::Servo) {
    if (!(servoMask_ & bit) || (airbrakeMask_ & bit) || (requestedServos_ & bit) || !hardware_.servo(start.channel,runFinalUs_[i])) return false;
    requestedServos_ |= bit; report(start,clock_()); return true;
  }
  if (start.kind!=recovery::OutputKind::Pyro || !(allowedPyros_ & bit) || (requestedPyros_ & bit)) return false;
  requestedPyros_ |= bit; pendingPyros_ |= bit; pyroStarts_[i]=start;
  return true;
}
bool OutputExecutor::airbrakes(uint64_t owner,const std::array<float,4> &opening) {
  if (!owner_ || owner!=owner_) return false;
  for (unsigned i=0;i<4;++i) if ((airbrakeMask_&(1u<<i)) && (!std::isfinite(opening[i]) || opening[i]<0 || opening[i]>1)) return false;
  bool allWritten=true;
  for (uint8_t i=0;i<4;++i) if (airbrakeMask_&(1u<<i)) {
    const auto pulse=uint16_t(std::lround(runInitialUs_[i]+opening[i]*(int(runFinalUs_[i])-int(runInitialUs_[i]))));
    // Always attempt all channels, including after an individual write failure.
    if (!hardware_.servo(i+1,pulse)) allWritten=false;
  }
  return allWritten;
}
void OutputExecutor::inhibit(uint64_t owner) {
  if (owner_==owner) { pendingPyros_=0; allowedPyros_=0; }
}
void OutputExecutor::poll(bool allowNewStarts) {
  if (!owner_) return;
  const auto now=clock_();
  for (unsigned i=0;i<recovery::ChannelCount;++i)
    if ((firingPyros_ & (1u<<i)) && uint32_t(now-pulseStartUs_[i])>=recovery::PyroPulseMs*1000)
      firingPyros_ &= uint8_t(~(1u<<i));
  if (allowNewStarts && pendingPyros_ && !pyroEnabled_) { pyroEnabled_=true; pyroEnabledAtUs_=now; }
  if (allowNewStarts && pyroEnabled_ && uint32_t(now-pyroEnabledAtUs_)>=recovery::PyroEnableSettleMs*1000) {
    for (unsigned i=0;i<recovery::ChannelCount;++i) {
      const uint8_t bit=uint8_t(1u<<i);
      if (!(pendingPyros_ & bit)) continue;
      firingPyros_ |= bit; pendingPyros_ &= uint8_t(~bit); pulseStartUs_[i]=now;
      report(pyroStarts_[i],now);
    }
  }
  if (!pendingPyros_ && !firingPyros_) pyroEnabled_=false;
  hardware_.pyros(pyroEnabled_,firingPyros_);
}
std::size_t OutputExecutor::drain(recovery::VirtualStart *starts, std::size_t capacity) {
  const auto count=reportedStartCount_<capacity ? reportedStartCount_ : capacity;
  for (std::size_t i=0;i<count;++i) starts[i]=reportedStarts_[i];
  for (std::size_t i=count;i<reportedStartCount_;++i) reportedStarts_[i-count]=reportedStarts_[i];
  reportedStartCount_-=count; return count;
}
unsigned OutputExecutor::stop(uint64_t owner) {
  if (!owner_ || owner!=owner_) return 0;
  unsigned pending=0;
  for (unsigned i=0;i<recovery::ChannelCount;++i) if (pendingPyros_ & (1u<<i)) ++pending;
  hardware_.pyros(false,0);
  for (uint8_t i=0;i<recovery::ChannelCount;++i) if (servoMask_ & (1u<<i)) {
    if (airbrakeMask_ & (1u<<i)) {
      if (hardware_.servo(i+1,runInitialUs_[i])) {
        preparedMask_ |= uint8_t(1u<<i); preparedInitial_[i]=runInitialUs_[i]; preparedFinal_[i]=runFinalUs_[i];
      } else hardware_.releaseServo(i+1);
    } else hardware_.releaseServo(i+1);
  }
  owner_=0; airbrakeMask_=0; servoMask_=pendingPyros_=firingPyros_=0; pyroEnabled_=false;
  return pending;
}
}
