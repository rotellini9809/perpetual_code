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
  uint8_t valid=0;
  for (unsigned i=0;i<recovery::ChannelCount;++i) {
    const auto &s=config.servos[i];
    if (((preparedMask_ & ~previewMask_) & (1u<<i)) && s.calibrated && s.initialUs==preparedInitial_[i] && s.finalUs==preparedFinal_[i])
      valid |= uint8_t(1u<<i);
  }
  return valid;
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
        const auto &a=config->rules[i].assignments[j];
        if (a.kind==recovery::OutputKind::Servo) assigned |= uint8_t(1u<<(a.channel-1));
      }
  const uint8_t valid=uint8_t(previewMask_ | (config ? assigned & preparedServos(*config) : 0));
  for (uint8_t i=0;i<recovery::ChannelCount;++i)
    if ((preparedMask_ & ~valid) & (1u<<i)) hardware_.releaseServo(i+1);
  preparedMask_ &= valid;
}
const char *OutputExecutor::prepare(const recovery::Configuration &config, uint8_t mask) {
  if (owner_ || previewMask_) return "BUSY";
  if (!clock_) return "TIMING_UNAVAILABLE";
  for (unsigned i=0;i<recovery::ChannelCount;++i)
    if ((mask & (1u<<i)) && !config.servos[i].calibrated) return "SERVOS_NOT_CONFIGURED";
  const auto original=preparedServos(config);
  for (uint8_t i=0;i<recovery::ChannelCount;++i) {
    const uint8_t bit=uint8_t(1u<<i);
    if (!(mask & bit) || (original & bit)) continue;
    if (!hardware_.servo(i+1,config.servos[i].initialUs)) {
      for (uint8_t j=0;j<recovery::ChannelCount;++j)
        if ((preparedMask_ & ~original) & (1u<<j)) hardware_.releaseServo(j+1);
      hardware_.releaseServo(i+1); preparedMask_=original;
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
  const uint8_t airMask=enableAirbrakes ? config.airbrakes.channelMask : 0;
  uint8_t servos=airMask, pyros=0;
  for (std::size_t i=0;i<config.ruleCount;++i)
    for (std::size_t j=0;j<config.rules[i].assignmentCount;++j) {
      const auto &a=config.rules[i].assignments[j];
      if (a.channel<1 || a.channel>recovery::ChannelCount ||
          (a.kind!=recovery::OutputKind::Pyro && a.kind!=recovery::OutputKind::Servo)) return "INVALID_ARGUMENT";
      if (a.kind==recovery::OutputKind::Servo && (config.airbrakes.channelMask & (1u<<(a.channel-1)))) return "INVALID_ARGUMENT";
      (a.kind==recovery::OutputKind::Pyro ? pyros : servos) |= uint8_t(1u<<(a.channel-1));
    }
  for (unsigned i=0;i<recovery::ChannelCount;++i)
    if ((servos & (1u<<i)) && !config.servos[i].calibrated) return "SERVOS_NOT_CONFIGURED";
  if (requirePrepared && (servos & ~preparedServos(config))) return "SERVOS_NOT_PREPARED";
  if (!requirePrepared) if (const auto error=prepare(config,servos)) return error;
  owner_=owner; airbrakeMask_=airMask; allowedPyros_=pyros; servoMask_=servos;
  preparedMask_ &= uint8_t(~servos); // Transfer held PWM to the run without detach/reposition.
  requestedPyros_=requestedServos_=pendingPyros_=highPyros_=0; enabled_=false; startCount_=0;
  hardware_.pyros(false,0);
  for (uint8_t i=0;i<recovery::ChannelCount;++i) {
    initialUs_[i]=config.servos[i].initialUs; finalUs_[i]=config.servos[i].finalUs;
  }
  epochUs_=clock_(); // Exclude servo preparation, as the sensor source starts next.
  return nullptr;
}
void OutputExecutor::report(recovery::VirtualStart start, uint32_t now) {
  start.timeMs=uint32_t(now-epochUs_)/1000;
  if (startCount_<started_.size()) started_[startCount_++]=start;
}
bool OutputExecutor::submit(const recovery::VirtualStart &start) {
  if (!owner_ || start.channel<1 || start.channel>recovery::ChannelCount) return false;
  const auto i=start.channel-1; const uint8_t bit=uint8_t(1u<<i);
  if (start.kind==recovery::OutputKind::Servo) {
    if (!(servoMask_ & bit) || (airbrakeMask_ & bit) || (requestedServos_ & bit) || !hardware_.servo(start.channel,finalUs_[i])) return false;
    requestedServos_ |= bit; report(start,clock_()); return true;
  }
  if (start.kind!=recovery::OutputKind::Pyro || !(allowedPyros_ & bit) || (requestedPyros_ & bit)) return false;
  requestedPyros_ |= bit; pendingPyros_ |= bit; pyroStarts_[i]=start;
  return true;
}
bool OutputExecutor::airbrakes(uint64_t owner,const std::array<float,4> &opening) {
  if (!owner_ || owner!=owner_) return false;
  for (unsigned i=0;i<4;++i) if ((airbrakeMask_&(1u<<i)) && (!std::isfinite(opening[i]) || opening[i]<0 || opening[i]>1)) return false;
  bool ok=true;
  for (uint8_t i=0;i<4;++i) if (airbrakeMask_&(1u<<i)) {
    const auto pulse=uint16_t(std::lround(initialUs_[i]+opening[i]*(int(finalUs_[i])-int(initialUs_[i]))));
    // Always attempt all channels, including after an individual write failure.
    if (!hardware_.servo(i+1,pulse)) ok=false;
  }
  return ok;
}
void OutputExecutor::inhibit(uint64_t owner) {
  if (owner_==owner) { pendingPyros_=0; allowedPyros_=0; }
}
void OutputExecutor::poll(bool allowNewStarts) {
  if (!owner_) return;
  const auto now=clock_();
  for (unsigned i=0;i<recovery::ChannelCount;++i)
    if ((highPyros_ & (1u<<i)) && uint32_t(now-pulseUs_[i])>=recovery::PyroPulseMs*1000)
      highPyros_ &= uint8_t(~(1u<<i));
  if (allowNewStarts && pendingPyros_ && !enabled_) { enabled_=true; enableUs_=now; }
  if (allowNewStarts && enabled_ && uint32_t(now-enableUs_)>=recovery::PyroEnableSettleMs*1000) {
    for (unsigned i=0;i<recovery::ChannelCount;++i) {
      const uint8_t bit=uint8_t(1u<<i);
      if (!(pendingPyros_ & bit)) continue;
      highPyros_ |= bit; pendingPyros_ &= uint8_t(~bit); pulseUs_[i]=now;
      report(pyroStarts_[i],now);
    }
  }
  if (!pendingPyros_ && !highPyros_) enabled_=false;
  hardware_.pyros(enabled_,highPyros_);
}
std::size_t OutputExecutor::drain(recovery::VirtualStart *starts, std::size_t capacity) {
  const auto count=startCount_<capacity ? startCount_ : capacity;
  for (std::size_t i=0;i<count;++i) starts[i]=started_[i];
  for (std::size_t i=count;i<startCount_;++i) started_[i-count]=started_[i];
  startCount_-=count; return count;
}
unsigned OutputExecutor::stop(uint64_t owner) {
  if (!owner_ || owner!=owner_) return 0;
  unsigned pending=0;
  for (unsigned i=0;i<recovery::ChannelCount;++i) if (pendingPyros_ & (1u<<i)) ++pending;
  hardware_.pyros(false,0);
  for (uint8_t i=0;i<recovery::ChannelCount;++i) if (servoMask_ & (1u<<i)) {
    if (airbrakeMask_ & (1u<<i)) {
      if (hardware_.servo(i+1,initialUs_[i])) {
        preparedMask_ |= uint8_t(1u<<i); preparedInitial_[i]=initialUs_[i]; preparedFinal_[i]=finalUs_[i];
      } else hardware_.releaseServo(i+1);
    } else hardware_.releaseServo(i+1);
  }
  owner_=0; airbrakeMask_=0; servoMask_=pendingPyros_=highPyros_=0; enabled_=false;
  return pending;
}
}
