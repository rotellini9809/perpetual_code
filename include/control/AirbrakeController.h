#pragma once
#include "control/AirbrakeLaw.h"
#include "control/AirbrakeStatus.h"
#include "navigation/Navigation.h"
#include <cmath>

namespace control {
class AirbrakeController {
public:
  void begin(uint8_t channels) { channels_=channels; aborted_=enabled_=false; state_={}; status_={}; status_.channelMask=channels; close(Configured); }
  void setLaw(UpdateLaw law) { law_=law ? law : updateAirbrakes; }
  const AirbrakeStatus &status() const { return status_; }
  const std::array<float,4> &commands() const { return commands_; }
  void close(uint16_t reason) {
    enabled_=false; state_={}; commands_={}; status_.flags=channels_ ? uint16_t(Configured|reason|(aborted_ ? Aborted : 0)) : 0;
    for (unsigned i=0;i<4;++i) status_.opening[i]=(channels_&(1u<<i)) ? 0 : 65535;
  }
  void update(const navigation::NavigationOutput &n, uint8_t failed) {
    status_.failedConditions=failed;
    if (!channels_) return;
    if (n.phase==recovery::FlightPhase::Abort) aborted_=true;
    bool valid=n.valid && std::isfinite(n.verticalAcceleration) && std::isfinite(n.dtSeconds) && n.dtSeconds>0 && n.dtSeconds<=.1f;
    for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) valid=valid && std::isfinite(n.covariance[i][j]) && (i!=j || n.covariance[i][j]>=0);
    for (float value : {n.altitude,n.verticalVelocity,n.accelerationBias}) valid=valid && std::isfinite(value);
    uint16_t reason=(failed ? RuleFalse : 0)|(valid ? 0 : InvalidEstimate)|(aborted_ ? Aborted : 0)|(n.startsAllowed ? 0 : NavigationBlocked);
    if (reason) { close(reason); return; }
    if (!enabled_) state_={};
    ControlInput input; input.state={{n.altitude,n.verticalVelocity,n.accelerationBias}};
    input.covariance=n.covariance; input.acceleration=n.verticalAcceleration; input.timeUs=n.timeUs;
    input.dtSeconds=n.dtSeconds; input.phase=n.phase; input.valid=true;
    const auto request=law_(input,state_);
    valid=request.valid;
    for (unsigned i=0;i<4;++i) if (channels_&(1u<<i)) valid=valid && std::isfinite(request.opening[i]);
    if (!valid) { close(InvalidRequest); return; }
    enabled_=true; status_.flags=Configured|Enabled;
    for (unsigned i=0;i<4;++i) if (channels_&(1u<<i)) {
      const float u=std::fmax(0,std::fmin(1,request.opening[i]));
      status_.opening[i]=uint16_t(std::lround(u*10000)); commands_[i]=status_.opening[i]/10000.0f;
    }
  }
private:
  uint8_t channels_=0;
  bool aborted_=false,enabled_=false;
  UpdateLaw law_=updateAirbrakes;
  LawState state_;
  AirbrakeStatus status_;
  std::array<float,4> commands_{};
};
}
