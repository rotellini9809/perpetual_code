#pragma once
#include "control/AirbrakeLaw.h"
#include "control/AirbrakeStatus.h"
#include "navigation/Navigation.h"
#include <cmath>

namespace control {
class AirbrakeController {
public:
  void begin(uint8_t channels) { channelMask_=channels; aborted_=enabled_=false; state_={}; status_={}; status_.channelMask=channels; close(Configured); }
  void setLaw(UpdateLaw law) { law_=law ? law : updateAirbrakes; }
  const AirbrakeStatus &status() const { return status_; }
  const std::array<float,4> &commands() const { return commands_; }
  void close(uint16_t reason) {
    enabled_=false; state_={}; commands_={}; status_.flags=channelMask_ ? uint16_t(Configured|reason|(aborted_ ? Aborted : 0)) : 0;
    for (unsigned i=0;i<4;++i) status_.opening[i]=(channelMask_&(1u<<i)) ? 0 : 65535;
  }
  void update(const navigation::NavigationOutput &nav, uint8_t failedConditions) {
    status_.failedConditions=failedConditions;
    if (!channelMask_) return;
    if (nav.phase==recovery::FlightPhase::Abort) aborted_=true;
    bool valid=nav.valid && std::isfinite(nav.verticalAcceleration) && std::isfinite(nav.dtSeconds) && nav.dtSeconds>0 && nav.dtSeconds<=.1f;
    for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) valid=valid && std::isfinite(nav.covariance[i][j]) && (i!=j || nav.covariance[i][j]>=0);
    for (float value : {nav.altitude,nav.verticalVelocity,nav.accelerationBias}) valid=valid && std::isfinite(value);
    uint16_t reason=(failedConditions ? RuleFalse : 0)|(valid ? 0 : InvalidEstimate)|(aborted_ ? Aborted : 0)|(nav.startsAllowed ? 0 : NavigationBlocked);
    if (reason) { close(reason); return; }
    if (!enabled_) state_={};
    ControlInput input; input.state={{nav.altitude,nav.verticalVelocity,nav.accelerationBias}};
    input.covariance=nav.covariance; input.acceleration=nav.verticalAcceleration; input.timeUs=nav.timeUs;
    input.dtSeconds=nav.dtSeconds; input.phase=nav.phase; input.valid=true;
    const auto request=law_(input,state_);
    valid=request.valid;
    for (unsigned i=0;i<4;++i) if (channelMask_&(1u<<i)) valid=valid && std::isfinite(request.opening[i]);
    if (!valid) { close(InvalidRequest); return; }
    enabled_=true; status_.flags=Configured|Enabled;
    for (unsigned i=0;i<4;++i) if (channelMask_&(1u<<i)) {
      const float opening=std::fmax(0,std::fmin(1,request.opening[i]));
      status_.opening[i]=uint16_t(std::lround(opening*10000)); commands_[i]=status_.opening[i]/10000.0f;
    }
  }
private:
  uint8_t channelMask_=0;
  bool aborted_=false,enabled_=false;
  UpdateLaw law_=updateAirbrakes;
  LawState state_;
  AirbrakeStatus status_;
  std::array<float,4> commands_{};
};
}
