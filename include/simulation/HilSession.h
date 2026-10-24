#pragma once
#include "flight/ArmingController.h"
#include "simulation/HilSensorSource.h"

namespace simulation {
// Host injection adapter only. FlightRuntime owns the common flight loop.
class HilSession {
public:
  using Clock = timing::Clock;
  HilSession(ConfigurationService &config, flight::RunController &run, Clock clock = nullptr,
             sensors::FrameConvention convention = sensors::FrameConvention::LoggedCalibratedBody,
             storage::RecordingSink *recorder = nullptr)
      : source_(convention), controller_(config, run, clock, recorder) {}
  sensors::FrameConvention convention() const { return source_.convention(); }
  const char *preparationError() const { return controller_.runtime().preparationError(); }
  const char *begin(float referenceAltitude, float nominalHz,
                    const navigation::SensorCalibration &calibration = navigation::SensorCalibration{}, uint32_t recordBytes = 0);
  const char *sample(uint32_t timeMs, const float accelMps2[3], const float gyroRadps[3], float pressurePa,
                     flight::FrameResult &result, float temperatureC = std::numeric_limits<float>::quiet_NaN());
  const char *process(const sensors::Reading &reading, flight::FrameResult &result);
  unsigned stop(storage::RecordingEnd reason = storage::RecordingEnd::Stopped) {
    const auto pending = controller_.stop(reason); source_.clear(); return pending;
  }
  bool active() const { return controller_.armed(); }
  bool blockedByOtherRun() const { return controller_.runtime().blockedByOtherRun(); }
  flight::RuntimeStatus status() const { return controller_.runtime().status(); }
  const char *disarm() { if (const auto error = controller_.disarmError()) return error; stop(); return nullptr; }
  const flight::ArmingController &controller() const { return controller_; }
  void setBattery(const BatteryMonitor &battery) { controller_.setBattery(battery); }
  const flight::RuntimeTiming &timing() const { return controller_.runtime().timing(); }
private:
  sensors::HilSensorSource source_;
  flight::ArmingController controller_;
  uint64_t baroSequence_ = 0;
};
}
