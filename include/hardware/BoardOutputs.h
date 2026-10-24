#pragma once
#include "flight/OutputExecutor.h"
#include "hardware/PyroController.h"
#include "hardware/ServoController.h"

// Board adapter: no timing, rules or simulation policy here.
class BoardOutputs : public flight::OutputHardware {
public:
  BoardOutputs(PyroController &pyros, ServoController &servos) : pyros_(pyros), servos_(servos) {}
  void begin() { pyros_.begin(); servos_.begin(); }
  bool continuity(uint8_t channel) override {
    return pyros_.checkContinuity(channel).state==PyroController::ContinuityState::Connected;
  }
  void pyros(bool enabled, uint8_t mask) override { pyros_.drive(enabled,mask); }
  bool servo(uint8_t channel, uint16_t pulseUs) override { return servos_.setPulse(channel,pulseUs); }
  void releaseServo(uint8_t channel) override { servos_.stop(channel); }
private:
  PyroController &pyros_;
  ServoController &servos_;
};
