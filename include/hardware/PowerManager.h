#pragma once
#include <cstdint>

class PowerHardware {
public:
  virtual ~PowerHardware() = default;
  virtual const char *suspend() = 0;
  virtual const char *resume() = 0;
};

// Peripheral power saving. MCU and communication transports remain active.
class PowerManager {
public:
  using Clock = uint32_t (*)(); // Milliseconds; optional for manual-only command fixtures.
  static constexpr uint32_t IdleMs = 300000;
  explicit PowerManager(PowerHardware &hardware, Clock clock=nullptr) : hardware_(hardware), clock_(clock) {}
  bool sleeping() const { return sleeping_; }
  const char *name() const { return sleeping_ ? "sleeping" : "active"; }
  const char *error() const { return error_ ? error_ : "None"; }
  void activity() { if (clock_) { idleSince_=clock_(); tracking_=true; } }
  void poll(bool busy) {
    if (!clock_ || sleeping_) return;
    if (!tracking_) activity();
    if (!busy && uint32_t(clock_()-idleSince_)>=IdleMs) {
      sleep(false);
      activity(); // Failed suspension is retried after another idle interval, never every loop.
    }
  }
  const char *sleep(bool busy) {
    if (sleeping_) return nullptr;
    if (busy) return "BUSY";
    error_=hardware_.suspend();
    if (error_) return error_;
    sleeping_=true; return nullptr;
  }
  const char *wake() {
    activity();
    if (!sleeping_) return nullptr;
    error_=hardware_.resume();
    if (error_) return error_;
    sleeping_=false; return nullptr;
  }
private:
  PowerHardware &hardware_;
  Clock clock_;
  const char *error_=nullptr;
  uint32_t idleSince_=0;
  bool tracking_=false;
  bool sleeping_=false;
};
