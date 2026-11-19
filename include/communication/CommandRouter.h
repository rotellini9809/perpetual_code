#pragma once
#include "configuration/ConfigurationCommands.h"
#include "simulation/SensorHilCommands.h"
#include "simulation/EstimateReplayCommands.h"
#include "storage/FlightCommands.h"
#include "simulation/SimulationCommands.h"
#include "communication/ArmingCommands.h"
#include "hardware/PowerManager.h"
#include "communication/CommandPolicy.h"

// Routes siblings and owns the development-mode command policy.
// Bluetooth module renaming, owned by MAIN's radio driver (absent in PC fixtures).
class BluetoothNaming {
public:
  virtual ~BluetoothNaming() = default;
  // nullptr when accepted; the module restarts once the accepting reply is sent.
  virtual const char *requestName(const char *suffix, char *name, std::size_t capacity) = 0;
};

class CommandRouter : public CommandExtension {
public:
  enum class Source { None, Usb, Bluetooth };
  CommandRouter(ConfigurationCommands &configuration, EstimateReplayCommands &estimates,
                SensorHilCommands &hil, ConfigurationService &settings, flight::RunController &run, FlightCommands *flights = nullptr,
                SimulationCommands *simulation = nullptr, ArmingCommands *arming = nullptr)
      : configuration_(configuration), estimates_(estimates), hil_(hil), settings_(settings), run_(run), flights_(flights), simulation_(simulation), arming_(arming) {}
  bool dispatch(const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity) override;
  bool dispatchFrom(Source source, const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity);
  void disconnect(Source source = Source::Usb);
  void setPower(PowerManager &power, bool (*transportBusy)() = nullptr) { power_=&power; transportBusy_=transportBusy; }
  void pollPower();
  void setBluetooth(BluetoothNaming &bluetooth) { bluetooth_=&bluetooth; }
private:
  BluetoothNaming *bluetooth_=nullptr;
  bool dispatchCommand(const char *command, const char *arguments, const protocol::CommandPolicy &policy, bool hasId, uint32_t id, char *output, std::size_t capacity);
  // Rarely used replies, kept out of the dispatch frames (see CommandRouter.cpp).
  __attribute__((noinline)) bool controlReply(Source source, const char *command, bool valid, bool hasId, uint32_t id, char *output, std::size_t capacity);
  __attribute__((noinline)) bool powerReply(const char *command, bool valid, bool hasId, uint32_t id, char *output, std::size_t capacity);
  __attribute__((noinline)) bool bluetoothNameReply(const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity);
  __attribute__((noinline)) bool airbrakeStatusReply(const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity);
  bool occupied() const;
  bool powerBusy() const;
  Source owner_ = Source::None;
  ConfigurationCommands &configuration_;
  EstimateReplayCommands &estimates_;
  SensorHilCommands &hil_;
  ConfigurationService &settings_;
  flight::RunController &run_;
  FlightCommands *flights_;
  SimulationCommands *simulation_;
  ArmingCommands *arming_;
  PowerManager *power_=nullptr;
  bool (*transportBusy_)()=nullptr;
};

class CommandSession : public CommandExtension {
public:
  CommandSession(CommandRouter &router, CommandRouter::Source source) : router_(router), source_(source) {}
  bool dispatch(const char *command, const char *arguments, bool hasId, uint32_t id, char *output, std::size_t capacity) override {
    return router_.dispatchFrom(source_, command, arguments, hasId, id, output, capacity);
  }
  void disconnect() { router_.disconnect(source_); }
private:
  CommandRouter &router_;
  CommandRouter::Source source_;
};
