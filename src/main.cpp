#include "diagnostics/BuildMode.h"
#ifdef MAIN
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <hardware/sync.h>
#include "hardware/board_pins.h"
#include "hardware/PyroController.h"
#include "hardware/BoardOutputs.h"
#include "hardware/Sensors.h"
#include "hardware/HardwareSensorSource.h"
#include "communication/CommandService.h"
#include "communication/CommandEndpoint.h"
#include "hardware/Jdy34Link.h"
#include "storage/FlashLoggerAdapter.h"
#include "configuration/ConfigurationCommands.h"
#include "configuration/ActuatorPolicy.h"
#include "simulation/EstimateReplayCommands.h"
#include "communication/CommandRouter.h"
#include "hardware/StatusLed.h"

Adafruit_NeoPixel statusPixel(1, RGB_LED, NEO_RGB + NEO_KHZ800);
void writeStatusLed(uint8_t r, uint8_t g, uint8_t b) {
  statusPixel.setPixelColor(0,statusPixel.Color(r,g,b)); statusPixel.show();
}
StatusLed statusLed(writeStatusLed);
PyroController pyros;
SPIClassRP2040 memorySpi(FLASH_SPI_PORT, MISO_FLASH, CS_FLASH1, SCK_FLASH, MOSI_FLASH);
FlashLogger flash(memorySpi, CS_FLASH1);
storage::FlashLoggerAdapter flashDevice(flash);
uint32_t navigationClock() { return micros(); }
ServoController servos;
BoardOutputs outputHardware(pyros,servos);
flight::OutputExecutor outputExecutor(outputHardware,navigationClock);
BatteryMonitor battery;
uint16_t readBatteryAdc() {
  analogRead(BATT_SENSE); // Discard the first conversion after switching from continuity's ADC input.
  uint32_t total=0;
  for (unsigned i=0;i<8;++i) {
    const auto raw=analogRead(BATT_SENSE);
    if (raw>=4095) return 4095; // A saturated measurement is unavailable, not a clamped voltage.
    total+=raw;
  }
  return uint16_t((total+4)/8);
}
Sensors sensorDevice;
sensors::HardwareSensorSource hardwareSource(sensorDevice, navigationClock);
storage::ProfiledFlashDevice profiledFlash(flashDevice, navigationClock);
storage::ConfigurationStore configurationStore(profiledFlash);
storage::ImuCalibrationStore imuCalibrationStore(profiledFlash);
storage::FlightRecorder recorder(profiledFlash);
void wakeStorage() { __sev(); }
storage::QueuedRecorder recordingWorker(recorder, navigationClock, wakeStorage);
// Arduino-Pico allocates an independent 8 KiB stack for core 1.
bool core1_separate_stack = true;
void setup1() { recordingWorker.workerStarted(); }
void loop1() { if (!recordingWorker.poll()) __wfe(); }
// Development policy, exposed by capabilities; physical execution is opt-in in simulation.
const recovery::ValidationLimits configLimits{1000, 2000, recovery::PyroPulseMs, 3600000};
ConfigurationService settings(configurationStore, configLimits);
ConfigurationCommands configuration(settings,true,true);
flight::RunController run;
EstimateReplayCommands replay(settings, run);
simulation::HilSession hilSession(settings, run, navigationClock, sensors::FrameConvention::LoggedCalibratedBody, &recordingWorker);
SensorHilCommands hil(hilSession);
simulation::SimulationSession standaloneSimulation(settings, run, navigationClock, &recordingWorker, &outputExecutor);
SimulationCommands simulationCommands(standaloneSimulation);
flight::ArmingController physicalFlight(settings, run, navigationClock, &recordingWorker, &outputExecutor);
class BoardArmingChecks : public flight::PhysicalChecks {
public:
  flight::PhysicalReadiness inspect() override {
    flight::PhysicalReadiness r;
    r.imuReady = hardwareSource.imuReady(); r.baroReady = hardwareSource.baroReady();
    // BRING-UP: the IMU solder fault remains a physical-mode blocker. Never fake
    // hardware measurements. Simulation explicitly skips physical hardware checks.
    // Output preparation/continuity are shared with simulation in OutputExecutor.
    return r;
  }
} physicalChecks;
ArmingCommands armingCommands(physicalFlight, hardwareSource, physicalChecks, &standaloneSimulation, &hilSession);
FlightCommands flightCommands(recorder, settings, run, &profiledFlash, &recordingWorker, navigationClock);
CommandRouter router(configuration, replay, hil, settings, run, &flightCommands, &simulationCommands, &armingCommands);
CommandSession usbSession(router, CommandRouter::Source::Usb);
CommandSession bluetoothSession(router, CommandRouter::Source::Bluetooth);
CommandEndpoint usbEndpoint(usbSession), bluetoothEndpoint(bluetoothSession);
bluetooth::Jdy34Link bluetoothLink;
// `bluetooth name` from either transport; the radio driver performs it once the reply is sent.
class LinkNaming : public BluetoothNaming {
public:
  const char *requestName(const char *suffix, char *name, std::size_t capacity) override {
    const char *error = bluetoothLink.requestName(suffix);
    if (!error) bluetooth::Jdy34Link::boardName(suffix,name,capacity);
    return error;
  }
} bluetoothNaming;
uint32_t bluetoothUartBaud = bluetooth::Jdy34Link::UartBaud;
uint32_t bluetoothQuietSince = 0; // Last time a reply or AT request was still being sent.
CommandService::Status commandStatus;
bool usbConnected = false;
bool bluetoothGate = false;
uint32_t bluetoothEpoch = 0;
const char *bluetoothQuery = nullptr;
std::size_t bluetoothQueryOffset = 0;
uint32_t lastHardwarePollUs = 0, lastBaroMs = 0, lastImuMs = 0;
uint64_t lastBaroSequence = 0, lastImuSequence = 0;

void updateHardwareStatus() {
  if (hardwareSource.latest().baroSequence != lastBaroSequence) {
    lastBaroSequence = hardwareSource.latest().baroSequence; lastBaroMs = millis();
  }
  if (hardwareSource.latest().imuSequence != lastImuSequence) {
    lastImuSequence = hardwareSource.latest().imuSequence; lastImuMs = millis();
  }
  commandStatus.imuReady = hardwareSource.imuReady(); commandStatus.baroReady = hardwareSource.baroReady();
  commandStatus.imuId = sensorDevice.imuWhoAmI(); commandStatus.baroId = sensorDevice.baroChipId();
  commandStatus.imuError = sensorDevice.lastImuError(); commandStatus.baroError = sensorDevice.lastBaroError();
  commandStatus.imuSampleAgeMs = millis() - lastImuMs;
  commandStatus.imuSampleValid = lastImuSequence && hardwareSource.latest().imuValid && commandStatus.imuSampleAgeMs <= 100;
  for (unsigned i=0;i<3;++i) {
    commandStatus.accelMps2[i] = hardwareSource.latest().accelMps2[i];
    commandStatus.gyroRadps[i] = hardwareSource.latest().gyroRadps[i];
  }
  commandStatus.baroSampleAgeMs = millis() - lastBaroMs;
  commandStatus.baroSampleValid = lastBaroSequence && hardwareSource.latest().baroValid && commandStatus.baroSampleAgeMs <= 100;
  commandStatus.pressurePa = hardwareSource.latest().pressurePa;
  commandStatus.temperatureC = hardwareSource.latest().temperatureC;
}

class BoardPower : public PowerHardware {
public:
  const char *suspend() override {
    outputExecutor.releasePrepared(); outputHardware.pyros(false,0);
    hardwareSource.suspend();
    if (!sensorDevice.suspend()) { hardwareSource.begin(); return "SENSOR_SLEEP_FAILED"; }
    if (!flash.powerDown()) { hardwareSource.begin(); return "FLASH_SLEEP_FAILED"; }
    battery.clear(); return nullptr;
  }
  const char *resume() override {
    if (!flash.initialize(FLASH_CLOCK_HZ)) return "FLASH_WAKE_FAILED";
    commandStatus.flashJedec=flash.readJEDEC();
    commandStatus.flashIdentified=FlashLogger::isW25Q128(commandStatus.flashJedec);
    // BRING-UP: wake reprobes real devices. Missing IMU remains unavailable and
    // blocks physical arming; a working barometer can still resume independently.
    hardwareSource.begin();
    lastBaroSequence=lastImuSequence=0; lastHardwarePollUs=micros();
    sensors::Reading unused; hardwareSource.poll(unused);
    updateHardwareStatus(); // A sensors request that caused wake must see the new probe, not the sleeping snapshot.
    battery.clear(); return nullptr;
  }
} boardPower;
PowerManager power(boardPower,[]()->uint32_t { return millis(); });
bool transportBusy() { return usbEndpoint.pending() || bluetoothEndpoint.pending(); }

template <class Port> void sendPending(Port &port, CommandEndpoint &endpoint) {
  if (!endpoint.pending()) return;
  const int available = port.availableForWrite();
  if (available <= 0) return;
  auto count = endpoint.remaining();
  if (count > std::size_t(available)) count = std::size_t(available);
  if (count > 64) count = 64;
  endpoint.sent(port.write(endpoint.data(), count));
}

void syncBluetoothSession() {
  if (bluetoothEpoch == bluetoothLink.epoch()) return;
  bluetoothEndpoint.reset(); bluetoothSession.disconnect();
  bluetoothEpoch = bluetoothLink.epoch();
}

void pollBluetooth() {
  if (Serial2.overflow()) { ++commandStatus.bluetoothRxOverflows; bluetoothLink.overflow(); }
  syncBluetoothSession();
  // Continue receiving link notifications during response backpressure. The
  // client must wait for its complete reply before sending the next request.
  for (unsigned budget = 0; budget < 64 && Serial2.available() > 0; ++budget) {
    const bool application = bluetoothLink.feed(static_cast<char>(Serial2.read()), millis());
    syncBluetoothSession();
    if (application && !bluetoothEndpoint.pending()) {
      for (std::size_t i = 0; i < bluetoothLink.length(); ++i) bluetoothEndpoint.feed(bluetoothLink.line()[i], commandStatus);
      bluetoothEndpoint.feed('\n', commandStatus);
    }
  }
  // Arduino-Pico UART reports writable as 0/1, not FIFO free space. Feed AT
  // requests one writable byte at a time, exactly like application responses.
  // Start a rename only after its accepting reply reached the phone or PC: the module still has to
  // transmit the bytes over the air after they leave the UART, and the rename then drops the link.
  if (bluetoothEndpoint.pending() || bluetoothQuery) bluetoothQuietSince = millis();
  else if (bluetoothLink.renamePending() && uint32_t(millis()-bluetoothQuietSince) >= 500) { Serial2.flush(); bluetoothLink.startRename(millis()); }
  if (!bluetoothQuery) {
    bluetoothQuery = bluetoothLink.poll(millis()); bluetoothQueryOffset = 0;
  }
  // Setup of a factory module switches between 230400 and 9600; requests are complete by then.
  if (bluetoothLink.uartBaud() != bluetoothUartBaud && !bluetoothQuery) {
    Serial2.flush(); Serial2.end(); bluetoothUartBaud = bluetoothLink.uartBaud(); Serial2.begin(bluetoothUartBaud);
    commandStatus.bluetoothBaud = bluetoothUartBaud;
  }
  if (bluetoothQuery && Serial2.availableForWrite() > 0) {
    if (Serial2.write(uint8_t(bluetoothQuery[bluetoothQueryOffset]))) ++bluetoothQueryOffset;
    if (!bluetoothQuery[bluetoothQueryOffset]) bluetoothQuery = nullptr;
  }
  if (bluetoothGate != bluetoothLink.ready()) {
    bluetoothGate = bluetoothLink.ready(); gpio_put(PWRC_BLE, bluetoothGate);
  }
  commandStatus.bluetoothReady = bluetoothLink.ready();
  commandStatus.bluetoothConnected = bluetoothLink.connected();
  commandStatus.bluetoothError = bluetoothLink.error();
  commandStatus.bluetoothName = bluetoothLink.name();
  commandStatus.bluetoothRename = bluetoothLink.renameState(); commandStatus.bluetoothConfigured = bluetoothLink.configured();
  commandStatus.bluetoothStage = bluetoothLink.stageName();
  if (bluetoothLink.connected()) sendPending(Serial2, bluetoothEndpoint);
}

void setup() {
  // Inactive outputs before USB initialization; no temporary enable override.
  outputHardware.begin(); // Servo PWM stays detached until an explicit physical run.
  analogReadResolution(12);
  physicalFlight.setBattery(battery);
  standaloneSimulation.setBattery(battery);
  hilSession.setBattery(battery);
  Serial.begin(115200);
  statusPixel.begin(); statusPixel.setBrightness(40); // Same normal brightness for all modes, including rainbow.
  statusLed.update(millis(),LedMode::Disarmed);
  router.setPower(power,transportBusy);
  router.setBluetooth(bluetoothNaming);
  delay(10);
  const bool flashReady = flash.initialize(FLASH_CLOCK_HZ);
  commandStatus.flashJedec = flash.readJEDEC();
  commandStatus.flashIdentified = FlashLogger::isW25Q128(commandStatus.flashJedec);
  configuration.boot(flashReady); // Read only: missing settings never trigger formatting.
  if (flashReady) imuCalibrationStore.load();
  physicalFlight.setImuCalibrationStore(imuCalibrationStore);
  // Mount existing flights and reconcile interrupted recordings before accepting
  // commands/runs. Core 1 stays idle until a recording starts. Never autoformat:
  // failures remain visible through flights status/failure and physical preflight.
  if (flashReady) recorder.initialize();
  // BOARD BRING-UP: IMU soldering is currently faulty; barometer was verified on USB.
  // Probe both independently; barometer diagnostics remain usable if the IMU fails.
  // Missing IMU frames are never faked and still block physical arming.
  hardwareSource.begin();
  // Hardware UART1: MCU TX GP4, RX GP5. PWRC is AT access, not power/reset.
  gpio_init(PWRC_BLE); gpio_put(PWRC_BLE, false); gpio_set_dir(PWRC_BLE, GPIO_OUT);
  Serial2.setTX(TX_BLE); Serial2.setRX(RX_BLE); Serial2.setFIFOSize(256); Serial2.begin(bluetooth::Jdy34Link::UartBaud); commandStatus.bluetoothBaud=bluetooth::Jdy34Link::UartBaud;
  bluetoothLink.begin(millis()); bluetoothEpoch = bluetoothLink.epoch();
  // Blue means development/disarmed, not flight-ready.
}

void loop() {
  if (!power.sleeping()) battery.poll(millis(),readBatteryAdc); // Real board voltage, also during simulation/HIL.
  standaloneSimulation.poll(); // Due sensor work runs before USB, even disconnected.
  if (!power.sleeping()) physicalFlight.maintainImuApproval();
  if (physicalFlight.active()) {
    flight::FrameResult result;
    physicalFlight.poll(result); // Same runtime/executor as simulation, with real sensors.
  }
  if (!run.active() && !physicalFlight.active()) outputExecutor.reconcile(settings.applied());
  // Idle-only hardware diagnostics: simulation/HIL use their chosen source exclusively.
  const auto nowUs = micros();
  if (!power.sleeping() && !run.active() && !physicalFlight.active() && uint32_t(nowUs - lastHardwarePollUs) >= 2000) {
    lastHardwarePollUs = nowUs;
    sensors::Reading unused;
    hardwareSource.poll(unused);
  }
  const bool connected = static_cast<bool>(Serial);
  if (connected != usbConnected) {
    usbEndpoint.reset();
    if (!connected) usbSession.disconnect();
    usbConnected = connected;
  }
  commandStatus.uptimeMs = millis();
  commandStatus.powerMode=power.name();
  commandStatus.powerError=power.error();
  commandStatus.batteryMv = battery.millivolts();
  commandStatus.configValid = configuration.configValid();
  commandStatus.configError = configuration.configError();
  commandStatus.flashReady = !power.sleeping() && configuration.storageReady();
  commandStatus.recordingReady = !power.sleeping() && recordingWorker.initialized();
  commandStatus.estimateReplayActive = replay.active();
  commandStatus.sensorHilActive = hil.active();
  commandStatus.simulationActive = standaloneSimulation.active();
  const flight::ArmingController *owner = nullptr;
  if (physicalFlight.active() || physicalFlight.state() == flight::ArmingState::Closing) owner = &physicalFlight;
  else if (standaloneSimulation.active() || standaloneSimulation.controller().state() == flight::ArmingState::Closing) owner = &standaloneSimulation.controller();
  else if (hil.active() || hilSession.controller().state() == flight::ArmingState::Closing) owner = &hilSession.controller();
  commandStatus.armingState = owner ? flight::armingStateName(owner->state()) : "disarmed";
  commandStatus.flightMode = owner ? flight::flightModeName(owner->mode()) : "none";
  commandStatus.outputMode = (owner && (owner->mode()==flight::FlightMode::Physical || owner->runtime().status().outputMode == storage::OutputMode::Physical)) || outputExecutor.hasPreparation() ? "PHYSICAL" : "VIRTUAL";
  commandStatus.outputsActive = (owner && owner->runtime().status().outputsActive) || outputExecutor.hasPreparation();
  commandStatus.phase = owner && owner->navigation().valid ? int(owner->navigation().phase) : -1;
  commandStatus.gyroSamples = owner ? owner->calibrationSamples() : 0;
  commandStatus.gyroTarget = flight::ArmingCalibration::TargetSamples;
  updateHardwareStatus();
  const auto ledMode=power.sleeping() ? LedMode::PowerSaving : physicalFlight.state()==flight::ArmingState::Arming ? LedMode::Calibrating :
      commandStatus.phase==5 ? LedMode::Boost : commandStatus.phase==6 ? LedMode::Coast :
      commandStatus.phase==7 ? LedMode::Abort : commandStatus.phase>=0 ? static_cast<LedMode>(commandStatus.phase) : LedMode::Disarmed;
  statusLed.update(millis(),ledMode);
  pollBluetooth();
  // Both endpoints get a bounded turn every loop; neither waits for TX space.
  // USB remains normally active even while the radio is absent or backpressured.
  if (connected) {
    sendPending(Serial, usbEndpoint);
    for (unsigned budget = 0; budget < 64 && !usbEndpoint.pending() && Serial.available() > 0; ++budget)
      usbEndpoint.feed(static_cast<char>(Serial.read()), commandStatus);
  }
  router.pollPower(); // Five minutes idle; no sleep during flight, writes, previews or active transfers.
}
#endif // MAIN
