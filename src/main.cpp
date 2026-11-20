#include "diagnostics/BuildMode.h"
#ifdef MAIN
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <hardware/sync.h>
#include <hardware/watchdog.h>
#include "hardware/board_pins.h"
#include "hardware/PyroController.h"
#include "hardware/BoardOutputs.h"
#include "hardware/Sensors.h"
#include "hardware/HardwareSensorSource.h"
#include "hardware/SensorRates.h"
#include "communication/CommandService.h"
#include "communication/CommandEndpoint.h"
#include "hardware/Jdy34Link.h"
#include "storage/FlashLoggerAdapter.h"
#include "configuration/ConfigurationCommands.h"
#include "configuration/ActuatorPolicy.h"
#include "simulation/EstimateReplayCommands.h"
#include "communication/CommandRouter.h"
#include "hardware/StatusLed.h"
#include "diagnostics/BoardDiagnostics.h"

// Stack high-water marks: both stacks are painted at startup and scanned on `diag`.
// Core 0 starts at the top of SCRATCH_Y (4 KiB). It is measured down to the end of SCRATCH_X
// data: with core1_separate_stack, SCRATCH_X holds no data and is the only room below SCRATCH_Y
// before the heap limit, so a core 0 stack deeper than 4 KiB lands there. Core 1 runs on the
// 8 KiB block Arduino-Pico allocates for core1_separate_stack.
extern "C" {
extern uint32_t __scratch_x_end__, __StackTop;
extern uint32_t *core1_separate_stack_address;
}
// Stack guard: each core's MSPLIM sits StackGuardBytes above the bottom of its stack, so an
// overflow faults instead of silently writing below it. The default HardFault handler is a
// breakpoint (a hang without a debugger): this one lifts the limit, keeps the cause in RAM that
// survives reset, and reboots; `diag` reports it as last_fault.
// The record lives in watchdog scratch 0-3 (left to the application by the SDK), which survive a
// watchdog reboot.
constexpr uint32_t FaultMagic = 0xFA017E00u, StackGuardBytes = 256;
extern "C" __attribute__((used, noreturn)) void reportFault(uint32_t sp) {
  watchdog_hw->scratch[1] = *reinterpret_cast<volatile uint32_t *>(0xE000ED28u); // SCB CFSR
  watchdog_hw->scratch[2] = *reinterpret_cast<volatile uint32_t *>(0xE000ED2Cu); // SCB HFSR
  watchdog_hw->scratch[3] = sp;
  watchdog_hw->scratch[0] = FaultMagic | get_core_num();
  watchdog_reboot(0, 0, 0);
  for (;;) {}
}
extern "C" __attribute__((naked)) void isr_hardfault() {
  asm volatile("movs r0, #0\n msr msplim, r0\n mrs r0, msp\n b reportFault\n");
}
void setStackLimit(const uint32_t *bottom) {
  asm volatile("msr msplim, %0" :: "r"(reinterpret_cast<uintptr_t>(bottom) + StackGuardBytes));
}
namespace {
constexpr uint32_t StackPaint = 0x5AC3A55Cu, Core1StackBytes = 0x2000;
uint32_t *core1StackBottom = nullptr;
__attribute__((noinline)) void paintStack(uint32_t *bottom) {
  uint32_t *sp; asm volatile("mov %0, sp" : "=r"(sp));
  for (volatile uint32_t *p = bottom; p + 32 < sp; ++p) *p = StackPaint; // 128 bytes below the live frame stay untouched.
}
StackUse stackUse(const uint32_t *bottom, const uint32_t *top) {
  StackUse use; if (!bottom || top <= bottom) return use;
  use.capacityBytes = uint32_t(top - bottom) * 4;
  const volatile uint32_t *p = bottom; while (p < top && *p == StackPaint) ++p;
  use.usedBytes = uint32_t(top - p) * 4; use.exhausted = p == bottom;
  bool first = true;
  for (const volatile uint32_t *q = bottom; q < top; ++q) if (*q == StackPaint) {
    use.intactBytes += 4; const uint32_t offset = uint32_t(q - bottom) * 4;
    if (first) { use.intactLowOffset = offset; first = false; }
    use.intactHighOffset = offset;
  }
  return use;
}
}

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
void setup1() {
  core1StackBottom = core1_separate_stack_address; paintStack(core1StackBottom); setStackLimit(core1StackBottom);
  recordingWorker.workerStarted();
}
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
    // A missing IMU or barometer blocks physical mode. Never fake hardware
    // measurements. Simulation explicitly skips physical hardware checks.
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
BoardDiagnostics diagnostics;
uint32_t lastLoopUs = 0, runFifoBase = 0, runFifoEnd = 0, runStartMs = 0;
bool loopUsbInput = false; // The previous loop read USB command input.
bool runWasActive = false;
const flight::ArmingController *lastRun = nullptr; // Physical, simulation or HIL owner of the last run.
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
    // Wake reprobes real devices. A missing IMU stays unavailable and blocks
    // physical arming; a working barometer still resumes independently.
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

void refreshDiagnostics(BoardDiagnostics &d) {
  d.stack[0] = stackUse(&__scratch_x_end__, &__StackTop);
  d.stack[1] = stackUse(core1StackBottom, core1StackBottom ? core1StackBottom + Core1StackBytes/4 : nullptr);
  d.runActive = runWasActive;
  if (!lastRun) return;
  const auto &t = lastRun->runtime().timing();
  d.step = t.step; d.compute = t.compute; d.storage = t.storage; d.overBudget = t.overBudget;
  d.frameDtMinUs = t.frameDtMinUs; d.frameDtMaxUs = t.frameDtMaxUs; d.sequenceGaps = t.sequenceGaps;
  // Simulation and HIL do not read the IMU, so its FIFO overruns only mean something for a physical run.
  d.fifoOverruns = lastRun == &physicalFlight ? (runWasActive ? sensorDevice.imuFifoOverruns() : runFifoEnd) - runFifoBase : 0;
}

void setup() {
  paintStack(&__scratch_x_end__); // First: before any deep call chain.
  setStackLimit(&__scratch_x_end__);
  if ((watchdog_hw->scratch[0] & ~0xFFu) == FaultMagic) {
    diagnostics.lastFault = true; diagnostics.lastFaultCore = watchdog_hw->scratch[0] & 0xFFu;
    diagnostics.lastFaultCfsr = watchdog_hw->scratch[1]; diagnostics.lastFaultHfsr = watchdog_hw->scratch[2];
    diagnostics.lastFaultSp = watchdog_hw->scratch[3];
    diagnostics.lastFaultStackOverflow = diagnostics.lastFaultCfsr & (1u << 20); // UFSR STKOF
  }
  watchdog_hw->scratch[0] = 0;
  diagnostics.loopPeriodUs = 1000000u / sensors::ImuRateHz; diagnostics.refresh = refreshDiagnostics;
  {
    static const char *const reasons[] = {"unknown","power_on","run_pin","soft","watchdog","debug","glitch","brownout"};
    const unsigned reason = unsigned(rp2040.getResetReason());
    diagnostics.resetReason = reason < 8 ? reasons[reason] : "unknown";
    diagnostics.resetChipBits = *reinterpret_cast<volatile uint32_t *>(POWMAN_BASE + POWMAN_CHIP_RESET_OFFSET);
    diagnostics.resetWatchdogBits = *reinterpret_cast<volatile uint32_t *>(WATCHDOG_BASE + WATCHDOG_REASON_OFFSET);
  }
  commandStatus.diagnostics = &diagnostics;
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
  // commands/runs. Core 1 stays idle until a recording starts. A never-used flash
  // (no directory, erased banks) is prepared here, like `flights init`; used or
  // damaged storage is never formatted and its failure stays visible.
  if (flashReady) recorder.prepareAtBoot();
  // Probe both sensors independently; barometer diagnostics remain usable if the IMU fails.
  // Missing IMU frames are never faked and still block physical arming.
  hardwareSource.begin();
  // Hardware UART1: MCU TX GP4, RX GP5. PWRC is AT access, not power/reset.
  gpio_init(PWRC_BLE); gpio_put(PWRC_BLE, false); gpio_set_dir(PWRC_BLE, GPIO_OUT);
  Serial2.setTX(TX_BLE); Serial2.setRX(RX_BLE); Serial2.setFIFOSize(256); Serial2.begin(bluetooth::Jdy34Link::UartBaud); commandStatus.bluetoothBaud=bluetooth::Jdy34Link::UartBaud;
  bluetoothLink.begin(millis()); bluetoothEpoch = bluetoothLink.epoch();
  // Blue means development/disarmed, not flight-ready.
}

void loop() {
  const uint32_t loopUs = micros();
  // Loop evidence belongs to the run: it stops at run end, so commands after landing
  // (flight list, download) never show up as flight stalls. Before any run it covers idle.
  if (lastLoopUs && (runWasActive || !lastRun)) {
    const uint32_t period = loopUs - lastLoopUs;
    if (period > diagnostics.loop.maxUs) { // Phase and input are still those of the loop that just ended.
      diagnostics.loopMaxRunMs = millis() - runStartMs; diagnostics.loopMaxPhase = commandStatus.phase;
      diagnostics.loopMaxUsbInput = loopUsbInput;
    }
    diagnostics.loop.add(period);
    if (period > diagnostics.loopPeriodUs) ++diagnostics.loopOverPeriod;
  }
  lastLoopUs = loopUs; loopUsbInput = false;
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
  if (!power.sleeping() && !run.active() && !physicalFlight.active() && uint32_t(nowUs - lastHardwarePollUs) >= 1000) { // Below the 2083 us IMU period at 480 Hz.
    const uint32_t gap = nowUs - lastHardwarePollUs;
    if (commandStatus.idlePolls) {
      if (gap > 1000000u / sensors::ImuRateHz) ++commandStatus.idleLateGaps;
      if (gap > commandStatus.idleGapMaxUs) commandStatus.idleGapMaxUs = gap;
    }
    lastHardwarePollUs = nowUs;
    sensors::Reading frame;
    const auto readStart = micros();
    const bool emitted = hardwareSource.poll(frame);
    const uint32_t elapsed = micros() - readStart;
    // Frame-time quality as navigation would see it (dt and sequence continuity).
    static uint64_t lastFrameUs = 0; static uint32_t lastSequence = 0;
    if (emitted) {
      if (lastSequence && frame.imuSequence == lastSequence + 1 && frame.timeUs > lastFrameUs) {
        const auto dt = uint32_t(frame.timeUs - lastFrameUs);
        if (!commandStatus.idleDtMinUs || dt < commandStatus.idleDtMinUs) commandStatus.idleDtMinUs = dt;
        if (dt > commandStatus.idleDtMaxUs) commandStatus.idleDtMaxUs = dt;
      } else if (lastSequence) ++commandStatus.idleSequenceGaps;
      lastFrameUs = frame.timeUs; lastSequence = frame.imuSequence;
    }
    commandStatus.imuFifoOverruns = sensorDevice.imuFifoOverruns(); commandStatus.imuFifoErrors = sensorDevice.imuFifoErrors();
    ++commandStatus.idlePolls;
    commandStatus.idleReadUs += elapsed;
    if (elapsed > commandStatus.idleReadMaxUs) commandStatus.idleReadMaxUs = elapsed;
    if (sensorDevice.sample().imuFresh && sensorDevice.sample().imuValid) ++commandStatus.idleImuSamples;
    if (sensorDevice.sample().baroFresh && sensorDevice.sample().baroValid) ++commandStatus.idleBaroSamples;
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
  if (owner && !runWasActive) { // A run begins: loop and run evidence restart for it.
    diagnostics.loop = {}; diagnostics.loopOverPeriod = 0; diagnostics.loopMaxRunMs = 0; diagnostics.loopMaxPhase = -1;
    diagnostics.loopMaxUsbInput = false; runStartMs = millis(); runFifoBase = sensorDevice.imuFifoOverruns(); lastRun = owner;
    diagnostics.run = owner == &physicalFlight ? "physical" : owner == &standaloneSimulation.controller() ? "simulation" : "hil";
  }
  if (owner == &physicalFlight) {
    // The IMU is not read while the recording is prepared (initial erase on core 1), and the
    // overrun this leaves is detected by the first read after it. Count from the first processed
    // frame, the one that re-anchors the timing; later overruns are real losses.
    const auto state = physicalFlight.state();
    if (state == flight::ArmingState::Preparing || (state == flight::ArmingState::Armed && physicalFlight.runtime().timing().step.count <= 1))
      runFifoBase = sensorDevice.imuFifoOverruns();
  }
  if (!owner && runWasActive) runFifoEnd = sensorDevice.imuFifoOverruns(); // The run ends: freeze its count.
  runWasActive = owner != nullptr;
  commandStatus.armingState = owner ? flight::armingStateName(owner->state()) : "disarmed";
  commandStatus.flightMode = owner ? flight::flightModeName(owner->mode()) : "none";
  commandStatus.outputMode = (owner && (owner->mode()==flight::FlightMode::Physical || owner->runtime().status().outputMode == storage::OutputMode::Physical)) || outputExecutor.hasPreparation() ? "PHYSICAL" : "VIRTUAL";
  commandStatus.outputsActive = (owner && owner->runtime().status().outputsActive) || outputExecutor.hasPreparation();
  commandStatus.phase = owner && owner->navigation().valid ? int(owner->navigation().phase) : -1;
  commandStatus.attitudeValid = commandStatus.phase >= 0;
  if (commandStatus.attitudeValid) {
    const auto &n = owner->navigation(); commandStatus.tiltDeg = n.tiltRadians * 57.2957795f;
    for (unsigned i = 0; i < 3; ++i) commandStatus.upBody[i] = n.upBody[i];
  }
  commandStatus.gyroSamples = owner ? owner->calibrationSamples() : 0;
  commandStatus.gyroTarget = flight::ArmingCalibration::TargetSamples;
  updateHardwareStatus();
  const auto ledMode=power.sleeping() ? LedMode::PowerSaving : (physicalFlight.state()==flight::ArmingState::Arming || physicalFlight.state()==flight::ArmingState::Preparing) ? LedMode::Calibrating :
      commandStatus.phase==5 ? LedMode::Boost : commandStatus.phase==6 ? LedMode::Coast :
      commandStatus.phase==7 ? LedMode::Abort : commandStatus.phase>=0 ? static_cast<LedMode>(commandStatus.phase) : LedMode::Disarmed;
  statusLed.update(millis(),ledMode);
  pollBluetooth();
  // Both endpoints get a bounded turn every loop; neither waits for TX space.
  // USB remains normally active even while the radio is absent or backpressured.
  if (connected) {
    sendPending(Serial, usbEndpoint);
    for (unsigned budget = 0; budget < 64 && !usbEndpoint.pending() && Serial.available() > 0; ++budget, loopUsbInput = true)
      usbEndpoint.feed(static_cast<char>(Serial.read()), commandStatus);
  }
  router.pollPower(); // Five minutes idle; no sleep during flight, writes, previews or active transfers.
}
#endif // MAIN
