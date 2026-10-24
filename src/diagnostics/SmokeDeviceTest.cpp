#include "diagnostics/BuildMode.h"
#ifdef SMOKE_TEST
#include <Adafruit_NeoPixel.h>
#include "hardware/board_pins.h"
#include "hardware/BoardOutputs.h"
#include "hardware/Sensors.h"
#include "storage/FlashLogger.h"
#include "configuration/ActuatorPolicy.h"
#include "communication/CommandService.h"
#include "communication/CommandProtocol.h"

// Port of origin/main e65ce64's board smoke test. Uses current drivers and the
// production output timer; detailed raw SPI sweeps remain in SENSOR_TEST.
namespace {
Adafruit_NeoPixel led(1, RGB_LED, NEO_RGB + NEO_KHZ800);
PyroController pyros;
ServoController servos;
BoardOutputs hardware(pyros,servos);
uint32_t clockUs() { return micros(); }
flight::OutputExecutor outputs(hardware,clockUs);
recovery::Configuration testPlan;
constexpr uint64_t TestOwner=1;
Sensors sensors;
SPIClassRP2040 memorySpi(FLASH_SPI_PORT,MISO_FLASH,CS_FLASH1,SCK_FLASH,MOSI_FLASH);
FlashLogger flash(memorySpi,CS_FLASH1);
bool connected=false, memoryOk=false, fired=false, imuSeen=false, baroSeen=false, discardLine=false;
uint32_t lastRead=0, lastReport=0, imuAt=0, baroAt=0;

void color(uint8_t r,uint8_t g,uint8_t b) { led.setPixelColor(0,led.Color(r,g,b)); led.show(); }
const char *continuityName(PyroController::ContinuityState state) {
  switch (state) {
    case PyroController::ContinuityState::Connected: return "CONNECTED";
    case PyroController::ContinuityState::Open: return "OPEN";
    case PyroController::ContinuityState::BatteryNotConnected: return "BATTERY NOT CONNECTED";
    default: return "UNCERTAIN";
  }
}
void report() {
  const auto now=millis(); const auto &sample=sensors.sample();
  const bool imuValid=imuSeen && sample.imuValid && uint32_t(now-imuAt)<=100;
  const bool baroValid=baroSeen && sample.baroValid && uint32_t(now-baroAt)<=100;
  Serial.printf("t_ms=%lu | FLASH identified=%u | IMU ready=%u valid=%u | BARO ready=%u valid=%u\n",
      (unsigned long)now,memoryOk,sensors.imuOk(),imuValid,sensors.baroOk(),baroValid);
  if (imuValid)
    Serial.printf("IMU a[m/s2]=%.3f,%.3f,%.3f w[rad/s]=%.4f,%.4f,%.4f T=%.2fC\n",
        sample.accelMps2[0],sample.accelMps2[1],sample.accelMps2[2],
        sample.gyroRadps[0],sample.gyroRadps[1],sample.gyroRadps[2],sample.imuTempC);
  else Serial.printf("IMU sample unavailable | error=%ld\n",long(sensors.lastImuError()));
  if (baroValid) Serial.printf("BARO P=%.3fPa T=%.3fC\n",sample.pressurePa,sample.baroTempC);
  else Serial.printf("BARO sample unavailable | error=%d\n",int(sensors.lastBaroError()));
  for (uint8_t channel=1;channel<=4;++channel) {
    const auto reading=pyros.checkContinuity(channel);
    Serial.printf("PYRO %u continuity=%u / %s\n",channel,reading.raw,continuityName(reading.state));
  }
  const bool healthy=memoryOk && sensors.ok() && imuValid && baroValid;
  color(healthy ? 0 : 80,healthy ? 80 : 0,0);
}
void diagnostics() {
  color(0,0,255);
  Serial.println("\n=== SMOKE_TEST: board identification and live readings ===");
  // Reinitialization changes volatile sensor setup only; no sensor NVM/flash writes.
  // BRING-UP: the known IMU solder fault remains FAIL; never replace missing measurements.
  sensors.begin(); imuSeen=baroSeen=false;
  memoryOk=flash.initialize(FLASH_CLOCK_HZ);
  Serial.printf("FLASH JEDEC=0x%06lX expected EF4018/EF7018 | %s | error=%s (identification only)\n",
      (unsigned long)flash.readJEDEC(),memoryOk ? "PASS" : "FAIL",FlashLogger::errorName(flash.lastError()));
  Serial.printf("IMU ID=0x%02X expected 70 | %s | error=%ld\n",sensors.imuWhoAmI(),
      sensors.imuOk() ? "PASS" : "FAIL",long(sensors.lastImuError()));
  Serial.printf("BARO ID=0x%02X raw=0x%02X expected 50 | %s | error=%d | SPI mode=%u Hz=%lu\n",
      sensors.baroChipId(),sensors.baroRawChipId(),sensors.baroOk() ? "PASS" : "FAIL",int(sensors.lastBaroError()),
      sensors.baroSpiMode()==SPI_MODE3 ? 3 : 0,(unsigned long)sensors.baroSpiClockHz());
  Serial.println(memoryOk && sensors.ok() ? "Identification checks PASS" : "Identification checks FAIL: see individual devices");
  Serial.println("No flight/configuration flash writes. Servo PWM detached. Bluetooth/flight service inactive.");
  Serial.println("D: repeat identification/init | F: PHYSICAL pyro 1 pulse, once per boot | H: help (then Enter)");
  Serial.printf("F uses production timing: %lu ms enable settling, %lu ms pulse. Continuity is reported, not a firing interlock.\n",
      (unsigned long)recovery::PyroEnableSettleMs,(unsigned long)recovery::PyroPulseMs);
  Serial.println("For raw register/mode/speed diagnostics select SENSOR_TEST.");
  lastRead=lastReport=millis();
}
const char *fire() {
  if (fired) return "ALREADY_FIRED";
  if (const auto error=outputs.begin(testPlan,TestOwner)) return error;
  if (!outputs.submit({1,recovery::OutputKind::Pyro,1,0})) {
    outputs.stop(TestOwner); return "OUTPUT_ERROR";
  }
  fired=true;
  return nullptr;
}
class SmokeCommands : public CommandExtension {
  bool dispatch(const char *command,const char *arguments,bool hasId,uint32_t id,char *reply,std::size_t capacity) override {
    protocol::Tokens tokens(arguments); const char *error=nullptr;
    if (!tokens.valid || tokens.count) error="INVALID_ARGUMENT";
    else if (!strcmp(command,"F") || !strcmp(command,"f")) error=fire();
    else if (!strcmp(command,"D") || !strcmp(command,"d")) diagnostics();
    else if (!strcmp(command,"H") || !strcmp(command,"h"))
      Serial.println("D repeat identification/init | F PHYSICAL pyro 1 once per boot; use SENSOR_TEST for raw SPI diagnostics.");
    else error="UNKNOWN_COMMAND";
    return protocol::finish(command,protocol::Json{},error,hasId,id,reply,capacity,true);
  }
} commands;
CommandService input;
const CommandService::Status unusedStatus;
char reply[CommandService::MaxReply];
}

void setup() {
  hardware.begin(); // Pyros disabled; all servo PWM detached before waiting for USB.
  for (uint8_t pin : {CS_IMU,CS_BARO}) pinMode(pin,INPUT_PULLUP);
  digitalWrite(CS_FLASH1,HIGH); pinMode(CS_FLASH1,OUTPUT);
  testPlan.ruleCount=1; auto &rule=testPlan.rules[0]; rule.id=1;
  snprintf(rule.name.data(),rule.name.size(),"Smoke pyro 1");
  rule.conditionCount=rule.assignmentCount=1; rule.assignments[0].channel=1;
  testPlan.pyros[0].pulseMs=recovery::PyroPulseMs;
  input.setExtension(&commands); // Whole lines only: an app's "info" must never be interpreted as F.
  led.begin(); led.setBrightness(40); color(0,0,255); Serial.begin(115200);
}
void loop() {
  outputs.poll(); // Runs even if USB disconnects during the pulse.
  if (outputs.owner()) {
    if (outputs.pulsing()) {
      for (unsigned budget=0;budget<32 && Serial.available();++budget) discardLine=Serial.read()!='\n';
      return; // Input during firing is discarded, never deferred into another action.
    }
    outputs.stop(TestOwner);
    if (Serial) Serial.println("PYRO 1 pulse complete; pyros disabled.");
  }
  if (!Serial) { connected=false; discardLine=false; input.reset(); return; }
  if (!connected) { connected=true; diagnostics(); }
  if (Serial.available()) {
    const char byte=char(Serial.read());
    if (discardLine) discardLine=byte!='\n';
    else if (input.feed(byte,unusedStatus,reply,sizeof(reply))) Serial.print(reply);
    if (outputs.owner()) return; // The reply is sent before enable/pulse timing starts on the next loop.
  }
  const auto now=millis();
  if (uint32_t(now-lastRead)>=5) {
    lastRead=now; sensors.read(); const auto &sample=sensors.sample();
    if (sample.imuFresh) { imuSeen=true; imuAt=now; }
    if (sample.baroFresh) { baroSeen=true; baroAt=now; }
  }
  if (uint32_t(now-lastReport)>=1000) { lastReport=now; report(); }
}
#endif
