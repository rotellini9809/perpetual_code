#include "diagnostics/BuildMode.h"
#ifdef BLUETOOTH_TEST
#include <Arduino.h>
#include <hardware/gpio.h>
#include "hardware/board_pins.h"
#include "diagnostics/BluetoothTestProtocol.h"
#include "hardware/BluetoothName.h"
#include "communication/CommandProtocol.h"
#include <cstdio>
#include <cstdlib>

namespace {
// MCU RX=GP5 receives module TXD; MCU TX=GP4 drives module RXD (UART1).
auto &btSerial = Serial2;
bluetooth_test::Line usbLine, btLine;
bluetooth_test::LinkSnapshot link;
bluetooth::NameController bluetoothName;
uint32_t baud = 9600, rxBytes = 0, txBytes = 0, overflows = 0, rejectedLines = 0;
uint32_t atStart = 0, lastRx = 0, atRxStart = 0, linkAt = 0, radioPings = 0;
bool atBusy = false, atMatched = false, usbConnected = false;
bool atGateHigh = false;
bool nameCommand = false;
char atQuery[32]{};

void sendRadio(const char *text) {
  btSerial.print(text); btSerial.print("\r\n"); txBytes += std::strlen(text)+2;
}
void status() {
  Serial.printf("BT_TEST status baud=%lu rx_pin=%u tx_pin=%u pwrc_pin=%u uart_ready=%u rx_level=%u pwrc_level=%u at_gate_high=%u at_busy=%u rx_bytes=%lu tx_bytes=%lu overflows=%lu rejected_lines=%lu radio_pings=%lu link_known_mask=%u link_connected_mask=%u link_age_ms=%lu\n",
      (unsigned long)baud,unsigned(RX_BLE),unsigned(TX_BLE),unsigned(PWRC_BLE),
      unsigned(bool(btSerial)),unsigned(digitalRead(RX_BLE)),unsigned(digitalRead(PWRC_BLE)),unsigned(atGateHigh),unsigned(atBusy),
      (unsigned long)rxBytes,(unsigned long)txBytes,(unsigned long)overflows,(unsigned long)rejectedLines,
      (unsigned long)radioPings,unsigned(link.known),unsigned(link.connected),
      (unsigned long)(link.known ? uint32_t(millis()-linkAt) : 0));
}
void help() {
  Serial.println("USB: status | baud <rate> (local UART only) | gate low|high (AT PWRC level) | at <query> | send <text> | help");
  Serial.println("Queries: AT AT+VERSION AT+NAME AT+NAMB AT+BAUD AT+STAT AT+LADDR AT+BTMODE");
  Serial.println("Phone/BT terminal: ping -> PONG; echo <text> -> ECHO <text>. End with CR or LF.");
  Serial.println("Name: name set \"Perpetual\" | name show. Explicit rename uses module memory only, after model identification/readback.");
}
void nameStatus(const char *prefix) {
  if (!Serial) return;
  protocol::Json json;
  json.add("{\"state\":\"%s\",\"verified\":%s,\"model\":\"%s\",\"requested\":",bluetoothName.stateName(),
      bluetoothName.verified() ? "true" : "false",bluetoothName.modelName());
  json.quoted(bluetoothName.requestedName()); json.add(",\"name\":");
  if (bluetoothName.verified()) json.quoted(bluetoothName.verifiedName()); else json.add("null");
  json.add(",\"may_have_changed\":%s,\"error\":\"%s\",\"storage\":\"module\"}",
      bluetoothName.mayHaveChanged() ? "true" : "false",bluetoothName.error());
  Serial.printf("%s %s\n",prefix,json.bytes);
}
void beginQuery(const char *query,bool forName = false) {
  // Discard old UART bytes before issuing the query; unsolicited traffic alone
  // cannot pass a probe. PWRC low selects AT access, high restores transparent data.
  for (unsigned n=0;n<256 && btSerial.available();++n) btSerial.read();
  btLine.clear(); std::strcpy(atQuery,query); atMatched = false; nameCommand = forName;
  if (!std::strcmp(query,"AT+STAT")) link = {};
  digitalWrite(PWRC_BLE,atGateHigh ? HIGH : LOW); delay(20);
  atStart = lastRx = millis(); atRxStart = rxBytes; atBusy = true;
  Serial.printf("AT_BEGIN %s\n",atQuery); sendRadio(atQuery);
}
void command(const char *line) {
  if (!std::strcmp(line,"status")) { status(); return; }
  if (!std::strcmp(line,"help")) { help(); return; }
  if (!std::strcmp(line,"name show")) { nameStatus("NAME_STATUS"); return; }
  if (atBusy || bluetoothName.active()) { Serial.println("ERROR BUSY"); return; }
  if (!std::strncmp(line,"name ",5)) {
    protocol::Tokens tokens(line);
    if (!tokens.valid || tokens.count != 3 || !tokens.is(1,"set")) { Serial.println("ERROR INVALID_ARGUMENT"); return; }
    if (const auto error = bluetoothName.setName(tokens[2])) Serial.printf("ERROR %s\n",error);
    else Serial.println("NAME_BEGIN accepted (wait for NAME_RESULT)");
    return;
  }
  if (!std::strcmp(line,"gate low") || !std::strcmp(line,"gate high")) {
    atGateHigh = !std::strcmp(line,"gate high"); Serial.printf("AT_GATE %s\n",atGateHigh ? "high" : "low"); return;
  }
  if (!std::strncmp(line,"baud ",5)) {
    char *end = nullptr; const auto value = std::strtoul(line+5,&end,10);
    if (line[5] < '0' || line[5] > '9' || *end || !bluetooth_test::baudAllowed(value)) { Serial.println("ERROR INVALID_BAUD"); return; }
    btSerial.end(); btSerial.begin(value); baud = value; btLine.clear(); link = {};
    Serial.printf("LOCAL_BAUD %lu\n",(unsigned long)baud); return;
  }
  if (!std::strncmp(line,"at ",3) && bluetooth_test::queryAllowed(line+3)) { beginQuery(line+3); return; }
  if (!std::strncmp(line,"send ",5) && line[5]) {
    // A disconnected module may interpret text as AT. Keep this diagnostic free
    // of raw setters, reset, pairing changes and remote disconnect commands.
    const char *payload = line+5;
    while (*payload == ' ') ++payload;
    if (!*payload || !std::strncmp(payload,"AT",2)) { Serial.println("ERROR PAYLOAD"); return; }
    sendRadio(payload); Serial.println("RADIO_TX queued; reception is not verified by this acknowledgment"); return;
  }
  Serial.println("ERROR UNKNOWN_COMMAND_OR_QUERY");
}
void received(bluetooth_test::LineResult result) {
  if (result == bluetooth_test::LineResult::Rejected) {
    ++rejectedLines; if (Serial) Serial.println("BT_RX rejected (non-ASCII or oversized line)"); return;
  }
  if (result != bluetooth_test::LineResult::Ready) return;
  const char *text = btLine.text();
  if (Serial) Serial.printf("BT_RX %s\n",text);
  if (atBusy) {
    if (nameCommand ? bluetoothName.acceptReply(text) : bluetooth_test::queryReply(atQuery,text)) {
      atMatched = true;
      if (!std::strcmp(atQuery,"AT+STAT") && link.add(text)) linkAt = millis();
    }
  } else if (!std::strcmp(text,"ping")) { sendRadio("PONG"); ++radioPings; }
  else if (!std::strncmp(text,"echo ",5)) {
    // The response prefix prevents generic echo loops between terminals.
    btSerial.print("ECHO "); txBytes += 5; sendRadio(text+5);
  }
}
void inactiveOutputs() {
  for (uint8_t pin : {PYRO_ENABLE,PYRO_1,PYRO_2,PYRO_3,PYRO_4,
                      SERVO_PWM_1,SERVO_PWM_2,SERVO_PWM_3,SERVO_PWM_4}) {
    gpio_init(pin); gpio_put(pin,0); gpio_set_dir(pin,GPIO_OUT);
  }
  // Arduino-Pico pinMode(OUTPUT) resets the output latch. Initialize first,
  // preload the safe level, then enable the output direction, without a glitch.
  for (uint8_t pin : {CS_FLASH1,CS_IMU,CS_BARO}) { gpio_init(pin); gpio_put(pin,1); gpio_set_dir(pin,GPIO_OUT); }
}
}
void setup() {
  inactiveOutputs();
  gpio_init(PWRC_BLE); gpio_put(PWRC_BLE,1); gpio_set_dir(PWRC_BLE,GPIO_OUT);
  Serial.begin(115200);
  btSerial.setTX(TX_BLE); btSerial.setRX(RX_BLE); btSerial.setFIFOSize(256); btSerial.begin(baud);
}
void loop() {
  const bool connected = bool(Serial);
  if (connected != usbConnected) {
    usbConnected = connected; usbLine.clear();
    if (connected) {
      Serial.println("=== BLUETOOTH_TEST JDY-34 | hardware UART1 | outputs inactive ===");
      Serial.println("Flash/sensors untouched. Only explicit name set changes module settings. AT+STAT is a snapshot.");
      status(); help();
    }
  }
  if (btSerial.overflow()) ++overflows;
  for (unsigned budget=0;budget<64 && btSerial.available();++budget) {
    const auto c = btSerial.read(); ++rxBytes; lastRx = millis();
    received(btLine.feed(static_cast<unsigned char>(c)));
  }
  // JDY firmware variants may omit CR/LF on replies. Finish an AT-only partial
  // line after a quiet interval; radio payloads always require an explicit newline.
  if (atBusy && btLine.pending() && uint32_t(millis()-lastRx)>=40) received(btLine.feed('\n'));
  if (atBusy && ((atMatched && uint32_t(millis()-lastRx)>=60) || uint32_t(millis()-atStart)>=800)) {
    atBusy = false; digitalWrite(PWRC_BLE,HIGH); btLine.clear();
    if (nameCommand) {
      bluetoothName.finishCommand();
      if (!bluetoothName.active()) nameStatus("NAME_RESULT");
    } else if (Serial) Serial.printf("AT_RESULT ok=%u command=%s rx_bytes=%lu\n",unsigned(atMatched),atQuery,(unsigned long)(rxBytes-atRxStart));
  }
  if (!atBusy && bluetoothName.active()) {
    bluetooth::NameController::Command next;
    if (bluetoothName.nextCommand(next)) beginQuery(next.text,true);
  }
  if (!connected) return; // Phone ping/echo works independently of the USB terminal.
  for (unsigned budget=0;budget<32 && Serial.available();++budget) {
    const auto result = usbLine.feed(static_cast<unsigned char>(Serial.read()));
    if (result == bluetooth_test::LineResult::Ready) { command(usbLine.text()); break; }
    if (result == bluetooth_test::LineResult::Rejected) { Serial.println("ERROR INVALID_LINE"); break; }
  }
}
#endif
