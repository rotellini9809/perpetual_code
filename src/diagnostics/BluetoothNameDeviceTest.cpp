#include "diagnostics/BuildMode.h"
#ifdef BLUETOOTH_NAME_TEST
#include <Arduino.h>
#include <hardware/gpio.h>
#include "hardware/board_pins.h"
#include "diagnostics/BluetoothTestProtocol.h"
#include "communication/CommandProtocol.h"
#include "hardware/BluetoothName.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
// Bench-only, deliberately synchronous raw UART test. No flight runtime, radio
// echo, model discovery state machine or external-flash access. GP4 TX / GP5 RX
// are UART1 (Serial2). Pin directions refer to the MCU, not the module.
auto &btSerial = Serial2;
bluetooth_test::Line usbLine;
uint32_t baud = 9600;
bool gateHigh = false, usbConnected = false;
const char *ending = "\r\n", *endingName = "crlf";

struct Reply {
  uint8_t bytes[256]{};
  size_t size = 0;
  uint32_t total = 0;
  bool overflow = false;

  // Compare complete lines, including replies without a final CR/LF. Never
  // interpret a substring, binary input or truncated buffer as a successful ACK.
  bool matches(const char *expected, const char *key = nullptr, const char *value = nullptr) const {
    if (overflow || total != size) return false;
    bool found = false;
    size_t start = 0;
    for (size_t i=0;i<=size;++i) {
      if (i<size && bytes[i]!='\r' && bytes[i]!='\n') {
        if (bytes[i]<32 || bytes[i]>126) return false;
        continue;
      }
      const size_t length = i-start;
      const char *line = reinterpret_cast<const char *>(bytes+start);
      if ((length>=4 && !std::memcmp(line,"+ERR",4)) ||
          (length>=5 && !std::memcmp(line,"ERROR",5))) return false;
      bool equal = !key && length==std::strlen(expected) && !std::memcmp(line,expected,length);
      if (key) {
        const auto n = std::strlen(key);
        const bool tagged = length>n && !std::memcmp(line,key,n) && (line[n]=='=' || line[n]==':');
        equal = tagged && length==n+1+std::strlen(value) && !std::memcmp(line+n+1,value,std::strlen(value));
        if (tagged && !equal) return false;
      }
      found |= equal;
      start = i+1;
    }
    return found;
  }
  bool acknowledged() const { return matches("+OK") || matches("OK"); }
};
Reply reply; // Static storage, no large buffers on the device stack.

void dump(const char *label,const uint8_t *bytes,size_t size) {
  Serial.printf("%s_HEX",label);
  for (size_t i=0;i<size;++i) Serial.printf(" %02X",unsigned(bytes[i]));
  Serial.println();
  Serial.printf("%s_ASCII \"",label);
  for (size_t i=0;i<size;++i) {
    const auto c = bytes[i];
    if (c=='\r') Serial.print("\\r");
    else if (c=='\n') Serial.print("\\n");
    else if (c=='\\' || c=='\"') { Serial.print('\\'); Serial.print(char(c)); }
    else if (c>=32 && c<=126) Serial.print(char(c));
    else Serial.printf("\\x%02X",unsigned(c));
  }
  Serial.println("\"");
}
void status() {
  Serial.printf("BT_NAME_TEST status baud=%lu rx_pin=%u tx_pin=%u pwrc_pin=%u uart_ready=%u rx_level=%u pwrc_level=%u eol=%s\n",
      (unsigned long)baud,unsigned(RX_BLE),unsigned(TX_BLE),unsigned(PWRC_BLE),
      unsigned(bool(btSerial)),unsigned(digitalRead(RX_BLE)),unsigned(digitalRead(PWRC_BLE)),endingName);
}
void help() {
  Serial.println("status | probe | at AT | at AT+VERSION | at AT+NAME | at AT+NAMB");
  Serial.println("at AT+STAT | at AT+LADDR | at AT+BTMODE | module reboot (AT+RESET; drops radio links)");
  Serial.println("baud <rate> (MCU only) | gate low|high | eol crlf|cr|lf|none");
  Serial.println("name set \"Perpetual\" : AT -> NAME set/read -> NAMB set/read; module memory only.");
}
void exchange(const char *command) {
  // Log pre-existing bytes separately; they cannot acknowledge this request.
  reply = {};
  const auto drainAt = millis();
  while (btSerial.available() && uint32_t(millis()-drainAt)<50) {
    const auto c = btSerial.read();
    if (reply.size<sizeof(reply.bytes)) reply.bytes[reply.size++] = uint8_t(c);
    ++reply.total;
  }
  if (reply.total) {
    dump("STALE",reply.bytes,reply.size);
    Serial.printf("STALE_BYTES %lu\n",(unsigned long)reply.total);
  }
  reply = {}; btSerial.overflow();
  char tx[64]{};
  std::snprintf(tx,sizeof(tx),"%s%s",command,ending);
  Serial.printf("QUERY_BEGIN %s\n",command);
  dump("TX",reinterpret_cast<const uint8_t *>(tx),std::strlen(tx));
  btSerial.print(tx); btSerial.flush();
  const auto start = millis();
  auto lastRx = start;
  do {
    while (btSerial.available()) {
      const auto c = btSerial.read();
      if (reply.size<sizeof(reply.bytes)) reply.bytes[reply.size++] = uint8_t(c);
      ++reply.total; lastRx = millis();
      // Bound even a continuous stream of unsolicited traffic.
      if (uint32_t(millis()-start)>=1200) break;
    }
    if (reply.total && uint32_t(millis()-lastRx)>=100) break;
    delay(1);
  } while (uint32_t(millis()-start)<1200);
  reply.overflow = btSerial.overflow();
  dump("RX",reply.bytes,reply.size);
  Serial.printf("QUERY_END command=%s rx_bytes=%lu captured=%u overflow=%u ack=%u elapsed_ms=%lu\n",
      command,(unsigned long)reply.total,unsigned(reply.size),unsigned(reply.overflow),unsigned(reply.acknowledged()),(unsigned long)(millis()-start));
}
void rename(const char *name) {
  if (!bluetooth::NameController::validName(name)) { Serial.println("ERROR INVALID_NAME"); return; }
  bool mayHaveChanged = false;
  const char *step = "AT";
  exchange("AT");
  bool ok = reply.acknowledged();
  if (!ok && !reply.total && !reply.overflow) {
    Serial.println("AT_HANDSHAKE_RETRY (read-only, one retry)");
    exchange("AT"); ok = reply.acknowledged();
  }
  for (const char *key : {"NAME","NAMB"}) {
    if (!ok) break;
    char command[32]{}, prefix[8]{};
    std::snprintf(command,sizeof(command),"AT+%s%s",key,name);
    step = !std::strcmp(key,"NAME") ? "set_spp" : "set_ble";
    mayHaveChanged = true;
    exchange(command); ok = reply.acknowledged();
    if (!ok) break;
    std::snprintf(command,sizeof(command),"AT+%s",key);
    std::snprintf(prefix,sizeof(prefix),"+%s",key);
    step = !std::strcmp(key,"NAME") ? "read_spp" : "read_ble";
    exchange(command); ok = reply.matches(nullptr,prefix,name);
  }
  Serial.printf("NAME_RESULT verified=%u step=%s may_have_changed=%u rx_bytes=%lu\n",
      unsigned(ok),ok ? "complete" : step,unsigned(mayHaveChanged),(unsigned long)reply.total);
}
void command(const char *line) {
  if (!std::strcmp(line,"status")) { status(); return; }
  if (!std::strcmp(line,"help")) { help(); return; }
  if (!std::strcmp(line,"probe")) {
    for (const char *q : {"AT","AT+VERSION","AT+NAME","AT+NAMB"}) exchange(q);
    Serial.println("PROBE_END"); return;
  }
  if (!std::strcmp(line,"module reboot")) {
    digitalWrite(PWRC_BLE,LOW); gateHigh = false; delay(250);
    exchange("AT");
    if (!reply.total && !reply.overflow) exchange("AT");
    if (!reply.acknowledged()) { Serial.println("MODULE_REBOOT requested=0 ack=0"); return; }
    exchange("AT+RESET");
    const bool ack = reply.acknowledged();
    // Reboot is explicit and never retried. An ACK alone cannot prove reboot or
    // persistence: allow boot time, then the caller must probe/read names again.
    delay(1500);
    Serial.printf("MODULE_REBOOT requested=1 ack=%u\n",unsigned(ack)); return;
  }
  if (!std::strncmp(line,"at ",3) && bluetooth_test::queryAllowed(line+3)) {
    exchange(line+3); return;
  }
  if (!std::strncmp(line,"baud ",5)) {
    char *end = nullptr; const auto value = std::strtoul(line+5,&end,10);
    if (line[5]<'0' || line[5]>'9' || *end || !bluetooth_test::baudAllowed(value)) { Serial.println("ERROR INVALID_BAUD"); return; }
    btSerial.end(); btSerial.begin(value); baud = value;
    Serial.printf("LOCAL_BAUD %lu\n",(unsigned long)baud); return;
  }
  if (!std::strcmp(line,"gate low") || !std::strcmp(line,"gate high")) {
    gateHigh = !std::strcmp(line,"gate high"); digitalWrite(PWRC_BLE,gateHigh ? HIGH : LOW);
    delay(250); Serial.printf("GATE %s\n",gateHigh ? "high" : "low"); return;
  }
  if (!std::strncmp(line,"eol ",4)) {
    if (!std::strcmp(line+4,"crlf")) { ending="\r\n"; endingName="crlf"; }
    else if (!std::strcmp(line+4,"cr")) { ending="\r"; endingName="cr"; }
    else if (!std::strcmp(line+4,"lf")) { ending="\n"; endingName="lf"; }
    else if (!std::strcmp(line+4,"none")) { ending=""; endingName="none"; }
    else { Serial.println("ERROR INVALID_EOL"); return; }
    Serial.printf("EOL %s\n",endingName); return;
  }
  if (!std::strncmp(line,"name ",5)) {
    protocol::Tokens tokens(line);
    if (!tokens.valid || tokens.count!=3 || !tokens.is(1,"set")) { Serial.println("ERROR INVALID_ARGUMENT"); return; }
    rename(tokens[2]); return;
  }
  Serial.println("ERROR UNKNOWN_COMMAND");
}
void output(uint8_t pin,bool high) { gpio_init(pin); gpio_put(pin,high); gpio_set_dir(pin,GPIO_OUT); }
}
void setup() {
  for (uint8_t pin : {PYRO_ENABLE,PYRO_1,PYRO_2,PYRO_3,PYRO_4,SERVO_PWM_1,SERVO_PWM_2,SERVO_PWM_3,SERVO_PWM_4}) output(pin,false);
  for (uint8_t pin : {CS_FLASH1,CS_IMU,CS_BARO}) output(pin,true);
  output(PWRC_BLE,false);
  Serial.begin(115200);
  btSerial.setTX(TX_BLE); btSerial.setRX(RX_BLE); btSerial.setFIFOSize(256); btSerial.begin(baud);
}
void loop() {
  const bool connected = bool(Serial);
  if (connected!=usbConnected) {
    usbConnected = connected; usbLine.clear();
    if (connected) {
      Serial.println("=== BLUETOOTH_NAME_TEST JDY-34 | raw hardware UART1 | outputs inactive ===");
      status(); help();
    }
  }
  if (!connected) return;
  for (unsigned budget=0;budget<32 && Serial.available();++budget) {
    const auto result = usbLine.feed(static_cast<unsigned char>(Serial.read()));
    if (result==bluetooth_test::LineResult::Ready) { command(usbLine.text()); break; }
    if (result==bluetooth_test::LineResult::Rejected) { Serial.println("ERROR INVALID_LINE"); break; }
  }
}
#endif
