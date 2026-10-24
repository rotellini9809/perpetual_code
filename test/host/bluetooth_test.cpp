#include <cstdio>
#include "diagnostics/BluetoothTestProtocol.h"
#include <cassert>
#include <string>
#include "hardware/BluetoothName.h"
#include <cstring>
#include "hardware/Jdy34Link.h"

// Each namespace isolates one component scenario and its fault model.
namespace test_bluetooth_diagnostic {
using namespace bluetooth_test;
void run() {
  for (const char *q : {"AT","AT+VERSION","AT+NAME","AT+NAMB","AT+BAUD","AT+STAT","AT+LADDR","AT+BTMODE"}) assert(queryAllowed(q));
  for (const char *q : {"AT+RESET","AT+DEFAULT","AT+BAUD8","AT+NAMEtest","AT+PIN1234","AT+DISC","AT\r\nAT+RESET","AT ","at",""}) assert(!queryAllowed(q));
  assert(baudAllowed(9600) && baudAllowed(115200) && baudAllowed(1000000));
  assert(!baudAllowed(0) && !baudAllowed(9999));
  Line input;
  for (char c : std::string("AT+VERSION")) assert(input.feed(c)==LineResult::None);
  assert(input.pending() && input.feed('\r')==LineResult::Ready && !strcmp(input.text(),"AT+VERSION"));
  assert(input.feed('\n')==LineResult::None && !input.pending());
  for (unsigned i=0;i<Line::Capacity;++i) assert(input.feed('x')==LineResult::None);
  assert(input.feed('\n')==LineResult::Ready && strlen(input.text())==Line::Capacity);
  for (unsigned i=0;i<Line::Capacity+1;++i) input.feed('x');
  assert(input.feed('\n')==LineResult::Rejected);
  input.feed('a'); input.feed(0); input.feed('b'); assert(input.feed('\n')==LineResult::Rejected);
  input.feed(255); assert(input.feed('\r')==LineResult::Rejected);
  for (char c : std::string("ping")) input.feed(c);
  assert(input.feed('\n')==LineResult::Ready && !strcmp(input.text(),"ping"));
  input.feed('x'); input.clear(); assert(!input.pending());
  assert(queryReply("AT","+OK") && queryReply("AT","OK"));
  for (const char *noise : {"AT","+CONNECTED-ID=0","garbage","OKAY","+OK noise",""}) assert(!queryReply("AT",noise));
  assert(queryReply("AT+VERSION","+VERSION=JDY-34-V1.3"));
  assert(queryReply("AT+VERSION","+VERSION:JDY-34-V1.3"));
  assert(queryReply("AT+VERSION","+JDY-34-V1.2"));
  assert(!queryReply("AT+VERSION","+JDY-340-V1.2"));
  for (const char *noise : {"+VERSION=","+VER=1","+NAME=JDY-34","+OK",""}) assert(!queryReply("AT+VERSION",noise));
  LinkSnapshot link;
  assert(link.add("+STAT=0,1") && link.known==1 && link.connected==1);
  assert(link.add("+STAT=1,0") && link.known==3 && link.connected==1);
  assert(link.add("+STAT=0,0") && link.connected==0);
  assert(link.add("+STAT=7,1") && link.connected==128);
  for (const char *bad : {"+STAT=8,1","+STAT=0,2","+STAT=01","+STAT=0,1x","+STAT=0,","+STAT=","", "+STAT=0,1\r\n"}) {
    const auto before=link; assert(!link.add(bad) && link.known==before.known && link.connected==before.connected);
    assert(!queryReply("AT+STAT",bad));
  }
  assert(queryReply("AT+STAT","+STAT=7,1"));
  puts("Bluetooth diagnostic passed: bounded framing, query-only whitelist, baud checks, exact replies and per-channel status snapshots.");
}
} // namespace test_bluetooth_diagnostic

namespace test_bluetooth_name {
using bluetooth::NameController;

std::string take(NameController &controller) {
  NameController::Command command; assert(controller.nextCommand(command));
  NameController::Command unchanged; std::strcpy(unchanged.text,"unchanged");
  assert(!controller.nextCommand(unchanged) && !strcmp(unchanged.text,"unchanged"));
  return command.text;
}
void reply(NameController &controller,const char *line) { assert(controller.acceptReply(line)); controller.finishCommand(); }
void identify(NameController &controller,const char *version) {
  assert(take(controller)=="AT"); reply(controller,"+OK");
  assert(take(controller)=="AT+VERSION");
  assert(!controller.acceptReply("+OK")); assert(!controller.acceptReply("+CONNECTED-ID=0"));
  reply(controller,version);
}
void run() {
  NameController controller;
  assert(!controller.active() && !controller.verified() && !controller.mayHaveChanged());
  for (const char *bad : {static_cast<const char *>(nullptr),""," "," leading","trailing ","1234567890123456789","bad\nAT+RESET","bad\rname","bad\tname","bad\x7f","bad\xc3\xa8"}) {
    assert(controller.setName(bad) && !controller.active() && !controller.mayHaveChanged());
  }
  const std::string full(18,'x');
  assert(!controller.setName(full.c_str()));
  assert(!strcmp(controller.setName("other"),"BUSY"));
  identify(controller,"+JDY-34-V1.52");
  assert(!strcmp(controller.modelName(),"JDY-34") && !controller.mayHaveChanged());
  assert(take(controller)=="AT+NAME"+full && controller.mayHaveChanged()); reply(controller,"+OK");
  assert(!controller.verified());
  assert(take(controller)=="AT+NAME"); reply(controller,("+NAME="+full).c_str());
  assert(!controller.verified()); // SPP succeeded; BLE must be verified too.
  assert(take(controller)=="AT+NAMB"+full); reply(controller,"OK");
  assert(take(controller)=="AT+NAMB"); reply(controller,("+NAMB="+full).c_str());
  assert(controller.verified() && !controller.active() && controller.verifiedName()==full);
  assert(!controller.acceptReply("+ERROR")); controller.finishCommand(); assert(controller.verified());
  // Only the module stores the name: a new controller does not restore a local copy.
  NameController afterBoot; assert(!*afterBoot.verifiedName() && !*afterBoot.requestedName());
  assert(!controller.setName("Perpetual 01"));
  identify(controller,"+VERSION:JDY-34-V1.52");
  assert(!strcmp(controller.modelName(),"JDY-34"));
  assert(take(controller)=="AT+NAMEPerpetual 01"); reply(controller,"+OK");
  assert(take(controller)=="AT+NAME"); reply(controller,"+NAME=Perpetual 01");
  assert(!controller.verified());
  assert(take(controller)=="AT+NAMBPerpetual 01"); reply(controller,"+OK");
  assert(take(controller)=="AT+NAMB"); reply(controller,"+NAMB=Perpetual 01");
  assert(controller.verified()); NameController::Command none; assert(!controller.nextCommand(none));
  assert(!controller.setName(controller.requestedName())); // Alias input remains valid.
  identify(controller,"+VERSION=JDY-340-unknown");
  assert(!controller.active() && !strcmp(controller.error(),"UNSUPPORTED_MODULE") && !controller.mayHaveChanged());
  assert(!controller.setName("Perpetual")); assert(take(controller)=="AT"); controller.finishCommand();
  assert(controller.active() && !controller.mayHaveChanged());
  assert(take(controller)=="AT"); controller.finishCommand();
  assert(!strcmp(controller.error(),"NO_RESPONSE") && !controller.mayHaveChanged() && !controller.verified());
  assert(!controller.setName("Perpetual")); identify(controller,"+VERSION=JDY-34-V1.3");
  assert(take(controller)=="AT+NAMEPerpetual"); controller.finishCommand();
  assert(!strcmp(controller.error(),"NO_RESPONSE") && controller.mayHaveChanged());
  assert(!controller.nextCommand(none)); // A lost ACK never causes automatic NVM retries.
  assert(!controller.setName("Perpetual")); identify(controller,"+VERSION=JDY-34-V1.3");
  take(controller); reply(controller,"+ERROR"); assert(!strcmp(controller.error(),"MODULE_REJECTED"));
  assert(!controller.setName("Perpetual")); identify(controller,"+VERSION=JDY-34-V1.3");
  take(controller); reply(controller,"+OK"); assert(take(controller)=="AT+NAME");
  reply(controller,"+NAME=Something else");
  assert(!controller.verified() && !strcmp(controller.error(),"NAME_READBACK_MISMATCH") && !controller.nextCommand(none));
  assert(!controller.setName("Perpetual")); identify(controller,"+VERSION=JDY-34-V1.3");
  take(controller); reply(controller,"+OK"); take(controller); reply(controller,"+NAME=Perpetual");
  assert(take(controller)=="AT+NAMBPerpetual"); reply(controller,"+OK"); take(controller);
  assert(controller.acceptReply("+NAMB=Old name"));
  assert(controller.acceptReply("+NAMB=Perpetual")); controller.finishCommand();
  assert(!controller.verified() && controller.mayHaveChanged() && !strcmp(controller.error(),"NAME_READBACK_MISMATCH"));
  // Actual fitted JDY-34 V1.52 trace: direct version, plain OK for NAME,
  // colon for its SPP readback, +OK and equals for BLE.
  assert(!controller.setName("Perpetual")); identify(controller,"+JDY-34-SPP-BLE-V1.52");
  assert(take(controller)=="AT+NAMEPerpetual"); reply(controller,"OK");
  assert(take(controller)=="AT+NAME"); reply(controller,"+NAME:Perpetual");
  assert(take(controller)=="AT+NAMBPerpetual"); reply(controller,"+OK");
  assert(take(controller)=="AT+NAMB"); reply(controller,"+NAMB=Perpetual");
  assert(controller.verified());
  assert(!controller.setName("Perpetual")); assert(take(controller)=="AT"); controller.finishCommand();
  assert(take(controller)=="AT"); reply(controller,"+OK");
  assert(take(controller)=="AT+VERSION"); reply(controller,"+JDY-34-SPP-BLE-V1.52");
  assert(take(controller)=="AT+NAMEPerpetual"); controller.finishCommand();
  assert(!strcmp(controller.error(),"NO_RESPONSE") && controller.mayHaveChanged());
  assert(!controller.nextCommand(none)); // Only the handshake was retried.
  assert(!controller.setName("Perpetual")); assert(take(controller)=="AT"); reply(controller,"+ERROR");
  assert(!controller.active() && !controller.mayHaveChanged() && !controller.nextCommand(none));
  assert(!controller.setName("Perpetual")); assert(take(controller)=="AT"); reply(controller,"OK");
  assert(take(controller)=="AT+VERSION"); controller.finishCommand();
  assert(!controller.active() && !controller.mayHaveChanged() && !controller.nextCommand(none));
  puts("Bluetooth name passed: JDY-34-only setName, bounded handshake, real V1.52 trace, both-profile readback and no write retries.");
}
} // namespace test_bluetooth_name

namespace test_bluetooth_link {
using bluetooth::Jdy34Link;
bool line(Jdy34Link &link, const std::string &text, uint32_t now) {
  bool application = false;
  for (char c : text + "\n") application = link.feed(c,now);
  return application;
}
void boot(Jdy34Link &link, uint32_t base = 0, bool connected = false) {
  link.begin(base); assert(!link.poll(base+249));
  assert(!strcmp(link.poll(base+250),"AT\r\n"));
  assert(!line(link,"+OK\r",base+260)); assert(!link.poll(base+359));
  assert(!strcmp(link.poll(base+360),"AT+VERSION\r\n"));
  line(link,"+JDY-34-SPP-BLE-V1.52",base+370);
  assert(!strcmp(link.poll(base+470),"AT+BTMODE\r\n"));
  line(link,"+BTMODE=1",base+480);
  assert(!strcmp(link.poll(base+580),"AT+ENLOG\r\n"));
  line(link,"+ENLOG=1",base+590);
  assert(!strcmp(link.poll(base+690),"AT+STAT\r\n"));
  for(unsigned i=0;i<8;++i) line(link,"+STAT="+std::to_string(i)+","+(connected && i==1 ? "1" : "0"),base+700);
  assert(!link.poll(base+850) && !link.ready()); // Complete snapshot also needs ACK.
  line(link,"+OK",base+860); assert(!link.poll(base+960));
  assert(link.ready() && link.connected()==connected && !strcmp(link.error(),"None"));
}
void run() {
  Jdy34Link link; boot(link);
  assert(!line(link,"ping",1000)); // No application execution without a radio connection.
  const auto before=link.epoch();
  assert(!line(link,"+CONNECTED>>0xBC38987177C5,1\r",1010));
  assert(link.connected() && link.epoch()!=before);
  assert(line(link,"@1 ping\r",1020) && link.length()==8 && !memcmp(link.line(),"@1 ping\r",8));
  const std::string invalid("ping\0suffix",11);
  assert(line(link,invalid,1030) && link.length()==invalid.size() && !memcmp(link.line(),invalid.data(),invalid.size()));
  assert(line(link,std::string(500,'x'),1040) && link.length()==129);
  assert(!line(link,"+DISCONNECTED=1",1050) && !link.connected());
  assert(!line(link,"+CONNECTED-ID=0",1060) && link.connected());
  assert(!line(link,"+DISCONNECTED-ID=0",1070) && !link.connected());
  assert(!line(link,"+CONNECTED-ID=0junk",1080) && !link.connected());
  line(link,"+CONNECTED-ID=0",1090);
  for(char c:std::string("@50 config apply")) link.feed(c,1091);
  assert(!line(link,"+DISCONNECTED-ID=0",1092) && !link.connected());
  line(link,"+CONNECTED-ID=0",1093);
  for(unsigned i=0;i<500;++i) link.feed('x',1094);
  assert(!line(link,"+DISCONNECTED-ID=0",1095) && !link.connected());
  line(link,"+CONNECTED-ID=0",1096);
  link.feed('p',1100); link.overflow();
  assert(!line(link,"ing",1110) && line(link,"ping",1120));
  const auto epoch=link.epoch(); line(link,"+START=OK",1130);
  assert(!link.ready() && !link.connected() && link.epoch()!=epoch);
  assert(!link.poll(1379) && !strcmp(link.poll(1380),"AT\r\n"));
  boot(link,0xFFFFFC00u,true); // Timer rollover and an already-connected module.
  link.begin(0); assert(!strcmp(link.poll(250),"AT\r\n"));
  assert(!strcmp(link.poll(1250),"AT\r\n")); assert(!link.poll(2250));
  assert(!link.ready() && !strcmp(link.error(),"NO_RESPONSE"));
  link.begin(0); link.poll(250); line(link,"OK",260); link.poll(360); line(link,"+JDY-34-V1.52",370); link.poll(470);
  line(link,"+BTMODE=0",480); assert(!link.ready() && !strcmp(link.error(),"SINGLE_LINK_MODE_REQUIRED"));
  link.begin(0); link.poll(250); line(link,"OK",260); link.poll(360); line(link,"+JDY-34-V1.52",370); link.poll(470);
  line(link,"+BTMODE=1",480); link.poll(580); line(link,"+ENLOG=0",590);
  assert(!link.ready() && !strcmp(link.error(),"NOTIFICATIONS_REQUIRED"));
  link.begin(0); link.poll(250); line(link,"OK",260); link.poll(360); line(link,"+JDY-34-V1.52",370); link.poll(470);
  line(link,"+BTMODE=1",480); link.poll(580); line(link,"+ENLOG=1",590); link.poll(690);
  line(link,"+STAT=0,1",700); line(link,"+STAT=8,1",700); line(link,"+OK",700);
  assert(!link.poll(800) && !link.ready()); link.poll(1690);
  assert(!strcmp(link.error(),"NO_RESPONSE")); // Missing channels cannot establish the boot snapshot.
  link.begin(0); link.poll(250); line(link,"+ERROR",260);
  assert(!strcmp(link.error(),"MODULE_REJECTED"));
  boot(link); link.overflow(); assert(link.ready());
  puts("Bluetooth link passed: nonblocking startup, single-link checks, snapshot/notifications, reconnect, reset, overflow and binary-safe framing.");
}
}

int main() {
  std::puts("  bluetooth_diagnostic"); std::fflush(stdout);
  test_bluetooth_diagnostic::run();
  std::puts("  bluetooth_name"); std::fflush(stdout);
  test_bluetooth_name::run();
  test_bluetooth_link::run();
}
