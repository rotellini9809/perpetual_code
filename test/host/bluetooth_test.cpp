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
  for (const char *q : {"AT","AT+VERSION","AT+NAME","AT+NAMB","AT+BAUD","AT+MTU","AT+STAT","AT+LADDR","AT+BTMODE"}) assert(queryAllowed(q));
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
// Startup after the settle time; `start` is when the handshake is sent.
void startup(Jdy34Link &link, uint32_t start, bool connected = false, const std::string &name = "+NAME:Perpetual") {
  assert(!strcmp(link.poll(start),"AT\r\n"));
  assert(!line(link,"+OK\r",start+10)); assert(!link.poll(start+109));
  assert(!strcmp(link.poll(start+110),"AT+VERSION\r\n"));
  line(link,"+JDY-34-SPP-BLE-V1.52",start+120);
  assert(!strcmp(link.poll(start+220),"AT+BTMODE\r\n"));
  line(link,"+BTMODE=1",start+230);
  assert(!strcmp(link.poll(start+330),"AT+ENLOG\r\n"));
  line(link,"+ENLOG=1",start+340);
  assert(!strcmp(link.poll(start+440),"AT+NAME\r\n"));
  line(link,name,start+450);
  assert(!strcmp(link.poll(start+550),"AT+STAT\r\n"));
  for(unsigned i=0;i<8;++i) line(link,"+STAT="+std::to_string(i)+","+(connected && i==1 ? "1" : "0"),start+560);
  assert(!link.poll(start+700) && !link.ready()); // Complete snapshot also needs ACK.
  line(link,"+OK",start+710); assert(!link.poll(start+810));
  assert(link.ready() && link.connected()==connected && !strcmp(link.error(),"None"));
}
void boot(Jdy34Link &link, uint32_t base = 0, bool connected = false) {
  link.begin(base); assert(!link.poll(base+249));
  startup(link,base+250,connected);
  assert(!strcmp(link.name(),"Perpetual") && link.uartBaud()==Jdy34Link::UartBaud);
}
// A factory module: silent at 230400, configured at 9600, restarted and checked like any module.
// Four silent startup attempts at 230400 (from `start`, 1 s apart); true once 9600 is selected.
bool silent(Jdy34Link &link, uint32_t start) {
  for (unsigned i=0;i<4;++i) assert(!strcmp(link.poll(start+1000*i),"AT\r\n"));
  return !link.poll(start+4000) && link.uartBaud()==Jdy34Link::FactoryBaud;
}
void factorySetup() {
  Jdy34Link link; link.begin(0);
  assert(silent(link,250) && !link.ready());
  assert(!link.poll(4349) && !strcmp(link.poll(4350),"AT\r\n")); line(link,"+OK",4360);
  assert(!strcmp(link.poll(4460),"AT+VERSION\r\n")); line(link,"+JDY-34-SPP-BLE-V1.52",4470);
  assert(!strcmp(link.poll(4570),"AT+STAT\r\n"));
  for(unsigned i=0;i<8;++i) line(link,"+STAT="+std::to_string(i)+",0",4580);
  line(link,"+OK",4590);
  const char *writes[]={"AT+NAMEPerpetual\r\n","AT+NAMBPerpetual_BLE\r\n","AT+BTMODE1\r\n","AT+ENLOG1\r\n","AT+BAUDA\r\n"};
  uint32_t now=4690;
  for (const char *write : writes) { assert(!strcmp(link.poll(now),write)); line(link,"+OK",now+10); now+=110; } // 100 ms quiet after each reply.
  assert(!link.poll(now) && link.uartBaud()==Jdy34Link::UartBaud); // The module already switched to 230400.
  assert(!link.poll(now+99) && !strcmp(link.poll(now+100),"AT+RESET\r\n")); line(link,"OK",now+110);
  assert(!link.poll(now+210) && link.configured() && !link.ready());
  assert(!link.poll(now+210+1499)); // Module reboot.
  startup(link,now+210+1500,false,std::string("+NAME:JDY-34\0\0" "31",16)); // Factory NUL padding before the reset applies.
  assert(!strcmp(link.name(),"JDY-34"));
  // A live radio link is never reconfigured, and a module that is not a JDY-34 is left alone.
  Jdy34Link busy; busy.begin(0); assert(silent(busy,250)); busy.poll(4350); line(busy,"+OK",4360);
  busy.poll(4460); line(busy,"+JDY-34-SPP-BLE-V1.52",4470); busy.poll(4570); line(busy,"+STAT=0,1",4580);
  assert(!busy.ready() && !strcmp(busy.error(),"SETUP_LINK_CONNECTED") && busy.uartBaud()==Jdy34Link::UartBaud && !busy.configured());
  Jdy34Link other; other.begin(0); assert(silent(other,250)); other.poll(4350); line(other,"+OK",4360);
  other.poll(4460); line(other,"+HC-05",4470);
  assert(!strcmp(other.poll(5460),"AT+VERSION\r\n") && !strcmp(other.poll(6460),"AT+VERSION\r\n") && !other.poll(7460));
  assert(!strcmp(other.error(),"NO_RESPONSE") && other.uartBaud()==Jdy34Link::UartBaud);
  // Setup is tried once per power-up: a later silent restart does not try 9600 again.
  other.begin(8000); for (unsigned i=0;i<4;++i) other.poll(8250+1000*i);
  assert(!other.poll(12250) && !strcmp(other.error(),"NO_RESPONSE") && other.uartBaud()==Jdy34Link::UartBaud);
  // A slow module that announces its boot after the attempts gave up is checked again.
  line(other,"+START=OK",13000); assert(!other.poll(13249) && !strcmp(other.poll(13250),"AT\r\n"));
  // One ignored query after a restart is asked again instead of disabling Bluetooth.
  Jdy34Link lossy; lossy.begin(0); lossy.poll(250); line(lossy,"+OK",260); lossy.poll(360); line(lossy,"+JDY-34-SPP-BLE-V1.52",370);
  lossy.poll(470); line(lossy,"+BTMODE=1",480); assert(!strcmp(lossy.poll(580),"AT+ENLOG\r\n"));
  assert(!strcmp(lossy.poll(1580),"AT+ENLOG\r\n")); line(lossy,"+ENLOG=1",1590);
  assert(!strcmp(lossy.poll(1690),"AT+NAME\r\n") && !lossy.ready());
  // A module that answered at 230400 is configured: a later silent restart never tries 9600.
  Jdy34Link known; boot(known); known.begin(20000); for (unsigned i=0;i<4;++i) known.poll(20250+1000*i);
  assert(!known.poll(24250) && !strcmp(known.error(),"NO_RESPONSE") && known.uartBaud()==Jdy34Link::UartBaud);
}
void renaming() {
  char name[24];
  assert(Jdy34Link::boardName("default",name,sizeof(name)) && !strcmp(name,"Perpetual"));
  assert(Jdy34Link::boardName("Hop",name,sizeof(name)) && !strcmp(name,"Perpetual-Hop"));
  assert(Jdy34Link::boardName("A-1b",name,sizeof(name)) && !strcmp(name,"Perpetual-A-1b")); // 14 bytes; 18 with _BLE.
  for (const char *bad : {"","ABCDE","a_b","a b","é"}) assert(!Jdy34Link::boardName(bad,name,sizeof(name)));
  assert(!Jdy34Link::boardName("Hop",name,13));
  Jdy34Link link; assert(!strcmp(link.requestName("A1"),"BLUETOOTH_NOT_READY"));
  boot(link,0,true);
  assert(!strcmp(link.requestName("x_1"),"INVALID_NAME") && !link.renamePending());
  assert(!link.requestName("A1") && link.renamePending() && !strcmp(link.renameState(),"PENDING"));
  assert(!strcmp(link.requestName("B"),"BUSY"));
  assert(link.ready()); // Still serving until MAIN has sent the accepting reply.
  link.startRename(2000); assert(!link.ready() && !link.connected() && !strcmp(link.renameState(),"ACTIVE"));
  assert(!link.poll(2149) && !strcmp(link.poll(2150),"AT\r\n")); line(link,"+OK",2160);
  assert(!strcmp(link.poll(2260),"AT+NAMEPerpetual-A1\r\n")); line(link,"OK",2270);
  assert(!strcmp(link.poll(2370),"AT+NAMBPerpetual-A1_BLE\r\n")); line(link,"+OK",2380);
  assert(!strcmp(link.poll(2480),"AT+NAME\r\n")); line(link,"+NAME:Perpetual-A1",2490);
  assert(!strcmp(link.poll(2590),"AT+NAMB\r\n")); line(link,"+NAMB=Perpetual-A1_BLE",2600);
  assert(!strcmp(link.poll(2700),"AT+RESET\r\n") && !strcmp(link.renameState(),"OK")); line(link,"OK",2710);
  const auto epoch=link.epoch(); assert(!link.poll(2810) && link.epoch()!=epoch); // Restart drops the link.
  assert(!link.poll(2810+1499)); startup(link,2810+1500,false,"+NAME:Perpetual-A1");
  assert(!strcmp(link.name(),"Perpetual-A1"));
  // A write that cannot be read back recovers the link and reports why.
  assert(!link.requestName("default")); link.startRename(9000);
  link.poll(9150); line(link,"+OK",9160); link.poll(9260); line(link,"OK",9270); link.poll(9370); line(link,"+OK",9380);
  link.poll(9480); line(link,"+NAME:Perpetual-A1",9490);
  assert(!strcmp(link.renameState(),"NAME_NOT_VERIFIED") && !link.ready());
  assert(!link.poll(9490+249) && !strcmp(link.poll(9490+250),"AT\r\n"));
  startup(link,9490+250+1000); // The repeated handshake still finds the module.
  assert(!link.requestName("Z9")); link.startRename(20000); link.poll(20150); line(link,"+OK",20160);
  assert(!strcmp(link.poll(20260),"AT+NAMEPerpetual-Z9\r\n")); assert(!link.poll(21260)); // Not repeated.
  assert(!strcmp(link.renameState(),"NO_RESPONSE") && !link.ready());
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
  // Silent at 230400: a module that never answered could be a factory one, so try 9600 once.
  Jdy34Link fresh; fresh.begin(0); assert(silent(fresh,250));
  for (unsigned i=0;i<3;++i) assert(!strcmp(fresh.poll(4350+1000*i),"AT\r\n"));
  assert(!fresh.poll(7350) && !fresh.ready() && !strcmp(fresh.error(),"NO_RESPONSE") && fresh.uartBaud()==Jdy34Link::UartBaud);
  link.begin(0); link.poll(250); line(link,"OK",260); link.poll(360); line(link,"+JDY-34-V1.52",370); link.poll(470);
  line(link,"+BTMODE=0",480); assert(!link.ready() && !strcmp(link.error(),"SINGLE_LINK_MODE_REQUIRED"));
  link.begin(0); link.poll(250); line(link,"OK",260); link.poll(360); line(link,"+JDY-34-V1.52",370); link.poll(470);
  line(link,"+BTMODE=1",480); link.poll(580); line(link,"+ENLOG=0",590);
  assert(!link.ready() && !strcmp(link.error(),"NOTIFICATIONS_REQUIRED"));
  link.begin(0); link.poll(250); line(link,"OK",260); link.poll(360); line(link,"+JDY-34-V1.52",370); link.poll(470);
  line(link,"+BTMODE=1",480); link.poll(580); line(link,"+ENLOG=1",590); link.poll(690); line(link,"+NAME:Perpetual",700);
  assert(!strcmp(link.poll(800),"AT+STAT\r\n"));
  line(link,"+STAT=0,1",810); line(link,"+STAT=8,1",810); line(link,"+OK",810);
  assert(!link.poll(910) && !link.ready());
  assert(!strcmp(link.poll(1800),"AT+STAT\r\n") && !strcmp(link.poll(2800),"AT+STAT\r\n")); link.poll(3800);
  assert(!strcmp(link.error(),"NO_RESPONSE")); // Missing channels cannot establish the boot snapshot.
  link.begin(0); link.poll(250); line(link,"+ERROR",260);
  assert(!strcmp(link.error(),"MODULE_REJECTED"));
  boot(link); link.overflow(); assert(link.ready());
  // A missing name reply does not block the link: the name is informative only.
  link.begin(0); link.poll(250); line(link,"OK",260); link.poll(360); line(link,"+JDY-34-V1.52",370); link.poll(470);
  line(link,"+BTMODE=1",480); link.poll(580); line(link,"+ENLOG=1",590); assert(!strcmp(link.poll(690),"AT+NAME\r\n"));
  assert(!strcmp(link.poll(1690),"AT+NAME\r\n") && !strcmp(link.poll(2690),"AT+NAME\r\n"));
  assert(!strcmp(link.poll(3690),"AT+STAT\r\n") && !link.name()[0]);
  factorySetup(); renaming();
  puts("Bluetooth link passed: nonblocking startup, name, factory setup, verified rename, single-link checks, snapshot/notifications, reconnect, reset, overflow and binary-safe framing.");
}
}

int main() {
  std::puts("  bluetooth_diagnostic"); std::fflush(stdout);
  test_bluetooth_diagnostic::run();
  std::puts("  bluetooth_name"); std::fflush(stdout);
  test_bluetooth_name::run();
  test_bluetooth_link::run();
}
