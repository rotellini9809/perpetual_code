#include "hardware/Jdy34Link.h"
#include <cstdio>
#include <cstring>

namespace bluetooth {
static_assert(Jdy34Link::UartBaud == 230400, "AT+BAUDA selects 230400 baud");
namespace {
constexpr const char *DefaultName = "Perpetual";
bool ack(const char *s) { return !std::strcmp(s,"OK") || !std::strcmp(s,"+OK"); }
bool channel(const char *s, const char *prefix, unsigned &id) {
  const auto n = std::strlen(prefix);
  if (std::strncmp(s,prefix,n) || s[n] < '0' || s[n] > '7' || s[n+1]) return false;
  id = unsigned(s[n]-'0'); return true;
}
// "+KEY=value" or "+KEY:value" (firmware 1.52 answers +NAME:<name> and +NAMB=<name>).
const char *value(const char *s, const char *key) {
  const auto n = std::strlen(key);
  return !std::strncmp(s,key,n) && (s[n] == ':' || s[n] == '=') ? s+n+1 : nullptr;
}
bool nameByte(char c) { return (c>='A' && c<='Z') || (c>='a' && c<='z') || (c>='0' && c<='9') || c=='-' || c=='_'; }
}
bool Jdy34Link::boardName(const char *suffix, char *name, std::size_t capacity) {
  if (!suffix || !name || !capacity) return false;
  if (!std::strcmp(suffix,"default")) return std::snprintf(name,capacity,"%s",DefaultName) < int(capacity);
  const auto n = std::strlen(suffix);
  if (n < 1 || n > 4) return false;
  for (std::size_t i=0;i<n;++i) if (!nameByte(suffix[i]) || suffix[i]=='_') return false;
  return std::snprintf(name,capacity,"%s-%s",DefaultName,suffix) < int(capacity);
}
void Jdy34Link::begin(uint32_t now, uint32_t settleMs) {
  stage_ = Stage::Settle; since_ = now; hold_ = settleMs; waiting_ = matched_ = ack_ = discard_ = false;
  connected_ = known_ = attempts_ = 0; length_ = completed_ = notificationLength_ = 0; error_ = "STARTING"; ++epoch_;
  name_[0] = 0;
}
void Jdy34Link::fail(const char *error, uint32_t now) {
  if (renaming()) { renameState_ = error; begin(now); return; } // The module was not restarted: recover the link.
  if (setup()) baud_ = UartBaud; // Back to the operating baud; the setup error stays visible.
  failedAt_ = stage_; stage_ = Stage::Failed; error_ = error; connections(0);
}
const char *Jdy34Link::stageName() const {
  static const char *const names[] = {"Settle","Handshake","Version","Mode","Notifications","Name","Status","Ready","Failed",
    "SetupHandshake","SetupVersion","SetupStatus","SetupName","SetupBleName","SetupMode","SetupNotifications","SetupBaud","SetupReset",
    "RenameHandshake","RenameName","RenameBleName","RenameCheckName","RenameCheckBleName","RenameReset"};
  return names[unsigned(stage_ == Stage::Failed ? failedAt_ : stage_)];
}
void Jdy34Link::connections(uint8_t mask) {
  if (connected_ != mask) { connected_ = mask; ++epoch_; }
}
void Jdy34Link::overflow() {
  ++epoch_; length_ = notificationLength_ = 0; discard_ = true;
  if (!ready()) fail("UART_OVERFLOW",since_);
}
const char *Jdy34Link::poll(uint32_t now) {
  if (stage_ == Stage::Ready || stage_ == Stage::Failed) return nullptr;
  if (!waiting_ && uint32_t(now-since_) < hold_) return nullptr;
  if (stage_ == Stage::Settle) stage_ = Stage::Handshake;
  if (waiting_) {
    const bool snapshot = stage_ == Stage::Status || stage_ == Stage::SetupStatus;
    const bool complete = matched_ && (!snapshot || (known_ == 255 && ack_));
    if (complete && uint32_t(now-receivedAt_) >= 100) {
      waiting_ = false;
      if (!advance(now)) return nullptr;
    } else if (uint32_t(now-since_) >= 1000) {
      waiting_ = false;
      if (!timedOut(now)) return nullptr;
    } else return nullptr;
  }
  matched_ = ack_ = false; since_ = now; hold_ = 0; waiting_ = true; ++attempts_;
  if (stage_ == Stage::Status || stage_ == Stage::SetupStatus) known_ = 0;
  return request();
}
bool Jdy34Link::advance(uint32_t now) {
  switch (stage_) {
    case Stage::Status: stage_ = Stage::Ready; error_ = "None"; return false;
    case Stage::SetupBaud: baud_ = UartBaud; stage_ = Stage::SetupReset; wait(now,100); return false; // The module switched already.
    case Stage::SetupReset: configured_ = true; begin(now,RestartSettleMs); return false;
    case Stage::RenameCheckBleName: renameState_ = "OK"; stage_ = Stage::RenameReset; return true; // Names apply after a restart.
    case Stage::RenameReset: begin(now,RestartSettleMs); return false;
    default:
      if (stage_ == Stage::Handshake && baud_ == UartBaud) setupTried_ = true; // A configured module: never set up again.
      stage_ = Stage(unsigned(stage_)+1); attempts_ = 0; return true;
  }
}
uint8_t Jdy34Link::attemptLimit() const {
  switch (stage_) {
    case Stage::Handshake: return StartupAttempts;
    case Stage::Version: case Stage::Mode: case Stage::Notifications: case Stage::Name: case Stage::Status:
    case Stage::SetupHandshake: case Stage::SetupVersion: case Stage::SetupStatus:
    case Stage::RenameHandshake: case Stage::RenameCheckName: case Stage::RenameCheckBleName: return QueryAttempts;
    default: return 1; // Writes and resets.
  }
}
bool Jdy34Link::timedOut(uint32_t now) {
  if (attempts_ < attemptLimit()) return true;
  switch (stage_) {
    case Stage::Handshake:
      if (baud_ == UartBaud && !setupTried_) { // A factory module answers at 9600 instead: set it up once.
        setupTried_ = true; baud_ = FactoryBaud; stage_ = Stage::SetupHandshake; attempts_ = 0; wait(now,100);
        return false;
      }
      break;
    case Stage::Name: name_[0] = 0; stage_ = Stage::Status; attempts_ = 0; return true; // The name is informative only.
    case Stage::SetupReset: configured_ = true; begin(now,RestartSettleMs); return false; // It may restart before replying.
    case Stage::RenameReset: begin(now,RestartSettleMs); return false;
    default: break;
  }
  fail("NO_RESPONSE",now); return false;
}
const char *Jdy34Link::request() {
  switch (stage_) {
    case Stage::Handshake: case Stage::SetupHandshake: case Stage::RenameHandshake: return "AT\r\n";
    case Stage::Version: case Stage::SetupVersion: return "AT+VERSION\r\n";
    case Stage::Mode: return "AT+BTMODE\r\n";
    case Stage::Notifications: return "AT+ENLOG\r\n";
    case Stage::Name: case Stage::RenameCheckName: return "AT+NAME\r\n";
    case Stage::RenameCheckBleName: return "AT+NAMB\r\n";
    case Stage::Status: case Stage::SetupStatus: return "AT+STAT\r\n";
    case Stage::SetupName: std::snprintf(request_,sizeof(request_),"AT+NAME%s\r\n",DefaultName); return request_;
    case Stage::SetupBleName: std::snprintf(request_,sizeof(request_),"AT+NAMB%s_BLE\r\n",DefaultName); return request_;
    case Stage::SetupMode: return "AT+BTMODE1\r\n"; // SPP or BLE, one link at a time.
    case Stage::SetupNotifications: return "AT+ENLOG1\r\n";
    case Stage::SetupBaud: return "AT+BAUDA\r\n";
    case Stage::SetupReset: case Stage::RenameReset: return "AT+RESET\r\n";
    case Stage::RenameName: std::snprintf(request_,sizeof(request_),"AT+NAME%s\r\n",requested_); return request_;
    case Stage::RenameBleName: std::snprintf(request_,sizeof(request_),"AT+NAMB%s_BLE\r\n",requested_); return request_;
    default: return nullptr;
  }
}
void Jdy34Link::reply(const char *text, bool printable, uint32_t now) {
  // Name replies of factory modules carry NUL padding; their printable prefix is the name.
  const bool nameStage = stage_ == Stage::Name || stage_ == Stage::RenameCheckName || stage_ == Stage::RenameCheckBleName;
  if (!printable && !nameStage) return;
  if (!std::strcmp(text,"+ERROR") || !std::strcmp(text,"ERROR")) { fail("MODULE_REJECTED",now); return; }
  receivedAt_ = now;
  const char *v = nullptr;
  switch (stage_) {
    case Stage::Handshake: case Stage::SetupHandshake: case Stage::RenameHandshake: if (ack(text)) matched_ = true; break;
    case Stage::Version: case Stage::SetupVersion: if (std::strstr(text,"JDY-34-")) matched_ = true; break;
    case Stage::Mode:
      if ((v = value(text,"+BTMODE"))) { if (std::strcmp(v,"1")) fail("SINGLE_LINK_MODE_REQUIRED",now); else matched_ = true; }
      break;
    case Stage::Notifications:
      if ((v = value(text,"+ENLOG"))) { if (std::strcmp(v,"1")) fail("NOTIFICATIONS_REQUIRED",now); else matched_ = true; }
      break;
    case Stage::Name:
      if ((v = value(text,"+NAME"))) {
        std::size_t n = 0;
        while (n < sizeof(name_)-1 && nameByte(v[n])) { name_[n] = v[n]; ++n; }
        name_[n] = 0; matched_ = true;
      }
      break;
    case Stage::RenameCheckName:
      if ((v = value(text,"+NAME"))) { if (std::strcmp(v,requested_)) fail("NAME_NOT_VERIFIED",now); else matched_ = true; }
      break;
    case Stage::RenameCheckBleName:
      if ((v = value(text,"+NAMB"))) {
        char expected[sizeof(requested_)+4]; std::snprintf(expected,sizeof(expected),"%s_BLE",requested_);
        if (std::strcmp(v,expected)) fail("NAME_NOT_VERIFIED",now); else matched_ = true;
      }
      break;
    case Stage::Status: case Stage::SetupStatus:
      if (ack(text)) ack_ = true;
      if (std::strlen(text)==9 && !std::strncmp(text,"+STAT=",6) && text[6]>='0' && text[6]<='7' && text[7]==',' && (text[8]=='0' || text[8]=='1')) {
        const auto bit = uint8_t(1u << unsigned(text[6]-'0')); known_ |= bit;
        if (stage_ == Stage::SetupStatus && text[8]=='1') { fail("SETUP_LINK_CONNECTED",now); return; } // Never reconfigure a live link.
        connections(text[8]=='1' ? uint8_t(connected_ | bit) : uint8_t(connected_ & ~bit)); matched_ = true;
      }
      break;
    default: if (ack(text)) matched_ = true; break; // Setup and rename writes, resets.
  }
}
bool Jdy34Link::event(const char *s, uint32_t now) {
  if (!std::strcmp(s,"+START=OK")) {
    // The module (re)booted: check it again, also after giving up on a slow boot.
    if (ready() || stage_ == Stage::Failed) begin(now);
    return true;
  }
  unsigned id = 0;
  if (channel(s,"+CONNECTED-ID=",id)) { ++epoch_; connections(uint8_t(1u << id)); return true; }
  if (channel(s,"+DISCONNECTED-ID=",id) || channel(s,"+DISCONNECTED=",id)) {
    connections(uint8_t(connected_ & ~(1u << id))); return true;
  }
  // SPP firmware 1.52 reports +CONNECTED>>0x<12 hex digits>,<channel>.
  const auto n = std::strlen(s);
  if (n == 28 && !std::strncmp(s,"+CONNECTED>>0x",14) && s[26]==',' && s[27]>='0' && s[27]<='7') {
    bool hex = true;
    for (unsigned i=14;i<26;++i) hex = hex && ((s[i]>='0' && s[i]<='9') || (s[i]>='A' && s[i]<='F') || (s[i]>='a' && s[i]<='f'));
    if (hex) { ++epoch_; connections(uint8_t(1u << unsigned(s[27]-'0'))); return true; }
  }
  return false;
}
bool Jdy34Link::feed(char byte, uint32_t now) {
  if (byte != '\n') {
    if (!discard_ && length_ < sizeof(line_)-1) line_[length_++] = byte;
    // Notifications may immediately follow an unfinished radio command, without
    // a separating LF. Track their suffix even after an oversized input line.
    if (byte == '+') notificationLength_ = 0;
    if ((byte == '+' || notificationLength_) && notificationLength_ < sizeof(notification_)-1)
      notification_[notificationLength_++] = byte;
    return false;
  }
  auto eventBytes = notificationLength_;
  notificationLength_ = 0;
  if (eventBytes && notification_[eventBytes-1]=='\r') --eventBytes;
  notification_[eventBytes] = 0;
  bool eventAscii = true;
  for (std::size_t i=0;i<eventBytes;++i) eventAscii = eventAscii && notification_[i]>=32 && notification_[i]<=126;
  if (eventAscii && eventBytes && event(notification_,now)) { length_ = 0; discard_ = false; return false; }
  if (discard_) { discard_ = false; length_ = 0; return false; }
  completed_ = length_; length_ = 0; line_[completed_] = 0;
  // Module replies are ASCII. Preserve controls/NULs in application input so
  // the common command parser rejects them rather than executing a clean prefix.
  std::size_t n = completed_;
  if (n && line_[n-1] == '\r') --n;
  bool printable = true;
  for (std::size_t i=0;i<n;++i) printable = printable && line_[i]>=32 && line_[i]<=126;
  char text[130]; std::memcpy(text,line_,n); text[n] = 0;
  if (ready()) return connected() && !(printable && text[0]=='+');
  if (waiting_ && stage_ != Stage::Failed) reply(text,printable,now);
  return false;
}
const char *Jdy34Link::requestName(const char *suffix) {
  if (!ready()) return "BLUETOOTH_NOT_READY";
  if (renamePending_) return "BUSY";
  char name[sizeof(requested_)];
  if (!boardName(suffix,name,sizeof(name))) return "INVALID_NAME";
  std::memcpy(requested_,name,sizeof(requested_)); renamePending_ = true; renameState_ = "PENDING";
  return nullptr;
}
void Jdy34Link::startRename(uint32_t now) {
  if (!renamePending_) return;
  renamePending_ = false;
  if (!ready()) { renameState_ = "BLUETOOTH_NOT_READY"; return; }
  renameState_ = "ACTIVE"; stage_ = Stage::RenameHandshake; attempts_ = 0;
  wait(now,150); // MAIN pulls PWRC low (AT access while connected) once the link is no longer ready.
}
}
