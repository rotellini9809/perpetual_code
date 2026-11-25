#include "hardware/Jdy34Link.h"
#include <cstdio>
#include <cstring>

namespace bluetooth {
static_assert(Jdy34Link::UartBaud == 230400, "AT+BAUDA selects 230400 baud");
namespace {
constexpr const char *DefaultName = "Perpetual";
bool isAck(const char *text) { return !std::strcmp(text,"OK") || !std::strcmp(text,"+OK"); }
bool parseChannelId(const char *text, const char *prefix, unsigned &id) {
  const auto n = std::strlen(prefix);
  if (std::strncmp(text,prefix,n) || text[n] < '0' || text[n] > '7' || text[n+1]) return false;
  id = unsigned(text[n]-'0'); return true;
}
// "+KEY=value" or "+KEY:value" (firmware 1.52 answers +NAME:<name> and +NAMB=<name>).
const char *valueAfterKey(const char *text, const char *key) {
  const auto n = std::strlen(key);
  return !std::strncmp(text,key,n) && (text[n] == ':' || text[n] == '=') ? text+n+1 : nullptr;
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
  stage_ = Stage::Settle; stageSinceMs_ = now; holdMs_ = settleMs; awaitingReply_ = replyMatched_ = statusAcked_ = discardLine_ = false;
  connectedChannels_ = knownChannels_ = attempts_ = 0; lineLength_ = completedLength_ = notificationLength_ = 0; error_ = "STARTING"; ++epoch_;
  name_[0] = 0;
}
void Jdy34Link::fail(const char *error, uint32_t now) {
  if (renaming()) { renameState_ = error; begin(now); return; } // The module was not restarted: recover the link.
  if (setup()) uartBaud_ = UartBaud; // Back to the operating baud; the setup error stays visible.
  failedAt_ = stage_; stage_ = Stage::Failed; error_ = error; connections(0);
}
const char *Jdy34Link::stageName() const {
  static const char *const names[] = {"Settle","Handshake","Version","Mode","Notifications","Name","Status","Ready","Failed",
    "SetupHandshake","SetupVersion","SetupStatus","SetupName","SetupBleName","SetupMode","SetupNotifications","SetupBaud","SetupReset",
    "RenameHandshake","RenameName","RenameBleName","RenameCheckName","RenameCheckBleName","RenameReset"};
  return names[unsigned(stage_ == Stage::Failed ? failedAt_ : stage_)];
}
void Jdy34Link::connections(uint8_t mask) {
  if (connectedChannels_ != mask) { connectedChannels_ = mask; ++epoch_; }
}
void Jdy34Link::overflow() {
  ++epoch_; lineLength_ = notificationLength_ = 0; discardLine_ = true;
  if (!ready()) fail("UART_OVERFLOW",stageSinceMs_);
}
const char *Jdy34Link::poll(uint32_t now) {
  if (stage_ == Stage::Ready || stage_ == Stage::Failed) return nullptr;
  if (!awaitingReply_ && uint32_t(now-stageSinceMs_) < holdMs_) return nullptr;
  if (stage_ == Stage::Settle) stage_ = Stage::Handshake;
  if (awaitingReply_) {
    const bool snapshot = stage_ == Stage::Status || stage_ == Stage::SetupStatus;
    const bool complete = replyMatched_ && (!snapshot || (knownChannels_ == 255 && statusAcked_));
    if (complete && uint32_t(now-replyReceivedMs_) >= 100) {
      awaitingReply_ = false;
      if (!advance(now)) return nullptr;
    } else if (uint32_t(now-stageSinceMs_) >= 1000) {
      awaitingReply_ = false;
      if (!timedOut(now)) return nullptr;
    } else return nullptr;
  }
  replyMatched_ = statusAcked_ = false; stageSinceMs_ = now; holdMs_ = 0; awaitingReply_ = true; ++attempts_;
  if (stage_ == Stage::Status || stage_ == Stage::SetupStatus) knownChannels_ = 0;
  return request();
}
bool Jdy34Link::advance(uint32_t now) {
  switch (stage_) {
    case Stage::Status: stage_ = Stage::Ready; error_ = "None"; return false;
    case Stage::SetupBaud: uartBaud_ = UartBaud; stage_ = Stage::SetupReset; wait(now,100); return false; // The module switched already.
    case Stage::SetupReset: configured_ = true; begin(now,RestartSettleMs); return false;
    case Stage::RenameCheckBleName: renameState_ = "OK"; stage_ = Stage::RenameReset; return true; // Names apply after a restart.
    case Stage::RenameReset: begin(now,RestartSettleMs); return false;
    default:
      if (stage_ == Stage::Handshake && uartBaud_ == UartBaud) setupTried_ = true; // A configured module: never set up again.
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
      if (uartBaud_ == UartBaud && !setupTried_) { // A factory module answers at 9600 instead: set it up once.
        setupTried_ = true; uartBaud_ = FactoryBaud; stage_ = Stage::SetupHandshake; attempts_ = 0; wait(now,100);
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
    case Stage::SetupName: std::snprintf(requestBuffer_,sizeof(requestBuffer_),"AT+NAME%s\r\n",DefaultName); return requestBuffer_;
    case Stage::SetupBleName: std::snprintf(requestBuffer_,sizeof(requestBuffer_),"AT+NAMB%s_BLE\r\n",DefaultName); return requestBuffer_;
    case Stage::SetupMode: return "AT+BTMODE1\r\n"; // SPP or BLE, one link at a time.
    case Stage::SetupNotifications: return "AT+ENLOG1\r\n";
    case Stage::SetupBaud: return "AT+BAUDA\r\n";
    case Stage::SetupReset: case Stage::RenameReset: return "AT+RESET\r\n";
    case Stage::RenameName: std::snprintf(requestBuffer_,sizeof(requestBuffer_),"AT+NAME%s\r\n",requestedName_); return requestBuffer_;
    case Stage::RenameBleName: std::snprintf(requestBuffer_,sizeof(requestBuffer_),"AT+NAMB%s_BLE\r\n",requestedName_); return requestBuffer_;
    default: return nullptr;
  }
}
void Jdy34Link::reply(const char *text, bool printable, uint32_t now) {
  // Name replies of factory modules carry NUL padding; their printable prefix is the name.
  const bool nameStage = stage_ == Stage::Name || stage_ == Stage::RenameCheckName || stage_ == Stage::RenameCheckBleName;
  if (!printable && !nameStage) return;
  if (!std::strcmp(text,"+ERROR") || !std::strcmp(text,"ERROR")) { fail("MODULE_REJECTED",now); return; }
  replyReceivedMs_ = now;
  const char *value = nullptr;
  switch (stage_) {
    case Stage::Handshake: case Stage::SetupHandshake: case Stage::RenameHandshake: if (isAck(text)) replyMatched_ = true; break;
    case Stage::Version: case Stage::SetupVersion: if (std::strstr(text,"JDY-34-")) replyMatched_ = true; break;
    case Stage::Mode:
      if ((value = valueAfterKey(text,"+BTMODE"))) { if (std::strcmp(value,"1")) fail("SINGLE_LINK_MODE_REQUIRED",now); else replyMatched_ = true; }
      break;
    case Stage::Notifications:
      if ((value = valueAfterKey(text,"+ENLOG"))) { if (std::strcmp(value,"1")) fail("NOTIFICATIONS_REQUIRED",now); else replyMatched_ = true; }
      break;
    case Stage::Name:
      if ((value = valueAfterKey(text,"+NAME"))) {
        std::size_t n = 0;
        while (n < sizeof(name_)-1 && nameByte(value[n])) { name_[n] = value[n]; ++n; }
        name_[n] = 0; replyMatched_ = true;
      }
      break;
    case Stage::RenameCheckName:
      if ((value = valueAfterKey(text,"+NAME"))) { if (std::strcmp(value,requestedName_)) fail("NAME_NOT_VERIFIED",now); else replyMatched_ = true; }
      break;
    case Stage::RenameCheckBleName:
      if ((value = valueAfterKey(text,"+NAMB"))) {
        char expected[sizeof(requestedName_)+4]; std::snprintf(expected,sizeof(expected),"%s_BLE",requestedName_);
        if (std::strcmp(value,expected)) fail("NAME_NOT_VERIFIED",now); else replyMatched_ = true;
      }
      break;
    case Stage::Status: case Stage::SetupStatus:
      if (isAck(text)) statusAcked_ = true;
      if (std::strlen(text)==9 && !std::strncmp(text,"+STAT=",6) && text[6]>='0' && text[6]<='7' && text[7]==',' && (text[8]=='0' || text[8]=='1')) {
        const auto bit = uint8_t(1u << unsigned(text[6]-'0')); knownChannels_ |= bit;
        if (stage_ == Stage::SetupStatus && text[8]=='1') { fail("SETUP_LINK_CONNECTED",now); return; } // Never reconfigure a live link.
        connections(text[8]=='1' ? uint8_t(connectedChannels_ | bit) : uint8_t(connectedChannels_ & ~bit)); replyMatched_ = true;
      }
      break;
    default: if (isAck(text)) replyMatched_ = true; break; // Setup and rename writes, resets.
  }
}
bool Jdy34Link::event(const char *line, uint32_t now) {
  if (!std::strcmp(line,"+START=OK")) {
    // The module (re)booted: check it again, also after giving up on a slow boot.
    if (ready() || stage_ == Stage::Failed) begin(now);
    return true;
  }
  unsigned id = 0;
  if (parseChannelId(line,"+CONNECTED-ID=",id)) { ++epoch_; connections(uint8_t(1u << id)); return true; }
  if (parseChannelId(line,"+DISCONNECTED-ID=",id) || parseChannelId(line,"+DISCONNECTED=",id)) {
    connections(uint8_t(connectedChannels_ & ~(1u << id))); return true;
  }
  // SPP firmware 1.52 reports +CONNECTED>>0x<12 hex digits>,<channel>.
  const auto n = std::strlen(line);
  if (n == 28 && !std::strncmp(line,"+CONNECTED>>0x",14) && line[26]==',' && line[27]>='0' && line[27]<='7') {
    bool hex = true;
    for (unsigned i=14;i<26;++i) hex = hex && ((line[i]>='0' && line[i]<='9') || (line[i]>='A' && line[i]<='F') || (line[i]>='a' && line[i]<='f'));
    if (hex) { ++epoch_; connections(uint8_t(1u << unsigned(line[27]-'0'))); return true; }
  }
  return false;
}
bool Jdy34Link::feed(char byte, uint32_t now) {
  if (byte != '\n') {
    if (!discardLine_ && lineLength_ < sizeof(line_)-1) line_[lineLength_++] = byte;
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
  if (eventAscii && eventBytes && event(notification_,now)) { lineLength_ = 0; discardLine_ = false; return false; }
  if (discardLine_) { discardLine_ = false; lineLength_ = 0; return false; }
  completedLength_ = lineLength_; lineLength_ = 0; line_[completedLength_] = 0;
  // Module replies are ASCII. Preserve controls/NULs in application input so
  // the common command parser rejects them rather than executing a clean prefix.
  std::size_t n = completedLength_;
  if (n && line_[n-1] == '\r') --n;
  bool printable = true;
  for (std::size_t i=0;i<n;++i) printable = printable && line_[i]>=32 && line_[i]<=126;
  char text[130]; std::memcpy(text,line_,n); text[n] = 0;
  if (ready()) return connected() && !(printable && text[0]=='+');
  if (awaitingReply_ && stage_ != Stage::Failed) reply(text,printable,now);
  return false;
}
const char *Jdy34Link::requestName(const char *suffix) {
  if (!ready()) return "BLUETOOTH_NOT_READY";
  if (renamePending_) return "BUSY";
  char name[sizeof(requestedName_)];
  if (!boardName(suffix,name,sizeof(name))) return "INVALID_NAME";
  std::memcpy(requestedName_,name,sizeof(requestedName_)); renamePending_ = true; renameState_ = "PENDING";
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
