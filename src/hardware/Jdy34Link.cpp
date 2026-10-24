#include "hardware/Jdy34Link.h"
#include <cstring>

namespace bluetooth {
namespace {
bool ack(const char *s) { return !std::strcmp(s,"OK") || !std::strcmp(s,"+OK"); }
bool channel(const char *s, const char *prefix, unsigned &id) {
  const auto n = std::strlen(prefix);
  if (std::strncmp(s,prefix,n) || s[n] < '0' || s[n] > '7' || s[n+1]) return false;
  id = unsigned(s[n]-'0'); return true;
}
}
void Jdy34Link::begin(uint32_t now) {
  stage_ = Stage::Settle; since_ = now; waiting_ = matched_ = ack_ = discard_ = false;
  connected_ = known_ = attempts_ = 0; length_ = completed_ = notificationLength_ = 0; error_ = "STARTING"; ++epoch_;
}
void Jdy34Link::fail(const char *error) { stage_ = Stage::Failed; error_ = error; connections(0); }
void Jdy34Link::connections(uint8_t mask) {
  if (connected_ != mask) { connected_ = mask; ++epoch_; }
}
void Jdy34Link::overflow() {
  ++epoch_; length_ = notificationLength_ = 0; discard_ = true;
  if (!ready()) fail("UART_OVERFLOW");
}
const char *Jdy34Link::poll(uint32_t now) {
  if (stage_ == Stage::Ready || stage_ == Stage::Failed) return nullptr;
  if (stage_ == Stage::Settle) {
    if (uint32_t(now-since_) < 250) return nullptr;
    stage_ = Stage::Handshake;
  }
  if (waiting_) {
    const bool complete = matched_ && (stage_ != Stage::Status || (known_ == 255 && ack_));
    if (complete && uint32_t(now-receivedAt_) >= 100) {
      waiting_ = false; stage_ = Stage(unsigned(stage_)+1);
      if (ready()) { error_ = "None"; return nullptr; }
    } else if (uint32_t(now-since_) >= 1000) {
      if (stage_ == Stage::Handshake && attempts_ < 2) waiting_ = false;
      else { fail("NO_RESPONSE"); return nullptr; }
    } else return nullptr;
  }
  matched_ = ack_ = false; since_ = now; waiting_ = true;
  switch (stage_) {
    case Stage::Handshake: ++attempts_; return "AT\r\n";
    case Stage::Version: return "AT+VERSION\r\n";
    case Stage::Mode: return "AT+BTMODE\r\n";
    case Stage::Notifications: return "AT+ENLOG\r\n";
    case Stage::Status: known_ = 0; return "AT+STAT\r\n";
    default: return nullptr;
  }
}
bool Jdy34Link::event(const char *s, uint32_t now) {
  if (!std::strcmp(s,"+START=OK")) {
    if (ready()) begin(now);
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
  if (!waiting_ || stage_ == Stage::Failed || !printable) return false;
  if (!std::strcmp(text,"+ERROR") || !std::strcmp(text,"ERROR")) { fail("MODULE_REJECTED"); return false; }
  receivedAt_ = now;
  if (stage_ == Stage::Handshake && ack(text)) matched_ = true;
  if (stage_ == Stage::Version && std::strstr(text,"JDY-34-")) matched_ = true;
  if (stage_ == Stage::Mode && !std::strncmp(text,"+BTMODE=",8)) {
    if (std::strcmp(text,"+BTMODE=1")) fail("SINGLE_LINK_MODE_REQUIRED"); else matched_ = true;
  }
  if (stage_ == Stage::Notifications && !std::strncmp(text,"+ENLOG=",7)) {
    if (std::strcmp(text,"+ENLOG=1")) fail("NOTIFICATIONS_REQUIRED"); else matched_ = true;
  }
  if (stage_ == Stage::Status) {
    if (ack(text)) ack_ = true;
    if (std::strlen(text)==9 && !std::strncmp(text,"+STAT=",6) && text[6]>='0' && text[6]<='7' && text[7]==',' && (text[8]=='0' || text[8]=='1')) {
      const auto bit = uint8_t(1u << unsigned(text[6]-'0')); known_ |= bit;
      connections(text[8]=='1' ? uint8_t(connected_ | bit) : uint8_t(connected_ & ~bit)); matched_ = true;
    }
  }
  return false;
}
}
