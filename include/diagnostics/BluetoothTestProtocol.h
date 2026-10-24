#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>

// Test-only text protocol; no flight commands or mutable module settings.
namespace bluetooth_test {
enum class LineResult { None, Ready, Rejected };
class Line {
public:
  static constexpr std::size_t Capacity = 128;
  LineResult feed(unsigned char c) {
    if (c == '\r' || c == '\n') {
      const auto result = rejected_ ? LineResult::Rejected : size_ ? LineResult::Ready : LineResult::None;
      bytes_[size_] = 0; size_ = 0; rejected_ = false; return result;
    }
    if (c < 32 || c > 126 || size_ == Capacity) rejected_ = true;
    if (!rejected_) bytes_[size_++] = char(c);
    return LineResult::None;
  }
  bool pending() const { return size_ || rejected_; }
  void clear() { size_ = 0; rejected_ = false; bytes_[0] = 0; }
  const char *text() const { return bytes_; }
private:
  char bytes_[Capacity+1]{};
  std::size_t size_ = 0;
  bool rejected_ = false;
};
inline bool queryAllowed(const char *s) {
  for (const char *q : {"AT", "AT+VERSION", "AT+NAME", "AT+NAMB", "AT+BAUD", "AT+STAT", "AT+LADDR", "AT+BTMODE"})
    if (!std::strcmp(s,q)) return true;
  return false;
}
inline bool baudAllowed(uint32_t baud) {
  for (uint32_t b : {1200u,2400u,4800u,9600u,19200u,38400u,57600u,115200u,128000u,230400u,256000u,460800u,600000u,750000u,1000000u})
    if (b == baud) return true;
  return false;
}
// A snapshot of individual JDY-34 channels, not a connection-status GPIO or
// authentication. Do not treat a single disconnected channel as global disconnect.
struct LinkSnapshot {
  uint8_t known = 0, connected = 0;
  bool add(const char *line) {
    if (std::strlen(line) != 9 || std::strncmp(line,"+STAT=",6) || line[6] < '0' || line[6] > '7' ||
        line[7] != ',' || (line[8] != '0' && line[8] != '1')) return false;
    const auto bit = uint8_t(1u << (line[6]-'0'));
    known |= bit;
    connected = uint8_t((connected & ~bit) | (line[8] == '1' ? bit : 0)); return true;
  }
};
inline bool queryReply(const char *query, const char *line) {
  if (!queryAllowed(query)) return false;
  if (!std::strcmp(query,"AT")) return !std::strcmp(line,"+OK") || !std::strcmp(line,"OK");
  if (!std::strcmp(query,"AT+VERSION") && !std::strncmp(line,"+JDY-34-",8) && line[8]) return true;
  if (!std::strcmp(query,"AT+STAT")) { LinkSnapshot snapshot; return snapshot.add(line); }
  const auto n = std::strlen(query+2);
  return !std::strncmp(line,query+2,n) && (line[n] == '=' || line[n] == ':') && line[n+1];
}
}
