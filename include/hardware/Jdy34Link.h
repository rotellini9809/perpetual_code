#pragma once
#include <cstddef>
#include <cstdint>

namespace bluetooth {
// Portable, nonblocking JDY-34 startup and notification filter. UART IO belongs
// to MAIN. Boot only queries module NVM; it never renames/reconfigures the radio.
class Jdy34Link {
public:
  void begin(uint32_t now);
  const char *poll(uint32_t now); // At most one CRLF-terminated AT request.
  bool feed(char byte, uint32_t now); // True: one application line, without LF.
  void overflow(); // Invalidate the session and discard through the next LF.
  bool ready() const { return stage_ == Stage::Ready; }
  bool connected() const { return ready() && connected_ != 0; }
  const char *error() const { return error_; }
  uint32_t epoch() const { return epoch_; }
  const char *line() const { return line_; }
  std::size_t length() const { return completed_; }
private:
  enum class Stage { Settle, Handshake, Version, Mode, Notifications, Status, Ready, Failed };
  Stage stage_ = Stage::Settle;
  char line_[130] = {};
  char notification_[64] = {};
  std::size_t notificationLength_ = 0;
  std::size_t length_ = 0, completed_ = 0;
  uint32_t since_ = 0, receivedAt_ = 0, epoch_ = 0;
  uint8_t connected_ = 0, known_ = 0, attempts_ = 0;
  bool waiting_ = false, matched_ = false, ack_ = false, discard_ = false;
  const char *error_ = "STARTING";
  void fail(const char *error);
  void connections(uint8_t mask);
  bool event(const char *line, uint32_t now);
};
}
