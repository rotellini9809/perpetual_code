#pragma once
#include "communication/CommandService.h"
#include <cstring>

// One parser, retry cache and bounded pending reply per connection. No IO waits.
class CommandEndpoint {
public:
  explicit CommandEndpoint(CommandExtension &handler) { service_.setExtension(&handler); }
  void reset() { service_.reset(); length_ = offset_ = 0; }
  bool pending() const { return offset_ < length_; }
  void feed(char byte, const CommandService::Status &status) {
    if (!pending() && service_.feed(byte, status, reply_, sizeof(reply_))) {
      length_ = std::strlen(reply_); offset_ = 0;
    }
  }
  const uint8_t *data() const { return reinterpret_cast<const uint8_t *>(reply_ + offset_); }
  std::size_t remaining() const { return length_ - offset_; }
  void sent(std::size_t count) { offset_ += count < remaining() ? count : remaining(); }
private:
  CommandService service_;
  char reply_[CommandService::MaxReply] = {};
  std::size_t length_ = 0, offset_ = 0;
};
