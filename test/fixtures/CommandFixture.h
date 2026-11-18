#pragma once
#include "communication/CommandService.h"
#include <cassert>
#include <cstdio>
#include <string>

namespace command_test {
inline void has(const std::string &reply, const char *expected) {
  if (reply.find(expected)==std::string::npos) {
    std::fprintf(stderr,"Reply: %s\nMissing: %s\n",reply.c_str(),expected);
    assert(false);
  }
}
inline std::string send(CommandService &usb, const std::string &line) {
  CommandService::Status status;
  char output[CommandService::MaxReply];
  std::string result;
  for (char byte : line + '\n')
    if (usb.feed(byte, status, output, sizeof(output))) result += output;
  assert(!result.empty() && result.back() == '\n');
  return result;
}
}
