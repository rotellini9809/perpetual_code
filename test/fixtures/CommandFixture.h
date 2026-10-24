#pragma once
#include "communication/CommandService.h"
#include <cassert>
#include <string>

namespace command_test {
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
