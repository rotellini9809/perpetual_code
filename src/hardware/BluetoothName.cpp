#include "hardware/BluetoothName.h"
#include <cstring>
#include <cstdio>

namespace bluetooth {
namespace {
const char *valueOf(const char *line,const char *key) {
  const auto n = std::strlen(key);
  return !std::strncmp(line,key,n) && (line[n] == '=' || line[n] == ':') ? line+n+1 : nullptr;
}
bool isJdy34(const char *value) {
  return !std::strcmp(value,"JDY-34") || (!std::strncmp(value,"JDY-34-",7) && value[7]);
}
}
bool NameController::validName(const char *name) {
  if (!name || !*name || name[0] == ' ') return false;
  std::size_t n = 0;
  for (;name[n];++n) {
    if (n == MaxNameBytes || static_cast<unsigned char>(name[n]) < 32 || static_cast<unsigned char>(name[n]) > 126) return false;
  }
  return name[n-1] != ' ';
}
const char *NameController::setName(const char *name) {
  if (active()) return "BUSY";
  if (!validName(name)) return "INVALID_NAME";
  // Use a temporary in case a caller reuses requestedName() as the input.
  char copy[MaxNameBytes+1]{}; std::strcpy(copy,name); std::memcpy(name_,copy,sizeof(name_));
  state_ = State::Handshake; identified_ = false; handshakeAttempts_ = 0; error_ = "None";
  awaiting_ = received_ = rejected_ = matches_ = mayHaveChanged_ = false;
  return nullptr;
}
bool NameController::nextCommand(Command &output) {
  if (!active() || awaiting_) return false;
  const char *query = nullptr;
  switch (state_) {
    case State::Handshake: query = "AT"; ++handshakeAttempts_; break;
    case State::Identify: query = "AT+VERSION"; break;
    case State::VerifyName: query = "AT+NAME"; break;
    case State::VerifyBleName: query = "AT+NAMB"; break;
    case State::SetName: case State::SetBleName: break;
    default: return false;
  }
  Command command;
  if (query) std::strcpy(command.text,query);
  else {
    std::snprintf(command.text,sizeof(command.text),"AT+%s%s",state_ == State::SetName ? "NAME" : "NAMB",name_);
    // Even a lost ACK can follow a successful module write. Do not auto-retry.
    mayHaveChanged_ = true;
  }
  output = command; awaiting_ = true; received_ = rejected_ = matches_ = false; return true;
}
bool NameController::acceptReply(const char *line) {
  if (!awaiting_ || !line) return false;
  if (!std::strcmp(line,"ERROR") || !std::strcmp(line,"+ERROR") || !std::strcmp(line,"+ERR") ||
      !std::strncmp(line,"+ERROR=",7) || !std::strncmp(line,"+ERR=",5)) {
    received_ = rejected_ = true; return true;
  }
  if (state_ == State::Identify) {
    auto value = valueOf(line,"+VERSION");
    // JDY-34's documented response is +JDY-34-V1.2. Also accept its
    // +VERSION-tagged form; a broadcast name is never a model ID.
    if (!value && !std::strncmp(line,"+JDY-",5)) value = line+1;
    if (!value || !*value) return false;
    identified_ = isJdy34(value);
  } else if (state_ == State::Handshake || state_ == State::SetName || state_ == State::SetBleName) {
    if (std::strcmp(line,"+OK") && std::strcmp(line,"OK")) return false;
  } else {
    const auto value = valueOf(line,state_ == State::VerifyName ? "+NAME" : "+NAMB");
    if (!value) return false;
    // Conflicting duplicate replies invalidate readback rather than letting a
    // later desired value hide the earlier mismatch.
    const bool equal = !std::strcmp(value,name_);
    matches_ = received_ ? matches_ && equal : equal;
  }
  received_ = true; return true;
}
void NameController::finishCommand() {
  if (!awaiting_) return;
  awaiting_ = false;
  if (rejected_) { fail("MODULE_REJECTED"); return; }
  if (!received_) {
    // The fitted JDY-34 can ignore the first command after an MCU reboot.
    // Retry only this read-only handshake once; never retry a name write.
    if (state_ == State::Handshake && handshakeAttempts_ < 2) return;
    fail("NO_RESPONSE"); return;
  }
  switch (state_) {
    case State::Handshake: state_ = State::Identify; break;
    case State::Identify:
      if (!identified_) fail("UNSUPPORTED_MODULE");
      else state_ = State::SetName;
      break;
    case State::SetName: state_ = State::VerifyName; break;
    case State::VerifyName:
      if (!matches_) fail("NAME_READBACK_MISMATCH");
      else state_ = State::SetBleName;
      break;
    case State::SetBleName: state_ = State::VerifyBleName; break;
    case State::VerifyBleName:
      if (!matches_) fail("NAME_READBACK_MISMATCH"); else state_ = State::Succeeded;
      break;
    default: break;
  }
}
const char *NameController::stateName() const {
  switch (state_) {
    case State::Handshake: return "handshaking";
    case State::Idle: return "idle";
    case State::Identify: return "identifying";
    case State::SetName: return "setting_name";
    case State::VerifyName: return "verifying_name";
    case State::SetBleName: return "setting_ble_name";
    case State::VerifyBleName: return "verifying_ble_name";
    case State::Succeeded: return "succeeded";
    case State::Failed: return "failed";
  }
  return "unknown";
}
const char *NameController::modelName() const {
  return identified_ ? "JDY-34" : "unknown";
}
}
