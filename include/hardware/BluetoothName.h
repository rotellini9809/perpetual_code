#pragma once
#include <cstddef>

namespace bluetooth {
// JDY-34-only, transport-independent rename transaction. The module owns persistent storage;
// no name is added to the flight configuration or the external flash.
class NameController {
public:
  static constexpr std::size_t MaxNameBytes = 18;
  struct Command { char text[32]{}; };
  enum class State { Idle, Handshake, Identify, SetName, VerifyName, SetBleName, VerifyBleName, Succeeded, Failed };
  // Call on the UI's Save action, with exclusive UART ownership while disarmed.
  // Acceptance starts an asynchronous operation, not a successful rename.
  const char *setName(const char *name);
  bool nextCommand(Command &output); // At most one command outstanding.
  bool acceptReply(const char *line); // Only feed replies to that command.
  void finishCommand(); // Call after reply quiet time or transport deadline.
  bool active() const { return state_ >= State::Handshake && state_ <= State::VerifyBleName; }
  bool verified() const { return state_ == State::Succeeded; }
  bool mayHaveChanged() const { return mayHaveChanged_; }
  const char *requestedName() const { return name_; }
  const char *verifiedName() const { return verified() ? name_ : ""; }
  const char *error() const { return error_; }
  State state() const { return state_; }
  const char *stateName() const;
  const char *modelName() const;
  static bool validName(const char *name);
private:
  void fail(const char *error) { state_ = State::Failed; error_ = error; }
  char name_[MaxNameBytes+1]{};
  State state_ = State::Idle;
  bool identified_ = false;
  unsigned handshakeAttempts_ = 0;
  bool awaiting_ = false, received_ = false, rejected_ = false, matches_ = false, mayHaveChanged_ = false;
  const char *error_ = "None";
};
}
