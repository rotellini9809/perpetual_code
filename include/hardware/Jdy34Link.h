#pragma once
#include <cstddef>
#include <cstdint>

namespace bluetooth {
// Portable, nonblocking JDY-34 driver: startup checks, one-time setup of a factory module,
// verified renaming and the notification filter. UART IO belongs to MAIN.
// A module answering at 230400 baud is only queried at startup. Only a module that answers at the
// factory 9600 baud instead is configured (baud, names, single-link mode, notifications), at most
// once per power-up, then restarted and checked like any other module.
class Jdy34Link {
public:
  static constexpr uint32_t UartBaud = 230400, FactoryBaud = 9600;
  // Board names: "Perpetual" by default, or "Perpetual-" plus 1..4 letters, digits or '-'.
  // The BLE name is the same with "_BLE" appended, within the module's 18-byte limit.
  // A suffix of "default" selects "Perpetual".
  static bool boardName(const char *suffix, char *name, std::size_t capacity);
  void begin(uint32_t now, uint32_t settleMs = 250);
  const char *poll(uint32_t now); // At most one CRLF-terminated AT request.
  bool feed(char byte, uint32_t now); // True: one application line, without LF.
  void overflow(); // Invalidate the session and discard through the next LF.
  bool ready() const { return stage_ == Stage::Ready; }
  bool connected() const { return ready() && connected_ != 0; }
  const char *error() const { return error_; }
  uint32_t epoch() const { return epoch_; }
  const char *line() const { return line_; }
  std::size_t length() const { return completed_; }
  uint32_t uartBaud() const { return baud_; } // MAIN reconfigures its UART when this changes.
  const char *name() const { return name_; } // SPP name read at startup; empty when unknown.
  bool configured() const { return configured_; } // A factory module was set up this power-up.
  // Renaming: requestName() validates and queues; MAIN calls startRename() once the reply that
  // accepted the request has been sent, because the module drops its radio link to apply it.
  const char *requestName(const char *suffix);
  bool renamePending() const { return renamePending_; }
  void startRename(uint32_t now);
  const char *renameState() const { return renameState_; }
  const char *stageName() const; // Current or last step, to tell where startup stopped.
private:
  enum class Stage { Settle, Handshake, Version, Mode, Notifications, Name, Status, Ready, Failed,
    SetupHandshake, SetupVersion, SetupStatus, SetupName, SetupBleName, SetupMode, SetupNotifications, SetupBaud, SetupReset,
    RenameHandshake, RenameName, RenameBleName, RenameCheckName, RenameCheckBleName, RenameReset };
  static constexpr uint32_t RestartSettleMs = 1500; // Module reboot after AT+RESET.
  // The module sometimes ignores a request (the first after a quiet period, or one shortly after a
  // restart). Queries are repeated: the startup handshake for about 4 s, other queries 3 times.
  // Writes and resets are sent once: never repeat a setter whose outcome is unknown.
  static constexpr uint8_t StartupAttempts = 4, QueryAttempts = 3;
  Stage stage_ = Stage::Settle;
  char line_[130] = {};
  char notification_[64] = {};
  char name_[20] = {}, requested_[20] = {}, request_[40] = {};
  std::size_t notificationLength_ = 0;
  std::size_t length_ = 0, completed_ = 0;
  uint32_t since_ = 0, hold_ = 0, receivedAt_ = 0, epoch_ = 0, baud_ = UartBaud;
  uint8_t connected_ = 0, known_ = 0, attempts_ = 0;
  bool waiting_ = false, matched_ = false, ack_ = false, discard_ = false;
  bool setupTried_ = false, configured_ = false, renamePending_ = false;
  const char *error_ = "STARTING";
  Stage failedAt_ = Stage::Settle; // Step that failed, kept while stage_ is Failed.
  const char *renameState_ = "None";
  bool setup() const { return stage_ >= Stage::SetupHandshake && stage_ <= Stage::SetupReset; }
  bool renaming() const { return stage_ >= Stage::RenameHandshake && stage_ <= Stage::RenameReset; }
  uint8_t attemptLimit() const;
  void wait(uint32_t now, uint32_t ms) { since_ = now; hold_ = ms; waiting_ = false; }
  void fail(const char *error, uint32_t now);
  void connections(uint8_t mask);
  bool advance(uint32_t now); // True: send the next request now.
  bool timedOut(uint32_t now); // True: send the next request now.
  const char *request();
  void reply(const char *text, bool printable, uint32_t now);
  bool event(const char *line, uint32_t now);
};
}
