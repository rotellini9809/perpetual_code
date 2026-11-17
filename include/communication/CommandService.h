#pragma once
#include <cstddef>
#include <cstdint>
#include <limits>

class CommandExtension {
public:
  virtual ~CommandExtension() = default;
  virtual bool dispatch(const char *command, const char *arguments, bool hasId, uint32_t id,
                        char *output, std::size_t capacity) = 0;
};

class CommandService {
public:
  static constexpr std::size_t MaxLine = 128;
  static constexpr std::size_t MaxReply = 1024;
  struct Status {
    uint32_t uptimeMs = 0;
    uint16_t batteryMv = UINT16_MAX;
    const char *powerMode="active";
    const char *powerError="None";
    uint32_t flashJedec = 0;
    bool flashIdentified = false;
    bool flashReady = false;
    bool configValid = false;
    bool recordingReady = false;
    bool estimateReplayActive = false;
    bool sensorHilActive = false;
    bool simulationActive = false;
    bool outputsActive = false;
    const char *outputMode = "VIRTUAL";
    bool imuReady = false, baroReady = false, imuSampleValid = false, baroSampleValid = false;
    uint8_t imuId = 0, baroId = 0;
    int32_t imuError = 0, baroError = 0;
    float pressurePa = std::numeric_limits<float>::quiet_NaN();
    float temperatureC = std::numeric_limits<float>::quiet_NaN();
    float accelMps2[3] = {}, gyroRadps[3] = {};
    uint32_t imuSampleAgeMs = 0, baroSampleAgeMs = 0;
    const char *armingState = "disarmed", *flightMode = "none";
    int phase = -1;
    uint32_t gyroSamples = 0, gyroTarget = 0;
    const char *configError = "NotLoaded";
    uint32_t bluetoothBaud = 9600, bluetoothRxOverflows = 0;
    bool bluetoothReady = false, bluetoothConnected = false;
    const char *bluetoothError = "NOT_INITIALIZED";
    const char *bluetoothName = ""; // Module SPP name read at startup; BLE adds "_BLE".
    const char *bluetoothRename = "None"; // None, PENDING, ACTIVE, OK or the failure code.
    bool bluetoothConfigured = false; // A factory module was set up during this power-up.
    const char *bluetoothStage = ""; // Startup step, or the step that failed.
  };
  // Consume one byte; true means one LF-terminated reply is ready.
  // Caller must deliver that reply before feeding more bytes.
  bool feed(char byte, const Status &status, char *reply, std::size_t capacity);
  void reset();
  void setExtension(CommandExtension *extension) { extension_ = extension; }
private:
  CommandExtension *extension_ = nullptr;
  bool haveId_ = false;
  uint32_t lastId_ = 0;
  char lastRequest_[MaxLine + 1] = {};
  char lastReply_[MaxReply] = {};
  char line_[MaxLine + 1] = {};
  std::size_t length_ = 0;
  bool overflow_ = false;
  bool invalid_ = false;
  bool dispatch(const Status &status, char *reply, std::size_t capacity);
};
