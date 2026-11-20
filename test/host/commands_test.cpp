#include <cstdio>
#include "communication/CommandService.h"
#include "communication/CommandPolicy.h"
#include "communication/RecordingReply.h"
#include <cassert>
#include <string>
#include <vector>
#include "../fixtures/CommandFixture.h"
#include "configuration/ConfigurationCommands.h"
#include <cstring>
#include <iostream>
#include "communication/CommandRouter.h"
#include "communication/CommandEndpoint.h"
#include "simulation/EstimateReplayCommands.h"
#include "../fixtures/ReplayFixture.h"
#include "simulation/SensorHilCommands.h"
#include <cmath>
#include <limits>
#include "hardware/StatusLed.h"

struct FakePowerHardware : PowerHardware {
  unsigned sleeps=0, wakes=0;
  const char *sleepError=nullptr, *wakeError=nullptr;
  const char *suspend() override { ++sleeps; return sleepError; }
  const char *resume() override { ++wakes; return wakeError; }
};

// Each namespace isolates one component scenario and its fault model.
namespace test_usb_command_service {
void protocolValues() {
  uint32_t n; float f;
  assert(protocol::unsignedNumber("4294967295", n) && n == UINT32_MAX);
  assert(!protocol::unsignedNumber("4294967296", n));
  assert(!protocol::unsignedNumber("-1", n));
  assert(protocol::floating("nan", f) && !protocol::finiteNumber("nan", f));
  assert(!protocol::floating("1e999", f));
  protocol::Tokens quoted("add 10 \"escaped \\\"name\\\"\"");
  assert(quoted.valid && quoted.count == 3 && quoted.is(2, "escaped \"name\""));
  assert(!protocol::Tokens("\"unfinished").valid);
  assert(!protocol::Tokens(std::string(129, 'x').c_str()).valid);
  assert(!protocol::Tokens("a b c d e f g h i j k l m").valid);
  protocol::Json json; json.quoted("a\n\"\\");
  assert(strcmp(json.bytes, "\"a\\u000a\\\"\\\\\"") == 0);
  json.add("%s", std::string(800, 'x').c_str()); assert(!json.ok);
  const auto length = json.length; json.add("ignored"); assert(json.length == length);
}
std::vector<std::string> send(CommandService &service, const std::string &bytes, uint16_t batteryMv=UINT16_MAX) {
  CommandService::Status status;
  status.uptimeMs = 1234; status.flashJedec = 0xEF4018; status.flashIdentified = true;
  status.batteryMv=batteryMv;
  std::vector<std::string> replies;
  char reply[CommandService::MaxReply];
  for (char byte : bytes)
    if (service.feed(byte, status, reply, sizeof(reply))) replies.emplace_back(reply);
  return replies;
}
void run() {
  CommandService service;
  {
    CommandService custom; CommandService::Status status;
    status.bluetoothBaud=600000; status.bluetoothRxOverflows=3;
    char reply[CommandService::MaxReply]; bool ready=false;
    for (char c : std::string("bluetooth status\n")) ready=custom.feed(c,status,reply,sizeof(reply));
    assert(ready && std::strstr(reply,"\"baud\":600000") && std::strstr(reply,"\"uart_rx_overflows\":3"));
  }
  { // diag: refreshed on request, JSON with an ID, readable text without one.
    CommandService custom; CommandService::Status status; BoardDiagnostics diagnostics; char reply[CommandService::MaxReply];
    auto ask = [&](const char *line) { bool ready=false; for (char c : std::string(line)) ready=custom.feed(c,status,reply,sizeof(reply)); assert(ready); return std::string(reply); };
    assert(ask("@1 diag\n").find("UNAVAILABLE") != std::string::npos);
    status.diagnostics=&diagnostics; diagnostics.loopPeriodUs=2083; diagnostics.run="simulation";
    diagnostics.refresh=[](BoardDiagnostics &d) { d.stack[0]={4096,1500,false}; d.stack[1]={8192,8192,true}; d.loop.add(900); d.loop.add(2500); d.loopOverPeriod=1; d.loopMaxRunMs=1234; d.loopMaxPhase=2; d.loopMaxUsbInput=true; d.frameDtMinUs=2083; d.frameDtMaxUs=2084; };
    const auto json=ask("@2 diag\n");
    assert(json.find("\"stack_bytes\":[[1500,4096],[8192,8192]],\"stack_exhausted\":[false,true]") != std::string::npos);
    assert(json.find("\"loop\":[2,3400,2500],\"loop_period_us\":2083,\"loop_over_period\":1,\"run\":\"simulation\"") != std::string::npos);
    assert(json.find("\"frame_dt_us\":[2083,2084],\"sequence_gaps\":0,\"fifo_overruns\":0") != std::string::npos);
    assert(json.find("\"loop_max_at\":{\"run_ms\":1234,\"phase\":2,\"usb_input\":true}") != std::string::npos);
    const auto text=ask("diag\n");
    assert(text.find("max 2500 us at 1234 ms (phase 2, USB input)") != std::string::npos);
    assert(text.find("Stack core 0: 1500/4096 bytes | core 1: 8192/8192 bytes EXHAUSTED") != std::string::npos);
    assert(ask("@3 diag extra\n").find("INVALID_ARGUMENT") != std::string::npos);
  }
  assert(send(service, "pi").empty());
  assert(send(service, "ng\n").at(0) == "PONG\n");
  auto replies = send(service, "\n \r\n ping\r\ninfo\nstatus\nhelp\n");
  assert(replies.size() == 4 && replies[0] == "PONG\n");
  assert(replies[2].find("Readiness: use preflight") != std::string::npos);
  assert(replies[2].find("EF4018") != std::string::npos);
  replies = send(service, "@42 status\r\n@0 ping\n@4294967295 info\n");
  assert(replies.size() == 3);
  assert(replies[0].find("\"id\":42") != std::string::npos);
  assert(replies[0].find("\"phase\":null") != std::string::npos);
  assert(replies[0].find("\"battery_v\":null") != std::string::npos);
  assert(send(service,"@43 status\n",8240).at(0).find("\"battery_v\":8.240")!=std::string::npos);
  assert(replies[1].find("\"id\":0") != std::string::npos);
  assert(replies[2].find("\"id\":4294967295") != std::string::npos);
  for (const auto &text : replies) {
    assert(text.front() == '{' && text.back() == '\n');
    assert(text.find('\n') == text.size() - 1);
  }
  for (const char *request : {"@4294967296 ping\n", "@-1 ping\n", "@ ping\n", "@1ping\n"})
    assert(send(service, request).at(0).find("INVALID_ARGUMENT") != std::string::npos);
  assert(send(service, "@5 arm\n").at(0).find("UNKNOWN_COMMAND") != std::string::npos);
  assert(send(service, "ping extra\n").at(0).find("INVALID_ARGUMENT") != std::string::npos);
  assert(send(service, "@5 ping extra\n").at(0).find("\"type\":\"error\"") != std::string::npos);
  assert(send(service, std::string("ping\0suffix\n", 12)).at(0).find("INVALID_ARGUMENT") != std::string::npos);
  assert(send(service, "pi\rng\n").at(0).find("INVALID_ARGUMENT") != std::string::npos);
  replies = send(service, std::string(129, 'x') + "ping\nstatus\n");
  assert(replies.size() == 2 && replies[0].find("LINE_TOO_LONG") != std::string::npos);
  assert(replies[1].find("Disarmed") == 0);
  replies = send(service, "@8 " + std::string(130, 'x') + "\nping\n");
  assert(replies[0].find("\"id\":8") != std::string::npos);
  assert(replies[0].find("LINE_TOO_LONG") != std::string::npos && replies[1] == "PONG\n");
  // Exactly MaxLine bytes are accepted; the next byte crosses the boundary.
  replies = send(service, "ping" + std::string(124, ' ') + "\n");
  assert(replies.at(0) == "PONG\n");
  send(service, "sta"); service.reset();
  assert(send(service, "ping\n").at(0) == "PONG\n");
  assert(send(service, "ping\\n").empty()); // Literal backslash-n isn't LF.
  service.reset();
  // Read-only IDs correlate requests; repeating status is safe and reevaluates it.
  assert(send(service, "@2 ping\n@2 ping\n").size() == 2);
  {
    CommandService::Status status;
    status.baroReady = true; status.baroId = 0x50;
    status.baroSampleValid = true; status.pressurePa = 101325; status.temperatureC = 20;
    char output[CommandService::MaxReply];
    std::string result;
    for (char c : std::string("@42 sensors\n"))
      if (service.feed(c, status, output, sizeof(output))) result = output;
    assert(result.find("\"imu_ready\":false") != std::string::npos);
    assert(result.find("\"baro_ready\":true") != std::string::npos);
    assert(result.find("\"pressure_pa\":101325.000") != std::string::npos);
    assert(result.find("\"imu_sample_valid\":false") != std::string::npos && result.find("\"accel_mps2\":null") != std::string::npos);
    status.imuReady = status.imuSampleValid = true; status.imuSampleAgeMs = 7; status.uptimeMs = 12345;
    status.accelMps2[0] = 1.25f; status.accelMps2[1] = -2; status.accelMps2[2] = 9.8f;
    status.gyroRadps[0] = .1f; status.gyroRadps[1] = .2f; status.gyroRadps[2] = .3f;
    for (char c : std::string("@45 sensors\n"))
      if (service.feed(c, status, output, sizeof(output))) result = output;
    assert(result.find("\"accel_mps2\":[1.25,-2,9.8]") != std::string::npos);
    assert(result.find("\"gyro_radps\":[0.1,0.2,0.3]") != std::string::npos && result.find("\"imu_sample_age_ms\":7") != std::string::npos);
    assert(result.find("\"uptime_ms\":12345") != std::string::npos);
    status.imuSampleValid = false; // Stale samples must not expose old vectors.
    for (char c : std::string("@46 sensors\n"))
      if (service.feed(c, status, output, sizeof(output))) result = output;
    assert(result.find("\"accel_mps2\":null") != std::string::npos && result.find("\"gyro_radps\":null") != std::string::npos);
    status.imuSampleValid = true; status.gyroRadps[1] = std::numeric_limits<float>::quiet_NaN();
    for (char c : std::string("@47 sensors\n"))
      if (service.feed(c, status, output, sizeof(output))) result = output;
    assert(result.find("\"imu_sample_valid\":false") != std::string::npos && result.find("\"accel_mps2\":null") != std::string::npos);
    assert(result.size() < CommandService::MaxReply && result.back() == '\n');
    status.imuReady = status.imuSampleValid = false;
    for (char c : std::string("@43 status\n"))
      if (service.feed(c, status, output, sizeof(output))) result = output;
    assert(result.find("\"imu_ready\":false") != std::string::npos && result.find("\"baro_ready\":true") != std::string::npos);
    assert(result.find("\"reasons\"") == std::string::npos && result.back() == '\n'); // Readiness belongs to preflight.
    status.baroSampleValid = false; status.baroSampleAgeMs = 5000;
    for (char c : std::string("@44 sensors\n"))
      if (service.feed(c, status, output, sizeof(output))) result = output;
    assert(result.find("\"pressure_pa\":null") != std::string::npos && result.find("\"temperature_c\":null") != std::string::npos);
    assert(send(service, "sensors extra\n").at(0).find("INVALID_ARGUMENT") != std::string::npos);
    status.armingState = "armed"; status.flightMode = "simulation"; status.phase = 1;
    status.outputsActive = true; status.outputMode = "PHYSICAL";
    status.gyroTarget = 500; status.uptimeMs = UINT32_MAX; status.configError = "UnsupportedVersion";
    for (char c : std::string("@4294967295 status\n"))
      if (service.feed(c, status, output, sizeof(output))) result = output;
    assert(result.find("\"attitude\":null") != std::string::npos);
    status.attitudeValid = true; status.tiltDeg = 34.56f; status.upBody[0] = 0.8236f; status.upBody[1] = -0.5672f; status.upBody[2] = 0;
    for (char c : std::string("@4294967295 status\n"))
      if (service.feed(c, status, output, sizeof(output))) result = output;
    assert(result.find("\"attitude\":{\"tilt_deg\":34.6,\"up_body\":[0.824,-0.567,0.000]}") != std::string::npos);
    status.gyroOnlyValid = true; status.gyroOnlyTiltDeg = 41.04f; status.gyroOnlyUpBody[0] = 0.7543f; status.gyroOnlyUpBody[2] = 0.6568f;
    for (char c : std::string("@4294967295 status\n"))
      if (service.feed(c, status, output, sizeof(output))) result = output;
    assert(result.find("\"attitude\":{\"tilt_deg\":34.6,\"up_body\":[0.824,-0.567,0.000],"
                       "\"gyro_only\":{\"tilt_deg\":41.0,\"up_body\":[0.754,0.000,0.657]}}") != std::string::npos);
    assert(result.size() < CommandService::MaxReply);
    status.gyroOnlyValid = false;
    assert(result.find("\"mode\":\"armed\"") != std::string::npos && result.find("\"phase\":1") != std::string::npos);
    assert(result.find("\"flight_mode\":\"simulation\"") != std::string::npos);
    assert(result.find("\"gyro_calibration_skipped\":true") != std::string::npos);
    assert(result.find("\"outputs_active\":true") != std::string::npos && result.find("\"output_mode\":\"PHYSICAL\"") != std::string::npos);
    assert(result.find("\"uptime_ms\":4294967295}") != std::string::npos && result.size() < CommandService::MaxReply);
  }
  std::puts("USB command service tests passed.");
}
} // namespace test_usb_command_service

namespace test_configuration_commands {
using command_test::send;
using namespace storage;
class MemoryFlash : public FlashDevice {
public:
  std::vector<uint8_t> bytes = std::vector<uint8_t>(storage::FlashLayout::DirectoryA, 255); // Device journal.
  unsigned mutations = 0;
  bool failProgram = false;
  bool read(uint32_t a, void *p, std::size_t n) override {
    if (a > bytes.size() || n > bytes.size() - a) return false;
    memcpy(p, bytes.data() + a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    if (failProgram) return false;
    assert(a + n <= bytes.size()); ++mutations;
    const auto *source = static_cast<const uint8_t *>(p);
    for (std::size_t i = 0; i < n; ++i) { assert((bytes[a + i] & source[i]) == source[i]); bytes[a + i] &= source[i]; }
    return true;
  }
  bool eraseSector(uint32_t a) override {
    assert(a % 4096 == 0 && a + 4096 <= bytes.size()); ++mutations; memset(bytes.data() + a, 255, 4096); return true;
  }
};
const recovery::ValidationLimits limits{1000, 2000, 1000, 3600000};

using command_test::has;
void run() {
  MemoryFlash flash; ConfigurationStore store(flash); ConfigurationService settings(store, limits); ConfigurationCommands config(settings);
  CommandService usb; usb.setExtension(&config); config.boot(true);
  assert(!config.configValid() && flash.mutations == 0);
  has(send(usb, "config show"), "\"available\":false");
  has(send(usb, "capabilities"), "\"max_rules\":8");
  has(send(usb, "pyro set 1 100"), "NO_DRAFT");
  has(send(usb, "@1 config begin"), "\"ok\":true");
  has(send(usb, "@1 config begin"), "\"ok\":true");
  has(send(usb, "@1 config discard"), "ID_CONFLICT");
  has(send(usb, "@0 config discard"), "STALE_REQUEST");
  has(send(usb, "config begin"), "DRAFT_ACTIVE");
  has(send(usb, "config validate"), "EmptyPlan");
  has(send(usb, "rule add 10 \"Main \\\"example\\\"\""), "\"ok\":true");
  has(send(usb, "rule show 10"), "Main \\\"example\\\"");
  has(send(usb, "rule add 10 Duplicate"), "DUPLICATE_RULE_ID");
  has(send(usb, "condition add 10 apogee"), "\"ok\":true");
  has(send(usb, "output assign 10 pyro 1"), "\"ok\":true");
  has(send(usb, "output assign 10 pyro 2 100"), "\"ok\":true");
  has(send(usb, "output show 10 1"), "\"delay_ms\":100");
  has(send(usb, "output unassign 10 pyro 2"), "\"ok\":true");
  has(send(usb, "output assign 10 pyro 2 -1"), "INVALID_ARGUMENT");
  has(send(usb, "output assign 10 pyro 2 3600001"), "INVALID_ARGUMENT");
  has(send(usb, "output assign 10 pyro 2 1 2"), "INVALID_ARGUMENT");
  has(send(usb, "output delay 10 pyro 1 100"), "INVALID_ARGUMENT");
  has(send(usb, "config validate"), "\"valid\":true");
  has(send(usb, "pyro show 1"), "\"pulse_ms\":1000");
  has(send(usb, "rule delay 10 250"), "\"ok\":true");
  has(send(usb, "rule show 10"), "\"delay_ms\":250");
  has(send(usb, "rule delay 10 0"), "\"ok\":true");
  has(send(usb, "pyro set 1 -1"), "INVALID_ARGUMENT");
  has(send(usb, "pyro set 1 100000"), "INVALID_ARGUMENT");
  has(send(usb, "pyro set 1 100"), "\"ok\":true");
  has(send(usb, "config validate"), "\"valid\":true"); assert(flash.mutations == 0);
  has(send(usb, "@2 config apply"), "\"persisted\":true"); assert(config.configValid());
  const auto mutations = flash.mutations;
  has(send(usb, "@2 config apply"), "\"persisted\":true"); assert(flash.mutations == mutations);
  has(send(usb, "@3 config show"), "\"available\":true");
  has(send(usb, "@2 config apply"), "STALE_REQUEST"); assert(flash.mutations == mutations);
  ConfigurationService rebootSettings(store, limits); ConfigurationCommands reboot(rebootSettings); reboot.boot(true); assert(reboot.configValid());
  usb.reset(); usb.setExtension(&reboot);
  has(send(usb, "rule show 10"), "Main \\\"example\\\"");
  has(send(usb, "config begin"), "\"ok\":true");
  has(send(usb, "rule rename 10 Changed"), "\"ok\":true");
  has(send(usb, "config discard"), "\"ok\":true");
  has(send(usb, "rule show 10"), "Main \\\"example\\\"");
  has(send(usb, "config begin"), "\"ok\":true");
  has(send(usb, "rule add 20 Backup"), "\"ok\":true");
  has(send(usb, "condition add 20 since_rule_ms 10 250"), "\"ok\":true");
  has(send(usb, "output assign 20 pyro 1"), "OUTPUT_ALREADY_ASSIGNED");
  has(send(usb, "output assign 20 servo 1"), "\"ok\":true");
  has(send(usb, "servo set 1 1000 2000"), "\"hardware_moved\":false");
  has(send(usb, "servo set 1 1000 2000 500"), "INVALID_ARGUMENT");
  has(send(usb, "rule remove 10"), "RULE_REFERENCED");
  has(send(usb, "condition add 10 since_rule_ms 20 0"), "\"ok\":true");
  has(send(usb, "config validate"), "DependencyCycle");
  has(send(usb, "condition remove 10 1"), "\"ok\":true");
  for (const char *condition : {"phase descent", "altitude < 100", "altitude_range -5 100", "speed >= 10", "speed_range 0 20", "since_launch_ms 100", "since_apogee_ms 100"}) {
    has(send(usb, std::string("condition replace 10 0 ") + condition), "\"ok\":true");
    has(send(usb, "config validate"), "\"valid\":true");
  }
  has(send(usb, "condition replace 10 0 altitude < nan"), "INVALID_ARGUMENT");
  has(send(usb, "condition replace 10 0 altitude < inf"), "INVALID_ARGUMENT");
  has(send(usb, "condition replace 10 0 altitude_range 100 -1"), "\"ok\":true");
  has(send(usb, "config validate"), "InvalidCondition");
  has(send(usb, "condition replace 10 0 apogee"), "\"ok\":true");
  reboot.setDisarmed(false);
  has(send(usb, "pyro set 1 200"), "ARMED");
  has(send(usb, "config show"), "\"ok\":true");
  reboot.setDisarmed(true); reboot.disconnect(); usb.reset();
  has(send(usb, "config apply"), "NO_DRAFT");
  has(send(usb, "config begin"), "\"ok\":true");
  flash.failProgram = true;
  has(send(usb, "config apply"), "STORAGE_ERROR"); assert(!reboot.configValid());
  assert(!reboot.storageReady());
  std::cout << "Configuration USB integration tests passed.\n";
}
} // namespace test_configuration_commands

namespace test_estimate_replay {
using command_test::send;
using namespace replay_test;

using command_test::has;
void run() {
  MemoryFlash flash; storage::ConfigurationStore store(flash);
  assert(store.save(fixture(), limits, true));
  ConfigurationService settings(store, limits); ConfigurationCommands config(settings);
  flight::RunController run; config.boot(true);
  EstimateReplayCommands replay(settings, run); simulation::HilSession hilSession(settings, run); SensorHilCommands hil(hilSession);
  CommandRouter router(config, replay, hil, settings, run); CommandService usb; usb.setExtension(&router);

  const auto before = flash.mutations;
  has(send(usb, "replay sample 0 0 0 0"), "NOT_REPLAYING");
  has(send(usb, "config begin"), "\"ok\":true");
  has(send(usb, "replay begin"), "DRAFT_ACTIVE");
  send(usb, "config discard");
  has(send(usb, "replay begin"), "VIRTUAL");
  has(send(usb, "config begin"), "BUSY");
  has(send(usb, "config apply"), "BUSY");
  has(send(usb, "config show"), "\"available\":true");
  has(send(usb, "replay begin"), "BUSY");
  has(send(usb, "replay sample 0 0 0 0"), "\"starts\":[]");
  has(send(usb, "replay sample 1 1 10 20"), "\"triggered\":[]");
  has(send(usb, "@1 replay sample 100 3 100 0"), "[10,\"pyro\",1,100]");
  const auto duplicate = send(usb, "@1 replay sample 100 3 100 0"); has(duplicate, "[10,\"pyro\",1,100]");
  has(send(usb, "replay status"), "\"samples\":3");
  has(send(usb, "replay sample 100 3 100 0"), "INVALID_TIMESTAMP");
  has(send(usb, "replay sample 99 3 100 0"), "INVALID_TIMESTAMP");
  has(send(usb, "replay sample 101 6 100 0"), "INVALID_ARGUMENT");
  has(send(usb, "replay sample 101 3 garbage 0"), "INVALID_ARGUMENT");
  has(send(usb, "replay sample 101 3 1e999 0"), "INVALID_ARGUMENT");
  has(send(usb, "replay sample -1 3 100 0"), "INVALID_ARGUMENT");
  has(send(usb, "replay sample 4294967296 3 100 0"), "INVALID_ARGUMENT");
  has(send(usb, "replay sample 350 4 nan nan"), "\"starts\":[]");
  has(send(usb, "replay sample 400 4 20 -10"), "\"triggered\":[20]");
  has(send(usb, "replay sample 450 4 20 -10"), "\"triggered\":[]");
  has(send(usb, "replay sample 500 4 20 -10"), "[20,\"pyro\",2,500]");
  has(send(usb, "replay sample 600 5 0 0"), "\"phase\":4");
  has(send(usb, "replay stop"), "\"active\":false");
  has(send(usb, "replay begin"), "\"active\":true");
  has(send(usb, "replay sample 0 3 100 0"), "[10,\"pyro\",1,0]");
  has(send(usb, "replay stop"), "\"cancelled_pending\":0");
  send(usb, "replay begin"); router.disconnect(); usb.reset();
  has(send(usb, "replay status"), "\"active\":false");
  assert(flash.mutations == before); // Replay/config inspections never touch flash.
  auto maximum = fixture(); maximum.ruleCount = 8;
  for (unsigned i = 0; i < 4; ++i) {
    maximum.pyros[i].pulseMs = 100; maximum.servos[i] = maximum.servos[0];
  }
  for (unsigned i = 0; i < 8; ++i) {
    auto &r = maximum.rules[i]; r = {}; r.id = 0xFFFFFFFFu - i;
    strcpy(r.name.data(), "All outputs"); r.conditionCount = r.assignmentCount = 1;
    r.assignments[0].kind = i < 4 ? recovery::OutputKind::Pyro : recovery::OutputKind::Servo;
    r.assignments[0].channel = uint8_t(i % 4 + 1);
  }
  assert(store.save(maximum, limits, true)); config.boot(true);
  has(send(usb, "replay begin"), "\"active\":true");
  const auto all = send(usb, "replay sample 4294967295 3 100 0");
  has(all, "[4294967295,\"pyro\",1,4294967295]");
  has(all, "[4294967288,\"servo\",4,4294967295]");
  assert(all.size() < CommandService::MaxReply);
  std::cout << "Estimate replay USB tests passed (no flash mutations).\n";
}
} // namespace test_estimate_replay

namespace test_sensor_hil {
using command_test::send;
using namespace replay_test;

using command_test::has;
uint32_t fakeClock() { static uint32_t time = 0xFFFFFFF0u; time += 20; return time; }
class FakeHardwareSource : public sensors::SensorSource {
public:
  sensors::Reading reading;
  bool pending = true;
  bool poll(sensors::Reading &out) override { if (!pending) return false; out = reading; pending = false; return true; }
  sensors::FrameConvention convention() const override { return sensors::FrameConvention::SensorAxes; }
};
void sourceTests() {
  FakeHardwareSource hardware;
  hardware.reading.imuValid = true; hardware.reading.imuSequence = 1;
  hardware.reading.accelMps2[0] = 9.80665f; hardware.reading.gyroRadps[1] = 0.1f;
  hardware.reading.baroValid = true; hardware.reading.pressurePa = 101325;
  navigation::SensorCalibration calibration;
  calibration.configured = true; calibration.gyroBias[1] = 0.1f;
  calibration.upAxis = navigation::UpAxis::NegativeZ;
  sensors::HilSensorSource hil;
  assert(hil.submit(hardware.reading) && !hil.submit(hardware.reading));
  navigation::SensorPipeline normal, injected;
  const navigation::SensorCalibration unset;
  assert(!normal.begin(hardware, unset, 0, 200));
  assert(normal.begin(hardware, calibration, 0, 200));
  assert(injected.begin(hil, calibration, 0, 200));
  assert(!normal.begin(hil, calibration, 0, 200)); // No source replacement while active.
  navigation::NavigationOutput a, b;
  assert(normal.poll(a) && injected.poll(b));
  assert(a.valid && b.valid && a.altitude == b.altitude && a.verticalVelocity == b.verticalVelocity);
  assert(std::fabs(a.verticalSpecificForce - 9.80665f) < 1e-4f);
  assert(!normal.poll(a) && !injected.poll(b));
  normal.end();
  auto invalid = calibration; invalid.upAxis = static_cast<navigation::UpAxis>(255);
  assert(!normal.begin(hardware, invalid, 0, 200));
  sensors::HilSensorSource logged(sensors::FrameConvention::LoggedCalibratedBody);
  assert(normal.begin(logged, unset, 0, 200));
  auto body = hardware.reading; body.accelMps2[0] = 0; body.accelMps2[2] = 9.80665f; body.gyroRadps[1] = 0;
  assert(logged.submit(body)); assert(normal.poll(b)); assert(b.valid);
  assert(b.altitude == a.altitude); // Logged-body adapter avoids a second rotation/bias subtraction.
  assert(b.verticalSpecificForce == a.verticalSpecificForce);
  for (unsigned i = 0; i < 3; ++i) assert(b.eulerRadians[i] == a.eulerRadians[i]);
}
void mountingTests() {
  using navigation::UpAxis;
  // Inverse mappings: encode one BODY trajectory in each physical sensor mounting.
  // Bias is always attached to sensor axes, regardless of mounting selection.
  const int sensorAxes[6][3] = {{1,2,3}, {-1,2,-3}, {-2,1,3}, {2,-1,3}, {-3,2,1}, {3,2,-1}};
  navigation::SensorCalibration defaults;
  assert(defaults.upAxis == UpAxis::PositiveX);
  for (unsigned mounting = 0; mounting < 6; ++mounting) {
    FakeHardwareSource hardware;
    sensors::HilSensorSource body(sensors::FrameConvention::LoggedCalibratedBody);
    navigation::SensorCalibration calibration;
    calibration.configured = true;
    if (mounting) calibration.upAxis = static_cast<UpAxis>(mounting); // Omitted means +X.
    calibration.accelBias[0] = 0.25f; calibration.accelBias[1] = -0.5f; calibration.accelBias[2] = 0.125f;
    calibration.gyroBias[0] = -0.125f; calibration.gyroBias[1] = 0.25f; calibration.gyroBias[2] = 0.5f;
    navigation::SensorPipeline rawPipeline, bodyPipeline;
    assert(rawPipeline.begin(hardware, calibration, 0, 200));
    assert(bodyPipeline.begin(body, defaults, 0, 200));
    for (unsigned n = 0; n < 80; ++n) {
      sensors::Reading expected;
      expected.imuValid = expected.baroValid = true;
      expected.timeUs = expected.baroTimeUs = n * 5000;
      expected.imuSequence = expected.baroSequence = n + 1;
      expected.pressurePa = 101325;
      expected.accelMps2[0] = n < 20 ? 9.75f : n < 60 ? 35.0f : -0.25f;
      expected.accelMps2[1] = 0.25f; expected.accelMps2[2] = -0.125f;
      expected.gyroRadps[0] = 0.03125f; expected.gyroRadps[1] = -0.015625f; expected.gyroRadps[2] = 0.0078125f;
      hardware.reading = expected; hardware.pending = true;
      for (unsigned i = 0; i < 3; ++i) {
        const int axis = sensorAxes[mounting][i];
        const unsigned j = unsigned(axis < 0 ? -axis : axis) - 1;
        const float sign = axis < 0 ? -1.0f : 1.0f;
        hardware.reading.accelMps2[i] = sign * expected.accelMps2[j] + calibration.accelBias[i];
        hardware.reading.gyroRadps[i] = sign * expected.gyroRadps[j] + calibration.gyroBias[i];
      }
      assert(body.submit(expected));
      navigation::NavigationOutput a, b;
      assert(rawPipeline.poll(a) && bodyPipeline.poll(b) && a.valid && b.valid);
      assert(a.phase == b.phase && a.altitude == b.altitude && a.verticalVelocity == b.verticalVelocity);
      assert(a.verticalSpecificForce == b.verticalSpecificForce && a.eulerRadians == b.eulerRadians);
    }
  }
}
void run() {
  MemoryFlash flash; storage::ConfigurationStore store(flash); assert(store.save(fixture(), limits, true));
  ConfigurationService settings(store, limits); ConfigurationCommands config(settings);
  flight::RunController run; config.boot(true);
  EstimateReplayCommands estimates(settings, run);
  simulation::HilSession hilSession(settings, run, fakeClock);
  SensorHilCommands hil(hilSession);
  CommandRouter router(config, estimates, hil, settings, run);
  CommandService usb; usb.setExtension(&router);

  sourceTests(); mountingTests(); const auto before = flash.mutations;
  has(send(usb, "hil sample 0 0 0 9.81 0 0 0 101325"), "NOT_REPLAYING");
  send(usb, "config begin"); has(send(usb, "hil begin 0 200 logged_body"), "DRAFT_ACTIVE"); send(usb, "config discard");
  has(send(usb, "hil begin 0 200 raw_axes"), "INVALID_ARGUMENT");
  has(send(usb, "hil begin 0 200 logged_body"), "\"active\":true");
  has(send(usb, "config begin"), "BUSY"); has(send(usb, "replay begin"), "BUSY");
  has(send(usb, "config show"), "\"available\":true");
  has(send(usb, "@1 hil sample 0 0 0 9.80665 0 0 0 101325"), "\"imu_valid\":true");
  has(send(usb, "@1 hil sample 0 0 0 9.80665 0 0 0 101325"), "\"imu_valid\":true");
  has(send(usb, "hil status"), "\"samples\":1");
  has(send(usb, "hil sample 0 0 0 9.81 0 0 0 101325"), "INVALID_TIMESTAMP");
  has(send(usb, "hil sample 5 nan 0 9.81 0 0 0 nan"), "\"imu_valid\":false");
  has(send(usb, "hil sample 10 0 0 9.81 0 0 0 nan"), "\"imu_valid\":true");
  has(send(usb, "hil sample 11 junk 0 0 0 0 0 101325"), "INVALID_ARGUMENT");
  has(send(usb, "hil sample 11 1e999 0 0 0 0 0 101325"), "INVALID_ARGUMENT");
  has(send(usb, "hil sample -1 0 0 0 0 0 0 101325"), "INVALID_ARGUMENT");
  has(send(usb, "hil stop"), "\"max_compute_us\":20");
  has(send(usb, "replay begin"), "\"active\":true");
  has(send(usb, "hil begin 0 200 logged_body"), "BUSY"); send(usb, "replay stop");
  send(usb, "hil begin 0 200 logged_body"); router.disconnect(); usb.reset();
  has(send(usb, "hil status"), "\"active\":false"); assert(flash.mutations == before);
  auto all = fixture(); all.ruleCount = 8;
  for (unsigned i = 0; i < 4; ++i) { all.pyros[i].pulseMs = 100; all.servos[i] = all.servos[0]; }
  for (unsigned i = 0; i < 8; ++i) {
    auto &r = all.rules[i]; r = {}; r.id = 0xFFFFFFFFu - i; strcpy(r.name.data(), "All");
    r.conditionCount = r.assignmentCount = 1; r.conditions[0].kind = recovery::ConditionKind::Phase;
    r.conditions[0].phases = recovery::AirborneMask;
    r.assignments[0].kind = i < 4 ? recovery::OutputKind::Pyro : recovery::OutputKind::Servo;
    r.assignments[0].channel = uint8_t(i % 4 + 1);
  }
  assert(store.save(all, limits, true)); config.boot(true);
  send(usb, "hil begin 0 200 logged_body");
  for (uint32_t t = 4294967095u; t < 4294967295u; t += 5) // BOOST: 200 ms after a 3 g sample.
    send(usb, "hil sample " + std::to_string(t) + " 35 0 0 0 0 0 nan");
  const auto burst = send(usb, "hil sample 4294967295 35 0 0 0 0 0 nan");
  has(burst, "[4294967295,\"pyro\",1,4294967295]");
  has(burst, "[4294967288,\"servo\",4,4294967295]");
  assert(burst.size() < CommandService::MaxReply);
  std::cout << "Sensor source/HIL tests passed: common calibration path, locking, timing, freshness and no flash writes.\n";
}
} // namespace test_sensor_hil

namespace test_recording_commands {
using command_test::send;
using namespace replay_test;
uint32_t elapsedUs = 0;
uint32_t clockUs() { return elapsedUs; }
struct Device : storage::FlashDevice {
  std::vector<uint8_t> bytes = std::vector<uint8_t>(storage::FlashLayout::Capacity, 255);
  unsigned programs = 0, erases = 0;
  bool failProgram = false;
  bool read(uint32_t a, void *p, std::size_t n) override {
    elapsedUs += 10;
    if (a > bytes.size() || n > bytes.size()-a) return false;
    memcpy(p, bytes.data()+a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    elapsedUs += 100;
    ++programs; if (failProgram || a > bytes.size() || n > bytes.size()-a) return false;
    const auto *in = static_cast<const uint8_t *>(p);
    for (std::size_t i = 0; i < n; ++i) { assert((bytes[a+i] & in[i]) == in[i]); bytes[a+i] &= in[i]; }
    return true;
  }
  bool eraseSector(uint32_t a) override {
    elapsedUs += 1000;
    assert(a%4096 == 0 && a+4096 <= bytes.size()); ++erases;
    memset(bytes.data()+a, 255, 4096); return true;
  }
};

using command_test::has;
void run() {
  Device flash; const timing::Clock clock = clockUs;
  storage::ProfiledFlashDevice profiled(flash, clock);
  storage::ConfigurationStore store(profiled); assert(store.save(fixture(), limits, true));
  ConfigurationService config(store, limits); config.boot(true); ConfigurationCommands settings(config);
  flight::RunController run; storage::FlightRecorder recorder(profiled);
  storage::QueuedRecorder queued(recorder, clock);

  storage::RecordingSink *sink = &recorder;
  simulation::HilSession session(config, run, clock, sensors::FrameConvention::LoggedCalibratedBody, sink);
  SensorHilCommands hil(session); EstimateReplayCommands estimates(config, run);
  FlightCommands flights(recorder, config, run, &profiled, nullptr, clock); CommandRouter router(settings, estimates, hil, config, run, &flights);
  CommandService usb; usb.setExtension(&router);

  const auto before = flash.programs;
  has(send(usb, "hil begin 0 200 logged_body record 4096"), "STORAGE_NOT_READY");
  has(send(usb, "hil begin 0 200 logged_body"), "\"active\":true");
  has(send(usb, "hil sample 0 0 0 9.80665 0 0 0 101325"), "\"imu_valid\":true");
  send(usb, "hil stop"); assert(flash.programs == before); // Plain HIL remains write-free.
  flash.bytes[storage::FlashLayout::DirectoryA+512] = 0; // Blank headers alone are not enough to authorize initialization.
  has(send(usb, "flights init"), "NotBlank"); assert(flash.programs == before);
  flash.bytes[storage::FlashLayout::DirectoryA+512] = 255;
  has(send(usb, "flights init"), "\"ready\":true");
  const auto initialized = flash.programs; send(usb, "flights init"); assert(flash.programs == initialized);
  const auto timingWrites = flash.programs;
  has(send(usb, "flights timing reset"), "\"reset\":true"); assert(flash.programs == timingWrites);
  has(send(usb, "flights timing"), "\"program\":[0,0,0]");
  has(send(usb, "hil begin 0 200 logged_body record 4096"), "\"recording_sequence\":\"1\"");
  has(send(usb, "flights timing reset"), "BUSY");
  has(send(usb, "flights timing"), "\"timing_valid\":true");
  has(send(usb, "hil timing"), "\"nominal_period_us\":5000");
  has(send(usb, "flights info 1"), "BUSY"); has(send(usb, "config begin"), "BUSY");
  has(send(usb, "flights delete 1"), "BUSY");
  for (unsigned i = 0; i < 45; ++i) {
    const auto command = "hil sample " + std::to_string(i*5) + " 0 0 9.80665 0 0 0 101325 22.5";
    has(send(usb, command), "\"ok\":true");
  }
  // All 45 samples are PAD: they stay in the pre-launch history and are written at close.
  has(send(usb, "hil stop"), "\"saved_samples\":45");
  assert(session.timing().prepareUs > 0 && session.timing().finishUs > 0);
  assert(session.timing().step.count == 45 && session.timing().storage.count == 45);
  has(send(usb, "hil timing"), "\"timing_valid\":true");
  has(send(usb, "flights timing"), "\"failures\":[0,0,0]");
  has(send(usb, "hil status"), "\"dropped_samples\":0");
  has(send(usb, "flights info 1"), "\"samples\":45");
  has(send(usb, "flights info 1"), "\"status\":2");
  has(send(usb, "flights info 1"), "\"sample_transport\":\"raw-v4\"");
  // Raw block bodies travel as 448-byte base64 chunks: 1792 = 4 x 448.
  has(send(usb, "flights block 1 0 0"), "\"bytes\":448,\"frames\":45,\"data\":\"UFNCSwQAAAc");
  has(send(usb, "flights block 1 0 1344"), "\"bytes\":448");
  has(send(usb, "flights block 1 0 1792"), "Bounds");
  has(send(usb, "flights info 18446744073709551616"), "INVALID_ARGUMENT");
  has(send(usb, "flights block 1 1000 0"), "Bounds");
  has(send(usb, "flights list 1"), "\"available\":false");
  has(send(usb, "hil begin 0 200 logged_body record 4096"), "\"recording_sequence\":\"2\"");
  has(send(usb, "@500 hil sample 0 0 0 9.80665 0 0 0 101325"), "\"ok\":true");
  has(send(usb, "@500 hil sample 0 0 0 9.80665 0 0 0 101325"), "\"ok\":true");
  router.disconnect(); usb.reset();
  has(send(usb, "flights info 2"), "\"samples\":1");
  has(send(usb, "flights info 2"), "\"status\":2");
  has(send(usb, "flights list 0"), "\"sequence\":\"2\"");
  has(send(usb, "flights list 1"), "\"sequence\":\"1\"");
  // PAD never reaches flash: fly (BOOST) long enough to fill and write one block.
  send(usb, "hil begin 0 200 logged_body record 4096"); flash.failProgram = true;
  for (unsigned i = 0; i < 60; ++i) has(send(usb, "hil sample " + std::to_string(i*5) + " 40 0 9.80665 0 0 0 101325"), "\"imu_valid\":true");
  has(send(usb, "hil status"), "\"recording_error\":\"Sample\"");
  has(send(usb, "hil status"), "\"active\":true");
  has(send(usb, "flights failure"), "\"component\":\"Sample\"");
  has(send(usb, "flights failure"), "\"cause\":\"Io\"");
  send(usb, "hil stop"); flash.failProgram = false;
  has(send(usb, "flights recover"), "\"ready\":true");
  has(send(usb, "flights info 3"), "\"status\":3");
  config.setDisarmed(false);
  has(send(usb, "flights info 1"), "ARMED");
  has(send(usb, "flights init"), "ARMED");
  const auto beforeRejectedDelete=flash.programs;
  has(send(usb, "flights delete 1"), "ARMED"); assert(flash.programs==beforeRejectedDelete);
  config.setDisarmed(true);
  // Exercise maximum-width sample values, identities and all eight start slots.
  recovery::Configuration wide; wide.ruleCount = 1;
  auto &rule = wide.rules[0]; rule.id = UINT32_MAX; strcpy(rule.name.data(), "All eight outputs");
  rule.conditionCount = 1; rule.assignmentCount = recovery::MaxAssignments;
  for (unsigned i = 0; i < recovery::ChannelCount; ++i) {
    wide.pyros[i].pulseMs = 100;
    auto &servo = wide.servos[i]; servo.calibrated = true;
    servo.initialUs = 1000; servo.finalUs = 2000;
    rule.assignments[i].channel = uint8_t(i+1);
    rule.assignments[i+4].kind = recovery::OutputKind::Servo; rule.assignments[i+4].channel = uint8_t(i+1);
  }
  assert(recorder.begin(wide, limits, 4096, storage::SensorMode::Simulation, storage::OutputMode::Virtual,
                        sensors::FrameConvention::LoggedCalibratedBody));
  for (const auto &a : rule.assignments) assert(recorder.recordStart(rule.id, a.kind, a.channel, UINT32_MAX));
  storage::FlightSample sample; sample.phase = recovery::FlightPhase::Descent; // LANDED would be thinned to 10 Hz.
  sample.values.fill(-std::numeric_limits<float>::max()); sample.timeUs = UINT64_MAX-1; sample.imuSequence = UINT32_MAX-1;
  assert(recorder.append(sample)); ++sample.timeUs; ++sample.imuSequence; assert(recorder.append(sample)); assert(recorder.finish(true));
  const auto info = send(usb, "@4294967294 flights info 4");
  has(info, "\"ok\":true"); has(info, "[4294967295,1,4,2,4294967295]");
  const auto rows = send(usb, "@4294967295 flights block 4 0 0");
  has(rows, "\"ok\":true"); has(rows, "\"frames\":2");
  assert(info.size() < CommandService::MaxReply && rows.size() < CommandService::MaxReply);
  // Keep the worker paused: stop/disconnect return immediately, and every flash
  // mutation/new run remains gated until ownership is explicitly acknowledged.
  queued.workerStarted();
  simulation::HilSession asyncSession(config,run,clock,sensors::FrameConvention::LoggedCalibratedBody,&queued);
  SensorHilCommands asyncHil(asyncSession);
  simulation::SimulationSession asyncSim(config,run,clock,&queued); SimulationCommands simCommands(asyncSim);
  flight::ArmingController physical(config,run,clock,&queued);
  sensors::HilSensorSource missingHardware;
  struct MissingChecks : flight::PhysicalChecks {
    flight::PhysicalReadiness inspect() override { return {}; }
  } missingChecks;
  ArmingCommands arming(physical,missingHardware,missingChecks,&asyncSim,&asyncSession);
  FlightCommands asyncFlights(recorder,config,run,&profiled,&queued);
  CommandRouter asyncRouter(settings,estimates,asyncHil,config,run,&asyncFlights,&simCommands,&arming);
  FakePowerHardware powerHardware; PowerManager power(powerHardware); asyncRouter.setPower(power);
  usb.setExtension(&asyncRouter); usb.reset();
  send(usb,"hil begin 0 200 logged_body record 4096");
  send(usb,"hil sample 0 0 0 9.80665 0 0 0 101325");
  const auto closingWrites=flash.programs, closingErases=flash.erases;
  has(send(usb,"hil stop completed"),"\"recording_busy\":true");
  has(send(usb,"disarm"),"\"arming_state\":\"closing\"");
  assert(!run.active() && queued.busy() && flash.programs==closingWrites);
  for (const auto *command : {"sleep","config begin","config apply","hil begin 0 200 logged_body","replay begin","sim begin","flights recover","flights info 1","flights timing"})
    has(send(usb,command),"BUSY");
  has(send(usb,"config show"),"\"ok\":true");
  has(send(usb,"hil status"),"\"recording_busy\":true");
  assert(flash.programs==closingWrites && flash.erases==closingErases);
  while (queued.busy()) assert(queued.poll());
  has(send(usb,"hil status"),"\"recording_busy\":false");
  has(send(usb,"hil status"),"\"saved_samples\":1");
  send(usb,"hil begin 0 200 logged_body record 4096");
  send(usb,"hil sample 0 0 0 9.80665 0 0 0 101325");
  asyncRouter.disconnect(); assert(queued.busy());
  while (queued.busy()) assert(queued.poll());
  // A late persistence failure is observable after stop already returned.
  send(usb,"hil begin 0 200 logged_body record 4096");
  send(usb,"hil sample 0 0 0 9.80665 0 0 0 101325");
  has(send(usb,"hil stop"),"\"recording_busy\":true");
  flash.failProgram=true; while (queued.busy()) queued.poll(); flash.failProgram=false;
  has(send(usb,"hil status"),"\"recording_error\":\"Sample\"");
  has(send(usb,"hil status"),"\"recording_busy\":false");
  send(usb,"flights recover");
  has(send(usb,"sim begin record"),"\"active\":true");
  const auto simOrigin=elapsedUs;
  for (uint32_t t=0;t<=sensors::SimulatedSensorSource::durationMs();++t) { elapsedUs=simOrigin+t*1000; asyncSim.poll(); }
  assert(!asyncSim.active() && asyncSim.status().recordingBusy && asyncSim.status().savedSamples==0);
  while (queued.busy()) assert(queued.poll());
  // The paused worker only ever saw the first 128 queued (PAD) samples: they are
  // reduced to the pre-launch history (0.5 s at full rate plus 10 Hz before).
  const auto simSaved=asyncSim.status().savedSamples;
  assert(asyncSim.status().samples==6600 && simSaved>=100 && simSaved<=110 && !asyncSim.status().recordingBusy);
  send(usb,"hil begin 0 200 logged_body record 4096");
  // Starting a different HIL recording must not replace the previous simulation's final report.
  assert(asyncSim.status().samples==6600 && asyncSim.status().savedSamples==simSaved && !asyncSim.status().recordingBusy);
  send(usb,"hil stop"); while (queued.busy()) queued.poll();
  // A damaged interrupted flight is visible without blocking healthy history.
  assert(recorder.begin(fixture(),limits,4096,storage::SensorMode::Simulation,storage::OutputMode::Virtual,
                        sensors::FrameConvention::LoggedCalibratedBody));
  const auto damaged=*recorder.current(); const auto headerBytes=recorder.metadata()->layout.headerBytes;
  // 120 airborne 15-byte FRAMEs: the first block (114) is committed, the rest stays in RAM.
  for(unsigned i=0;i<120;++i) {
    storage::FlightSample s; s.phase=recovery::FlightPhase::Boost; s.timeUs=i*1000; s.imuSequence=i+1; assert(recorder.append(s));
  }
  assert(recorder.service() && recorder.committedSamples()==114); recorder.abort(); flash.bytes[damaged.start+headerBytes+36]^=1;
  const auto damagedBytes=std::vector<uint8_t>(flash.bytes.begin()+damaged.start,flash.bytes.begin()+damaged.start+damaged.allocatedBytes);
  has(send(usb,"flights recover"),"\"corrupt_flights\":1");
  has(send(usb,"flights status"),"\"corrupt_flights\":1");
  has(send(usb,"flights list 0"),"\"corrupt\":true");
  has(send(usb,"flights list 0"),"\"status\":6");
  has(send(usb,"flights info "+std::to_string(damaged.sequence)),"Corrupt");
  has(send(usb,"flights info 1"),"\"samples\":45");
  has(send(usb,"flights recover"),"\"ready\":true");
  assert(std::equal(damagedBytes.begin(),damagedBytes.end(),flash.bytes.begin()+damaged.start));
  usb.setExtension(&router); usb.reset(); router.setPower(power);
  has(send(usb,"flights block 1 0 0"),"\"ok\":true");
  has(send(usb,"sleep"),"BUSY"); assert(!powerHardware.sleeps);
  elapsedUs+=5000000; // A stopped/cancelled transfer's lease expires without retaining ownership.
  has(send(usb,"sleep"),"\"power_mode\":\"sleeping\"");
  has(send(usb,"wake"),"\"power_mode\":\"active\"");
  std::cout << "Recording USB tests passed: explicit blank initialization, opt-in writes, full/drop counts, ID retries, disconnect, storage-fault isolation and bounded downloads.\n";
}
} // namespace test_recording_commands

namespace test_dual_transport {
using namespace replay_test;
using command_test::send;
uint32_t clock() { return 1000; }
bool pending=false;
uint32_t powerMs=0;
uint32_t powerClock() { return powerMs; }
bool transportBusy() { return pending; }
using command_test::has;
std::string drain(CommandEndpoint &endpoint, unsigned chunk) {
  std::string result;
  while(endpoint.pending()) {
    const auto n=endpoint.remaining()<chunk ? endpoint.remaining() : chunk;
    result.append(reinterpret_cast<const char *>(endpoint.data()),n); endpoint.sent(n);
  }
  return result;
}
void run() {
  MemoryFlash flash; storage::ConfigurationStore store(flash); assert(store.save(fixture(),limits,true));
  ConfigurationService settings(store,limits); settings.boot(true);
  ConfigurationCommands configuration(settings); flight::RunController run;
  EstimateReplayCommands replay(settings,run);
  simulation::HilSession hilSession(settings,run); SensorHilCommands hil(hilSession);
  simulation::SimulationSession sim(settings,run,clock); SimulationCommands simulation(sim);
  CommandRouter router(configuration,replay,hil,settings,run,nullptr,&simulation);
  CommandSession usbSession(router,CommandRouter::Source::Usb), btSession(router,CommandRouter::Source::Bluetooth);
  CommandService usb, bt; usb.setExtension(&usbSession); bt.setExtension(&btSession);
  // IDs, partial frames and retry caches belong to the individual endpoint.
  has(send(usb,"@1 config begin"),"\"draft_active\":true");
  has(send(bt,"@1 ping"),"PONG");
  has(send(bt,"control status"),"\"owner\":\"usb\"");
  has(send(bt,"config discard"),"CONTROL_BUSY"); assert(settings.hasDraft());
  btSession.disconnect(); assert(settings.hasDraft());
  usbSession.disconnect(); assert(!settings.hasDraft()); usb.reset(); bt.reset();
  has(send(bt,"@1 config begin"),"\"draft_active\":true");
  const auto cached=send(bt,"@1 config begin");
  assert(send(bt,"@1 config begin")==cached);
  has(send(usb,"config begin"),"CONTROL_BUSY");
  has(send(usb,"config show"),"\"ok\":true");
  has(send(bt,"config discard"),"\"ok\":true");
  has(send(usb,"replay begin"),"\"ok\":true"); assert(replay.active());
  has(send(bt,"replay sample 0 0 0 0"),"CONTROL_BUSY");
  has(send(bt,"replay status"),"\"active\":true");
  btSession.disconnect(); assert(replay.active());
  usbSession.disconnect(); assert(!replay.active());
  has(send(bt,"hil begin 0 200 logged_body"),"\"ok\":true"); assert(hil.active());
  has(send(usb,"hil sample 0 0 0 9.80665 0 0 0 101325"),"CONTROL_BUSY");
  has(send(bt,"hil sample 0 0 0 9.80665 0 0 0 101325"),"\"ok\":true");
  usbSession.disconnect(); assert(hil.active());
  btSession.disconnect(); assert(!hil.active());
  has(send(bt,"sim begin"),"\"ok\":true"); assert(sim.active());
  has(send(usb,"sim stop"),"CONTROL_BUSY");
  has(send(usb,"sim timing"),"\"ok\":true");
  usbSession.disconnect(); assert(sim.active());
  btSession.disconnect(); assert(sim.active()); // Autonomous flight survives its owner.
  bt.reset();
  has(send(usb,"sim stop"),"\"ok\":true"); assert(!sim.active()); // Explicit takeover.
  has(send(bt,"control status"),"\"owner\":\"none\"");
  has(send(bt,"bluetooth status"),"NOT_INITIALIZED");
  has(send(bt,"bluetooth status extra"),"INVALID_ARGUMENT");
  has(send(bt,"control status extra"),"INVALID_ARGUMENT");
  // A slow radio reply neither overwrites its own response nor blocks USB.
  CommandEndpoint usbEndpoint(usbSession), btEndpoint(btSession); CommandService::Status status;
  for(char c:std::string("pi")) btEndpoint.feed(c,status);
  for(char c:std::string("ping\n")) usbEndpoint.feed(c,status);
  assert(drain(usbEndpoint,3)=="PONG\n" && !btEndpoint.pending());
  for(char c:std::string("ng\n")) btEndpoint.feed(c,status);
  btEndpoint.sent(1);
  for(char c:std::string("ping\n")) usbEndpoint.feed(c,status);
  assert(drain(usbEndpoint,1)=="PONG\n" && drain(btEndpoint,2)=="ONG\n");
  for(char c:std::string("@77 config begin\n")) btEndpoint.feed(c,status);
  const auto response=drain(btEndpoint,1);
  for(char c:std::string("@77 config begin\n")) btEndpoint.feed(c,status);
  assert(drain(btEndpoint,7)==response);
  btSession.disconnect(); btEndpoint.reset();
  for(char c:std::string("@1 ping\n")) btEndpoint.feed(c,status);
  has(drain(btEndpoint,1),"PONG"); // Reconnection resets numbering and partial TX.
  FakePowerHardware hardware; PowerManager power(hardware,powerClock); router.setPower(power,transportBusy);
  has(send(usb,"wake"),"\"power_mode\":\"active\""); assert(!hardware.wakes);
  has(send(usb,"sleep now"),"INVALID_ARGUMENT"); assert(!hardware.sleeps);
  settings.setDisarmed(false); has(send(usb,"sleep"),"BUSY"); settings.setDisarmed(true);
  has(send(usb,"config begin"),"\"ok\":true");
  has(send(bt,"sleep"),"CONTROL_BUSY"); has(send(usb,"sleep"),"BUSY"); send(usb,"config discard");
  send(usb,"sim begin"); has(send(usb,"sleep"),"BUSY"); send(usb,"sim stop");
  pending=true; has(send(usb,"sleep"),"BUSY"); pending=false; assert(!hardware.sleeps);
  hardware.sleepError="SENSOR_SLEEP_FAILED";
  has(send(usb,"sleep"),"SENSOR_SLEEP_FAILED"); assert(!power.sleeping()); hardware.sleepError=nullptr;
  const auto writes=flash.mutations;
  has(send(usb,"sleep"),"\"power_mode\":\"sleeping\""); assert(hardware.sleeps==2);
  has(send(bt,"sleep"),"\"power_mode\":\"sleeping\""); assert(hardware.sleeps==2);
  for (const char *command : {"ping","status","info","help","capabilities","control status","bluetooth status","sim status","sim timing","sim events"})
    send(bt,command);
  assert(power.sleeping() && !hardware.wakes);
  hardware.wakeError="FLASH_WAKE_FAILED";
  for (const char *command : {"arm","config begin","servo preview 1 1500","servo arm","pyro continuity","flights recover","sim begin","replay begin","hil begin 0 200 logged_body","preflight","sensors"}) {
    has(send(bt,command),"FLASH_WAKE_FAILED"); assert(power.sleeping());
  }
  assert(!settings.hasDraft() && !run.active()); // Wake failure never executes the original request.
  hardware.wakeError="FLASH_WAKE_FAILED"; has(send(bt,"wake"),"FLASH_WAKE_FAILED"); assert(power.sleeping());
  hardware.wakeError=nullptr; has(send(bt,"config begin"),"\"draft_active\":true");
  assert(!power.sleeping() && settings.hasDraft()); send(bt,"config discard");
  assert(flash.mutations==writes);
  // A connected client can keep polling for five minutes without preventing sleep.
  for (unsigned i=0;i<300;++i) {
    assert(!power.sleeping()); powerMs+=1000;
    send(usb,"ping"); send(usb,"status"); send(usb,"control status"); send(usb,"sim status"); router.pollPower();
  }
  assert(power.sleeping());
  has(send(bt,"config show"),"\"ok\":true"); assert(!power.sleeping());
  powerMs+=PowerManager::IdleMs-1; send(usb,"sensors"); router.pollPower(); assert(!power.sleeping());
  powerMs+=PowerManager::IdleMs-1; router.pollPower(); assert(!power.sleeping());
  ++powerMs; pending=true; router.pollPower(); assert(!power.sleeping());
  pending=false; router.pollPower(); assert(power.sleeping());
  send(usb,"sim begin"); assert(!power.sleeping() && sim.active());
  powerMs+=PowerManager::IdleMs*2; router.pollPower(); assert(!power.sleeping());
  send(usb,"sim stop"); powerMs+=PowerManager::IdleMs-1; router.pollPower(); assert(!power.sleeping());
  ++powerMs; router.pollPower(); assert(power.sleeping());
  send(usb,"wake"); assert(!power.sleeping() && flash.mutations==writes);
  puts("Dual transport passed: independent framing/retries/backpressure, shared ownership and source-specific disconnect/takeover.");
}
}

namespace test_power_idle {
uint32_t now=UINT32_MAX-100;
uint32_t clock() { return now; }
void run() {
  FakePowerHardware hardware; PowerManager power(hardware,clock); power.poll(false);
  now+=PowerManager::IdleMs-1; power.poll(false); assert(!hardware.sleeps);
  hardware.sleepError="SENSOR_SLEEP_FAILED";
  ++now; power.poll(false); assert(hardware.sleeps==1 && !power.sleeping());
  assert(!strcmp(power.error(),"SENSOR_SLEEP_FAILED"));
  ++now; power.poll(false); assert(hardware.sleeps==1); // No per-loop hardware retry after an automatic failure.
  hardware.sleepError=nullptr; now+=PowerManager::IdleMs; power.poll(false); assert(power.sleeping());
  assert(!strcmp(power.error(),"None")); power.wake();
  now+=PowerManager::IdleMs-1; power.poll(false); assert(!power.sleeping());
  ++now; power.poll(false); assert(power.sleeping());
}
}

namespace test_status_led {
uint32_t color=0; unsigned writes=0;
void write(uint8_t r,uint8_t g,uint8_t b) { color=(uint32_t(r)<<16)|(uint32_t(g)<<8)|b; ++writes; }
void run() {
  StatusLed led(write);
  const uint32_t expected[]={0xFFA500,0xFF0000,0xFF00FF,0x00FFFF,0x00FF00,0x000050};
  for (unsigned i=0;i<6;++i) { led.update(i*20,static_cast<LedMode>(i)); assert(color==expected[i]); }
  const auto count=writes; led.update(1000,LedMode::Disarmed); assert(writes==count);
  led.update(1100,LedMode::Calibrating); assert(color==255);
  led.update(1900,LedMode::Calibrating); assert(color==40);
  led.update(2700,LedMode::Calibrating); assert(color==255);
  led.update(UINT32_MAX-10,LedMode::PowerSaving); assert(color==0xFF0000);
  const auto before=writes; led.update(1,LedMode::PowerSaving); assert(writes==before);
  led.update(29,LedMode::PowerSaving); assert(color==0xFE0001); // Wrap-safe animation, full colour amplitude.
}
}

namespace test_command_policy_and_fragments {
void run() {
  using protocol::CommandGroup;
  enum { Read=1, Mutation=2, Operational=4, Maintenance=8, Begin=16 };
  struct Case { const char *command; const char *args; CommandGroup group; unsigned flags; };
  const Case cases[] = {
    {"config","machine",CommandGroup::Configuration,Read|Operational},
    {"config","machine bistage 35",CommandGroup::Configuration,Mutation|Operational|Maintenance},
    {"config","orientation",CommandGroup::Configuration,Read|Operational},
    {"config","orientation +X extra",CommandGroup::Configuration,Mutation|Operational|Maintenance},
    {"config","reset",CommandGroup::Configuration,Mutation|Operational|Maintenance},
    {"config","draft",CommandGroup::Configuration,Read|Operational},
    {"rule","list",CommandGroup::Configuration,Read|Operational},
    {"servo","release",CommandGroup::Configuration,Mutation|Operational},
    {"airbrake","status",CommandGroup::Configuration,Read},
    {"config","airbrakes",CommandGroup::Configuration,Read|Operational},
    {"config","airbrakes on",CommandGroup::Configuration,Mutation|Operational|Maintenance},
    {"servo","show",CommandGroup::Configuration,Read|Operational},
    {"pyro","continuity",CommandGroup::Configuration,Read|Operational},
    {"config","show \"unterminated",CommandGroup::Configuration,Read|Operational},
    {"flights","block 1 0 0",CommandGroup::Flights,Read|Operational},
    {"flights","format",CommandGroup::Flights,Mutation|Operational|Maintenance},
    {"flights","timing reset",CommandGroup::Flights,Mutation|Operational},
    {"flights","recover",CommandGroup::Flights,Mutation|Operational},
    {"sim","status",CommandGroup::Simulation,Read},
    {"sim","events",CommandGroup::Simulation,Read},
    {"sim","begin physical record",CommandGroup::Simulation,Mutation|Operational|Begin},
    {"sim","stop",CommandGroup::Simulation,Mutation|Operational},
    {"hil","begin",CommandGroup::Hil,Mutation|Operational|Begin},
    {"hil","timing",CommandGroup::Hil,Read},
    {"replay","begin",CommandGroup::Replay,Mutation|Operational|Begin},
    {"arm","extra",CommandGroup::Other,Mutation|Operational|Begin},
    {"ds","",CommandGroup::Other,Mutation|Operational},
    {"preflight","physical",CommandGroup::Other,Read|Operational},
    {"sensors","",CommandGroup::Other,Read|Operational},
    {"sleep","",CommandGroup::Power,Mutation},
    {"wake","extra",CommandGroup::Power,Mutation},
    {"control","status",CommandGroup::Control,Read},
    {"unknown","format",CommandGroup::Other,Read},
  };
  for (const auto &c : cases) {
    const protocol::Tokens tokens(c.args);
    const auto p=protocol::classifyCommand(c.command,tokens);
    const unsigned flags=(p.readOnly ? Read : 0)|(p.mutation ? Mutation : 0)|(p.operational ? Operational : 0)|
                         (p.maintenance ? Maintenance : 0)|(p.beginRun ? Begin : 0);
    assert(p.group==c.group && flags==c.flags);
  }
  flight::RuntimeStatus status;
  status.recordingSequence=UINT64_MAX; status.savedSamples=17; status.droppedSamples=3;
  status.pendingSamples=7; status.recordingBusy=true; status.recordingError=storage::RecorderError::Sample;
  protocol::Json fragment;
  protocol::appendRecordingBusy(fragment,status);
  protocol::appendRecordingCounts(fragment,status);
  protocol::appendRecordingError(fragment,status);
  protocol::appendRecordingPending(fragment,status);
  assert(!strcmp(fragment.bytes,R"(,"recording_busy":true,"recording_sequence":"18446744073709551615","saved_samples":17,"dropped_samples":3,"recording_error":"Sample","pending_samples":7)"));

}
}

int main() {
  test_command_policy_and_fragments::run();
  std::puts("  usb_command_service"); std::fflush(stdout);
  test_usb_command_service::protocolValues();
  test_usb_command_service::run();
  std::puts("  configuration_commands"); std::fflush(stdout);
  test_configuration_commands::run();
  std::puts("  estimate_replay"); std::fflush(stdout);
  test_estimate_replay::run();
  std::puts("  sensor_hil"); std::fflush(stdout);
  test_sensor_hil::run();
  std::puts("  recording_commands"); std::fflush(stdout);
  test_recording_commands::run();
  test_dual_transport::run();
  test_power_idle::run();
  test_status_led::run();
}
