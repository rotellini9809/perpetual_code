// PC-only firmware command endpoint over stdin/stdout. No test cases live here.
#include "communication/CommandRouter.h"
#include "ReplayFixture.h"
#include "CommandFixture.h"
#include <iostream>
#include <thread>
using namespace replay_test;
using command_test::send;
struct Device : storage::FlashDevice {
  std::vector<uint8_t> bytes = std::vector<uint8_t>(storage::FlashLayout::Capacity, 255);
  unsigned programs = 0, erases = 0;
  bool failProgram = false;
  bool read(uint32_t a, void *p, std::size_t n) override {
    if (a > bytes.size() || n > bytes.size()-a) return false;
    memcpy(p, bytes.data()+a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    ++programs; if (failProgram || a > bytes.size() || n > bytes.size()-a) return false;
    const auto *in = static_cast<const uint8_t *>(p);
    for (std::size_t i = 0; i < n; ++i) { assert((bytes[a+i] & in[i]) == in[i]); bytes[a+i] &= in[i]; }
    return true;
  }
  bool eraseSector(uint32_t a) override {
    assert(a >= 0x1000 && a%4096 == 0 && a+4096 <= bytes.size()); ++erases;
    memset(bytes.data()+a, 255, 4096); return true;
  }
};
int main(int argc, char **argv) {
  if (argc > 2 || (argc == 2 && strcmp(argv[1], "--queued"))) return 2;
  Device flash; const timing::Clock clock = nullptr;
  storage::ProfiledFlashDevice profiled(flash, clock);
  storage::ConfigurationStore store(profiled); assert(store.save(fixture(), limits, true));
  ConfigurationService config(store, limits); config.boot(true); ConfigurationCommands settings(config);
  flight::RunController run; storage::FlightRecorder recorder(profiled);
  const bool useQueue = argc == 2 && !strcmp(argv[1], "--queued");
  storage::QueuedRecorder queued(recorder, clock);
  std::atomic<bool> exitWorker{false}; std::thread worker;
  if (useQueue) {
    worker = std::thread([&] { queued.workerStarted(); while (!exitWorker.load()) { if (!queued.poll()) std::this_thread::yield(); } });
    while (!queued.online()) std::this_thread::yield();
  }
  storage::RecordingSink *sink = useQueue ? static_cast<storage::RecordingSink *>(&queued) : &recorder;
  simulation::HilSession session(config, run, clock, sensors::FrameConvention::LoggedCalibratedBody, sink);
  BatteryMonitor battery; // PC fixture only: a known ADC reading for storage/export integration.
  battery.poll(0,[]{ return uint16_t(2018); }); session.setBattery(battery);
  SensorHilCommands hil(session); EstimateReplayCommands estimates(config, run);
  FlightCommands flights(recorder, config, run, &profiled, useQueue ? &queued : nullptr); CommandRouter router(settings, estimates, hil, config, run, &flights);
  CommandService usb; usb.setExtension(&router);
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line == "__disconnect__") { router.disconnect(); usb.reset(); std::cout << "{}\n" << std::flush; }
    else if (line == "__damage_current__") {
      assert(!queued.busy()); storage::RecordingInfo info; const auto entry=*recorder.current();
      assert(recorder.inspect(entry.sequence,info) && info.samples);
      flash.bytes[entry.start+info.headerBytes+36]^=1;
      std::cout << "{}\n" << std::flush;
    }
    else std::cout << send(usb, line) << std::flush;
  }
  router.disconnect();
  if (useQueue) { while (queued.busy()) std::this_thread::yield(); exitWorker.store(true); worker.join(); }
  return 0;
}
