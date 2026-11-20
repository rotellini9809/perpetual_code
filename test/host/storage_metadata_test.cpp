#include <cstdio>
#include "storage/ConfigurationStore.h"
#include "storage/ImuCalibrationStore.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>
#include "storage/FlightHeaderStore.h"
#include "storage/FlightSampleStore.h"
#include "storage/BinaryEncoding.h"
#include "../fixtures/ReplayFixture.h"
#include "storage/FlightDirectory.h"

// Each namespace isolates one component scenario and its fault model.
namespace test_configuration_store {
using namespace storage;
using namespace recovery;
const ValidationLimits limits{1000, 2000, 1000, 60000};
struct PowerLoss {};
// Block 0 (device journal) plus a guard region that must never change.
class MemoryFlash : public FlashDevice {
public:
  std::vector<uint8_t> bytes = std::vector<uint8_t>(0x14000, 0xA5);
  long budget = -1;
  unsigned writes = 0, erases = 0;
  bool ioFail = false;
  MemoryFlash() { std::memset(bytes.data(), 255, FlashLayout::DirectoryA); }
  void tick() { if (budget == 0) throw PowerLoss{}; if (budget > 0) --budget; }
  bool read(uint32_t a, void *p, std::size_t n) override {
    if (ioFail || a > bytes.size() || n > bytes.size() - a) return false;
    std::memcpy(p, bytes.data() + a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    if (ioFail) return false;
    assert(a + n <= FlashLayout::DirectoryA && a % 256 == 0 && n == 256);
    const auto *source = static_cast<const uint8_t *>(p); ++writes;
    for (std::size_t i = 0; i < n; ++i) {
      assert(bytes[a + i] == 255); tick(); bytes[a + i] &= source[i];
    }
    return true;
  }
  bool eraseSector(uint32_t a) override {
    if (ioFail) return false;
    assert(a % 4096 == 0 && a < FlashLayout::DirectoryA); ++writes; ++erases;
    for (unsigned i = 0; i < 4096; ++i) { tick(); bytes[a + i] = 255; }
    return true;
  }
};
Configuration plan() {
  Configuration c; c.ruleCount = 1; c.pyros[0].pulseMs = 100;
  auto &r = c.rules[0]; r.id = 0x12345678; std::strcpy(r.name.data(), "Main");
  r.conditionCount = r.assignmentCount = 1; r.assignments[0].channel = 1;
  r.conditions[0].kind = ConditionKind::AltitudeComparison;
  r.conditions[0].comparison = Comparison::Less; r.conditions[0].value = -12.5f;
  return c;
}
std::vector<uint8_t> encoded(const Configuration &c) {
  std::vector<uint8_t> bytes(ConfigurationStore::MaxPayload); std::size_t n;
  assert(ConfigurationStore::encode(c, bytes.data(), bytes.size(), n)); bytes.resize(n); return bytes;
}
void neighbors(const MemoryFlash &f) {
  for (std::size_t i = FlashLayout::DirectoryA; i < f.bytes.size(); ++i) assert(f.bytes[i] == 0xA5);
}
// Newest record of `type` (highest sequence) across both banks: its header address.
uint32_t newest(const MemoryFlash &f, uint8_t type) {
  uint32_t found = UINT32_MAX; uint64_t best = 0;
  for (uint32_t a = 0; a < FlashLayout::DirectoryA; a += 256)
    if (binary::get32(f.bytes.data() + a, 4) == 0x52564450 && f.bytes[a + 4] == type) {
      const auto sequence = binary::get64(f.bytes.data() + a + 8, 8);
      if (found == UINT32_MAX || sequence >= best) { best = sequence; found = a; }
    }
  assert(found != UINT32_MAX); return found;
}
// Rewrites a record header CRC after a deliberate edit (simulates a valid but different record).
void resealHeader(MemoryFlash &f, uint32_t header) {
  const auto crc = binary::crc(f.bytes.data() + header, 252);
  binary::put(f.bytes.data() + header + 252, crc, 4);
  const uint32_t length = binary::get32(f.bytes.data() + header + 16, 4);
  const uint32_t commit = header + 256 * (1 + (length + 255) / 256);
  binary::put(f.bytes.data() + commit + 4, crc, 4); binary::put(f.bytes.data() + commit + 8, ~crc, 4);
}
void run() {
  auto c = plan(); const auto payload = encoded(c);
  assert(payload[0] == 1 && payload[1] == 100);
  assert(payload[37] == 0x78 && payload[38] == 0x56 && payload[39] == 0x34 && payload[40] == 0x12);
  Configuration out;
  assert(ConfigurationStore::decode(payload.data(), payload.size(), out)); assert(encoded(out) == payload);
  // Channel delays occupy the existing four-byte assignment slot, without changing offsets.
  auto oldDelay = payload; binary::put(oldDelay.data()+oldDelay.size()-6, 1000, 4);
  assert(ConfigurationStore::decode(oldDelay.data(), oldDelay.size(), out));
  assert(out.rules[0].assignments[0].delayMs == 1000 && encoded(out) == oldDelay);
  for (std::size_t n = 0; n < payload.size(); ++n) assert(!ConfigurationStore::decode(payload.data(), n, out));
  auto trailing = payload; trailing.push_back(0); assert(!ConfigurationStore::decode(trailing.data(), trailing.size(), out));
  auto malformed = payload; malformed[0] = 255; assert(!ConfigurationStore::decode(malformed.data(), malformed.size(), out));
  std::size_t used = 99; uint8_t tiny[1]; assert(!ConfigurationStore::encode(c, tiny, 1, used) && used == 0);

  MemoryFlash flash; ConfigurationStore store(flash);
  out.ruleCount = 7;
  assert(!store.load(out, limits) && store.error() == ConfigError::Missing && out.ruleCount == 7);
  const auto unchanged = flash.bytes;
  assert(!store.save(c, limits, false) && store.error() == ConfigError::Armed && flash.bytes == unchanged);
  assert(!store.save(Configuration{}, limits, true) && store.error() == ConfigError::InvalidConfiguration && flash.bytes == unchanged);
  // First save initializes bank A (one 32 KiB erase), then appends the rules record.
  assert(store.save(c, limits, true) && flash.erases == 8);
  assert(store.load(out, limits) && encoded(out) == payload); neighbors(flash);
  // Current policy is revalidated at boot, even if package integrity is valid.
  auto restricted = limits; restricted.maxPyroPulseMs = 50;
  out.ruleCount = 7; assert(!store.load(out, restricted) && store.error() == ConfigError::InvalidConfiguration && out.ruleCount == 7);
  const auto baseline = flash;
  c.pyros[0].pulseMs = 200; c.rules[0].assignments[0].delayMs = 250; const auto replacement = encoded(c);
  // A power cut anywhere in a replacement keeps the old rules or completes the new ones; never Missing.
  long cuts = 0;
  for (long cut = 0; cut <= 3 * 256 + 16; ++cut, ++cuts) {
    auto f = baseline; f.budget = cut; ConfigurationStore writer(f);
    try { writer.save(c, limits, true); } catch (const PowerLoss &) {}
    f.budget = -1; ConfigurationStore boot(f); Configuration recovered;
    assert(boot.load(recovered, limits));
    const auto bytes = encoded(recovered); assert(bytes == payload || bytes == replacement);
    neighbors(f);
  }
  // Committed rules damaged later: Corrupt, never a silent fall back to the older copy.
  auto twice = baseline; ConfigurationStore second(twice); assert(second.save(c, limits, true));
  auto broken = twice; broken.bytes[newest(broken, 3) + 256 + 10] ^= 1; ConfigurationStore corrupt(broken);
  assert(!corrupt.load(out, limits) && corrupt.error() == ConfigError::Corrupt);
  // A newest record without its commit never completed: the previous rules are used.
  auto torn = twice; const auto header = newest(torn, 3);
  const uint32_t commit = header + 256 * (1 + uint32_t((payload.size() + 2 + 255) / 256));
  std::memset(torn.bytes.data() + commit, 255, 256); ConfigurationStore fallback(torn);
  assert(fallback.load(out, limits) && encoded(out) == payload);
  // Unknown codec versions are reported, not decoded.
  for (uint8_t version : {1,2,4}) {
    auto future = baseline; const auto at = newest(future, 3);
    future.bytes[at + 256] = version; // Codec version prefix of the payload.
    binary::put(future.bytes.data() + at + 20, binary::crc(future.bytes.data() + at + 256, payload.size() + 2), 4);
    resealHeader(future, at); ConfigurationStore unsupported(future);
    assert(!unsupported.load(out, limits) && unsupported.error() == ConfigError::UnsupportedVersion);
  }
  auto semantic = baseline; const auto at = newest(semantic, 3);
  semantic.bytes[at + 256 + 2 + 95] = 99; // Unknown condition kind, with valid CRCs.
  binary::put(semantic.bytes.data() + at + 20, binary::crc(semantic.bytes.data() + at + 256, payload.size() + 2), 4);
  resealHeader(semantic, at); ConfigurationStore invalid(semantic);
  assert(!invalid.load(out, limits) && invalid.error() == ConfigError::InvalidConfiguration);
  auto io = baseline; io.ioFail = true; ConfigurationStore unavailable(io);
  assert(!unavailable.load(out, limits) && unavailable.error() == ConfigError::Io);

  // Settings are separate types: damage to one never touches another.
  assert(store.saveOrientation(4, true) && store.saveAirbrakes(true, true));
  StateMachineSettings machine; machine.bistage = true; machine.minimumCoastVelocity = 35.5f;
  assert(store.saveStateMachine(machine, true));
  auto damaged = flash; damaged.bytes[newest(damaged, 2) + 256] ^= 1; ConfigurationStore settings(damaged);
  uint8_t axis = 0; bool enabled = false; StateMachineSettings loaded;
  assert(!settings.loadOrientation(axis) && settings.error() == ConfigError::Corrupt);
  assert(settings.loadAirbrakes(enabled) && enabled && settings.loadStateMachine(loaded) && loaded.bistage && loaded.minimumCoastVelocity == 35.5f);
  assert(settings.saveOrientation(2, true) && settings.loadOrientation(axis) && axis == 2); // A new save clears the damage.

  // Compaction keeps the newest copy of every type, and a carried damage flag survives it.
  auto cycling = flash; ConfigurationStore cycle(cycling);
  auto rot = cycling; rot.bytes[newest(rot, 2) + 256] ^= 1; ConfigurationStore rotStore(rot);
  for (unsigned i = 0; i < 120; ++i) {
    c.pyros[0].pulseMs = uint16_t(100 + i % 50);
    assert(cycle.save(c, limits, true)); assert(rotStore.save(c, limits, true));
  }
  assert(cycle.load(out, limits) && encoded(out) == encoded(c));
  assert(cycle.loadOrientation(axis) && axis == 4 && cycle.loadAirbrakes(enabled) && enabled);
  assert(cycle.loadStateMachine(loaded) && loaded.bistage);
  assert(!rotStore.loadOrientation(axis) && rotStore.error() == ConfigError::Corrupt); // Still Corrupt after compactions.
  neighbors(cycling);
  // Power cuts across a compaction: rules and settings always load as before or after.
  auto full = cycling; ConfigurationStore filler(full);
  for (;;) { // Fill the active bank until the next save must compact.
    auto probe = full; ConfigurationStore trial(probe); const auto erasesBefore = probe.erases;
    assert(trial.save(c, limits, true));
    if (probe.erases != erasesBefore) break;
    assert(filler.save(c, limits, true));
  }
  Configuration previous; { ConfigurationStore r(full); assert(r.load(previous, limits)); }
  const auto before = encoded(previous);
  c.pyros[0].pulseMs = 190; const auto compacted = encoded(c);
  for (long cut = 0; cut <= 8 * 4096 + 40 * 256; cut += 97, ++cuts) {
    auto f = full; f.budget = cut; ConfigurationStore writer(f);
    try { writer.save(c, limits, true); } catch (const PowerLoss &) {}
    f.budget = -1; ConfigurationStore boot(f); Configuration recovered;
    assert(boot.load(recovered, limits));
    assert(boot.loadOrientation(axis) && axis == 4 && boot.loadStateMachine(loaded) && loaded.bistage);
    const auto bytes = encoded(recovered); assert(bytes == compacted || bytes == before);
  }
  // Factory reset: rules and settings go, IMU calibration and the identity revocation stay.
  ImuCalibrationStore imu(flash); ImuCalibrationRecord record; record.revision = 3; record.approval = ImuApproval::Verified;
  imu.load(); assert(imu.save(record) && imu.invalidate() && !imu.identityAllowed());
  assert(store.clear(true));
  assert(!store.load(out, limits) && store.error() == ConfigError::Missing && !store.loadOrientation(axis));
  assert(store.loadAirbrakes(enabled) && !enabled && store.loadStateMachine(loaded) && !loaded.bistage);
  ImuCalibrationStore afterReset(flash); assert(afterReset.load() && afterReset.record().revision == 3 && !afterReset.identityAllowed());
  // Test-only identity restore: calibration and revocation go, settings stay.
  assert(store.saveOrientation(2, true) && afterReset.restoreIdentityForTesting());
  assert(!afterReset.present() && afterReset.identityAllowed() && !afterReset.needsCalibration());
  ImuCalibrationStore restored(flash); assert(!restored.load() && restored.identityAllowed() && !restored.present());
  assert(store.loadOrientation(axis) && axis == 2);
  neighbors(flash);
  // Maximum current structural payload fits one journal record.
  Configuration maximum;
  maximum.ruleCount = MaxRules;
  for (std::size_t i = 0; i < 4; ++i) {
    maximum.pyros[i].pulseMs = 100; auto &s = maximum.servos[i];
    s.calibrated = true; s.initialUs = 1000; s.finalUs = 2000;
  }
  for (std::size_t i = 0; i < MaxRules; ++i) {
    auto &r = maximum.rules[i]; r.id = uint32_t(i + 1); std::strcpy(r.name.data(), "Example");
    r.conditionCount = MaxConditions; r.assignmentCount = 1;
    r.assignments[0].kind = i < 4 ? OutputKind::Pyro : OutputKind::Servo;
    r.assignments[0].channel = uint8_t(i % 4 + 1);
  }
  assert(store.save(maximum, limits, true)); assert(store.load(out, limits));
  assert(encoded(maximum) == encoded(out)); neighbors(flash);
  // Codec supports every condition/assignment slot even before semantic validation.
  for (auto &r : maximum.rules) r.assignmentCount = MaxAssignments;
  const auto largest = encoded(maximum);
  assert(largest.size() == 2103 && largest.size() < ConfigurationStore::MaxPayload);
  assert(ConfigurationStore::decode(largest.data(), largest.size(), out)); assert(encoded(out) == largest);
  std::cout << "Configuration store tests passed: " << cuts << " power cuts (saves and compaction); max payload " << largest.size() << " bytes.\n";
}
} // namespace test_configuration_store

namespace test_flight_header_store {
using namespace storage;
using namespace replay_test;
struct Cut {};
struct HeaderFlash : FlashDevice {
  std::vector<uint8_t> bytes = std::vector<uint8_t>(0x40000, 255);
  std::vector<unsigned> pageWrites = std::vector<unsigned>(0x40000 / 256, 0);
  long budget = -1;
  unsigned writes = 0, reads = 0, failAfterProgram = 0;
  bool failRead = false, failProgram = false;
  bool read(uint32_t a, void *p, std::size_t n) override {
    ++reads; if (failRead || a > bytes.size() || n > bytes.size() - a) return false;
    memcpy(p, bytes.data() + a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    assert(a >= FlashLayout::RingBegin && a + n <= bytes.size() && a % 256 == 0 && n == 256);
    assert(pageWrites[a/256]++ == 0); ++writes;
    if (failProgram) return false;
    const auto *in = static_cast<const uint8_t *>(p);
    for (unsigned i = 0; i < n; ++i) {
      if (budget == 0) throw Cut{};
      if (budget > 0) --budget;
      assert((bytes[a+i] & in[i]) == in[i]); bytes[a+i] &= in[i];
    }
    return writes != failAfterProgram;
  }
  bool eraseSector(uint32_t) override { assert(false); return false; }
};
bool create(FlightHeaderStore &store, const recovery::Configuration &config = fixture(), uint32_t address = 0x30000) {
  return store.create(address, config, limits, SensorMode::Simulation, OutputMode::Virtual, sensors::FrameConvention::LoggedCalibratedBody);
}
std::vector<uint8_t> encoded(const recovery::Configuration &config) {
  std::vector<uint8_t> bytes(ConfigurationStore::MaxPayload); std::size_t n;
  assert(ConfigurationStore::encode(config, bytes.data(), bytes.size(), n)); bytes.resize(n); return bytes;
}
void calibrationRoundtrip() {
  HeaderFlash flash; FlightHeaderStore header(flash);
  FlightCalibration c; c.available=c.sensorAxes=c.gyroCalibrated=true; c.upAxis=5;
  c.referenceAltitude=123.25f; c.airbrakeLawVersion=37;
  c.accelScale[0]=1.03125f; c.accelScale[2]=.96875f; c.accelCalibrationRevision=123;
  for (unsigned i=0;i<3;++i) { c.gyroBias[i]=(int(i)-1)*.03125f; c.accelBias[i]=(int(i)-1)*.25f; }
  auto invalid=c; invalid.gyroBias[0]=std::numeric_limits<float>::infinity();
  assert(!header.create(0x30000,fixture(),limits,SensorMode::Hardware,OutputMode::Physical,sensors::FrameConvention::SensorAxes,invalid) && !flash.writes);
  assert(header.create(0x30000,fixture(),limits,SensorMode::Hardware,OutputMode::Physical,sensors::FrameConvention::SensorAxes,c));
  c.gyroBias[0]=999; c.referenceAltitude=999; // Mutating current settings cannot change a recorded flight.
  FlightHeaderStore reboot(flash); assert(reboot.load(0x30000));
  const auto &saved=reboot.view()->calibration;
  assert(saved.airbrakeLawVersion==37);
  assert(saved.accelScale[0]==1.03125f && saved.accelScale[2]==.96875f && saved.accelCalibrationRevision==123);
  assert(saved.available && saved.sensorAxes && saved.gyroCalibrated && saved.upAxis==5);
  assert(saved.referenceAltitude==123.25f && saved.gyroBias[0]==-.03125f && saved.accelBias[2]==.25f);
  for (unsigned offset:{160u,161u,162u,164u,168u,208u}) {
    auto damaged=flash; auto *p=damaged.bytes.data()+0x30000;
    if (offset==164 || offset==168 || offset==208) binary::put(p+offset,0x7f800000,4);
    else p[offset]=offset==161 ? 6 : 255;
    if (offset==162) p[offset]=0;
    const auto length=(reboot.view()->layout.configBytes+255)/256*256;
    const auto checksum=binary::extendCrc(binary::extendCrc(binary::CrcInitial,p,252),p+512,length)^binary::CrcInitial;
    binary::put(p+252,checksum,4); binary::put(p+260,checksum,4); binary::put(p+264,~checksum,4);
    FlightHeaderStore bad(damaged); assert(!bad.load(0x30000) && bad.error()==HeaderError::Corrupt);
  }
}
void run() {
  calibrationRoundtrip();
  HeaderFlash flash; FlightHeaderStore header(flash);
  FlightHeaderLayout layout;
  assert(header.plan(fixture(), limits, layout)); assert(layout.slotCount == 3 && layout.headerBytes == 4096);
  assert(layout.summaryOffset == layout.tableOffset + layout.slotCount * FlightHeaderStore::SlotBytes);
  assert(flash.writes == 0 && flash.reads == 0);
  const auto pristine = flash.bytes;
  assert(!create(header, fixture(), 0x14000) && header.error() == HeaderError::Bounds);
  assert(!create(header, fixture(), 0x30001));
  assert(!create(header, fixture(), 0xFFFFF000));
  assert(!create(header, recovery::Configuration{}) && header.error() == HeaderError::InvalidConfiguration);
  assert(!header.create(0x30000, fixture(), limits, static_cast<SensorMode>(255), OutputMode::Virtual,
                        sensors::FrameConvention::LoggedCalibratedBody));
  assert(flash.bytes == pristine && flash.writes == 0);
  assert(!header.load(0x30000) && header.error() == HeaderError::Empty);
  auto config = fixture(); config.rules[0].assignments[0].delayMs = 75;
  config.rules[0].assignments[1].delayMs = 200; config.rules[1].assignments[0].delayMs = 325;
  assert(create(header, config));
  const auto frozen = config;
  const auto saved = encoded(config); config.rules[0].name[0] = 'X'; config.pyros[0].pulseMs = 999;
  assert(encoded(header.view()->configuration) == saved); // Frozen complete plan.
  assert(header.view()->layout.tableOffset == layout.tableOffset);
  assert(header.view()->sensorMode == SensorMode::Simulation && header.view()->outputMode == OutputMode::Virtual);
  assert(header.view()->starts[0].rule == 10 && header.view()->starts[0].channel == 1);
  assert(header.view()->starts[1].kind == recovery::OutputKind::Servo && header.view()->starts[2].rule == 20);
  const auto initial = flash; const auto writes = flash.writes, reads = flash.reads;
  // A valid CRC/commit cannot enable an old or unknown header format.
  for (unsigned field : {4u, 20u}) for (unsigned version : {1u, 2u, 3u, 4u, 5u, 6u, 7u, 9u}) { // Header v8 is current.
    auto wrong = initial; auto *descriptor = wrong.bytes.data() + 0x30000;
    if (field==20 && version==3) continue;
    storage::binary::put(descriptor + field, version, 2);
    const auto payloadBytes = (layout.configBytes + 255) / 256 * 256;
    const auto checksum = storage::binary::extendCrc(storage::binary::extendCrc(storage::binary::CrcInitial, descriptor, 252), descriptor + 512, payloadBytes) ^ storage::binary::CrcInitial;
    storage::binary::put(descriptor + 252, checksum, 4);
    storage::binary::put(descriptor + 256 + 4, checksum, 4);
    storage::binary::put(descriptor + 256 + 8, ~checksum, 4);
    const auto before = wrong.bytes; FlightHeaderStore unsupported(wrong);
    assert(!unsupported.load(0x30000) && unsupported.error() == HeaderError::UnsupportedVersion && !unsupported.view());
    assert(wrong.bytes == before && wrong.writes == writes);
  }
  assert(!create(header) && header.error() == HeaderError::Active);
  assert(!header.load(0x30000) && header.error() == HeaderError::Active);
  assert(!header.recordStart(999, recovery::OutputKind::Pyro, 1, 0) && header.error() == HeaderError::UnknownOutput);
  assert(!header.recordStart(10, recovery::OutputKind::Pyro, 2, 0));
  assert(header.recordStart(10, recovery::OutputKind::Pyro, 1, 0));
  assert(header.recordStart(10, recovery::OutputKind::Pyro, 1, 0));
  assert(!header.recordStart(10, recovery::OutputKind::Pyro, 1, 1) && header.error() == HeaderError::Conflict);
  assert(header.view()->starts[0].timeMs == 0 && header.view()->starts[0].state == StartState::Pending);
  assert(flash.writes == writes && flash.reads == reads); // Capture never accesses flash.
  assert(!header.finish(false) && header.error() == HeaderError::ProducerActive);
  assert(header.flushStarts()); assert(header.view()->starts[0].state == StartState::Recorded);
  const auto afterFirst = flash;
  assert(header.recordStart(10, recovery::OutputKind::Pyro, 1, 0) && header.flushStarts());
  assert(flash.writes == writes + 2); // No reprogramming for a duplicate notification.
  assert(header.recordStart(10, recovery::OutputKind::Servo, 1, UINT32_MAX));
  assert(header.finish(true));
  assert(!header.recordStart(20, recovery::OutputKind::Pyro, 2, 123) && header.error() == HeaderError::Closed);
  FlightHeaderStore reader(flash); assert(reader.load(0x30000));
  assert(encoded(reader.view()->configuration) == saved);
  assert(reader.view()->starts[0].state == StartState::Recorded && reader.view()->starts[0].timeMs == 0);
  assert(reader.view()->starts[1].timeMs == UINT32_MAX && reader.view()->starts[2].state == StartState::Empty);
  assert(!reader.recordStart(20, recovery::OutputKind::Pyro, 2, 42) && reader.error() == HeaderError::Closed);
  // The immutable header survives a corrupt/torn output slot.
  const auto firstSlot = 0x30000 + layout.tableOffset;
  flash.bytes[firstSlot + 24] ^= 1;
  assert(reader.load(0x30000) && reader.view()->starts[0].state == StartState::Invalid);
  flash = afterFirst;
  std::copy(flash.bytes.begin()+0x30000, flash.bytes.begin()+0x31000, flash.bytes.begin()+0x32000);
  assert(!reader.load(0x17000)); // Relocated headers cannot authorize old slot data.
  flash.bytes[0x30000 + 512] ^= 1;
  assert(!reader.load(0x30000) && reader.error() == HeaderError::Corrupt && !reader.view());
  // Every byte boundary of immutable payload/descriptor/commit writes.
  const unsigned createBytes = initial.writes * 256;
  for (unsigned cut = 0; cut <= createBytes; ++cut) {
    HeaderFlash f; FlightHeaderStore h(f); f.budget = cut;
    try { create(h, frozen); } catch (const Cut &) {}
    f.budget = -1; FlightHeaderStore boot(f);
    if (boot.load(0x30000)) {
      assert(encoded(boot.view()->configuration) == saved && boot.view()->starts[0].state == StartState::Empty);
      assert(!boot.recordStart(10, recovery::OutputKind::Pyro, 1, 0));
    } else assert(!boot.view());
    assert(std::equal(pristine.begin(), pristine.begin()+0x30000, f.bytes.begin()));
  }
  // Every byte boundary of one output slot, preserving a previously recorded start.
  for (unsigned cut = 0; cut <= 512; ++cut) {
    HeaderFlash f; FlightHeaderStore h(f); assert(create(h));
    assert(h.recordStart(10, recovery::OutputKind::Pyro, 1, 0) && h.flushStarts());
    assert(h.recordStart(10, recovery::OutputKind::Servo, 1, 250)); f.budget = cut;
    try { h.flushStarts(); } catch (const Cut &) {}
    f.budget = -1; FlightHeaderStore boot(f); assert(boot.load(0x30000));
    assert(boot.view()->starts[0].state == StartState::Recorded && boot.view()->starts[0].timeMs == 0);
    const auto &s = boot.view()->starts[1];
    if (s.state == StartState::Recorded) assert(s.timeMs == 250);
    else assert(s.state == StartState::Empty || s.state == StartState::Invalid);
  }
  HeaderFlash uncertain; FlightHeaderStore u(uncertain); assert(create(u));
  uncertain.failAfterProgram = uncertain.writes + 2;
  assert(u.recordStart(10, recovery::OutputKind::Pyro, 1, 0));
  assert(!u.flushStarts()); const auto attempted = uncertain.writes;
  assert(!u.flushStarts() && uncertain.writes == attempted && u.error() == HeaderError::Persistence);
  assert(!u.finish(true));
  FlightHeaderStore recovered(uncertain); assert(recovered.load(0x30000));
  assert(recovered.view()->starts[0].state == StartState::Recorded);
  // Reserve exactly eight independent channels, with all notifications pending together.
  auto all = fixture(); all.rules[0].assignmentCount = 8; all.ruleCount = 1;
  for (unsigned i = 0; i < 8; ++i) {
    auto &a = all.rules[0].assignments[i]; a.kind = i < 4 ? recovery::OutputKind::Pyro : recovery::OutputKind::Servo;
    a.channel = uint8_t(i % 4 + 1); all.pyros[i%4].pulseMs = 100; all.servos[i%4] = all.servos[0];
  }
  HeaderFlash eight; FlightHeaderStore e(eight); assert(create(e, all));
  assert(e.view()->layout.slotCount == 8 && e.view()->layout.headerBytes == 8192);
  const auto beforeNotifications = eight.writes;
  for (unsigned i = 0; i < 8; ++i) {
    const auto &a = all.rules[0].assignments[i]; assert(e.recordStart(10, a.kind, a.channel, i));
  }
  assert(eight.writes == beforeNotifications); assert(e.finish(true));
  assert(eight.writes == beforeNotifications + 16);
  // A large valid plan spans multiple payload pages, with a slot for every assigned channel.
  auto large = fixture(); large.ruleCount = 8;
  for (unsigned i = 0; i < 4; ++i) { large.pyros[i].pulseMs = 100; large.servos[i] = large.servos[0]; }
  for (unsigned i = 0; i < 8; ++i) {
    auto &r = large.rules[i]; r = {}; r.id = i + 1; strcpy(r.name.data(), "Large snapshot");
    r.conditionCount = 8; r.assignmentCount = 1;
    r.assignments[0].kind = i < 4 ? recovery::OutputKind::Pyro : recovery::OutputKind::Servo;
    r.assignments[0].channel = uint8_t(i%4 + 1);
  }
  HeaderFlash big; FlightHeaderStore b(big); assert(create(b, large));
  assert(b.view()->layout.configBytes > 1024 && encoded(b.view()->configuration) == encoded(large));
  assert(b.finish(true)); assert(b.load(0x30000));
  const auto configEnd = 0x30000 + 512 + b.view()->layout.configBytes - 1;
  big.bytes[configEnd] ^= 1;
  assert(!b.load(0x30000) && b.error() == HeaderError::Corrupt);
  // Mode metadata is descriptive: physical mode never calls an actuator API.
  HeaderFlash real; FlightHeaderStore hw(real);
  assert(hw.create(0x30000, fixture(), limits, SensorMode::Hardware, OutputMode::Physical, sensors::FrameConvention::SensorAxes));
  assert(hw.view()->outputMode == OutputMode::Physical && hw.view()->sensorMode == SensorMode::Hardware);
  assert(hw.finish(true)); assert(hw.load(0x30000));
  assert(hw.view()->frame == sensors::FrameConvention::SensorAxes);
  HeaderFlash bad; FlightHeaderStore failed(bad); bad.failProgram = true;
  assert(!create(failed) && failed.error() == HeaderError::Io && !failed.view());
  // Ambiguous durable header commit must be read, never overwritten.
  HeaderFlash late; FlightHeaderStore ambiguous(late);
  late.failAfterProgram = initial.writes;
  assert(!create(ambiguous)); const auto headerWrites = late.writes;
  assert(!create(ambiguous) && ambiguous.error() == HeaderError::NotErased && late.writes == headerWrites);
  FlightHeaderStore lateBoot(late); assert(lateBoot.load(0x30000));
  late.failRead = true; FlightHeaderStore unreadable(late);
  assert(!unreadable.load(0x30000) && unreadable.error() == HeaderError::Io);
  // Samples can exhaust their extent while the already-reserved start table remains writable.
  HeaderFlash combined; FlightHeaderStore meta(combined); assert(create(meta));
  FlightSampleStore samples(combined);
  assert(samples.begin(7, 0x30000, meta.view()->layout.headerBytes, 4096, FlashLayout::BlockBytes));
  for (unsigned i = 0; i < 2; ++i) { FlightSample s; s.timeUs = i; s.imuSequence = i + 1; assert(samples.append(s) && samples.flush()); }
  FlightSample extra; extra.timeUs = 2; extra.imuSequence = 3; // Two 2 KiB blocks fill the 4 KiB extent.
  assert(!samples.append(extra) && samples.error() == SampleError::Full);
  assert(meta.recordStart(10, recovery::OutputKind::Pyro, 1, 500) && meta.finish(true));
  FlightHeaderStore final(combined); assert(final.load(0x30000));
  assert(final.view()->starts[0].timeMs == 500);
  std::cout << "Flight header tests passed: frozen configuration, modes, slot ownership, eight pending starts, stopped-full independence, "
            << createBytes + 1 << " header cuts and 513 output-start cuts.\n";
}
} // namespace test_flight_header_store

namespace test_flight_directory {
using namespace storage;
struct PowerLoss {};
class MemoryFlash : public FlashDevice {
public:
  std::vector<uint8_t> bytes = std::vector<uint8_t>(FlashLayout::RingBegin, 255);
  long budget = -1;
  std::size_t mutations = 0;
  void tick() { if (budget == 0) throw PowerLoss{}; if (budget > 0) --budget; ++mutations; }
  bool read(uint32_t a, void *p, std::size_t n) override {
    if (a > bytes.size() || n > bytes.size() - a) return false;
    std::memcpy(p, bytes.data() + a, n); return true;
  }
  bool program(uint32_t a, const void *p, std::size_t n) override {
    assert(a >= FlashLayout::DirectoryA && a + n <= FlashLayout::RingBegin && n == 256 && a % 256 == 0);
    const auto *source = static_cast<const uint8_t *>(p);
    for (std::size_t i = 0; i < n; ++i) {
      assert((bytes[a + i] & source[i]) == source[i]); tick(); bytes[a + i] &= source[i];
    }
    return true;
  }
  bool eraseSector(uint32_t a) override {
    assert(a >= FlashLayout::DirectoryA && a + 4096 <= FlashLayout::RingBegin && a % 4096 == 0);
    for (unsigned i = 0; i < 4096; ++i) { tick(); bytes[a + i] = 255; }
    return true;
  }
};
FlightEntry flight(uint64_t sequence = 1, uint32_t start = FlashLayout::RingBegin) {
  FlightEntry e; e.sequence = sequence; e.start = start; e.allocatedBytes = FlashLayout::BlockBytes; return e;
}
void appendCuts(const MemoryFlash &initial, const FlightEntry &e) {
  for (long cut = 0; cut <= 512; ++cut) {
    auto flash = initial; FlightDirectory d(flash); assert(d.mount()); flash.budget = cut;
    try { d.update(e); } catch (const PowerLoss &) {}
    flash.budget = -1; FlightDirectory boot(flash); assert(boot.mount());
    assert(boot.count() == 1);
    assert(boot.entries()[0].committedBytes == 0 || boot.entries()[0].committedBytes == e.committedBytes);
    // A torn slot is consumed, never programmed again.
    assert(boot.update(e));
  }
}
void run() {
  MemoryFlash flash; FlightDirectory directory(flash);
  const auto pristine = flash.bytes;
  assert(!directory.mount() && directory.error() == DirectoryError::Missing);
  assert(pristine == flash.bytes);
  assert(directory.format());
  auto e = flight(); assert(directory.update(e));
  const auto baseline = flash;
  {
    auto padFlash=baseline; FlightDirectory pad(padFlash); assert(pad.mount());
    auto discarded=e; discarded.status=FlightStatus::Invalidated; discarded.allocatedBytes+=FlashLayout::BlockBytes;
    assert(!pad.update(discarded) && padFlash.bytes==baseline.bytes);
    discarded=e; discarded.status=FlightStatus::Invalidated; ++discarded.committedBytes;
    assert(!pad.update(discarded) && padFlash.bytes==baseline.bytes);
    discarded=e; discarded.status=FlightStatus::Invalidated;
    assert(pad.update(discarded));
    FlightDirectory remount(padFlash); assert(remount.mount() && remount.retainedCount()==0 && remount.highWater()==1);
    assert(!remount.update(e)); // Discarded identities cannot be revived.
  }
  e.committedBytes = 100; appendCuts(baseline, e);
  assert(directory.update(e));
  e.status = FlightStatus::Completed; assert(directory.update(e));
  auto next = flight(2, FlashLayout::blockAddress(1)); assert(directory.update(next));
  auto overlap = flight(3, FlashLayout::RingBegin); assert(!directory.update(overlap));
  next.status = FlightStatus::Interrupted; assert(directory.update(next));
  e.status = FlightStatus::Invalidated; assert(directory.update(e));
  const auto beforeCompact = flash;
  // Every byte boundary of each programmed page, plus representative partial erases.
  std::vector<long> cuts;
  for (long cut = 65536; cut <= 65536 + 1536; ++cut) cuts.push_back(cut);
  for (long sector = 0; sector < 16; ++sector)
    for (long offset : {0L, 1L, 2048L, 4095L}) cuts.push_back(sector * 4096 + offset);
  for (long cut : cuts) {
    auto f = beforeCompact; FlightDirectory d(f); assert(d.mount()); f.budget = cut;
    try { d.compact(); } catch (const PowerLoss &) {}
    f.budget = -1; FlightDirectory boot(f); assert(boot.mount());
    assert((boot.count() == 1 || boot.count() == 2) && boot.highWater() == 2);
    if (boot.count()==2) assert(boot.entries()[0].status==FlightStatus::Invalidated);
    assert(boot.entries()[boot.count()-1].status == FlightStatus::Interrupted);
  }
  assert(directory.compact());
  assert(directory.generation() == 2);
  assert(directory.compact()); assert(directory.generation() == 3);
  // Repeated checkpoints exercise automatic compaction/reserved slots.
  MemoryFlash checkpoints; FlightDirectory log(checkpoints); assert(log.format());
  e = flight(); assert(log.update(e));
  for (unsigned i = 1; i <= 300; ++i) { e.committedBytes = i; assert(log.update(e)); }
  FlightDirectory reboot(checkpoints); assert(reboot.mount());
  assert(reboot.entries()[0].committedBytes == 300 && reboot.generation() > 1);
  // Deliberately corrupt a committed record: fail closed, never silently roll back.
  MemoryFlash corrupted = baseline; corrupted.bytes[FlashLayout::DirectoryA + 512 + 32] ^= 1;
  FlightDirectory bad(corrupted); assert(!bad.mount() && bad.error() == DirectoryError::Corrupt);
  // Ring extents wrap: a flight in the last block continues in block 0 and owns it.
  MemoryFlash boundary; FlightDirectory ring(boundary); assert(ring.format());
  e=flight(1,FlashLayout::blockAddress(FlashLayout::RingBlocks-1)); e.allocatedBytes=2*FlashLayout::BlockBytes;
  assert(ring.update(e)); e.status=FlightStatus::Completed; assert(ring.update(e));
  const auto ringBefore=boundary.bytes;
  assert(!ring.update(flight(2,FlashLayout::RingBegin)) && ring.error()==DirectoryError::InvalidEntry && boundary.bytes==ringBefore);
  next=flight(2,FlashLayout::blockAddress(1)); assert(ring.update(next));
  FlightDirectory ringBoot(boundary); assert(ringBoot.mount() && ringBoot.count()==2);
  // An extent longer than the ring is rejected at mount without changing bytes.
  auto oversized=boundary;
  auto *extent=oversized.bytes.data()+FlashLayout::DirectoryA+FlightDirectory::SlotBytes;
  binary::put(extent+34,FlashLayout::RingBlocks+1,2);
  const auto extentCrc=binary::crc(extent,252);
  binary::put(extent+252,extentCrc,4); binary::put(extent+256+4,extentCrc,4);
  binary::put(extent+256+8,~extentCrc,4);
  const auto extentBefore=oversized.bytes;
  FlightDirectory rejectedExtent(oversized);
  assert(!rejectedExtent.mount() && rejectedExtent.error()==DirectoryError::Corrupt && oversized.bytes==extentBefore);
  // Unknown/nonblank directory isn't autoformatted.
  MemoryFlash unknown; unknown.bytes[FlashLayout::DirectoryA] = 0; const auto saved = unknown.bytes;
  FlightDirectory unrecognized(unknown); assert(!unrecognized.mount()); assert(unknown.bytes == saved);
  MemoryFlash capacity; FlightDirectory full(capacity); assert(full.format());
  for (uint64_t i = 1; i <= FlightDirectory::MaxEntries; ++i) {
    auto item = flight(i, FlashLayout::blockAddress(uint32_t(i - 1)));
    assert(full.update(item)); item.status = FlightStatus::Completed; assert(full.update(item));
  }
  const auto capacityBefore = capacity.bytes;
  assert(!full.update(flight(FlightDirectory::MaxEntries + 1, FlashLayout::blockAddress(100))) && full.error() == DirectoryError::Full);
  assert(capacityBefore == capacity.bytes);
  assert(full.compact()); FlightDirectory fullBoot(capacity); assert(fullBoot.mount());
  assert(fullBoot.count() == FlightDirectory::MaxEntries);
  // A valid CRC/commit cannot authorize metadata from an incompatible geometry.
  const auto bank = fullBoot.generation() % 2 ? FlashLayout::DirectoryA : FlashLayout::DirectoryB;
  const auto other = bank == FlashLayout::DirectoryA ? FlashLayout::DirectoryB : FlashLayout::DirectoryA;
  auto *body = capacity.bytes.data() + bank;
  body[52] ^= 0x10; // Persisted data-region start differs from this board's fixed layout.
  uint32_t checksum = 0xFFFFFFFFu;
  for (unsigned i = 0; i < 252; ++i) {
    checksum ^= body[i];
    for (unsigned bit = 0; bit < 8; ++bit) checksum = (checksum >> 1) ^ ((checksum & 1) ? 0xEDB88320u : 0);
  }
  checksum ^= 0xFFFFFFFFu;
  for (unsigned i = 0; i < 4; ++i) {
    body[252+i] = uint8_t(checksum >> (8*i));
    body[256+4+i] = uint8_t(checksum >> (8*i));
    body[256+8+i] = uint8_t((~checksum) >> (8*i));
  }
  // Destroy the alternative older bank so mount cannot legitimately use it.
  std::fill(capacity.bytes.begin()+other, capacity.bytes.begin()+other+FlashLayout::DirectoryBankBytes, 255);
  FlightDirectory incompatible(capacity); assert(!incompatible.mount());
  std::cout << "Flight directory tests passed: " << 513 + cuts.size()
            << " simulated power cuts, compaction, bounds, corruption and ring ownership.\n";
}
} // namespace test_flight_directory

int main() {
  std::puts("  configuration_store"); std::fflush(stdout);
  test_configuration_store::run();
  std::puts("  flight_header_store"); std::fflush(stdout);
  test_flight_header_store::run();
  std::puts("  flight_directory"); std::fflush(stdout);
  test_flight_directory::run();
}
