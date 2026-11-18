#include <cstdio>
#include "storage/FlightRecorder.h"
#include "storage/ProfiledFlashDevice.h"
#include "../fixtures/ReplayFixture.h"
#include <algorithm>
#include <iostream>
#include "storage/BinaryEncoding.h"

// Each namespace isolates one component scenario and its fault model.
namespace test_storage_lifecycle {
using namespace storage;
using namespace replay_test;
struct Cut {};
struct Device : FlashDevice {
  std::vector<uint8_t> bytes=std::vector<uint8_t>(0x50000,255);
  long budget=-1; unsigned erases=0; bool failProgram=false, corruptAfterProgram=false, corruptReads=false;
  FlashError typedCause=FlashError::Verify;
  bool read(uint32_t a,void *p,std::size_t n) override {
    assert(a<=bytes.size() && n<=bytes.size()-a); memcpy(p,bytes.data()+a,n);
    if (corruptReads && a>=FlashLayout::DataBegin && n) static_cast<uint8_t *>(p)[0]^=1;
    return true;
  }
  bool program(uint32_t a,const void *p,std::size_t n) override {
    if (failProgram) return false;
    assert(a<=bytes.size() && n<=bytes.size()-a); const auto *in=static_cast<const uint8_t *>(p);
    for (std::size_t i=0;i<n;++i) { if (!budget) throw Cut{}; if (budget>0) --budget; assert((bytes[a+i]&in[i])==in[i]); bytes[a+i]&=in[i]; }
    if (corruptAfterProgram && a>=FlashLayout::DataBegin) corruptReads=true;
    return true;
  }
  bool eraseSector(uint32_t a) override {
    assert(a%4096==0 && a>=FlashLayout::DirectoryA && a+4096<=bytes.size());
    if (a>=FlashLayout::DataBegin) {
      FlightDirectory proof(*this); assert(proof.mount()); bool owned=false;
      for (std::size_t i=0;i<proof.count();++i) {
        const auto &e=proof.entries()[i];
        if (e.status==FlightStatus::Recording && a>=e.start && a-e.start<e.allocatedBytes) owned=true;
      }
      assert(owned); // Physical erasure requires the new durable reservation.
    }
    ++erases; memset(bytes.data()+a,255,4096); return true;
  }
  FlashFailure lastFailure() const override { FlashFailure f; if (failProgram) f.cause=typedCause; return f; }
};
bool begin(FlightRecorder &r) { return r.begin(fixture(),limits,4096,SensorMode::Simulation,OutputMode::Virtual,sensors::FrameConvention::LoggedCalibratedBody); }
void close(FlightRecorder &r,unsigned time=0) { FlightSample s; s.timeUs=uint64_t(time)*1000; assert(r.append(s) && r.finish(true)); }
void run() {
  Device flash; FlightRecorder recorder(flash); assert(recorder.initializeBlank());
  assert(begin(recorder)); const auto first=*recorder.current(); close(recorder);
  const auto preserved=std::vector<uint8_t>(flash.bytes.begin()+first.start,flash.bytes.begin()+first.start+first.allocatedBytes);
  assert(begin(recorder)); const auto second=*recorder.current(); close(recorder,5);
  const auto seed=flash;
  // An orderly unflown close discards only after its directory commit. A reset
  // before that commit retains the uncertain interrupted recording instead.
  for (unsigned cut=0;cut<=512;++cut) {
    auto f=seed; FlightRecorder r(f); assert(r.initialize() && begin(r));
    const auto pad=*r.current(); FlightSample unflushed; assert(r.append(unflushed));
    const auto dataBefore=std::vector<uint8_t>(f.bytes.begin()+FlashLayout::DataBegin,f.bytes.end());
    const auto eraseCount=f.erases; f.budget=cut;
    try { r.finish(true,RecordingSummary{},RecordingRetention::Discard); } catch (const Cut &) {}
    f.budget=-1; FlightRecorder boot(f); assert(boot.initialize());
    assert(f.erases==eraseCount && std::equal(dataBefore.begin(),dataBefore.end(),f.bytes.begin()+FlashLayout::DataBegin));
    RecordingInfo kept; assert(boot.inspect(1,kept) && kept.samples==1);
    assert(boot.inspect(2,kept) && kept.samples==1);
    bool retained=false;
    for (std::size_t i=0;i<boot.directory().count();++i) if (boot.directory().entries()[i].sequence==pad.sequence) {
      const auto status=boot.directory().entries()[i].status;
      assert(status==FlightStatus::Interrupted || status==FlightStatus::Invalidated);
      retained=status==FlightStatus::Interrupted;
    }
    assert(boot.directory().retainedCount()==(retained ? 3u : 2u));
    assert(begin(boot) && boot.current()->sequence==pad.sequence+1);
    assert(boot.current()->start==(retained ? pad.start+pad.allocatedBytes : pad.start));
    boot.abort();
  }
  for (unsigned cut=0;cut<=512;++cut) {
    auto f=seed; FlightRecorder r(f); assert(r.initialize()); f.budget=cut;
    try { r.remove(2); } catch (const Cut &) {}
    f.budget=-1; FlightRecorder boot(f); assert(boot.initialize());
    RecordingInfo info; assert(boot.inspect(1,info) && info.samples==1);
    assert(std::equal(preserved.begin(),preserved.end(),f.bytes.begin()+first.start));
    bool retained=false;
    for (std::size_t i=0;i<boot.directory().count();++i) if (boot.directory().entries()[i].sequence==2) retained=boot.directory().entries()[i].status!=FlightStatus::Invalidated;
    assert(begin(boot));
    assert(boot.current()->sequence==3);
    assert(boot.current()->start==(retained ? second.start+second.allocatedBytes : second.start));
    close(boot,10);
  }
  const auto erases=flash.erases; assert(recorder.remove(2)); assert(flash.erases==erases);
  assert(recorder.remove(2)); // Idempotent while tombstone remains.
  RecordingInfo info; assert(!recorder.inspect(2,info) && recorder.error()==RecorderError::StaleSelection);
  for (unsigned i=3;i<=160;++i) {
    assert(begin(recorder) && recorder.current()->sequence==i && recorder.current()->start==second.start);
    const auto activeBytes=flash.bytes; assert(!recorder.remove(i) && recorder.error()==RecorderError::Busy && flash.bytes==activeBytes);
    close(recorder,i); assert(recorder.remove(i));
    assert(recorder.directory().retainedCount()==1);
    if (i%7==0) assert(recorder.initialize());
  }
  assert(recorder.directory().highWater()==160);
  assert(std::equal(preserved.begin(),preserved.end(),flash.bytes.begin()+first.start));
  assert(recorder.remove(1));
  FlightDirectory directory(flash); assert(directory.mount() && directory.compact());
  FlightDirectory reboot(flash); assert(reboot.mount() && reboot.count()==0 && reboot.highWater()==160);
  const auto compacted=flash.bytes; FlightEntry old=first; old.status=FlightStatus::Recording;
  assert(!reboot.update(old) && flash.bytes==compacted); // Deleted identities cannot resurrect.
  assert(recorder.initialize() && begin(recorder) && recorder.current()->sequence==161 && recorder.current()->start==first.start);
  RecordingSummary summary; summary.droppedSamples=7; summary.queuePeak=9; summary.queueCapacity=64; summary.reason=RecordingEnd::Stopped;
  FlightSample sample; assert(recorder.append(sample) && recorder.finish(true,summary));
  assert(recorder.initialize() && recorder.inspect(161,info));
  assert(info.summaryState==SummaryState::Valid && info.summary.savedSamples==1 && info.summary.droppedSamples==7 && info.summary.queuePeak==9 && info.summary.reason==RecordingEnd::Stopped);
  auto layout=recorder.metadata()->layout; flash.bytes[first.start+layout.summaryOffset+28]^=1;
  assert(recorder.inspect(161,info) && info.samples==1 && info.summaryState==SummaryState::Corrupt);
  flash.bytes[first.start+layout.headerBytes+36]^=1;
  assert(!recorder.inspect(161,info) && recorder.error()==RecorderError::Corrupt);
  assert(std::string(recorder.failure().component)=="Sample" && std::string(recorder.failure().cause)=="Corrupt");
  // Preserve the first typed storage fault, including the actual operation/address.
  Device faulty; ProfiledFlashDevice profiled(faulty); FlightRecorder failure(profiled);
  assert(failure.initializeBlank() && begin(failure));
  // Airborne samples fill the RAM block; the append that overflows it writes the block.
  sample.phase=recovery::FlightPhase::Coast; faulty.failProgram=true; bool failed=false;
  for (unsigned i=0;i<200 && !failed;++i) { sample.timeUs=i*1000; sample.imuSequence=i+1; failed=!failure.append(sample); }
  assert(failed && !failure.service()); const auto detail=failure.failure();
  assert(detail.error==RecorderError::Sample && std::string(detail.cause)=="Io" && detail.flash.cause==FlashError::Verify && detail.flash.operation==FlashOperation::Program);
  faulty.failProgram=false; uint8_t byte; assert(profiled.read(0,&byte,1));
  assert(!failure.inspect(1,info)); failure.abort();
  assert(failure.failure().flash.address==detail.flash.address && failure.failure().flash.cause==FlashError::Verify);
  assert(failure.initialize() && failure.failure().error==RecorderError::None);
  faulty.failProgram=true; faulty.typedCause=FlashError::Timeout;
  assert(!begin(failure) && failure.failure().flash.cause==FlashError::Timeout);
  Device verify; FlightRecorder verifying(verify); assert(verifying.initializeBlank() && begin(verifying));
  verify.corruptAfterProgram=true; failed=false;
  for (unsigned i=0;i<200 && !failed;++i) { sample.timeUs=i*1000; sample.imuSequence=i+1; failed=!verifying.append(sample); }
  assert(failed && !verifying.service());
  assert(verifying.failure().error==RecorderError::Sample && std::string(verifying.failure().cause)=="Verify");
  std::cout<<"Storage lifecycle passed: 513 delete cuts, 513 discard cuts, protected flights, 160 identities with reuse, durable high-water, summaries and first-fault diagnostics.\n";
}
} // namespace test_storage_lifecycle

namespace test_flight_corruption {
using namespace storage;
struct Cut {};
struct Device : FlashDevice {
  std::vector<uint8_t> bytes=std::vector<uint8_t>(0x50000,255);
  long budget=-1;
  unsigned programs=0, erases=0;
  bool failProgram=false;
  uint32_t failedRead=UINT32_MAX;
  bool read(uint32_t a,void *p,std::size_t n) override {
    if (a<=failedRead && failedRead-a<n) return false;
    assert(a<=bytes.size() && n<=bytes.size()-a); memcpy(p,bytes.data()+a,n); return true;
  }
  bool program(uint32_t a,const void *p,std::size_t n) override {
    ++programs; if (failProgram) return false;
    assert(a<=bytes.size() && n<=bytes.size()-a); const auto *s=static_cast<const uint8_t *>(p);
    for(std::size_t i=0;i<n;++i) {
      if (!budget) throw Cut{};
      if (budget>0) --budget;
      assert((bytes[a+i]&s[i])==s[i]); bytes[a+i]&=s[i];
    }
    return true;
  }
  bool eraseSector(uint32_t a) override {
    ++erases; assert(a%4096==0 && a+4096<=bytes.size()); memset(bytes.data()+a,255,4096); return true;
  }
};
bool begin(FlightRecorder &r) {
  return r.begin(replay_test::fixture(),replay_test::limits,4096,SensorMode::Simulation,OutputMode::Virtual,sensors::FrameConvention::LoggedCalibratedBody);
}
std::vector<uint8_t> data(const Device &d) {
  return std::vector<uint8_t>(d.bytes.begin()+FlashLayout::DataBegin,d.bytes.end());
}
void run() {
  Device flash; FlightRecorder recorder(flash); assert(recorder.initializeBlank() && begin(recorder));
  FlightSample sample; assert(recorder.append(sample) && recorder.finish(true));
  const auto healthy=*recorder.current();
  assert(begin(recorder)); const auto interrupted=*recorder.current(); const auto layout=recorder.metadata()->layout;
  // One committed block (114 airborne 15-byte FRAMEs) plus an unwritten RAM tail.
  sample.phase=recovery::FlightPhase::Coast;
  for(unsigned i=0;i<120;++i) { sample.timeUs=i*1000; sample.imuSequence=i+1; assert(recorder.append(sample)); }
  assert(recorder.service() && recorder.committedSamples()==114); recorder.abort(); const auto seed=flash;
  const auto sampleAddress=interrupted.start+layout.headerBytes;
  flash.bytes[sampleAddress+36]^=1; const auto damaged=flash;
  // Power loss during quarantine can leave Recording or Corrupt, never free space.
  for(unsigned cut=0;cut<=512;++cut) {
    auto f=damaged; const auto retained=data(f); const auto erases=f.erases;
    FlightRecorder r(f); f.budget=cut;
    try { r.initialize(); } catch(const Cut &) {}
    f.budget=-1; FlightRecorder boot(f); assert(boot.initialize() && boot.directory().corruptCount()==1);
    assert(boot.directory().entries()[1].status==FlightStatus::Corrupt);
    assert(boot.directory().entries()[1].committedBytes==interrupted.committedBytes);
    RecordingInfo info; assert(boot.inspect(healthy.sequence,info) && info.samples==1);
    assert(!boot.inspect(interrupted.sequence,info) && boot.error()==RecorderError::Corrupt);
    assert(f.erases==erases && data(f)==retained);
    const auto count=f.programs; assert(boot.initialize() && f.programs==count); // No repeated flag writes.
    assert(begin(boot) && boot.current()->sequence==3 &&
           boot.current()->start==interrupted.start+interrupted.allocatedBytes);
  }
  FlightRecorder boot(flash); assert(boot.initialize() && boot.directory().corruptCount()==1);
  const auto retained=data(flash);
  FlightDirectory directory(flash); assert(directory.mount() && directory.compact());
  assert(boot.initialize() && boot.directory().corruptCount()==1 && data(flash)==retained);
  const auto flaggedBytes=flash.bytes;
  for(auto status : {FlightStatus::Recording,FlightStatus::Completed,FlightStatus::Interrupted,FlightStatus::StoppedFull}) {
    auto restored=directory.entries()[1]; restored.status=status;
    assert(!directory.update(restored) && flash.bytes==flaggedBytes);
  }
  auto resized=directory.entries()[1]; resized.allocatedBytes+=4096;
  assert(!directory.update(resized) && flash.bytes==flaggedBytes);
  // Corrupt flags survive compaction; only explicit deletion releases the extent.
  assert(boot.remove(interrupted.sequence) && boot.directory().corruptCount()==0);
  assert(begin(boot) && boot.current()->sequence==3 && boot.current()->start==interrupted.start);

  // An unreadable immutable header also quarantines the interrupted flight.
  auto header=seed; header.bytes[interrupted.start+512]^=1;
  const auto headerData=data(header); FlightRecorder headerBoot(header);
  assert(headerBoot.initialize() && headerBoot.directory().corruptCount()==1 && data(header)==headerData);

  // Inspecting or directly reading damaged closed recordings persists the same flag.
  for(bool direct : {false,true}) {
    auto closed=seed; closed.bytes[healthy.start+4096+36]^=1;
    FlightRecorder reader(closed); assert(reader.initialize());
    RecordingInfo info; RawBlock block;
    assert(!(direct ? reader.readBlock(healthy.sequence,0,block) : reader.inspect(healthy.sequence,info)));
    assert(reader.error()==RecorderError::Corrupt && reader.directory().corruptCount()==1);
    assert(std::string(reader.failure().component)=="Sample");
    FlightRecorder remount(closed); assert(remount.initialize() && remount.directory().corruptCount()==1);
  }

  // A normal incomplete unpublished tail remains Interrupted, not Corrupt.
  auto tail=seed; tail.bytes[sampleAddress+2048]=0;
  FlightRecorder partial(tail); RecordingInfo info;
  assert(partial.initialize() && partial.directory().corruptCount()==0);
  assert(partial.inspect(interrupted.sequence,info) && info.samples==114 && info.entry.status==FlightStatus::Interrupted);

  // Transient read failures must not turn into permanent corruption flags.
  auto io=seed; io.failedRead=sampleAddress;
  const auto beforeIo=io.bytes; const auto ioPrograms=io.programs;
  FlightRecorder unreadable(io); assert(!unreadable.initialize() && unreadable.error()==RecorderError::Io);
  assert(io.bytes==beforeIo && io.programs==ioPrograms);
  io.failedRead=UINT32_MAX; FlightRecorder responsive(io);
  assert(responsive.initialize() && responsive.directory().corruptCount()==0);

  // A valid unsupported header version is not corruption and causes no writes.
  auto unsupported=seed; auto *p=unsupported.bytes.data()+interrupted.start; p[4]=9; // A future header version.
  const auto configPages=(layout.configBytes+255)/256*256;
  const auto crc=binary::extendCrc(binary::extendCrc(binary::CrcInitial,p,252),p+512,configPages)^binary::CrcInitial;
  binary::put(p+252,crc,4); binary::put(p+256+4,crc,4); binary::put(p+256+8,~crc,4);
  const auto oldBytes=unsupported.bytes; FlightRecorder future(unsupported);
  assert(!future.initialize() && std::string(future.failure().cause)=="UnsupportedVersion" && unsupported.bytes==oldBytes);

  // Failure to persist the flag remains a fatal directory error; do not report readiness.
  auto failed=damaged; failed.failProgram=true; FlightRecorder writerFailure(failed);
  assert(!writerFailure.initialize() && writerFailure.error()==RecorderError::Directory && !writerFailure.initialized());
  failed.failProgram=false; FlightRecorder retry(failed);
  assert(retry.initialize() && retry.directory().corruptCount()==1);

  // Corruption of the directory itself never authorizes any metadata repair.
  auto badDirectory=seed; badDirectory.bytes[FlashLayout::DirectoryA+512+32]^=1;
  const auto directoryBytes=badDirectory.bytes; FlightRecorder unknown(badDirectory);
  assert(!unknown.initialize() && unknown.error()==RecorderError::Directory && badDirectory.bytes==directoryBytes);
  std::cout<<"Flight corruption passed: 513 quarantine cuts, healthy exports, retained space, compaction, explicit deletion, header/sample flags, incomplete tails, I/O and unsupported-version separation.\n";
}
} // namespace test_flight_corruption

int main() {
  std::puts("  storage_lifecycle"); std::fflush(stdout);
  test_storage_lifecycle::run();
  std::puts("  flight_corruption"); std::fflush(stdout);
  test_flight_corruption::run();
}
