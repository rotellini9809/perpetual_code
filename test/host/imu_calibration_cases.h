#pragma once
#include "flight/ImuCalibration.h"
#include <limits>
#include <cstring>

namespace test_imu_calibration {
using namespace test_arming_controller;
constexpr float G=9.80665f;
// The device journal (block 0) only: calibration, identity revocation and settings records.
struct CalibrationFlash : storage::FlashDevice {
  std::vector<uint8_t> bytes=std::vector<uint8_t>(storage::FlashLayout::DirectoryA,255); unsigned writes=0; bool tear=false,failRead=false;
  bool read(uint32_t a,void *p,std::size_t n) override {
    if (failRead || a+n>bytes.size()) return false;
    std::memcpy(p,bytes.data()+a,n); return true;
  }
  bool program(uint32_t a,const void *p,std::size_t n) override {
    assert(a+n<=bytes.size()); ++writes;
    const auto *b=static_cast<const uint8_t *>(p);
    for (unsigned i=0;i<(tear?std::min<std::size_t>(17,n):n);++i) bytes[a+i]&=b[i];
    return !tear;
  }
  bool eraseSector(uint32_t a) override { assert(a%4096==0 && a<bytes.size()); ++writes; std::memset(bytes.data()+a,255,4096); return true; }
};
struct Source : sensors::SensorSource {
  sensors::Reading reading; bool pending=false;
  sensors::FrameConvention convention() const override { return sensors::FrameConvention::SensorAxes; }
  bool poll(sensors::Reading &r) override { if (!pending) return false; pending=false; r=reading; return true; }
};
void feed(flight::ImuCalibration &c,unsigned face,const storage::ImuCalibrationRecord &truth,unsigned count=260) {
  static uint64_t sequence=0;
  for (unsigned n=0;n<count;++n) {
    sensors::Reading r; r.imuValid=true; r.imuSequence=++sequence; r.timeUs=10000u*sequence;
    for (unsigned i=0;i<3;++i) r.accelMps2[i]=truth.bias[i]+(i==face/2 ? (face%2 ? -G:G)/truth.scale[i]:0);
    c.add(r);
  }
}
// The face's axis tilted by `degrees` towards the next sensor axis, the one after it, or both.
enum class Lean { Next, Other, Diagonal };
void feedTilted(flight::ImuCalibration &c,unsigned face,const storage::ImuCalibrationRecord &truth,float degrees,Lean lean=Lean::Next) {
  static uint64_t sequence=1000000;
  const unsigned axis=face/2, next=(axis+1)%3, other=(axis+2)%3; const float angle=degrees*0.017453292f;
  float up[3]={0,0,0}; up[axis]=(face%2 ? -G:G)*std::cos(angle);
  if (lean==Lean::Diagonal) up[next]=up[other]=G*std::sin(angle)*0.70710678f;
  else up[lean==Lean::Next ? next:other]=G*std::sin(angle);
  for (unsigned n=0;n<260;++n) {
    sensors::Reading r; r.imuValid=true; r.imuSequence=++sequence; r.timeUs=10000u*sequence;
    for (unsigned i=0;i<3;++i) r.accelMps2[i]=truth.bias[i]+up[i]/truth.scale[i];
    c.add(r);
  }
}
void testMisalignedFaces(const storage::ImuCalibrationRecord &truth) {
  // Calibration is strict: faces about 2 degrees off are accepted, 4 degrees off are rejected
  // (sin 4 deg x g = 0.68 m/s^2 on a transverse axis, above 0.4).
  flight::ImuCalibration c; c.begin(false,{});
  for (unsigned f=0;f<6;++f) { assert(c.capture(f)); feedTilted(c,f,truth,4,f%2 ? Lean::Other:Lean::Next); assert(c.mask()&(1u<<f)); }
  storage::ImuCalibrationRecord fitted; fitted.revision=1; assert(!c.result(fitted));
  c.begin(false,{});
  for (unsigned f=0;f<6;++f) { assert(c.capture(f)); feedTilted(c,f,truth,2,f%2 ? Lean::Other:Lean::Next); assert(c.mask()&(1u<<f)); }
  assert(c.result(fitted));
  for (unsigned i=0;i<3;++i) { assert(std::fabs(fitted.bias[i]-truth.bias[i])<.01f); assert(std::fabs(fitted.scale[i]/truth.scale[i]-1)<.001f); }
  // Verification only judges |a|: faces up to 10 degrees off in any direction are accepted.
  c.begin(true,fitted);
  assert(c.capture(0)); feedTilted(c,0,truth,9.5f,Lean::Diagonal); assert(!c.failed() && !c.collecting());
  assert(c.capture(2)); feedTilted(c,2,truth,11); // Beyond 10 deg: the capture stops, the face is captured again.
  assert(!c.collecting() && !c.failed() && !c.samples() && !std::strcmp(c.error(),"POSITION_OR_MOTION") && !(c.mask()&4));
  assert(c.capture(2) && !std::strcmp(c.error(),"None")); feedTilted(c,2,truth,9.5f,Lean::Other); assert(!c.collecting() && !c.failed());
  assert(c.capture(4)); feedTilted(c,4,truth,9.5f); assert(!c.failed());
  storage::ImuCalibrationRecord verified; assert(c.complete() && c.result(verified) && verified.approval==storage::ImuApproval::Verified);
  // Wrong coefficients fail on the face whose axis points up, whatever the alignment.
  auto wrongScale=fitted; wrongScale.scale[0]*=1.05f;
  c.begin(true,wrongScale); assert(c.capture(0)); feedTilted(c,0,truth,3); assert(c.failed());
  auto wrongBias=fitted; wrongBias.bias[2]+=.5f;
  c.begin(true,wrongBias); assert(c.capture(0)); feedTilted(c,0,truth,3); assert(!c.failed());
  assert(c.capture(2)); feedTilted(c,2,truth,3); assert(!c.failed() && !c.complete()); // Z is transverse: unseen.
  assert(c.capture(4)); feedTilted(c,4,truth,3); assert(c.failed() && !c.complete());
  // A face read with a different bias than its pair (the board disturbed between captures) gives
  // coefficients that verification rejects: Y biased +0.5 m/s^2 only on +Y.
  c.begin(false,{});
  for (unsigned f=0;f<6;++f) { assert(c.capture(f)); auto t=truth; if (f==2) t.bias[1]+=.5f; feedTilted(c,f,t,0); }
  // (Its transverse residuals stay below 0.4 m/s^2, so the calibration itself is accepted.)
  storage::ImuCalibrationRecord disturbed; disturbed.revision=1; assert(c.result(disturbed));
  // Half of it lands in the Y bias and half in the Y scale; on +Y both shift |a| the same way.
  c.begin(true,disturbed); assert(c.capture(0)); feedTilted(c,0,truth,4); assert(!c.failed());
  assert(c.capture(2)); feedTilted(c,2,truth,4); assert(c.failed() && !c.complete());
}
void run() {
  storage::ImuCalibrationRecord truth; truth.revision=1;
  truth.bias[0]=.25f; truth.bias[1]=-.18f; truth.bias[2]=.09f;
  truth.scale[0]=1.03f; truth.scale[1]=.97f; truth.scale[2]=1.01f;
  flight::ImuCalibration collector; collector.begin(false,{});
  assert(!collector.capture(6));
  for (unsigned f=0;f<6;++f) { assert(collector.capture(f)); feed(collector,f,truth); assert(!collector.collecting()); }
  storage::ImuCalibrationRecord measured;
  assert(collector.result(measured) && measured.revision==1 && measured.approval==storage::ImuApproval::Required);
  for (unsigned i=0;i<3;++i) { assert(std::fabs(measured.bias[i]-truth.bias[i])<.00001f); assert(std::fabs(measured.scale[i]-truth.scale[i])<.00001f); }
  collector.begin(true,measured); assert(collector.capture(0));
  feed(collector,2,truth); assert(!collector.samples() && !collector.collecting() && !collector.failed()); // Wrong face: stopped.
  assert(collector.capture(0)); feed(collector,0,truth); assert(!collector.complete());
  assert(collector.capture(2)); feed(collector,2,truth); assert(!collector.complete()); // +Z still missing.
  assert(collector.capture(4)); feed(collector,4,truth);
  assert(collector.complete() && collector.result(measured) && measured.approval==storage::ImuApproval::Verified);
  testMisalignedFaces(truth);
  collector.begin(false,{}); assert(collector.capture(0));
  feed(collector,0,truth,20); assert(collector.samples()==20);
  feed(collector,2,truth,10); // Within the settling time a wrong sample only restarts the window.
  assert(collector.collecting() && !collector.samples() && !std::strcmp(collector.error(),"None"));
  feed(collector,0,truth,20); assert(collector.samples()==20);
  sensors::Reading bad; bad.imuValid=true; bad.imuSequence=100000; bad.timeUs=1000000000;
  bad.accelMps2[0]=std::numeric_limits<float>::quiet_NaN(); collector.add(bad);
  assert(!collector.samples() && !collector.collecting() && !std::strcmp(collector.error(),"POSITION_OR_MOTION"));
  // A noisy window stops too; the face is then captured again.
  assert(collector.capture(0));
  for (unsigned n=0;n<260;++n) {
    sensors::Reading r; r.imuValid=true; r.imuSequence=200000+n; r.timeUs=2000000000ull+10000u*n;
    for (unsigned i=0;i<3;++i) r.accelMps2[i]=truth.bias[i]+(i==0 ? G/truth.scale[i]:0)+(i==1 ? (n%2 ? .3f:-.3f):0);
    collector.add(r);
  }
  assert(!collector.collecting() && !std::strcmp(collector.error(),"NOISY") && !(collector.mask()&1));
  collector.cancel(); assert(!collector.active());

  CalibrationFlash flash; storage::ImuCalibrationStore store(flash); assert(!store.load());
  assert(store.save(measured) && store.verified());
  for (unsigned i=0;i<3;++i) assert(std::isnan(store.record().residual[i])); // Not verified yet.
  { // Verification residuals travel in the calibration record (type 1) with its approval, and a reload reads them.
    const float residual[3]={.01f,-.02f,NAN}; assert(store.approve(storage::ImuApproval::Verified,residual));
    storage::ImuCalibrationStore reloaded(flash); reloaded.load(); const auto &r=reloaded.record();
    assert(reloaded.verified() && r.residual[0]==.01f && r.residual[1]==-.02f && std::isnan(r.residual[2]));
    assert(store.approve(storage::ImuApproval::InUse) && store.record().residual[1]==-.02f); // Other approval changes keep them.
    assert(store.approve(storage::ImuApproval::Verified));
  }
  const auto beforeInvalid=flash.writes; auto invalid=measured; invalid.bias[0]=std::numeric_limits<float>::quiet_NaN();
  assert(!store.save(invalid) && store.verified() && flash.writes==beforeInvalid);
  storage::ImuCalibrationStore reboot(flash); assert(reboot.load() && reboot.verified());
  assert(store.approve(storage::ImuApproval::InUse));
  assert(reboot.load() && !reboot.verified()); // Reset before or after Boost requires verification.
  assert(store.approve(storage::ImuApproval::Verified));
  flash.tear=true; assert(!store.approve(storage::ImuApproval::InUse)); flash.tear=false;
  assert(reboot.load() && !reboot.verified()); // Never fall back to an earlier approval.
  assert(reboot.identityAllowed()); // A torn write is not a failed verification: identity stays allowed.
  assert(reboot.needsCalibration() && !reboot.approve(storage::ImuApproval::Verified));
  assert(reboot.save(measured));
  assert(store.load() && store.verified());
  for (unsigned i=0;i<90;++i) assert(store.approve(i%2 ? storage::ImuApproval::Verified:storage::ImuApproval::InUse));
  assert(reboot.load() && reboot.verified()); // Journal compaction (126 record pages per bank).
  flash.failRead=true; assert(!reboot.load() && !reboot.verified()); flash.failRead=false;

  // The identity revocation survives journal compaction and a factory reset.
  CalibrationFlash factoryFlash; storage::ImuCalibrationStore factory(factoryFlash);
  assert(!factory.load() && factory.identityAllowed());
  storage::ImuCalibrationRecord identity; identity.revision=1; identity.identity=true;
  identity.approval=storage::ImuApproval::Verified;
  assert(factory.save(identity) && factory.approve(storage::ImuApproval::InUse));
  assert(factory.invalidate() && !factory.identityAllowed() && factory.needsCalibration());
  storage::ImuCalibrationStore factoryReboot(factoryFlash);
  assert(factoryReboot.load() && !factoryReboot.identityAllowed() && factoryReboot.needsCalibration());
  assert(!factoryReboot.approve(storage::ImuApproval::Verified) && !factoryReboot.save(identity));
  assert(factoryReboot.save(measured) && factoryReboot.verified());
  for (unsigned i=0;i<90;++i) assert(factoryReboot.approve(i%2 ? storage::ImuApproval::Verified:storage::ImuApproval::InUse));
  assert(factory.load() && !factory.identityAllowed());
  { storage::ConfigurationStore settings(factoryFlash); assert(settings.clear(true)); } // Factory reset.
  assert(factory.load() && !factory.identityAllowed() && factory.verified());
  // An unreadable journal has no records: it neither revokes the identity nor invents one.
  CalibrationFlash unknownFlash; unknownFlash.bytes[0]=0;
  storage::ImuCalibrationStore unknown(unknownFlash);
  assert(!unknown.load() && unknown.identityAllowed());
  // A revocation interrupted by a reset (Invalid saved, revocation not yet written) is completed at load.
  CalibrationFlash interruptedFlash; storage::ImuCalibrationStore interrupted(interruptedFlash);
  assert(!interrupted.load() && interrupted.save(identity) && interrupted.approve(storage::ImuApproval::Invalid));
  storage::ImuCalibrationStore resumed(interruptedFlash);
  assert(resumed.load() && !resumed.identityAllowed());
  storage::ImuCalibrationStore again(interruptedFlash); assert(again.load() && !again.identityAllowed());
  CalibrationFlash unreadableFlash; unreadableFlash.failRead=true;
  storage::ImuCalibrationStore unreadable(unreadableFlash);
  assert(!unreadable.load() && !unreadable.identityAllowed());
  // A custom calibration that fails before any use is dropped; identity is still allowed.
  CalibrationFlash customFlash; storage::ImuCalibrationStore custom(customFlash);
  auto attempt=measured; attempt.approval=storage::ImuApproval::Required; attempt.identity=false; // Saved, never verified.
  assert(!custom.load() && custom.identityAllowed() && custom.save(attempt) && !custom.verified());
  assert(custom.discard() && custom.identityAllowed() && custom.verified() && custom.record().identity);
  for (unsigned i=0;i<3;++i) assert(custom.record().bias[i]==0 && custom.record().scale[i]==1);
  assert(!custom.needsCalibration());
  // Without identity, a failed custom calibration must be redone.
  factory.load(); // The revoked identity stays revoked.
  assert(!factory.identityAllowed() && factory.save(attempt) && factory.discard() && factory.needsCalibration());
  // Wrong position stops the capture without failing; a stable correctly oriented but biased measurement fails.
  storage::ImuCalibrationRecord shifted; shifted.bias[0]=.8f;
  collector.begin(true,identity); assert(collector.capture(0)); feed(collector,2,identity);
  assert(!collector.failed() && !collector.collecting()); assert(collector.capture(0)); feed(collector,0,shifted);
  assert(collector.failed() && !collector.complete() && !collector.result(measured));

  MemoryFlash settingsFlash; storage::ConfigurationStore settingsStore(settingsFlash);
  auto plan=fixture(); plan.rules[0].assignmentCount=1;
  assert(settingsStore.save(plan,limits,true)); ConfigurationService config(settingsStore,limits); config.boot(true);
  assert(!config.setOrientation(0));
  Nor logs; storage::FlightRecorder recorder(logs); assert(recorder.initializeBlank());
  flight::RunController run; FakeOutputs gpio; flight::OutputExecutor outputs(gpio,clockUs);
  flight::ArmingController controller(config,run,clockUs,&recorder,&outputs);
  Checks checks; checks.healthy(); Source source;
  auto p=controller.preflight(flight::FlightMode::Physical,true,&checks);
  assert(p.has(flight::ArmBlocker::ImuCalibrationMissing));
  CalibrationFlash initialFlash; storage::ImuCalibrationStore initialStore(initialFlash);
  assert(!initialStore.load() && initialStore.identityAllowed());
  controller.setImuCalibrationStore(initialStore);
  assert(controller.preflight(flight::FlightMode::Physical,true,&checks).ready());
  assert(!strcmp(controller.imuCalibrationState(),"identity"));
  assert(!controller.beginImuCalibration(source,false)); controller.cancelImuCalibration();
  assert(!initialStore.present() && initialStore.identityAllowed());
  assert(!controller.arm(source,flight::FlightMode::Physical,0,120,{},4096,&checks));
  assert(initialStore.present() && initialStore.record().identity);
  for (unsigned i=0;i<3;++i) assert(initialStore.record().bias[i]==0 && initialStore.record().scale[i]==1);
  assert(!controller.disarm() && initialStore.verified());
  controller.setImuCalibrationStore(store); assert(controller.preflight(flight::FlightMode::Physical,true,&checks).ready());
  ArmingCommands commands(controller,source,checks);
  ConfigurationCommands configuration(config); EstimateReplayCommands estimates(config,run);
  simulation::HilSession hilSession(config,run,clockUs); SensorHilCommands hil(hilSession);
  CommandRouter router(configuration,estimates,hil,config,run,nullptr,nullptr,&commands);
  CommandSession usb(router,CommandRouter::Source::Usb),ble(router,CommandRouter::Source::Bluetooth);
  CommandService endpoint,other; endpoint.setExtension(&usb); other.setExtension(&ble);
  has(send(endpoint,"imu status"),"\"imu_calibration_state\":\"verified\"");
  has(send(endpoint,"imu verify"),"\"active\":true");
  has(send(other,"imu capture +X"),"CONTROL_BUSY");
  has(send(endpoint,"config orientation +Y"),"BUSY");
  has(send(endpoint,"imu capture nonsense"),"INVALID_ARGUMENT");
  has(send(endpoint,"imu capture +X"),"\"collecting\":true");
  has(send(endpoint,"imu cancel"),"\"active\":false"); assert(store.verified() && !config.runLocked());

  nowUs=0;
  assert(!controller.arm(source,flight::FlightMode::Physical,0,120,{},4096,&checks));
  assert(store.record().approval==storage::ImuApproval::InUse);
  storage::ImuCalibrationStore resetDuringArm(flash); assert(resetDuringArm.load() && !resetDuringArm.verified());
  assert(!controller.disarm()); assert(store.verified()); // No launch preserves approval.
  flight::FrameResult result;
  auto acquireFace=[&](unsigned face) {
    assert(!controller.captureImuFace(face));
    for (unsigned n=0;n<260 && controller.imuCalibration().collecting();++n) {
      nowUs+=10000; auto &r=source.reading; r={}; r.imuValid=true; r.timeUs=nowUs; r.imuSequence=nowUs/10000;
      for (unsigned i=0;i<3;++i) r.accelMps2[i]=truth.bias[i]+(i==face/2 ? (face%2?-G:G)/truth.scale[i]:0);
      source.pending=true; controller.poll(result);
    }
  };
  assert(!controller.beginImuCalibration(source,false));
  for (unsigned face=0;face<6;++face) acquireFace(face);
  assert(!controller.imuCalibration().active() && store.present() && !store.verified() && store.record().revision==2);
  // A new calibration is not verified yet: the residuals of the previous one no longer apply.
  assert(std::isnan(store.record().residual[0]) && send(endpoint,"imu status").find("verification_residual")==std::string::npos);
  assert(controller.preflight(flight::FlightMode::Physical,true,&checks).has(flight::ArmBlocker::ImuVerificationRequired));
  assert(!controller.beginImuCalibration(source,true)); acquireFace(0); acquireFace(2); acquireFace(4);
  assert(!controller.imuCalibration().active() && store.verified() && !strcmp(controller.imuCalibrationState(),"verified"));
  { // The passed verification's residuals are saved with the approval, survive a reload, and imu status reports them.
    for (unsigned i=0;i<3;++i) assert(std::fabs(store.record().residual[i])<.01f);
    storage::ImuCalibrationStore reloaded(flash); reloaded.load(); assert(std::fabs(reloaded.record().residual[2])<.01f);
    const auto reply=send(endpoint,"imu status"); has(reply,"\"verification_residual_mps2\":[");
    assert(reply.size()<700 && reply.find("RESPONSE_TOO_LARGE")==std::string::npos);
  }
  assert(!controller.arm(source,flight::FlightMode::Physical,0,120,{},4096,&checks));
  for (unsigned n=0;n<500;++n) {
    nowUs+=10000; auto &r=source.reading; r={}; r.imuValid=r.baroValid=true;
    r.timeUs=nowUs; r.imuSequence=n+1; r.baroTimeUs=nowUs; r.baroSequence=n+1; r.pressurePa=101325;
    for (unsigned i=0;i<3;++i) r.accelMps2[i]=truth.bias[i]+(i==0 ? G/truth.scale[i]:0);
    source.pending=true; controller.poll(result);
  }
  assert(controller.armed());
  assert(recorder.metadata()->calibration.accelCalibrationRevision==2);
  assert(std::fabs(recorder.metadata()->calibration.accelScale[0]-truth.scale[0])<.00001f);
  const auto calibrationWrites=flash.writes;
  for (unsigned n=0;n<22;++n) { // BOOST: 200 ms after a 3 g sample.
    nowUs+=10000; auto &r=source.reading; r.timeUs=nowUs; ++r.imuSequence; r.baroTimeUs=nowUs; ++r.baroSequence;
    r.accelMps2[0]=truth.bias[0]+40/truth.scale[0]; source.pending=true; controller.poll(result);
  }
  assert(controller.runtime().status().boostReached && !strcmp(controller.imuCalibrationReason(),"AFTER_BOOST"));
  assert(flash.writes==calibrationWrites); // No calibration flash access at Boost.
  controller.stop(); controller.maintainImuApproval();
  assert(controller.preflight(flight::FlightMode::Physical,true,&checks).has(flight::ArmBlocker::ImuVerificationRequired));
  assert(!store.verified());
  // A failed post-Boost verification requires a new calibration, not another verification.
  assert(!controller.beginImuCalibration(source,true));
  truth.bias[0]+=.8f; acquireFace(0); truth.bias[0]-=.8f;
  assert(!controller.active() && store.needsCalibration() && !store.identityAllowed());
  { // A failed verification keeps the face that failed; the faces not reached stay unmeasured.
    const auto &r=store.record(); assert(r.residual[0]>.35f && std::isnan(r.residual[1]) && std::isnan(r.residual[2]));
    has(send(endpoint,"imu status"),"null,null]");
  }
  assert(!strcmp(controller.imuError(),"IMU_VERIFICATION_FAILED"));
  assert(!strcmp(controller.imuCalibrationState(),"invalid"));
  assert(!strcmp(controller.beginImuCalibration(source,true),"IMU_CALIBRATION_MISSING"));
  assert(controller.preflight(flight::FlightMode::Physical,true,&checks).has(flight::ArmBlocker::ImuCalibrationMissing));
  assert(reboot.load() && reboot.needsCalibration() && !reboot.identityAllowed());
  assert(!controller.beginImuCalibration(source,false));
  for (unsigned face=0;face<6;++face) acquireFace(face);
  assert(!store.needsCalibration() && !store.verified() && !store.identityAllowed());
  assert(std::isnan(store.record().residual[0])); // The new calibration has not been verified.
  assert(!controller.beginImuCalibration(source,true)); acquireFace(0); acquireFace(2); acquireFace(4);
  assert(store.verified() && !store.identityAllowed() && !std::isnan(store.record().residual[1]));
  { storage::ConfigurationStore settings(flash); assert(settings.clear(true)); } // Factory reset keeps the calibration and its residuals.
  { storage::ImuCalibrationStore reset(flash); reset.load(); assert(reset.verified() && !std::isnan(reset.record().residual[1])); }
  // Simulations, including physical outputs, must leave calibration approval untouched.
  const auto beforeSimulation=flash.writes;
  navigation::SensorCalibration simulation; simulation.configured=true;
  assert(!controller.arm(source,flight::FlightMode::Simulation,0,120,simulation,0,&checks));
  controller.stop(); assert(store.verified() && flash.writes==beforeSimulation);
  MemoryFlash empty; storage::ConfigurationStore emptyStore(empty); ConfigurationService emptyConfig(emptyStore,limits); emptyConfig.boot(false);
  flight::ArmingController unavailable(emptyConfig,run); Checks unhealthy;
  ArmingCommands unavailableCommands(unavailable,source,unhealthy); CommandService unavailableUsb; unavailableUsb.setExtension(&unavailableCommands);
  const auto report=send(unavailableUsb,"@4294967295 preflight");
  has(report,"IMU_CALIBRATION_MISSING"); has(report,"\"ok\":true");
  // Device calibration must be possible before any recovery rules are configured.
  emptyConfig.boot(true); unavailable.setImuCalibrationStore(store);
  assert(!emptyConfig.configValid());
  flight::ArmingController initial(emptyConfig,run,clockUs); initial.setImuCalibrationStore(store);
  assert(!initial.beginImuCalibration(source,false) && emptyConfig.runLocked());
  nowUs+=180000001u; initial.poll(result);
  assert(!initial.active() && !emptyConfig.runLocked() && !strcmp(initial.imuError(),"IMU_CALIBRATION_TIMEOUT"));
  assert(store.verified());
  flash.tear=true;
  assert(!strcmp(controller.arm(source,flight::FlightMode::Physical,0,120,{},4096,&checks),"IMU_CALIBRATION_IO"));
  assert(!controller.active() && !config.runLocked() && !store.verified()); flash.tear=false;
  puts("IMU calibration passed: six-face bias/scale, independent verification, motion, torn writes, reboot, ownership, arming reservation and Boost consumption.");
}
}
