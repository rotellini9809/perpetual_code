#pragma once
#include "flight/ImuCalibration.h"
#include <limits>

namespace test_imu_calibration {
using namespace test_arming_controller;
constexpr float G=9.80665f;
struct CalibrationFlash : storage::FlashDevice {
  uint8_t bytes[4096],policy[4096]; unsigned writes=0; bool tear=false,failRead=false;
  CalibrationFlash() { std::memset(bytes,255,sizeof(bytes)); std::memset(policy,255,sizeof(policy)); }
  bool read(uint32_t a,void *p,std::size_t n) override {
    if (failRead) return false;
    if (a>=storage::FlashLayout::IdentityPolicyBegin) {
      a-=storage::FlashLayout::IdentityPolicyBegin;
      if (a+n>sizeof(policy)) return false;
      std::memcpy(p,policy+a,n); return true;
    }
    if (a+n>sizeof(bytes)) return false;
    std::memcpy(p,bytes+a,n); return true;
  }
  bool program(uint32_t a,const void *p,std::size_t n) override {
    uint8_t *target=bytes;
    if (a>=storage::FlashLayout::IdentityPolicyBegin) { a-=storage::FlashLayout::IdentityPolicyBegin; target=policy; }
    assert(a+n<=sizeof(bytes)); ++writes;
    const auto *b=static_cast<const uint8_t *>(p);
    for (unsigned i=0;i<(tear?std::min<std::size_t>(17,n):n);++i) target[a+i]&=b[i];
    return !tear;
  }
  bool eraseSector(uint32_t a) override { assert(!a); ++writes; std::memset(bytes,255,sizeof(bytes)); return true; }
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
  feed(collector,2,truth); assert(!collector.samples() && collector.collecting()); // Wrong face.
  feed(collector,0,truth); assert(!collector.complete());
  assert(collector.capture(2)); feed(collector,2,truth);
  assert(collector.complete() && collector.result(measured) && measured.approval==storage::ImuApproval::Verified);
  collector.begin(false,{}); assert(collector.capture(0));
  feed(collector,0,truth,20); assert(collector.samples()==20);
  sensors::Reading bad; bad.imuValid=true; bad.imuSequence=100000; bad.timeUs=1000000000;
  bad.accelMps2[0]=std::numeric_limits<float>::quiet_NaN(); collector.add(bad); assert(!collector.samples());
  collector.cancel(); assert(!collector.active());

  CalibrationFlash flash; storage::ImuCalibrationStore store(flash); assert(!store.load());
  assert(store.save(measured) && store.verified());
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
  for (unsigned i=0;i<30;++i) assert(store.approve(i%2 ? storage::ImuApproval::Verified:storage::ImuApproval::InUse));
  assert(reboot.load() && reboot.verified()); // Journal compaction.
  flash.failRead=true; assert(!reboot.load() && !reboot.verified()); flash.failRead=false;

  // The identity eligibility bit survives coefficient erases and journal compaction.
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
  for (unsigned i=0;i<30;++i) assert(factoryReboot.approve(i%2 ? storage::ImuApproval::Verified:storage::ImuApproval::InUse));
  assert(factory.load() && !factory.identityAllowed());
  factoryFlash.eraseSector(0); // Even lost coefficient records cannot resurrect factory identity.
  assert(!factory.load() && !factory.identityAllowed());
  // Only the policy byte decides: unreadable coefficients do not revoke the identity...
  CalibrationFlash unknownFlash; unknownFlash.bytes[0]=0;
  storage::ImuCalibrationStore unknown(unknownFlash);
  assert(!unknown.load() && unknown.identityAllowed());
  // ...while any value other than the erased one counts as revoked, and is never rewritten as allowed.
  CalibrationFlash damagedPolicy; damagedPolicy.policy[0]=0x5A; storage::ImuCalibrationStore damagedStore(damagedPolicy);
  assert(!damagedStore.load() && !damagedStore.identityAllowed() && damagedPolicy.policy[0]==0x5A);
  // A revocation interrupted by a reset (Invalid saved, byte not yet written) is completed at load.
  CalibrationFlash interruptedFlash; storage::ImuCalibrationStore interrupted(interruptedFlash);
  assert(!interrupted.load() && interrupted.save(identity) && interrupted.approve(storage::ImuApproval::Invalid));
  assert(interruptedFlash.policy[0]==255); storage::ImuCalibrationStore resumed(interruptedFlash);
  assert(resumed.load() && !resumed.identityAllowed() && interruptedFlash.policy[0]==0);
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
  factory.load(); // Coefficients were erased above; the revoked identity stays revoked.
  assert(!factory.identityAllowed() && factory.save(attempt) && factory.discard() && factory.needsCalibration());
  // Wrong position retries; a stable correctly oriented but biased measurement fails.
  storage::ImuCalibrationRecord shifted; shifted.bias[0]=.8f;
  collector.begin(true,identity); assert(collector.capture(0)); feed(collector,2,identity);
  assert(!collector.failed()); feed(collector,0,shifted);
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
  assert(controller.preflight(flight::FlightMode::Physical,true,&checks).has(flight::ArmBlocker::ImuVerificationRequired));
  assert(!controller.beginImuCalibration(source,true)); acquireFace(0); acquireFace(2);
  assert(!controller.imuCalibration().active() && store.verified() && !strcmp(controller.imuCalibrationState(),"verified"));
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
  for (unsigned n=0;n<4;++n) {
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
  assert(!strcmp(controller.imuError(),"IMU_VERIFICATION_FAILED"));
  assert(!strcmp(controller.imuCalibrationState(),"invalid"));
  assert(!strcmp(controller.beginImuCalibration(source,true),"IMU_CALIBRATION_MISSING"));
  assert(controller.preflight(flight::FlightMode::Physical,true,&checks).has(flight::ArmBlocker::ImuCalibrationMissing));
  assert(reboot.load() && reboot.needsCalibration() && !reboot.identityAllowed());
  assert(!controller.beginImuCalibration(source,false));
  for (unsigned face=0;face<6;++face) acquireFace(face);
  assert(!store.needsCalibration() && !store.verified() && !store.identityAllowed());
  assert(!controller.beginImuCalibration(source,true)); acquireFace(0); acquireFace(2);
  assert(store.verified() && !store.identityAllowed());
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
