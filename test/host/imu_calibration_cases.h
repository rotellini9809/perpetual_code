#pragma once
#include "flight/ImuCalibration.h"
#include <limits>

namespace test_imu_calibration {
using namespace test_arming_controller;
constexpr float G=9.80665f;
struct CalibrationFlash : storage::FlashDevice {
  uint8_t bytes[4096]; unsigned writes=0; bool tear=false,failRead=false;
  CalibrationFlash() { std::memset(bytes,255,sizeof(bytes)); }
  bool read(uint32_t a,void *p,std::size_t n) override {
    if (failRead || a+n>sizeof(bytes)) return false;
    std::memcpy(p,bytes+a,n); return true;
  }
  bool program(uint32_t a,const void *p,std::size_t n) override {
    assert(a+n<=sizeof(bytes)); ++writes;
    const auto *b=static_cast<const uint8_t *>(p);
    for (unsigned i=0;i<(tear?17:n);++i) bytes[a+i]&=b[i];
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
  assert(reboot.approve(storage::ImuApproval::Verified));
  assert(store.load() && store.verified());
  for (unsigned i=0;i<30;++i) assert(store.approve(i%2 ? storage::ImuApproval::Verified:storage::ImuApproval::InUse));
  assert(reboot.load() && reboot.verified()); // Journal compaction.
  flash.failRead=true; assert(!reboot.load() && !reboot.verified()); flash.failRead=false;

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
  assert(store.approve(storage::ImuApproval::Verified));
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
