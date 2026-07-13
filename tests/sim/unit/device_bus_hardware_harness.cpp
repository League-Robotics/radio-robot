// device_bus_hardware_harness.cpp — off-hardware acceptance harness for
// ticket 100-DBX (clasi/sprints/100-.../device-bus-cutover-ticket.md): the
// COMPLETE CUTOVER bridge, source/subsystems/device_bus_hardware.{h,cpp}.
//
// Two tiers, matching that ticket's own Verify section ("Host-test the
// conversion helpers... If the DeviceBus can't be constructed host-side
// without CODAL, note it and host-test only the pure conversion
// functions"):
//
//   TIER 1 — the pure conversion helpers (deviceBusMotorConfigToMsg()/
//   msgToDeviceBusMotorConfig()/otosBootConfigToDeviceBus()/
//   deviceBusPoseToEstimate()/msgNeutralToDeviceBus()), exercised directly
//   against hand-built Devices::/msg:: values — no DeviceBus/bus/fiber
//   construction at all.
//
//   TIER 2 — Devices::DeviceBus turns out to be fully host-constructible
//   (device_bus.h's own #ifdef HOST_BUILD constructor, already proven by
//   device_bus_lifecycle_harness.cpp), so DeviceBusHardware is too. This
//   tier constructs a REAL Subsystems::DeviceBusHardware (HOST_BUILD ctor,
//   no i2c, never start()'d — deviceBus_.start() is deliberately never
//   called here, so NO bus traffic occurs at all: every assertion below
//   exercises pure construction/apply()/state()/capabilities()/config()
//   plumbing) and drives it through the SAME Hal::Motor/Hal::Odometer
//   contract the real motion stack uses.
//
// Modeled on hardware_seam_harness.cpp / device_bus_lifecycle_harness.cpp's
// own hand-rolled assertion plumbing. Run by test_device_bus_hardware.py,
// which compiles and runs this binary via subprocess.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "subsystems/device_bus_hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors device_bus_lifecycle_harness.cpp) ---

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
}

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-4f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

void checkIntEq(int32_t actual, int32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %d, got %d", what.c_str(),
                  static_cast<int>(expected), static_cast<int>(actual));
    fail(buf);
  }
}

// --- Fixture helpers -----------------------------------------------------

Devices::MotorConfig sampleDeviceBusMotorConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.wheelTravelCalib = 0.7165f + static_cast<float>(port) * 0.01f;
  cfg.fwdSign = (port % 2 == 0) ? -1 : 1;
  cfg.velGains.kp = 0.0014f;
  cfg.velGains.ki = 0.005f;
  cfg.velGains.kff = 0.00135f;
  cfg.velGains.iMax = 0.3f;
  cfg.velGains.kaw = 20.0f;
  cfg.velFiltAlpha = 0.3f;
  cfg.velDeadband = 5.0f;
  cfg.slewRate = 25.0f;
  cfg.port = port;
  cfg.reversalDwell.has = true;
  cfg.reversalDwell.val = 100.0f;
  cfg.outputDeadband.has = true;
  cfg.outputDeadband.val = 0.03f;
  cfg.polled = (port <= 2);
  return cfg;
}

msg::MotorConfig sampleMsgMotorConfig(uint32_t port) {
  msg::MotorConfig cfg;
  cfg.travel_calib = 0.7165f + static_cast<float>(port) * 0.01f;
  cfg.fwd_sign = (port % 2 == 0) ? -1 : 1;
  cfg.vel_gains.kp = 0.0014f;
  cfg.vel_gains.ki = 0.005f;
  cfg.vel_gains.kff = 0.00135f;
  cfg.vel_gains.i_max = 0.3f;
  cfg.vel_gains.kaw = 20.0f;
  cfg.vel_filt_alpha = 0.3f;
  cfg.min_duty = 5.0f;
  cfg.slew_rate = 25.0f;
  cfg.port = port;
  cfg.reversal_dwell.has = true;
  cfg.reversal_dwell.val = 100.0f;
  cfg.output_deadband.has = true;
  cfg.output_deadband.val = 0.03f;
  cfg.polled = (port <= 2);
  return cfg;
}

// --- TIER 1 scenarios: pure conversion helpers ----------------------------

void scenarioDeviceBusMotorConfigToMsgRoundTrips() {
  beginScenario("deviceBusMotorConfigToMsg() carries every field across the isolation boundary");

  Devices::MotorConfig cfg = sampleDeviceBusMotorConfig(3);
  msg::MotorConfig out = Subsystems::deviceBusMotorConfigToMsg(cfg);

  checkFloatEq(out.travel_calib, cfg.wheelTravelCalib, "travel_calib <- wheelTravelCalib");
  checkIntEq(out.fwd_sign, cfg.fwdSign, "fwd_sign <- fwdSign");
  checkFloatEq(out.vel_gains.kp, cfg.velGains.kp, "vel_gains.kp");
  checkFloatEq(out.vel_gains.ki, cfg.velGains.ki, "vel_gains.ki");
  checkFloatEq(out.vel_gains.kff, cfg.velGains.kff, "vel_gains.kff");
  checkFloatEq(out.vel_gains.i_max, cfg.velGains.iMax, "vel_gains.i_max <- velGains.iMax");
  checkFloatEq(out.vel_gains.kaw, cfg.velGains.kaw, "vel_gains.kaw");
  checkFloatEq(out.vel_filt_alpha, cfg.velFiltAlpha, "vel_filt_alpha <- velFiltAlpha");
  checkFloatEq(out.min_duty, cfg.velDeadband, "min_duty <- velDeadband (documented same-quantity rename)");
  checkFloatEq(out.slew_rate, cfg.slewRate, "slew_rate <- slewRate");
  checkUintEq(out.port, cfg.port, "port");
  checkTrue(out.reversal_dwell.has, "reversal_dwell.has propagates");
  checkFloatEq(out.reversal_dwell.val, cfg.reversalDwell.val, "reversal_dwell.val");
  checkTrue(out.output_deadband.has, "output_deadband.has propagates");
  checkFloatEq(out.output_deadband.val, cfg.outputDeadband.val, "output_deadband.val");
  checkTrue(out.polled == cfg.polled, "polled");
}

void scenarioMsgToDeviceBusMotorConfigRoundTrips() {
  beginScenario("msgToDeviceBusMotorConfig() is the exact inverse of deviceBusMotorConfigToMsg()");

  msg::MotorConfig cfg = sampleMsgMotorConfig(4);
  Devices::MotorConfig out = Subsystems::msgToDeviceBusMotorConfig(cfg);

  checkFloatEq(out.wheelTravelCalib, cfg.travel_calib, "wheelTravelCalib <- travel_calib");
  checkIntEq(out.fwdSign, cfg.fwd_sign, "fwdSign <- fwd_sign");
  checkFloatEq(out.velGains.kp, cfg.vel_gains.kp, "velGains.kp");
  checkFloatEq(out.velGains.iMax, cfg.vel_gains.i_max, "velGains.iMax <- vel_gains.i_max");
  checkFloatEq(out.velDeadband, cfg.min_duty, "velDeadband <- min_duty");
  checkFloatEq(out.slewRate, cfg.slew_rate, "slewRate <- slew_rate");
  checkUintEq(out.port, cfg.port, "port");
  checkTrue(out.reversalDwell.has, "reversalDwell.has propagates");
  checkFloatEq(out.reversalDwell.val, cfg.reversal_dwell.val, "reversalDwell.val");
  checkTrue(out.outputDeadband.has, "outputDeadband.has propagates");
  checkTrue(out.polled == cfg.polled, "polled");

  // Full round trip: Devices -> msg -> Devices reproduces the original.
  Devices::MotorConfig original = sampleDeviceBusMotorConfig(1);
  Devices::MotorConfig roundTripped =
      Subsystems::msgToDeviceBusMotorConfig(Subsystems::deviceBusMotorConfigToMsg(original));
  checkFloatEq(roundTripped.wheelTravelCalib, original.wheelTravelCalib, "round trip: wheelTravelCalib");
  checkIntEq(roundTripped.fwdSign, original.fwdSign, "round trip: fwdSign");
  checkFloatEq(roundTripped.velDeadband, original.velDeadband, "round trip: velDeadband");
  checkUintEq(roundTripped.port, original.port, "round trip: port");
}

void scenarioOtosBootConfigToDeviceBusCopiesEveryField() {
  beginScenario("otosBootConfigToDeviceBus() copies every field 1:1");

  Config::OtosBootConfig cfg;
  cfg.offsetX = -47.7f;
  cfg.offsetY = 3.5f;
  cfg.offsetYaw = 0.1f;
  cfg.linearScale = 1.067f;
  cfg.angularScale = 0.987f;

  Devices::OtosConfig out = Subsystems::otosBootConfigToDeviceBus(cfg);

  checkFloatEq(out.offsetX, cfg.offsetX, "offsetX");
  checkFloatEq(out.offsetY, cfg.offsetY, "offsetY");
  checkFloatEq(out.offsetYaw, cfg.offsetYaw, "offsetYaw");
  checkFloatEq(out.linearScale, cfg.linearScale, "linearScale");
  checkFloatEq(out.angularScale, cfg.angularScale, "angularScale");
}

void scenarioDeviceBusPoseToEstimateNeverPublishedIsInvalid() {
  beginScenario("deviceBusPoseToEstimate() reports stamp.valid=false for a never-published sample");

  Devices::Sample<Devices::PoseReading> sample;  // default: valid=false, stamp=0
  msg::PoseEstimate out = Subsystems::deviceBusPoseToEstimate(sample);

  checkFalse(out.stamp.valid, "a default-constructed (never-published) ring sample is not valid");
}

void scenarioDeviceBusPoseToEstimateConvertsFreshSample() {
  beginScenario("deviceBusPoseToEstimate() converts a fresh, valid PoseReading correctly");

  Devices::Sample<Devices::PoseReading> sample;
  sample.value.x = 120.5f;
  sample.value.y = -30.25f;
  sample.value.heading = 1.5708f;
  sample.value.v_x = 250.0f;
  sample.value.v_y = -10.0f;
  sample.value.omega = 0.75f;
  sample.stamp = 12345000ULL;  // [us]
  sample.valid = true;

  msg::PoseEstimate out = Subsystems::deviceBusPoseToEstimate(sample);

  checkFloatEq(out.pose.x, 120.5f, "pose.x");
  checkFloatEq(out.pose.y, -30.25f, "pose.y");
  checkFloatEq(out.pose.h, 1.5708f, "pose.h");
  checkFloatEq(out.twist.v_x, 250.0f, "twist.v_x");
  checkFloatEq(out.twist.v_y, -10.0f, "twist.v_y");
  checkFloatEq(out.twist.omega, 0.75f, "twist.omega");
  checkTrue(out.stamp.valid, "a fresh, valid sample converts to stamp.valid=true");
  checkUintEq(out.stamp.last_upd, 12345u, "stamp.last_upd is the [us] ring stamp converted to [ms]");
}

void scenarioMsgNeutralToDeviceBusMapsByName() {
  beginScenario("msgNeutralToDeviceBus() maps BRAKE/COAST by name, not by underlying int value");

  checkTrue(Subsystems::msgNeutralToDeviceBus(msg::Neutral::BRAKE) == Devices::Neutral::Brake,
            "BRAKE -> Devices::Neutral::Brake");
  checkTrue(Subsystems::msgNeutralToDeviceBus(msg::Neutral::COAST) == Devices::Neutral::Coast,
            "COAST -> Devices::Neutral::Coast");
}

// --- TIER 2 scenarios: a real, host-constructed DeviceBusHardware ---------
// None of these ever call begin()/deviceBus_.start() -- no bus traffic
// occurs; every assertion exercises pure construction/apply()/state()/
// capabilities()/motorConfig() plumbing (handles.h's setters are plain,
// yield-free stores onto the underlying leaf's staged fields -- safe to
// call whether or not the fiber is running).

void scenarioMotorConfigReportsBootSnapshotPerPort() {
  beginScenario("DeviceBusHardware::motorConfig(i) returns the construction-time boot snapshot, verbatim, per port");

  msg::MotorConfig configs[Subsystems::DeviceBusHardware::kMotorCount];
  for (uint32_t i = 0; i < Subsystems::DeviceBusHardware::kMotorCount; ++i) {
    configs[i] = sampleMsgMotorConfig(i + 1);
  }
  Config::OtosBootConfig otosConfig;

  Subsystems::DeviceBusHardware hardware(configs, otosConfig);

  for (uint32_t i = 0; i < Subsystems::DeviceBusHardware::kMotorCount; ++i) {
    msg::MotorConfig got = hardware.motorConfig(i);
    checkUintEq(got.port, configs[i].port, "motorConfig(i).port matches the ctor's configs[i]");
    checkFloatEq(got.travel_calib, configs[i].travel_calib, "motorConfig(i).travel_calib matches the ctor's configs[i]");
  }

  // Out-of-range clamps to kMotorCount-1, mirroring NezhaHardware's own
  // clampIndex() convention.
  msg::MotorConfig clamped = hardware.motorConfig(99);
  checkUintEq(clamped.port, configs[3].port, "motorConfig(99) clamps to index kMotorCount-1 (port 4)");
}

void scenarioMotorCapabilitiesAreFixedDifferential() {
  beginScenario("DeviceBusMotor::capabilities() is a fixed differential-drive capability set");

  msg::MotorConfig configs[Subsystems::DeviceBusHardware::kMotorCount];
  for (uint32_t i = 0; i < Subsystems::DeviceBusHardware::kMotorCount; ++i) {
    configs[i] = sampleMsgMotorConfig(i + 1);
  }
  Subsystems::DeviceBusHardware hardware(configs, Config::OtosBootConfig());

  msg::MotorCapabilities caps = hardware.motor(0).capabilities();
  checkTrue(caps.duty_cycle, "duty_cycle supported");
  checkFalse(caps.voltage, "voltage NOT supported -- Devices::Motor handle has no voltage primitive");
  checkTrue(caps.velocity, "velocity (embedded PID) supported");
  checkFalse(caps.position, "position NOT supported -- Devices::Motor handle has no position-move primitive");
  checkTrue(caps.has_encoder, "has_encoder");
}

void scenarioApplyRejectsUnsupportedVoltageMode() {
  beginScenario("apply() rejects VOLTAGE (capability-gated) before touching the handle, active_ untouched");

  msg::MotorConfig configs[Subsystems::DeviceBusHardware::kMotorCount];
  for (uint32_t i = 0; i < Subsystems::DeviceBusHardware::kMotorCount; ++i) {
    configs[i] = sampleMsgMotorConfig(i + 1);
  }
  Subsystems::DeviceBusHardware hardware(configs, Config::OtosBootConfig());

  msg::MotorCommand cmd;
  cmd.setVoltage(5.0f);
  bool accepted = hardware.motor(0).apply(cmd);

  checkFalse(accepted, "apply() returns false for a VOLTAGE command (capabilities().voltage == false)");
  checkFalse(hardware.motorState(0).active, "active stays false -- a rejected command must not flip it");
}

void scenarioApplyVelocityThenNeutralTogglesActive() {
  beginScenario("apply() VELOCITY sets active=true; apply() NEUTRAL clears it back to false");

  msg::MotorConfig configs[Subsystems::DeviceBusHardware::kMotorCount];
  for (uint32_t i = 0; i < Subsystems::DeviceBusHardware::kMotorCount; ++i) {
    configs[i] = sampleMsgMotorConfig(i + 1);
  }
  Subsystems::DeviceBusHardware hardware(configs, Config::OtosBootConfig());

  msg::MotorCommand velCmd;
  velCmd.setVelocity(250.0f);
  bool accepted = hardware.motor(0).apply(velCmd);
  checkTrue(accepted, "apply() accepts a VELOCITY command (capabilities().velocity == true)");
  checkTrue(hardware.motorState(0).active, "active becomes true after a VELOCITY command");

  msg::MotorCommand neutralCmd;
  neutralCmd.setNeutral(msg::Neutral::COAST);
  accepted = hardware.motor(0).apply(neutralCmd);
  checkTrue(accepted, "apply() always accepts NEUTRAL, ungated by capabilities()");
  checkFalse(hardware.motorState(0).active, "active returns to false after a NEUTRAL command");
}

void scenarioMotorStartsDisconnectedBeforeAnyPublish() {
  beginScenario("a freshly constructed, never-started DeviceBusHardware reports every motor disconnected");

  msg::MotorConfig configs[Subsystems::DeviceBusHardware::kMotorCount];
  for (uint32_t i = 0; i < Subsystems::DeviceBusHardware::kMotorCount; ++i) {
    configs[i] = sampleMsgMotorConfig(i + 1);
  }
  Subsystems::DeviceBusHardware hardware(configs, Config::OtosBootConfig());

  for (uint32_t i = 0; i < Subsystems::DeviceBusHardware::kMotorCount; ++i) {
    checkFalse(hardware.motorState(i).connected,
               "motor " + std::to_string(i) + " reports disconnected -- the fiber was never started, no encoder sample was ever collected");
  }
}

void scenarioOdometerDefaultsBeforeAnyPublish() {
  beginScenario("DeviceBusHardware::odometer() wiring is sane before the fiber ever runs");

  msg::MotorConfig configs[Subsystems::DeviceBusHardware::kMotorCount];
  for (uint32_t i = 0; i < Subsystems::DeviceBusHardware::kMotorCount; ++i) {
    configs[i] = sampleMsgMotorConfig(i + 1);
  }
  Subsystems::DeviceBusHardware hardware(configs, Config::OtosBootConfig());

  Hal::Odometer* odometer = hardware.odometer();
  checkTrue(odometer != nullptr, "odometer() never returns null");
  checkTrue(odometer->present(), "present() reports true -- a device slot is always wired (see file header)");
  checkFalse(odometer->connected(), "connected() is false before the fiber ever runs a real OTOS read");
  checkFalse(odometer->fusableThisPass(),
             "fusableThisPass() is false before any ring publish -- stamp 0 == lastFusedStamp_ 0");
  checkFalse(odometer->pose().stamp.valid, "pose() reports stamp.valid=false before any ring publish");
}

}  // namespace

int main() {
  scenarioDeviceBusMotorConfigToMsgRoundTrips();
  scenarioMsgToDeviceBusMotorConfigRoundTrips();
  scenarioOtosBootConfigToDeviceBusCopiesEveryField();
  scenarioDeviceBusPoseToEstimateNeverPublishedIsInvalid();
  scenarioDeviceBusPoseToEstimateConvertsFreshSample();
  scenarioMsgNeutralToDeviceBusMapsByName();

  scenarioMotorConfigReportsBootSnapshotPerPort();
  scenarioMotorCapabilitiesAreFixedDifferential();
  scenarioApplyRejectsUnsupportedVoltageMode();
  scenarioApplyVelocityThenNeutralTogglesActive();
  scenarioMotorStartsDisconnectedBeforeAnyPublish();
  scenarioOdometerDefaultsBeforeAnyPublish();

  if (g_failureCount == 0) {
    std::printf("OK: all device_bus_hardware bridge scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the device_bus_hardware bridge scenarios\n",
              g_failureCount);
  return 1;
}
