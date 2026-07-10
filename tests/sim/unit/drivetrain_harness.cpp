// drivetrain_harness.cpp — off-hardware acceptance harness for ticket
// 094-004 (SUC-001/SUC-002/SUC-004): exercises the REWRITTEN
// Subsystems::Drivetrain -- now holding a Hardware& and owning a
// Motion::SegmentExecutor + an 8-slot segment ring -- against a REAL
// Subsystems::SimHardware plant (the ticket's own "test double or a real
// SimHardware instance" testing-plan note) for the ring/executor/
// escape-hatch/graceful-stop scenarios, and against a REAL
// Subsystems::NezhaHardware + the HOST_BUILD scripted I2CBus fake for the
// sprint's mandatory staging-only verification.
//
// Mirrors segment_executor_harness.cpp's/nezha_flipflop_harness.cpp's own
// shape: #includes the real headers (no mocks beyond the confined,
// sanctioned HOST_BUILD I2CBus fake), links against the real .cpp sources
// (drivetrain.cpp, sim_hardware.cpp, nezha_hardware.cpp, nezha_motor.cpp,
// body_kinematics.cpp, motion/{segment_executor,jerk_trajectory,
// stop_condition}.cpp, hal/sim/*.cpp, hal/velocity_pid.cpp,
// com/i2c_bus_host.cpp, plus vendored Ruckig), compiles with the plain
// system C++ compiler under -DHOST_BUILD=1 (segment_executor.cpp's
// kDeadTime compile split resolves to the sim value). Hand-rolled
// assertions, prints PASS/FAIL, exits nonzero on any failure.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "com/i2c_bus.h"
#include "hal/nezha/nezha_motor.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "motion/segment.h"
#include "runtime/queue.h"
#include "subsystems/drivetrain.h"
#include "subsystems/nezha_hardware.h"
#include "subsystems/sim_hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors drivetrain_harness.cpp's
// pre-094 shape / segment_executor_harness.cpp / nezha_flipflop_harness.cpp) ---

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

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

// --- SimHardware fixture helpers (mirrors sim_api.cpp's
// defaultMotorConfigSet()/defaultSimDrivetrainConfig() -- the same sane,
// self-contained sim defaults, kept independent so this harness needs no
// dependency on sim_api.cpp itself). ---

constexpr uint32_t kMotorCount = Subsystems::Hardware::kMotorCount;

struct MotorConfigSet {
  msg::MotorConfig cfg[kMotorCount];
};

MotorConfigSet defaultMotorConfigSet() {
  MotorConfigSet set;
  msg::Gains velGains;
  velGains.kp = 0.0022f;
  velGains.ki = 0.0018f;
  velGains.kff = 0.0038f;
  velGains.i_max = 0.3f;
  for (uint32_t i = 0; i < kMotorCount; ++i) {
    set.cfg[i] = msg::MotorConfig();
    set.cfg[i].setPort(i + 1);
    set.cfg[i].setFwdSign(1);
    set.cfg[i].setVelGains(velGains);
    set.cfg[i].setVelFiltAlpha(0.3f);
    set.cfg[i].setPolled(i + 1 == 1 || i + 1 == 2);
  }
  return set;
}

msg::DrivetrainConfig defaultDrivetrainConfig() {
  msg::DrivetrainConfig cfg;
  cfg.setTrackwidth(Hal::PhysicsWorld::kDefaultTrackwidth);
  cfg.setLeftPort(1);
  cfg.setRightPort(2);
  return cfg;
}

msg::PlannerConfig generousMotionConfig() {
  msg::PlannerConfig cfg;
  cfg.a_max = 1000.0f;
  cfg.a_decel = 1000.0f;
  cfg.v_body_max = 400.0f;
  cfg.yaw_rate_max = 3.0f;
  cfg.yaw_acc_max = 15.0f;
  cfg.j_max = 0.0f;       // trapezoid -- exercised elsewhere (test_jerk_trajectory.py)
  cfg.yaw_jerk_max = 0.0f;
  return cfg;
}

msg::DrivetrainCommand wheelsCommand(float left, float right) {
  msg::WheelTargets wt;
  wt.w_count = 2;
  wt.w_[0].speed.has = true;
  wt.w_[0].speed.val = left;
  wt.w_[1].speed.has = true;
  wt.w_[1].speed.val = right;
  msg::DrivetrainCommand cmd;
  cmd.setWheels(wt);
  return cmd;
}

msg::DrivetrainCommand neutralCommand() {
  msg::DrivetrainCommand cmd;
  cmd.setNeutral(msg::Neutral::BRAKE);
  return cmd;
}

// runPasses -- ticks `hardware` then `dt` `n` times at a fixed 20ms cadence,
// starting from `*now`, mirroring the bare loop's own ordering (hardware
// FIRST, so a setpoint staged last pass flushes this pass -- see
// main_loop.cpp/main.cpp). Advances `*now` in place.
// Shared REPLACE mailbox (MOVER) -- unused by these scenarios, but the
// tick() signature requires it (mirrors bb.replaceIn).
Rt::Mailbox<Motion::Segment> g_replaceIn;

void runPasses(Subsystems::SimHardware& hardware, Subsystems::Drivetrain& dt,
              Rt::WorkQueue<Motion::Segment, 8>& segmentIn,
              Rt::WorkQueue<msg::DrivetrainCommand, 8>& driveIn, uint32_t* now, int n) {
  for (int i = 0; i < n; ++i) {
    *now += 20;
    hardware.tick(*now);
    dt.tick(*now, segmentIn, g_replaceIn, driveIn);
  }
}

// --- Scenarios (SimHardware-backed) ---

// 1. Single-segment enqueue -> execute -> pop: a straight segment posted to
// segmentIn is drained into the ring, executed by the owned executor, and
// the plant's average encoder travel converges on the commanded distance.
void scenarioSingleSegmentEnqueueExecutePop() {
  beginScenario("single segment: enqueue via segmentIn, executes, average encoder converges");
  MotorConfigSet motorConfigs = defaultMotorConfigSet();
  Subsystems::SimHardware hardware(motorConfigs.cfg);
  hardware.begin();
  Subsystems::Drivetrain dt(hardware);
  dt.configure(defaultDrivetrainConfig());
  dt.configureMotion(generousMotionConfig());

  Rt::WorkQueue<Motion::Segment, 8> segmentIn;
  Rt::WorkQueue<msg::DrivetrainCommand, 8> driveIn;

  Motion::Segment seg;
  seg.distance = 300.0f;   // [mm]
  checkTrue(segmentIn.post(seg), "segmentIn accepts the posted segment");

  uint32_t now = 0;
  runPasses(hardware, dt, segmentIn, driveIn, &now, 400);   // 8s -- ample settle time

  msg::DrivetrainState s = dt.state();
  float avg = (s.enc()[0] + s.enc()[1]) * 0.5f;
  checkTrue(std::fabs(avg - 300.0f) < 15.0f, "average encoder travel converges near 300mm");
  checkTrue(std::fabs(s.vel()[0]) < 10.0f && std::fabs(s.vel()[1]) < 10.0f,
            "measured velocity has settled back near zero -- the segment converged and idled");
}

// 2. Escape-hatch preemption: `S` (WHEELS) posted mid-segment via driveIn
// clears the ring IMMEDIATELY -- the plant's steady-state velocity ends up
// matching the DIRECT WHEELS target, never the (much larger, still
// in-flight) segment's cruise target.
void scenarioEscapeHatchPreemptionClearsRingImmediately() {
  beginScenario("S mid-segment: escape hatch preempts, plant settles to the DIRECT target");
  MotorConfigSet motorConfigs = defaultMotorConfigSet();
  Subsystems::SimHardware hardware(motorConfigs.cfg);
  hardware.begin();
  Subsystems::Drivetrain dt(hardware);
  dt.configure(defaultDrivetrainConfig());
  dt.configureMotion(generousMotionConfig());

  Rt::WorkQueue<Motion::Segment, 8> segmentIn;
  Rt::WorkQueue<msg::DrivetrainCommand, 8> driveIn;

  Motion::Segment seg;
  seg.distance = 5000.0f;   // [mm] -- deliberately long; must never complete in this scenario
  checkTrue(segmentIn.post(seg), "segmentIn accepts the long segment");

  uint32_t now = 0;
  runPasses(hardware, dt, segmentIn, driveIn, &now, 50);   // 1s -- segment underway, well short of 5000mm

  msg::DrivetrainState mid = dt.state();
  checkTrue(mid.vel()[0] > 20.0f && mid.vel()[1] > 20.0f,
            "precondition: the segment is genuinely driving forward before preemption");

  checkTrue(driveIn.post(wheelsCommand(60.0f, -60.0f)),
            "driveIn accepts the escape-hatch WHEELS command (a spin -- distinct sign "
            "pattern from the segment's straight-line drive)");

  runPasses(hardware, dt, segmentIn, driveIn, &now, 150);   // 3s -- settle onto the DIRECT target

  msg::DrivetrainState after = dt.state();
  checkTrue(after.vel()[0] > 20.0f, "left wheel settles positive -- the DIRECT WHEELS target, "
                                    "not the abandoned segment's straight-drive target");
  checkTrue(after.vel()[1] < -20.0f, "right wheel settles negative -- proves the escape hatch "
                                     "preempted the ring (a straight segment never commands "
                                     "opposite-signed wheels)");
}

// 3. STOP (NEUTRAL) mid-segment triggers the executor's OWN graceful
// decel-to-zero -- the measured velocity decays toward zero and never
// reverses sign (no reverse-creep), rather than an instant zero-velocity
// command.
void scenarioStopMidSegmentGracefulDecelNoReverseCreep() {
  beginScenario("STOP mid-segment: graceful decel-to-zero, measured velocity never reverses sign");
  MotorConfigSet motorConfigs = defaultMotorConfigSet();
  Subsystems::SimHardware hardware(motorConfigs.cfg);
  hardware.begin();
  Subsystems::Drivetrain dt(hardware);
  dt.configure(defaultDrivetrainConfig());
  dt.configureMotion(generousMotionConfig());

  Rt::WorkQueue<Motion::Segment, 8> segmentIn;
  Rt::WorkQueue<msg::DrivetrainCommand, 8> driveIn;

  Motion::Segment seg;
  seg.distance = 2000.0f;   // [mm] -- long enough that STOP fires well before natural completion
  checkTrue(segmentIn.post(seg), "segmentIn accepts the segment");

  uint32_t now = 0;
  runPasses(hardware, dt, segmentIn, driveIn, &now, 50);   // 1s -- underway

  msg::DrivetrainState mid = dt.state();
  checkTrue(mid.vel()[0] > 20.0f && mid.vel()[1] > 20.0f,
            "precondition: genuinely driving forward before STOP");

  checkTrue(driveIn.post(neutralCommand()), "driveIn accepts NEUTRAL (STOP)");

  bool everNegative = false;
  float minVel = 1e9f;
  for (int i = 0; i < 250; ++i) {   // up to 5s to settle
    now += 20;
    hardware.tick(now);
    dt.tick(now, segmentIn, g_replaceIn, driveIn);
    msg::DrivetrainState s = dt.state();
    float v = (s.vel()[0] + s.vel()[1]) * 0.5f;
    if (v < minVel) minVel = v;
    // A small negative floor absorbs PID/plant settle noise around a
    // literal-0.0f commanded twist -- the ACTUAL no-reverse-creep contract
    // (segment_executor.h's own literal-0.0f snap) is proven at the
    // Motion::SegmentExecutor level by segment_executor_harness.cpp's own
    // regression scenario; this measured-plant check is the integration
    // proof that STOP reaches the executor's graceful path at all (not an
    // instant zero) rather than a bit-exact re-proof of the snap.
    if (v < -5.0f) everNegative = true;
  }

  checkTrue(!everNegative, "measured velocity never reverses sign while decelerating to STOP");
  msg::DrivetrainState final = dt.state();
  checkTrue(std::fabs(final.vel()[0]) < 10.0f && std::fabs(final.vel()[1]) < 10.0f,
            "measured velocity settles near zero after STOP's graceful decel");
}

// --- Scenario 4 (NezhaHardware + HOST_BUILD scripted I2CBus fake): the
// sprint's mandatory staging-only verification. ---

constexpr uint16_t kAddr7 = 0x10;
constexpr uint16_t kWireAddr = static_cast<uint16_t>(kAddr7 << 1);

void scriptGenerousPool(I2CBus& bus, int count) {
  static uint8_t canned[4] = {0, 0, 0, 0};
  for (int i = 0; i < count; ++i) {
    bus.scriptWrite(kWireAddr, /*status=*/0);
    bus.scriptRead(kWireAddr, canned, 4, /*status=*/0);
  }
}

void scenarioStagingOnlyNoI2CWriteUntilExplicitHardwareTick() {
  beginScenario(
      "staging-only: Drivetrain::tick() alone issues ZERO I2C writes -- only an explicit "
      "hardware.tick() flushes them");
  msg::MotorConfig configs[Subsystems::NezhaHardware::kMotorCount];
  for (uint32_t i = 0; i < Subsystems::NezhaHardware::kMotorCount; ++i) {
    configs[i] = msg::MotorConfig();
    configs[i].setPort(i + 1).setFwdSign(1).setTravelCalib(1.0f);
    configs[i].setPolled(i + 1 == 1 || i + 1 == 2);   // the drive pair
  }

  I2CBus::setClock(1000000);
  I2CBus bus;
  Subsystems::NezhaHardware hardware(bus, configs);
  scriptGenerousPool(bus, 40);

  Subsystems::Drivetrain dt(hardware);
  msg::DrivetrainConfig dtConfig;
  dtConfig.setTrackwidth(150.0f);
  dtConfig.setLeftPort(1);
  dtConfig.setRightPort(2);
  dt.configure(dtConfig);
  dt.configureMotion(generousMotionConfig());

  Rt::WorkQueue<Motion::Segment, 8> segmentIn;
  Rt::WorkQueue<msg::DrivetrainCommand, 8> driveIn;
  checkTrue(driveIn.post(wheelsCommand(150.0f, 150.0f)), "driveIn accepts the WHEELS command");

  uint32_t before = bus.txnCount(kAddr7);
  checkUintEq(before, 0, "precondition: no I2C traffic before any tick() at all");

  // Drivetrain::tick() alone -- drains driveIn, computes the governed
  // targets, and STAGES them via hardware.motor(port).apply(cmd)
  // (Hal::Motor::apply() -> the leaf's setVelocity(), itself staging-only:
  // NezhaMotor::setVelocity() only writes mode_/velocityTarget_, per
  // motor.h/nezha_motor.cpp's own contract). This must issue NO bus
  // transaction of any kind.
  dt.tick(1000, segmentIn, g_replaceIn, driveIn);
  checkUintEq(bus.txnCount(kAddr7), before,
              "Drivetrain::tick() (stage-only) issued ZERO I2C transactions");

  // Only an EXPLICIT hardware.tick() (the flip-flop's own REQUEST_DUE/
  // COLLECT_DUE schedule, unchanged by this sprint -- Subsystems::Hardware
  // keeps the name tick(), the 094-003 serviceBus() rename was dropped in
  // harmonization) ever touches the bus.
  hardware.tick(1010);   // REQUEST_DUE
  checkTrue(bus.txnCount(kAddr7) > before,
            "an explicit hardware.tick() call is what finally issues the I2C transaction");
}

}  // namespace

int main() {
  scenarioSingleSegmentEnqueueExecutePop();
  scenarioEscapeHatchPreemptionClearsRingImmediately();
  scenarioStopMidSegmentGracefulDecelNoReverseCreep();
  scenarioStagingOnlyNoI2CWriteUntilExplicitHardwareTick();

  if (g_failureCount == 0) {
    std::printf("OK: all Drivetrain 094-004 scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Drivetrain 094-004 scenarios\n",
              g_failureCount);
  return 1;
}
