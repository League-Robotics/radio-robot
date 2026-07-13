// drivetrain_harness.cpp — off-hardware acceptance harness for ticket
// 100-007 (THE CUTOVER): exercises the REWRITTEN Subsystems::Drivetrain --
// now the thin wafer adapter over source/drive/, holding a Drive::Drivetrain
// + Drive::MotionPlan + an 8-slot Drive::Goal ring instead of a Motion::
// SegmentExecutor -- against a REAL Subsystems::SimHardware plant for the
// ring/plan/escape-hatch scenarios, and against a REAL Subsystems::
// NezhaHardware + the HOST_BUILD scripted I2CBus fake for the sprint's
// mandatory staging-only verification. Supersedes the pre-cutover 094-004
// harness (git history has the Motion::Segment-shaped version).
//
// Mirrors segment_executor_harness.cpp's/nezha_flipflop_harness.cpp's own
// shape: #includes the real headers (no mocks beyond the confined,
// sanctioned HOST_BUILD I2CBus fake), links against the real .cpp sources
// (drivetrain.cpp, sim_hardware.cpp, nezha_hardware.cpp, nezha_motor.cpp,
// body_kinematics.cpp, source/drive/*.cpp, hal/sim/*.cpp,
// hal/velocity_pid.cpp, com/i2c_bus_host.cpp, plus vendored Ruckig),
// compiles with the plain system C++ compiler under -DHOST_BUILD=1.
// Hand-rolled assertions, prints PASS/FAIL, exits nonzero on any failure.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "com/i2c_bus.h"
#include "drive/drivetrain.h"
#include "hal/nezha/nezha_motor.h"
#include "kinematics/body_kinematics.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "runtime/queue.h"
#include "subsystems/drivetrain.h"
#include "subsystems/nezha_hardware.h"
#include "subsystems/sim_hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors drivetrain_harness.cpp's
// pre-100-007 shape / segment_executor_harness.cpp / nezha_flipflop_harness.cpp) ---

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
  // Gains calibrated to the sim plant (2026-07-11) -- mirrors
  // tests/_infra/sim/sim_api.cpp defaultMotorConfigSet()'s own fix: the
  // plant is exactly linear (vel = duty * kNominalMaxSpeed), so kff =
  // 1/kNominalMaxSpeed is the exact feed-forward; the previous 0.0038
  // overdrove every wheel ~1.25x its setpoint.
  msg::Gains velGains;
  velGains.kp = 0.0005f;
  velGains.ki = 0.0005f;
  velGains.kff = 1.0f / Hal::PhysicsWorld::kNominalMaxSpeed;   // = 0.0025
  velGains.i_max = 0.3f;
  for (uint32_t i = 0; i < kMotorCount; ++i) {
    set.cfg[i] = msg::MotorConfig();
    set.cfg[i].setPort(i + 1);
    set.cfg[i].setFwdSign(1);
    set.cfg[i].setVelGains(velGains);
    set.cfg[i].setVelFiltAlpha(1.0f);
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

// generousMotionConfig -- (100-007) now ALSO populates PlannerConfig's
// Drive::Limits fields (v_wheel_max/trim_v_max/trim_omega_max/
// wheel_step_max/track_k_s/track_k_theta/track_k_cross/min_speed) --
// drive_bridge.h's driveLimitsFromConfig() reads these directly, and a
// v_wheel_max/min_speed left at their 0.0f struct default makes EVERY
// Drive::Drivetrain::plan() call fail (CEILING_INFEASIBLE, wheelBudget =
// vWheelMax - headroom <= 0) or misclassify every pivot as arc mode
// (fabsf(0.0f) < 0.0f is false -- see gen_boot_config.py's own
// MIN_SPEED_DEFAULT comment for the full derivation). Values mirror
// tests/_infra/sim/sim_api.cpp's own defaultSimMotionConfig() Drive::Limits
// block.
msg::PlannerConfig generousMotionConfig() {
  msg::PlannerConfig cfg;
  cfg.a_max = 1000.0f;
  cfg.a_decel = 1000.0f;
  cfg.v_body_max = 400.0f;
  cfg.yaw_rate_max = 3.0f;
  cfg.yaw_acc_max = 15.0f;
  cfg.j_max = 0.0f;       // trapezoid -- exercised elsewhere (test_jerk_trajectory.py)
  cfg.yaw_jerk_max = 0.0f;
  cfg.v_wheel_max = 350.0f;      // [mm/s]
  cfg.wheel_step_max = 150.0f;   // [mm/s]
  cfg.track_k_s = 2.0f;          // [1/s]
  cfg.track_k_theta = 6.0f;      // [1/s]
  cfg.track_k_cross = 1.5e-5f;   // [rad/mm^2]
  cfg.trim_v_max = 60.0f;        // [mm/s]
  cfg.trim_omega_max = 1.0f;     // [rad/s]
  cfg.min_speed = 10.0f;         // [mm/s]
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

// Shared REPLACE mailbox (MOVER) + zero poseStep/chainTail -- unused by
// these scenarios (no delayed-pose-fix or cross-goal admission behavior to
// prove here), but the tick() signature requires them (mirrors
// bb.replaceIn/bb.poseStepped/bb.chainTail).
Rt::Mailbox<Drive::Goal> g_replaceIn;
const msg::PoseStep g_zeroPoseStep{};
Drive::ChainTail g_chainTail{};
// Fixed placeholder BodyState -- scenario 4's own NezhaHardware+I2CBus-fake
// setup has no Hal::PhysicsWorld/.plant() to read ground truth from, and
// never posts a Drive::Goal at all (DIRECT/WHEELS-mode only, no Drive::
// tracking to feed), so a fixed value is fine there.
const msg::PoseEstimate g_zeroBodyState{};

// groundTruthBodyState -- this bare harness has no Subsystems::PoseEstimator
// wired up (unlike tests/_infra/sim/sim_api.cpp's SimHandle, which ticks a
// real one every pass and publishes bb.bodyState from it -- the production
// shape). source/drive/ is stateless and pose-free BY DESIGN (motion_plan.h's
// own "pose ownership is OUTSIDE the subsystem" rule) -- it needs a
// GENUINELY ADVANCING measured pose to track against, unlike the retired
// Motion::SegmentExecutor (encoder-arc-integrated internally, no external
// pose input at all). Reading Hal::PhysicsWorld's own TRUE pose/velocity
// directly (Subsystems::SimHardware::plant()) is the simplest faithful
// stand-in for "PoseEstimator perfectly tracks truth" in an isolated,
// PoseEstimator-free harness -- exactly the same ground-truth accessors
// tests/_infra/sim/sim_api.cpp's own sim_get_true_pose_x/y/h()/
// sim_get_true_vel_l/r() expose for the SAME reason.
msg::PoseEstimate groundTruthBodyState(Subsystems::SimHardware& hardware, float trackwidth) {
  msg::PoseEstimate bodyState;
  bodyState.pose.x = hardware.plant().truePoseX();
  bodyState.pose.y = hardware.plant().truePoseY();
  bodyState.pose.h = hardware.plant().truePoseH();
  BodyKinematics::forward(hardware.plant().trueVelL(), hardware.plant().trueVelR(), trackwidth,
                           bodyState.twist.v_x, bodyState.twist.omega);
  bodyState.twist.v_y = 0.0f;
  return bodyState;
}

// runPasses -- ticks `hardware` then `dt` `n` times at a fixed 20ms cadence,
// starting from `*now`, mirroring the bare loop's own ordering (hardware
// FIRST, so a setpoint staged last pass flushes this pass -- see
// main_loop.cpp/main.cpp). Advances `*now` in place.
void runPasses(Subsystems::SimHardware& hardware, Subsystems::Drivetrain& dt,
              Rt::WorkQueue<Drive::Goal, 8>& segmentIn,
              Rt::WorkQueue<msg::DrivetrainCommand, 8>& driveIn, float trackwidth,
              uint32_t* now, int n) {
  for (int i = 0; i < n; ++i) {
    *now += 20;
    hardware.tick(*now);
    dt.tick(*now, segmentIn, g_replaceIn, driveIn, groundTruthBodyState(hardware, trackwidth),
            g_zeroPoseStep, g_chainTail);
  }
}

// --- Scenarios (SimHardware-backed) ---

// 1. Single-Goal enqueue -> plan -> execute -> pop: a straight Goal posted
// to segmentIn is drained into the ring, planned+executed by the held
// Drive::Drivetrain/MotionPlan, and the plant's average encoder travel
// converges on the commanded distance.
void scenarioSingleGoalEnqueueExecutePop() {
  beginScenario("single Goal: enqueue via segmentIn, plans+executes, average encoder converges");
  MotorConfigSet motorConfigs = defaultMotorConfigSet();
  Subsystems::SimHardware hardware(motorConfigs.cfg);
  hardware.begin();
  Subsystems::Drivetrain dt(hardware);
  dt.configure(defaultDrivetrainConfig());
  dt.configureMotion(generousMotionConfig());

  Rt::WorkQueue<Drive::Goal, 8> segmentIn;
  Rt::WorkQueue<msg::DrivetrainCommand, 8> driveIn;

  Drive::Goal goal;
  goal.arcLength = 300.0f;   // [mm]
  checkTrue(segmentIn.post(goal), "segmentIn accepts the posted Goal");

  uint32_t now = 0;
  runPasses(hardware, dt, segmentIn, driveIn, Hal::PhysicsWorld::kDefaultTrackwidth, &now, 400);   // 8s -- ample settle time

  msg::DrivetrainState s = dt.state();
  float avg = (s.enc()[0] + s.enc()[1]) * 0.5f;
  checkTrue(std::fabs(avg - 300.0f) < 15.0f, "average encoder travel converges near 300mm");
  checkTrue(std::fabs(s.vel()[0]) < 10.0f && std::fabs(s.vel()[1]) < 10.0f,
            "measured velocity has settled back near zero -- the segment converged and idled");
}

// 2. Escape-hatch preemption: `S` (WHEELS) posted mid-plan via driveIn
// clears the ring IMMEDIATELY -- the plant's steady-state velocity ends up
// matching the DIRECT WHEELS target, never the (much larger, still
// in-flight) segment's cruise target.
void scenarioEscapeHatchPreemptionClearsRingImmediately() {
  beginScenario("S mid-plan: escape hatch preempts, plant settles to the DIRECT target");
  MotorConfigSet motorConfigs = defaultMotorConfigSet();
  Subsystems::SimHardware hardware(motorConfigs.cfg);
  hardware.begin();
  Subsystems::Drivetrain dt(hardware);
  dt.configure(defaultDrivetrainConfig());
  dt.configureMotion(generousMotionConfig());

  Rt::WorkQueue<Drive::Goal, 8> segmentIn;
  Rt::WorkQueue<msg::DrivetrainCommand, 8> driveIn;

  Drive::Goal goal;
  goal.arcLength = 5000.0f;   // [mm] -- deliberately long; must never complete in this scenario
  checkTrue(segmentIn.post(goal), "segmentIn accepts the long Goal");

  uint32_t now = 0;
  runPasses(hardware, dt, segmentIn, driveIn, Hal::PhysicsWorld::kDefaultTrackwidth, &now, 50);   // 1s -- underway, well short of 5000mm

  msg::DrivetrainState mid = dt.state();
  checkTrue(mid.vel()[0] > 20.0f && mid.vel()[1] > 20.0f,
            "precondition: the plan is genuinely driving forward before preemption");

  checkTrue(driveIn.post(wheelsCommand(60.0f, -60.0f)),
            "driveIn accepts the escape-hatch WHEELS command (a spin -- distinct sign "
            "pattern from the segment's straight-line drive)");

  runPasses(hardware, dt, segmentIn, driveIn, Hal::PhysicsWorld::kDefaultTrackwidth, &now, 150);   // 3s -- settle onto the DIRECT target

  msg::DrivetrainState after = dt.state();
  checkTrue(after.vel()[0] > 20.0f, "left wheel settles positive -- the DIRECT WHEELS target, "
                                    "not the abandoned plan's straight-drive target");
  checkTrue(after.vel()[1] < -20.0f, "right wheel settles negative -- proves the escape hatch "
                                     "preempted the ring (a straight arc never commands "
                                     "opposite-signed wheels)");
}

// 3. STOP (NEUTRAL) mid-plan preempts INSTANTLY (100-007's own documented
// deviation from the pre-cutover graceful decel-to-zero -- see drivetrain.h's
// class comment): the measured velocity still decays toward zero via the
// PLANT's own inertia/velocity-PID response to a commanded 0.0f and never
// reverses sign (no reverse-creep), even though the MECHANISM is now an
// instant target change rather than a presolved graceful ramp.
void scenarioStopMidPlanInstantPreemptNoReverseCreep() {
  beginScenario("STOP mid-plan: instant preempt, measured velocity never reverses sign");
  MotorConfigSet motorConfigs = defaultMotorConfigSet();
  Subsystems::SimHardware hardware(motorConfigs.cfg);
  hardware.begin();
  Subsystems::Drivetrain dt(hardware);
  dt.configure(defaultDrivetrainConfig());
  dt.configureMotion(generousMotionConfig());

  Rt::WorkQueue<Drive::Goal, 8> segmentIn;
  Rt::WorkQueue<msg::DrivetrainCommand, 8> driveIn;

  Drive::Goal goal;
  goal.arcLength = 2000.0f;   // [mm] -- long enough that STOP fires well before natural completion
  checkTrue(segmentIn.post(goal), "segmentIn accepts the Goal");

  uint32_t now = 0;
  runPasses(hardware, dt, segmentIn, driveIn, Hal::PhysicsWorld::kDefaultTrackwidth, &now, 50);   // 1s -- underway

  msg::DrivetrainState mid = dt.state();
  checkTrue(mid.vel()[0] > 20.0f && mid.vel()[1] > 20.0f,
            "precondition: genuinely driving forward before STOP");

  checkTrue(driveIn.post(neutralCommand()), "driveIn accepts NEUTRAL (STOP)");

  bool everNegative = false;
  float minVel = 1e9f;
  for (int i = 0; i < 250; ++i) {   // up to 5s to settle
    now += 20;
    hardware.tick(now);
    dt.tick(now, segmentIn, g_replaceIn, driveIn,
            groundTruthBodyState(hardware, Hal::PhysicsWorld::kDefaultTrackwidth), g_zeroPoseStep,
            g_chainTail);
    msg::DrivetrainState s = dt.state();
    float v = (s.vel()[0] + s.vel()[1]) * 0.5f;
    if (v < minVel) minVel = v;
    // A small negative floor absorbs PID/plant settle noise around a
    // literal-0.0f commanded twist.
    if (v < -5.0f) everNegative = true;
  }

  checkTrue(!everNegative, "measured velocity never reverses sign while decelerating to STOP");
  msg::DrivetrainState final = dt.state();
  checkTrue(std::fabs(final.vel()[0]) < 10.0f && std::fabs(final.vel()[1]) < 10.0f,
            "measured velocity settles near zero after STOP");
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

  Rt::WorkQueue<Drive::Goal, 8> segmentIn;
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
  dt.tick(1000, segmentIn, g_replaceIn, driveIn, g_zeroBodyState, g_zeroPoseStep, g_chainTail);
  checkUintEq(bus.txnCount(kAddr7), before,
              "Drivetrain::tick() (stage-only) issued ZERO I2C transactions");

  // Only an EXPLICIT hardware.tick() (the flip-flop's own REQUEST_DUE/
  // COLLECT_DUE schedule) ever touches the bus.
  hardware.tick(1010);   // REQUEST_DUE
  checkTrue(bus.txnCount(kAddr7) > before,
            "an explicit hardware.tick() call is what finally issues the I2C transaction");
}

}  // namespace

int main() {
  scenarioSingleGoalEnqueueExecutePop();
  scenarioEscapeHatchPreemptionClearsRingImmediately();
  scenarioStopMidPlanInstantPreemptNoReverseCreep();
  scenarioStagingOnlyNoI2CWriteUntilExplicitHardwareTick();

  if (g_failureCount == 0) {
    std::printf("OK: all Drivetrain 100-007 scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Drivetrain 100-007 scenarios\n",
              g_failureCount);
  return 1;
}
