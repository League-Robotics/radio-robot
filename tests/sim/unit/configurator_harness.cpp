// configurator_harness.cpp — off-hardware acceptance harness for ticket
// 087-005 (SUC-002/SUC-003/SUC-005): exercises Rt::Configurator
// (source/runtime/configurator.{h,cpp}) against REAL (not mocked)
// Subsystems::Drivetrain/PoseEstimator/Planner/SimHardware instances plus a
// real Rt::Blackboard — no fakes, per the ticket's own testability goal.
//
// Compiles with the plain system C++ compiler (-DHOST_BUILD, needed for
// SimHardware/PhysicsWorld's std::mt19937 members — see
// test_sim_hardware.py's own precedent) together with every REAL source it
// exercises: subsystems/{drivetrain,pose_estimator,planner,sim_hardware}.cpp,
// kinematics/body_kinematics.cpp, estimation/ekf_tiny.cpp,
// motion/{velocity_ramp,stop_condition}.cpp, hal/sim/{physics_world,
// sim_motor,sim_odometer}.cpp, hal/velocity_pid.cpp, and
// runtime/configurator.cpp itself — no CMake, no ARM toolchain. Hand-rolled
// assertions, prints PASS/FAIL, exits nonzero on any failure. Run by
// test_configurator.py, which compiles and runs this binary via subprocess.
//
// Each scenario constructs its own local Drivetrain/PoseEstimator/Planner/
// SimHardware/Blackboard set (cheap, stack-allocated) rather than sharing a
// fixture — mirrors this project's other *_harness.cpp files (e.g.
// sim_hardware_harness.cpp), avoiding any member-initializer-order
// trickiness around SimHardware's constructor argument.
//
// Scenarios (see ticket 087-005's Acceptance Criteria):
//   1. kDrivetrain delta -> Drivetrain::configure() called, bb.drivetrainConfig
//      published with the new value (Drivetrain has no config() getter —
//      087-004's own faceplate shape — so the published bb cell IS the
//      design's replacement for a per-subsystem getter).
//   2. kMotor delta (one port) -> Hardware::config(port) (087-004's getter)
//      AND bb.motorConfig[port-1] both reflect the change; the other port
//      is unaffected.
//   3. kPlanner delta -> bb.plannerConfig published (Planner has no
//      config() getter either).
//   4. kOdometer delta on SimHardware (which HAS a real Hal::SimOdometer
//      via odometer()) -> bb.odometerConfig published.
//   5. publish(bb) before any delta seeds all four bb.*Config cells from
//      the boot configs / Hardware::config(port) read-back.
//   6. pending(bb) mirrors bb.configIn.empty() exactly.
//   7. Two deltas for the SAME target (kDrivetrain), touching DIFFERENT
//      fields, applied in FIFO order via two applyOne() calls — the second
//      delta's fold must NOT clobber the first delta's field (AC-7's
//      "field-masked, not full-replace" proof) — built as though from the
//      SAME stale baseline, proving the CONFIGURATOR's own fold (not the
//      caller's baseline discipline) is what prevents the clobber.
//   8. applyOne() pops AT MOST one delta per call — two queued deltas need
//      two calls to both apply.
//   9. applyOne() on an empty configIn is a well-defined no-op.

#include <cstdint>
#include <cstdio>
#include <string>

#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/odometer.h"
#include "messages/planner.h"
#include "runtime/blackboard.h"
#include "runtime/commands.h"
#include "runtime/configurator.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/planner.h"
#include "subsystems/pose_estimator.h"
#include "subsystems/sim_hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors sim_hardware_harness.cpp /
// runtime_blackboard_harness.cpp) ---

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

void checkFloatEq(float actual, float expected, const std::string& what) {
  if (actual != expected) {
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

// fillDefaultConfigs -- mirrors sim_hardware_harness.cpp's own helper
// exactly (SimHardware's constructor contract: configs[i].port == i+1).
void fillDefaultConfigs(msg::MotorConfig configs[Subsystems::Hardware::kPortCount]) {
  for (uint32_t i = 0; i < Subsystems::Hardware::kPortCount; ++i) {
    configs[i] = msg::MotorConfig{};
    configs[i].setPort(i + 1).setFwdSign(1).setTravelCalib(1.0f);
  }
}

// 1. kDrivetrain delta -> Drivetrain::configure() called, bb.drivetrainConfig
//    published with the new value.
void scenarioDrivetrainDeltaApplies() {
  beginScenario("kDrivetrain delta: folds, configures, publishes");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain;
  Subsystems::PoseEstimator poseEstimator;
  Subsystems::Planner planner;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, planner, hardware,
                                 msg::DrivetrainConfig{}, msg::PlannerConfig{});

  Rt::ConfigDelta delta;
  delta.target = Rt::ConfigDelta::kDrivetrain;
  delta.mask = Rt::bitOf(Rt::DrivetrainConfigField::kTrackwidth);
  delta.drivetrain.trackwidth = 199.0f;

  checkTrue(bb.configIn.post(delta), "post() succeeds");
  checkTrue(configurator.pending(bb), "pending() true before applyOne()");
  configurator.applyOne(bb);
  checkFalse(configurator.pending(bb), "pending() false after draining the one delta");

  checkFloatEq(bb.drivetrainConfig.trackwidth, 199.0f, "bb.drivetrainConfig.trackwidth published");
  // Drivetrain-scoped deltas re-propagate to PoseEstimator too (both share
  // msg::DrivetrainConfig -- see configurator.cpp); PoseEstimator::config()
  // (087-004) is the one target with a genuine LIVE (not boot-snapshot)
  // config getter, so this doubles as a direct "the target's own config()
  // now reflects the delta" proof (this ticket's AC-6 wording), not only
  // the published-cell proof above.
  checkFloatEq(poseEstimator.config().trackwidth, 199.0f,
               "PoseEstimator::config().trackwidth also updated (drivetrain-scoped re-propagation)");
}

// 2. kMotor delta (port 2) -> Hal::Motor::configure() called through
//    Hardware::motor(port); bb.motorConfig[1] reflects it; bb.motorConfig[0]
//    (untouched port) is unaffected.
//
// Note: Hardware::config(port) (087-004) is a boot-time snapshot only --
// ticket 087-004's own Implementation Notes confirm neither NezhaHardware
// nor SimHardware writes back into their config_[] cache after
// construction (there is no Hardware::configure() setter at all yet, only
// the per-motor Hal::Motor::configure() this Configurator calls through).
// It would therefore be WRONG to assert hardware.config(2) reflects a
// post-construction configure() call -- it never will, by the current
// codebase's own design. The published bb.motorConfig[] cell (populated
// from THIS Configurator's own persistent motorConfig_[] copy, which IS
// kept live) is the correct, and per architecture-update-r1.md's own
// framing ("Current config -- published by the Configurator on apply...
// Replaces every shadow"), the DESIGNATED evidence for "what is this
// port's current config" post-087.
void scenarioMotorDeltaApplies() {
  beginScenario("kMotor delta: folds, configures port 2, publishes");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain;
  Subsystems::PoseEstimator poseEstimator;
  Subsystems::Planner planner;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, planner, hardware,
                                 msg::DrivetrainConfig{}, msg::PlannerConfig{});

  Rt::ConfigDelta delta;
  delta.target = Rt::ConfigDelta::kMotor;
  delta.port = 2;
  delta.mask = Rt::bitOf(Rt::MotorConfigField::kSlewRate) |
               Rt::bitOf(Rt::MotorConfigField::kVelGainsKp);
  delta.motor.slew_rate = 555.0f;
  delta.motor.vel_gains.kp = 0.75f;

  checkTrue(bb.configIn.post(delta), "post() succeeds");
  configurator.applyOne(bb);

  checkFloatEq(bb.motorConfig[1].slew_rate, 555.0f, "bb.motorConfig[1].slew_rate published");
  checkFloatEq(bb.motorConfig[1].vel_gains.kp, 0.75f, "bb.motorConfig[1].vel_gains.kp published");
  // Every OTHER field of port 2's config must be untouched by the fold.
  checkFloatEq(bb.motorConfig[1].travel_calib, 1.0f,
               "bb.motorConfig[1].travel_calib (unmasked field) left at its boot default");

  // Untouched port (1, index 0) must be unaffected.
  checkFloatEq(bb.motorConfig[0].slew_rate, 0.0f, "port 1 (untouched) bb.motorConfig[0].slew_rate stays default");
}

// 3. kPlanner delta -> Planner::configure() called, bb.plannerConfig
//    published.
void scenarioPlannerDeltaApplies() {
  beginScenario("kPlanner delta: folds, configures, publishes");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain;
  Subsystems::PoseEstimator poseEstimator;
  Subsystems::Planner planner;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, planner, hardware,
                                 msg::DrivetrainConfig{}, msg::PlannerConfig{});

  Rt::ConfigDelta delta;
  delta.target = Rt::ConfigDelta::kPlanner;
  delta.mask = Rt::bitOf(Rt::PlannerConfigField::kMinSpeed);
  delta.planner.min_speed = 42.0f;

  checkTrue(bb.configIn.post(delta), "post() succeeds");
  configurator.applyOne(bb);

  checkFloatEq(bb.plannerConfig.min_speed, 42.0f, "bb.plannerConfig.min_speed published");
}

// 4. kOdometer delta on SimHardware (has a real Hal::SimOdometer via
//    odometer()) -> bb.odometerConfig published.
void scenarioOdometerDeltaApplies() {
  beginScenario("kOdometer delta: folds, publishes (odometer() present on SimHardware)");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain;
  Subsystems::PoseEstimator poseEstimator;
  Subsystems::Planner planner;
  Rt::Blackboard bb;

  checkTrue(hardware.odometer() != nullptr, "sanity: SimHardware::odometer() is non-null");

  Rt::Configurator configurator(drivetrain, poseEstimator, planner, hardware,
                                 msg::DrivetrainConfig{}, msg::PlannerConfig{});

  Rt::ConfigDelta delta;
  delta.target = Rt::ConfigDelta::kOdometer;
  delta.mask = Rt::bitOf(Rt::OdometerConfigField::kLinearScalar) |
               Rt::bitOf(Rt::OdometerConfigField::kAngularScalar);
  delta.odometer.linear_scalar = 1.5f;
  delta.odometer.angular_scalar = 0.9f;

  checkTrue(bb.configIn.post(delta), "post() succeeds");
  configurator.applyOne(bb);

  checkFloatEq(bb.odometerConfig.linear_scalar, 1.5f, "bb.odometerConfig.linear_scalar published");
  checkFloatEq(bb.odometerConfig.angular_scalar, 0.9f, "bb.odometerConfig.angular_scalar published");
}

// 5. publish(bb) before any delta seeds all four cells from boot
//    configs/Hardware::config(port) read-back, with no delta posted.
void scenarioPublishSeedsAllFourCells() {
  beginScenario("publish(bb): seeds all four cells at boot, no delta needed");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain;
  Subsystems::PoseEstimator poseEstimator;
  Subsystems::Planner planner;
  Rt::Blackboard bb;

  msg::DrivetrainConfig bootDt;
  bootDt.trackwidth = 128.0f;
  msg::PlannerConfig bootPlanner;
  bootPlanner.min_speed = 5.0f;

  Rt::Configurator configurator(drivetrain, poseEstimator, planner, hardware, bootDt, bootPlanner);

  checkTrue(bb.configIn.empty(), "sanity: configIn empty, publish() must not need a delta");
  configurator.publish(bb);

  checkFloatEq(bb.drivetrainConfig.trackwidth, 128.0f, "bb.drivetrainConfig seeded from boot config");
  checkFloatEq(bb.plannerConfig.min_speed, 5.0f, "bb.plannerConfig seeded from boot config");
  for (uint32_t port = 1; port <= Subsystems::Hardware::kPortCount; ++port) {
    char what[96];
    std::snprintf(what, sizeof(what), "bb.motorConfig[%u].travel_calib seeded from Hardware::config(port)",
                  static_cast<unsigned>(port - 1));
    checkFloatEq(bb.motorConfig[port - 1].travel_calib, 1.0f, what);
  }
  checkFloatEq(bb.odometerConfig.linear_scalar, 0.0f,
               "bb.odometerConfig seeded zero-default (no boot-config source, per otos_commands.h)");
}

// 6. pending(bb) mirrors bb.configIn.empty() exactly.
void scenarioPendingMirrorsConfigInEmpty() {
  beginScenario("pending(bb) mirrors !bb.configIn.empty()");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain;
  Subsystems::PoseEstimator poseEstimator;
  Subsystems::Planner planner;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, planner, hardware,
                                 msg::DrivetrainConfig{}, msg::PlannerConfig{});

  checkFalse(configurator.pending(bb), "pending() false on a fresh Blackboard");

  Rt::ConfigDelta delta;
  delta.target = Rt::ConfigDelta::kPlanner;
  checkTrue(bb.configIn.post(delta), "post() succeeds");
  checkTrue(configurator.pending(bb), "pending() true once a delta is posted");

  configurator.applyOne(bb);
  checkFalse(configurator.pending(bb), "pending() false again once drained");
}

// 7. Two deltas for the SAME target (kDrivetrain), touching DIFFERENT
//    fields, applied FIFO -- neither clobbers the other (the core
//    "field-masked, not full-replace" proof, AC-7).
void scenarioSameTargetDisjointFieldsDoNotClobber() {
  beginScenario("Two kDrivetrain deltas, disjoint fields, FIFO fold -- no clobber");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain;
  Subsystems::PoseEstimator poseEstimator;
  Subsystems::Planner planner;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, planner, hardware,
                                 msg::DrivetrainConfig{}, msg::PlannerConfig{});

  Rt::ConfigDelta first;
  first.target = Rt::ConfigDelta::kDrivetrain;
  first.mask = Rt::bitOf(Rt::DrivetrainConfigField::kTrackwidth);
  first.drivetrain.trackwidth = 150.0f;

  Rt::ConfigDelta second;
  second.target = Rt::ConfigDelta::kDrivetrain;
  second.mask = Rt::bitOf(Rt::DrivetrainConfigField::kRotationalSlip);
  second.drivetrain.rotational_slip = 0.85f;
  // Deliberately built as though from the SAME stale baseline as `first`
  // (trackwidth left at its default 0.0f in `second.drivetrain`, NOT
  // re-read from bb after `first` applied) -- proving the Configurator's
  // OWN fold, not the caller's baseline discipline, is what prevents the
  // clobber (commands.h's ConfigDelta comment / configurator.h's class
  // comment).

  checkTrue(bb.configIn.post(first), "post() #1 succeeds");
  checkTrue(bb.configIn.post(second), "post() #2 succeeds");

  configurator.applyOne(bb);
  checkFloatEq(bb.drivetrainConfig.trackwidth, 150.0f, "after delta #1: trackwidth applied");
  checkFloatEq(bb.drivetrainConfig.rotational_slip, 0.0f,
               "after delta #1: rotational_slip still at its pre-delta default");

  configurator.applyOne(bb);
  checkFloatEq(bb.drivetrainConfig.trackwidth, 150.0f,
               "after delta #2: trackwidth from delta #1 is NOT clobbered");
  checkFloatEq(bb.drivetrainConfig.rotational_slip, 0.85f,
               "after delta #2: rotational_slip from delta #2 applied");
}

// 8. applyOne() pops AT MOST one delta per call.
void scenarioApplyOneDrainsExactlyOnePerCall() {
  beginScenario("applyOne() pops exactly one delta per call, never more");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain;
  Subsystems::PoseEstimator poseEstimator;
  Subsystems::Planner planner;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, planner, hardware,
                                 msg::DrivetrainConfig{}, msg::PlannerConfig{});

  Rt::ConfigDelta a;
  a.target = Rt::ConfigDelta::kPlanner;
  a.mask = Rt::bitOf(Rt::PlannerConfigField::kMinSpeed);
  a.planner.min_speed = 1.0f;

  Rt::ConfigDelta b;
  b.target = Rt::ConfigDelta::kPlanner;
  b.mask = Rt::bitOf(Rt::PlannerConfigField::kArriveTol);
  b.planner.arrive_tol = 2.0f;

  checkTrue(bb.configIn.post(a), "post() #1 succeeds");
  checkTrue(bb.configIn.post(b), "post() #2 succeeds");
  checkUintEq(bb.configIn.size(), 2, "configIn holds both undrained deltas");

  configurator.applyOne(bb);
  checkUintEq(bb.configIn.size(), 1, "applyOne() drained exactly one");
  checkFloatEq(bb.plannerConfig.min_speed, 1.0f, "delta #1 applied");
  checkFloatEq(bb.plannerConfig.arrive_tol, 0.0f, "delta #2 not yet applied");

  configurator.applyOne(bb);
  checkUintEq(bb.configIn.size(), 0, "applyOne() drained the second, queue now empty");
  checkFloatEq(bb.plannerConfig.arrive_tol, 2.0f, "delta #2 now applied");
}

// 9. applyOne() called on an EMPTY configIn is a well-defined no-op.
void scenarioApplyOneOnEmptyQueueIsNoop() {
  beginScenario("applyOne() on an empty configIn is a no-op");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain;
  Subsystems::PoseEstimator poseEstimator;
  Subsystems::Planner planner;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, planner, hardware,
                                 msg::DrivetrainConfig{}, msg::PlannerConfig{});

  configurator.publish(bb);
  const msg::DrivetrainConfig before = bb.drivetrainConfig;

  checkTrue(bb.configIn.empty(), "sanity: configIn is empty");
  configurator.applyOne(bb);

  checkFloatEq(bb.drivetrainConfig.trackwidth, before.trackwidth,
               "bb.drivetrainConfig unchanged by a no-op applyOne()");
}

}  // namespace

int main() {
  scenarioDrivetrainDeltaApplies();
  scenarioMotorDeltaApplies();
  scenarioPlannerDeltaApplies();
  scenarioOdometerDeltaApplies();
  scenarioPublishSeedsAllFourCells();
  scenarioPendingMirrorsConfigInEmpty();
  scenarioSameTargetDisjointFieldsDoNotClobber();
  scenarioApplyOneDrainsExactlyOnePerCall();
  scenarioApplyOneOnEmptyQueueIsNoop();

  if (g_failureCount > 0) {
    std::printf("\n%d scenario failure(s)\n", g_failureCount);
    return 1;
  }
  std::printf("\nAll scenarios PASSED\n");
  return 0;
}
