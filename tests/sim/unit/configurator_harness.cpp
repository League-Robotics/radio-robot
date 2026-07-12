// configurator_harness.cpp — off-hardware acceptance harness for ticket
// 087-005 (SUC-002/SUC-003/SUC-005): exercises Rt::Configurator
// (source/runtime/configurator.{h,cpp}) against REAL (not mocked)
// Subsystems::Drivetrain/PoseEstimator/SimHardware instances plus a
// real Rt::Blackboard — no fakes, per the ticket's own testability goal.
//
// 094-002: Subsystems::Planner was relocated out of source/ entirely (see
// source_parked/094/subsystems/planner.h); Rt::Configurator no longer takes
// a Planner& (source/runtime/configurator.h's own header note), so this
// harness no longer constructs one either. The kPlanner ConfigDelta target
// (scenario 3 below) still folds onto msg::PlannerConfig and still publishes
// bb.plannerConfig — only the (now-removed) live subsystem call is gone.
//
// Compiles with the plain system C++ compiler (-DHOST_BUILD, needed for
// SimHardware/PhysicsWorld's std::mt19937 members — see
// test_sim_hardware.py's own precedent) together with every REAL source it
// exercises: subsystems/{drivetrain,pose_estimator,sim_hardware}.cpp,
// kinematics/body_kinematics.cpp, estimation/ekf_tiny.cpp,
// hal/sim/{physics_world, sim_motor,sim_odometer}.cpp, hal/velocity_pid.cpp,
// and runtime/configurator.cpp itself — no CMake, no ARM toolchain.
// Hand-rolled assertions, prints PASS/FAIL, exits nonzero on any failure.
// Run by test_configurator.py, which compiles and runs this binary via
// subprocess.
//
// Each scenario constructs its own local Drivetrain/PoseEstimator/
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
//   3. kPlanner delta -> bb.plannerConfig published (folded only — no live
//      Subsystems::Planner to configure() since 094-002's relocation).
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
//   10. [098-005/M7] kPlanner delta (heading_kp) reaches the LIVE
//      Subsystems::Drivetrain — Configurator::applyOne()'s kPlanner case now
//      ALSO calls drivetrain_.configureMotion(plannerConfig_) (a residue of
//      ticket 094-002 relocating Subsystems::Planner out of source/ left that
//      case fold-and-publish only, never reaching a live subsystem — see
//      configurator.cpp's own kPlanner case comment). Proven end to end: a
//      segment posted AFTER a live heading_kp delta commands a materially
//      different twist on its own first tick than an IDENTICAL segment
//      posted BEFORE the delta — the sim-harness "config-delta injection
//      surface" this scenario exercises is bb.configIn.post() directly
//      (scenarios 1-9's own pattern above), the same queue a binary `SET`/
//      `config` wire command (commands/binary_channel.cpp) ultimately feeds.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/odometer.h"
#include "messages/planner.h"
#include "motion/segment.h"
#include "runtime/blackboard.h"
#include "runtime/commands.h"
#include "runtime/configurator.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
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
void fillDefaultConfigs(msg::MotorConfig configs[Subsystems::Hardware::kMotorCount]) {
  for (uint32_t i = 0; i < Subsystems::Hardware::kMotorCount; ++i) {
    configs[i] = msg::MotorConfig{};
    configs[i].setPort(i + 1).setFwdSign(1).setTravelCalib(1.0f);
  }
}

// 1. kDrivetrain delta -> Drivetrain::configure() called, bb.drivetrainConfig
//    published with the new value.
void scenarioDrivetrainDeltaApplies() {
  beginScenario("kDrivetrain delta: folds, configures, publishes");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kMotorCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain(hardware);
  Subsystems::PoseEstimator poseEstimator;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, hardware,
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

// 2. kMotor delta (index 1, physical port 2) -> Hal::Motor::configure()
//    called through Hardware::motor(i); bb.motorConfig[1] reflects it;
//    bb.motorConfig[0] (untouched motor) is unaffected.
//
// Note: Hardware::config(i) (087-004) is a boot-time snapshot only --
// ticket 087-004's own Implementation Notes confirm neither NezhaHardware
// nor SimHardware writes back into their motorConfigs_[] cache after
// construction (there is no Hardware::configure() setter at all yet, only
// the per-motor Hal::Motor::configure() this Configurator calls through).
// It would therefore be WRONG to assert hardware.config(1) reflects a
// post-construction configure() call -- it never will, by the current
// codebase's own design. The published bb.motorConfig[] cell (populated
// from THIS Configurator's own persistent motorConfig_[] copy, which IS
// kept live) is the correct, and per architecture-update-r1.md's own
// framing ("Current config -- published by the Configurator on apply...
// Replaces every shadow"), the DESIGNATED evidence for "what is this
// port's current config" post-087.
void scenarioMotorDeltaApplies() {
  beginScenario("kMotor delta: folds, configures port 2, publishes");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kMotorCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain(hardware);
  Subsystems::PoseEstimator poseEstimator;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, hardware,
                                 msg::DrivetrainConfig{}, msg::PlannerConfig{});

  Rt::ConfigDelta delta;
  delta.target = Rt::ConfigDelta::kMotor;
  delta.port = 1;   // 0-based motor index -- physical port 2
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

// 3. kPlanner delta -> folds onto msg::PlannerConfig, bb.plannerConfig
//    published (094-002: no live Subsystems::Planner left to configure()).
void scenarioPlannerDeltaApplies() {
  beginScenario("kPlanner delta: folds, publishes (no live Planner to configure)");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kMotorCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain(hardware);
  Subsystems::PoseEstimator poseEstimator;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, hardware,
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
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kMotorCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain(hardware);
  Subsystems::PoseEstimator poseEstimator;
  Rt::Blackboard bb;

  checkTrue(hardware.odometer() != nullptr, "sanity: SimHardware::odometer() is non-null");

  Rt::Configurator configurator(drivetrain, poseEstimator, hardware,
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
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kMotorCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain(hardware);
  Subsystems::PoseEstimator poseEstimator;
  Rt::Blackboard bb;

  msg::DrivetrainConfig bootDt;
  bootDt.trackwidth = 128.0f;
  msg::PlannerConfig bootPlanner;
  bootPlanner.min_speed = 5.0f;

  Rt::Configurator configurator(drivetrain, poseEstimator, hardware, bootDt, bootPlanner);

  checkTrue(bb.configIn.empty(), "sanity: configIn empty, publish() must not need a delta");
  configurator.publish(bb);

  checkFloatEq(bb.drivetrainConfig.trackwidth, 128.0f, "bb.drivetrainConfig seeded from boot config");
  checkFloatEq(bb.plannerConfig.min_speed, 5.0f, "bb.plannerConfig seeded from boot config");
  for (uint32_t idx = 0; idx < Subsystems::Hardware::kMotorCount; ++idx) {
    char what[96];
    std::snprintf(what, sizeof(what), "bb.motorConfig[%u].travel_calib seeded from Hardware::config(i)",
                  static_cast<unsigned>(idx));
    checkFloatEq(bb.motorConfig[idx].travel_calib, 1.0f, what);
  }
  checkFloatEq(bb.odometerConfig.linear_scalar, 0.0f,
               "bb.odometerConfig seeded zero-default (no boot-config source, per otos_commands.h)");
}

// 6. pending(bb) mirrors bb.configIn.empty() exactly.
void scenarioPendingMirrorsConfigInEmpty() {
  beginScenario("pending(bb) mirrors !bb.configIn.empty()");
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kMotorCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain(hardware);
  Subsystems::PoseEstimator poseEstimator;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, hardware,
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
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kMotorCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain(hardware);
  Subsystems::PoseEstimator poseEstimator;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, hardware,
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
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kMotorCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain(hardware);
  Subsystems::PoseEstimator poseEstimator;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, hardware,
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
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kMotorCount];
  fillDefaultConfigs(motorConfigs);
  Subsystems::SimHardware hardware(motorConfigs);
  Subsystems::Drivetrain drivetrain(hardware);
  Subsystems::PoseEstimator poseEstimator;
  Rt::Blackboard bb;

  Rt::Configurator configurator(drivetrain, poseEstimator, hardware,
                                 msg::DrivetrainConfig{}, msg::PlannerConfig{});

  configurator.publish(bb);
  const msg::DrivetrainConfig before = bb.drivetrainConfig;

  checkTrue(bb.configIn.empty(), "sanity: configIn is empty");
  configurator.applyOne(bb);

  checkFloatEq(bb.drivetrainConfig.trackwidth, before.trackwidth,
               "bb.drivetrainConfig unchanged by a no-op applyOne()");
}

// 10. [098-005/M7] A kPlanner delta carrying heading_kp reaches the LIVE
// Subsystems::Drivetrain -- proven by comparing the commanded twist
// Drivetrain stages on the FIRST tick of two back-to-back rotate-in-place
// segments, IDENTICAL in shape, one before and one after the delta is
// applied through Configurator::applyOne().
//
// Why "first tick" isolates exactly the P-term, deterministically, with NO
// dependence on plant lag/noise: Motion::SegmentExecutor::start() (called
// the SAME tick a segment is dequeued) latches its encoder baseline from
// THAT tick's own observation, so thetaMeasured (the delta since baseline)
// is EXACTLY 0.0f on a segment's first tick, always -- while the
// Ruckig-planned desired.position at t=+20ms is already slightly nonzero (a
// trapezoidal/jerk-limited profile ramps up from rest). So on tick one,
// omega = desired.velocity + heading_kp*(desired.position - 0) +
// heading_kd*(desired.velocity - omegaMeasured) -- with heading_kd left at
// 0.0f in both segments (untouched by this delta), only the heading_kp term
// differs between segment A (boot heading_kp=0.0f) and segment B (posted
// after heading_kp=50.0f arrives via bb.configIn). desired.velocity/
// desired.position at t=+20ms are IDENTICAL for A and B (same segment shape,
// same motion-limit config fields -- only heading_kp changed), so any
// magnitude difference between A's and B's commanded twist is attributable
// SOLELY to the live heading_kp delta having reached the executor that
// actually ran segment B. Deliberately uses a large heading_kp (50.0f, not
// the bench-tuned 6.0f -- data/robots/tovez.json, untouched by this ticket)
// to make the effect obviously larger than any float/kinematics-governor
// noise, since this scenario proves the WIRING, not a tuning value.
void scenarioPlannerHeadingKpDeltaReachesLiveDrivetrainNextSegment() {
  beginScenario(
      "098-005/M7: kPlanner heading_kp delta reaches the live Drivetrain -- next segment's "
      "commanded twist reflects the new gain, no restart");
  // A REAL (nonzero) velocity-PID config -- unlike fillDefaultConfigs() (used
  // by scenarios 1-9 above, which never actually run a segment's physics),
  // this scenario needs the plant to genuinely respond so segment A converges
  // via the M4 tolerance+dwell gate rather than the STOP_TIME stall backstop.
  // Values mirror drivetrain_harness.cpp's own defaultMotorConfigSet() (a
  // plant known to converge cleanly).
  msg::MotorConfig motorConfigs[Subsystems::Hardware::kMotorCount];
  msg::Gains velGains;
  velGains.kp = 0.0005f;
  velGains.ki = 0.0005f;
  velGains.kff = 1.0f / Hal::PhysicsWorld::kNominalMaxSpeed;
  velGains.i_max = 0.3f;
  for (uint32_t i = 0; i < Subsystems::Hardware::kMotorCount; ++i) {
    motorConfigs[i] = msg::MotorConfig();
    motorConfigs[i].setPort(i + 1).setFwdSign(1).setVelGains(velGains).setVelFiltAlpha(1.0f);
  }
  Subsystems::SimHardware hardware(motorConfigs);
  hardware.begin();
  Subsystems::Drivetrain drivetrain(hardware);
  Subsystems::PoseEstimator poseEstimator;
  Rt::Blackboard bb;

  msg::DrivetrainConfig dtConfig;
  dtConfig.setTrackwidth(150.0f).setLeftPort(1).setRightPort(2);
  drivetrain.configure(dtConfig);

  msg::PlannerConfig bootPlannerConfig;
  bootPlannerConfig.yaw_rate_max = 3.0f;   // [rad/s]
  bootPlannerConfig.yaw_acc_max = 15.0f;   // [rad/s^2]
  bootPlannerConfig.heading_kp = 0.0f;     // boot: open-loop -- isolates the P-term below
  bootPlannerConfig.heading_kd = 0.0f;
  drivetrain.configureMotion(bootPlannerConfig);

  Rt::Configurator configurator(drivetrain, poseEstimator, hardware, dtConfig, bootPlannerConfig);

  // --- Segment A: open-loop (boot heading_kp=0.0f). ---
  Motion::Segment segA;
  segA.distance = 0.0f;
  segA.direction = 0.5f;      // [rad] PRE_PIVOT-only (finalHeading == direction)
  segA.finalHeading = 0.5f;
  checkTrue(bb.segmentIn.post(segA), "segmentIn accepts segment A");

  // Captured on the segment's SECOND tick (i == 1), not its first: a fresh
  // SegmentExecutor::start() latches its baseline/elapsed-clock on the SAME
  // tick it fires, so the FIRST tick always samples the Ruckig plan at
  // elapsed==0 (desired.position/velocity both exactly 0.0f, omega==0.0f
  // regardless of heading_kp) -- confirmed by direct trace during this
  // scenario's own development. The SECOND tick samples the plan at
  // elapsed==20ms (genuinely nonzero desired.position/velocity) while
  // thetaMeasured is STILL ~0.0f (the wheel had nothing but a 0.0f target to
  // respond to for the ONE dt in between) -- exactly the deterministic,
  // plant-lag-free P-term isolation this scenario's file header describes.
  uint32_t now = 0;
  float cmdA = 0.0f;
  for (int i = 0; i < 300; ++i) {   // 6s -- ample settle time for a 0.5rad turn at 3rad/s
    now += 20;
    hardware.tick(now);
    drivetrain.tick(now, bb.segmentIn, bb.replaceIn, bb.driveIn);
    if (i == 1) cmdA = drivetrain.state().cmd()[0];   // [mm/s] left wheel, 2nd-tick commanded
  }
  msg::DrivetrainState settled = drivetrain.state();
  checkTrue(std::fabs(settled.vel()[0]) < 5.0f && std::fabs(settled.vel()[1]) < 5.0f,
            "precondition: segment A has converged and the plant is at rest before segment B");

  // --- Live delta: heading_kp 0.0f -> 50.0f, posted to bb.configIn exactly
  // as scenarios 1-9 above post theirs, then drained through the SAME
  // Configurator::applyOne() call main.cpp's loop now makes once per pass
  // (098-005/M7). ---
  Rt::ConfigDelta delta;
  delta.target = Rt::ConfigDelta::kPlanner;
  delta.mask = Rt::bitOf(Rt::PlannerConfigField::kHeadingKp);
  delta.planner.heading_kp = 50.0f;
  checkTrue(bb.configIn.post(delta), "post() heading_kp delta succeeds");
  checkTrue(configurator.pending(bb), "pending() true before applyOne()");
  configurator.applyOne(bb);
  checkFalse(configurator.pending(bb), "pending() false after draining the one delta");
  checkFloatEq(bb.plannerConfig.heading_kp, 50.0f, "bb.plannerConfig.heading_kp published");

  // --- Segment B: SAME shape as segment A, posted AFTER the live delta --
  // "the VERY NEXT segment". Captured on its own second tick, same argument
  // as segment A above. ---
  Motion::Segment segB;
  segB.distance = 0.0f;
  segB.direction = 0.5f;
  segB.finalHeading = 0.5f;
  checkTrue(bb.segmentIn.post(segB), "segmentIn accepts segment B");

  float cmdB = 0.0f;
  for (int i = 0; i < 2; ++i) {
    now += 20;
    hardware.tick(now);
    drivetrain.tick(now, bb.segmentIn, bb.replaceIn, bb.driveIn);
    if (i == 1) cmdB = drivetrain.state().cmd()[0];
  }

  // If the kPlanner delta never reached the live Drivetrain (the 094-002
  // regression this ticket fixes), segment B's own Motion::SegmentExecutor
  // would still be configured with the STALE heading_kp=0.0f boot value --
  // cmdB would equal cmdA (same segment shape, same open-loop trajectory,
  // same zero P-term). A meaningfully larger magnitude proves the new gain
  // reached the executor that actually ran segment B, with no restart.
  checkTrue(std::fabs(cmdB) > std::fabs(cmdA) + 2.0f,
            "segment B's commanded wheel speed reflects the new heading_kp -- meaningfully larger "
            "in magnitude than segment A's open-loop value, no restart between them");
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
  scenarioPlannerHeadingKpDeltaReachesLiveDrivetrainNextSegment();

  if (g_failureCount > 0) {
    std::printf("\n%d scenario failure(s)\n", g_failureCount);
    return 1;
  }
  std::printf("\nAll scenarios PASSED\n");
  return 0;
}
