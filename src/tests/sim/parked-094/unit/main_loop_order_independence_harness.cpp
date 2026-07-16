// PARKED (sprint 094, ticket 094-002): hand-drives a stale FOUR-subsystem
// pipeline (Hardware, Drivetrain, PoseEstimator, Planner) predating sprint
// 093's MainLoop gut; Subsystems::Planner is central to the property under
// test, so this cannot be trivially un-parked without a rewrite. See
// src/tests/sim/parked-094/README.md and clasi/issues/restore-goto-pursuit-
// with-pose-estimator.md.
//
// main_loop_order_independence_harness.cpp -- off-hardware acceptance proof
// for ticket 087-009 (SUC-001, re-confirmed against the FULL rebuilt loop
// per that ticket's own Acceptance Criteria -- not just the isolated
// primitives from ticket 002/007's own narrower proofs: ticket 002's
// runtime_blackboard_harness.cpp only round-trips the Mailbox/WorkQueue
// TYPES; ticket 007's dev_loop_pose_estimator_harness.cpp proves the REAL
// Rt::MainLoop::tick() matches a hand-driven reference that mirrors its OWN
// fixed call order, for PoseEstimator only -- neither actually swaps the
// call order between two pipelines and diffs the FULL resulting state).
//
// architecture-update-r1.md's Decision 6 (synchronous update) claims tick
// ORDER cannot affect the result: every subsystem reads only the committed
// snapshot x[k] (whatever was true at the END of the PREVIOUS pass) and
// writes only its own cell; the loop's commit step (bulk-copy into x[k+1])
// runs strictly AFTER every subsystem has ticked, and routeOutputs()
// (posting each emitter's output into its consumer's NEXT-pass input queue)
// also runs strictly after all four ticks. Since `Rt::MainLoop::tick()`
// itself has one fixed, hardcoded internal call order (hardware ->
// drivetrain -> poseEstimator -> planner), the property can't be tested by
// permuting the real function's own body without invasive surgery -- so
// this harness proves it the same way dev_loop_pose_estimator_harness.cpp
// proves its own narrower property: TWO independently, by-hand-driven
// pipelines (OrderedPipeline below), wired identically (same configs, same
// staged Planner DISTANCE goal, same Drivetrain governance authority) and
// fed the IDENTICAL `now` sequence, but ticking their four subsystems in
// OPPOSITE orders every single pass -- FORWARD (hardware, drivetrain,
// poseEstimator, planner -- Rt::MainLoop::tick()'s own real order) vs.
// REVERSE (planner, poseEstimator, drivetrain, hardware). Both pipelines'
// routeOutputs-equivalent and commit steps run strictly AFTER their own
// four ticks, exactly like the real loop -- so if Decision 6 genuinely
// holds, FORWARD and REVERSE must produce bit-identical x[k+1] (every
// committed state cell: both ports' MotorState, DrivetrainState,
// encoderPose, fusedPose, PlannerState) after every single pass, despite
// the different within-pass call order.
//
// Deliberately exercises real inter-subsystem coupling, not an inert setup:
// Drivetrain is put into WHEELS authority (setWheelTargets) so its own
// output reaches Hardware every pass ((093/094 teardown) via direct
// Hardware::apply(const Hal::DrivetrainToHardwareCommand&), gated on
// active(), NOT bb.motorIn[] -- that per-port queue is gone; see this
// file's OrderedPipeline::tick()); Planner is staged with a real DISTANCE
// goal so its
// own output competes for driveIn (Decision 1's plannerEngagedThisPass
// gate) and its STOP_DISTANCE terminal-decel anticipation (ticket 086-003,
// dead-time-compensated by ticket 087-009) runs every pass too -- the same
// class of stop-condition/anticipation math this ticket retuned, now
// proven order-independent as a first-class part of its own acceptance.
//
// Same ad hoc-compile convention as the other src/tests/sim/unit/*_harness.cpp
// files (hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure) -- compiled by test_main_loop_order_independence.py together
// with the real src/firm/subsystems/{drivetrain,sim_hardware,pose_estimator,
// planner}.cpp, src/firm/estimation/ekf_tiny.cpp, src/firm/kinematics/
// body_kinematics.cpp, src/firm/motion/{velocity_ramp,stop_condition}.cpp,
// src/firm/hal/sim/*.cpp, and src/firm/hal/velocity_pid.cpp -- deliberately NOT
// linking src/firm/runtime/main_loop.cpp itself (this harness drives the four
// subsystems directly, not through Rt::MainLoop) nor any command_processor/
// dev_commands/telemetry_commands source (never exercised here).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "runtime/blackboard.h"
#include "runtime/queue.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/planner.h"
#include "subsystems/pose_estimator.h"
#include "subsystems/sim_hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors the other *_harness.cpp files) ---

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

// Exact (bit-for-bit) equality -- both pipelines are pure functions of
// identical inputs modulo call order, so exact equality is the right bar
// (same argument dev_loop_pose_estimator_harness.cpp's own checkNear makes).
void checkExact(float actual, float expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- FORWARD=%.9g, REVERSE=%.9g (differ)", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

// --- Fixture helpers (mirrors dev_loop_pose_estimator_harness.cpp's own) ---

void fillDefaultConfigs(msg::MotorConfig configs[Subsystems::Hardware::kPortCount]) {
  msg::Gains gains;
  gains.kp = 0.0022f;
  gains.ki = 0.0018f;
  gains.kff = 0.0038f;
  gains.i_max = 0.3f;

  for (uint32_t i = 0; i < Subsystems::Hardware::kPortCount; ++i) {
    configs[i] = msg::MotorConfig{};
    configs[i].setPort(i + 1)
        .setFwdSign(1)
        .setTravelCalib(1.0f)
        .setVelGains(gains)
        .setVelFiltAlpha(1.0f)
        .setOutputDeadband(0.0f)
        .setReversalDwell(0.0f);
  }
}

msg::DrivetrainConfig makeDtConfig() {
  msg::DrivetrainConfig cfg;
  cfg.setTrackwidth(128.0f)
      .setRotationalSlip(0.92f)
      .setLeftPort(1)
      .setRightPort(2)
      .setEkfQXy(800.0f)
      .setEkfQTheta(4.0f)
      .setEkfROtosXy(50.0f)
      .setEkfROtosTheta(0.01f);
  return cfg;
}

msg::PlannerConfig makePlannerConfig() {
  msg::PlannerConfig cfg;
  cfg.setADecel(800.0f).setAMax(800.0f).setVBodyMax(1000.0f).setYawRateMax(10.0f)
      .setYawAccMax(20.0f).setArriveTol(25.0f).setTurnInPlaceGate(35.0f);
  return cfg;
}

enum class Order { FORWARD, REVERSE };

// OrderedPipeline -- drives the four real subsystems by hand, exactly the
// way Rt::MainLoop::tick() does (routeOutputs + commit strictly AFTER all
// four ticks), except the four ticks themselves run in `order`. See this
// file's header comment for why this is the right way to test Decision 6's
// order-independence claim without permuting Rt::MainLoop::tick() itself.
struct OrderedPipeline {
  Subsystems::SimHardware hardware;
  Subsystems::Drivetrain drivetrain;
  Subsystems::PoseEstimator poseEstimator;
  Subsystems::Planner planner;
  Order order;

  // (093/094 teardown) No local motorIn[]/motorResetIn[] any more --
  // Rt::Blackboard's own matching members are gone (blackboard.h's file
  // header) and Subsystems::Hardware::tick() no longer takes them. This
  // pipeline's own "routeOutputs equivalent" below now forwards Drivetrain's
  // output straight to Hardware::apply(const Hal::DrivetrainToHardwareCommand&)
  // -- a distribution path that is unaffected by this teardown (see
  // hardware.h) -- instead of posting to a per-port motorIn[] queue that no
  // longer exists, so the scenario's underlying determinism proof (does
  // FORWARD vs. REVERSE tick order still produce bit-identical committed
  // state on a pipeline that genuinely moves?) stays intact.
  Rt::WorkQueue<msg::DrivetrainCommand, 8> driveIn;

  // Committed x[k] -- exactly what bb.motors[]/bb.fusedPose/bb.otos* would
  // hold: refreshed ONLY at this pipeline's own commit step, below.
  msg::MotorState committedLeft;
  msg::MotorState committedRight;
  msg::PoseEstimate committedFusedPose;
  msg::PoseEstimate committedOtos;
  bool committedOtosValid = false;

  explicit OrderedPipeline(const msg::MotorConfig configs[Subsystems::Hardware::kPortCount],
                           const msg::DrivetrainConfig& dtConfig,
                           const msg::PlannerConfig& plannerConfig, Order tickOrder)
      : hardware(configs), order(tickOrder) {
    hardware.begin();
    drivetrain.configure(dtConfig);
    poseEstimator.configure(dtConfig);
    planner.configure(plannerConfig);
  }

  void tick(uint32_t now) {
    Subsystems::DrivetrainPorts p = drivetrain.ports();

    auto tickHardware = [&]() { hardware.tick(now); };
    auto tickDrivetrain = [&]() {
      // 090-001: Drivetrain::tick() now takes the FULL per-port array
      // (standing in for bb.motors[]) and resolves its own bound pair (p)
      // internally -- only the two bound-pair slots need real data, since
      // Drivetrain never reads any other slot.
      msg::MotorState motors[Subsystems::Hardware::kPortCount] = {};
      motors[p.left - 1] = committedLeft;
      motors[p.right - 1] = committedRight;
      drivetrain.tick(now, motors, Subsystems::Hardware::kPortCount, driveIn);
    };
    auto tickPose = [&]() {
      poseEstimator.tick(now, committedLeft, committedRight,
                         committedOtosValid ? &committedOtos : nullptr, poseResetIn_);
    };
    auto tickPlanner = [&]() {
      planner.tick(now, committedLeft, committedRight, committedFusedPose);
    };

    if (order == Order::FORWARD) {
      tickHardware();
      tickDrivetrain();
      tickPose();
      tickPlanner();
    } else {
      tickPlanner();
      tickPose();
      tickDrivetrain();
      tickHardware();
    }

    // routeOutputs equivalent -- strictly AFTER all four ticks, same as
    // Rt::MainLoop::tick() used to. (093/094 teardown) There is no
    // bb.motorIn[] to post into any more -- forwards straight to
    // Hardware::apply(const Hal::DrivetrainToHardwareCommand&) instead (a
    // distribution path this teardown leaves untouched, see hardware.h),
    // still gated on drivetrain.active() exactly like the old motorIn[]
    // post was (Decision 2's active()-gated discard), regardless of `order`.
    if (drivetrain.hasCommand()) {
      Hal::DrivetrainToHardwareCommand cmd = drivetrain.takeCommand();
      if (drivetrain.active()) {
        hardware.apply(cmd);
      }
    }
    if (planner.hasCommand()) {
      msg::DrivetrainCommand cmd = planner.takeCommand();
      if (planner.hasActiveCommand()) {
        driveIn.post(cmd);
      }
    }

    // Commit x[k+1] -- strictly AFTER routeOutputs, same as Rt::MainLoop::tick().
    committedLeft = hardware.motor(p.left).state();
    committedRight = hardware.motor(p.right).state();
    Hal::Odometer* odometer = hardware.odometer();
    if (odometer != nullptr) {
      odometer->tick(now);
      committedOtos = odometer->pose();
      committedOtosValid = true;
    } else {
      committedOtosValid = false;
    }
    committedFusedPose = poseEstimator.fusedPose();
  }

 private:
  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn_;
};

// runComparison -- constructs a FORWARD and a REVERSE OrderedPipeline from
// byte-identical configs, stages the SAME Planner DISTANCE goal and
// Drivetrain WHEELS authority on both, then ticks them in lockstep,
// asserting every committed state cell matches after every single pass.
void runComparison(int passCount, const std::string& scenarioName) {
  beginScenario(scenarioName);

  msg::MotorConfig configs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(configs);
  msg::DrivetrainConfig dtConfig = makeDtConfig();
  msg::PlannerConfig plannerConfig = makePlannerConfig();

  OrderedPipeline fwd(configs, dtConfig, plannerConfig, Order::FORWARD);
  OrderedPipeline rev(configs, dtConfig, plannerConfig, Order::REVERSE);

  // Drivetrain governance: both pipelines' drivetrains start WHEELS-active
  // with an asymmetric twist (so dTheta != 0, exercising the EKF's yaw
  // channel too, not just x/y).
  fwd.drivetrain.setWheelTargets(180.0f, 220.0f);
  rev.drivetrain.setWheelTargets(180.0f, 220.0f);

  // Planner: a real DISTANCE goal (SMOOTH stop style, the default) -- the
  // same STOP_DISTANCE anticipation ticket 087-009 retuned runs every pass
  // on both pipelines.
  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::DISTANCE;
  cmd.goal.distance.distance = 150.0f;  // [mm]
  cmd.goal.distance.speed = 150.0f;     // [mm/s]
  fwd.planner.apply(cmd, 0);
  rev.planner.apply(cmd, 0);

  uint32_t now = 0;
  for (int i = 0; i < passCount; ++i) {
    now += 20;  // [ms] -- matches main.cpp's own kPeriod

    fwd.tick(now);
    rev.tick(now);

    char label[64];
    std::snprintf(label, sizeof(label), "pass %d", i);
    std::string p(label);

    checkExact(rev.committedLeft.position.val, fwd.committedLeft.position.val,
               p + ": committedLeft.position");
    checkExact(rev.committedRight.position.val, fwd.committedRight.position.val,
               p + ": committedRight.position");
    checkExact(rev.committedLeft.velocity.val, fwd.committedLeft.velocity.val,
               p + ": committedLeft.velocity");
    checkExact(rev.committedRight.velocity.val, fwd.committedRight.velocity.val,
               p + ": committedRight.velocity");
    checkExact(rev.committedFusedPose.pose.x, fwd.committedFusedPose.pose.x,
               p + ": fusedPose.pose.x");
    checkExact(rev.committedFusedPose.pose.y, fwd.committedFusedPose.pose.y,
               p + ": fusedPose.pose.y");
    checkExact(rev.committedFusedPose.pose.h, fwd.committedFusedPose.pose.h,
               p + ": fusedPose.pose.h");

    msg::DrivetrainState dtFwd = fwd.drivetrain.state();
    msg::DrivetrainState dtRev = rev.drivetrain.state();
    checkTrue(dtFwd.active == dtRev.active, p + ": drivetrain.state().active matches");

    msg::PlannerState plFwd = fwd.planner.state();
    msg::PlannerState plRev = rev.planner.state();
    checkTrue(plFwd.active == plRev.active, p + ": planner.state().active matches");
    checkExact(plRev.body_twist.v_x, plFwd.body_twist.v_x, p + ": planner.state().body_twist.v_x");
    checkExact(plRev.body_twist.omega, plFwd.body_twist.omega,
               p + ": planner.state().body_twist.omega");
  }

  // Sanity: the sequence actually moved the robot AND the Planner goal
  // actually ran its course -- a trivially-passing all-zero/all-idle setup
  // would satisfy every checkExact() above without proving anything.
  checkTrue(std::fabs(fwd.committedFusedPose.pose.x) > 5.0f ||
                std::fabs(fwd.committedFusedPose.pose.y) > 5.0f,
            "sanity: the driven sequence actually produced motion");
  checkTrue(!fwd.planner.hasActiveCommand(),
            "sanity: the staged DISTANCE goal actually completed within passCount passes");
}

}  // namespace

int main() {
  runComparison(/*passCount=*/120,
                "FORWARD (hardware,drivetrain,poseEstimator,planner) vs. REVERSE "
                "(planner,poseEstimator,drivetrain,hardware) tick order produce "
                "bit-identical committed state every pass (SUC-001, 087-009)");

  if (g_failureCount == 0) {
    std::printf(
        "OK: re-ordering the mandatory-tick call sequence produces bit-identical x[k+1]\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the order-independence scenarios\n",
              g_failureCount);
  return 1;
}
