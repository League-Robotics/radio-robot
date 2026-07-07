// planner_harness.cpp -- off-hardware acceptance harness for ticket 084-001
// (SUC-001/SUC-002/SUC-003): exercises Subsystems::Planner
// (source/subsystems/planner.{h,cpp}) in isolation against hand-built
// msg::PlannerCommand/msg::MotorState/msg::PoseEstimate fixtures -- no real
// hardware, no wire verb (this ticket lands none).
//
// Mirrors drivetrain_harness.cpp's shape exactly: #includes only
// subsystems/planner.h + messages/*.h (all dependency-free -- no
// MicroBit.h, no I2CBus), links against subsystems/planner.cpp,
// motion/velocity_ramp.cpp, and motion/stop_condition.cpp (Planner's own
// two real dependencies, both themselves dependency-free), compiles with
// the plain system C++ compiler -- no CMake, no ARM toolchain. Hand-rolled
// assertions, prints PASS/FAIL, exits nonzero on any failure. Run by
// test_planner.py, which compiles and runs this binary via subprocess.
//
// Ticket 087-003 note: Planner's own tick() signature and its output edge
// (hasCommand()/takeCommand() -> msg::DrivetrainCommand) are UNCHANGED in
// shape by sprint 087's blackboard wiring (see planner.h's doc comment on
// that edge) -- Planner is documented as the second producer of
// Rt::Mailbox<msg::DrivetrainCommand> driveIn (Decision 1), but the actual
// post into driveIn happens in the DRAINING caller (the loop's
// routeOutputs, ticket 007), not inside Planner itself. No scenario below
// changes: every existing hasCommand()/takeCommand() assertion still
// exercises the exact same edge, unmodified.
//
// Ticket 089-003 note: DISTANCE migrates onto Motion::JerkTrajectory (see
// planner.h's own "TWO COEXISTING MOTION-GENERATION MECHANISMS" class
// comment) -- this file now also links planner.cpp's new
// motion/jerk_trajectory.{h,cpp} dependency (test_planner.py's own compile
// command gained the extra source/include entries this pulls in). The
// DISTANCE-specific scenarios below were updated accordingly:
// scenarioDistanceGoalAnticipatesStopWithSpeedCap (086-003/087-009's
// closed-form ramp-anticipation cap test) is RENAMED and REWRITTEN as
// scenarioDistanceGoalStopFiredBeforeConvergenceForcesFreshDecel -- that old
// cap is DEAD CODE for DISTANCE now (applyStopAnticipation() itself is
// UNCHANGED and still serves TIMED/VELOCITY/STREAM/TURN/ROTATION/GOTO_GOAL,
// see planner.cpp), so its old closed-form expected values no longer apply
// to this goal kind; the new version tests the JerkTrajectory-equivalent
// property (a stop firing before the plan's own natural convergence forces
// a fresh, non-reversing decel-to-rest). Several new scenarios cover
// 089-003's other acceptance criteria: a full-trace no-reverse proof (AC3),
// and the Revision 2 divergence-triggered replan (AC7/AC8/AC9) -- normal
// (lagging-plant) and gross (stalled-plant) divergence, plus a guard-2
// (no-reverse-target) regression pin.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "messages/common.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "subsystems/planner.h"

namespace {

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

void checkFloatNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g (+/- %g), got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(tol),
                  static_cast<double>(actual));
    fail(buf);
  }
}

void checkStrEq(const char* actual, const char* expected, const std::string& what) {
  if (std::strcmp(actual, expected) != 0) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected \"%s\", got \"%s\"", what.c_str(), expected,
                  actual);
    fail(buf);
  }
}

msg::PlannerConfig generousConfig() {
  msg::PlannerConfig cfg;
  cfg.a_max = 1000.0f;
  cfg.a_decel = 1000.0f;
  cfg.v_body_max = 1000.0f;
  cfg.yaw_rate_max = 10.0f;
  cfg.yaw_acc_max = 100.0f;
  cfg.j_max = 0.0f;
  cfg.yaw_jerk_max = 0.0f;
  return cfg;
}

msg::MotorState obsPosition(float position) {
  msg::MotorState s;
  s.position.has = true;
  s.position.val = position;
  return s;
}

msg::PoseEstimate poseAt(float x, float y, float h) {
  msg::PoseEstimate p;
  p.pose.x = x;
  p.pose.y = y;
  p.pose.h = h;
  return p;
}

// 1. Fresh Planner: idle, no held command, no active goal.
void scenarioFreshPlannerIsIdle() {
  beginScenario("fresh Planner starts idle with no held command/event");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  checkFalse(planner.hasActiveCommand(), "no active command before any apply()");
  checkFalse(planner.hasCommand(), "no held command before any tick()");
  checkFalse(planner.hasEvent(), "no held event before any tick()");
  checkTrue(planner.state().mode == msg::DriveMode::IDLE, "fresh state().mode is IDLE");
}

// 2. hasCommand()/takeCommand(): false before tick(), true after (even while
// idle -- mirrors Drivetrain::tick()'s "unconditionally" contract), false
// again immediately after takeCommand().
void scenarioHasCommandTakeCommandClearsEvenWhileIdle() {
  beginScenario("hasCommand()/takeCommand(): set by tick() unconditionally, cleared by takeCommand()");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  planner.tick(1000, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
  checkTrue(planner.hasCommand(), "tick() holds a command even with no active goal");

  msg::DrivetrainCommand held = planner.takeCommand();
  checkTrue(held.control_kind == msg::DrivetrainCommand::ControlKind::TWIST,
            "idle output is a TWIST command");
  checkFloatNear(held.control.twist.v_x, 0.0f, 1e-6f, "idle TWIST is zero");
  checkFloatNear(held.control.twist.omega, 0.0f, 1e-6f, "idle TWIST is zero");
  checkFalse(planner.hasCommand(), "takeCommand() clears hasCommand()");
}

// 3. configure() actually reaches the owned VelocityRamp: with NO configure()
// call (all-zero PlannerConfig, a_max == 0), a staged VELOCITY goal never
// ramps away from zero -- proving the ramp reads ITS limits from
// Planner::configure(), not some other default.
void scenarioConfigureReachesOwnedRamp() {
  beginScenario("configure() forwards limits to the owned VelocityRamp");
  Subsystems::Planner planner;  // deliberately never configured

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::VELOCITY;
  cmd.goal.velocity.v_x = 50.0f;
  planner.apply(cmd, 0);

  for (int i = 0; i < 5; ++i) {
    planner.tick(1000 + i * 100, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
  }
  msg::DrivetrainCommand held = planner.takeCommand();
  checkFloatNear(held.control.twist.v_x, 0.0f, 1e-6f,
                 "a_max == 0 (never configured) means the ramp never leaves zero");
}

// 4. VELOCITY goal_kind: open-ended (no stops_[], no implicit synthesis) --
// ramps to the commanded twist and stays active indefinitely. 084-005
// (Decision 6, planner.cpp's velocityShapedMode()): a VELOCITY goal with no
// caller-supplied stop condition is "open-ended" and reports
// DriveMode::STREAMING -- the SAME wire-facing bucket a bare `S`/`R` uses --
// NOT its own bespoke DriveMode::VELOCITY (which planner.cpp no longer ever
// emits; see that function's own doc comment).
void scenarioVelocityGoalRampsAndStaysOpenEnded() {
  beginScenario("VELOCITY goal_kind ramps to target and never self-terminates");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::VELOCITY;
  cmd.goal.velocity.v_x = 80.0f;
  cmd.goal.velocity.omega = 1.0f;
  planner.apply(cmd, 0);

  checkTrue(planner.hasActiveCommand(), "apply() activates the command");
  checkTrue(planner.state().mode == msg::DriveMode::STREAMING,
            "state().mode == STREAMING (084-005: no stop => open-ended)");

  for (int i = 0; i < 20; ++i) {
    planner.tick(1000 + i * 100, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});
    planner.takeCommand();
  }

  checkTrue(planner.hasActiveCommand(), "VELOCITY with no stops_[] never self-terminates");
  checkFalse(planner.hasEvent(), "no event queued -- goal never fired a stop condition");
  msg::DrivetrainCommand held = planner.takeCommand();
  checkFloatNear(held.control.twist.v_x, 80.0f, 0.5f, "ramped to the commanded v_x");
  checkFloatNear(held.control.twist.omega, 1.0f, 0.001f, "ramped to the commanded omega");
}

// 4b. VELOCITY goal_kind WITH a caller-supplied stop condition: 084-005's
// velocityShapedMode() reports DriveMode::TIMED instead of ::STREAMING for
// this SAME goal kind -- the mapping is purely data-driven (stops_count_val()
// > 0), not a distinct GoalKind. This is the isolated-Planner-level proof
// that a bounded `R` (motion_commands.cpp's handleR, when the wire carries a
// stop= clause) lands in the same wire bucket as a plain `T`.
void scenarioVelocityGoalWithStopReportsTimed() {
  beginScenario("VELOCITY goal_kind WITH a caller stop reports TIMED, not STREAMING");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::VELOCITY;
  cmd.goal.velocity.v_x = 80.0f;
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_TIME;
  cmd.stops_[0].a = 500.0f;  // [ms]
  planner.apply(cmd, 0);

  checkTrue(planner.state().mode == msg::DriveMode::TIMED,
            "state().mode == TIMED (084-005: a caller stop => self-terminating)");
}

// 5. DISTANCE goal_kind: Planner synthesizes the DISTANCE stop itself from
// the goal's own distance field (no stops_[] needed from the caller);
// ABRUPT style fires the event on the SAME tick the stop condition fires,
// with the ramp immediately reset to zero.
void scenarioDistanceGoalFiresImplicitStopAbrupt() {
  beginScenario("DISTANCE goal_kind synthesizes its own stop; ABRUPT completes immediately");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::DISTANCE;
  cmd.goal.distance.distance = 200.0f;
  cmd.goal.distance.speed = 100.0f;
  cmd.style = msg::StopStyle::ABRUPT;
  std::strncpy(cmd.corr_id, "dist1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  checkTrue(planner.state().mode == msg::DriveMode::DISTANCE, "state().mode == DISTANCE");

  // Tick 1: baseline captured (enc0 = 0mm) -- first-ever tick has dt == 0,
  // so nothing fires yet.
  planner.tick(1000, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});
  planner.takeCommand();
  checkTrue(planner.hasActiveCommand(), "not yet -- baseline tick only");

  // Tick 2: 100mm traveled -- short of the 200mm threshold.
  planner.tick(2000, obsPosition(100.0f), obsPosition(100.0f), msg::PoseEstimate{});
  planner.takeCommand();
  checkTrue(planner.hasActiveCommand(), "100mm traveled -- short of the 200mm DISTANCE stop");
  checkFalse(planner.hasEvent(), "no event yet");

  // Tick 3: 200mm traveled -- DISTANCE stop fires; ABRUPT -> immediate finish.
  planner.tick(3000, obsPosition(200.0f), obsPosition(200.0f), msg::PoseEstimate{});
  msg::DrivetrainCommand held = planner.takeCommand();

  checkFalse(planner.hasActiveCommand(), "ABRUPT fire completes the goal on the same tick");
  checkTrue(planner.state().mode == msg::DriveMode::IDLE, "state().mode returns to IDLE");
  checkFloatNear(held.control.twist.v_x, 0.0f, 1e-6f, "ABRUPT reset zeroes the held twist immediately");

  checkTrue(planner.hasEvent(), "DISTANCE stop firing queues an event");
  Subsystems::Planner::Event evt = planner.takeEvent();
  checkStrEq(evt.reason, "dist", "reason token is \"dist\" for a fired STOP_DISTANCE");
  checkStrEq(evt.corrId, "dist1", "corrId round-trips from the staged PlannerCommand");
  checkFalse(planner.hasEvent(), "takeEvent() clears hasEvent()");
}

// 5b. [089-003 REWRITE -- was scenarioDistanceGoalAnticipatesStopWithSpeedCap,
// 086-003/087-009's closed-form ramp-anticipation cap test] DISTANCE goal_kind
// now stages a position-control Motion::JerkTrajectory solve-to-rest at
// apply() time instead of a ramp_ target -- applyStopAnticipation()'s
// STOP_DISTANCE cap this scenario used to test is DEAD CODE for DISTANCE now
// (the function itself is UNCHANGED and still serves TIMED/VELOCITY/STREAM/
// TURN/ROTATION/GOTO_GOAL, planner.cpp). The property this scenario now
// tests is planner.h's ticket item 4 / AC4: a SMOOTH-style stop firing
// BEFORE linear_'s own plan has naturally converged to rest forces a fresh,
// non-reversing decel-to-rest, seeded from the channel's own current state
// (never leftObs/rightObs). SMOOTH (not ABRUPT, unlike most DISTANCE
// scenarios here) is used deliberately -- ABRUPT bypasses
// armDistanceStopDecel() entirely (planner.cpp's tick()), so it cannot
// exercise this branch.
void scenarioDistanceGoalStopFiredBeforeConvergenceForcesFreshDecel() {
  beginScenario(
      "089-003 AC4: SMOOTH stop firing before linear_'s own convergence forces a "
      "fresh decel-to-rest, no reverse");
  Subsystems::Planner planner;
  planner.configure(generousConfig());  // a_max=a_decel=1000mm/s^2, v_body_max=1000mm/s, j_max=0

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::DISTANCE;
  cmd.goal.distance.distance = 500.0f;  // [mm]
  cmd.goal.distance.speed = 200.0f;     // [mm/s] -- undisturbed plan duration ~2.7s
  // cmd.style left at its default (SMOOTH) -- see comment above on why.
  std::strncpy(cmd.corr_id, "fast1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  planner.tick(0, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});  // baseline
  planner.takeCommand();

  // Plant "overshoots" the plan: encoder advances at a constant 300mm/s
  // (faster than the plan's own 200mm/s cruise), reaching the 500mm
  // STOP_DISTANCE threshold at t=500/300=1.667s -- well before the
  // undisturbed plan's own ~2.7s natural convergence -- forcing
  // armDistanceStopDecel()'s REAL re-solve branch (unlike scenario 5's
  // ABRUPT completion, which never reaches that branch at all).
  const float kPlantSpeed = 300.0f;  // [mm/s]
  bool everReversed = false;
  bool sawSmoothArm = false;
  bool completedWithDist = false;
  uint32_t t = 0;
  for (int i = 0; i < 120; ++i) {  // up to 6s -- comfortably past arming + convergence
    t += 50;
    float pos = std::min(500.0f, kPlantSpeed * static_cast<float>(t) * 0.001f);
    planner.tick(t, obsPosition(pos), obsPosition(pos), msg::PoseEstimate{});
    msg::DrivetrainCommand held = planner.takeCommand();
    if (held.control.twist.v_x < -0.5f) everReversed = true;
    if (planner.hasActiveCommand() && pos >= 500.0f) sawSmoothArm = true;
    if (!planner.hasActiveCommand()) {
      completedWithDist = (std::strcmp(planner.takeEvent().reason, "dist") == 0);
      break;
    }
  }
  checkFalse(everReversed, "the forced decel-to-rest re-solve never reverses");
  checkTrue(sawSmoothArm,
            "SMOOTH style does not complete on the same tick the stop fires -- ramp/decel-down observed");
  checkTrue(completedWithDist, "still completes via reason=dist once the decel-to-rest converges");
}

// 5c. [089-003, AC3] DISTANCE goal_kind: the full commanded velocity trace,
// sampled across the REAL apply()+tick() staging path with a closely-tracking
// (undisturbed) encoder feed, never reverses and completes via the goal's
// own STOP_DISTANCE (not the STOP_TIME safety net) -- mirrors
// test_ruckig_smoke.py's/jerk_trajectory_harness.cpp's own no-reverse
// assertion, now proven against the real goal-staging path (planner.h's
// class comment; ticket 089-003's own acceptance bar).
void scenarioDistanceGoalRuckigTraceNeverReverses() {
  beginScenario(
      "089-003 AC3: DISTANCE's full commanded velocity trace never reverses "
      "(real apply()+tick() path)");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::DISTANCE;
  cmd.goal.distance.distance = 500.0f;  // [mm]
  cmd.goal.distance.speed = 200.0f;     // [mm/s]
  cmd.style = msg::StopStyle::ABRUPT;   // isolate the trace from the SMOOTH
                                        // ramp-down (covered by 5b above)
  std::strncpy(cmd.corr_id, "trace1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  // Hand-derived trapezoid (j_max == 0 -> Ruckig's own infinite-jerk
  // sentinel -> an exact trapezoid, matching this closed form) for a
  // CLOSELY-TRACKING encoder feed -- exercises the undisturbed case (no
  // divergence-replan expected; that is covered by the dedicated scenarios
  // below), the common/expected real-world path.
  const float kAMax = 1000.0f;   // [mm/s^2]
  const float kVMax = 200.0f;    // [mm/s]
  const float kDistance = 500.0f;  // [mm]
  const float kTAccel = kVMax / kAMax;                       // 0.2s
  const float kDAccel = 0.5f * kAMax * kTAccel * kTAccel;    // 20mm
  const float kDCruise = kDistance - 2.0f * kDAccel;          // 460mm
  const float kTCruise = kDCruise / kVMax;                    // 2.3s
  const float kTDecelStart = kTAccel + kTCruise;              // 2.5s
  const float kTTotal = kTDecelStart + kTAccel;               // 2.7s

  auto trapezoidPosition = [&](float time) -> float {
    if (time <= 0.0f) return 0.0f;
    if (time < kTAccel) return 0.5f * kAMax * time * time;
    if (time < kTDecelStart) return kDAccel + kVMax * (time - kTAccel);
    if (time < kTTotal) {
      float s = time - kTDecelStart;
      return kDAccel + kDCruise + kVMax * s - 0.5f * kAMax * s * s;
    }
    return kDistance;
  };

  planner.tick(0, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});  // baseline
  planner.takeCommand();

  bool everReversed = false;
  bool completedWithDist = false;
  for (int ms = 100; ms <= 3200; ms += 100) {
    float pos = trapezoidPosition(static_cast<float>(ms) * 0.001f);
    planner.tick(static_cast<uint32_t>(ms), obsPosition(pos), obsPosition(pos), msg::PoseEstimate{});
    msg::DrivetrainCommand held = planner.takeCommand();
    if (held.control.twist.v_x < -0.5f) everReversed = true;
    if (!planner.hasActiveCommand()) {
      completedWithDist = (std::strcmp(planner.takeEvent().reason, "dist") == 0);
      break;
    }
  }
  checkFalse(everReversed, "commanded v_x never goes negative across the full trace");
  checkTrue(completedWithDist,
            "goal completes via STOP_DISTANCE (reason=dist), not the STOP_TIME safety net");
}

// 5d. [089-003 Revision 2, AC7/AC9] DISTANCE goal_kind: a lagging (slipping)
// plant -- injected at the test-fixture level (not via the sim plant, which
// tracks too closely to diverge naturally, per the ticket's own testing
// plan) -- triggers NORMAL divergence-triggered retarget()s and still
// completes via the goal's OWN STOP_DISTANCE, not the STOP_TIME safety net
// (architecture-update.md (089) Decision 10's whole purpose; AC9's crisp
// sim-level proof). WITHOUT the fix, the undisturbed plan converges to rest
// at its own fixed ~2.7s having covered only 85% of the commanded distance,
// then commands 0 forever -- the encoder freezes short and the goal only
// completes via the 7s STOP_TIME net.
//
// The plant model applies the commanded v_x TWO TICKS LATE (kOutputHops, the
// SAME two-pass output dead time kDeadTime already accounts for) in addition
// to the 15% slip -- mirroring the real Planner->driveIn->Drivetrain->
// motorIn->Hardware pipeline latency the dead-time projection is built
// against (Decision 8's revision). Without this delay, a zero-latency test
// plant "receives" each replan's own corrected command instantaneously, which
// makes the projection's deliberate v*tau under-target (the real wheel's
// own continued coast through that same dead time is meant to close this
// exact gap) look like a permanent shortfall instead of the correction it
// is -- an artifact of an unrealistically fast test plant, not of the
// mechanism itself.
void scenarioDistanceGoalDivergenceReplanCorrectsLaggingPlant() {
  beginScenario(
      "089-003 AC7/AC9: a lagging plant triggers NORMAL retarget()s and still completes via "
      "dist, not the STOP_TIME safety net");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::DISTANCE;
  cmd.goal.distance.distance = 500.0f;
  cmd.goal.distance.speed = 200.0f;
  cmd.style = msg::StopStyle::ABRUPT;
  std::strncpy(cmd.corr_id, "lag1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  planner.tick(0, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});
  planner.takeCommand();

  // Closed-loop plant simulation: the plant only achieves kSlip (85%) of
  // whatever velocity Planner commanded kOutputHops ticks ago -- a bounded,
  // ROUTINE tracking lag (Decision 10's own "plausibly 5-15mm over a 500mm
  // D" framing) plus the real output dead time (see comment above).
  const float kSlip = 0.85f;
  const float kDt = 0.02f;  // [s] 20ms tick, matches kAssumedPassPeriod exactly
  const int kOutputHops = 2;
  float delayBuf[kOutputHops] = {0.0f, 0.0f};
  int delayHead = 0;
  float encoderPos = 0.0f;
  bool everReversed = false;
  bool completedWithDist = false;
  uint32_t t = 0;
  for (int i = 0; i < 750; ++i) {  // up to 15s -- comfortably past the 7s STOP_TIME net
    t += static_cast<uint32_t>(kDt * 1000.0f);
    planner.tick(t, obsPosition(encoderPos), obsPosition(encoderPos), msg::PoseEstimate{});
    msg::DrivetrainCommand held = planner.takeCommand();
    float vCmd = held.control.twist.v_x;
    if (vCmd < -0.5f) everReversed = true;
    float vApplied = delayBuf[delayHead];
    delayBuf[delayHead] = vCmd;
    delayHead = (delayHead + 1) % kOutputHops;
    encoderPos += kSlip * vApplied * kDt;
    if (!planner.hasActiveCommand()) {
      completedWithDist = (std::strcmp(planner.takeEvent().reason, "dist") == 0);
      break;
    }
  }
  checkFalse(everReversed, "the divergence-corrected trace never reverses despite the lagging plant");
  checkTrue(completedWithDist,
            "a lagging plant completes via STOP_DISTANCE (reason=dist) thanks to the divergence "
            "replan, not the STOP_TIME safety net");
}

// 5e. [089-003 Revision 2, AC7] DISTANCE goal_kind: a stalled-then-freed
// plant (a genuine departure from the plan, not routine tracking lag)
// triggers the GROSS divergence path (reanchor(), not retarget()) and still
// completes via dist, with no reverse. Same output-dead-time plant model as
// scenario 5d above (see its own comment) -- a later NORMAL retarget can
// still fire after the initial reanchor (e.g. from residual timing/
// discretization drift), and that retarget's own dead-time projection needs
// the matching delay to land on target, exactly like 5d.
void scenarioDistanceGoalGrossDivergenceReanchorsAfterStall() {
  beginScenario(
      "089-003 AC7: a stalled-then-freed plant (gross divergence) triggers reanchor() and "
      "still completes via dist");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::DISTANCE;
  cmd.goal.distance.distance = 500.0f;
  cmd.goal.distance.speed = 200.0f;
  cmd.style = msg::StopStyle::ABRUPT;
  std::strncpy(cmd.corr_id, "stall1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  planner.tick(0, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});
  planner.takeCommand();

  // Frozen (wedged) for the first second, then tracks the commanded
  // velocity normally -- the undisturbed plan would think it has covered
  // ~180mm by t=1s while the measured encoder shows 0mm, a divergence well
  // past kGrossDivergenceThreshold.
  const float kDt = 0.02f;  // [s] matches kAssumedPassPeriod, see 5d's comment
  const int kOutputHops = 2;
  float delayBuf[kOutputHops] = {0.0f, 0.0f};
  int delayHead = 0;
  float encoderPos = 0.0f;
  bool everReversed = false;
  bool completedWithDist = false;
  uint32_t t = 0;
  for (int i = 0; i < 750; ++i) {
    t += static_cast<uint32_t>(kDt * 1000.0f);
    planner.tick(t, obsPosition(encoderPos), obsPosition(encoderPos), msg::PoseEstimate{});
    msg::DrivetrainCommand held = planner.takeCommand();
    float vCmd = held.control.twist.v_x;
    if (vCmd < -0.5f) everReversed = true;
    float vApplied = delayBuf[delayHead];
    delayBuf[delayHead] = vCmd;
    delayHead = (delayHead + 1) % kOutputHops;
    if (t >= 1000) {
      encoderPos += vApplied * kDt;  // freed -- full tracking from here on
    }
    // else: stalled -- encoderPos stays frozen at 0 regardless of vCmd.
    if (!planner.hasActiveCommand()) {
      completedWithDist = (std::strcmp(planner.takeEvent().reason, "dist") == 0);
      break;
    }
  }
  checkFalse(everReversed, "the gross-divergence reanchor's trace never reverses");
  checkTrue(completedWithDist, "a stalled-then-freed plant still completes via dist after the reanchor");
}

// 5f. [089-003 Revision 2, AC8] DISTANCE goal_kind: guard 2 (no-reverse-
// target) skips a would-be-backward replan near the target -- a synthetic
// observation showing 497mm already traveled (still short of the 500mm
// STOP_DISTANCE threshold -- NOT_FIRED) whose dead-time-projected remaining
// is <= 0 must NOT trigger a replan, even though the RAW divergence against
// the plan's own (still-early) position is large. Proven by comparing
// against an undisturbed control run: if the guard skipped the replan (as
// required), the commanded v_x at this tick is unaffected by the synthetic
// observation, matching the control exactly.
void scenarioDistanceGoalGuardSkipsNearTargetBackwardReplan() {
  beginScenario(
      "089-003 AC8: guard 2 (no-reverse-target) skips a would-be-backward replan near the "
      "target");
  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::DISTANCE;
  cmd.goal.distance.distance = 500.0f;
  cmd.goal.distance.speed = 200.0f;
  cmd.style = msg::StopStyle::ABRUPT;

  // Control: an undisturbed, closely-tracking encoder feed (40mm at
  // t=300ms matches the plan's own trapezoid position exactly -- see
  // scenario 5c) -- divergence stays ~0, no replan for either planner.
  Subsystems::Planner control;
  control.configure(generousConfig());
  control.apply(cmd, 0);
  control.tick(0, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});
  control.takeCommand();
  control.tick(300, obsPosition(40.0f), obsPosition(40.0f), msg::PoseEstimate{});
  msg::DrivetrainCommand controlHeld = control.takeCommand();

  Subsystems::Planner test;
  test.configure(generousConfig());
  test.apply(cmd, 0);
  test.tick(0, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});
  test.takeCommand();
  // A synthetic observation showing 497mm already traveled at t=300ms --
  // NOT yet >= 500 (still NOT_FIRED), but its dead-time-projected remaining
  // (3mm - planSpeed*kDeadTime ~ 3 - 8 = -5mm) is <= 0: guard 2 must skip
  // the replan entirely, leaving this tick's commanded v_x identical to the
  // undisturbed control's -- despite a large RAW divergence (~457mm)
  // against the plan's own still-early position that would otherwise (sans
  // guard 2) trigger a gross reanchor pointing backward.
  test.tick(300, obsPosition(497.0f), obsPosition(497.0f), msg::PoseEstimate{});
  msg::DrivetrainCommand testHeld = test.takeCommand();

  checkTrue(test.hasActiveCommand(), "497mm < 500mm -- STOP_DISTANCE has not fired yet");
  checkFloatNear(testHeld.control.twist.v_x, controlHeld.control.twist.v_x, 1.0f,
                 "guard 2 skips the near-target replan -- commanded v_x matches the undisturbed "
                 "control");
}

// 6. TIMED goal_kind: Planner synthesizes an implicit TIME stop from
// duration; default (SMOOTH) style ramps to (0,0) before completing, and the
// event is queued only once convergence (or the soft deadline) is reached --
// NOT on the same tick the stop condition first fires.
void scenarioTimedGoalSmoothRampDown() {
  beginScenario("TIMED goal_kind: implicit TIME stop, SMOOTH ramp-down before completion");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::TIMED;
  cmd.goal.timed.v_x = 50.0f;
  cmd.goal.timed.omega = 0.0f;
  cmd.goal.timed.duration = 300;  // [ms]
  // cmd.style left at its default (SMOOTH == 0).
  std::strncpy(cmd.corr_id, "t1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  checkTrue(planner.state().mode == msg::DriveMode::TIMED, "state().mode == TIMED");

  planner.tick(1000, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});  // baseline (dt=0)
  planner.takeCommand();

  planner.tick(1100, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});  // dt=0.1s -> v=50
  msg::DrivetrainCommand held = planner.takeCommand();
  checkFloatNear(held.control.twist.v_x, 50.0f, 1e-3f, "ramped to the commanded 50 mm/s");
  checkFalse(planner.hasEvent(), "TIME stop has not fired yet (elapsed=100ms < 300ms)");

  planner.tick(1400, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});  // elapsed=400ms >= 300ms
  planner.takeCommand();
  checkTrue(planner.hasActiveCommand(),
            "SMOOTH style: firing the stop ARMS the ramp-down, does not finish immediately");
  checkFalse(planner.hasEvent(), "no event yet -- still ramping down to (0,0)");

  planner.tick(1500, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});  // dt=0.1s -> v: 50->0
  msg::DrivetrainCommand finalHeld = planner.takeCommand();
  checkFloatNear(finalHeld.control.twist.v_x, 0.0f, 1e-3f, "converged to (0,0)");
  checkFalse(planner.hasActiveCommand(), "converged -- goal is now complete");
  checkTrue(planner.hasEvent(), "convergence queues the completion event");

  Subsystems::Planner::Event evt = planner.takeEvent();
  checkStrEq(evt.reason, "time", "reason token is \"time\" for a fired STOP_TIME");
  checkStrEq(evt.corrId, "t1", "corrId round-trips from the staged PlannerCommand");
}

// 7. TURN goal_kind: TurnGoal.speed is treated as an already-signed omega
// (no v); relies entirely on caller-supplied stops_[] (a HEADING stop here)
// -- Planner synthesizes nothing for this goal kind.
void scenarioTurnGoalUsesSignedSpeedAndCallerStop() {
  beginScenario("TURN goal_kind: signed omega pass-through, caller-supplied HEADING stop");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::TURN;
  cmd.goal.turn.speed = -2.0f;  // already-signed: CW
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_HEADING;
  cmd.stops_[0].a = -1.5707963f;  // target delta: -90 deg
  cmd.stops_[0].b = 0.05f;        // eps
  cmd.style = msg::StopStyle::ABRUPT;  // isolate the stop-condition mechanics
                                        // from the SMOOTH ramp-down (covered
                                        // separately by the TIMED scenario)
  planner.apply(cmd, 0);

  // 084-005: TURN always carries its own caller-supplied HEADING stop (see
  // handleTURN, motion_commands.cpp), so velocityShapedMode() always
  // resolves to TIMED here -- never STREAMING, never the retired
  // DriveMode::VELOCITY.
  checkTrue(planner.state().mode == msg::DriveMode::TIMED,
            "state().mode == TIMED (084-005: TURN always self-terminates)");

  planner.tick(1000, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));  // baseline
  msg::DrivetrainCommand held = planner.takeCommand();
  checkFloatNear(held.control.twist.v_x, 0.0f, 1e-6f, "TURN commands v=0 (turn-in-place)");

  planner.tick(1100, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));
  planner.takeCommand();
  checkFloatNear(planner.state().body_twist.omega, -2.0f, 1e-3f,
                 "ramped to the already-signed commanded omega");

  planner.tick(1200, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, -1.5707963f));
  planner.takeCommand();
  checkFalse(planner.hasActiveCommand(), "HEADING stop fires once the target heading is reached");
  checkTrue(planner.hasEvent(), "HEADING stop firing queues an event");
  checkStrEq(planner.takeEvent().reason, "heading", "reason token is \"heading\"");
}

// 7b. TURN goal_kind: ticket 086-003's terminal anticipation caps the
// commanded turn rate as the HEADING stop's remaining heading error shrinks
// -- BEFORE the stop itself fires -- the angular-rate analog of 5b's linear
// cap. Uses generousConfig()'s yaw_acc_max=100 rad/s^2 UNCHANGED (needed so
// the 0.1s ticks below still converge the ramp to target within a single
// tick, exactly like scenario 7 above) with a tighter eps (0.01 rad instead
// of scenario 7's 0.05) so the cap's binding window (remaining <
// omega^2/(2*yaw_acc_max) = 0.02 rad here) sits comfortably OUTSIDE eps --
// distinguishing "capped, but not yet fired" mid-turn.
void scenarioTurnGoalAnticipatesHeadingStopWithRateCap() {
  beginScenario("TURN goal_kind: terminal rate cap anticipates the HEADING stop (086-003)");
  Subsystems::Planner planner;
  planner.configure(generousConfig());  // yaw_acc_max = 100 rad/s^2

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::TURN;
  cmd.goal.turn.speed = -2.0f;  // already-signed: CW
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_HEADING;
  cmd.stops_[0].a = -1.5707963f;  // target delta: -90 deg
  cmd.stops_[0].b = 0.01f;        // tight eps -- see comment above on why
  cmd.style = msg::StopStyle::ABRUPT;
  planner.apply(cmd, 0);

  planner.tick(1000, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));  // baseline
  planner.takeCommand();

  // Far from the target heading (90 deg of error remains) -- omegaCap =
  // sqrt(2*100*1.5708) = 17.7 rad/s, way above the 2.0 rad/s commanded
  // rate: the cap does not bind (a_max/yaw_acc_max=100 + 0.1s dt converge
  // within this one tick, exactly like scenario 7).
  planner.tick(1100, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));
  msg::DrivetrainCommand held = planner.takeCommand();
  checkFloatNear(held.control.twist.omega, -2.0f, 1e-3f,
                 "far from the target heading -- anticipation cap does not bind");

  // 0.015 rad (~0.86 deg) of heading error remains -- omegaCap =
  // sqrt(2*100*0.015) = sqrt(3) rad/s, BELOW the 2.0 rad/s commanded rate,
  // but still outside the 0.01 rad eps -- the goal is still running.
  planner.tick(1200, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, -1.5557963f));
  held = planner.takeCommand();
  checkTrue(planner.hasActiveCommand(),
            "0.015rad of heading error remains -- still outside eps, still running");
  checkFloatNear(held.control.twist.omega, -std::sqrt(3.0f), 1e-2f,
                 "anticipation caps the commanded rate as the HEADING stop's remaining angle shrinks");
  checkTrue(held.control.twist.omega > -1.9f,
            "the anticipatory cap measurably reduces the rate BEFORE the stop fires (086-003)");

  // Finally, reaching the target heading exactly still fires the stop.
  planner.tick(1300, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, -1.5707963f));
  planner.takeCommand();
  checkFalse(planner.hasActiveCommand(), "HEADING stop still fires once the target heading is reached");
  checkStrEq(planner.takeEvent().reason, "heading", "reason token is \"heading\"");
}

// 8. ROTATION goal_kind: RotationGoal.speed is an already-signed omega;
// relies on a caller-supplied ROTATION stop (encoder-arc based).
void scenarioRotationGoalUsesSignedSpeedAndCallerStop() {
  beginScenario("ROTATION goal_kind: signed omega pass-through, caller-supplied ROTATION stop");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::ROTATION;
  cmd.goal.rotation.speed = 1.5f;  // already-signed: CCW
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_ROTATION;
  cmd.stops_[0].a = 50.0f;  // [mm] target per-wheel arc
  cmd.style = msg::StopStyle::ABRUPT;  // isolate the stop-condition mechanics
                                        // from the SMOOTH ramp-down (covered
                                        // separately by the TIMED scenario)
  planner.apply(cmd, 0);

  // 084-005: RT always carries its own caller-supplied ROTATION stop (see
  // handleRT, motion_commands.cpp), so velocityShapedMode() always resolves
  // to TIMED here -- same reasoning as the TURN scenario above.
  checkTrue(planner.state().mode == msg::DriveMode::TIMED,
            "state().mode == TIMED (084-005: RT always self-terminates)");

  planner.tick(1000, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});  // baseline
  planner.takeCommand();

  planner.tick(1100, obsPosition(-20.0f), obsPosition(20.0f), msg::PoseEstimate{});
  planner.takeCommand();
  checkTrue(planner.hasActiveCommand(), "arc=20mm < 50mm threshold -- not yet");

  planner.tick(1200, obsPosition(-50.0f), obsPosition(50.0f), msg::PoseEstimate{});
  planner.takeCommand();
  checkFalse(planner.hasActiveCommand(), "arc=50mm reaches the ROTATION stop");
  checkStrEq(planner.takeEvent().reason, "rot", "reason token is \"rot\"");
}

// 8b. ROTATION goal_kind: ticket 086-003's terminal anticipation caps the
// commanded turn rate as the ROTATION stop's remaining per-wheel arc
// shrinks -- BEFORE the stop itself fires. yaw_acc_max is overridden down
// to 2.0 rad/s^2 (from generousConfig()'s 100 rad/s^2 default) so the cap's
// binding window (remaining < omega^2/(2*yaw_acc_max) = 0.5625mm here) is
// comfortably sized against round encoder-arc fixture values, and 1s ticks
// (mirroring 5b's own cadence choice) keep the ramp converging to each new
// target within a single tick despite the smaller yaw_acc_max (domegaMax =
// yaw_acc_max*dt = 2.0 rad/s, >= the 1.5 rad/s omega magnitude ramped
// through). See applyStopAnticipation()'s own comment (planner.cpp) on why
// STOP_ROTATION's remaining (a per-wheel arc, mm) is applied to this
// formula as a documented approximation rather than a unit-correct one.
//
// Ticket 087-009 update: the cap is now the same dead-time-compensated
// closed form 5b's own comment above describes, `reach =
// yaw_acc_max*kDeadTime` (kDeadTime = 0.040s) -- Tick 3's expected value
// below is recomputed against it (reach = 2.0*0.04 = 0.08 -> omegaCap =
// -0.08 + sqrt(0.08^2 + 2*2.0*0.3) = -0.08 + sqrt(1.2064) = 1.01836), not
// the un-compensated sqrt(2*2.0*0.3) = sqrt(1.2) = 1.09545 086-003
// measured. Tick 2's "does not bind" check is unaffected (omegaCap at
// remaining=50mm is still ~14.1, far above the 1.5 rad/s commanded rate
// either way).
void scenarioRotationGoalAnticipatesStopWithRateCap() {
  beginScenario("ROTATION goal_kind: terminal rate cap anticipates the ROTATION stop (086-003)");
  Subsystems::Planner planner;
  msg::PlannerConfig cfg = generousConfig();
  cfg.yaw_acc_max = 2.0f;  // [rad/s^2] -- see comment above on why
  planner.configure(cfg);

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::ROTATION;
  cmd.goal.rotation.speed = 1.5f;  // already-signed: CCW
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_ROTATION;
  cmd.stops_[0].a = 50.0f;  // [mm] target per-wheel arc
  cmd.style = msg::StopStyle::ABRUPT;
  planner.apply(cmd, 0);

  planner.tick(1000, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});  // baseline
  planner.takeCommand();

  // arc=0mm, 50mm remaining -- omegaCap = sqrt(2*2.0*50) = sqrt(200) rad/s,
  // way above the 1.5 rad/s commanded rate: the cap does not bind.
  planner.tick(2000, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});
  msg::DrivetrainCommand held = planner.takeCommand();
  checkFloatNear(held.control.twist.omega, 1.5f, 1e-2f,
                 "far from the stop -- anticipation cap does not bind");

  // arc=49.7mm, 0.3mm remaining -- dead-time-compensated omegaCap =
  // -0.08 + sqrt(0.08^2 + 2*2.0*0.3) = 1.01836 rad/s (087-009; see this
  // scenario's own comment above), BELOW the 1.5 rad/s commanded rate: the
  // cap now binds, well before the ROTATION stop itself would fire at 50mm.
  planner.tick(3000, obsPosition(-49.7f), obsPosition(49.7f), msg::PoseEstimate{});
  held = planner.takeCommand();
  checkTrue(planner.hasActiveCommand(), "arc=49.7mm -- short of the 50mm ROTATION stop, still running");
  checkFloatNear(held.control.twist.omega, 1.01836f, 1e-2f,
                 "anticipation caps the commanded rate as the ROTATION stop's remaining arc shrinks");
  checkTrue(held.control.twist.omega < 1.4f,
            "the anticipatory cap measurably reduces the rate BEFORE the stop fires (086-003)");

  // Finally, reaching the target arc exactly still fires the stop.
  planner.tick(4000, obsPosition(-50.0f), obsPosition(50.0f), msg::PoseEstimate{});
  planner.takeCommand();
  checkFalse(planner.hasActiveCommand(), "ROTATION stop still fires once the target arc is reached");
  checkStrEq(planner.takeEvent().reason, "rot", "reason token is \"rot\"");
}

// 9. GOTO_GOAL goal_kind, PURSUE-only path (ticket 084-004): when the
// initial bearing to the relative target is within turn_in_place_gate,
// PRE_ROTATE is skipped and PURSUE starts immediately, driving straight
// toward the (world-frame) target and completing -- a self-synthesized
// POSITION stop, reason "pos" -- once within arrive_tol. GOTO_GOAL accepts
// no caller-supplied stops_[] at all (unlike TURN/ROTATION above) -- see
// planner.h's class comment -- so PlannerConfig.turn_in_place_gate/
// arrive_tol (not a caller-supplied stop) drive this scenario.
void scenarioGotoGoalPursuesDirectlyWhenBearingWithinGate() {
  beginScenario("GOTO_GOAL: bearing within the gate skips PRE_ROTATE, PURSUE drives straight and arrives");
  Subsystems::Planner planner;
  msg::PlannerConfig cfg = generousConfig();
  cfg.turn_in_place_gate = 35.0f;  // [deg]
  cfg.arrive_tol = 10.0f;          // [mm]
  planner.configure(cfg);

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::GOTO_GOAL;
  cmd.goal.goto_goal.x = 300.0f;
  cmd.goal.goto_goal.y = 0.0f;   // bearing = 0 deg -- well within the 35 deg gate
  cmd.goal.goto_goal.speed = 120.0f;
  cmd.style = msg::StopStyle::ABRUPT;  // isolate the stop-condition mechanics
                                        // from the SMOOTH ramp-down (covered
                                        // separately by the TIMED scenario)
  planner.apply(cmd, 0);

  checkTrue(planner.state().mode == msg::DriveMode::GO_TO, "state().mode == GO_TO");
  checkFloatNear(planner.state().target_x, 300.0f, 1e-6f, "state().target_x reports the goal's relative x");
  checkFloatNear(planner.state().target_speed, 120.0f, 1e-6f, "state().target_speed reports the goal speed");

  planner.tick(1000, msg::MotorState{}, msg::MotorState{}, poseAt(0.0f, 0.0f, 0.0f));  // baseline; PURSUE starts immediately
  msg::DrivetrainCommand held = planner.takeCommand();
  checkFloatNear(held.control.twist.omega, 0.0f, 1e-6f, "target is straight ahead -- no steering needed");

  planner.tick(1100, msg::MotorState{}, msg::MotorState{}, poseAt(0.0f, 0.0f, 0.0f));
  planner.takeCommand();
  checkTrue(planner.hasActiveCommand(), "not yet at the target position");

  planner.tick(1200, msg::MotorState{}, msg::MotorState{}, poseAt(300.0f, 0.0f, 0.0f));
  planner.takeCommand();
  checkFalse(planner.hasActiveCommand(), "POSITION stop fires once within arrive_tol of the target");
  checkStrEq(planner.takeEvent().reason, "pos", "reason token is \"pos\"");
}

// 9b. GOTO_GOAL goal_kind, PRE_ROTATE -> PURSUE handoff (ticket 084-004):
// when the initial bearing exceeds turn_in_place_gate, PRE_ROTATE engages
// (spin-in-place toward the bearing) and hands off to PURSUE -- with no
// event, no ramp-down -- once the bearing gate is reached; PURSUE then
// drives to the (world-frame) target and completes with reason "pos".
void scenarioGotoGoalPreRotatesThenPursuesAndArrives() {
  beginScenario("GOTO_GOAL: bearing beyond the gate pre-rotates, hands off to PURSUE, and arrives");
  Subsystems::Planner planner;
  msg::PlannerConfig cfg = generousConfig();
  cfg.turn_in_place_gate = 35.0f;  // [deg]
  cfg.arrive_tol = 10.0f;          // [mm]
  planner.configure(cfg);

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::GOTO_GOAL;
  cmd.goal.goto_goal.x = 0.0f;
  cmd.goal.goto_goal.y = 300.0f;  // bearing = atan2(300, 0) = +90 deg -- well past the 35 deg gate
  cmd.goal.goto_goal.speed = 120.0f;
  cmd.style = msg::StopStyle::ABRUPT;  // isolate the stop-condition mechanics
                                        // from the SMOOTH ramp-down (covered
                                        // separately by the TIMED scenario)
  planner.apply(cmd, 0);

  checkTrue(planner.state().mode == msg::DriveMode::GO_TO, "state().mode == GO_TO");

  // Tick 1: baseline captured; PRE_ROTATE has been staged (v=0, omega > 0).
  planner.tick(1000, msg::MotorState{}, msg::MotorState{}, poseAt(0.0f, 0.0f, 0.0f));
  msg::DrivetrainCommand held = planner.takeCommand();
  checkFloatNear(held.control.twist.v_x, 0.0f, 1e-6f, "PRE_ROTATE commands v=0 (turn-in-place)");

  planner.tick(1100, msg::MotorState{}, msg::MotorState{}, poseAt(0.0f, 0.0f, 0.0f));
  planner.takeCommand();
  checkTrue(planner.state().body_twist.omega > 0.0f, "PRE_ROTATE spins CCW toward the +90 deg bearing");
  checkTrue(planner.hasActiveCommand(), "still pre-rotating");

  // Heading reaches the +90 deg bearing (within the 35 deg gate) -- PRE_ROTATE
  // hands off to PURSUE with no event (not a goal completion).
  planner.tick(1200, msg::MotorState{}, msg::MotorState{}, poseAt(0.0f, 0.0f, 1.5707963f));
  planner.takeCommand();
  checkTrue(planner.hasActiveCommand(), "PRE_ROTATE->PURSUE handoff keeps the goal active");
  checkFalse(planner.hasEvent(), "the PRE_ROTATE->PURSUE handoff queues no event");

  // Now facing the target (heading=+90 deg); PURSUE should drive it (v > 0).
  planner.tick(1300, msg::MotorState{}, msg::MotorState{}, poseAt(0.0f, 0.0f, 1.5707963f));
  planner.takeCommand();
  checkTrue(planner.state().body_twist.v_x > 0.0f, "PURSUE drives toward the target once facing it");

  // Arrive at the world-frame target: (x=0, y=300) rotated by the baseline
  // heading0=0 -- gTargetXWorld_/gTargetYWorld_ == (0, 300) directly.
  planner.tick(1400, msg::MotorState{}, msg::MotorState{}, poseAt(0.0f, 300.0f, 1.5707963f));
  planner.takeCommand();
  checkFalse(planner.hasActiveCommand(), "POSITION stop fires once within arrive_tol of the target");
  checkTrue(planner.hasEvent(), "arrival queues a completion event");
  checkStrEq(planner.takeEvent().reason, "pos", "reason token is \"pos\" for a fired STOP_POSITION");
}

// 10. STOP goal_kind: halts an in-progress goal immediately, with NO event
// queued (ticket 084-002 acceptance: "STOP halts immediately with no EVT").
void scenarioStopGoalKindHaltsSilently() {
  beginScenario("STOP goal_kind halts immediately with no EVT");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand velCmd;
  velCmd.goal_kind = msg::PlannerCommand::GoalKind::VELOCITY;
  velCmd.goal.velocity.v_x = 80.0f;
  planner.apply(velCmd, 0);
  planner.tick(1000, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
  planner.tick(1100, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
  planner.takeCommand();
  checkTrue(planner.hasActiveCommand(), "precondition: VELOCITY goal is running");

  msg::PlannerCommand stopCmd;
  stopCmd.setStop(true);
  planner.apply(stopCmd, 1200);

  checkFalse(planner.hasActiveCommand(), "STOP halts immediately -- no ramp-down phase");
  checkFalse(planner.hasEvent(), "STOP queues no event");
  checkTrue(planner.state().mode == msg::DriveMode::IDLE, "state().mode == IDLE after STOP");

  planner.tick(1300, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
  msg::DrivetrainCommand held = planner.takeCommand();
  checkFloatNear(held.control.twist.v_x, 0.0f, 1e-6f, "held output is zero after STOP");
}

// 11. STREAM goal_kind: open-ended like VELOCITY, no implicit stop synthesis.
void scenarioStreamGoalIsOpenEnded() {
  beginScenario("STREAM goal_kind is open-ended (no implicit stop)");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::STREAM;
  cmd.goal.stream.v_x = 60.0f;
  cmd.goal.stream.omega = 0.5f;
  planner.apply(cmd, 0);

  checkTrue(planner.state().mode == msg::DriveMode::STREAMING, "state().mode == STREAMING");

  for (int i = 0; i < 10; ++i) {
    planner.tick(1000 + i * 100, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
    planner.takeCommand();
  }
  checkTrue(planner.hasActiveCommand(), "STREAM never self-terminates");
  checkFalse(planner.hasEvent(), "no event queued for an open-ended STREAM goal");
}

}  // namespace

int main() {
  scenarioFreshPlannerIsIdle();
  scenarioHasCommandTakeCommandClearsEvenWhileIdle();
  scenarioConfigureReachesOwnedRamp();
  scenarioVelocityGoalRampsAndStaysOpenEnded();
  scenarioVelocityGoalWithStopReportsTimed();
  scenarioDistanceGoalFiresImplicitStopAbrupt();
  scenarioDistanceGoalStopFiredBeforeConvergenceForcesFreshDecel();
  scenarioDistanceGoalRuckigTraceNeverReverses();
  scenarioDistanceGoalDivergenceReplanCorrectsLaggingPlant();
  scenarioDistanceGoalGrossDivergenceReanchorsAfterStall();
  scenarioDistanceGoalGuardSkipsNearTargetBackwardReplan();
  scenarioTimedGoalSmoothRampDown();
  scenarioTurnGoalUsesSignedSpeedAndCallerStop();
  scenarioTurnGoalAnticipatesHeadingStopWithRateCap();
  scenarioRotationGoalUsesSignedSpeedAndCallerStop();
  scenarioRotationGoalAnticipatesStopWithRateCap();
  scenarioGotoGoalPursuesDirectlyWhenBearingWithinGate();
  scenarioGotoGoalPreRotatesThenPursuesAndArrives();
  scenarioStopGoalKindHaltsSilently();
  scenarioStreamGoalIsOpenEnded();

  if (g_failureCount == 0) {
    std::printf("OK: all Planner scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Planner scenarios\n", g_failureCount);
  return 1;
}
