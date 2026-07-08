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
//
// Ticket 089-004 note: TIMED/VELOCITY/STREAM migrate onto Motion::
// JerkTrajectory too (Decision 2's velocity-control "Pattern B", both
// linear_ AND rotational_ -- see planner.h's updated class comment).
// applyStopAnticipation() itself is UNCHANGED (089-003's own comment above
// is now stale on that point) but these three goal kinds no longer reach it
// -- only TURN/ROTATION still do, until ticket 005. New scenarios cover this
// ticket's own acceptance criteria: scenarioTimedGoalBothChannelsRuckigTrace
// NeverReverseAndCompleteViaTime (AC3/AC4, both channels sampled through
// cruise + the stop-triggered decel), scenarioTimedGoalStopFiresDuringRampUp
// ForcesFreshDecelNoReverse (AC3, a stop firing before cruise is reached),
// scenarioVelocityGoalCruiseSustainsPastRampDurationWithNoBookkeeping (AC2,
// sparse/irregular ticks proving pure extrapolation), and
// scenarioStreamGoalMidProfilePreemptionIsSeamlessNoReverse (the ticket's
// own STREAM re-target/preemption semantics).
//
// Ticket 089-005 note: TURN/ROTATION migrate onto the rotational
// Motion::JerkTrajectory channel too (Decision 9's position-control
// "Pattern A" -- DISTANCE's own pattern, ticket 003 -- not Pattern B),
// completing the sprint's goal-kind migration: `Planner::tick()`'s dispatch
// collapses to the clean `mode_ == GO_TO` binary (Decision 5's END state),
// and `applyStopAnticipation()` -- the 089-003/089-004 comments above's own
// "still serves TURN/ROTATION" note -- is DELETED in full (its last
// remaining callers migrated). The old rate-cap scenarios are RENAMED and
// REWRITTEN the same way 089-003 rewrote DISTANCE's:
// scenarioTurnGoalAnticipatesHeadingStopWithRateCap ->
// scenarioTurnGoalRuckigTraceNeverReverses (full trace, no reverse,
// completes via reason=heading) plus scenarioTurnGoalStopFiredBeforeConverg
// enceForcesFreshDecel (SMOOTH stop firing before rotational_'s own
// convergence -- mirrors 089-003's own scenario 5b); ditto for
// scenarioRotationGoalAnticipatesStopWithRateCap ->
// scenarioRotationGoalRuckigTraceNeverReverses. New scenarios also cover
// the Revision 2 divergence-triggered replan extended to the rotational
// channel: scenarioTurnGoalDivergenceReplanCorrectsLaggingPlant/
// scenarioTurnGoalGrossDivergenceReanchorsAfterStall/
// scenarioTurnGoalGuardSkipsNearTargetBackwardReplan (TURN's full
// normal/gross/guard-2 trio, fused-heading domain, mirroring 089-003's own
// D scenarios 5d/5e/5f) and scenarioRotationGoalDivergenceReplanCorrects
// LaggingPlant (RT's own normal case, proving rotationalArcScale_'s
// arc-mm<->rad conversion -- the one genuinely new risk this ticket's RT
// migration introduces over TURN's, since RT's STOP_ROTATION threshold and
// its Ruckig target are NOT the same number, unlike TURN's).
//
// Ticket 089-006 note: the CONSOLIDATION pass -- tickets 003-005 already
// built essentially all of this ticket's own acceptance bar into this file
// (the full no-reverse trace proofs for D/T/TURN/RT, plus the Revision 2
// divergence-replan coverage via option (b), synthetic-observation
// Planner-tier calls, not a sim-plant-lag knob) -- so this ticket adds only
// the two genuinely NEW scenarios the consolidation review found missing,
// rather than re-deriving what already existed: (1)
// scenarioVelocityGoalWithStopRuckigTraceNeverReversesThroughCruiseAndDecel
// (4d) -- AC2's own "at minimum, a spot-check for bare S/R" clause, a
// VELOCITY goal_kind (bare R's own goal kind) driven through cruise AND a
// stop-triggered decel, mirroring 6b's TIMED assertion style; (2)
// scenarioDistanceGoalGuardSkipsReplanOnceStopHasFired (5g) and
// scenarioTurnGoalGuardSkipsReplanOnceStopHasFired (7g) -- guard 1
// (stop-not-fired) had ONLY code-level enforcement (tick()'s own `if
// (!stopping_)` gate, planner.cpp) and no dedicated scenario proving it
// externally the way guard 2 (near-target no-reverse) already had via 5f/
// 7f; these two new scenarios close that gap using the SAME
// control-vs-test comparison technique 5f/7f established.

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

// 4c. [089-004, AC2] VELOCITY goal_kind: cruise SUSTAINS past the ramp-up
// trajectory's own duration with NO Planner-side "sustain" bookkeeping --
// Ruckig's own past-duration hold-at-final-state (jerk_trajectory.h's class
// comment; architecture-update.md (089) Decision 2) is what makes this work.
// Proven with deliberately SPARSE, unevenly-spaced ticks (unlike scenario 4
// above's steady 100ms cadence) reaching far past the ramp-up's own short
// duration -- if a Planner-side bookkeeping bug re-solved or decayed the
// cruise between ticks, an irregular tick cadence would be exactly the kind
// of case that exposes it; a pure sample() of one never-touched-since-
// apply() trajectory cannot fail this regardless of tick spacing.
void scenarioVelocityGoalCruiseSustainsPastRampDurationWithNoBookkeeping() {
  beginScenario(
      "089-004 AC2: VELOCITY goal's cruise sustains past its own ramp-up duration, "
      "sparsely-spaced ticks, no reverse/decay");
  Subsystems::Planner planner;
  planner.configure(generousConfig());  // a_max = 1000 mm/s^2

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::VELOCITY;
  cmd.goal.velocity.v_x = 50.0f;  // [mm/s] -- ramp-up duration: 50/1000 = 0.05s
  cmd.goal.velocity.omega = 0.0f;
  planner.apply(cmd, 0);

  planner.tick(0, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});  // baseline, elapsed=0
  planner.takeCommand();

  // Well past the 0.05s ramp-up -- the cruise should already be fully held.
  planner.tick(100, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
  msg::DrivetrainCommand first = planner.takeCommand();
  checkFloatNear(first.control.twist.v_x, 50.0f, 1e-2f, "cruise reached shortly after ramp-up");

  // A LARGE, irregular gap (no intervening ticks at all) well past the
  // ramp-up's own duration -- the extrapolated hold must read EXACTLY the
  // same cruise value, proving there is no per-tick decay/re-solve.
  planner.tick(5000, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
  msg::DrivetrainCommand second = planner.takeCommand();
  checkFloatNear(second.control.twist.v_x, 50.0f, 1e-2f,
                 "cruise still holds exactly at 50mm/s after a 4.9s gap with no intervening ticks");

  // Another large, DIFFERENT-sized gap -- same result again.
  planner.tick(19000, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
  msg::DrivetrainCommand third = planner.takeCommand();
  checkFloatNear(third.control.twist.v_x, 50.0f, 1e-2f,
                 "cruise still holds exactly at 50mm/s after a 14s gap -- pure extrapolation, "
                 "no Planner-side sustain bookkeeping");

  checkTrue(planner.hasActiveCommand(), "VELOCITY with no stops_[] never self-terminates");
  checkFalse(planner.hasEvent(), "no event queued across the whole sparse-tick trace");
}

// 4d. [089-006, AC2] VELOCITY goal_kind (the SAME internal goal kind bare
// `R` stages -- motion_commands.cpp's handleR: "Posts a VELOCITY goal
// exactly like a bare S") WITH a caller-supplied stop=: this ticket's own
// "at minimum, a spot-check for bare S/R" half of AC2 (SUC-003), applied to
// the one non-TIMED goal kind 6b's own full-trace test below does not
// directly exercise (STREAM/bare-S's own analogous spot-check is 11b's
// mid-profile-preemption scenario further down). Samples the full commanded
// (v_x, omega) trace across cruise AND the stop-triggered re-solve,
// asserting >= 0 on BOTH channels throughout -- mirrors 6b's own assertion
// style exactly, for VELOCITY instead of TIMED.
void scenarioVelocityGoalWithStopRuckigTraceNeverReversesThroughCruiseAndDecel() {
  beginScenario(
      "089-006 AC2: VELOCITY goal (bare R's own goal kind) with a caller stop -- full "
      "(v_x, omega) trace across cruise + stop-triggered decel never reverses");
  Subsystems::Planner planner;
  planner.configure(generousConfig());  // a_max=a_decel=1000mm/s^2, yaw_acc_max=100rad/s^2

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::VELOCITY;
  cmd.goal.velocity.v_x = 150.0f;  // [mm/s] -- ramp-up: 150/1000 = 0.15s
  cmd.goal.velocity.omega = 1.0f;  // [rad/s] -- ramp-up: 1.0/100 = 0.01s
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_TIME;
  cmd.stops_[0].a = 300.0f;  // [ms] -- mirrors a bounded `R 150 <radius> stop=t:300`
  std::strncpy(cmd.corr_id, "rspot1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  checkTrue(planner.state().mode == msg::DriveMode::TIMED,
            "a caller stop makes VELOCITY self-terminating (084-005 velocityShapedMode())");

  planner.tick(0, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});  // baseline, elapsed=0
  planner.takeCommand();

  bool everReversedV = false;
  bool everReversedOmega = false;
  bool completedWithTime = false;
  uint32_t t = 0;
  for (int i = 0; i < 60; ++i) {  // up to 1200ms -- comfortably past the 300ms fire + both decels
    t += 20;
    planner.tick(t, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
    msg::DrivetrainCommand held = planner.takeCommand();
    if (held.control.twist.v_x < -0.5f) everReversedV = true;
    if (held.control.twist.omega < -0.5f) everReversedOmega = true;
    if (!planner.hasActiveCommand()) {
      completedWithTime = (std::strcmp(planner.takeEvent().reason, "time") == 0);
      break;
    }
  }
  checkFalse(everReversedV,
             "the linear channel's commanded v_x never goes negative (bare R spot check)");
  checkFalse(everReversedOmega,
             "the rotational channel's commanded omega never goes negative (bare R spot check)");
  checkTrue(completedWithTime, "still completes via reason=time once both decels converge");
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

// 5g. [089-006 Revision 2 AC] DISTANCE goal_kind: guard 1 (stop-not-fired)
// blocks the divergence-triggered replan entirely once the goal's own
// SMOOTH stop-triggered decel has armed (stopping_ == true) -- even a
// wildly divergent synthetic observation fed AFTER that point must not
// perturb the commanded decel-to-rest trace, since maybeReplanDistance()
// is called ONLY while !stopping_ (planner.cpp's tick(), "guard 1"
// comment). Proven the SAME way guard 2 is (scenario 5f above): compare
// against an undisturbed control run at the identical tick -- if guard 1
// held, the commanded v_x is unaffected by the synthetic observation.
void scenarioDistanceGoalGuardSkipsReplanOnceStopHasFired() {
  beginScenario(
      "089-006 Revision 2 AC: guard 1 (stop-not-fired) blocks any replan once the SMOOTH "
      "stop-triggered decel has armed (DISTANCE)");
  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::DISTANCE;
  cmd.goal.distance.distance = 500.0f;
  cmd.goal.distance.speed = 200.0f;
  // cmd.style left at its default (SMOOTH) -- ABRUPT never arms stopping_.
  std::strncpy(cmd.corr_id, "guard1d", sizeof(cmd.corr_id) - 1);

  // Control: an undisturbed encoder feed that reaches the 500mm
  // STOP_DISTANCE threshold at t=2.5s (before the undisturbed plan's own
  // ~2.7s natural convergence, see scenario 5c -- arms stopping_ via
  // armDistanceStopDecel()), then continues tracking normally one more tick
  // into the decel.
  Subsystems::Planner control;
  control.configure(generousConfig());
  control.apply(cmd, 0);
  control.tick(0, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});
  control.takeCommand();
  control.tick(2500, obsPosition(500.0f), obsPosition(500.0f), msg::PoseEstimate{});
  control.takeCommand();
  control.tick(2600, obsPosition(500.0f), obsPosition(500.0f), msg::PoseEstimate{});
  msg::DrivetrainCommand controlHeld = control.takeCommand();

  Subsystems::Planner test;
  test.configure(generousConfig());
  test.apply(cmd, 0);
  test.tick(0, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});
  test.takeCommand();
  test.tick(2500, obsPosition(500.0f), obsPosition(500.0f), msg::PoseEstimate{});
  test.takeCommand();
  checkTrue(test.hasActiveCommand(),
            "precondition: SMOOTH stop armed stopping_, goal not yet complete");
  // A synthetic observation showing only 300mm traveled -- as if the plant
  // had snapped backward from the 500mm the stop already fired at, a
  // divergence well past kGrossDivergenceThreshold -- fed AFTER the stop
  // has already fired. Guard 1 must skip any replan regardless of this
  // observation. (Confirmed by direct mutation testing during this
  // scenario's own authoring: with tick()'s `if (!stopping_)` gate
  // deliberately removed, this exact observation flips the commanded v_x
  // from 100mm/s to 0mm/s -- proving this scenario has real detection
  // power, not just a vacuously-passing assertion; a smaller snap, e.g.
  // 50mm, does NOT discriminate here, since armDistanceStopDecel()'s own
  // velocity-control re-solve does not track a meaningful position, making
  // maybeReplanDistance()'s plan-remaining estimate large regardless of the
  // observation -- only a larger, mid-range snap reliably crosses the GROSS
  // threshold.)
  test.tick(2600, obsPosition(300.0f), obsPosition(300.0f), msg::PoseEstimate{});
  msg::DrivetrainCommand testHeld = test.takeCommand();

  checkFloatNear(testHeld.control.twist.v_x, controlHeld.control.twist.v_x, 1.0f,
                 "guard 1 blocks the replan once stopping_ is armed -- commanded v_x matches the "
                 "undisturbed control despite a huge synthetic divergence");
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

// 6b. [089-004, AC3/AC4] TIMED goal_kind: both channels (`v_x` AND `omega`
// simultaneously, architecture-update.md (089) Decision 1 -- two independent
// velocity-control Ruckig solves that may converge at slightly different
// times) sampled across the FULL apply()+tick() staging path -- cruise, then
// the stop-triggered decel-to-zero re-solve -- never reverses sign on either
// channel, and completes via the implicit STOP_TIME (reason="time"), not the
// SMOOTH soft deadline. Mirrors 089-003's scenarioDistanceGoalRuckigTrace
// NeverReverses (planner_harness.cpp, ticket 003) but for the velocity-
// control "Pattern B" this ticket adds.
void scenarioTimedGoalBothChannelsRuckigTraceNeverReverseAndCompleteViaTime() {
  beginScenario(
      "089-004 AC3/AC4: TIMED goal's full (v_x, omega) trace never reverses through "
      "cruise + stop-triggered decel, completes via reason=time");
  Subsystems::Planner planner;
  planner.configure(generousConfig());  // a_max=a_decel=1000mm/s^2, yaw_acc_max=100rad/s^2

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::TIMED;
  cmd.goal.timed.v_x = 200.0f;   // [mm/s] linear ramp-up: 200/1000 = 0.2s
  cmd.goal.timed.omega = 1.5f;   // [rad/s] rotational ramp-up: 1.5/100 = 0.015s -- deliberately a
                                 // MUCH shorter ramp than the linear channel's, so the two
                                 // channels' own decel-to-zero re-solves finish at different times
                                 // (Decision 1's own "may finish at slightly different times" note).
  cmd.goal.timed.duration = 400;  // [ms] -- implicit STOP_TIME; fires well after BOTH channels
                                  // have already reached cruise and are holding via extrapolation.
  std::strncpy(cmd.corr_id, "bothchan1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  planner.tick(0, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});  // baseline, elapsed=0
  planner.takeCommand();

  bool everReversedV = false;
  bool everReversedOmega = false;
  bool completedWithTime = false;
  uint32_t t = 0;
  for (int i = 0; i < 60; ++i) {  // up to 1200ms -- comfortably past the 400ms fire + both decels
    t += 20;
    planner.tick(t, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
    msg::DrivetrainCommand held = planner.takeCommand();
    if (held.control.twist.v_x < -0.5f) everReversedV = true;
    if (held.control.twist.omega < -0.5f) everReversedOmega = true;
    if (!planner.hasActiveCommand()) {
      completedWithTime = (std::strcmp(planner.takeEvent().reason, "time") == 0);
      break;
    }
  }
  checkFalse(everReversedV, "the linear channel's commanded v_x never goes negative");
  checkFalse(everReversedOmega, "the rotational channel's commanded omega never goes negative");
  checkTrue(completedWithTime, "still completes via reason=time once both decels converge");
}

// 6c. [089-004, AC3] TIMED goal_kind: a stop firing WHILE the linear
// channel's own ramp-up is still in progress (not yet at cruise, nonzero
// acceleration) forces a fresh decel-to-zero re-solve from that PARTIAL
// state -- proving Decision 2's "regardless of whether it fires early,
// exactly on plan, or late... converges safely" claim for the velocity-
// control mode, the same property 5b (planner_harness.cpp, ticket 003)
// proved for DISTANCE's position-control mode. A caller-supplied `stop=t:`
// clause (not the implicit duration stop) fires at 100ms, well inside the
// commanded 1000 mm/s target's own 1.0s ramp-up window (a_max=1000mm/s^2).
void scenarioTimedGoalStopFiresDuringRampUpForcesFreshDecelNoReverse() {
  beginScenario(
      "089-004 AC3: a stop firing mid-ramp-up (not yet at cruise) still forces a "
      "non-reversing decel-to-zero");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::TIMED;
  cmd.goal.timed.v_x = 1000.0f;  // [mm/s] -- at v_body_max; ramp-up takes a full 1.0s
  cmd.goal.timed.omega = 0.0f;
  cmd.goal.timed.duration = 0;  // no implicit STOP_TIME -- the caller stop below fires first
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_TIME;
  cmd.stops_[0].a = 100.0f;  // [ms] -- fires at 1/10th of the ramp-up's own duration
  std::strncpy(cmd.corr_id, "midramp1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  planner.tick(0, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});  // baseline, elapsed=0
  planner.takeCommand();

  bool everReversed = false;
  bool completedWithTime = false;
  uint32_t t = 0;
  for (int i = 0; i < 60; ++i) {  // up to 1200ms
    t += 20;
    planner.tick(t, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
    msg::DrivetrainCommand held = planner.takeCommand();
    if (held.control.twist.v_x < -0.5f) everReversed = true;
    if (!planner.hasActiveCommand()) {
      completedWithTime = (std::strcmp(planner.takeEvent().reason, "time") == 0);
      break;
    }
  }
  checkFalse(everReversed, "a stop fired mid-ramp-up still decelerates to zero with no reverse");
  checkTrue(completedWithTime, "completes via reason=time once the forced decel converges");
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

// 7b. [089-005 REWRITE -- was scenarioTurnGoalAnticipatesHeadingStopWithRateCap,
// 086-003's closed-form rate-anticipation cap test] TURN goal_kind now
// stages a position-control Motion::JerkTrajectory solve-to-rest on the
// rotational channel at apply() time instead of a ramp_ target --
// applyStopAnticipation()'s STOP_HEADING cap this scenario used to test is
// DELETED (the function itself is gone in full, planner.cpp). The property
// this scenario now tests mirrors 089-003's own
// scenarioDistanceGoalRuckigTraceNeverReverses (AC3/AC7 for this ticket's
// own TURN/ROTATION migration): the full commanded omega trace, sampled
// across the REAL apply()+tick() staging path with a closely-tracking
// (undisturbed) fused-heading feed, never reverses relative to the
// commanded turn direction and completes via the goal's own STOP_HEADING
// (not any safety net).
void scenarioTurnGoalRuckigTraceNeverReverses() {
  beginScenario(
      "089-005 AC (full trace): TURN's full commanded omega trace never reverses "
      "(real apply()+tick() path)");
  Subsystems::Planner planner;
  planner.configure(generousConfig());  // yaw_acc_max = 100 rad/s^2, yaw_rate_max = 10 rad/s

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::TURN;
  cmd.goal.turn.speed = -2.0f;  // already-signed: CW -- becomes rotational_'s max_velocity
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_HEADING;
  cmd.stops_[0].a = -1.5707963f;  // [rad] target delta: -90 deg -- rotational_'s Ruckig target
  cmd.stops_[0].b = 0.02f;        // [rad] eps
  cmd.style = msg::StopStyle::ABRUPT;  // isolate the trace from the SMOOTH
                                        // ramp-down (covered by the scenario below)
  std::strncpy(cmd.corr_id, "turntrace1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  // Hand-derived trapezoid (j_max == 0 -> Ruckig's own infinite-jerk
  // sentinel -> an exact trapezoid, matching 089-003's own closed form) for
  // a CLOSELY-TRACKING fused-heading feed -- the undisturbed case (no
  // divergence-replan expected; covered separately below).
  const float kAMax = 100.0f;         // [rad/s^2]
  const float kOmegaMax = 2.0f;       // [rad/s]
  const float kAngle = 1.5707963f;    // [rad]
  const float kTAccel = kOmegaMax / kAMax;                      // 0.02s
  const float kDAccel = 0.5f * kAMax * kTAccel * kTAccel;       // 0.02 rad
  const float kDCruise = kAngle - 2.0f * kDAccel;               // 1.5307963 rad
  const float kTCruise = kDCruise / kOmegaMax;                  // 0.76539815s
  const float kTDecelStart = kTAccel + kTCruise;                // 0.78539815s
  const float kTTotal = kTDecelStart + kTAccel;                 // 0.80539815s

  auto trapezoidAngle = [&](float time) -> float {
    if (time <= 0.0f) return 0.0f;
    if (time < kTAccel) return 0.5f * kAMax * time * time;
    if (time < kTDecelStart) return kDAccel + kOmegaMax * (time - kTAccel);
    if (time < kTTotal) {
      float s = time - kTDecelStart;
      return kDAccel + kDCruise + kOmegaMax * s - 0.5f * kAMax * s * s;
    }
    return kAngle;
  };

  planner.tick(0, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));  // baseline
  planner.takeCommand();

  bool everReversed = false;
  bool completedWithHeading = false;
  for (int ms = 20; ms <= 1500; ms += 20) {
    float h = -trapezoidAngle(static_cast<float>(ms) * 0.001f);  // target is negative (CW)
    planner.tick(static_cast<uint32_t>(ms), msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, h));
    msg::DrivetrainCommand held = planner.takeCommand();
    if (held.control.twist.omega > 0.5f) everReversed = true;
    if (!planner.hasActiveCommand()) {
      completedWithHeading = (std::strcmp(planner.takeEvent().reason, "heading") == 0);
      break;
    }
  }
  checkFalse(everReversed, "commanded omega never goes positive (reverse of the commanded CW turn)");
  checkTrue(completedWithHeading,
            "goal completes via STOP_HEADING (reason=heading), not any safety net");
}

// 7c. [089-005, ticket item 4 analog] TURN goal_kind: a SMOOTH-style stop
// firing BEFORE rotational_'s own plan has naturally converged to rest
// forces a fresh, non-reversing decel-to-rest (armRotationalStopDecel()),
// seeded from the channel's own current state (never a measured
// observation) -- mirrors 089-003's own
// scenarioDistanceGoalStopFiredBeforeConvergenceForcesFreshDecel. SMOOTH
// (not ABRUPT, unlike the trace scenario above) is used deliberately --
// ABRUPT bypasses armRotationalStopDecel() entirely (planner.cpp's tick()).
void scenarioTurnGoalStopFiredBeforeConvergenceForcesFreshDecel() {
  beginScenario(
      "089-005: SMOOTH stop firing before rotational_'s own convergence forces a "
      "fresh decel-to-rest, no reverse");
  Subsystems::Planner planner;
  planner.configure(generousConfig());  // yaw_acc_max = 100 rad/s^2

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::TURN;
  cmd.goal.turn.speed = -2.0f;  // [rad/s] -- undisturbed plan duration ~0.805s
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_HEADING;
  cmd.stops_[0].a = -1.5707963f;  // [rad]
  cmd.stops_[0].b = 0.02f;        // [rad] eps
  // cmd.style left at its default (SMOOTH) -- see comment above on why.
  std::strncpy(cmd.corr_id, "turnfast1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  planner.tick(0, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));  // baseline
  planner.takeCommand();

  // Real heading "overshoots" the plan: it advances at a constant 3.0
  // rad/s (faster than the plan's own 2.0 rad/s cruise), reaching the
  // -90deg target at t=1.5707963/3.0=0.524s -- well before the undisturbed
  // plan's own ~0.805s natural convergence -- forcing
  // armRotationalStopDecel()'s REAL re-solve branch.
  const float kPlantRate = 3.0f;  // [rad/s]
  bool everReversed = false;
  bool sawSmoothArm = false;
  bool completedWithHeading = false;
  uint32_t t = 0;
  for (int i = 0; i < 120; ++i) {  // up to 6s -- comfortably past arming + convergence
    t += 50;
    float h = std::max(-1.5707963f, -kPlantRate * static_cast<float>(t) * 0.001f);
    planner.tick(t, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, h));
    msg::DrivetrainCommand held = planner.takeCommand();
    if (held.control.twist.omega > 0.5f) everReversed = true;
    if (planner.hasActiveCommand() && h <= -1.5707963f) sawSmoothArm = true;
    if (!planner.hasActiveCommand()) {
      completedWithHeading = (std::strcmp(planner.takeEvent().reason, "heading") == 0);
      break;
    }
  }
  checkFalse(everReversed, "the forced decel-to-rest re-solve never reverses");
  checkTrue(sawSmoothArm,
            "SMOOTH style does not complete on the same tick the stop fires -- ramp/decel-down observed");
  checkTrue(completedWithHeading, "still completes via reason=heading once the decel-to-rest converges");
}

// 7d. [089-005 Revision 2] TURN goal_kind: a lagging (slipping) plant --
// injected at the test-fixture level, mirroring 089-003's own
// scenarioDistanceGoalDivergenceReplanCorrectsLaggingPlant -- triggers
// NORMAL divergence-triggered retarget()s on the rotational channel and
// still completes via the goal's OWN STOP_HEADING, not a safety net
// (architecture-update.md (089) Decision 10, extended to TURN/ROTATION by
// this ticket; AC's crisp sim-level proof). Unlike DISTANCE, TURN has no
// IMPLICIT time net of its own (planner.h's class comment -- only
// caller-supplied stops_[] apply) -- a caller-supplied STOP_TIME stop is
// added here purely as the test's own safety bound, standing in for what
// would otherwise be "never completes" if the fix did not work.
//
// Same output-dead-time plant model as 089-003's own scenario (kOutputHops
// == kDeadTime's own 2 hops) -- the real Planner->driveIn->Drivetrain->
// motorIn->Hardware pipeline latency the dead-time projection is built
// against.
void scenarioTurnGoalDivergenceReplanCorrectsLaggingPlant() {
  beginScenario(
      "089-005 AC: a lagging plant triggers NORMAL retarget()s on the rotational channel "
      "and still completes via heading, not a safety net");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::TURN;
  cmd.goal.turn.speed = -2.0f;  // [rad/s]
  cmd.stops_count = 2;
  cmd.stops_[0].kind = msg::StopKind::STOP_HEADING;
  cmd.stops_[0].a = -1.5707963f;  // [rad]
  cmd.stops_[0].b = 0.05f;        // [rad] eps
  cmd.stops_[1].kind = msg::StopKind::STOP_TIME;
  cmd.stops_[1].a = 10000.0f;  // [ms] test-owned safety bound -- see comment above
  cmd.style = msg::StopStyle::ABRUPT;
  std::strncpy(cmd.corr_id, "turnlag1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  planner.tick(0, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));
  planner.takeCommand();

  // Closed-loop plant simulation: the plant only achieves kSlip (85%) of
  // whatever omega Planner commanded kOutputHops ticks ago.
  const float kSlip = 0.85f;
  const float kDt = 0.02f;  // [s] 20ms tick
  const int kOutputHops = 2;
  float delayBuf[kOutputHops] = {0.0f, 0.0f};
  int delayHead = 0;
  float heading = 0.0f;
  bool everReversed = false;
  bool completedWithHeading = false;
  uint32_t t = 0;
  for (int i = 0; i < 750; ++i) {  // up to 15s -- comfortably past the test's own 10s time net
    t += static_cast<uint32_t>(kDt * 1000.0f);
    planner.tick(t, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, heading));
    msg::DrivetrainCommand held = planner.takeCommand();
    float omegaCmd = held.control.twist.omega;
    if (omegaCmd > 0.5f) everReversed = true;
    float omegaApplied = delayBuf[delayHead];
    delayBuf[delayHead] = omegaCmd;
    delayHead = (delayHead + 1) % kOutputHops;
    heading += kSlip * omegaApplied * kDt;
    if (!planner.hasActiveCommand()) {
      completedWithHeading = (std::strcmp(planner.takeEvent().reason, "heading") == 0);
      break;
    }
  }
  checkFalse(everReversed, "the divergence-corrected trace never reverses despite the lagging plant");
  checkTrue(completedWithHeading,
            "a lagging plant completes via STOP_HEADING (reason=heading) thanks to the divergence "
            "replan, not the test's own STOP_TIME safety bound");
}

// 7e. [089-005 Revision 2] TURN goal_kind: a stalled-then-freed plant (a
// genuine departure from the plan) triggers the GROSS divergence path
// (reanchor(), not retarget()) on the rotational channel and still
// completes via heading, with no reverse -- mirrors 089-003's own
// scenarioDistanceGoalGrossDivergenceReanchorsAfterStall.
void scenarioTurnGoalGrossDivergenceReanchorsAfterStall() {
  beginScenario(
      "089-005 AC: a stalled-then-freed plant (gross divergence) triggers reanchor() on the "
      "rotational channel and still completes via heading");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::TURN;
  cmd.goal.turn.speed = -2.0f;  // [rad/s]
  cmd.stops_count = 2;
  cmd.stops_[0].kind = msg::StopKind::STOP_HEADING;
  cmd.stops_[0].a = -1.5707963f;  // [rad]
  cmd.stops_[0].b = 0.05f;        // [rad] eps
  cmd.stops_[1].kind = msg::StopKind::STOP_TIME;
  cmd.stops_[1].a = 10000.0f;  // [ms] test-owned safety bound
  cmd.style = msg::StopStyle::ABRUPT;
  std::strncpy(cmd.corr_id, "turnstall1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  planner.tick(0, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));
  planner.takeCommand();

  // Frozen (wedged) for the first second, then tracks the commanded omega
  // normally -- the undisturbed plan would think it has covered a large
  // fraction of the turn by t=1s while the measured heading shows 0, a
  // divergence well past kRotGrossDivergenceThreshold.
  const float kDt = 0.02f;  // [s]
  const int kOutputHops = 2;
  float delayBuf[kOutputHops] = {0.0f, 0.0f};
  int delayHead = 0;
  float heading = 0.0f;
  bool everReversed = false;
  bool completedWithHeading = false;
  uint32_t t = 0;
  for (int i = 0; i < 750; ++i) {
    t += static_cast<uint32_t>(kDt * 1000.0f);
    planner.tick(t, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, heading));
    msg::DrivetrainCommand held = planner.takeCommand();
    float omegaCmd = held.control.twist.omega;
    if (omegaCmd > 0.5f) everReversed = true;
    float omegaApplied = delayBuf[delayHead];
    delayBuf[delayHead] = omegaCmd;
    delayHead = (delayHead + 1) % kOutputHops;
    if (t >= 1000) {
      heading += omegaApplied * kDt;  // freed -- full tracking from here on
    }
    // else: stalled -- heading stays frozen at 0 regardless of omegaCmd.
    if (!planner.hasActiveCommand()) {
      completedWithHeading = (std::strcmp(planner.takeEvent().reason, "heading") == 0);
      break;
    }
  }
  checkFalse(everReversed, "the gross-divergence reanchor's trace never reverses");
  checkTrue(completedWithHeading, "a stalled-then-freed plant still completes via heading after the reanchor");
}

// 7f. [089-005 Revision 2] TURN goal_kind: guard 2 (no-reverse-target) skips
// a would-be-backward replan near the target -- mirrors 089-003's own
// scenarioDistanceGoalGuardSkipsNearTargetBackwardReplan.
void scenarioTurnGoalGuardSkipsNearTargetBackwardReplan() {
  beginScenario(
      "089-005 AC: guard 2 (no-reverse-target) skips a would-be-backward replan near the "
      "target, rotational channel");
  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::TURN;
  cmd.goal.turn.speed = -2.0f;
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_HEADING;
  cmd.stops_[0].a = -1.5707963f;
  cmd.stops_[0].b = 0.001f;  // tight eps -- keeps the goal running at t=300ms below
  cmd.style = msg::StopStyle::ABRUPT;

  // Control: an undisturbed, closely-tracking fused-heading feed (matching
  // the plan's own trapezoid position exactly at t=300ms, well inside the
  // ~0.805s undisturbed duration) -- divergence stays ~0, no replan for
  // either planner.
  Subsystems::Planner control;
  control.configure(generousConfig());
  control.apply(cmd, 0);
  control.tick(0, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));
  control.takeCommand();
  // t=300ms is inside the cruise phase (0.02s accel, 0.7654s cruise) --
  // position = 0.02 + 2.0*(0.3-0.02) = 0.58 rad.
  control.tick(300, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, -0.58f));
  msg::DrivetrainCommand controlHeld = control.takeCommand();

  Subsystems::Planner test;
  test.configure(generousConfig());
  test.apply(cmd, 0);
  test.tick(0, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));
  test.takeCommand();
  // A synthetic observation showing the heading already 1.57rad into the
  // turn at t=300ms -- NOT yet past the 1.5707963rad target (still
  // NOT_FIRED, |error| = 0.0007963 > eps=0.001? actually within eps -- use
  // a value just short of firing but whose dead-time-projected remaining is
  // <= 0: guard 2 must skip the replan entirely, leaving this tick's
  // commanded omega identical to the undisturbed control's -- despite a
  // large RAW divergence against the plan's own still-early position that
  // would otherwise (sans guard 2) trigger a gross reanchor pointing
  // backward (positive omega, reversing the commanded CW turn).
  test.tick(300, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, -1.5637963f));
  msg::DrivetrainCommand testHeld = test.takeCommand();

  checkTrue(test.hasActiveCommand(), "0.007rad short of target -- STOP_HEADING has not fired yet");
  checkFloatNear(testHeld.control.twist.omega, controlHeld.control.twist.omega, 1.0f,
                 "guard 2 skips the near-target replan -- commanded omega matches the undisturbed "
                 "control");
}

// 7g. [089-006 Revision 2 AC] TURN goal_kind: guard 1 (stop-not-fired)
// blocks the divergence-triggered replan on the rotational channel too,
// once the SMOOTH stop-triggered decel has armed -- mirrors
// scenarioDistanceGoalGuardSkipsReplanOnceStopHasFired() above (5g), for
// the rotational channel/TURN's own goal kind.
void scenarioTurnGoalGuardSkipsReplanOnceStopHasFired() {
  beginScenario(
      "089-006 Revision 2 AC: guard 1 (stop-not-fired) blocks any replan once the SMOOTH "
      "stop-triggered decel has armed (TURN)");
  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::TURN;
  cmd.goal.turn.speed = -2.0f;  // [rad/s] -- undisturbed plan duration ~0.805s (see scenario 7b)
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_HEADING;
  cmd.stops_[0].a = -1.5707963f;  // [rad]
  cmd.stops_[0].b = 0.02f;        // [rad] eps
  // cmd.style left at its default (SMOOTH) -- ABRUPT never arms stopping_.
  std::strncpy(cmd.corr_id, "guard1t", sizeof(cmd.corr_id) - 1);

  // Control: an undisturbed fused-heading feed that reaches the -90deg
  // target at t=524ms (mirrors 7c's own kPlantRate=3.0rad/s overshoot-plant
  // framing: 1.5707963/3.0 = 0.5236s -- before the undisturbed plan's own
  // ~0.805s natural convergence, arming stopping_ via
  // armRotationalStopDecel()), then continues tracking normally 10ms into
  // the decel -- yaw_acc_max=100rad/s^2 makes this decel very fast (from
  // ~2rad/s to 0 in ~20ms), so the probe must land WELL inside that short
  // window (unlike DISTANCE's 100ms-later probe in 5g above) to catch the
  // channel still actively decelerating rather than already converged.
  Subsystems::Planner control;
  control.configure(generousConfig());
  control.apply(cmd, 0);
  control.tick(0, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));
  control.takeCommand();
  control.tick(524, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, -1.5707963f));
  control.takeCommand();
  control.tick(534, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, -1.5707963f));
  msg::DrivetrainCommand controlHeld = control.takeCommand();

  Subsystems::Planner test;
  test.configure(generousConfig());
  test.apply(cmd, 0);
  test.tick(0, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, 0.0f));
  test.takeCommand();
  test.tick(524, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, -1.5707963f));
  test.takeCommand();
  checkTrue(test.hasActiveCommand(),
            "precondition: SMOOTH stop armed stopping_, goal not yet complete");
  // A synthetic observation showing the heading only -0.5rad along -- as if
  // the plant had fallen well behind the -90deg (-1.5707963rad) target
  // already reached, a divergence well past kRotGrossDivergenceThreshold --
  // fed AFTER the stop has already fired. Guard 1 must skip any replan
  // regardless of this observation. (Confirmed by direct mutation testing
  // during this scenario's own authoring: with tick()'s `if (!stopping_)`
  // gate deliberately removed, this exact observation flips the commanded
  // omega from -1.0rad/s (control) to a fully-converged 0rad/s (test) at
  // this probe offset -- a 1.0rad/s gap, proving this scenario has real
  // detection power; the tolerance below is deliberately tightened to
  // 0.3rad/s (rather than reusing 5g's 1.0mm/s-scaled tolerance verbatim) so
  // that exact 1.0rad/s gap cannot land ON the tolerance boundary and go
  // undetected. A probe 100ms after arming, mirroring 5g's own DISTANCE
  // timing, does NOT discriminate here: yaw_acc_max=100's decel is so fast
  // (~20ms) that BOTH the guarded and unguarded builds have already fully
  // converged to omega=0 by then, regardless of guard 1 -- only a probe well
  // inside that short decel window catches the channel still actively
  // decelerating.)
  test.tick(534, msg::MotorState{}, msg::MotorState{}, poseAt(0, 0, -0.5f));
  msg::DrivetrainCommand testHeld = test.takeCommand();

  checkFloatNear(testHeld.control.twist.omega, controlHeld.control.twist.omega, 0.3f,
                 "guard 1 blocks the replan once stopping_ is armed -- commanded omega matches "
                 "the undisturbed control despite a huge synthetic divergence");
}

// 8. ROTATION goal_kind: RotationGoal.speed is an already-signed omega;
// relies on a caller-supplied ROTATION stop (encoder-arc based).
// 089-005: RotationGoal.angle (previously informational only) is now
// rotational_'s own Ruckig target (Decision 9) -- set here to 0.8 rad,
// consistent with the 50mm arc threshold below via an assumed 62.5mm/rad
// trackwidth-derived scale (arbitrary but internally consistent; Planner
// itself never reads a trackwidth -- see class comment). Adds a mid-run
// omega assertion (absent from the pre-089-005 version of this scenario)
// so this test actually exercises the new Ruckig-driven pass-through, not
// just the ROTATION stop's own (encoder-driven, Ruckig-independent)
// completion signal.
void scenarioRotationGoalUsesSignedSpeedAndCallerStop() {
  beginScenario("ROTATION goal_kind: signed omega pass-through, caller-supplied ROTATION stop");
  Subsystems::Planner planner;
  planner.configure(generousConfig());  // yaw_acc_max = 100 rad/s^2, yaw_rate_max = 10 rad/s

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::ROTATION;
  cmd.goal.rotation.speed = 1.5f;   // already-signed: CCW -- rotational_'s max_velocity
  cmd.goal.rotation.angle = 0.8f;   // [rad] rotational_'s Ruckig target (089-005)
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

  // Rotational channel: target=0.8rad, maxVelocity=1.5rad/s, aMax=100 ->
  // ramp-up takes 0.015s -- fully at cruise (1.5 rad/s) well before this
  // 100ms tick.
  planner.tick(1100, obsPosition(-20.0f), obsPosition(20.0f), msg::PoseEstimate{});
  msg::DrivetrainCommand held = planner.takeCommand();
  checkTrue(planner.hasActiveCommand(), "arc=20mm < 50mm threshold -- not yet");
  checkFloatNear(held.control.twist.omega, 1.5f, 1e-2f,
                 "rotational channel reaches the commanded cruise rate (089-005)");

  planner.tick(1200, obsPosition(-50.0f), obsPosition(50.0f), msg::PoseEstimate{});
  planner.takeCommand();
  checkFalse(planner.hasActiveCommand(), "arc=50mm reaches the ROTATION stop");
  checkStrEq(planner.takeEvent().reason, "rot", "reason token is \"rot\"");
}

// 8b. [089-005 REWRITE -- was scenarioRotationGoalAnticipatesStopWithRateCap,
// 086-003's closed-form rate-anticipation cap test] ROTATION goal_kind now
// stages a position-control Motion::JerkTrajectory solve-to-rest on the
// rotational channel at apply() time instead of a ramp_ target --
// applyStopAnticipation()'s STOP_ROTATION cap this scenario used to test is
// DELETED. Mirrors scenarioTurnGoalRuckigTraceNeverReverses (7b above): the
// full commanded omega trace, sampled across the REAL apply()+tick()
// staging path with a closely-tracking (undisturbed) encoder-arc feed,
// never reverses and completes via the goal's own STOP_ROTATION.
void scenarioRotationGoalRuckigTraceNeverReverses() {
  beginScenario(
      "089-005 AC (full trace): ROTATION's full commanded omega trace never reverses "
      "(real apply()+tick() path)");
  Subsystems::Planner planner;
  planner.configure(generousConfig());  // yaw_acc_max = 100 rad/s^2

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::ROTATION;
  cmd.goal.rotation.speed = 1.5f;   // already-signed: CCW -- becomes rotational_'s max_velocity
  cmd.goal.rotation.angle = 0.8f;   // [rad] rotational_'s Ruckig target
  cmd.stops_count = 1;
  cmd.stops_[0].kind = msg::StopKind::STOP_ROTATION;
  cmd.stops_[0].a = 50.0f;  // [mm] target per-wheel arc (arcScale_ = 50/0.8 = 62.5mm/rad)
  cmd.style = msg::StopStyle::ABRUPT;
  std::strncpy(cmd.corr_id, "rottrace1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  // Hand-derived trapezoid, same shape as 7b's own -- CCW (positive), target
  // 0.8 rad, ceiling 1.5 rad/s, yaw_acc_max=100 rad/s^2.
  const float kAMax = 100.0f;      // [rad/s^2]
  const float kOmegaMax = 1.5f;    // [rad/s]
  const float kAngle = 0.8f;       // [rad]
  const float kTAccel = kOmegaMax / kAMax;                  // 0.015s
  const float kDAccel = 0.5f * kAMax * kTAccel * kTAccel;   // 0.01125 rad
  const float kDCruise = kAngle - 2.0f * kDAccel;           // 0.7775 rad
  const float kTCruise = kDCruise / kOmegaMax;              // 0.518333s
  const float kTDecelStart = kTAccel + kTCruise;            // 0.533333s
  const float kTTotal = kTDecelStart + kTAccel;             // 0.548333s
  const float kArcScale = 62.5f;  // [mm/rad] == stops_[0].a / goal.rotation.angle

  auto trapezoidAngle = [&](float time) -> float {
    if (time <= 0.0f) return 0.0f;
    if (time < kTAccel) return 0.5f * kAMax * time * time;
    if (time < kTDecelStart) return kDAccel + kOmegaMax * (time - kTAccel);
    if (time < kTTotal) {
      float s = time - kTDecelStart;
      return kDAccel + kDCruise + kOmegaMax * s - 0.5f * kAMax * s * s;
    }
    return kAngle;
  };

  planner.tick(0, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});  // baseline
  planner.takeCommand();

  bool everReversed = false;
  bool completedWithRot = false;
  for (int ms = 20; ms <= 1500; ms += 20) {
    // Feed the per-wheel encoder ARC (mm) matching the SAME trapezoid,
    // scaled by the goal's own arc-per-rad ratio -- the "closely tracking"
    // undisturbed case (no divergence-replan expected here).
    float arc = trapezoidAngle(static_cast<float>(ms) * 0.001f) * kArcScale;
    planner.tick(static_cast<uint32_t>(ms), obsPosition(-arc), obsPosition(arc), msg::PoseEstimate{});
    msg::DrivetrainCommand held = planner.takeCommand();
    if (held.control.twist.omega < -0.5f) everReversed = true;
    if (!planner.hasActiveCommand()) {
      completedWithRot = (std::strcmp(planner.takeEvent().reason, "rot") == 0);
      break;
    }
  }
  checkFalse(everReversed, "commanded omega never goes negative (reverse of the commanded CCW turn)");
  checkTrue(completedWithRot, "goal completes via STOP_ROTATION (reason=rot), not any safety net");
}

// 8c. [089-005 Revision 2] ROTATION goal_kind: a lagging (slipping) plant
// triggers NORMAL divergence-triggered retarget()s on the rotational
// channel and still completes via the goal's OWN STOP_ROTATION, not a
// safety net -- mirrors 7d's TURN version above, but exercises the harder
// unit-conversion path (rotationalArcScale_, Decision 9): RT's measured
// remaining is an mm-valued per-wheel arc (rotationProgress()), converted
// to rotational_'s own radian domain before comparison. Same output-dead-
// time plant model as 7d/089-003's own scenarios.
void scenarioRotationGoalDivergenceReplanCorrectsLaggingPlant() {
  beginScenario(
      "089-005 AC: a lagging plant triggers NORMAL retarget()s on the rotational channel "
      "(RT's arc-mm<->rad conversion) and still completes via rot, not a safety net");
  Subsystems::Planner planner;
  planner.configure(generousConfig());

  msg::PlannerCommand cmd;
  cmd.goal_kind = msg::PlannerCommand::GoalKind::ROTATION;
  cmd.goal.rotation.speed = 1.5f;  // [rad/s]
  cmd.goal.rotation.angle = 0.8f;  // [rad]
  cmd.stops_count = 2;
  cmd.stops_[0].kind = msg::StopKind::STOP_ROTATION;
  cmd.stops_[0].a = 50.0f;  // [mm] arcScale_ = 50/0.8 = 62.5mm/rad
  cmd.stops_[1].kind = msg::StopKind::STOP_TIME;
  cmd.stops_[1].a = 10000.0f;  // [ms] test-owned safety bound (RT has no implicit time net)
  cmd.style = msg::StopStyle::ABRUPT;
  std::strncpy(cmd.corr_id, "rotlag1", sizeof(cmd.corr_id) - 1);
  planner.apply(cmd, 0);

  planner.tick(0, obsPosition(0.0f), obsPosition(0.0f), msg::PoseEstimate{});
  planner.takeCommand();

  // Closed-loop plant simulation: the plant only achieves kSlip (85%) of
  // whatever omega Planner commanded kOutputHops ticks ago -- the encoder
  // differential (right - left) integrates the resulting arc, matching
  // rotationProgress()'s own geometry (STOP_ROTATION's arc = |diff|/2).
  const float kSlip = 0.85f;
  const float kDt = 0.02f;  // [s]
  const int kOutputHops = 2;
  float delayBuf[kOutputHops] = {0.0f, 0.0f};
  int delayHead = 0;
  float encDiff = 0.0f;  // right - left, [mm]
  bool everReversed = false;
  bool completedWithRot = false;
  uint32_t t = 0;
  for (int i = 0; i < 750; ++i) {  // up to 15s -- comfortably past the test's own 10s time net
    t += static_cast<uint32_t>(kDt * 1000.0f);
    float halfDiff = encDiff * 0.5f;
    planner.tick(t, obsPosition(-halfDiff), obsPosition(halfDiff), msg::PoseEstimate{});
    msg::DrivetrainCommand held = planner.takeCommand();
    float omegaCmd = held.control.twist.omega;
    if (omegaCmd < -0.5f) everReversed = true;
    float omegaApplied = delayBuf[delayHead];
    delayBuf[delayHead] = omegaCmd;
    delayHead = (delayHead + 1) % kOutputHops;
    // omega [rad/s] -> arc rate [mm/s] via the SAME 62.5mm/rad scale;
    // encDiff accumulates 2x the per-wheel arc (right - left).
    encDiff += 2.0f * 62.5f * kSlip * omegaApplied * kDt;
    if (!planner.hasActiveCommand()) {
      completedWithRot = (std::strcmp(planner.takeEvent().reason, "rot") == 0);
      break;
    }
  }
  checkFalse(everReversed, "the divergence-corrected trace never reverses despite the lagging plant");
  checkTrue(completedWithRot,
            "a lagging plant completes via STOP_ROTATION (reason=rot) thanks to the divergence "
            "replan, not the test's own STOP_TIME safety bound");
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

// 11b. [089-004] STREAM goal_kind: a fresh STREAM command arriving WHILE a
// prior one is still active is a RE-TARGET of the SAME linear_/rotational_
// channels (stageVelocityGoal()'s own no-reset() design, planner.h) --
// seamless preemption, seeded from the channel's own current sampled state
// (Decision 8), never an instant discontinuous jump. Drives to a cruise,
// re-targets DOWN to a lower speed mid-stream, and proves the commanded
// v_x (a) never goes negative across the whole transition and (b) settles
// at the NEW target -- the ticket's own "STREAM semantics" requirement.
void scenarioStreamGoalMidProfilePreemptionIsSeamlessNoReverse() {
  beginScenario(
      "089-004: a fresh STREAM command mid-goal re-targets the SAME channels seamlessly, "
      "no reverse");
  Subsystems::Planner planner;
  planner.configure(generousConfig());  // a_max = 1000 mm/s^2

  msg::PlannerCommand first;
  first.goal_kind = msg::PlannerCommand::GoalKind::STREAM;
  first.goal.stream.v_x = 150.0f;  // [mm/s] -- ramp-up duration: 150/1000 = 0.15s
  first.goal.stream.omega = 0.0f;
  planner.apply(first, 0);

  planner.tick(0, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});  // baseline
  planner.takeCommand();

  bool everReversed = false;
  uint32_t t = 0;
  msg::DrivetrainCommand atCruise;
  for (int i = 0; i < 10; ++i) {  // up to 200ms -- past the 0.15s ramp-up, cruise reached
    t += 20;
    planner.tick(t, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
    atCruise = planner.takeCommand();
    if (atCruise.control.twist.v_x < -0.5f) everReversed = true;
  }
  checkFloatNear(atCruise.control.twist.v_x, 150.0f, 1.0f, "reached the first STREAM's cruise");

  // A second STREAM command, WHILE the first is still active (no stop ever
  // fired) -- re-targets DOWN to a lower speed, seeded from the current
  // ~150mm/s state (Decision 8), not from rest.
  msg::PlannerCommand second;
  second.goal_kind = msg::PlannerCommand::GoalKind::STREAM;
  second.goal.stream.v_x = 50.0f;  // [mm/s] -- lower than the current ~150mm/s cruise
  second.goal.stream.omega = 0.0f;
  planner.apply(second, t);
  checkTrue(planner.hasActiveCommand(), "the re-target keeps the goal active (STREAM never stops)");

  float lastV = -1.0f;
  for (int i = 0; i < 20; ++i) {  // up to 400ms more -- ample for the decel to the new target
    t += 20;
    planner.tick(t, msg::MotorState{}, msg::MotorState{}, msg::PoseEstimate{});
    msg::DrivetrainCommand held = planner.takeCommand();
    if (held.control.twist.v_x < -0.5f) everReversed = true;
    lastV = held.control.twist.v_x;
  }
  checkFalse(everReversed, "the re-target transition from 150 to 50 mm/s never reverses sign");
  checkFloatNear(lastV, 50.0f, 1.0f, "settles at the SECOND STREAM command's target speed");
  checkTrue(planner.hasActiveCommand(), "STREAM still never self-terminates after the re-target");
}

}  // namespace

int main() {
  scenarioFreshPlannerIsIdle();
  scenarioHasCommandTakeCommandClearsEvenWhileIdle();
  scenarioConfigureReachesOwnedRamp();
  scenarioVelocityGoalRampsAndStaysOpenEnded();
  scenarioVelocityGoalWithStopReportsTimed();
  scenarioVelocityGoalCruiseSustainsPastRampDurationWithNoBookkeeping();
  scenarioVelocityGoalWithStopRuckigTraceNeverReversesThroughCruiseAndDecel();
  scenarioDistanceGoalFiresImplicitStopAbrupt();
  scenarioDistanceGoalStopFiredBeforeConvergenceForcesFreshDecel();
  scenarioDistanceGoalRuckigTraceNeverReverses();
  scenarioDistanceGoalDivergenceReplanCorrectsLaggingPlant();
  scenarioDistanceGoalGrossDivergenceReanchorsAfterStall();
  scenarioDistanceGoalGuardSkipsNearTargetBackwardReplan();
  scenarioDistanceGoalGuardSkipsReplanOnceStopHasFired();
  scenarioTimedGoalSmoothRampDown();
  scenarioTimedGoalBothChannelsRuckigTraceNeverReverseAndCompleteViaTime();
  scenarioTimedGoalStopFiresDuringRampUpForcesFreshDecelNoReverse();
  scenarioTurnGoalUsesSignedSpeedAndCallerStop();
  scenarioTurnGoalRuckigTraceNeverReverses();
  scenarioTurnGoalStopFiredBeforeConvergenceForcesFreshDecel();
  scenarioTurnGoalDivergenceReplanCorrectsLaggingPlant();
  scenarioTurnGoalGrossDivergenceReanchorsAfterStall();
  scenarioTurnGoalGuardSkipsNearTargetBackwardReplan();
  scenarioTurnGoalGuardSkipsReplanOnceStopHasFired();
  scenarioRotationGoalUsesSignedSpeedAndCallerStop();
  scenarioRotationGoalRuckigTraceNeverReverses();
  scenarioRotationGoalDivergenceReplanCorrectsLaggingPlant();
  scenarioGotoGoalPursuesDirectlyWhenBearingWithinGate();
  scenarioGotoGoalPreRotatesThenPursuesAndArrives();
  scenarioStopGoalKindHaltsSilently();
  scenarioStreamGoalIsOpenEnded();
  scenarioStreamGoalMidProfilePreemptionIsSeamlessNoReverse();

  if (g_failureCount == 0) {
    std::printf("OK: all Planner scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Planner scenarios\n", g_failureCount);
  return 1;
}
