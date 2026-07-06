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
  scenarioTimedGoalSmoothRampDown();
  scenarioTurnGoalUsesSignedSpeedAndCallerStop();
  scenarioRotationGoalUsesSignedSpeedAndCallerStop();
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
