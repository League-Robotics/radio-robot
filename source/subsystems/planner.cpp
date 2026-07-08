// planner.cpp -- Subsystems::Planner implementation. See planner.h for the
// class-level design notes.
#include "subsystems/planner.h"

#include <math.h>

#include "motion/stop_condition.h"

namespace Subsystems {

namespace {

// reasonTokenFor -- maps a fired msg::StopKind to its wire reason= token.
// Matches source_old/commands/MotionCommand.cpp's mc_reasonToken() for the
// five kinds this tree implements (motion/stop_condition.h's scope).
// Returns a pointer to a string-literal (static storage) -- safe to hold as
// a bare `const char*` in Planner::pendingReason_.
const char* reasonTokenFor(msg::StopKind kind) {
  switch (kind) {
    case msg::StopKind::STOP_TIME:
      return "time";
    case msg::StopKind::STOP_DISTANCE:
      return "dist";
    case msg::StopKind::STOP_HEADING:
      return "heading";
    case msg::StopKind::STOP_POSITION:
      return "pos";
    case msg::StopKind::STOP_ROTATION:
      return "rot";
    default:
      return "";
  }
}

// verbFallbackFor -- 090-004: the wire verb implied by a goal's own
// DriveMode, for the goal kinds whose msg::PlannerCommand.verb arrives
// empty (S/T/D/G -- mirrors Rt::MotionCommand::verb's own "empty for
// S/T/D/G, disambiguating letter otherwise" convention,
// source/commands/motion_commands.cpp). Ported from main_loop.cpp's former
// motionVerbForMode(): that function needed the SAME two inputs (a
// DriveMode plus a disambiguating override) to name a completed goal's "done
// <verb>" wire text, but read them from loop-local state sampled around
// tick(); this class already has both at STAGE time -- stageCommon()'s own
// `mode` parameter is exactly the resolved DriveMode motionVerbForMode()
// used to dispatch on, so the fallback resolves right there (see
// stageCommon()) instead of needing a second, later read of loop state that
// no longer exists. Purely a short data token -- the same category as
// reasonTokenFor()'s reason strings above -- never wire text;
// CommandProcessor::emitEvent() still owns 100% of the "EVT ..." grammar
// (planner.h's class comment; .claude/rules/naming-and-style.md sec 4).
const char* verbFallbackFor(msg::DriveMode mode) {
  switch (mode) {
    case msg::DriveMode::STREAMING:
      return "S";
    case msg::DriveMode::TIMED:
      return "T";
    case msg::DriveMode::DISTANCE:
      return "D";
    case msg::DriveMode::GO_TO:
      return "G";
    default:
      return "";
  }
}

// kDegToRad -- degrees -> radians, for PlannerConfig.turn_in_place_gate
// (stored in DEGREES, not radians -- main.cpp's defaultPlannerConfig()'s own
// comment: "matches docs/protocol-v2.md sec 10's G default", 35 deg).
constexpr float kDegToRad = 3.14159265f / 180.0f;

// kPreRotateOmega -- fixed spin-in-place rate for GOTO_GOAL's PRE_ROTATE
// phase. Matches source/commands/motion_commands.cpp's kTurnOmega exactly
// (~70 deg/s): Planner has no DrivetrainConfig.trackwidth to derive a
// wheel-speed-based rate the way source_old's _startPreRotate() did (see
// planner.h's class comment on why this class cannot gain that dependency);
// a fixed, well-under-yaw_rate_max rate is the same deterministic,
// sim-testable precedent ticket 084-003 already established for TURN/RT.
constexpr float kPreRotateOmega = 1.2217f;  // [rad/s]

// velocityShapedMode -- 084-005's Decision 6: the TLM `mode=` wire mapping
// for every "velocity-shaped" goal kind (VELOCITY/TURN/ROTATION -- the three
// that stage a raw (v, omega) ramp target with no goal-kind-intrinsic stop
// of their own, unlike DISTANCE/TIMED/GOTO_GOAL). architecture-update.md
// (084) Decision 6 (Alternative b, the chosen one): a command that carries
// at least one stop condition self-terminates and reports
// `DriveMode::TIMED` ('T' on the wire) -- the SAME bucket plain `T` already
// uses; an unbounded one reports `DriveMode::STREAMING` ('S') -- the SAME
// bucket plain `S` already uses. `TURN`/`RT` (source/commands/
// motion_commands.cpp's handleTURN/handleRT) unconditionally append their
// own built-in HEADING/ROTATION stop before staging, so cmd.stops_count_val()
// is never 0 for them -- they always land in the TIMED/'T' bucket, matching
// the approved mapping table (this ticket's own doc comment) exactly. `R`
// (handleR) adds a stop only when the wire carries a `stop=` clause -- a
// bare `R` is open-ended (STREAMING/'S'), a bounded one self-terminates
// (TIMED/'T'). This is a data-driven rule, not a per-verb special case: it
// reads only the STAGED msg::PlannerCommand's own stop count, matching
// source_old's own internal STREAM/TIMED/ARC -> single Goal::VELOCITY
// collapse (this ticket does the inverse: one Planner-internal GoalKind::
// VELOCITY collapses back out to two DriveMode values by this one
// property). No `msg::DriveMode` schema change -- STREAMING/TIMED already
// exist; this only changes which of the two apply()'s VELOCITY/TURN/
// ROTATION cases pass to stageGoal() (previously always the bespoke
// DriveMode::VELOCITY, now never emitted at all -- see dev_loop.cpp's
// motionVerbForMode(), whose own VELOCITY case is now dead/defensive-only).
msg::DriveMode velocityShapedMode(const msg::PlannerCommand& cmd) {
  return cmd.stops_count_val() > 0 ? msg::DriveMode::TIMED : msg::DriveMode::STREAMING;
}

// kOutputHops/kAssumedPassPeriod/kDeadTime -- ticket 087-009's fixed
// two-output-pass dead-time compensation, originally derived for the
// now-deleted applyStopAnticipation() (STOP_DISTANCE/STOP_HEADING/
// STOP_ROTATION's pre-Ruckig anticipation cap -- removed in full by ticket
// 089-005, its last remaining callers (TURN/ROTATION) having migrated onto
// Motion::JerkTrajectory, whose whole-trajectory solve already produces the
// equivalent anticipation as an intrinsic property of the plan). Hoisted to
// file scope (089-003) and now shared by maybeReplanDistance() (DISTANCE's
// divergence-triggered replan) and maybeReplanRotational() (TURN/ROTATION's
// own, 089-005) -- architecture-update.md (089) Decision 10's own
// instruction to reuse one tau definition rather than redefine one under a
// new name.
constexpr float kOutputHops = 2.0f;
constexpr float kAssumedPassPeriod = 0.020f;  // [s] matches main.cpp's kPeriod
constexpr float kDeadTime = kOutputHops * kAssumedPassPeriod;  // [s]

}  // namespace

void Planner::copyCallerStops(const msg::PlannerCommand& cmd) {
  stopsCount_ = cmd.stops_count_val();
  if (stopsCount_ > 4) stopsCount_ = 4;
  for (uint8_t i = 0; i < stopsCount_; ++i) {
    stops_[i] = cmd.stops()[i];
  }
}

void Planner::appendStop(msg::StopKind kind, float a, float b, float ax) {
  if (stopsCount_ >= 4) return;  // cap already full -- see class comment
  msg::StopCondition c;
  c.kind = kind;
  c.a = a;
  c.b = b;
  c.ax = ax;
  stops_[stopsCount_++] = c;
}

void Planner::stageCommon(msg::DriveMode mode, const msg::PlannerCommand& cmd) {
  style_ = cmd.style;
  for (int i = 0; i < 64; ++i) corrId_[i] = cmd.corr_id[i];
  // 090-004: bounded copy (cmd.verb is a generated char[64]; verb_ mirrors
  // Rt::MotionCommand::verb's own char[8] -- see planner.h's field comment)
  // -- every wire verb this field ever carries ("R"/"TURN"/"RT") is well
  // under 7 characters, so this never truncates in practice. Empty for
  // S/T/D/G (motion_commands.cpp never sets cmd.verb for those) -- the
  // fallback below fills in THIS goal's own DriveMode-implied letter for
  // exactly those cases (verbFallbackFor()'s own doc comment), so
  // queueEvent() can always just read verb_ verbatim with no further
  // resolution needed at completion time.
  int vi = 0;
  for (; cmd.verb[vi] != '\0' && vi < 7; ++vi) verb_[vi] = cmd.verb[vi];
  verb_[vi] = '\0';
  if (verb_[0] == '\0') {
    const char* fallback = verbFallbackFor(mode);
    int fi = 0;
    for (; fallback[fi] != '\0' && fi < 7; ++fi) verb_[fi] = fallback[fi];
    verb_[fi] = '\0';
  }
  stopping_ = false;
  baselineCaptured_ = false;
  activeCmd_ = true;
  mode_ = mode;
}

void Planner::stageGoal(float v, float omega, msg::DriveMode mode,
                        const msg::PlannerCommand& cmd) {
  ramp_.setTarget(v, omega);
  stagedV_ = v;
  stagedOmega_ = omega;
  stageCommon(mode, cmd);
}

void Planner::stageRotationalGoal(float targetAngle, float omega, float arcScale,
                                  msg::DriveMode mode, const msg::PlannerCommand& cmd,
                                  uint32_t now) {
  // Linear channel: TURN/ROTATION are turn-in-place (v = 0 always) -- a
  // trivial position-control solve-to-rest at 0 (ticket item 3; mirrors
  // DISTANCE's own solveToRest() pattern rather than inventing a separate
  // v=0 special case). Already at rest (reset()), so this is a ~zero-
  // duration solve -- sampled every tick like any other channel, never
  // silently skipped (see class comment).
  linear_.reset();
  linearCeiling_ = 0.0f;
  linear_.solveToRest(0.0f, 0.0f);
  linearSolveMs_ = now;

  // Rotational channel: position-control solve-to-rest at targetAngle
  // (Decision 9). max_velocity is the caller's own already-resolved rate
  // magnitude (kTurnOmega/kRotationOmega, motion_commands.cpp) -- NOT
  // config_.yaw_rate_max directly; solveToRest() clamps against that
  // global ceiling underneath regardless (Decision 9's revision).
  rotational_.reset();
  rotationalCeiling_ = fabsf(omega);
  rotational_.solveToRest(targetAngle, rotationalCeiling_);
  rotationalTarget_ = targetAngle;
  rotationalArcScale_ = arcScale;
  rotationalSolveMs_ = now;
  lastReplanMs_ = now;

  stagedV_ = 0.0f;
  stagedOmega_ = omega;
  jerkRotationGoal_ = true;
  stageCommon(mode, cmd);
}

void Planner::stageVelocityGoal(float v, float omega, msg::DriveMode mode,
                                const msg::PlannerCommand& cmd, uint32_t now) {
  // 089-004 (architecture-update.md Decision 2, "Pattern B"): both channels
  // solve velocity-control-to-cruise right here, at apply() time -- no
  // reset() first (see this method's own doc comment, planner.h): every
  // solveToVelocity() call seeds from the channel's own remembered last
  // sample (Decision 8), which is exactly right whether that memory is
  // "at rest" (a fresh goal after any prior goal has fully idled -- tick()'s
  // own !activeCmd_ branch keeps both channels pinned at rest while idle)
  // or "mid-cruise" (a STREAM command preempting a still-active one -- the
  // continuity this goal kind's own semantics require). max_velocity = this
  // channel's own commanded magnitude; each solve call clamps against
  // configure()'s global ceiling underneath regardless (Decision 2's
  // revision) -- a channel commanded 0 for the whole goal (e.g. a straight
  // T's omega) still gets the identical call, no special-casing (Decision
  // 1).
  linearCeiling_ = fabsf(v);
  linear_.solveToVelocity(v, linearCeiling_);
  linearSolveMs_ = now;

  rotationalCeiling_ = fabsf(omega);
  rotational_.solveToVelocity(omega, rotationalCeiling_);
  rotationalSolveMs_ = now;

  stagedV_ = v;
  stagedOmega_ = omega;
  jerkVelocityGoal_ = true;
  stageCommon(mode, cmd);
}

void Planner::apply(const msg::PlannerCommand& cmd, uint32_t now) {
  // apply() captures no MotionBaseline -- see the class comment. 089-003:
  // `now` is now used to anchor the linear channel's own elapsed-time clock
  // for DISTANCE (linearSolveMs_, read back by linearElapsed()) -- the
  // "future ticket may want a staged-but-not-yet-ticked timestamp" this
  // comment used to flag.

  targetX_ = 0.0f;
  targetY_ = 0.0f;
  targetSpeed_ = 0.0f;
  distanceTarget_ = 0.0f;
  // Reset unconditionally so a new non-GOTO_GOAL command issued while a G
  // goal was still in-flight leaves no stale phase behind (gPhase_ is only
  // ever consulted while mode_ == GO_TO, so this matters for hygiene/
  // debuggability more than correctness -- see class comment).
  gPhase_ = GPhase::IDLE;
  // 089-004/005: reset unconditionally, same reasoning as gPhase_ above --
  // stageVelocityGoal() (TIMED/VELOCITY/STREAM, below) is the ONLY place
  // that sets jerkVelocityGoal_ back to true; stageRotationalGoal() (TURN/
  // ROTATION, below) is the ONLY place that sets jerkRotationGoal_ back to
  // true. Every other goal_kind case leaves both false, which is exactly
  // the discrimination tick() needs (class comment) -- mode_ == TIMED alone
  // is shared by plain T, TURN, and ROTATION alike.
  jerkVelocityGoal_ = false;
  jerkRotationGoal_ = false;

  switch (cmd.goal_kind) {
    case msg::PlannerCommand::GoalKind::VELOCITY: {
      // v_y is intentionally ignored -- mirrors Drivetrain::setTwist()'s own
      // v_y-ignored precedent (differential-only this sprint). 089-004:
      // stages onto linear_/rotational_ via stageVelocityGoal() instead of
      // ramp_.setTarget() -- see planner.h's class comment.
      copyCallerStops(cmd);
      targetSpeed_ = cmd.goal.velocity.v_x;
      stageVelocityGoal(cmd.goal.velocity.v_x, cmd.goal.velocity.omega, velocityShapedMode(cmd), cmd,
                        now);
      break;
    }

    case msg::PlannerCommand::GoalKind::GOTO_GOAL: {
      // GOTO_GOAL manages its own stops_[] end to end -- copyCallerStops()
      // is never called here (see class comment): docs/protocol-v2.md
      // sec 10's G contract accepts no stop= clauses, and the PRE_ROTATE ->
      // PURSUE handoff (enterPursue()) needs a clean stop-set swap with no
      // caller-supplied slot to preserve.
      stopsCount_ = 0;

      float x = cmd.goal.goto_goal.x;
      float y = cmd.goal.goto_goal.y;
      float speed = cmd.goal.goto_goal.speed;
      targetX_ = x;
      targetY_ = y;
      targetSpeed_ = speed;

      // Bearing to the relative target, robot frame, AT COMMAND TIME -- no
      // pose needed: (x, y) is already expressed in the robot's own frame
      // (ported concept, source_old/control/PlannerBegin.cpp's beginGoTo()).
      float bearing = atan2f(y, x);
      float gate = config_.turn_in_place_gate * kDegToRad;

      if (fabsf(bearing) > gate) {
        gPhase_ = GPhase::PRE_ROTATE;
        float omega = (bearing >= 0.0f) ? kPreRotateOmega : -kPreRotateOmega;
        appendStop(msg::StopKind::STOP_HEADING, bearing, gate);
        float nominal = (fabsf(omega) > 1e-3f) ? (fabsf(bearing) / fabsf(omega)) * 1000.0f
                                                : 0.0f;  // [ms]
        appendStop(msg::StopKind::STOP_TIME, nominal * 2.0f + 2000.0f);
        stageGoal(0.0f, omega, msg::DriveMode::GO_TO, cmd);
      } else {
        // World-frame anchor + PURSUE's own stops_[] are resolved on the
        // FIRST tick() (captureBaseline()/enterPursue()) -- apply() has no
        // pose to convert (x, y) into world coordinates with (class comment).
        gPhase_ = GPhase::PURSUE;
        stageGoal(speed, 0.0f, msg::DriveMode::GO_TO, cmd);
      }
      break;
    }

    case msg::PlannerCommand::GoalKind::TURN: {
      // TurnGoal.speed is an already-signed angular rate -- see class
      // comment. Turn-in-place: v = 0. velocityShapedMode() always resolves
      // to TIMED here -- handleTURN (motion_commands.cpp) unconditionally
      // appends its own built-in HEADING stop before staging, so
      // cmd.stops_count_val() is never 0 (see velocityShapedMode()'s own
      // doc comment above).
      //
      // 089-005 (Decision 9): the rotational channel's Ruckig target is
      // cmd.stops_[0].a -- the ALREADY-RESOLVED, signed, shortest-path
      // heading delta handleTURN computes from LIVE fused heading at
      // command time (unchanged division of labor: apply() still has no
      // pose argument and still does not resolve this itself, it just
      // reads the resolved value, now from the stop condition's own `a`
      // field instead of only using it as a stop threshold). copyCallerStops()
      // must run FIRST so stops_[0] is populated before this reads it.
      copyCallerStops(cmd);
      float target = stops_[0].a;  // [rad] signed heading delta
      stageRotationalGoal(target, cmd.goal.turn.speed, /*arcScale=*/1.0f,
                          velocityShapedMode(cmd), cmd, now);
      break;
    }

    case msg::PlannerCommand::GoalKind::DISTANCE: {
      copyCallerStops(cmd);
      float speed = cmd.goal.distance.speed;
      float distance = cmd.goal.distance.distance;
      float v = (distance < 0.0f) ? -speed : speed;
      float mag = (distance < 0.0f) ? -distance : distance;

      // Implicit stops (ported concept from source_old's beginDistance(),
      // which added these itself, not its wire caller): the DISTANCE stop
      // itself, plus a generous TIME safety net (2x nominal travel time +
      // 2s) computed purely from this goal's own fields. UNCHANGED by
      // 089-003 (Decision 4): Motion::evaluateStopCondition() stays the
      // authoritative completion signal regardless of which mechanism
      // shapes the commanded velocity in between.
      appendStop(msg::StopKind::STOP_DISTANCE, mag);
      float spdMax = fabsf(speed);
      if (spdMax < 1.0f) spdMax = 1.0f;
      float nominal = (mag / spdMax) * 1000.0f;  // [ms]
      appendStop(msg::StopKind::STOP_TIME, nominal * 2.0f + 2000.0f);

      distanceTarget_ = distance;

      // 089-003 (architecture-update.md Decision 2, "Pattern A"): DISTANCE's
      // target is fully known here, at apply() time -- no live pose/
      // observation dependency (unlike GOTO_GOAL) -- so, unique among the
      // still-ramp_-driven goal kinds, the WHOLE position-control solve
      // happens right here instead of being deferred to a per-tick ramp
      // target. linear_.reset() unconditionally seeds from rest: this
      // ticket's own scope is "the target known entirely at apply() time,"
      // and a fresh D issued while a PRIOR D is still in flight restarting
      // from a clean (0, 0, 0) seed is a documented, deliberate
      // simplification (the same "fresh ramp-up" precedent enterPursue()
      // already established for GOTO_GOAL) -- not an attempt to preserve
      // velocity continuity across separate D commands, which apply()'s own
      // no-observations constraint (class comment) makes unobservable
      // anyway (Decision 8: seed from the channel's OWN state, never a
      // measurement, and apply() has no live measurement to seed from in
      // the first place). max_velocity = min(commandedSpeed, v_body_max) --
      // solveToRest() itself clamps against the configured global ceiling
      // (jerk_trajectory.cpp), so only the commanded-speed magnitude is
      // passed here (Decision 2's revision).
      linear_.reset();
      linearCeiling_ = fabsf(speed);
      linear_.solveToRest(distance, linearCeiling_);
      linearTarget_ = distance;
      linearSolveMs_ = now;
      lastReplanMs_ = now;

      stagedV_ = v;        // still latched for captureBaseline()'s vSign
      stagedOmega_ = 0.0f;  // DISTANCE is turn-in-place-free (omega always 0)
      stageCommon(msg::DriveMode::DISTANCE, cmd);
      break;
    }

    case msg::PlannerCommand::GoalKind::TIMED: {
      copyCallerStops(cmd);
      if (cmd.goal.timed.duration > 0) {
        // Implicit STOP_TIME from the goal's own duration field (ported
        // concept from source_old's beginTimed()).
        appendStop(msg::StopKind::STOP_TIME, static_cast<float>(cmd.goal.timed.duration));
      }
      // 089-004: stages onto linear_/rotational_ via stageVelocityGoal()
      // instead of ramp_.setTarget() -- see planner.h's class comment.
      stageVelocityGoal(cmd.goal.timed.v_x, cmd.goal.timed.omega, msg::DriveMode::TIMED, cmd, now);
      break;
    }

    case msg::PlannerCommand::GoalKind::ROTATION: {
      // RotationGoal.speed is an already-signed angular rate -- see class
      // comment (the arc-target ROTATION stop needs Drivetrain's
      // trackwidth, which this class does not have -- the caller resolves
      // it). Turn-in-place: v = 0. velocityShapedMode() always resolves to
      // TIMED here -- handleRT (motion_commands.cpp) unconditionally
      // appends its own built-in ROTATION stop before staging (same
      // reasoning as the TURN case above).
      //
      // 089-005 (Decision 9): the rotational channel's Ruckig target is
      // cmd.goal.rotation.angle -- an EXISTING msg::RotationGoal field
      // handleRT already populates (relAngle * kCdegToRad, signed) but this
      // class previously only treated as informational. RT's built-in
      // ROTATION stop threshold (stops_[0].a, a per-wheel ARC in mm) is a
      // DIFFERENT number from this radian target (a trackwidth-dependent
      // conversion) -- rotationalArcScale_ (mm per rad) is derived once,
      // here, from these same two already-resolved fields, so
      // maybeReplanRotational() can compare its divergence in one
      // consistent domain without Planner gaining a new
      // DrivetrainConfig.trackwidth dependency (class comment).
      copyCallerStops(cmd);
      float target = cmd.goal.rotation.angle;  // [rad] signed relative rotation
      float arcScale = 1.0f;                   // [mm/rad] degenerate-target fallback
      if (fabsf(target) > 1e-6f) {
        arcScale = stops_[0].a / fabsf(target);
      }
      stageRotationalGoal(target, cmd.goal.rotation.speed, arcScale, velocityShapedMode(cmd), cmd,
                          now);
      break;
    }

    case msg::PlannerCommand::GoalKind::STREAM: {
      // v_y is intentionally ignored -- see the VELOCITY case above. 089-004:
      // each fresh STREAM command is a RE-TARGET of the SAME linear_/
      // rotational_ channels (stageVelocityGoal()'s own no-reset() design) --
      // seamless preemption of a still-active prior STREAM command falls out
      // of Decision 8's seeding contract for free, with no special-casing
      // here (see planner.h's doc comment on stageVelocityGoal()).
      copyCallerStops(cmd);
      stageVelocityGoal(cmd.goal.stream.v_x, cmd.goal.stream.omega, msg::DriveMode::STREAMING, cmd,
                        now);
      break;
    }

    case msg::PlannerCommand::GoalKind::STOP: {
      ramp_.reset();
      activeCmd_ = false;
      stopping_ = false;
      mode_ = msg::DriveMode::IDLE;
      gPhase_ = GPhase::IDLE;
      // No EVT -- STOP is a silent, immediate halt (ticket 084-002
      // acceptance: "STOP halts immediately with no EVT").
      break;
    }

    case msg::PlannerCommand::GoalKind::NONE:
    default:
      break;  // no-op, matches source_old's own NONE/default case
  }
}

void Planner::captureBaseline(uint32_t now, const msg::MotorState& leftObs,
                              const msg::MotorState& rightObs,
                              const msg::PoseEstimate& fusedPose) {
  baseline_.t0 = now;
  baseline_.enc0 = 0.0f;
  baseline_.encDiff0 = 0.0f;
  if (leftObs.position.has && rightObs.position.has) {
    baseline_.enc0 = (leftObs.position.val + rightObs.position.val) * 0.5f;
    baseline_.encDiff0 = rightObs.position.val - leftObs.position.val;
  }
  baseline_.heading0 = fusedPose.pose.h;
  baseline_.pose0X = fusedPose.pose.x;
  baseline_.pose0Y = fusedPose.pose.y;
  baseline_.vSign = (stagedV_ > 0.0f) ? 1.0f : (stagedV_ < 0.0f ? -1.0f : 0.0f);
  baseline_.omegaSign = (stagedOmega_ > 0.0f) ? 1.0f : (stagedOmega_ < 0.0f ? -1.0f : 0.0f);

  if (mode_ == msg::DriveMode::GO_TO) {
    // World-frame anchor for the relative (targetX_, targetY_) offset --
    // resolved HERE, not apply() (which has no pose to convert with; see
    // class comment), at the equivalent "at command time" moment, since no
    // movement occurs between apply() and this first tick(). Computed
    // unconditionally for BOTH phases -- matches source_old's beginGoTo(),
    // which resolves the world target before branching into PRE_ROTATE vs
    // PURSUE.
    float h = baseline_.heading0;
    gTargetXWorld_ = baseline_.pose0X + targetX_ * cosf(h) - targetY_ * sinf(h);
    gTargetYWorld_ = baseline_.pose0Y + targetX_ * sinf(h) + targetY_ * cosf(h);
    if (gPhase_ == GPhase::PURSUE) {
      enterPursue(now);
    }
  }
}

void Planner::enterPursue(uint32_t now) {
  gPhase_ = GPhase::PURSUE;
  // Fresh TIME-net baseline for the pursue phase -- ported concept:
  // source_old's MotionCommand::start() re-baselines t0 when PURSUE is
  // (re)configured, distinct from PRE_ROTATE's own baseline. heading0/
  // pose0X/pose0Y are not read again after this point -- gTargetXWorld_/
  // gTargetYWorld_ already captured them.
  baseline_.t0 = now;
  stopsCount_ = 0;
  float distance = sqrtf(targetX_ * targetX_ + targetY_ * targetY_);  // [mm]
  float pursueSpeed = (targetSpeed_ > 1.0f) ? targetSpeed_ : 1.0f;
  float timeout = 2.0f * (distance / pursueSpeed) * 1000.0f + 4000.0f;  // [ms]
  appendStop(msg::StopKind::STOP_POSITION, gTargetYWorld_, config_.arrive_tol, gTargetXWorld_);
  appendStop(msg::StopKind::STOP_TIME, timeout);
  // Fresh ramp-up into the pursue speed -- ported: source_old's _bvc.reset()
  // before (re)configuring the PURSUE MotionCommand.
  ramp_.reset();
}

void Planner::pursueSteer(const msg::PoseEstimate& fusedPose) {
  float h = fusedPose.pose.h;
  float dxW = gTargetXWorld_ - fusedPose.pose.x;
  float dyW = gTargetYWorld_ - fusedPose.pose.y;
  float dx = dxW * cosf(h) + dyW * sinf(h);
  float dy = -dxW * sinf(h) + dyW * cosf(h);
  float d2 = dx * dx + dy * dy;
  float dRemaining = sqrtf(d2);

  // Terminal decel cap: clamp the commanded speed so the ramp has time to
  // decelerate to zero before the POSITION stop fires.
  float v = targetSpeed_;
  float vCap = sqrtf(2.0f * config_.a_decel * dRemaining);
  if (vCap < v) v = vCap;

  // Curvature clamp: bound kappa so passing abeam the target (small
  // dRemaining, dy != 0) cannot drive omega into a tight orbit. kappaMax =
  // 2 / max(dRemaining, 2*arrive_tol) limits the turning radius to at most
  // 0.5*arrive_tol at the tightest point.
  float kappaMax = 2.0f / fmaxf(dRemaining, 2.0f * config_.arrive_tol);
  float kappa = (d2 > 0.1f) ? fmaxf(-kappaMax, fminf(kappaMax, 2.0f * dy / d2)) : 0.0f;
  float omega = v * kappa;

  ramp_.setTarget(v, omega);
}

float Planner::linearElapsed(uint32_t now) const {
  return static_cast<float>(static_cast<int32_t>(now - linearSolveMs_)) * 0.001f;
}

float Planner::rotationalElapsed(uint32_t now) const {
  return static_cast<float>(static_cast<int32_t>(now - rotationalSolveMs_)) * 0.001f;
}

void Planner::maybeReplanDistance(uint32_t now, const msg::MotorState& leftObs,
                                  const msg::MotorState& rightObs,
                                  const msg::PoseEstimate& fusedPose) {
  // Guard 3 (deadband + rate limit, Decision 10): at most one replan per
  // kMinReplanInterval.
  if (static_cast<int32_t>(now - lastReplanMs_) < static_cast<int32_t>(kMinReplanInterval)) {
    return;
  }

  // DISTANCE always carries its own STOP_DISTANCE (appended in apply()) --
  // find it by kind (the same defensive search maybeReplanRotational() uses
  // below), rather than assuming stops_[0] (robust against a future
  // stops_[] ordering change).
  const msg::StopCondition* distCond = nullptr;
  for (uint8_t i = 0; i < stopsCount_; ++i) {
    if (stops_[i].kind == msg::StopKind::STOP_DISTANCE) {
      distCond = &stops_[i];
      break;
    }
  }
  if (distCond == nullptr) return;  // defensive -- DISTANCE always appends one

  // vSign: the goal's commanded direction (matches distanceProgress()'s own
  // convention, motion/stop_condition.cpp). A degenerate speed==0 DISTANCE
  // goal has no meaningful direction to replan in -- skip.
  float vSign = baseline_.vSign;
  if (vSign == 0.0f) return;

  float measuredRemainingMag = 0.0f;
  Motion::StopEvalResult r = Motion::remainingToStop(*distCond, baseline_, leftObs, rightObs,
                                                     fusedPose, &measuredRemainingMag);
  if (r != Motion::StopEvalResult::NOT_FIRED) return;  // FIRED: guard 1 -- the
  // stop-evaluation loop later this SAME tick owns completion, not a replan.
  // (UNSUPPORTED cannot happen for STOP_DISTANCE.)

  // Plan's own remaining, in linear_'s CURRENT frame (whatever the most
  // recent solveToRest()/retarget() established) -- Decision 10's "available
  // internally -- Decision 8's remembered last sample." fabsf() makes this
  // directly comparable to measuredRemainingMag (also a nonnegative
  // magnitude) regardless of which direction/frame linear_ is currently
  // solving in.
  Motion::JerkTrajectory::State state = linear_.sample(linearElapsed(now));
  float planRemainingMag = fabsf(linearTarget_ - state.position);

  float divergence = fabsf(planRemainingMag - measuredRemainingMag);
  if (divergence < kDivergenceThreshold) return;  // within tolerance -- no replan

  // Guard 2 (no-reverse-target): dead-time-project the MEASURED remaining
  // ("remaining = target - (measured + v*tau)", Decision 8's revision --
  // target/measured collapse to measuredRemainingMag - planSpeed*tau here,
  // since measuredRemainingMag IS target-minus-measured already). `v` is the
  // PLAN's own last-sampled speed (never a measured velocity -- Decision 8's
  // "PLAN leads velocity" framing), so this projection cannot reopen
  // 087-009's measured-velocity-feedback limit-cycle risk.
  float planSpeedMag = fabsf(state.velocity);
  float projectedRemainingMag = measuredRemainingMag - planSpeedMag * kDeadTime;
  if (projectedRemainingMag <= 0.0f) return;  // never solve backward

  if (divergence >= kGrossDivergenceThreshold) {
    // GROSS case (Decision 8's revision): full re-anchor from measurement,
    // including velocity. Position: convert the (un-projected)
    // measuredRemainingMag back into linear_'s current frame the same way
    // "remaining" is frame-independent (linearTarget_ - remaining, matching
    // this method's own planRemainingMag derivation above). Velocity: the
    // measured per-wheel velocity, averaged (straight-driving assumption,
    // same as distanceProgress()'s own position averaging) -- 0.0f if
    // either wheel's velocity observation is momentarily absent.
    float measuredPositionSigned = linearTarget_ - vSign * measuredRemainingMag;
    float measuredVelocitySigned = 0.0f;
    if (leftObs.velocity.has && rightObs.velocity.has) {
      measuredVelocitySigned = (leftObs.velocity.val + rightObs.velocity.val) * 0.5f;
    }
    linear_.reanchor(measuredPositionSigned, measuredVelocitySigned);
    // linearTarget_ is unchanged -- reanchor() reuses linear_'s own
    // internally remembered target (jerk_trajectory.h's own doc comment).
  } else {
    // NORMAL case: retarget() to the dead-time-projected remaining, seeded
    // (internally, by linear_ itself) from its own last velocity/
    // acceleration -- never from measurement.
    float newRemainingSigned = vSign * projectedRemainingMag;
    linear_.retarget(newRemainingSigned);
    linearTarget_ = newRemainingSigned;
  }
  linearSolveMs_ = now;
  lastReplanMs_ = now;
}

void Planner::maybeReplanRotational(uint32_t now, const msg::MotorState& leftObs,
                                    const msg::MotorState& rightObs,
                                    const msg::PoseEstimate& fusedPose) {
  // Guard 3 (deadband + rate limit, Decision 10) -- lastReplanMs_/
  // kMinReplanInterval are SHARED with maybeReplanDistance() (planner.h's
  // own comment on why: at most one position-control goal is ever active).
  if (static_cast<int32_t>(now - lastReplanMs_) < static_cast<int32_t>(kMinReplanInterval)) {
    return;
  }

  // TURN always carries its own built-in STOP_HEADING; RT its own built-in
  // STOP_ROTATION (both appended first, before any caller stop=, by
  // handleTURN/handleRT) -- find whichever is present the same defensive
  // way maybeReplanDistance() finds its own STOP_DISTANCE, rather than
  // assuming stops_[0].
  const msg::StopCondition* rotCond = nullptr;
  for (uint8_t i = 0; i < stopsCount_; ++i) {
    if (stops_[i].kind == msg::StopKind::STOP_HEADING ||
        stops_[i].kind == msg::StopKind::STOP_ROTATION) {
      rotCond = &stops_[i];
      break;
    }
  }
  if (rotCond == nullptr) return;  // defensive -- TURN/ROTATION always append one

  // omegaSign: the goal's commanded turn direction (matches
  // rotationProgress()'s own convention, motion/stop_condition.cpp). A
  // degenerate omega==0 goal has no meaningful direction to replan in --
  // skip (mirrors maybeReplanDistance()'s own vSign guard).
  float omegaSign = baseline_.omegaSign;
  if (omegaSign == 0.0f) return;

  float measuredRemainingNative = 0.0f;
  Motion::StopEvalResult r = Motion::remainingToStop(*rotCond, baseline_, leftObs, rightObs,
                                                     fusedPose, &measuredRemainingNative);
  if (r != Motion::StopEvalResult::NOT_FIRED) return;  // FIRED: guard 1 -- the
  // stop-evaluation loop later this SAME tick owns completion, not a replan.

  // rotationalArcScale_ converts RT's per-wheel-ARC-mm measured remaining
  // into rotational_'s own radian domain (1.0f, a no-op, for TURN -- see
  // stageRotationalGoal()'s and this method's own doc comments, planner.h).
  // Every comparison/projection/retarget below then happens in radians,
  // exactly mirroring maybeReplanDistance()'s mm-domain math.
  float measuredRemainingRad = measuredRemainingNative / rotationalArcScale_;

  // Plan's own remaining, in rotational_'s CURRENT frame (whatever the most
  // recent solveToRest()/retarget() established) -- same derivation as
  // maybeReplanDistance()'s planRemainingMag, for the rotational channel.
  Motion::JerkTrajectory::State state = rotational_.sample(rotationalElapsed(now));
  float planRemainingRad = fabsf(rotationalTarget_ - state.position);

  float divergence = fabsf(planRemainingRad - measuredRemainingRad);
  if (divergence < kRotDivergenceThreshold) return;  // within tolerance -- no replan

  // Guard 2 (no-reverse-target): dead-time-project the measured remaining,
  // using the PLAN's own last-sampled rate (never a measured one -- same
  // reasoning as maybeReplanDistance()'s own comment).
  float planSpeedMagRad = fabsf(state.velocity);
  float projectedRemainingRad = measuredRemainingRad - planSpeedMagRad * kDeadTime;
  if (projectedRemainingRad <= 0.0f) return;  // never solve backward

  if (divergence >= kRotGrossDivergenceThreshold) {
    // GROSS case: full re-anchor from measurement. Position: convert the
    // (un-projected) measuredRemainingRad back into rotational_'s current
    // frame, matching this method's own planRemainingRad derivation above.
    // Velocity: always 0.0f -- see this method's own declaration comment
    // (planner.h) on why no reliable measured angular-rate signal exists
    // for either goal kind; reanchor() already accepts a velocity
    // discontinuity at the reseed by design (jerk_trajectory.h).
    float measuredPositionSigned = rotationalTarget_ - omegaSign * measuredRemainingRad;
    rotational_.reanchor(measuredPositionSigned, 0.0f);
    // rotationalTarget_ is unchanged -- reanchor() reuses rotational_'s own
    // internally remembered target (jerk_trajectory.h's own doc comment).
  } else {
    // NORMAL case: retarget() to the dead-time-projected remaining, seeded
    // (internally, by rotational_ itself) from its own last velocity/
    // acceleration -- never from measurement.
    float newRemainingSigned = omegaSign * projectedRemainingRad;
    rotational_.retarget(newRemainingSigned);
    rotationalTarget_ = newRemainingSigned;
  }
  rotationalSolveMs_ = now;
  lastReplanMs_ = now;
}

void Planner::armDistanceStopDecel(uint32_t now) {
  // 089-003, ticket item 4: called the instant a SMOOTH-style DISTANCE
  // goal's stop condition fires. If linear_'s own plan has already
  // naturally converged to rest (the common case -- Decision 4/9), this is
  // a no-op: nothing to re-solve. Otherwise, re-solve a fresh
  // velocity-control decel-to-rest seeded from linear_'s own current state
  // (already refreshed by this tick's earlier sample() call -- Decision 8,
  // never leftObs/rightObs).
  if (linearElapsed(now) >= linear_.duration()) return;
  linear_.solveToVelocity(0.0f, linearCeiling_);
  linearSolveMs_ = now;
}

void Planner::armVelocityStopDecel(uint32_t now) {
  // 089-004: unlike armDistanceStopDecel() above, this does NOT guard on
  // "elapsed >= duration()" first -- see planner.h's own doc comment on
  // why that guard's meaning does not carry over from a position-control
  // plan (where "converged" means "at rest") to a velocity-control cruise
  // (where "converged" means "at cruise speed," very much NOT at rest).
  // Always re-solve both channels' decel-to-zero, seeded from each
  // channel's own current sampled state (Decision 8) -- a channel already
  // at rest (e.g. a straight T's omega == 0 rotational channel) collapses
  // to a trivial, ~zero-duration no-op trajectory on its own, no guard
  // needed.
  linear_.solveToVelocity(0.0f, linearCeiling_);
  linearSolveMs_ = now;
  rotational_.solveToVelocity(0.0f, rotationalCeiling_);
  rotationalSolveMs_ = now;
}

void Planner::armRotationalStopDecel(uint32_t now) {
  // 089-005: TURN/ROTATION's own analog of armDistanceStopDecel() above --
  // reuses ITS "skip if already converged" guard (Pattern A, position-
  // control, like DISTANCE), not armVelocityStopDecel()'s unconditional
  // one. Only the rotational channel needs a re-solve: the linear channel's
  // trivial zero-target solve (stageRotationalGoal()) is always already
  // converged (a ~zero-duration trajectory) by the time any stop fires.
  if (rotationalElapsed(now) >= rotational_.duration()) return;
  rotational_.solveToVelocity(0.0f, rotationalCeiling_);
  rotationalSolveMs_ = now;
}

void Planner::queueEvent(const char* reason) {
  hasEvent_ = true;
  heldEvent_ = msg::Event{};
  heldEvent_.kind = msg::Event::Kind::GOAL_DONE;
  int v = 0;
  for (; verb_[v] != '\0' && v < 7; ++v) heldEvent_.verb[v] = verb_[v];
  heldEvent_.verb[v] = '\0';
  int i = 0;
  for (; reason[i] != '\0' && i < 15; ++i) heldEvent_.reason[i] = reason[i];
  heldEvent_.reason[i] = '\0';
  for (int j = 0; j < 64; ++j) heldEvent_.corrId[j] = corrId_[j];
}

void Planner::holdTwistCommand(float v, float omega) {
  // Cached so state() can report the SAME value regardless of which
  // mechanism (ramp_ or a JerkTrajectory channel) produced it -- see
  // planner.h's currentV_/currentOmega_ doc comment.
  currentV_ = v;
  currentOmega_ = omega;
  msg::BodyTwist3 twist;
  twist.v_x = v;
  twist.v_y = 0.0f;
  twist.omega = omega;
  msg::DrivetrainCommand cmd;
  cmd.setTwist(twist);
  heldCommand_ = cmd;
  hasCommand_ = true;
}

void Planner::tick(uint32_t now, const msg::MotorState& leftObs, const msg::MotorState& rightObs,
                   const msg::PoseEstimate& fusedPose) {
  float dt = haveLastTick_ ? static_cast<int32_t>(now - lastTickMs_) * 0.001f : 0.0f;
  lastTickMs_ = now;
  haveLastTick_ = true;

  // 089-005: the FINAL, stable dispatch (Decision 5, revised) -- a clean
  // `mode_ == GO_TO` binary: GO_TO stays on `ramp_`/`pursueSteer()`
  // (UNCHANGED for the whole sprint); every other goal kind samples the
  // Ruckig channels. `distanceGoal`/`velocityGoal`/`rotationalGoal` are
  // internal bookkeeping WITHIN the "not GO_TO" side (which Ruckig
  // sub-pattern owns this goal -- `mode_ == DISTANCE` is unique, no latch
  // needed; `jerkVelocityGoal_`/`jerkRotationGoal_` disambiguate `T` from
  // `TURN`/`ROTATION`, all three sharing `mode_ == TIMED` via
  // velocityShapedMode()'s own collapse) -- captured ONCE, here, since
  // mode_ itself flips to IDLE the instant a goal completes below, but this
  // tick's OWN output must still route through the right channel(s) either
  // way.
  const bool distanceGoal = (mode_ == msg::DriveMode::DISTANCE);
  const bool velocityGoal = jerkVelocityGoal_;
  const bool rotationalGoal = jerkRotationGoal_;
  float distanceV = 0.0f;      // this tick's sampled linear-channel velocity (DISTANCE only)
  float velocityV = 0.0f;      // this tick's sampled linear-channel velocity (TIMED/VELOCITY/STREAM)
  float velocityOmega = 0.0f;  // this tick's sampled rotational-channel velocity (ditto)
  float rotationalOmega = 0.0f;  // this tick's sampled rotational-channel velocity (TURN/ROTATION)

  if (!activeCmd_) {
    ramp_.reset();
    // Keep linear_/rotational_ pinned at a clean rest baseline while idle
    // too -- the SAME "zero everything while idle" contract ramp_.reset()
    // already has. Without this, a goal ending via STOP or an ABRUPT-fired
    // stop (both force THIS tick's OUTPUT to zero without ever
    // decelerating either channel's own remembered last-sample state)
    // would leave a stale, still-cruising velocity behind for the NEXT
    // goal's staging helper to seed from -- producing a bogus "already at
    // speed" trajectory (a zero-duration jump straight to cruise) instead
    // of a real ramp-up. A no-op for DISTANCE/TURN/ROTATION (whose apply()
    // always calls reset() itself unconditionally already) and for
    // GOTO_GOAL (which never touches these two channels).
    linear_.reset();
    rotational_.reset();
  } else {
    if (!baselineCaptured_) {
      captureBaseline(now, leftObs, rightObs, fusedPose);
      baselineCaptured_ = true;
    }

    if (mode_ == msg::DriveMode::GO_TO) {
      if (gPhase_ == GPhase::PURSUE && !stopping_) {
        // Re-steer toward the world-frame anchor from THIS tick's
        // fusedPose, BEFORE the ramp advances -- ported ordering from
        // source_old's driveAdvance() PURSUE hook (recompute (v, omega),
        // THEN tick the profiler). Gated on !stopping_: once a stop
        // condition has armed the SMOOTH ramp-down (ramp_.setTarget(0, 0)
        // below), this hook must NOT keep re-targeting the ramp away from
        // zero every subsequent tick -- mode_/gPhase_ only flip back to
        // IDLE once the ramp-down actually converges (the `stopping_`
        // branch below), so without this guard PURSUE would fight its own
        // completion indefinitely.
        pursueSteer(fusedPose);
      }
      ramp_.advance(dt);
    } else if (distanceGoal) {
      // No anticipation cap needed here -- unlike the ramp_ path,
      // linear_'s position-control plan already decelerates to rest AT the
      // target as an intrinsic property of the whole-trajectory solve
      // (Decision 4). The divergence-triggered replan (Decision 10) is the
      // ONLY per-tick correction, and only while the goal's own stop has
      // not fired (guard 1).
      if (!stopping_) {
        maybeReplanDistance(now, leftObs, rightObs, fusedPose);
      }
      distanceV = linear_.sample(linearElapsed(now)).velocity;
    } else if (velocityGoal) {
      // 089-004: no per-tick anticipation cap needed here either -- exactly
      // like DISTANCE above, Ruckig's own velocity-control plan (the
      // cruise ramp-up, or the stop-triggered decel-to-zero re-solve below)
      // already produces the commanded shape as an intrinsic property of
      // the solve. Sampled every tick regardless of stopping_: the
      // re-solved decel trajectory armVelocityStopDecel() installs IS what
      // this same sample() call reads once stopping_ is armed -- one
      // mechanism, no separate ramp_.advance()-style branch needed.
      velocityV = linear_.sample(linearElapsed(now)).velocity;
      velocityOmega = rotational_.sample(rotationalElapsed(now)).velocity;
    } else if (rotationalGoal) {
      // 089-005: TURN/ROTATION's own position-control plan, mirroring
      // DISTANCE's branch above -- the divergence-triggered replan
      // (Decision 10) is the only per-tick correction, only while the
      // goal's own stop has not fired (guard 1). The linear channel's
      // trivial zero-target solve is sampled too (never specially
      // skipped -- ticket item 3) but its value is not needed for the
      // held twist (v = 0 always for a turn-in-place goal).
      if (!stopping_) {
        maybeReplanRotational(now, leftObs, rightObs, fusedPose);
      }
      linear_.sample(linearElapsed(now));
      float rotElapsed = rotationalElapsed(now);
      rotationalOmega = rotational_.sample(rotElapsed).velocity;
      // Snap to a LITERAL 0.0f once the STOP-TRIGGERED decel-to-zero
      // (armRotationalStopDecel()) has fully converged (elapsed >=
      // duration(), only while stopping_) instead of trusting the raw
      // sampled value. Gated on stopping_ specifically -- NOT applied to
      // the ONGOING (not-yet-stopped) position-control solve, where
      // reaching elapsed >= duration() naturally (the plan's own target
      // converges) must keep sampling the real value: the divergence-
      // triggered replan (maybeReplanRotational()) is what is supposed to
      // keep re-extending that plan against a lagging real plant, and
      // forcing a hard 0 here would fight it, stalling the goal short
      // (confirmed by direct regression: planner_harness.cpp's own lagging-
      // plant scenario stopped completing via reason=rot once this snap was
      // applied unconditionally). Once stopping_, though, every solve
      // armRotationalStopDecel() makes targets EXACTLY 0 rad/s at its own
      // terminal state -- so "converged" here always, unconditionally,
      // means the TRUE omega is 0. Ruckig's own past-duration "hold at
      // final state" (jerk_trajectory.h) does not guarantee this is
      // BIT-EXACT the way Motion::VelocityRamp's own approach() did
      // (`cur + (tgt - cur)` cancels to EXACTLY tgt when tgt == 0, by IEEE
      // 754 construction, once |tgt - cur| <= one tick's step) -- the
      // polynomial `at_time()` evaluation behind Ruckig's own solve can
      // leave a ~1e-15-scale residual instead. That residual is
      // functionally negligible on its own, but it defeats Hal::
      // MotorVelocityPid's own zero-threshold deadband (`spAbs <= minDuty`,
      // minDuty == 0.0f for an unconfigured stiction floor): the
      // integrator-freeze fix (086-002) never engages for a target that
      // never reaches a LITERAL 0.0f, so the (otherwise correctly frozen)
      // integrator keeps actively chasing ordinary wheel-coast/
      // quantization noise, producing a sustained, slowly-decaying
      // reverse-spin residual -- confirmed by direct measurement (a dense
      // post-completion PID trace) during this ticket's own debugging of
      // test_motion_overshoot_regression.py's RT 9000 settle check.
      if (stopping_ && rotElapsed >= rotational_.duration()) {
        rotationalOmega = 0.0f;
      }
    }

    if (stopping_) {
      // SMOOTH ramp-down (or a position-control plan's decel-to-rest
      // re-solve) in progress: terminate once converged or the soft
      // deadline passes (matches source_old/commands/MotionCommand.cpp's
      // tick() stopping sub-phase).
      bool converged;
      if (distanceGoal) {
        converged = linearElapsed(now) >= linear_.duration();
      } else if (velocityGoal) {
        // 089-004: both channels must have converged on their own
        // (possibly different-duration, Decision 1) decel-to-zero re-solve
        // -- e.g. a T with unequal l/r targets may finish rotating before
        // (or after) it finishes translating, same as VelocityRamp's own
        // independent-channel advance() already exhibited.
        converged = linearElapsed(now) >= linear_.duration() &&
                    rotationalElapsed(now) >= rotational_.duration();
      } else if (rotationalGoal) {
        // 089-005: only the rotational channel's own convergence matters --
        // the linear channel's trivial zero-target solve is always already
        // converged (armRotationalStopDecel()'s own doc comment).
        converged = rotationalElapsed(now) >= rotational_.duration();
      } else {
        converged = ramp_.atTarget();
      }
      int32_t dtDeadline = static_cast<int32_t>(now - softDeadline_);
      if (converged || dtDeadline >= 0) {
        queueEvent(pendingReason_);
        activeCmd_ = false;
        stopping_ = false;
        mode_ = msg::DriveMode::IDLE;
        jerkVelocityGoal_ = false;
        jerkRotationGoal_ = false;
        gPhase_ = GPhase::IDLE;
      }
    } else {
      // Normal running sub-phase: evaluate stop conditions, OR-combined --
      // the first one to fire terminates the goal. UNCHANGED by 089-003
      // (Decision 4) -- Motion::evaluateStopCondition() stays authoritative
      // regardless of which mechanism is shaping the commanded velocity.
      for (uint8_t i = 0; i < stopsCount_; ++i) {
        Motion::StopEvalResult r =
            Motion::evaluateStopCondition(stops_[i], baseline_, now, leftObs, rightObs, fusedPose);
        if (r == Motion::StopEvalResult::FIRED) {
          if (mode_ == msg::DriveMode::GO_TO && gPhase_ == GPhase::PRE_ROTATE &&
              stops_[i].kind == msg::StopKind::STOP_HEADING) {
            // Bearing gate reached: hand off to PURSUE -- not a goal
            // completion (no event, no ramp-down). Ported concept:
            // source_old's driveAdvance() PRE_ROTATE-terminated branch's
            // HEADING-fired case.
            enterPursue(now);
            break;
          }
          const char* reason = reasonTokenFor(stops_[i].kind);
          if (style_ == msg::StopStyle::ABRUPT) {
            if (distanceGoal) {
              // Immediate halt, no smooth ramp-down, regardless of
              // mechanism -- this tick's held output is forced to 0.
              distanceV = 0.0f;
            } else if (velocityGoal) {
              velocityV = 0.0f;
              velocityOmega = 0.0f;
            } else if (rotationalGoal) {
              rotationalOmega = 0.0f;
            } else {
              ramp_.reset();
            }
            queueEvent(reason);
            activeCmd_ = false;
            stopping_ = false;
            mode_ = msg::DriveMode::IDLE;
            jerkVelocityGoal_ = false;
            jerkRotationGoal_ = false;
            gPhase_ = GPhase::IDLE;
          } else {
            // SMOOTH: ramp/decel to rest; tick() emits the event once
            // converged or the soft deadline passes (the `stopping_`
            // branch above).
            stopping_ = true;
            softDeadline_ = now + kSoftDeadlineMs;
            pendingReason_ = reason;
            if (distanceGoal) {
              armDistanceStopDecel(now);
            } else if (velocityGoal) {
              armVelocityStopDecel(now);
            } else if (rotationalGoal) {
              armRotationalStopDecel(now);
            } else {
              ramp_.setTarget(0.0f, 0.0f);
            }
          }
          break;
        }
        // UNSUPPORTED is treated identically to NOT_FIRED here -- see
        // motion/stop_condition.h's class comment.
      }
    }
  }

  if (distanceGoal) {
    holdTwistCommand(distanceV, 0.0f);
  } else if (velocityGoal) {
    holdTwistCommand(velocityV, velocityOmega);
  } else if (rotationalGoal) {
    holdTwistCommand(0.0f, rotationalOmega);
  } else {
    holdTwistCommand(ramp_.currentV(), ramp_.currentOmega());
  }
}

bool Planner::hasCommand() const { return hasCommand_; }

msg::DrivetrainCommand Planner::takeCommand() {
  hasCommand_ = false;
  return heldCommand_;
}

bool Planner::hasEvent() const { return hasEvent_; }

msg::Event Planner::takeEvent() {
  hasEvent_ = false;
  return heldEvent_;
}

msg::PlannerState Planner::state() const {
  msg::PlannerState s;
  s.mode = mode_;
  s.target_x = targetX_;
  s.target_y = targetY_;
  s.target_speed = targetSpeed_;
  s.distance_target = distanceTarget_;
  s.deadline = stopping_ ? softDeadline_ : 0;
  // 089-003: currentV_/currentOmega_ (cached by holdTwistCommand(), the
  // SAME call tick() uses to hold the output takeCommand() returns) rather
  // than ramp_.currentV()/currentOmega() directly -- ramp_ is untouched for
  // a JerkTrajectory-driven goal kind like DISTANCE, so reading it here
  // would report a stale value. See planner.h's currentV_/currentOmega_
  // doc comment.
  s.body_twist.v_x = currentV_;
  s.body_twist.v_y = 0.0f;
  s.body_twist.omega = currentOmega_;
  s.active = activeCmd_;
  return s;
}

void Planner::configure(const msg::PlannerConfig& config) {
  config_ = config;
  ramp_.configure(config);
  // 089-003/004: both channels configured here. linear_ does real work for
  // DISTANCE (089-003) and TIMED/VELOCITY/STREAM (089-004, both channels);
  // rotational_ is unused by TURN/ROTATION until ticket 005 but is kept
  // configured regardless rather than left silently unready (planner.h's
  // class comment, architecture-update.md (089) Decision 1/ticket item 1).
  linear_.configure(config, /*isRotational=*/false);
  rotational_.configure(config, /*isRotational=*/true);
}

bool Planner::hasActiveCommand() const { return activeCmd_; }

}  // namespace Subsystems
