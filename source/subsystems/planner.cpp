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
// two-output-pass dead-time compensation (see applyStopAnticipation()'s own
// doc comment below for the full derivation/rationale). Hoisted to file
// scope (089-003) so BOTH applyStopAnticipation() (STOP_DISTANCE/
// STOP_ROTATION's anticipation cap -- STOP_DISTANCE has been dead code since
// 089-003, STOP_HEADING/STOP_ROTATION still serving TURN/ROTATION until
// ticket 005; TIMED/VELOCITY/STREAM no longer reach this function as of
// 089-004) and maybeReplanDistance() (DISTANCE's new divergence-triggered
// replan, architecture-update.md (089) Decision 10) share the SAME tau
// definition, per that Decision's own instruction to reuse it rather than
// redefine one under a new name.
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
  // 089-004: reset unconditionally, same reasoning as gPhase_ above --
  // stageVelocityGoal() (TIMED/VELOCITY/STREAM cases only, below) is the
  // ONLY place that sets this back to true; every other goal_kind case
  // (including TURN/ROTATION, which also stage mode_ == TIMED) leaves it
  // false, which is exactly the discrimination tick() needs (class comment).
  jerkVelocityGoal_ = false;

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
      copyCallerStops(cmd);
      stageGoal(0.0f, cmd.goal.turn.speed, velocityShapedMode(cmd), cmd);
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
      copyCallerStops(cmd);
      stageGoal(0.0f, cmd.goal.rotation.speed, velocityShapedMode(cmd), cmd);
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

void Planner::applyStopAnticipation(const msg::MotorState& leftObs,
                                    const msg::MotorState& rightObs,
                                    const msg::PoseEstimate& fusedPose) {
  // Start from the staged (v, omega) -- the ramp's own eventual target
  // absent any cap -- and tighten whichever component an active DISTANCE/
  // ROTATION/HEADING stop's remaining-to-go geometry says to tighten. See
  // planner.h's class comment for why GO_TO never reaches this method.
  float v = stagedV_;
  float omega = stagedOmega_;

  // kOutputHops/kAssumedPassPeriod/kDeadTime (ticket 087-009) --
  // architecture-update-r1.md's Decision 6/2 adds TWO full passes of dead
  // time between this tick computing a cap and that cap actually reaching
  // the wheel's commanded PWM: Planner's output -> bb.driveIn, drained by
  // Drivetrain next pass; Drivetrain's own output -> bb.motorIn[], drained
  // by Hardware the pass after THAT (see ticket 007's completion notes' own
  // "TWO passes" accounting) -- versus ticket 006's transitional same-pass
  // `hardware.apply(drivetrain.takeCommand())`. kAssumedPassPeriod matches
  // main.cpp's own `kPeriod` (the loop's target best-effort cadence) --
  // deliberately a FIXED constant, NOT this tick's own measured `now` delta:
  // an earlier version of this fix used the measured delta, which is a
  // reasonable proxy in the real loop (where Planner ticks exactly once per
  // mandatory pass) but silently breaks in planner_harness.cpp's own
  // scenarios, which deliberately advance `now` by large, non-representative
  // steps (e.g. a full simulated second) to force ramp convergence within a
  // single call -- caught by that harness's own tight, hand-derived
  // expected-value assertions failing once this fix used the measured delta.
  // A fixed constant sidesteps that fragility entirely.
  //
  // Below, the STOP_DISTANCE/STOP_ROTATION branches fold that dead time into
  // the cap using the closed-form "stopping distance with a reaction time"
  // solution (the same shape as the highway stopping-sight-distance formula,
  // d = v*T + v^2/(2a)) -- solved for v given d = remaining:
  //   v = -a*T + sqrt((a*T)^2 + 2*a*remaining)
  // which reduces exactly to the un-compensated sqrt(2*a*remaining) when
  // T == 0. This is a PURE FUNCTION of `remaining` (the encoder-derived
  // geometry) and constants (a_decel/yaw_acc_max, kDeadTime) -- deliberately
  // NOT a function of the CURRENTLY MEASURED wheel speed. An earlier version
  // of this fix subtracted `measuredSpeed * deadTime` from `remaining`
  // directly, which closes a loop through the plant's own (delayed)
  // velocity response: as the cap drives speed down, the measured-speed
  // term shrinks, which relaxes the cap, which lets speed climb back up --
  // a genuine limit-cycle oscillation, caught by tracing a dense per-tick
  // velocity trace through D 200 200 700's terminal approach (velocity
  // dipped to ~0 mid-approach, then rebounded to 72mm/s just before the stop
  // fired). The closed-form formula below has no such feedback path: it
  // depends only on `remaining`, which shrinks monotonically, so the cap it
  // produces is monotonic too.
  //
  // kOutputHops/kAssumedPassPeriod/kDeadTime themselves now live at file
  // scope (089-003) -- see the anonymous namespace above -- so
  // maybeReplanDistance()'s divergence-triggered replan (architecture-
  // update.md (089) Decision 10) can reuse the SAME tau definition rather
  // than redefining one under a new name; no behavior change here.

  for (uint8_t i = 0; i < stopsCount_; ++i) {
    const msg::StopCondition& cond = stops_[i];
    float remaining = 0.0f;
    Motion::StopEvalResult r =
        Motion::remainingToStop(cond, baseline_, leftObs, rightObs, fusedPose, &remaining);
    if (r == Motion::StopEvalResult::UNSUPPORTED) continue;  // STOP_TIME etc -- no geometry here

    if (cond.kind == msg::StopKind::STOP_DISTANCE) {
      // pursueSteer()-style linear terminal decel cap, dead-time-compensated
      // (see this method's own comment above).
      float reach = config_.a_decel * kDeadTime;
      float vCap = -reach + sqrtf(reach * reach + 2.0f * config_.a_decel * remaining);
      float mag = fminf(fabsf(v), vCap);
      v = (stagedV_ < 0.0f) ? -mag : mag;
    } else if (cond.kind == msg::StopKind::STOP_HEADING) {
      // TURN's angular-rate cap: remaining is a genuine heading error (rad),
      // dimensionally consistent with yaw_acc_max (rad/s^2) -- exact analog
      // of the DISTANCE cap above.
      float omegaCap = sqrtf(fmaxf(0.0f, 2.0f * config_.yaw_acc_max * remaining));
      float mag = fminf(fabsf(omega), omegaCap);
      omega = (stagedOmega_ < 0.0f) ? -mag : mag;
    } else if (cond.kind == msg::StopKind::STOP_ROTATION) {
      // ROTATION's angular-rate cap: remaining here is a per-wheel ARC (mm,
      // see evaluateStopCondition()'s own STOP_ROTATION comment), not an
      // angle -- Planner has no DrivetrainConfig.trackwidth to convert arc
      // mm <-> rad with (see class comment on why RotationGoal.speed is
      // accepted pre-signed instead). Applying yaw_acc_max (rad/s^2)
      // directly to an mm-valued remaining is therefore a deliberate,
      // documented approximation, not a unit-correct derivation: it still
      // gives the right SHAPE of cap (0 at remaining==0, growing with
      // remaining, so it never binds far from the stop and always binds at
      // it) using the only two numbers this class actually has (omega,
      // yaw_acc_max) -- the same concept-not-byte-for-byte simplification
      // precedent already established for RT's coast-anticipation/
      // rotational-slip (planner.h's GOTO_GOAL class comment).
      //
      // Dead-time compensation (087-009): same closed-form reaction-time cap
      // as STOP_DISTANCE above (see this method's own comment), applied in
      // this same already-approximated per-wheel-ARC-mm domain. Kept for
      // consistency/future-proofing even though it is provably a NO-OP at
      // today's config values: RT's omega (kRotationOmega, motion_commands.
      // cpp, ~1.745 rad/s) vs. yaw_acc_max (20 rad/s^2) only cross at
      // remaining = omega^2/(2*yaw_acc_max) =~ 0.076mm -- out of a ~100mm
      // total arc (90deg at the default 128mm trackwidth) -- far below one
      // tick's own arc travel (~2.7mm at this omega), so the cap never binds
      // in practice, before OR after this ticket's dead-time term (a min()
      // against a bound that high above the staged omega never changes the
      // result). This matches 086-003's own completion notes ("does not
      // close RT's own overshoot to near-zero the way it closed D 200 200
      // 500's"): RT's terminal overshoot is dominated by the SMOOTH
      // ramp-down's post-fire coast, not this pre-fire cap -- see ticket
      // 087-009's completion notes for why that coast is left un-recovered.
      float reach = config_.yaw_acc_max * kDeadTime;
      float omegaCap = -reach + sqrtf(reach * reach + 2.0f * config_.yaw_acc_max * remaining);
      float mag = fminf(fabsf(omega), omegaCap);
      omega = (stagedOmega_ < 0.0f) ? -mag : mag;
    }
  }

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
  // find it the same way applyStopAnticipation() finds its own stops,
  // rather than assuming stops_[0] (robust against a future stops_[]
  // ordering change).
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

void Planner::queueEvent(const char* reason) {
  hasEvent_ = true;
  heldEvent_ = Event{};
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

  // 089-003/004 KNOWN INTERMEDIATE STATE (see planner.h's class comment):
  // goal-kind-aware checks -- `mode_ == DISTANCE` (unique, no latch needed)
  // and the `jerkVelocityGoal_` latch (needed because mode_ == TIMED is
  // shared with TURN/ROTATION, velocityShapedMode()'s own collapse) --
  // captured ONCE, here, since mode_ itself flips to IDLE the instant a
  // goal completes below, but this tick's OWN output must still route
  // through the right channel(s) either way. Ticket 005 migrates TURN/
  // ROTATION the same way, after which this collapses to the clean
  // `mode_ == GO_TO` vs. not split (Decision 5) -- do not generalize early.
  const bool distanceGoal = (mode_ == msg::DriveMode::DISTANCE);
  const bool velocityGoal = jerkVelocityGoal_;
  float distanceV = 0.0f;      // this tick's sampled linear-channel velocity (DISTANCE only)
  float velocityV = 0.0f;      // this tick's sampled linear-channel velocity (TIMED/VELOCITY/STREAM)
  float velocityOmega = 0.0f;  // this tick's sampled rotational-channel velocity (ditto)

  if (!activeCmd_) {
    ramp_.reset();
    // 089-004: keep linear_/rotational_ pinned at a clean rest baseline
    // while idle too -- the SAME "zero everything while idle" contract
    // ramp_.reset() already has. Without this, a goal ending via STOP or an
    // ABRUPT-fired stop (both force THIS tick's OUTPUT to zero without
    // ever decelerating either channel's own remembered last-sample state)
    // would leave a stale, still-cruising velocity behind for
    // stageVelocityGoal()'s NEXT solveToVelocity() call to seed from --
    // producing a bogus "already at speed" trajectory (a zero-duration
    // jump straight to cruise) instead of a real ramp-up. A no-op for
    // DISTANCE (whose apply() always calls linear_.reset() itself
    // unconditionally already) and for TURN/ROTATION/GOTO_GOAL (which never
    // touch these two channels).
    linear_.reset();
    rotational_.reset();
  } else {
    if (!baselineCaptured_) {
      captureBaseline(now, leftObs, rightObs, fusedPose);
      baselineCaptured_ = true;
    }

    if (distanceGoal) {
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
    } else if (mode_ == msg::DriveMode::GO_TO && gPhase_ == GPhase::PURSUE && !stopping_) {
      // Re-steer toward the world-frame anchor from THIS tick's fusedPose,
      // BEFORE the ramp advances -- ported ordering from source_old's
      // driveAdvance() PURSUE hook (recompute (v, omega), THEN tick the
      // profiler). Gated on !stopping_: once a stop condition has armed the
      // SMOOTH ramp-down (ramp_.setTarget(0, 0) below), this hook must NOT
      // keep re-targeting the ramp away from zero every subsequent tick --
      // mode_/gPhase_ only flip back to IDLE once the ramp-down actually
      // converges (the `stopping_` branch below), so without this guard
      // PURSUE would fight its own completion indefinitely.
      pursueSteer(fusedPose);
      ramp_.advance(dt);
    } else if (mode_ != msg::DriveMode::GO_TO && !stopping_) {
      // 086-003: the same anticipation pattern, extended to DISTANCE/TURN/
      // ROTATION -- GO_TO is excluded (PURSUE's pursueSteer() above already
      // owns its own anticipation; PRE_ROTATE's STOP_HEADING is a
      // phase-handoff gate, not a terminal stop -- see planner.h's class
      // comment on applyStopAnticipation()). Gated on !stopping_ for the
      // same reason pursueSteer()'s own call is: once a stop condition has
      // armed the SMOOTH ramp-down (ramp_.setTarget(0, 0) below), this must
      // not keep re-targeting the ramp away from zero. 089-003: DISTANCE no
      // longer reaches this branch (distanceGoal above takes it first) --
      // applyStopAnticipation()'s STOP_DISTANCE branch has been DEAD CODE
      // for DISTANCE since then. 089-004: TIMED/VELOCITY/STREAM no longer
      // reach this branch either (velocityGoal above takes them first) --
      // applyStopAnticipation() itself is UNCHANGED and still live only for
      // TURN/ROTATION's STOP_HEADING/STOP_ROTATION branches, until ticket
      // 005.
      applyStopAnticipation(leftObs, rightObs, fusedPose);
      ramp_.advance(dt);
    } else {
      ramp_.advance(dt);
    }

    if (stopping_) {
      // SMOOTH ramp-down (or DISTANCE's decel-to-rest re-solve) in
      // progress: terminate once converged or the soft deadline passes
      // (matches source_old/commands/MotionCommand.cpp's tick() stopping
      // sub-phase).
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
            } else {
              ramp_.reset();
            }
            queueEvent(reason);
            activeCmd_ = false;
            stopping_ = false;
            mode_ = msg::DriveMode::IDLE;
            jerkVelocityGoal_ = false;
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

Planner::Event Planner::takeEvent() {
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
