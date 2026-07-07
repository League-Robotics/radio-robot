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

void Planner::stageGoal(float v, float omega, msg::DriveMode mode,
                        const msg::PlannerCommand& cmd) {
  ramp_.setTarget(v, omega);
  stagedV_ = v;
  stagedOmega_ = omega;
  style_ = cmd.style;
  for (int i = 0; i < 64; ++i) corrId_[i] = cmd.corr_id[i];
  stopping_ = false;
  baselineCaptured_ = false;
  activeCmd_ = true;
  mode_ = mode;
}

void Planner::apply(const msg::PlannerCommand& cmd, uint32_t now) {
  // apply() captures no MotionBaseline -- see the class comment. `now` is
  // accepted for the ticket-locked signature and is currently unused; a
  // future ticket may want a staged-but-not-yet-ticked timestamp.
  (void)now;

  targetX_ = 0.0f;
  targetY_ = 0.0f;
  targetSpeed_ = 0.0f;
  distanceTarget_ = 0.0f;
  // Reset unconditionally so a new non-GOTO_GOAL command issued while a G
  // goal was still in-flight leaves no stale phase behind (gPhase_ is only
  // ever consulted while mode_ == GO_TO, so this matters for hygiene/
  // debuggability more than correctness -- see class comment).
  gPhase_ = GPhase::IDLE;

  switch (cmd.goal_kind) {
    case msg::PlannerCommand::GoalKind::VELOCITY: {
      // v_y is intentionally ignored -- mirrors Drivetrain::setTwist()'s own
      // v_y-ignored precedent (differential-only this sprint).
      copyCallerStops(cmd);
      targetSpeed_ = cmd.goal.velocity.v_x;
      stageGoal(cmd.goal.velocity.v_x, cmd.goal.velocity.omega, velocityShapedMode(cmd), cmd);
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
      // 2s) computed purely from this goal's own fields.
      appendStop(msg::StopKind::STOP_DISTANCE, mag);
      float spdMax = fabsf(speed);
      if (spdMax < 1.0f) spdMax = 1.0f;
      float nominal = (mag / spdMax) * 1000.0f;  // [ms]
      appendStop(msg::StopKind::STOP_TIME, nominal * 2.0f + 2000.0f);

      distanceTarget_ = distance;
      stageGoal(v, 0.0f, msg::DriveMode::DISTANCE, cmd);
      break;
    }

    case msg::PlannerCommand::GoalKind::TIMED: {
      copyCallerStops(cmd);
      if (cmd.goal.timed.duration > 0) {
        // Implicit STOP_TIME from the goal's own duration field (ported
        // concept from source_old's beginTimed()).
        appendStop(msg::StopKind::STOP_TIME, static_cast<float>(cmd.goal.timed.duration));
      }
      stageGoal(cmd.goal.timed.v_x, cmd.goal.timed.omega, msg::DriveMode::TIMED, cmd);
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
      // v_y is intentionally ignored -- see the VELOCITY case above.
      copyCallerStops(cmd);
      stageGoal(cmd.goal.stream.v_x, cmd.goal.stream.omega, msg::DriveMode::STREAMING, cmd);
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
  constexpr float kOutputHops = 2.0f;
  constexpr float kAssumedPassPeriod = 0.020f;  // [s] matches main.cpp's kPeriod
  constexpr float kDeadTime = kOutputHops * kAssumedPassPeriod;  // [s]

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

void Planner::queueEvent(const char* reason) {
  hasEvent_ = true;
  heldEvent_ = Event{};
  int i = 0;
  for (; reason[i] != '\0' && i < 15; ++i) heldEvent_.reason[i] = reason[i];
  heldEvent_.reason[i] = '\0';
  for (int j = 0; j < 64; ++j) heldEvent_.corrId[j] = corrId_[j];
}

void Planner::holdTwistCommand(float v, float omega) {
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

  if (!activeCmd_) {
    ramp_.reset();
  } else {
    if (!baselineCaptured_) {
      captureBaseline(now, leftObs, rightObs, fusedPose);
      baselineCaptured_ = true;
    }

    if (mode_ == msg::DriveMode::GO_TO && gPhase_ == GPhase::PURSUE && !stopping_) {
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
    } else if (mode_ != msg::DriveMode::GO_TO && !stopping_) {
      // 086-003: the same anticipation pattern, extended to DISTANCE/TURN/
      // ROTATION -- GO_TO is excluded (PURSUE's pursueSteer() above already
      // owns its own anticipation; PRE_ROTATE's STOP_HEADING is a
      // phase-handoff gate, not a terminal stop -- see planner.h's class
      // comment on applyStopAnticipation()). Gated on !stopping_ for the
      // same reason pursueSteer()'s own call is: once a stop condition has
      // armed the SMOOTH ramp-down (ramp_.setTarget(0, 0) below), this must
      // not keep re-targeting the ramp away from zero.
      applyStopAnticipation(leftObs, rightObs, fusedPose);
    }

    ramp_.advance(dt);

    if (stopping_) {
      // SMOOTH ramp-down in progress: terminate once converged or the soft
      // deadline passes (matches source_old/commands/MotionCommand.cpp's
      // tick() stopping sub-phase).
      bool converged = ramp_.atTarget();
      int32_t dtDeadline = static_cast<int32_t>(now - softDeadline_);
      if (converged || dtDeadline >= 0) {
        queueEvent(pendingReason_);
        activeCmd_ = false;
        stopping_ = false;
        mode_ = msg::DriveMode::IDLE;
        gPhase_ = GPhase::IDLE;
      }
    } else {
      // Normal running sub-phase: evaluate stop conditions, OR-combined --
      // the first one to fire terminates the goal.
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
            ramp_.reset();
            queueEvent(reason);
            activeCmd_ = false;
            stopping_ = false;
            mode_ = msg::DriveMode::IDLE;
            gPhase_ = GPhase::IDLE;
          } else {
            // SMOOTH: ramp to (0,0); tick() emits the event once converged
            // or the soft deadline passes (the `stopping_` branch above).
            stopping_ = true;
            softDeadline_ = now + kSoftDeadlineMs;
            pendingReason_ = reason;
            ramp_.setTarget(0.0f, 0.0f);
          }
          break;
        }
        // UNSUPPORTED is treated identically to NOT_FIRED here -- see
        // motion/stop_condition.h's class comment.
      }
    }
  }

  holdTwistCommand(ramp_.currentV(), ramp_.currentOmega());
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
  s.body_twist.v_x = ramp_.currentV();
  s.body_twist.v_y = 0.0f;
  s.body_twist.omega = ramp_.currentOmega();
  s.active = activeCmd_;
  return s;
}

void Planner::configure(const msg::PlannerConfig& config) {
  config_ = config;
  ramp_.configure(config);
}

bool Planner::hasActiveCommand() const { return activeCmd_; }

}  // namespace Subsystems
