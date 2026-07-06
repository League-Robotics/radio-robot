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
      stageGoal(cmd.goal.velocity.v_x, cmd.goal.velocity.omega, msg::DriveMode::VELOCITY, cmd);
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
      // comment. Turn-in-place: v = 0.
      copyCallerStops(cmd);
      stageGoal(0.0f, cmd.goal.turn.speed, msg::DriveMode::VELOCITY, cmd);
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
      // it). Turn-in-place: v = 0.
      copyCallerStops(cmd);
      stageGoal(0.0f, cmd.goal.rotation.speed, msg::DriveMode::VELOCITY, cmd);
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
