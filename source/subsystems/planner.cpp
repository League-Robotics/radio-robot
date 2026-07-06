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

}  // namespace

void Planner::copyCallerStops(const msg::PlannerCommand& cmd) {
  stopsCount_ = cmd.stops_count_val();
  if (stopsCount_ > 4) stopsCount_ = 4;
  for (uint8_t i = 0; i < stopsCount_; ++i) {
    stops_[i] = cmd.stops()[i];
  }
}

void Planner::appendStop(msg::StopKind kind, float a) {
  if (stopsCount_ >= 4) return;  // cap already full -- see class comment
  msg::StopCondition c;
  c.kind = kind;
  c.a = a;
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
      // Placeholder this ticket -- see class comment (ticket 084-004 ports
      // the PRE_ROTATE/PURSUE state machine here). Straight-line hold at the
      // commanded speed; termination relies entirely on cmd.stops_[].
      copyCallerStops(cmd);
      targetX_ = cmd.goal.goto_goal.x;
      targetY_ = cmd.goal.goto_goal.y;
      targetSpeed_ = cmd.goal.goto_goal.speed;
      stageGoal(cmd.goal.goto_goal.speed, 0.0f, msg::DriveMode::GO_TO, cmd);
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
      }
    } else {
      // Normal running sub-phase: evaluate stop conditions, OR-combined --
      // the first one to fire terminates the goal.
      for (uint8_t i = 0; i < stopsCount_; ++i) {
        Motion::StopEvalResult r =
            Motion::evaluateStopCondition(stops_[i], baseline_, now, leftObs, rightObs, fusedPose);
        if (r == Motion::StopEvalResult::FIRED) {
          const char* reason = reasonTokenFor(stops_[i].kind);
          if (style_ == msg::StopStyle::ABRUPT) {
            ramp_.reset();
            queueEvent(reason);
            activeCmd_ = false;
            stopping_ = false;
            mode_ = msg::DriveMode::IDLE;
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
