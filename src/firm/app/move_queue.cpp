// move_queue.cpp -- App::MoveQueue implementation. See move_queue.h's file
// header for the module's boundary and storage rationale.
#include "app/move_queue.h"

namespace App {

MoveQueue::MoveQueue(Drive& drive, Odometry& odom, const Devices::Clock& clock)
    : drive_(drive), odom_(odom), clock_(clock) {}

void MoveQueue::activate(const msg::Move& move, uint64_t now, float pathLength, float theta) {
  if (move.velocity_kind == msg::Move::VelocityKind::WHEELS) {
    drive_.setWheels(move.velocity.wheels.v_left, move.velocity.wheels.v_right);
  } else {
    // TWIST (and the defensive NONE fallback -- see move_queue.h's own
    // header: a well-formed Move never reaches here with velocity_kind ==
    // NONE, that shape check is RobotLoop::handleMove()'s job).
    drive_.setTwist(move.velocity.twist.v_x, move.velocity.twist.v_y, move.velocity.twist.omega);
  }

  Motion::StopCondition::Kind kind = Motion::StopCondition::Kind::Time;
  float threshold = 0.0f;
  switch (move.stop_kind) {
    case msg::Move::StopKind::DISTANCE:
      kind = Motion::StopCondition::Kind::Distance;
      threshold = move.stop.distance;
      break;
    case msg::Move::StopKind::ANGLE:
      kind = Motion::StopCondition::Kind::Angle;
      threshold = move.stop.angle;
      break;
    case msg::Move::StopKind::TIME:
    case msg::Move::StopKind::NONE:
    default:
      // NONE is the same defensive fallback as VelocityKind::NONE above --
      // a well-formed Move never reaches here with stop_kind == NONE.
      kind = Motion::StopCondition::Kind::Time;
      threshold = move.stop.time;
      break;
  }

  active_.occupied = true;
  active_.moveId = move.id;
  active_.kind = kind;
  active_.threshold = threshold;
  active_.timeout = move.timeout;
  active_.activationNow = now;
  active_.activationPathLength = pathLength;
  active_.activationTheta = theta;
}

MoveQueue::EnqueueResult MoveQueue::enqueue(const msg::Move& move, uint32_t corrId) {
  EnqueueResult result;
  result.corrId = corrId;

  if (move.replace) {
    pendingCount_ = 0;  // flush -- no completion ack for any of them
    activate(move, clock_.nowMicros(), odom_.pathLength(), odom_.theta());
    return result;
  }

  if (!active_.occupied) {
    // Queue was empty -- nothing to flush/preempt, but the activation
    // itself is identical to the replace==true path above.
    activate(move, clock_.nowMicros(), odom_.pathLength(), odom_.theta());
    return result;
  }

  if (pendingCount_ >= kMaxPending) {
    // Nothing above this line mutated any queue state -- the existing
    // active Move and all 4 pending Moves are provably unchanged.
    result.err = msg::ErrCode::ERR_FULL;
    return result;
  }

  pending_[pendingCount_] = move;
  ++pendingCount_;
  return result;
}

MoveQueue::TickResult MoveQueue::tick(uint64_t now, const Odometry& odom) {
  TickResult result;
  if (!active_.occupied) return result;

  Motion::StopCondition sc(active_.kind, active_.threshold, active_.timeout,
                            active_.activationNow, active_.activationPathLength,
                            active_.activationTheta);
  Motion::StopCondition::Outcome outcome = sc.tick(now, odom.pathLength(), odom.theta());
  if (outcome == Motion::StopCondition::Outcome::Continue) return result;

  result.completed = true;
  result.completion.moveId = active_.moveId;
  result.completion.timedOut = (outcome == Motion::StopCondition::Outcome::TimedOut);

  active_.occupied = false;

  if (pendingCount_ > 0) {
    msg::Move next = pending_[0];
    for (int i = 1; i < pendingCount_; ++i) pending_[i - 1] = pending_[i];
    --pendingCount_;
    activate(next, now, odom.pathLength(), odom.theta());
  } else {
    drive_.stop();
  }

  return result;
}

void MoveQueue::flush() {
  pendingCount_ = 0;
  active_.occupied = false;
  active_.moveId = 0;
  drive_.stop();
}

}  // namespace App
