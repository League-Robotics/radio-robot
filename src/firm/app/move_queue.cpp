// move_queue.cpp -- App::MoveQueue implementation. See move_queue.h's file
// header for the module's boundary and storage rationale.
#include "app/move_queue.h"

#include <cmath>
#include <limits>

#include "motion/velocity_shaper.h"

namespace App {

MoveQueue::MoveQueue(Drive& drive, Odometry& odom, const Devices::Clock& clock,
                      const StateEstimator& stateEstimator, uint32_t stopLead,
                      ShaperLimits shaperLimits)
    : drive_(drive), odom_(odom), clock_(clock), stateEstimator_(stateEstimator),
      stopLead_(stopLead), shaperLimits_(shaperLimits) {}

void MoveQueue::activate(const msg::Move& move, uint64_t now, float pathLength, float theta) {
  // Disabled-axis gate -- see ShaperLimits's own doc comment (move_queue.h)
  // for the "0 == off, byte-identical to pre-shaping behavior" contract.
  bool linearShaping =
      shaperLimits_.aMax > 0.0f && shaperLimits_.aDecel > 0.0f && shaperLimits_.jMax > 0.0f;
  bool angularShaping = shaperLimits_.alphaMax > 0.0f && shaperLimits_.alphaDecel > 0.0f &&
                        shaperLimits_.yawJerkMax > 0.0f;

  active_.velocityKind = move.velocity_kind;

  if (move.velocity_kind == msg::Move::VelocityKind::WHEELS) {
    active_.cruiseVLeft = move.velocity.wheels.v_left;
    active_.cruiseVRight = move.velocity.wheels.v_right;
    active_.cruiseVX = 0.0f;
    active_.cruiseVY = 0.0f;
    active_.cruiseOmega = 0.0f;

    // Shaping enabled: stage the CARRIED-OVER running shaper state (0 on a
    // fresh boot's first-ever Move; a chained/replaced Move's own
    // just-ended value otherwise -- SUC-051 continuity, no instant jump)
    // rather than the raw cruise target; shapeAndStage()'s own tick() calls
    // ramp it toward the cruise target from there. Shaping disabled: stage
    // the raw target immediately (UNCHANGED pre-shaping behavior) and keep
    // the shaper's own state mirror in sync (syncTo()) so a LATER
    // live-enabled shaping call doesn't inherit a stale value.
    float vLeft = active_.cruiseVLeft;
    float vRight = active_.cruiseVRight;
    if (linearShaping) {
      vLeft = shaperVLeft_.commandedSpeed();
      vRight = shaperVRight_.commandedSpeed();
    } else {
      shaperVLeft_.syncTo(vLeft);
      shaperVRight_.syncTo(vRight);
    }
    drive_.setWheels(vLeft, vRight);
  } else {
    // TWIST (and the defensive NONE fallback -- see move_queue.h's own
    // header: a well-formed Move never reaches here with velocity_kind ==
    // NONE, that shape check is RobotLoop::handleMove()'s job).
    active_.cruiseVX = move.velocity.twist.v_x;
    active_.cruiseVY = move.velocity.twist.v_y;
    active_.cruiseOmega = move.velocity.twist.omega;
    active_.cruiseVLeft = 0.0f;
    active_.cruiseVRight = 0.0f;

    float vx = active_.cruiseVX;
    float omega = active_.cruiseOmega;
    if (linearShaping) {
      vx = shaperVX_.commandedSpeed();
    } else {
      shaperVX_.syncTo(vx);
    }
    if (angularShaping) {
      omega = shaperOmega_.commandedSpeed();
    } else {
      shaperOmega_.syncTo(omega);
    }
    drive_.setTwist(vx, active_.cruiseVY, omega);
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

  lastShapeNow_ = now;  // dt baseline for this Move's own first shapeAndStage() call
}

// shapeAndStage -- see move_queue.h's own tick()/shapeAndStage() doc
// comments for the full contract. Per-velocity-kind axis selection:
//   WHEELS  -- v_left/v_right shaped INDEPENDENTLY, both on the LINEAR
//              axis (aMax/aDecel) regardless of the Move's own stop_kind
//              -- a wheel target has no "angular" component of its own to
//              shape; an Angle-kind WHEELS Move (a differential turn
//              commanded by raw wheel speeds rather than TWIST.omega) gets
//              accel-ramped but never decel-tapered on this axis (its own
//              "remaining" stays +infinity, matching the Kind::Time
//              posture) -- a known, documented scope limitation: WHEELS
//              moves are not the ticket's own primary target (90-degree
//              TWIST turns and TWIST distance stops are), and a
//              wheel-speed-only remaining-angle mapping is not this
//              module's concern to invent.
//   TWIST   -- v_x shaped on the LINEAR axis, omega shaped on the ANGULAR
//              axis, EACH independently gated by its own ShaperLimits
//              fields and each using ITS OWN kind-matched `remaining`
//              (Distance-kind -> remainingLinear real, remainingAngular
//              +infinity; Angle-kind -> the reverse; Time-kind -> both
//              +infinity). A Move whose stop_kind doesn't match a nonzero
//              component it's ALSO commanding (e.g. a Distance-kind TWIST
//              Move with a nonzero omega -- an arc, not a pure straight)
//              still gets that OTHER component accel-ramped (remaining
//              stays +infinity for it, exactly the Kind::Time posture) --
//              never decel-tapered, since this module has no basis to
//              measure "remaining" on an axis the Move's own stop
//              condition doesn't watch.
void MoveQueue::shapeAndStage(uint64_t now, float pathLength, float theta) {
  bool linearShaping =
      shaperLimits_.aMax > 0.0f && shaperLimits_.aDecel > 0.0f && shaperLimits_.jMax > 0.0f;
  bool angularShaping = shaperLimits_.alphaMax > 0.0f && shaperLimits_.alphaDecel > 0.0f &&
                        shaperLimits_.yawJerkMax > 0.0f;
  if (!linearShaping && !angularShaping) return;  // Drive already holds the raw cruise target

  float dt = static_cast<float>(now - lastShapeNow_) / 1.0e6f;  // [us] -> [s]
  if (dt < 0.0f) dt = 0.0f;  // clock-monotonicity defense, same posture as StopCondition's own
  lastShapeNow_ = now;

  const float kInfinity = std::numeric_limits<float>::infinity();
  float remainingLinear = kInfinity;
  float remainingAngular = kInfinity;
  if (active_.kind == Motion::StopCondition::Kind::Distance) {
    remainingLinear = active_.threshold - std::fabs(pathLength - active_.activationPathLength);
  } else if (active_.kind == Motion::StopCondition::Kind::Angle) {
    remainingAngular = active_.threshold - std::fabs(theta - active_.activationTheta);
  }
  // Kind::Time -- both axes stay +infinity: accel/jerk-limited ramp-up
  // still applies (Motion::VelocityShaper::next()'s own accel/jerk
  // clamps), no decel taper (a Time Move ends on elapsed wall-clock time,
  // not position -- move_queue.h's own tick() doc comment).

  if (active_.velocityKind == msg::Move::VelocityKind::WHEELS) {
    if (!linearShaping) return;
    float vLeft = shaperVLeft_.next(active_.cruiseVLeft, remainingLinear, dt, shaperLimits_.aMax,
                                     shaperLimits_.aDecel, shaperLimits_.jMax);
    float vRight = shaperVRight_.next(active_.cruiseVRight, remainingLinear, dt, shaperLimits_.aMax,
                                       shaperLimits_.aDecel, shaperLimits_.jMax);
    drive_.setWheels(vLeft, vRight);
    return;
  }

  // TWIST.
  float vx = active_.cruiseVX;
  float omega = active_.cruiseOmega;
  if (linearShaping) {
    vx = shaperVX_.next(active_.cruiseVX, remainingLinear, dt, shaperLimits_.aMax,
                         shaperLimits_.aDecel, shaperLimits_.jMax);
  }
  if (angularShaping) {
    omega = shaperOmega_.next(active_.cruiseOmega, remainingAngular, dt, shaperLimits_.alphaMax,
                               shaperLimits_.alphaDecel, shaperLimits_.yawJerkMax);
  }
  drive_.setTwist(vx, active_.cruiseVY, omega);
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

  // Anticipation lead (turn-prediction campaign) -- see tick()'s own doc
  // comment (move_queue.h) for the full rationale. Defaults to the raw
  // current reading; only overridden below when both stopLead_ > 0 and the
  // estimator's own body peer is warmed up.
  float pathLength = odom.pathLength();
  float theta = odom.theta();

  if (stopLead_ > 0 && (active_.kind == Motion::StopCondition::Kind::Angle ||
                        active_.kind == Motion::StopCondition::Kind::Distance)) {
    uint32_t nowMs = static_cast<uint32_t>(now / 1000);
    BodyEstimate predicted = stateEstimator_.bodyAt(nowMs + stopLead_);
    if (predicted.valid) {
      theta = predicted.heading;
      float age = static_cast<float>((nowMs + stopLead_) - predicted.basisTime) / 1000.0f;  // [s]
      float speed = std::sqrt(predicted.v_x * predicted.v_x + predicted.v_y * predicted.v_y);
      pathLength = odom.pathLength() + speed * age;
    }
  }

  Motion::StopCondition::Outcome outcome = sc.tick(now, pathLength, theta);
  if (outcome == Motion::StopCondition::Outcome::Continue) {
    // Velocity shaping (decel-into-the-goal campaign) -- reuses the SAME
    // (possibly anticipation-predicted) pathLength/theta just computed
    // above for the stop-condition comparison; see move_queue.h's own
    // tick()/shapeAndStage() doc comments. Only reached on Continue --
    // a Move ending THIS cycle is about to be superseded by a
    // chain-advance activate() or drive_.stop() below regardless, so
    // shaping it first would be immediately overwritten.
    shapeAndStage(now, pathLength, theta);
    return result;
  }

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
    // The robot has genuinely stopped -- reset() every shaper's own
    // (commandedSpeed, commandedAccel) state too, not just Drive's own
    // staged targets, so the NEXT unrelated Move (whenever it activates)
    // ramps from a true (0, 0) instead of inheriting a stale nonzero pair
    // from a taper that never finished (e.g. this Move ended via the
    // timeout backstop mid-taper). See move_queue.h's own shaper* member
    // doc comment.
    shaperVX_.reset();
    shaperOmega_.reset();
    shaperVLeft_.reset();
    shaperVRight_.reset();
  }

  return result;
}

void MoveQueue::flush() {
  pendingCount_ = 0;
  active_.occupied = false;
  active_.moveId = 0;
  drive_.stop();
  // Same reasoning as tick()'s own empty-queue-drain path above -- the
  // robot has genuinely stopped, so every shaper's own state must reset
  // to (0, 0) too, not just Drive's own staged targets.
  shaperVX_.reset();
  shaperOmega_.reset();
  shaperVLeft_.reset();
  shaperVRight_.reset();
}

}  // namespace App
