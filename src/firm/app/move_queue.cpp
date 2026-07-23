// move_queue.cpp -- App::MoveQueue implementation. See move_queue.h's file
// header for the module's boundary and storage rationale.
#include "app/move_queue.h"

#include <cmath>
#include <limits>

#include "motion/velocity_shaper.h"

namespace App {

namespace {

// Land-at-zero completion predicate (118 ticket 004, issue
// land-at-zero-completion-delete-stop-lead.md).
//
// The issue's own text suggested gating on a STATIC epsilon just above the
// output-deadband-equivalent floor (~15mm/s per wheel, nezha_motor.cpp's
// own writeShapedDuty() sub-deadband boost -- ~0.23 rad/s at the 128mm
// reference trackwidth), reasoning that a target below the floor "never
// converges" since NezhaMotor boosts a sub-floor nonzero duty back up.
// Verified against the actual code and empirically (sim tour-closure
// gate) this reasoning does not transfer to this predicate: the deadband
// boost lives several layers downstream, inside NezhaMotor's own final
// duty write -- it never clamps Motion::VelocityShaper's own
// commandedSpeed_, which is pure arithmetic and legitimately decays
// through and below that floor. A STATIC epsilon (either on commandedSpeed_
// alone, or on the `remaining` value the accel-only decel-ceiling formula
// implies at that speed) never actually binds before the raw
// threshold/timeout backstop does, for the SAME reason the deleted
// anticipation-lead constant itself needed repeated retuning: the
// jerk-limited ramp-down
// (velocity_shaper.cpp's own accel-slew clamp) trails the SAME decel
// ceiling the shaper's own `remaining` argument implies by an amount that
// depends on where in the taper's own curve the query lands, not a fixed
// offset a single static threshold can capture.
//
// What DOES work, verified against the sim tour-closure gate's own exact
// path (TOUR_1/TOUR_2 x ideal/realistic, the closure gate's own acceptance
// bands): a DYNAMIC, self-referential stopping-distance check --
// `remaining <= (commandedSpeed^2 / (2*decelCeiling)) * kStoppingMarginFactor`
// -- "have we already entered our own braking envelope for our CURRENT
// commanded speed." This is is the same closed-form `v^2/(2*a)` stopping-
// distance formula velocity_shaper.cpp's own decel-taper ceiling already
// uses, self-consistent by construction (it re-evaluates every tick
// against whatever commandedSpeed_ currently is, rather than a single
// fixed target), and structurally cannot misfire at Move activation
// (commandedSpeed_ starts at/near 0, making the RHS ~0, while `remaining`
// starts at the Move's own full threshold).
//
// The margin factor accounts for the ACTUAL post-Drive::stop() deceleration
// being measurably tighter than the smooth taper's own decel ceiling
// (Drive::stop() bypasses VelocityShaper's jerk/accel limits entirely and
// commands the motor's raw velocity-PID loop to zero directly) -- swept
// against sim ground truth, the SAME empirical-sweep methodology this
// project already uses for every Motion::VelocityShaper ceiling that has no
// simpler closed form (a_max/a_decel/alpha_max/alpha_decel/j_max/
// yaw_jerk_max, each robot JSON's own control._shaper_note archaeology).
//
// TWO values, not one -- chosen by whether a chain-advance is imminent
// (pendingCount() > 0) or the queue is about to drain to a genuine stop
// (pendingCount() == 0). This split exists because the two measurement
// conventions this project's own acceptance suites use for "did the turn
// land" disagree about what "coast" even means:
//   - test_tour_closure_gate.py's own per-turn accuracy check reads sim
//     ground truth at the completion-ack INSTANT (TurnCheck, this file's
//     own `_run_tour_capture()`), because a tour leg's own next Move is
//     already queued (SUC-003 one-leg lookahead) and starts driving the
//     SAME cycle -- there is no settle window between legs to coast into,
//     so this reading never sees whatever the real motor/PID does after
//     Drive::stop() would have run (chain-advance never calls it).
//   - test_gui_button_acceptance.py's own preset/SEG checks read pose after
//     genuine quiescence (`settle_pose()`, a real quiet-window poll) --
//     because each button press is its OWN Move with nothing queued behind
//     it, the robot actually reaches Drive::stop() and its real velocity-PID
//     coasts the remaining residual speed to zero, and settle_pose()
//     faithfully captures that coast as part of "where the robot ended up."
// Swept independently against each measurement convention on this file's
// own exact paths (TOUR_1/TOUR_2 x ideal/realistic for the chain case;
// isolated 90-degree twists x ideal/realistic for the final case):
//   - Chain-advance (pendingCount() > 0): a broad, flat plateau (0.82-0.84)
//     all measure worst=2.398deg against the tour-closure gate's own
//     2.5deg shaped band; 0.83 (mid-plateau) ships as the default,
//     mirroring this project's "pick mid-plateau, not a knife-edge point"
//     convention for every prior empirically-swept shaper constant. Firing
//     any LATER (a bigger factor) does not help this reading (it only
//     measures the decision instant, never the coast) and firing earlier
//     erodes the shaped-band margin.
//   - Final move (pendingCount() == 0): a broad, flat plateau (0.90-1.10)
//     measures worst=1.189deg settle-based against the button-acceptance
//     suite's own 3.0deg tolerance; 1.00 (mid-plateau) ships as the
//     default. Firing at the CHAIN-ADVANCE factor here (0.83) measures
//     4.997deg settle-based -- comfortably over that 3.0deg tolerance --
//     because it lets residual commanded speed stay high enough at the
//     decision instant that the REAL post-Drive::stop() coast (not
//     something this predicate's timing can shrink away once Drive::stop()
//     has already been called) adds several more degrees before the plant
//     actually comes to rest. Firing earlier (a bigger factor) declares
//     completion while more residual speed remains, so less of it survives
//     into the real coast.
constexpr float kStoppingMarginFactorChain = 0.83f;  // dimensionless
constexpr float kStoppingMarginFactorFinal = 1.00f;  // dimensionless

}  // namespace

MoveQueue::MoveQueue(Drive& drive, Odometry& odom, const Devices::Clock& clock,
                      ShaperLimits shaperLimits)
    : drive_(drive), odom_(odom), clock_(clock), shaperLimits_(shaperLimits) {}

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

// landAtZero -- see move_queue.h's own tick() doc comment for the full
// contract. TWIST moves only: a WHEELS Move's own linearly-shaped axes
// (v_left/v_right) have no stop_kind-matched pairing the way a TWIST
// Move's v_x/omega do (shapeAndStage()'s own per-kind breakdown above), so
// ticket 004's scope -- TWIST Angle/Distance stops only -- excludes WHEELS
// structurally, via the velocityKind check below, not via a second
// remaining/epsilon derivation for wheel-space axes.
bool MoveQueue::landAtZero(float pathLength, float theta) const {
  if (active_.velocityKind != msg::Move::VelocityKind::TWIST) return false;

  // See this file's own anonymous-namespace comment (kStoppingMarginFactorChain/
  // kStoppingMarginFactorFinal) for the full derivation of why this predicate
  // needs two different margins: whether THIS completion hands off to an
  // already-queued Move (pendingCount() > 0, chain-advance -- Drive::stop()
  // never runs, so only the ack-instant decision matters) or drains the
  // queue to a genuine stop (pendingCount() == 0 -- Drive::stop() runs for
  // real and the plant's own residual speed coasts further before rest).
  float marginFactor =
      pendingCount_ > 0 ? kStoppingMarginFactorChain : kStoppingMarginFactorFinal;

  if (active_.kind == Motion::StopCondition::Kind::Distance) {
    bool linearShaping =
        shaperLimits_.aMax > 0.0f && shaperLimits_.aDecel > 0.0f && shaperLimits_.jMax > 0.0f;
    if (!linearShaping) return false;  // no taper -- the backstop is the only completion path
    float remaining = active_.threshold - std::fabs(pathLength - active_.activationPathLength);
    // "Have we already entered our own braking envelope for our CURRENT
    // commanded speed."
    float cmd = shaperVX_.commandedSpeed();
    float epsilonRemaining = (cmd * cmd) / (2.0f * shaperLimits_.aDecel) * marginFactor;
    return remaining <= epsilonRemaining;
  }

  if (active_.kind == Motion::StopCondition::Kind::Angle) {
    bool angularShaping = shaperLimits_.alphaMax > 0.0f && shaperLimits_.alphaDecel > 0.0f &&
                          shaperLimits_.yawJerkMax > 0.0f;
    if (!angularShaping) return false;
    float remaining = active_.threshold - std::fabs(theta - active_.activationTheta);
    float cmd = shaperOmega_.commandedSpeed();
    float epsilonRemaining = (cmd * cmd) / (2.0f * shaperLimits_.alphaDecel) * marginFactor;
    return remaining <= epsilonRemaining;
  }

  return false;  // Kind::Time -- no spatial `remaining`, never qualifies.
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

  float pathLength = odom.pathLength();
  float theta = odom.theta();

  // Backstop (threshold/timeout) is always-armed and evaluated first --
  // "first to fire wins" (move_queue.h's own tick() doc comment). Only
  // when it does NOT already end the Move this cycle (Continue) is the
  // land-at-zero completion path (118 ticket 004) checked as an
  // ADDITIONAL way for the Move to end -- treated identically to
  // StopConditionMet (never TimedOut): the taper decided this Move is
  // done, not the timeout backstop.
  Motion::StopCondition::Outcome outcome = sc.tick(now, pathLength, theta);
  if (outcome == Motion::StopCondition::Outcome::Continue && landAtZero(pathLength, theta)) {
    outcome = Motion::StopCondition::Outcome::StopConditionMet;
  }

  if (outcome == Motion::StopCondition::Outcome::Continue) {
    // Velocity shaping (decel-into-the-goal campaign) -- reuses the SAME
    // pathLength/theta just computed above for the stop-condition
    // comparison; see move_queue.h's own tick()/shapeAndStage() doc
    // comments. Only reached on Continue -- a Move ending THIS cycle is
    // about to be superseded by a chain-advance activate() or
    // drive_.stop() below regardless, so shaping it first would be
    // immediately overwritten.
    shapeAndStage(now, pathLength, theta);
    return result;
  }

  result.completed = true;
  result.completion.moveId = active_.moveId;
  result.completion.timedOut = (outcome == Motion::StopCondition::Outcome::TimedOut);

  active_.occupied = false;

  // Reset the axis this Move's own stop_kind was tapering, on EVERY
  // completion (backstop OR land-at-zero), not just the empty-queue drain
  // below. Rationale (118 ticket 004, discovered empirically against the
  // sim tour-closure gate): the shaper* members are deliberately
  // MoveQueue-level, not ActiveMove-level, so a same-axis chained Move
  // continues its ramp smoothly (SUC-051 continuity, move_queue.h's own
  // shaper* doc comment) -- but that same continuity means a Move that
  // ends with a NONZERO residual commandedSpeed_ (any Move can, whether it
  // ended via the exact-threshold backstop or the land-at-zero predicate
  // above, both of which tolerate the taper not having fully reached zero)
  // leaks that residual into the chain-advanced Move's own activation
  // baseline, and from there into landAtZero()'s own `cmd` read for
  // WHATEVER Move next uses this same axis -- corrupting that LATER Move's
  // completion decision with a value that describes the PREVIOUS Move, not
  // its own taper. Resetting here, unconditionally, cuts that leak at the
  // source; SUC-051 continuity is preserved for the case it actually
  // matters (a Move's OWN shaping while it runs), just not across a
  // completion boundary.
  if (active_.kind == Motion::StopCondition::Kind::Angle) {
    shaperOmega_.reset();
  } else if (active_.kind == Motion::StopCondition::Kind::Distance) {
    shaperVX_.reset();
  }

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
