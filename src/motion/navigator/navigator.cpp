// navigator.cpp -- Motion::Navigator::tick()/start()/cancel() (135-003).
// See navigator.h for the full contract, including the ownership-of-
// planner_.tick() rationale and the OTOS sign-convention citation.
#include "navigator.h"

#include <algorithm>
#include <cmath>

namespace Motion {

namespace {

struct BodyOffset {
  float distance = 0.0f;  // [mm]
  float bearing = 0.0f;   // [rad]
};

// Body-frame distance/bearing from `pose` to `(targetX, targetY)`.
//
// Duplicates arc_solver.cpp's own internal world-to-body transform rather
// than reusing ArcSolution::bearing, because ArcSolution only ever
// populates `bearing` with the real value in the target-behind (stop ==
// true) branch -- arc_solver.h's own ArcSolution doc comment: "0.0 for...
// an ordinary (non-stop) solution". Navigator's own turnFirstAngle policy
// test (SUC-004) needs the real bearing in the ORDINARY case too: a
// bearing between turnFirstAngle (~50 deg) and behindAngle (~90 deg) is
// exactly the case ArcSolver::solve() returns a normal, non-stop solution
// for. Kept minimal and pure (no state), matching arc_solver.cpp's own
// style, so the duplication stays trivially auditable against it.
BodyOffset bodyOffset(const Pose& pose, float targetX, float targetY) {
  const float dx = targetX - pose.x;
  const float dy = targetY - pose.y;
  const float cosH = std::cos(pose.heading);
  const float sinH = std::sin(pose.heading);
  const float bodyX = dx * cosH + dy * sinH;
  const float bodyY = -dx * sinH + dy * cosH;
  return {std::hypot(bodyX, bodyY), std::atan2(bodyY, bodyX)};
}

}  // namespace

void Navigator::start(const GotoTarget& target) {
  target_ = target;
  if (target_.tolerance <= 0.0f) target_.tolerance = limits_.defaultArrivalTolerance;

  phase_ = Phase::Active;
  pivotPhase_ = PivotPhase::None;
  justStarted_ = true;
  startCycleStart_ = 0;

  haveObservedOtosSample_ = false;
  lastOtosSampleTime_ = 0;
  disconnected_ = false;
  disconnectStartCycle_ = 0;
  haveGoodPose_ = false;
  lastGoodPose_ = Pose{};
  lastGoodWheelPose_ = Pose{};

  previousOmega_ = 0.0f;
  hasIssued_ = false;
  lastIssuedOmega_ = 0.0f;
  lastIssuedArcLength_ = 0.0f;
  issuedAtPose_ = Pose{};

  frozen_ = false;
}

void Navigator::cancel() {
  phase_ = Phase::Idle;
  pivotPhase_ = PivotPhase::None;
  // No Planner call here -- SUC-003: a preempted goto emits no completion
  // ack, and the caller is already about to issue its own MOVE/WHEELS/
  // estop() through the normal path, which supersedes whatever this
  // Navigator last issued via replace=true.
}

NavResult Navigator::tick(Types::RobotState& state) {
  NavResult result{};
  if (phase_ == Phase::Idle) return result;

  if (justStarted_) {
    startCycleStart_ = state.time.cycleStart;
    justStarted_ = false;
  }
  ++tickCount_;

  // Execute whatever was staged last cycle (see this file's header
  // comment for why this "tick, then decide" ordering is the causally
  // correct one), THEN decide what to stage for the Move that will
  // activate NEXT cycle.
  const TickResult moveResult = planner_.tick(state);
  planner_.update(state);

  // Overall goto timeout -- independent of OTOS/pose entirely.
  if (target_.timeout > 0 &&
      (state.time.cycleStart - startCycleStart_) >= target_.timeout) {
    return abortResult();
  }

  // --- Pose source: OTOS when connected, bounded dead-reckoning from
  // state.pose deltas on disconnect (SUC-005). ---
  Pose pose{};
  bool allowSolve = true;
  bool haveSolvablePose = true;

  if (state.otos.connected) {
    pose.x = state.otos.x;
    pose.y = state.otos.y;
    // The ONE sign flip this file performs -- see navigator.h's header
    // comment ("OTOS sign convention") for the full citation. state.otos.
    // heading is the wire/hardware-mounted sign (ticket 008); negating it
    // mirrors planner.cpp:513's own reconciliation, landing in the same
    // encoder-sign convention Move::omega/ArcSolution::omega already use.
    pose.heading = -state.otos.heading;

    lastGoodPose_ = pose;
    lastGoodWheelPose_ = Pose{state.pose.x, state.pose.y, state.pose.heading};
    haveGoodPose_ = true;
    disconnected_ = false;

    const bool fresh = !haveObservedOtosSample_ ||
                        state.otos.sampleTime != lastOtosSampleTime_;
    haveObservedOtosSample_ = true;
    lastOtosSampleTime_ = state.otos.sampleTime;
    allowSolve = fresh;  // stale-but-connected: hold, never blind-replace
  } else {
    if (!disconnected_) {
      disconnected_ = true;
      disconnectStartCycle_ = state.time.cycleStart;
    }
    if ((state.time.cycleStart - disconnectStartCycle_) >= kNavOtosDisconnectAbortWindow) {
      return abortResult();
    }
    if (!haveGoodPose_) {
      // Never had a fix at all since start() -- nothing to dead-reckon
      // from yet; wait inside the bounded disconnect window.
      haveSolvablePose = false;
    } else {
      pose.x = lastGoodPose_.x + (state.pose.x - lastGoodWheelPose_.x);
      pose.y = lastGoodPose_.y + (state.pose.y - lastGoodWheelPose_.y);
      pose.heading = lastGoodPose_.heading + (state.pose.heading - lastGoodWheelPose_.heading);
    }
    // allowSolve stays true: keep converging (approximately) through the
    // bounded window, per SUC-005 step 2.
  }

  if (!haveSolvablePose) return result;

  // True-world-frame pose for BEARING/ARC-SOLVE geometry (135-004,
  // "Landmine 4" -- see NavigatorLimits::yawSign's own comment in
  // navigator.h). `pose` above is deliberately kept in encoder-sign
  // convention (it just served the SUC-005 dead-reckoning bookkeeping);
  // `limits_.yawSign` (default +1.0) is a no-op here, leaving worldPose ==
  // pose exactly -- ticket 003's original, unchanged behavior for every
  // ctest/sim scenario. A robot configured with yawSign=-1.0 (measured,
  // per-robot) instead recovers the raw OTOS/world value ArcSolver's
  // tangent-arc geometry and the pivot bearing need. Valid in BOTH
  // branches above (OTOS-connected and dead-reckoning), since
  // `pose.heading` is uniformly encoder-sign convention in either case.
  const Pose worldPose{pose.x, pose.y, limits_.yawSign * pose.heading};

  // --- Arrival (independent of allowSolve -- a stale-but-connected or
  // dead-reckoned pose is still the best truth available this cycle).
  // Rotation-invariant distance -- either pose gives the same answer;
  // worldPose used here only for one consistent convention. ---
  const BodyOffset toTarget = bodyOffset(worldPose, target_.x, target_.y);
  if (!frozen_ && toTarget.distance <= target_.tolerance) {
    frozen_ = true;
  }
  if (frozen_) {
    // Require the Move to have actually SETTLED (TickResult::settled --
    // "within the arrival epsilon AND at rest"), not merely `completed`:
    // Planner's own completion test also fires on overshoot
    // ("plannedRemaining <= epsilon on a SIGNED residual... a move that
    // OVERSHOOTS completes at once", planner.cpp's own comment) which can
    // land `completed == true` while the wheels are still commanded well
    // above rest. Falling back to `lifecycle() == Idle` catches that case
    // a cycle or two later, once the natural post-completion drain
    // (still serviced every cycle regardless of `frozen_` -- planner_.
    // tick()/update() run unconditionally above) has actually reached
    // zero.
    if ((moveResult.completed && moveResult.settled) ||
        planner_.lifecycle() == MoveLifecycle::Idle) {
      return doneResult();
    }
    return result;
  }

  // --- Pivot sub-machine progression (SUC-004). These transitions react
  // to what the Planner just did, independent of allowSolve. ---
  if (pivotPhase_ == PivotPhase::StoppingForPivot) {
    if (moveResult.completed) {
      issuePivotMove(worldPose);
      pivotPhase_ = PivotPhase::Pivoting;
    }
    return result;
  }
  if (pivotPhase_ == PivotPhase::Pivoting) {
    // 135-004 "Landmine 2": do NOT wait for moveResult.completed alone --
    // planner.cpp's own terminal fine-align (MoveLifecycle::Aligning) holds
    // a LANDED Twist Angle Move open for up to ~2 s of low-speed trim
    // nudges before moveResult.completed ever fires (planner_types.h's own
    // MoveLifecycle::Aligning doc comment: "a Twist Angle Move whose
    // profile has LANDED"). Aligning beginning IS the "landed" signal this
    // sub-machine needs -- replacing the pivot with the outbound cruise
    // arc now, one cycle later (not necessarily Aligning-settled), is the
    // documented design intent (sprint.md Architecture): the NEXT cycle's
    // replace=true issue flushes Aligning along with everything else it
    // already flushes (planner.cpp's own move()/replace semantics), the
    // same way it flushes any other in-flight Move.
    if (moveResult.completed || planner_.lifecycle() == MoveLifecycle::Aligning) {
      pivotPhase_ = PivotPhase::None;
      hasIssued_ = false;  // force an unconditional re-issue once cruising resumes
      previousOmega_ = 0.0f;
    }
    return result;  // fall through to cruising on the NEXT cycle
  }

  if (!allowSolve) return result;  // stale-but-connected: hold this cycle

  // --- Normal cruise: pivot-first check, then solve + material-change
  // throttle. ---
  if (std::fabs(toTarget.bearing) >= limits_.turnFirstAngle) {
    beginPivotSequence(worldPose);
    return result;
  }

  // Per-goto cruise-speed override (135-004, wire parity with
  // envelope.proto's GoTo.speed) -- <= 0 falls open to the config
  // default, matching every other <=0 bound in this codebase. A local
  // copy, not a limits_ mutation: limits_ is `const NavigatorLimits&`
  // (possibly shared/live-pushed) and this override is scoped to ONE
  // goto, not a standing config change.
  NavigatorLimits effectiveLimits = limits_;
  if (target_.speed > 0.0f) effectiveLimits.speed = target_.speed;

  // solve() is fed worldPose (true-world convention) and returns omega in
  // that SAME convention -- previousOmega_/lastIssuedOmega_ stay in this
  // native convention throughout (see NavigatorLimits::yawSign's own
  // comment in navigator.h); only the
  // Move::omega assigned below gets converted to the wire convention.
  const ArcSolution solution = ArcSolver::solve(
      worldPose, Pose{target_.x, target_.y, 0.0f}, effectiveLimits, previousOmega_);
  previousOmega_ = solution.omega;

  if (solution.stop) {
    // Reachable only if behindAngle <= turnFirstAngle in a pathological
    // config -- the ordinary target-behind case is already caught by the
    // turnFirstAngle check above (behindAngle > turnFirstAngle in every
    // sane configuration). Handled the same way regardless, for safety.
    beginPivotSequence(worldPose);
    return result;
  }

  bool mustIssue = !hasIssued_;
  if (!mustIssue &&
      std::fabs(solution.omega - lastIssuedOmega_) > kNavOmegaReplaceThreshold) {
    mustIssue = true;
  }
  if (!mustIssue &&
      std::fabs(solution.arcLength - lastIssuedArcLength_) > kNavArcLengthReplaceThreshold) {
    mustIssue = true;
  }
  if (!mustIssue) {
    // Mandatory half-arc refresh: re-send once the robot has covered this
    // fraction of the arc length it was last COMMANDED, regardless of
    // whether the solution itself has moved materially -- ported from
    // pathplan.planner.py's own `covered = hypot(currentPose - sentPose)`
    // vs. `sentDistance * throttle.refreshFraction` (planner.py:1239-1242).
    const float covered = std::hypot(pose.x - issuedAtPose_.x, pose.y - issuedAtPose_.y);
    if (covered >= lastIssuedArcLength_ * kNavRefreshFraction) mustIssue = true;
  }

  if (mustIssue) {
    Move m;
    m.id = 0;  // internal segment -- ticket 004 never acks this individually
    m.kind = Move::Kind::Distance;
    m.velocityKind = Move::VelocityKind::Twist;
    m.threshold = solution.arcLength;
    m.v_x = solution.v_x;
    // yawSign: solution.omega is in worldPose's own convention (default
    // +1.0 -- a no-op, matching ticket 003's original, unchanged
    // behavior) -- this is the ONE point it converts to the wire's
    // Move::omega convention. See NavigatorLimits::yawSign's own comment.
    m.omega = limits_.yawSign * solution.omega;
    m.timeout = segmentTimeout(solution.arcLength, effectiveLimits.speed);
    planner_.move(m, true);

    lastIssuedOmega_ = solution.omega;
    lastIssuedArcLength_ = solution.arcLength;
    issuedAtPose_ = pose;
    hasIssued_ = true;
    ++replaceCount_;
  }

  return result;
}

void Navigator::beginPivotSequence(const Pose& worldPose) {
  // SUC-004: never replace an in-flight arc with a pivot at speed (that
  // would ratio-lock hard-brake the reversing wheel) -- ramp to rest via
  // the ordinary planned-stop sequencing first, then pivot from rest.
  if (planner_.active()) {
    planner_.plannedStop(0);
    pivotPhase_ = PivotPhase::StoppingForPivot;
  } else {
    issuePivotMove(worldPose);
    pivotPhase_ = PivotPhase::Pivoting;
  }
}

void Navigator::issuePivotMove(const Pose& worldPose) {
  // Recompute fresh -- time has passed (at minimum, the stop's own decel
  // ramp) since the bearing that triggered this sequence was measured.
  // `worldPose` (true-world convention, NOT tick()'s encoder-sign `pose`)
  // -- see NavigatorLimits::yawSign's own comment in navigator.h.
  //
  // 135-004 "Landmine 3": this Move is issued straight into planner_.
  // move() below, never through App::RobotLoop::handleMove() -- so it
  // never sees that function's own per-direction rotation-calibration
  // correction (robot_loop.cpp, "the per-direction rotation gain/offset
  // is an OPEN-LOOP correction"). Correct on purpose, not a gap: that
  // correction only ever matters with OTOS ABSENT (it pre-distorts an
  // open-loop, scrub-limited wheel estimate; with OTOS present the loop
  // already closes on optical truth and pre-distorting would make it
  // stop at the wrong place), and a Navigator pivot only ever happens
  // with OTOS CONNECTED (App::RobotLoop::handleGoto()'s own acceptance
  // gate) -- exactly the condition under which the correction would
  // already be a no-op if it were somehow reached. Do not add it here.
  const BodyOffset toTarget = bodyOffset(worldPose, target_.x, target_.y);
  const float bearing = toTarget.bearing;

  Move m;
  m.id = 0;  // internal segment
  m.kind = Move::Kind::Angle;
  m.velocityKind = Move::VelocityKind::Twist;
  m.threshold = std::fabs(bearing);
  m.v_x = 0.0f;
  // yawSign: `bearing` is in worldPose's own convention -- see the
  // cruise arc's own identical conversion above and NavigatorLimits::
  // yawSign's own comment in navigator.h.
  m.omega = limits_.yawSign * std::copysign(pivotOmega(), bearing);
  m.timeout = pivotTimeout(std::fabs(bearing));
  planner_.move(m, true);

  previousOmega_ = 0.0f;  // cruise resumes from rest; no carried curvature bias
}

float Navigator::pivotOmega() const {
  // [rad/s] both wheels at the configured cruise linear speed, driven
  // differentially -- omega = 2*speed/trackWidth, the same relationship
  // arc_solver.cpp's own clampOmegaStep() uses in the other direction (an
  // omega-step budget from a wheel-speed budget). Derived from
  // NavigatorLimits fields already read elsewhere, rather than adding a
  // new field for a pivot-specific cruise omega. Planner's own profiler
  // clamps this to PlannerLimits::Ceilings::omegaMax regardless of what is
  // requested here, exactly as it does for any other Move's requested
  // cruise.
  if (limits_.trackWidth <= 0.0f) return 1.0f;  // fail-open fallback
  return 2.0f * limits_.speed / limits_.trackWidth;
}

float Navigator::segmentTimeout(float arcLength, float speed) const {
  // Ported from pathplan.planner._moveTimeoutFor() -- see navigator.h's
  // own comment on kNavSegmentTimeoutMultiplier/Floor/Cap.
  const float safeSpeed = std::max(speed, 1.0f);
  const float idealDuration = std::fabs(arcLength) / safeSpeed * 1000.0f;  // [ms]
  return std::min(idealDuration * kNavSegmentTimeoutMultiplier + kNavSegmentTimeoutFloor,
                   kNavSegmentTimeoutCap);
}

float Navigator::pivotTimeout(float bearingMagnitude) const {
  const float omega = std::max(pivotOmega(), 0.01f);
  const float idealDuration = bearingMagnitude / omega * 1000.0f;  // [ms]
  return std::min(idealDuration * kNavSegmentTimeoutMultiplier + kNavSegmentTimeoutFloor,
                   kNavSegmentTimeoutCap);
}

NavResult Navigator::doneResult() {
  NavResult result;
  result.completed = true;
  result.id = target_.id;
  result.fault = false;
  phase_ = Phase::Idle;
  pivotPhase_ = PivotPhase::None;
  return result;
}

NavResult Navigator::abortResult() {
  planner_.estop();  // "zero the Move" -- the ticket's own wording
  NavResult result;
  result.completed = true;
  result.id = target_.id;
  result.fault = true;
  phase_ = Phase::Idle;
  pivotPhase_ = PivotPhase::None;
  return result;
}

}  // namespace Motion
