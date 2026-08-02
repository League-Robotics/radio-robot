#include "planner.h"

#include <algorithm>
#include <cmath>

#include "profile.h"

namespace Motion {

namespace {

// Completion epsilons -- PHYSICAL arrival tolerances, sized to what the
// plant can actually measure and reach.
//
// These were 1e-3 mm / 1e-5 rad, documented as "float-measurement noise
// floors, NOT motion-margin constants (the profile's terminal step lands
// exactly)". That premise holds for a sim plant and fails on hardware:
// completion tests `plannedRemaining <= epsilon` on a SIGNED residual, so
// a move that OVERSHOOTS completes instantly while one that UNDERSHOOTS by
// more than the epsilon never completes at all. Real wheels stop short --
// the profile's final decel step falls below App::Drive's dead-zone
// intercept, correctedCommand() returns exactly 0, and the wheel parks a
// fraction of a mm out. With the wheels stopped the in-flight prediction
// that would otherwise carry plannedRemaining negative is also 0, so the
// residual is pinned and the Move hangs to its MOVE_TIMEOUT backstop
// (measured 2026-07-28: a 500mm leg parked 0.4mm short -- 400x the old
// 1e-3 epsilon -- and sat 13s past arrival; other runs hit the full 30s).
// The settle creep that exists to close exactly this gap cannot help: it
// is gated on active_.settling, which only ever gets set when
// PlannerLimits::requireSettle is on, and that defaults false and is not
// baked from robot config.
//
// RESTORED to the original float-noise floors after the stall backstop
// below made them the wrong tool. Widening these to 1.0 mm / 0.003 rad did
// stop the hangs, but at a cost that only showed up later: `done` fires on
// the tick the residual first drops under the epsilon, BEFORE the profile's
// exact terminal step executes, so a wide epsilon truncates the last
// epsilon of EVERY move. That is a systematic short-fall on every leg and
// turn (planner_scenarios_test caught it: a 400 mm move landing 0.9 mm
// short against a 0.001 mm tolerance), and it still could not survive a
// plant that lands further short than whatever value was chosen.
//
// The stall backstop handles the stuck case directly and without a
// tolerance to outgrow, so these go back to being what their original
// comment said they were: noise floors absorbing last-ulp rounding in the
// re-measurement, not motion margin.
constexpr float kDoneEpsilonLinear = 1e-3f;   // [mm]
constexpr float kDoneEpsilonAngular = 1e-5f;  // [rad]

// Stall backstop (Planner::ActiveMove::stallTicks documents the failure it
// defends against). "Has stopped and is going nowhere" needs three things:
//
//  - AT REST. Reuses PlannerLimits::settleRestVelocity/settleRestOmega, the
//    same thresholds settleReached() already calls "stopped".
//  - NOT MOVING THE RESIDUAL. Tested on `anchoredRemaining`, which only
//    changes when a real encoder sample lands (planner.h) -- the noise-free
//    signal, deliberately not the predicted one. The tolerances are a few
//    measurement quanta: the encoder quantum is 0.0716 mm, and a pivot's
//    heading quantum is 2*0.0716/trackWidth ~= 0.0011 rad.
//  - FOR LONG ENOUGH that a slow crawl is not mistaken for a stall. At
//    <5 mm/s, 0.5 s is under 2.5 mm of travel.
//
// 0.5 s replaces a 30 s MOVE_TIMEOUT, and unlike an epsilon it cannot be
// re-broken by a weaker plant landing further short.
constexpr float kStallWindow = 0.5f;            // [s]
constexpr float kStallEpsilonLinear = 0.25f;    // [mm] ~3.5 encoder quanta
constexpr float kStallEpsilonAngular = 0.004f;  // [rad] ~3.6 heading quanta

// Settle-confirm gates (PlannerLimits::requireSettle). Unlike the done
// epsilons above these ARE physical tolerances -- "close enough to the
// target, and stopped" for a real, lagging plant.
// (Settle-confirm arrival tolerances moved to PlannerLimits::
// settleEpsilonLinear/settleEpsilonAngular -- reachability depends on the
// robot's stiction-limited minimum creep step, a per-robot property.)
// Rest floors -- the settle gate's ONLY velocity criterion. (An earlier
// revision widened the gate to max(floor, one decel step), reasoning a
// commanded plant one step from zero is at rest next interval -- but a
// REAL plant with time constant tau COASTS ~v*tau past the target after
// the command reaches zero: at alphaDecel*dt = 0.25 rad/s that admitted
// +0.9 deg of post-settle coast per turn. The floors are sized so the
// worst coast is within the arrival epsilons.)
// (Rest floors moved to PlannerLimits::settleRestVelocity/settleRestOmega
// -- a per-robot noise property, not a universal constant.)

float sign(float value) { return value < 0.0f ? -1.0f : 1.0f; }

// Below this a wheel's unit share is "not commanded" and it constrains
// nothing -- a twist whose omega exactly cancels one wheel (v_x ==
// omega*trackWidth/2) is a legitimate one-wheel arc, not a fault.
constexpr float kUnitEpsilon = 1e-6f;  // [1]

// Relative slack for the ratio lock's tie-break. The two wheels' allowed
// speeds are algebraically identical on a tracking plant; anything inside
// this band is float rounding, not a real divergence, and the DOMINANT
// wheel's (exact) answer wins.
constexpr float kRatioTie = 1e-6f;  // [1]

// Below this a wheel's own last commanded velocity is indistinguishable
// from "not moving" -- planWheels() treats it as directionally compatible
// with anything (no meaningful direction to conflict with) rather than
// testing its sign.
constexpr float kMinWheelSpeed = 1e-3f;  // [mm/s]

// One wheel's share of the shape-space ceilings. profileStep() is
// homogeneous of degree 1, so scaling every limit by the wheel's unit
// magnitude is exactly equivalent to profiling in shape space and scaling
// the answer -- which is what makes the ratio lock's tie a real tie.
AxisLimits scaleLimits(const AxisLimits& limits, float scale) {
  AxisLimits out;
  out.vMax = limits.vMax * scale;
  out.aMax = limits.aMax * scale;
  out.aDecel = limits.aDecel * scale;
  out.jMax = limits.jMax * scale;
  out.aDecelPlan = limits.aDecelPlan * scale;
  return out;
}

// Signed, clamped age between two robot-clock stamps [ms] -> [s]. The
// loop stamps cycleStart at the TOP of the cycle but collects encoder
// samples several ms LATER in the schedule, so a fresh sample's time is
// legitimately AHEAD of `now` -- unsigned subtraction would wrap to a
// ~50-day age and corrupt every prediction (surfaced the moment the sim
// mirrored the real schedule, 2026-07-26).
float ageSeconds(uint32_t now, uint32_t basis) {
  return std::max(
      0.0f, static_cast<float>(static_cast<int32_t>(now - basis)) * 0.001f);
}

// Signed one-axis ramp for Time/Wheels Moves: hold toward `cruise`, and
// once the remaining ticks are just enough to reach `boundary` at the
// decel ceiling, step toward it.
float timedRamp(float previous, float cruise, float boundary,
                float accelStep, float decelStep, float ticksLeft) {
  const float toBoundary = std::fabs(previous - boundary);
  const float stepsNeeded =
      decelStep > 0.0f ? std::ceil(toBoundary / decelStep) : 0.0f;
  // STRICT comparison: at ticksLeft == stepsNeeded the ramp holds cruise
  // one more tick and still lands within the Move (an elapsed-time Move's
  // landing is the clock, not a distance) -- anticipating one tick early
  // staged a zero-command frame BEFORE the completion ack could ride out,
  // which chain observers correctly read as a hand-off dip.
  if (ticksLeft < stepsNeeded) {
    const float step = std::min(decelStep, toBoundary);
    return previous + (boundary > previous ? step : -step);
  }
  if (cruise > previous) return std::min(cruise, previous + accelStep);
  return std::max(cruise, previous - std::max(accelStep, decelStep));
}

// Direction of the profiled axis for measurement/carry accounting. A
// Twist Move carries it on the commanded twist's sign; a Wheels Move has
// v_x == omega == 0, so its direction lives structurally on the pair --
// the mean for the linear axis, the differential for the angular axis.
// (Without this, a Wheels Move with a Distance/Angle stop measured a
// frozen remaining == threshold and could only ever end on its timeout.)
float linearDirection(const Move& m) {
  if (m.velocityKind == Move::VelocityKind::Wheels) {
    return sign(0.5f * (m.vLeft + m.vRight));
  }
  return sign(m.v_x);
}

float angularDirection(const Move& m) {
  if (m.velocityKind == Move::VelocityKind::Wheels) {
    return sign(m.vRight - m.vLeft);
  }
  return sign(m.omega);
}

}  // namespace

Planner::Planner(const PlannerLimits& limits) : limits_(limits) {
  left_.configure(limits_.velocityFilterWeight);
  right_.configure(limits_.velocityFilterWeight);
  pose_.configure(limits_.trackWidth);
}

void Planner::applyShaperLimits(float aMax, float aDecel, float alphaMax,
                                float alphaDecel, float jerkMax,
                                float yawJerkMax) {
  limits_.aMax = aMax;
  limits_.aDecel = aDecel;
  limits_.alphaMax = alphaMax;
  limits_.alphaDecel = alphaDecel;
  limits_.jerkMax = jerkMax;
  limits_.yawJerkMax = yawJerkMax;
  shaperConfigured_ = true;
}

bool Planner::move(const Move& next, bool replace) {
  // Shape validation: direction comes from the velocity sign, so the
  // profiled axis must actually be commanded; Wheels Moves are Time-bounded
  // only at v1 (sketch §3).
  //
  // v_y is carried but not actuated on a differential drivetrain. A Move
  // whose ONLY commanded linear velocity is sideways therefore asks for a
  // motion this robot cannot make, and is rejected here rather than
  // accepted and silently driven as nothing -- the Distance/Angle guards
  // below test v_x/omega, which a pure-v_y Move leaves at zero.
  //
  // A Kind::Stop entry commands nothing and measures nothing, so none of
  // the shape guards apply to it -- it is valid by construction (see
  // plannedStop() below, the only thing that builds one).
  const bool valid =
      next.kind == Move::Kind::Stop ||
      (next.threshold >= 0.0f &&
       ((next.velocityKind == Move::VelocityKind::Twist &&
         (next.kind != Move::Kind::Distance || next.v_x != 0.0f) &&
         (next.kind != Move::Kind::Angle || next.omega != 0.0f)) ||
        (next.velocityKind == Move::VelocityKind::Wheels)));
  if (!valid) return false;

  if (replace) {
    pendingCount_ = 0;
    active_.occupied = false;  // the replacement activates next tick()
  }
  const int total = (active_.occupied ? 1 : 0) + pendingCount_;
  if (total >= kQueueDepth) return false;
  pending_[pendingCount_++] = next;
  return true;
}

bool Planner::plannedStop(uint32_t moveId) {
  Move entry;
  entry.id = moveId;
  entry.kind = Move::Kind::Stop;
  // No velocity, no threshold, no timeout: a planned stop's own bounded
  // backstop is plannedStopWindow(), not the shared timeout path -- landing
  // slowly is not a fault, and the timeout path raises one.
  return move(entry, /*replace=*/false);
}

float Planner::plannedStopWindow() const {
  const float drain = limits_.aDecel > 0.0f
                          ? 1000.0f * limits_.vMax / limits_.aDecel  // [ms]
                          : 0.0f;
  // settleWindow when the robot has one configured (it is the same
  // "how long may arrival take" allowance), else a fixed floor so an
  // unconfigured planner still converges.
  constexpr float kDefaultSettleAllowance = 500.0f;  // [ms]
  const float settle =
      limits_.settleWindow > 0.0f ? limits_.settleWindow : kDefaultSettleAllowance;
  return drain + settle;
}

void Planner::estop() {
  pendingCount_ = 0;
  active_.occupied = false;
  profileVelocity_ = 0.0f;
  profileAccel_ = 0.0f;
  cmdLeft_ = 0.0f;
  cmdRight_ = 0.0f;
  // The history too: after a stop there is no travel left to anticipate.
  cmdLeftPrevious_ = 0.0f;
  cmdRightPrevious_ = 0.0f;
}

// Age the staged command by one tick. Called from the two places that
// overwrite cmdLeft_/cmdRight_ -- exactly one of which runs per tick --
// and never before measure(), which reads both generations.
void Planner::rollCommandHistory() {
  cmdLeftPrevious_ = cmdLeft_;
  cmdRightPrevious_ = cmdRight_;
}

TickResult Planner::tick(const Types::RobotState& state) {
  TickResult result{};
  const uint32_t now = state.time.cycleStart;
  const float dt = limits_.controlPeriod * 0.001f;  // [s]
  ticked_ = true;

  // SENSE -> ESTIMATE (sketch §4): filter on fresh samples only, integrate
  // the pose from the measured anchors, blend OTOS heading when fresh. The
  // ZOH predict-to-now happens in measure(), not here -- see below.
  left_.ingest(state.wheelLeft.position, state.wheelLeft.velocity,
               state.wheelLeft.sampleTime);
  right_.ingest(state.wheelRight.position, state.wheelRight.velocity,
                state.wheelRight.sampleTime);
  // Integrate the pose from the MEASURED anchors, never from the ZOH
  // extrapolation. pathLength accumulates |ds|, so any zero-mean jitter in
  // its input is rectified into a one-way drift: an extrapolated input
  // would make a standing robot's odometer keep climbing at roughly
  // |velocity noise| * period per tick. Anchors only move when a real
  // sample lands, so a standing robot's path length is exactly flat. The
  // extrapolation is still applied -- as an additive lookahead term in
  // measure(), where it informs the plan without accumulating (sketch §4:
  // "traveled distance is ALWAYS anchored to measured positions").
  pose_.integrate(left_.basisPosition(), right_.basisPosition());
  if (state.otos.present && limits_.headingOtosWeight > 0.0f &&
      static_cast<int32_t>(now - state.otos.sampleTime) <=
          static_cast<int32_t>(limits_.otosStaleness)) {
    pose_.blendHeading(state.otos.heading, limits_.headingOtosWeight);
  }

  if (!active_.occupied) activateNext(now);
  if (!active_.occupied) {
    drainToZero(dt);
    return result;
  }

  // DECIDE: completion first (against last tick's plan), then plan the
  // (possibly newly activated) Move's next interval.
  const Move& m = active_.move;
  const uint32_t elapsed = now - active_.activationTime;  // [ms]
  const Measurement measured = measure(now);

  const bool timedOut = m.timeout > 0.0f &&
                        static_cast<float>(elapsed) >= m.timeout;

  // Stall backstop. Only for the two kinds whose completion is a POSITION
  // the plant has to reach and that are meant to land at rest -- a Time
  // Move's stop condition is the clock, and a Move handing off at speed
  // (activeBoundary_ > 0) is not supposed to stop at all, so neither can
  // legitimately stall. Evaluated before `done` so a stalled Move completes
  // on the same tick it is recognised.
  const bool stallApplies =
      (m.kind == Move::Kind::Distance || m.kind == Move::Kind::Angle) &&
      activeBoundary_ <= 0.0f;
  bool stalled = false;
  if (stallApplies) {
    const bool angular = (m.kind == Move::Kind::Angle);
    const bool atRest =
        angular ? std::fabs(measured.omega) <= limits_.settleRestOmega
                : std::fabs(measured.bodyVelocity) <= limits_.settleRestVelocity;
    const float epsilon =
        angular ? kStallEpsilonAngular : kStallEpsilonLinear;
    // Progress is measured against where the residual stood when the window
    // opened, NOT against the previous tick: a residual creeping by less
    // than epsilon per tick would otherwise reset the counter forever while
    // going nowhere.
    if (!atRest) {
      active_.hasMoved = true;
      active_.stallTicks = 0;  // moving: not stalled, wherever it is
    } else if (!active_.hasMoved) {
      // Still breaking away from the activation tick -- not a stall.
      active_.stallTicks = 0;
    } else if (active_.stallTicks > 0 &&
               std::fabs(measured.anchoredRemaining -
                         active_.stallRemaining) <= epsilon) {
      ++active_.stallTicks;
    } else {
      active_.stallTicks = 1;
      active_.stallRemaining = measured.anchoredRemaining;
    }
    stalled = dt > 0.0f && static_cast<float>(active_.stallTicks) * dt >=
                               kStallWindow;
  }

  bool done = timedOut || stalled;
  // ARRIVED: a Distance/Angle Move that is inside the robot's own
  // configured arrival tolerance (settleEpsilonLinear/Angular) AND at rest
  // is finished. The kDoneEpsilon* tests below are float NOISE FLOORS, not
  // reachability tolerances -- a real wheel parks a fraction of a mm short
  // because the profile's terminal step falls under the motor
  // write-suppression deadband, and with the wheels stopped the in-flight
  // prediction that would carry the residual negative is also zero, so the
  // residual is PINNED and the Move waits out the whole kStallWindow
  // (measured on the robot 2026-07-30: ~0.4-0.5 s of dead time per
  // boundary, concentrated on turns; measured in the planner harness: the
  // residual pinned at 0.8764 mm for 11 consecutive ticks == 0.517 s ==
  // kStallWindow). settleReached() is already computed here and already
  // reported as TickResult::settled; this lets it END the Move rather than
  // only describe it.
  //
  // Cannot fire early: it requires the body to be AT REST, so it is
  // unreachable at cruise. Suppressed for a Move handing off at speed
  // (activeBoundary_ > 0), which is not supposed to stop at all, and gated
  // on hasMoved so a Move cannot complete on its own activation tick before
  // breaking away. Residual left on the target is absorbed by the next
  // chained Move through the cumulative baseline ledger -- exactly the
  // mechanism src/firm/main.cpp already relies on with requireSettle off.
  const bool arrived =
      activeBoundary_ <= 0.0f && active_.hasMoved &&
      m.velocityKind == Move::VelocityKind::Twist &&
      (m.kind == Move::Kind::Distance || m.kind == Move::Kind::Angle) &&
      settleReached(measured, dt);
  if (!done && arrived) {
    done = true;
  } else if (!done && active_.settling) {
    done = true;  // profile-complete already fired; only the gate is pending
  } else if (!done) {
    switch (m.kind) {
      case Move::Kind::Time:
        // Complete on the tick whose interval CONTAINS the expiry -- the
        // same tick the ramp's taper lands at the boundary. When the
        // threshold falls mid-interval, waiting for elapsed >= threshold
        // adds a whole extra tick commanding zero while still active: one
        // more pre-ack zero-target cycle than the loop schedule's single
        // ack-visibility lag, which chain observers correctly read as a
        // hand-off gap. Exact-multiple thresholds still complete exactly
        // at the threshold, and every Move gets at least one planned tick.
        done = elapsed > 0 && static_cast<float>(elapsed) +
                                      limits_.controlPeriod >
                                  m.threshold;
        break;
      case Move::Kind::Distance:
        // Carry boundary (>0): hand off the tick the crossing falls inside,
        // debiting the sub-tick residual to the next Move's cumulative
        // baseline. Final/orthogonal boundary (0): the terminal step has
        // already closed the sum -- complete at (float-noise) zero.
        done = measured.plannedRemaining <= kDoneEpsilonLinear ||
               (activeBoundary_ > 0.0f &&
                measured.plannedRemaining <=
                    profileVelocity_ * dt + kDoneEpsilonLinear);
        break;
      case Move::Kind::Angle:
        done = measured.plannedRemaining <= kDoneEpsilonAngular ||
               (activeBoundary_ > 0.0f &&
                measured.plannedRemaining <=
                    profileVelocity_ * dt + kDoneEpsilonAngular);
        break;
      case Move::Kind::Stop:
        // A planned stop ends when the command has drained to zero AND the
        // body has actually come to rest -- "completes at rest" is the
        // whole point of asking for a stop rather than just letting the
        // queue run dry. plannedStopWindow() is the bounded backstop so a
        // noisy plant cannot wedge the queue here (see that method).
        done = (cmdLeft_ == 0.0f && cmdRight_ == 0.0f &&
                settleReached(measured, dt)) ||
               static_cast<float>(elapsed) >= plannedStopWindow();
        break;
    }
  }

  bool settled = false;
  if (done) {
    settled = settleReached(measured, dt);
    // Settle-confirm: hold the completion back until the body has actually
    // arrived and stopped. Only for a Distance/Angle Move landing at rest
    // -- a Time Move's stop condition IS the clock, and a Move handing off
    // at speed into a same-axis successor (activeBoundary_ > 0) is not
    // supposed to come to rest at all. A timeout aborts the motion and is
    // never deferred.
    const bool settleApplies =
        limits_.requireSettle && !timedOut && activeBoundary_ <= 0.0f &&
        (m.kind == Move::Kind::Distance || m.kind == Move::Kind::Angle) &&
        m.velocityKind == Move::VelocityKind::Twist;
    if (settleApplies && !settled) {
      if (!active_.settling) {
        active_.settling = true;
        active_.settleStart = now;
      }
      const float waited = static_cast<float>(now - active_.settleStart);
      // Past the window: complete anyway, reporting settled = false. This
      // is NOT a timeout -- the Move's own stop condition was met, only
      // the physical confirmation of it was not.
      done = waited >= limits_.settleWindow;
    }
  }

  if (done) {
    result.completed = true;
    result.moveId = m.id;
    result.timedOut = timedOut;
    result.settled = settled;
    // Cumulative-baseline carry (chain-exact accounting): a normally
    // completed Twist Distance/Angle Move hands the next Move BOTH
    // cumulative baselines -- "where the boundary IS" on its own axis
    // (baseline + threshold) and its own UNCHANGED baseline on the other
    // axis (a straight leg intends zero heading change; a pivot intends
    // zero path change). Carrying the full ledger across KINDS is what
    // closes a mixed leg/turn tour: each turn targets the cumulative
    // n*90deg and each leg's heading-hold pulls back to the carried
    // square heading, so per-landing residuals cancel instead of
    // accumulating (measured on the bench tour: +17 deg over 8 moves
    // with same-kind-only carry, every leg/turn boundary re-anchoring
    // to wherever the pose drifted). A timeout aborts the motion, and a
    // Time/Wheels Move has no single-axis intent -- no carry, next
    // baselines re-anchor to the pose.
    //
    // A Kind::Stop entry is the one exception: it declares no axis intent
    // at all (it neither travels a planned distance nor turns a planned
    // angle), so it leaves the ledger exactly as its predecessor left it
    // rather than re-anchoring. "leg, planned stop, leg" then accounts
    // identically to "leg, leg" -- the stop is a pause in the sequence,
    // not a new origin.
    if (m.kind != Move::Kind::Stop) {
      carryValid_ = !timedOut &&
                    m.velocityKind == Move::VelocityKind::Twist &&
                    (m.kind == Move::Kind::Distance ||
                     m.kind == Move::Kind::Angle);
      if (m.kind == Move::Kind::Distance) {
        carryPath_ = active_.baselinePath + linearDirection(m) * m.threshold;
        carryHeading_ = active_.baselineHeading;
      } else {
        carryPath_ = active_.baselinePath;
        carryHeading_ =
            active_.baselineHeading + angularDirection(m) * m.threshold;
      }
    }
    active_.occupied = false;
    activateNext(now);
    if (!active_.occupied) {
      drainToZero(dt);
      return result;
    }
  }

  // Re-measure: a same-tick hand-off above swapped the active Move, and
  // `remaining` is measured against ITS baseline and axis.
  planActive(now, dt, done ? measure(now) : measured);
  return result;
}

void Planner::activateNext(uint32_t now) {
  carryValid_ = carryValid_ && pendingCount_ > 0;  // carry consumed below or dropped
  if (pendingCount_ == 0) {
    return;
  }
  const Move next = pending_[0];
  for (int i = 1; i < pendingCount_; ++i) pending_[i - 1] = pending_[i];
  --pendingCount_;

  active_.occupied = true;
  active_.move = next;
  active_.activationTime = now;
  active_.closingIssued = false;
  active_.settling = false;
  active_.settleStart = now;
  active_.stallRemaining = 0.0f;
  active_.stallTicks = 0;
  active_.hasMoved = false;
  active_.baselinePath =
      0.5f * (left_.basisPosition() + right_.basisPosition());
  active_.baselineHeading = pose_.heading();
  // A Kind::Stop entry passes the cumulative ledger THROUGH untouched --
  // see the completion-side comment in tick(). It has no baselines of its
  // own to adopt, and clearing the carry here would make an intervening
  // planned stop silently re-anchor the tour that follows it.
  if (next.kind != Move::Kind::Stop) {
    if (carryValid_ && next.velocityKind == Move::VelocityKind::Twist &&
        (next.kind == Move::Kind::Distance || next.kind == Move::Kind::Angle)) {
      // Full-ledger adoption (see the completion-side comment): both axes'
      // cumulative baselines, regardless of the predecessor's kind.
      active_.baselinePath = carryPath_;
      active_.baselineHeading = carryHeading_;
    }
    carryValid_ = false;
  }

  // Per-wheel plan for this Move: the constant left:right ratio, each
  // wheel's own signed distance target, and the shape-space ceilings.
  // Captured ONCE here rather than recomputed per tick -- the shape is a
  // property of the Move, and re-deriving it every tick would let a
  // divide-by-a-measured-quantity wander.
  active_.shape = shapeOf(next, limits_.trackWidth);
  active_.wheelLimits = shapeLimits(active_.shape, limits_);
  if (active_.shape.valid && active_.wheelLimits.aDecel > 0.0f) {
    lastShapeDecel_ = active_.wheelLimits.aDecel;
  }
  active_.phase = MovePhase::Idle;
  active_.decelLatched = false;
  // axisPerLambda: how much of the Move's OWN axis quantity advances per
  // unit lambda. Distance profiles the body path, whose speed is the mean
  // wheel speed; Angle profiles the heading, whose rate is the wheel
  // difference over the track. Anything else never reaches planWheels().
  active_.axisPerLambda = 1.0f;
  if (active_.shape.valid) {
    if (next.kind == Move::Kind::Distance) {
      active_.axisPerLambda =
          std::fabs(0.5f * (active_.shape.unitLeft + active_.shape.unitRight));
    } else if (next.kind == Move::Kind::Angle) {
      active_.axisPerLambda =
          std::fabs(active_.shape.unitRight - active_.shape.unitLeft) /
          limits_.trackWidth;
    }
  }
  if (!(active_.axisPerLambda > 0.0f)) active_.axisPerLambda = 1.0f;

  // Same-axis carry keeps the profile's ramp continuity; an axis change
  // starts the new axis's profile from rest (we landed at ~0 there).
  const Axis axis = axisOf(next);
  if (axis != lastAxis_) {
    profileVelocity_ = 0.0f;
    profileAccel_ = 0.0f;
  }
  lastAxis_ = axis;
  activeBoundary_ = 0.0f;
}

Planner::Measurement Planner::measure(uint32_t now) const {
  Measurement out;
  out.bodyVelocity = 0.5f * (left_.velocity() + right_.velocity());  // [mm/s]
  out.omega =
      (right_.velocity() - left_.velocity()) / limits_.trackWidth;  // [rad/s]
  if (!active_.occupied) return out;

  // Anticipation: how far the body still travels between the last measured
  // anchor and the instant this tick's command takes effect. Two adjacent
  // spans, each attributed to the command that actually drives it:
  //
  //   [anchorTime, now]  -- already elapsed, unobserved (the sample is
  //       older than the loop). With one cycle of staging latency the
  //       command in force over it was staged the tick BEFORE last.
  //   [now, now + delay] -- not yet elapsed; the command staged last tick
  //       is still in force until this tick's replaces it.
  //
  // Both spans use the COMMANDED velocity, not the measured one. Under the
  // velocity-tracking plant the whole profiler is built on they agree --
  // but the command is exact where the encoder's derivative is very noisy
  // (sketch §4), and this term is differenced for the angular axis, where
  // per-wheel noise does not cancel the way it does in the mean. If the
  // plant fails to track, the error is bounded by one sample interval and
  // fully corrected by the next anchor; it never accumulates, because the
  // pose itself is anchored to measured positions.
  const float delay = limits_.actuationDelay * 0.001f;  // [s]
  const float ageLeft = ageSeconds(now, left_.basisTime());    // [s]
  const float ageRight = ageSeconds(now, right_.basisTime());  // [s]
  const float elapsedLeft = delay > 0.0f ? cmdLeftPrevious_ : cmdLeft_;
  const float elapsedRight = delay > 0.0f ? cmdRightPrevious_ : cmdRight_;
  const float predictLeft =
      elapsedLeft * ageLeft + cmdLeft_ * delay;  // [mm]
  const float predictRight =
      elapsedRight * ageRight + cmdRight_ * delay;  // [mm]
  const float predictPath = 0.5f * (predictLeft + predictRight);       // [mm]
  const float predictHeading =
      (predictRight - predictLeft) / limits_.trackWidth;               // [rad]

  const Move& m = active_.move;
  switch (m.kind) {
    case Move::Kind::Time:
      break;  // a Time Move's residual is the clock, not a distance
    case Move::Kind::Stop:
      break;  // a planned stop's residual is the body's own speed, below
    case Move::Kind::Distance: {
      // Traveled distance is the SIGNED mean-wheel displacement along the
      // Move's direction -- a telescoping measure (final minus baseline
      // anchor positions), immune to noise/quantum rectification. The
      // former pathLength() measure accumulated |ds| per cycle, so the
      // encoder quantum's flicker during a slow settle-creep INFLATED it
      // ~0.07 mm per jitter cycle and completed Moves measurably short.
      const float dir = linearDirection(m);
      const float meanPosition =
          0.5f * (left_.basisPosition() + right_.basisPosition());
      out.anchoredRemaining =
          m.threshold - dir * (meanPosition - active_.baselinePath);
      out.plannedRemaining = out.anchoredRemaining - dir * predictPath;
      break;
    }
    case Move::Kind::Angle: {
      const float dir = angularDirection(m);
      out.anchoredRemaining =
          m.threshold - (pose_.heading() - active_.baselineHeading) * dir;
      out.plannedRemaining = out.anchoredRemaining - dir * predictHeading;
      break;
    }
  }
  return out;
}

bool Planner::settleReached(const Measurement& measured, float dt) const {
  if (!active_.occupied) return false;


  const Move& m = active_.move;
  // A planned stop is tested BEFORE the Twist gate: it has no velocity
  // variant at all, and "did the body stop?" is the whole of its
  // completion test. Both axes must be quiet -- a stop that left the robot
  // spinning is not a stop.
  if (m.kind == Move::Kind::Stop) {
    return std::fabs(measured.bodyVelocity) <= limits_.settleRestVelocity &&
           std::fabs(measured.omega) <= limits_.settleRestOmega;
  }
  if (m.velocityKind != Move::VelocityKind::Twist) return false;
  switch (m.kind) {
    case Move::Kind::Stop:
      return false;  // unreachable: handled above
    case Move::Kind::Time:
      return false;  // nothing physical to confirm
    case Move::Kind::Distance:
      return std::fabs(measured.anchoredRemaining) <=
                 limits_.settleEpsilonLinear &&
             std::fabs(measured.bodyVelocity) <= limits_.settleRestVelocity;
    case Move::Kind::Angle:
      return std::fabs(measured.anchoredRemaining) <=
                 limits_.settleEpsilonAngular &&
             std::fabs(measured.omega) <= limits_.settleRestOmega;
  }
  return false;
}

Planner::Axis Planner::axisOf(const Move& m) {
  // A planned stop profiles no axis -- Axis::None also makes whatever
  // follows it start its profile from rest, which is exactly right: we
  // just came to a stop.
  if (m.kind == Move::Kind::Stop) return Axis::None;
  if (m.velocityKind == Move::VelocityKind::Wheels) return Axis::Wheels;
  if (m.kind == Move::Kind::Angle) return Axis::Angular;
  return Axis::Linear;
}

// Lookahead: the SHAPE-SPACE speed (lambda -- the dominant wheel's own
// speed) to land the active Move at, so a chain flows at speed instead of
// stopping between legs.
//
// Two tiers of compatibility, not one:
//
//   EXACT (shapesCompatible()) -- the two Moves command the identical
//   left:right ratio. Free pass: no landing-lambda cap beyond distance
//   feasibility, because nothing about the wheels' commands changes across
//   the hand-off. Still the ONLY tier available to a Wheels-velocityKind
//   Move (direct per-wheel commands are not profiled the way Distance/
//   Angle Moves are) and to a Distance<->Angle axis change (axisOf()) --
//   both land at rest exactly as before.
//
//   RELAXED (shapeDirectionsAgree() + curvatureHandoffLambdaCap(), same
//   axis, neither Move a Wheels Move) -- the ratios may differ, so long as
//   neither wheel's rotation direction reverses, and the landing lambda is
//   capped so the per-wheel differential the ratio change itself demands
//   fits inside a bounded number of control cycles at that wheel's own
//   accel authority. This is what lets a chain of continuously-varying-
//   curvature segments (a rounded rectangle, a figure-eight, a spline) flow
//   through a boundary instead of decelerating to rest at every one, which
//   the exact-ratio-only test forced (every curvature change is a ratio
//   change). See planWheels() for how the incoming Move consumes the
//   landing state this produces.
//
// It is automatically right about a planned stop, whose shape is invalid
// by construction -- nothing ever hands off at speed into a stop.
float Planner::boundaryLambda(float dt) const {
  if (pendingCount_ == 0) return 0.0f;
  // Fail closed: never plan a hand-off we have no configured authority to
  // brake out of.
  if (limits_.aDecel <= 0.0f) return 0.0f;

  const Move& next = pending_[0];
  const MoveShape nextShape = shapeOf(next, limits_.trackWidth);
  if (!nextShape.valid || !active_.shape.valid) return 0.0f;

  const bool exactRatio = shapesCompatible(active_.shape, nextShape);
  if (!exactRatio) {
    // A Distance<->Angle hand-off, or anything touching a Wheels-
    // velocityKind Move, is not a curvature change to relax at all -- it
    // needs rest, same as always.
    if (axisOf(active_.move) != axisOf(next)) return 0.0f;
    if (active_.move.velocityKind == Move::VelocityKind::Wheels ||
        next.velocityKind == Move::VelocityKind::Wheels) {
      return 0.0f;
    }
    if (!shapeDirectionsAgree(active_.shape, nextShape)) return 0.0f;
  }

  // Neither Move's own cruise may be exceeded at the hand-off.
  float cap = std::min(active_.shape.cruise, nextShape.cruise);

  // And the successor must still be able to land: enter no faster than it
  // can brake to rest within its own distance. A Time successor has no
  // distance to land within, so its cruise is the only cap -- tighter, and
  // more honest, than the blanket vMax the per-Kind version used.
  if (nextShape.hasDistanceTarget) {
    const float dominantDistance = std::max(std::fabs(nextShape.distanceLeft),
                                            std::fabs(nextShape.distanceRight));
    cap = std::min(cap, maxEntryVelocity(dominantDistance, 0.0f,
                                         shapeLimits(nextShape, limits_), dt));
  }

  if (!exactRatio) {
    // Bound the landing speed so the per-wheel differential the curvature
    // change demands -- |unit_w(next) - unit_w(active)| * lambda -- is
    // absorbable within a handful of control cycles at this wheel's own
    // accel ceiling, rather than handed to planWheels() as a step. A few
    // cycles (not one): the goal is a brief, visible-but-minor slow-into-
    // the-curve, not a hair-trigger cap that forces near-rest on any
    // curvature change at all -- and planWheels() ramps whatever residual
    // gap remains over the following ticks regardless, same as any other
    // acceleration. See curvatureHandoffLambdaCap()'s own doc comment for
    // the derivation and clasi/issues/replace-rescales-carried-profile-
    // velocity-by-new-shape.md for what an UNBOUNDED shape change at speed
    // does today (measured: 433 mm/s of misinterpreted per-wheel command).
    constexpr int kHandoffBlendCycles = 8;
    const float wheelDecelCeiling = axisOf(active_.move) == Axis::Angular
        ? limits_.alphaDecel * 0.5f * limits_.trackWidth
        : limits_.aDecel;
    cap = std::min(cap, curvatureHandoffLambdaCap(active_.shape, nextShape,
                                                  wheelDecelCeiling, dt,
                                                  kHandoffBlendCycles));
  }

  return std::max(0.0f, cap);
}

void Planner::planActive(uint32_t now, float dt, const Measurement& measured) {
  rollCommandHistory();
  const Move& m = active_.move;
  const float elapsed = static_cast<float>(now - active_.activationTime);
  const float period = limits_.controlPeriod;  // [ms]

  // M1 terminal-settle: once the profile has closed its sum, the final
  // approach is a CLOSED-LOOP, BIDIRECTIONAL creep on the MEASURED
  // residual -- the profile itself is positive-frame and legally lands
  // with up to one decel step of residual speed, which a lagging plant
  // coasts PAST the target (measured: +3.3 deg past a turn, from where a
  // forward-only profile can never return). The creep is a plain P law
  // with tight caps: it walks the residual to the arrival epsilon from
  // EITHER side, decelerating as it converges, so the rest gate and the
  // arrival gate become satisfiable together.
  if (active_.settling &&
      (m.kind == Move::Kind::Distance || m.kind == Move::Kind::Angle)) {
    // Gain sized for convergence well inside a ~2 s window from the
    // worst hand-off residual; safe from hunting now that the accel
    // feedforward and filter-lag compensation are disabled during
    // settling (they were the churn source, not the gain).
    constexpr float kCreepGain = 2.5f;         // [1/s]
    constexpr float kCreepMaxLinear = 35.0f;   // [mm/s]
    constexpr float kCreepMaxAngular = 0.35f;  // [rad/s]
    if (m.kind == Move::Kind::Distance) {
      const float dir = sign(m.v_x);
      const float previous = 0.5f * (cmdLeftPrevious_ + cmdRightPrevious_);
      float v = dir * std::clamp(kCreepGain * measured.anchoredRemaining,
                                 -kCreepMaxLinear, kCreepMaxLinear);
      // The creep obeys the axis accel/decel limits like any other
      // commanded motion -- a P law is not a license for command steps.
      v = std::clamp(v, previous - limits_.aDecel * dt,
                     previous + limits_.aMax * dt);
      profileVelocity_ = std::fabs(v);
      profileAccel_ = 0.0f;
      cmdLeft_ = v;
      cmdRight_ = v;
      applyHeadingHold();
    } else {
      const float dir = sign(m.omega);
      const float previousOmega =
          (cmdRightPrevious_ - cmdLeftPrevious_) / limits_.trackWidth;
      float omega =
          dir * std::clamp(kCreepGain * measured.anchoredRemaining,
                           -kCreepMaxAngular, kCreepMaxAngular);
      omega = std::clamp(omega, previousOmega - limits_.alphaDecel * dt,
                         previousOmega + limits_.alphaMax * dt);
      profileVelocity_ = std::fabs(omega);
      profileAccel_ = 0.0f;
      const float halfTrack = 0.5f * limits_.trackWidth;
      cmdLeft_ = -omega * halfTrack;
      cmdRight_ = omega * halfTrack;
    }
    return;
  }

  if (m.velocityKind == Move::VelocityKind::Wheels) {
    // Per-wheel ramp toward the commanded pair. Time-bounded Moves taper
    // to rest on the clock; Distance/Angle-bounded Wheels Moves (the wire
    // protocol's other arms) ramp and HOLD -- their completion is the
    // standard measured-threshold test in tick(), and the post-completion
    // drain ramps down (the pre-Planner-integration semantics carried
    // forward: wheels Moves are direct wheel commands, not profiled
    // landings).
    const float ticksLeft =
        m.kind == Move::Kind::Time ? (m.threshold - elapsed) / period
                                   : 1.0e9f;
    const float accelStep = limits_.aMax * dt;
    const float decelStep = limits_.aDecel * dt;
    const float vCap = limits_.vMax;  // [mm/s] wheel-space ceiling
    cmdLeft_ = timedRamp(cmdLeft_, std::clamp(m.vLeft, -vCap, vCap), 0.0f,
                         accelStep, decelStep, ticksLeft);
    cmdRight_ = timedRamp(cmdRight_, std::clamp(m.vRight, -vCap, vCap), 0.0f,
                          accelStep, decelStep, ticksLeft);
    activeBoundary_ = 0.0f;
    return;
  }

  switch (m.kind) {
    case Move::Kind::Stop: {
      // Planned stop: per-wheel ramp to zero at the decel ceiling -- the
      // same law drainToZero() applies to an empty queue, run here as a
      // real queue entry so its arrival is sequenced, observable, and
      // acked. A zero/absent decel ceiling snaps straight to zero rather
      // than never converging (an unconfigured shaper must still stop).
      const float decelStep = limits_.aDecel * dt;
      rampCommandsToZero(decelStep);
      profileVelocity_ = std::max(0.0f, profileVelocity_ - decelStep);
      profileAccel_ = 0.0f;
      activeBoundary_ = 0.0f;
      active_.phase = MovePhase::Decel;  // a stop is all decel, by definition
      active_.decelLatched = true;
      break;
    }
    case Move::Kind::Distance:
    case Move::Kind::Angle: {
      planWheels(dt, measured);
      if (m.kind == Move::Kind::Distance) applyHeadingHold();
      break;
    }
    case Move::Kind::Time: {
      // Both twist axes ramp toward cruise; the linear axis may carry into
      // the next Move, the angular axis lands at zero.
      const float ticksLeft = (m.threshold - elapsed) / period;
      // The lookahead answers in shape space (the dominant wheel's own
      // speed); this branch works in BODY linear units, so scale by how
      // much body translation one unit of lambda produces -- which is the
      // mean of the shape's units. A Time Move with no net translation
      // (a timed pivot) has no linear carry to make.
      const float shapeMean =
          active_.shape.valid
              ? std::fabs(0.5f * (active_.shape.unitLeft +
                                  active_.shape.unitRight))
              : 0.0f;
      activeBoundary_ = boundaryLambda(dt) * shapeMean;
      const float vPrev = 0.5f * (cmdLeft_ + cmdRight_);
      const float omegaPrev = (cmdRight_ - cmdLeft_) / limits_.trackWidth;
      const float v = timedRamp(vPrev,
                                std::clamp(m.v_x, -limits_.vMax, limits_.vMax),
                                sign(m.v_x) * activeBoundary_,
                                limits_.aMax * dt, limits_.aDecel * dt,
                                ticksLeft);
      const float omega =
          timedRamp(omegaPrev, m.omega, 0.0f, limits_.alphaMax * dt,
                    limits_.alphaDecel * dt, ticksLeft);
      profileVelocity_ = std::fabs(v);
      const float halfTrack = 0.5f * limits_.trackWidth;
      cmdLeft_ = v - omega * halfTrack;
      cmdRight_ = v + omega * halfTrack;
      break;
    }
  }
}

// The per-wheel profiler: one distance ledger per wheel, both profiled
// against their own remaining, then ratio-locked back onto one command.
//
// SHAPE SPACE. The Move's constant left:right ratio (shape.h) lets the
// whole plan be one scalar -- lambda, the DOMINANT wheel's own speed --
// with wheel w commanded lambda * unit_w. Everything outside this function
// (the completion tests, activeBoundary_, profileVelocity_, the settle
// creep) stays in the Move's own axis units; active_.axisPerLambda is the
// single bridge, applied on the way in and undone on the way out. That is
// what makes this a drop-in for the two per-Kind cases it replaces: for a
// straight axisPerLambda is 1, and for a pivot it is 2/trackWidth, so
// profileVelocity_ comes back out as the body speed / yaw rate it always
// was.
//
// WHY PER WHEEL AT ALL, when a constant ratio means both wheels' remaining
// distances are proportional? Because on a REAL plant they stop being
// proportional -- one wheel slips, stalls below its breakaway duty, or
// simply runs a few percent slow -- and the wheel that is BEHIND is the
// one whose braking feasibility actually binds. Profiling the body axis
// alone cannot see that; it lands the mean and lets the difference become
// heading error.
//
// THE RATIO LOCK. Both wheels are profiled independently, then the SAME
// feasible fraction is applied to both: lambda is the min over wheels of
// (that wheel's allowed speed / its unit). Scaling both wheels by one
// number leaves the commanded ratio -- and therefore the commanded turn
// radius, and therefore the heading the Move sweeps -- exactly intact. The
// alternative (letting each wheel run its own profile independently) lets
// the ratio drift mid-move, which IS heading error by construction.
//
// WHY THIS PRESERVES THE DISCRETE-EXACT LANDING. profileStep() is
// homogeneous of degree 1 in (remaining, previous, cruise, boundary, aMax,
// aDecel, jMax): every term in profile.cpp is a sum or comparison of
// velocity- or distance-dimensioned quantities, with no absolute constant
// but kTiny. On a plant that tracks, remaining_w = |unit_w| * R for a
// common R, so wheel w's arguments are exactly |unit_w| times the
// shape-space arguments and its answer is exactly |unit_w| * Lambda for
// one common Lambda. The two wheels therefore AGREE identically, min() is
// a tie, and the exact terminal step (remaining/dt) survives untouched.
// When they disagree -- only possible because the plant diverged -- the
// behind wheel binds, which is the conservative and correct direction.
void Planner::planWheels(float dt, const Measurement& measured) {
  const MoveShape& shape = active_.shape;
  if (!shape.valid || active_.axisPerLambda <= 0.0f) {
    // Unshapeable Move (rejected at move() time, so this is belt-and-
    // braces): command nothing rather than something arbitrary.
    cmdLeft_ = 0.0f;
    cmdRight_ = 0.0f;
    active_.phase = MovePhase::Idle;
    return;
  }

  const float unit[2] = {shape.unitLeft, shape.unitRight};
  const float magnitude[2] = {std::fabs(unit[0]), std::fabs(unit[1])};

  // Axis units -> shape space. plannedRemaining is already positive-frame.
  const float remainingLambda =
      std::max(0.0f, measured.plannedRemaining) / active_.axisPerLambda;
  const float previousAccelLambda = profileAccel_ / active_.axisPerLambda;

  const float boundary = boundaryLambda(dt);         // [mm/s] shape space
  activeBoundary_ = boundary * active_.axisPerLambda;  // [axis units]

  // Per-wheel `previous`: THIS WHEEL'S OWN last commanded velocity
  // (cmdLeftPrevious_/cmdRightPrevious_), not profileVelocity_ rescaled by
  // axisPerLambda. The rescale is exact only when this Move's ratio
  // matches the predecessor's (axisPerLambda unchanged, the old
  // shapesCompatible()-only world); across a curvature-changing hand-off
  // it is the measured defect in clasi/issues/replace-rescales-carried-
  // profile-velocity-by-new-shape.md -- 433 mm/s of misinterpreted
  // per-wheel command from dividing a carried BODY-frame speed by the
  // NEW Move's (possibly very different) axisPerLambda. Reading the
  // wheel's own last command instead sidesteps that division entirely,
  // is bit-identical to the old formula in the steady-state (no hand-off)
  // case -- both reduce to lambda_prev * magnitude[w] -- and is exactly
  // right across ANY hand-off, chained or replace(): it is simply what
  // this wheel was actually last asked to do.
  //
  // profileStep()'s `previous` is a POSITIVE-frame magnitude: valid only
  // when the wheel's actual last direction already agrees with THIS
  // Move's own unit sign for it. A wheel that must reverse direction has
  // no such value -- profileStep() has no notion of "still coasting the
  // wrong way," it would misread a negative previous as already past
  // zero and about to overshoot. boundaryLambda() already refuses an
  // at-speed hand-off across a direction reversal
  // (shapeDirectionsAgree()), so a same-direction mismatch here can only
  // come from replace() (which bypasses that gate) -- treated the same
  // as an axis change always was: start this wheel fresh from rest
  // rather than trust a value the profiler cannot interpret. That is a
  // strictly safer fallback than today's rescale, which can command a
  // wheel PAST its own ceiling in the wrong direction; starting from
  // rest never can.
  const float cmdPreviousWheel[2] = {cmdLeftPrevious_, cmdRightPrevious_};
  float previousWheel[2] = {0.0f, 0.0f};
  for (int w = 0; w < 2; ++w) {
    const bool sameDirection = std::fabs(cmdPreviousWheel[w]) <= kMinWheelSpeed ||
                               cmdPreviousWheel[w] * unit[w] >= 0.0f;
    previousWheel[w] = sameDirection ? std::fabs(cmdPreviousWheel[w]) : 0.0f;
  }

  // Profile each wheel against ITS OWN remaining distance, cruise, and
  // ceilings -- all just this wheel's share of the shape.
  float allowed[2] = {0.0f, 0.0f};
  bool closing[2] = {false, false};
  StepPhase phase[2] = {StepPhase::Accel, StepPhase::Accel};
  bool constrains[2] = {false, false};
  for (int w = 0; w < 2; ++w) {
    if (magnitude[w] <= kUnitEpsilon) continue;  // this wheel is commanded still
    constrains[w] = true;
    const AxisLimits lim = scaleLimits(active_.wheelLimits, magnitude[w]);
    const ProfileResult step =
        profileStep(remainingLambda * magnitude[w],
                    previousWheel[w], shape.cruise * magnitude[w],
                    boundary * magnitude[w], lim, dt,
                    previousAccelLambda * magnitude[w]);
    allowed[w] = step.velocity / magnitude[w];  // back to shape space
    closing[w] = step.closing;
    phase[w] = step.phase;
  }

  // THE RATIO LOCK, with the tie broken toward the DOMINANT wheel. The
  // dominant wheel's |unit| is a literal 1.0f (shape.cpp snaps it), so its
  // arithmetic is exact and only the sub-dominant wheel eats rounding.
  // Without this tie-break a 1-ulp difference lets the sub-dominant wheel
  // steal the closing step and the landing quietly loses its exactness.
  const int dominant = magnitude[0] >= magnitude[1] ? 0 : 1;
  const int other = 1 - dominant;
  float lambda = 0.0f;
  int binding = dominant;
  if (constrains[dominant] && constrains[other]) {
    lambda = allowed[dominant];
    if (allowed[other] < allowed[dominant] * (1.0f - kRatioTie)) {
      lambda = allowed[other];
      binding = other;
    }
  } else if (constrains[dominant]) {
    lambda = allowed[dominant];
  } else if (constrains[other]) {
    lambda = allowed[other];
    binding = other;
  }

  // Hard per-wheel ACCEL ceiling, independent of the tie-break above.
  // allowed[w] is each wheel's own accel-respecting optimum IN ISOLATION;
  // adopting the more-constrained wheel's answer for BOTH wheels is only
  // guaranteed safe for the non-binding wheel when its `previousWheel[]`
  // was already proportional to THIS Move's own ratio -- true on every
  // tick but the first after a curvature-changing hand-off, where it
  // still carries the PREDECESSOR Move's ratio (see previousWheel[]'s own
  // comment above). On that one tick the two wheels' individually-optimal
  // answers, reinterpreted through THIS Move's (different) per-wheel
  // magnitude, can disagree by far more than either wheel's own accel
  // authority -- and the tie-break's plain min-selection does not by
  // itself protect the wheel it did NOT bind on. This closes that gap
  // directly, wheel by wheel, rather than trusting the tie-break alone.
  //
  // Ceiling only, never floor: pushing a wheel to decelerate harder than
  // its configured aDecel is a harder brake than planned, not a runaway
  // -- always the SAFE direction to be wrong in, unlike exceeding aMax,
  // which is the actual defect this whole change guards against (see
  // clasi/issues/replace-rescales-carried-profile-velocity-by-new-shape.md).
  // boundaryLambda()'s curvatureHandoffLambdaCap() already keeps this
  // rail from doing the real work in the common case; it only bites on
  // the rare tick where that cap and the tie-break still disagree.
  for (int w = 0; w < 2; ++w) {
    if (!constrains[w]) continue;
    const AxisLimits lim = scaleLimits(active_.wheelLimits, magnitude[w]);
    if (lim.aMax <= 0.0f) continue;  // unconfigured: no ceiling to enforce
    const float ceilLambda = (previousWheel[w] + lim.aMax * dt) / magnitude[w];
    lambda = std::min(lambda, ceilLambda);
  }

  // Never accelerate at the end: once braking has begun, the command may
  // fall or hold but never rise again. Deliberately non-INCREASING rather
  // than strictly decreasing -- if a successor is queued mid-decel and
  // lifts the boundary, holding is right and re-accelerating is not.
  // The clamp is the BINDING wheel's own previous command translated back
  // to shape space (previousWheel[binding]/magnitude[binding]), the same
  // per-wheel-actual quantity profileStep() was just fed for it -- not the
  // old axisPerLambda-rescaled scalar, for the same reason planWheels()
  // stopped feeding that to profileStep() above.
  StepPhase raw = phase[binding];
  if (active_.decelLatched) {
    const float previousLambdaBinding = magnitude[binding] > kUnitEpsilon
        ? previousWheel[binding] / magnitude[binding]
        : 0.0f;
    lambda = std::min(lambda, previousLambdaBinding);
    if (raw != StepPhase::Closing) raw = StepPhase::Decel;
  } else if (raw == StepPhase::Decel || raw == StepPhase::Closing) {
    active_.decelLatched = true;
  }

  switch (raw) {
    case StepPhase::Accel: active_.phase = MovePhase::Accel; break;
    case StepPhase::Hold: active_.phase = MovePhase::Hold; break;
    case StepPhase::Decel:
    case StepPhase::Closing: active_.phase = MovePhase::Decel; break;
  }

  // Shape space -> axis units, and out to the wheels. The unit ratio
  // carries the Move's direction, so no separate sign is applied here.
  const float axisVelocity = lambda * active_.axisPerLambda;
  profileAccel_ = (axisVelocity - profileVelocity_) / dt;
  profileVelocity_ = axisVelocity;
  active_.closingIssued = closing[binding];
  cmdLeft_ = lambda * unit[0];
  cmdRight_ = lambda * unit[1];
}

void Planner::applyHeadingHold() {
  if (limits_.headingHoldGain <= 0.0f) return;
  // P on the uncommanded axis, back toward the Move's activation heading.
  const float error = active_.baselineHeading - pose_.heading();  // [rad]
  const float omegaCorrection = limits_.headingHoldGain * error;  // [rad/s]
  float differential = omegaCorrection * 0.5f * limits_.trackWidth;  // [mm/s]

  // Clamp the CORRECTION, never the profiled velocity: the faster wheel
  // must stay inside vMax, and the mean of the pair -- which is what the
  // odometry integrates as ds, and therefore what the distance accounting
  // depends on -- must come out exactly as profiled.
  const float profiled = 0.5f * (cmdLeft_ + cmdRight_);  // [mm/s]
  const float headroom = std::max(0.0f, limits_.vMax - std::fabs(profiled));
  differential = std::clamp(differential, -headroom, headroom);

  cmdLeft_ = profiled - differential;
  cmdRight_ = profiled + differential;
}

void Planner::drainToZero(float dt) {
  rollCommandHistory();
  // Drain at the ceiling the PROFILE planned against, not the body-frame
  // linear one. profileStep() only issues its exact terminal step when the
  // landing velocity is within one decel step OF THAT SHAPE's ceiling, so
  // draining at a smaller ceiling leaves a residual command the plant
  // integrates as unaccounted travel past the target -- measured as ~18 um
  // of per-wheel landing error on a tight arc, whose shape-space decel is
  // nearly 2x the linear one. Identical for the 1:1 and -1:1 shapes, where
  // the two ceilings coincide.
  const float decelStep = drainDecel(dt);
  rampCommandsToZero(decelStep);
  profileVelocity_ = std::max(0.0f, profileVelocity_ - limits_.aDecel * dt);
}

// The decel ceiling the drain and the planned stop ramp at: the active
// shape's own, when there is one, else the body-frame linear ceiling.
float Planner::drainDecel(float dt) const {  // [s] -> [mm/s] per interval
  if (active_.occupied && active_.shape.valid &&
      active_.wheelLimits.aDecel > 0.0f) {
    return active_.wheelLimits.aDecel * dt;
  }
  if (lastShapeDecel_ > 0.0f) return lastShapeDecel_ * dt;
  return limits_.aDecel * dt;
}

// Ramp the staged pair toward zero by one decel step, PROPORTIONALLY: the
// dominant wheel takes the full step and the other is scaled to match, so
// the commanded left:right ratio is preserved all the way down.
//
// Subtracting the same absolute step from each wheel instead -- which is
// what this did before the per-wheel rework -- silently changes the ratio
// whenever the wheels differ, so a robot stopping out of an arc would
// straighten as it slowed. Harmless for the 1:1 and -1:1 shapes (equal
// magnitudes, identical arithmetic, every pre-existing test unchanged),
// wrong for everything else. The ratio lock guards the profiled command;
// this guards the same property through the drain and the planned stop.
void Planner::rampCommandsToZero(float decelStep) {  // [mm/s] per interval
  const float dominant = std::max(std::fabs(cmdLeft_), std::fabs(cmdRight_));
  if (decelStep <= 0.0f || dominant <= decelStep) {
    // An unconfigured decel ceiling must still stop, and a pair already
    // inside one step lands exactly on zero rather than overshooting it.
    cmdLeft_ = 0.0f;
    cmdRight_ = 0.0f;
    return;
  }
  const float scale = (dominant - decelStep) / dominant;
  cmdLeft_ *= scale;
  cmdRight_ *= scale;
}

void Planner::update(Types::RobotState& state) const {
  // 130-005: this used to be "the ONE place the closed loop is summed into
  // the open loop" (stagedLeft = cmdLeft_ + trimLeft_, Motion::WheelTrim's
  // correction). That closed loop is deleted -- App::Drive is now the ONE
  // wheel-speed controller every cmdVelocity writer shares (drive.h's own
  // header) -- so Motion::Planner publishes the bare profiled command,
  // untouched, and Drive does 100% of the correction on the way to duty.
  const float stagedLeft = cmdLeft_;    // [mm/s]
  const float stagedRight = cmdRight_;  // [mm/s]
  state.wheelLeft.cmdVelocity = stagedLeft;
  state.wheelRight.cmdVelocity = stagedRight;

  // cmdAccel (130-003): a forward finite difference of the staged command
  // across the one-tick interval that just elapsed -- the accel Drive's own
  // Stage B/C consume (drive.h's own header). update() is const and cannot
  // roll the history itself (that happens in tick(), the one mutating half
  // of this class's own two-method contract), so it reads the previous
  // generation rollCommandHistory() already aged for it.
  const float dt = limits_.controlPeriod * 0.001f;  // [s]
  state.wheelLeft.cmdAccel = (stagedLeft - cmdLeftPrevious_) / dt;
  state.wheelRight.cmdAccel = (stagedRight - cmdRightPrevious_) / dt;

  // The real (124) Command section carries mode + the commanded twist, not
  // a move id -- completion/ack identity rides TickResult, never the state.
  state.command.moveActive = active_.occupied;
  state.command.mode =
      active_.occupied ? Types::Mode::Velocity : Types::Mode::Idle;
  // Report what is actually being ASKED of the wheels -- this is telemetry
  // for a human, not the planner's own ledger.
  state.command.v_x = 0.5f * (stagedLeft + stagedRight);
  state.command.omega = (stagedRight - stagedLeft) / limits_.trackWidth;

  // NOTE (128-016, robot-state-pose-needs-exactly-one-writer.md): this
  // block used to also write state.pose.* from pose_ (PoseTracker, this
  // class's own internal working estimate, OTOS-blended whenever
  // limits_.headingOtosWeight > 0) -- a SECOND writer of RobotState::pose
  // alongside Motion::Odometry::integrate()/App::RobotLoop::publishPose(),
  // ordering-dependent on which ran last a given cycle. Deleted: Odometry
  // is now pose's ONE writer (robot_state.h's own Pose section doc
  // comment). pose_ remains this class's own internal working estimate --
  // still fed into state.estimate.body just below -- and bodyVelocity/
  // omegaBody stay live for that same assignment; they are simply no
  // longer ALSO pushed onto state.pose here.
  const float bodyVelocity = 0.5f * (left_.velocity() + right_.velocity());
  const float omegaBody =
      (right_.velocity() - left_.velocity()) / limits_.trackWidth;

  state.estimate.wheelLeft = {left_.basisPosition(), left_.velocity(),
                              left_.basisTime(), left_.valid()};
  state.estimate.wheelRight = {right_.basisPosition(), right_.velocity(),
                               right_.basisTime(), right_.valid()};
  // The body basis is stamped at the WHEEL anchors, not at the tick: the
  // pose is integrated from measured anchor positions, so it is an
  // estimate as of the older of the two anchors, and a consumer holding a
  // copied state must extrapolate from there -- which is exactly what the
  // basisTime + velocity pair is for. Stamping it `now` would claim a
  // freshness the pose does not have.
  const uint32_t basisTime =
      left_.basisTime() < right_.basisTime() ? left_.basisTime()
                                             : right_.basisTime();
  state.estimate.body = {pose_.x(),    pose_.y(), pose_.heading(),
                         bodyVelocity, 0.0f,      omegaBody,
                         basisTime,    ticked_ && left_.valid() &&
                                           right_.valid()};
}

}  // namespace Motion
