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
// The `arrived` event (tick()'s own doc comment) is what actually closes
// this gap -- a Distance/Angle Move within settleEpsilonLinear/Angular AND
// at rest completes directly, without ever needing this tight epsilon to
// be reached at all.
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

// TERMINAL FINE-ALIGN (134-003, MoveLifecycle::Aligning). Two derived
// bounds; the phase's two TUNED numbers are config, not constants
// (PlannerLimits::Landing::alignTol/alignMaxNudges).
//
// kAlignRestWindow -- how long the body must READ at rest, with the staged
// command already drained to zero, before its heading is trusted enough to
// size a corrective nudge from. Same value and the same reasoning as
// kStallWindow above: "has actually stopped" is a claim about a body, not
// about one sample, and the filtered omega this tests is an EMA over
// samples that are one sample-age plus one actuationDelay stale. The bench
// report's §6 measured the next command landing in the SAME control cycle
// as the previous DONE ack, with the plant still moving in 3/5 trials --
// a heading read there is noise, and a nudge sized from noise is a nudge
// spent closing a residual that was never there. Deliberately its own
// constant rather than a reuse of kStallWindow: the stall backstop answers
// a different question ("is this Move going nowhere?") and the two are
// free to move apart.
constexpr float kAlignRestWindow = 0.5f;  // [s]
//
// kAlignSettleAllowance -- the plant-coast slack added to each nudge's own
// bounded window, mirroring plannedStopWindow()'s own settle allowance and
// sized the same way: enough for a commanded ramp that has reached zero to
// actually stop the body.
constexpr float kAlignSettleAllowance = 500.0f;  // [ms]
//
// kAlignNudgeOmegaFraction -- the corrective pivot's angular cruise, as a
// fraction of the robot's own configured omegaMax. Sized to reproduce the
// host-side graft this phase moves into firmware: 333 measured nudges were
// delivered at 30 mm/s of wheel speed, and on `tovez`'s configured 3.0
// rad/s ceiling this fraction gives 0.45 rad/s, which across its 128 mm
// track IS 28.8 mm/s. Derived from configured limits rather than written
// as a wheel-speed literal because the planner plans in body/shape space
// and has no business knowing a wheel speed. It is a CEILING, not a
// schedule -- profileStep() plans the actual command against the measured
// residual, so a sub-degree trim never approaches it.
constexpr float kAlignNudgeOmegaFraction = 0.15f;  // [1]

// Arrival gates (settleReached(), used directly by tick()'s `arrived`
// event -- 130-008 deleted the settle-confirm DEFER path that used to sit
// between profile-complete and these, but the tolerances themselves are
// unchanged and still very much live). Unlike the done epsilons above
// these ARE physical tolerances -- "close enough to the target, and
// stopped" for a real, lagging plant.
// (Arrival tolerances live in PlannerLimits::settleEpsilonLinear/
// settleEpsilonAngular -- reachability depends on the robot's
// stiction-limited minimum creep step, a per-robot property.)
// Rest floors -- settleReached()'s ONLY velocity criterion. (An earlier
// revision widened the gate to max(floor, one decel step), reasoning a
// commanded plant one step from zero is at rest next interval -- but a
// REAL plant with time constant tau COASTS ~v*tau past the target after
// the command reaches zero: at alphaDecel*dt = 0.25 rad/s that admitted
// +0.9 deg of post-settle coast per turn. The floors are sized so the
// worst coast is within the arrival epsilons.)
// (Rest floors live in PlannerLimits::settleRestVelocity/settleRestOmega
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

// What this Move was ASKED to do on its own axis, as opposed to what its
// `threshold` commands the wheels to do (134-001, Move::requestedThreshold's
// own doc comment). Only the cumulative-baseline ledger wants this; every
// profiling/measurement path deliberately keeps reading `threshold`, which
// is the command an ingestion-side corrector shaped for this plant.
//
// <= 0 is UNSET (Move::requestedThreshold's fail-open convention): a caller
// that never sets the field gets `threshold` back, which for a caller with
// no ingestion-side rewrite in front of it IS the intent.
float requestedThresholdOf(const Move& m) {
  return m.requestedThreshold > 0.0f ? m.requestedThreshold : m.threshold;
}

}  // namespace

Planner::Planner(const PlannerLimits& limits) : limits_(limits) {
  left_.configure(limits_.plant.velocityFilterWeight);
  right_.configure(limits_.plant.velocityFilterWeight);
  pose_.configure(limits_.plant.trackWidth);
}

void Planner::applyShaperLimits(float aMax, float aDecel, float alphaMax,
                                float alphaDecel, float jerkMax,
                                float yawJerkMax) {
  limits_.ceilings.aMax = aMax;
  limits_.ceilings.aDecel = aDecel;
  limits_.ceilings.alphaMax = alphaMax;
  limits_.ceilings.alphaDecel = alphaDecel;
  limits_.ceilings.jerkMax = jerkMax;
  limits_.ceilings.yawJerkMax = yawJerkMax;
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
    // 134-006: a preemption abandons whatever chain the ledger was keeping
    // -- the replacement is a new intent, not the next link -- so the
    // replacement re-anchors to its own activation pose. Explicit now that
    // the carry can legitimately outlive an empty queue (activateNext());
    // before that it could only ever be false here, and the drop was an
    // accident of the queue-occupancy proxy rather than a decision.
    carryValid_ = false;
    idleLatched_ = false;
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
  const float drain = limits_.ceilings.aDecel > 0.0f
                          ? 1000.0f * limits_.ceilings.vMax / limits_.ceilings.aDecel  // [ms]
                          : 0.0f;
  // PlannerLimits::settleWindow (the "how long may arrival take" allowance
  // this used to read when the robot had one configured) is deleted by
  // 130-009 -- dead everywhere except this one reader once the
  // settle-confirm defer path itself was dissolved (130-008). Falls back
  // to this fixed floor unconditionally now, same value every robot JSON
  // already baked (2500ms), so an unconfigured planner still converges.
  constexpr float kDefaultSettleAllowance = 500.0f;  // [ms]
  return drain + kDefaultSettleAllowance;
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
  // ANY -> Idle (tick()'s own doc comment): unconditional, this instant --
  // not deferred to the next tick(), so an observer reading lifecycle()
  // right after estop() never sees a stale active-Move state. That
  // includes Aligning: a panic stop abandons the trim mid-nudge, with no
  // completion ack, exactly as it abandons any other active Move.
  lifecycle_ = MoveLifecycle::Idle;
  align_ = AlignState{};
  // 134-006: and the ledger. A panic stop is the canonical "somebody else
  // is in charge of the robot now" event, so whatever cumulative baseline
  // was staged is void and the next Move re-anchors to its own pose. This
  // used to fall out of activateNext()'s queue-occupancy proxy (estop()
  // empties the queue); with the proxy gone it has to be said here.
  carryValid_ = false;
  idleLatched_ = false;
}

// Age the staged command by one tick. Called from the two places that
// overwrite cmdLeft_/cmdRight_ -- exactly one of which runs per tick --
// and never before measure(), which reads both generations.
void Planner::rollCommandHistory() {
  cmdLeftPrevious_ = cmdLeft_;
  cmdRightPrevious_ = cmdRight_;
}

// tick() -- the Move lifecycle state machine (130-008,
// planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md
// item 2). Motion::MoveLifecycle (planner_types.h) is the explicit state;
// this comment is the transition table that used to exist only as the
// order of a chain of `if`s over interacting booleans (`occupied`,
// `hasMoved`, `settling`).
//
// STATES
//
//   Idle       no active Move; the staged command is already zero.
//   Draining   no active Move; ramping the staged command toward zero.
//   Breakaway  active Move, arrival/stall detection not yet armed -- the
//              body has not been MEASURED to have left rest.
//   Tracking   active Move driving its profile (MovePhase sub-phase:
//              Accel/Hold/Decel).
//   Aligning   134-003: a Twist Angle Move whose profile has LANDED, held
//              open while it trims its cumulative-heading residual. The
//              Move is still active and still un-acked here.
//   Stopping   an active Kind::Stop entry ramping the body to rest.
//
// TRANSITIONS
//
//   Idle/Draining -> Breakaway    activateNext() pops a Distance/Angle
//                                 Kind entry (either velocityKind).
//   Idle/Draining -> Tracking     activateNext() pops a Kind::Time entry
//                                 -- stall/arrival detection never applies
//                                 to a clock-bounded Move, so there is
//                                 nothing to break away from.
//   Idle/Draining -> Stopping     activateNext() pops a Kind::Stop entry.
//   Breakaway -> Tracking          the measured velocity/omega crosses OUT
//                                 of the rest floor (settleRestVelocity/
//                                 settleRestOmega below) -- one-way: a Move
//                                 that later coasts back through the floor
//                                 has still left rest once. (Unlike this
//                                 transition, `decelLatched` -- planWheels()'s
//                                 own doc comment -- is a CONDITIONAL latch
//                                 as of 131-006: it releases when a later
//                                 tick's fresh recomputation shows the Move
//                                 was never truly braking, so it is no
//                                 longer a clean analogy for "one-way.")
//   {Breakaway,Tracking} -> Aligning
//                                 134-003: a TWIST ANGLE Move's own
//                                 completion event fires (EVENT 3 or 4
//                                 below -- never 1 or 2, a timed-out or
//                                 stalled Move aborts instead), it is not
//                                 handing off at speed (activeBoundary_ <=
//                                 0), and both align config fields are set.
//                                 The Move does NOT complete here; it is
//                                 held open, and `TickResult::completed`
//                                 fires on the way OUT instead. See
//                                 alignStep() for the settle -> measure ->
//                                 nudge -> re-settle loop this runs.
//   Aligning -> {Breakaway,Tracking,Stopping,Draining,Idle}
//                                 the alignment finishes: the residual is
//                                 inside alignTol, the alignMaxNudges
//                                 budget is spent, or the Move's own
//                                 wall-clock timeout fires from inside the
//                                 phase (which still outranks it, so a
//                                 non-converging alignment can never wedge
//                                 the queue). NOW the Move completes and
//                                 the row below applies.
//   {Breakaway,Tracking,Stopping,Aligning}
//     -> {Breakaway,Tracking,Stopping,Draining,Idle}
//                                 a Move COMPLETES (see EVENTS below) and
//                                 activateNext() either finds a next
//                                 pending entry (-> whichever active state
//                                 its own Kind selects, per the first row
//                                 of this table) or does not (-> Draining,
//                                 then Idle once the drain reaches zero).
//   ANY -> Idle                   estop(): the queue and the active Move
//                                 are cleared and the command zeroed in
//                                 the same call, unconditionally.
//   ANY -> {Breakaway,Tracking,Stopping}, on the NEXT tick()
//                                 move(..., replace=true): the active
//                                 Move is cleared and the replacement
//                                 queued; activateNext() on the following
//                                 tick() activates it exactly as any other
//                                 queued entry.
//
// EVENTS that complete the active Move (`done`), tested in this priority
// order -- this replaces what used to be implicit in the order of an
// `if`/`else if` chain:
//
//   1. timeout           m.timeout > 0 and elapsed >= m.timeout. Checked
//                        first regardless of Kind: a safety backstop
//                        always outranks the motion it is backstopping.
//   2. stall-window       the body has been at rest, making no measured
//      expiry            progress, for kStallWindow -- Distance/Angle
//                        Kind only, and only once Tracking (see
//                        `stallApplies` below).
//   3. arrived            Distance/Angle Twist Kind, Tracking, not
//                        handing off at speed (activeBoundary_ <= 0):
//                        inside settleEpsilonLinear/Angular AND at rest.
//   4. profile-complete   the Kind-specific test: Time's clock,
//                        Distance/Angle's planned-residual epsilon
//                        (kDoneEpsilon*), or a Kind::Stop's
//                        drained-command-and-at-rest test.
//
// 134-003 inserts ONE state between events 3/4 and the ack, and changes
// nothing about the events themselves: a Twist Angle Move that fires event
// 3 or 4 with activeBoundary_ <= 0 enters Aligning instead of completing,
// and completes when the trim converges, exhausts its nudge budget, or the
// Move's own event-1 timeout fires from inside the phase. Events 1 and 2
// still abort outright -- a timed-out or stalled Move has not delivered its
// motion and has nothing honest to trim toward.
//
// One reporting subtlety: an event-1 expiry from INSIDE Aligning ends the
// Move (the queue is never wedged) but is NOT reported as
// `TickResult::timedOut`. That field is the wire's kFlagFaultMoveTimeout, a
// fault on the MOTION, and a Move in Aligning has already met its stop
// condition -- see `motionTimedOut` at the completion site.
//
// `TickResult::settled` is always settleReached(), evaluated truthfully
// regardless of which event completed the Move -- 130-008 deletes the
// settle-confirm DEFER path that used to hold a completion reached via
// event 4 back until settled (its own `Settling` sub-state, gated on
// PlannerLimits::requireSettle). Event 3 already answers the identical
// question directly and completes the instant it holds, so deferring a
// completion reached via 1/2/4 bought nothing that reporting `settled`
// truthfully at the same tick does not.
TickResult Planner::tick(const Types::RobotState& state) {
  TickResult result{};
  const uint32_t now = state.time.cycleStart;
  const float dt = limits_.plant.controlPeriod * 0.001f;  // [s]
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
  // 131-004: pass each wheel's positionEpoch through so pose_ re-anchors
  // across a rebaseline instead of differencing across the discontinuity
  // (Motion::PoseTracker::integrate()'s own doc comment, estimation.h).
  pose_.integrate(left_.basisPosition(), right_.basisPosition(), state.wheelLeft.positionEpoch,
                  state.wheelRight.positionEpoch);

  // Heading comes from the OTOS whenever the chip has it, because every
  // angular decision below -- the Angle stop condition, terminal fine-align,
  // and the carryHeading_ ledger -- measures pose_.heading(). Wheel heading is
  // scrub-limited (rotationalSlip is a fitted fudge, not a measurement), which
  // is precisely why open-loop turns need a per-direction gain/offset
  // calibration to land. Optical heading needs none.
  //
  // Absent or stale chip falls straight back to the wheel path, with no jump
  // at either transition (PoseTracker::applyOtosHeading()).
  // NEGATED, and that is not a fudge -- it reconciles a real, pre-existing
  // sign inversion. Measured 2026-08-05 on a single commanded rotation: the
  // OTOS turned +84.58deg while the encoder-derived heading reported
  // -82.45deg. Same magnitude, opposite sign. The wheel path has always been
  // internally self-consistent (it closes the loop on its own heading), so
  // nothing ever surfaced it -- but it is why every host tool carries a
  // YAW_SIGN = -1, and feeding unnegated optical truth into a planner built
  // on the inverted convention makes the Angle stop condition count the wrong
  // way and spin forever.
  //
  // The RIGHT fix is the body kinematics' omega sign, so commanded omega and
  // world CCW finally agree. That is deliberately not done here: it changes
  // the meaning of omega on the wire and invalidates both stored per-direction
  // rotation calibrations, so it needs its own change with the bench free.
  pose_.applyOtosHeading(-state.otos.heading, state.otos.present, state.otos.connected);
  // OTOS heading blend -- DELETED by 130-009 along with
  // PlannerLimits::headingOtosWeight/otosStaleness: the feature was live
  // code (pose_.blendHeading()) but configured off in every robot JSON
  // (OTOS blend weight 0 everywhere), and the sprint scoped it out in
  // favor of a from-scratch estimator-v2 fusion design (tracked separately,
  // clasi/issues/later/estimator-v2-otos-fusion-sim-first.md) rather than
  // reshaping this ad hoc blend. pose_.blendHeading() itself is untouched
  // (estimation.h/.cpp) -- a future estimator-v2 ticket may call it again
  // from a different site; only this call site and its two config fields
  // are gone.

  if (!active_.occupied) activateNext(now);
  if (!active_.occupied) {
    // queue-empty: nothing to drive. Idle once the staged command is
    // already at zero, Draining while it is still being ramped down --
    // the only place either of those two states is decided, read directly
    // off the command rather than tracked as a separate flag.
    lifecycle_ = (cmdLeft_ == 0.0f && cmdRight_ == 0.0f)
                     ? MoveLifecycle::Idle
                     : MoveLifecycle::Draining;
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

  // TERMINAL FINE-ALIGN (134-003). This Move's profile has already landed
  // and the Move is being held open to trim its cumulative-heading
  // residual, so alignStep()'s own bounded settle -> measure -> nudge ->
  // re-settle loop stands in for the stall/arrival/profile-complete chain
  // below for as long as it runs. The wall-clock timeout is the ONE thing
  // that still outranks it (EVENT 1 in this function's own doc comment: a
  // safety backstop always outranks the motion it is backstopping), which
  // is what makes it impossible for a non-converging alignment to wedge
  // the queue.
  const bool aligning = lifecycle_ == MoveLifecycle::Aligning;
  bool done = false;
  if (aligning) {
    done = timedOut || alignStep(now, dt, measured);
    // Not finished: alignStep() has already staged this tick's command,
    // exactly one staging call per tick like every other path out of here.
    if (!done) return result;
  }

  // Stall backstop + the Breakaway -> Tracking advance. Only for the two
  // kinds whose completion is a POSITION the plant has to reach and that
  // are meant to land at rest -- a Time Move's stop condition is the
  // clock, and a Move handing off at speed (activeBoundary_ > 0) is not
  // supposed to stop at all, so neither can legitimately stall or needs
  // arrival detection armed. Evaluated before `done` so a stalled Move
  // completes on the same tick it is recognised.
  //
  // Never during Aligning (134-003): that Move's profile has already
  // landed, so it IS at rest and making no progress by construction --
  // every settle tick would read as a stall. The alignment's own nudge
  // budget and per-nudge window are its backstops, plus the Move timeout.
  const bool stallApplies =
      !aligning &&
      (m.kind == Move::Kind::Distance || m.kind == Move::Kind::Angle) &&
      activeBoundary_ <= 0.0f;
  bool stalled = false;
  if (m.kind == Move::Kind::Stop) {
    lifecycle_ = MoveLifecycle::Stopping;
  } else if (stallApplies) {
    const bool angular = (m.kind == Move::Kind::Angle);
    const bool atRest =
        angular ? std::fabs(measured.omega) <= limits_.landing.settleRestOmega
                : std::fabs(measured.bodyVelocity) <= limits_.landing.settleRestVelocity;
    const float epsilon =
        angular ? kStallEpsilonAngular : kStallEpsilonLinear;
    // Progress is measured against where the residual stood when the window
    // opened, NOT against the previous tick: a residual creeping by less
    // than epsilon per tick would otherwise reset the counter forever while
    // going nowhere.
    if (!atRest) {
      // Left rest: arrival/stall detection is armed for the remainder of
      // this Move. One-way -- a Move that later coasts back through the
      // rest floor has still left rest once. (`decelLatched` is a
      // DIFFERENT, conditional latch as of 131-006 -- see planWheels()'s
      // own doc comment -- so it is no longer a clean "one-way" analogy.)
      lifecycle_ = MoveLifecycle::Tracking;
      active_.stallTicks = 0;  // moving: not stalled, wherever it is
    } else if (lifecycle_ == MoveLifecycle::Breakaway) {
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
  // else: stallApplies is false (a Kind::Time Move, or a Distance/Angle
  // Move already handing off at speed) -- lifecycle_ is left exactly as
  // activateNext() set it (Tracking for Kind::Time; a stale Breakaway
  // reading is possible here for a Distance/Angle Move whose boundary
  // turned positive before it was ever observed leaving rest, but
  // harmless -- every consumer of "has this Move left rest" below
  // independently re-tests activeBoundary_ <= 0.0f, so a stale Breakaway
  // label can never suppress or admit a completion it should not).

  // `done` is already true and settled when `aligning` -- the alignment
  // finished (or its Move timed out) and this tick is its completion tick;
  // re-deriving it from the stall/timeout pair would silently discard that.
  if (!aligning) done = timedOut || stalled;
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
  // on lifecycle_ == Tracking so a Move cannot complete on its own
  // activation tick before breaking away. Residual left on the target is
  // absorbed by the next chained Move through the cumulative baseline
  // ledger -- exactly the mechanism src/firm/main.cpp already relies on.
  const bool arrived =
      activeBoundary_ <= 0.0f && lifecycle_ == MoveLifecycle::Tracking &&
      m.velocityKind == Move::VelocityKind::Twist &&
      (m.kind == Move::Kind::Distance || m.kind == Move::Kind::Angle) &&
      settleReached(measured, dt);
  if (!done && arrived) {
    done = true;
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
                                      limits_.plant.controlPeriod >
                                  m.threshold;
        break;
      case Move::Kind::Distance:
        // Carry boundary (>0): hand off the tick the crossing falls inside,
        // debiting the sub-tick residual to the next Move's cumulative
        // baseline. Final/orthogonal boundary (0): the terminal step has
        // already closed the sum -- complete at (float-noise) zero.
        //
        // NOT gated on profileVelocity_ the way Move::Kind::Angle is just
        // below -- tried and reverted (130-010): it measurably regressed
        // src/tests/sim/system/test_straight_leg_crab_regression.py (a
        // dedicated 119-005 permanent regression test with a tight 0.3deg
        // tolerance) to a genuine +2.0deg crab on an otherwise-isolated
        // straight leg, where applyHeadingHold()'s own per-tick correction
        // (planActive()'s Move::Kind::Distance case) stops running the
        // instant the Move leaves the active Distance branch -- extending
        // how long the Move stays active before calling itself done, on
        // THIS axis, extends the window that correction has to lock in
        // whatever small per-wheel asymmetry the plant's own tail leaves,
        // rather than shortening it. The reported defect
        // (sim-tour-turn-shaping-undershoots-90-degree-turns.md) is a
        // ROTATION (Angle) undershoot; the fix below is scoped to that.
        done = measured.plannedRemaining <= kDoneEpsilonLinear ||
               (activeBoundary_ > 0.0f &&
                measured.plannedRemaining <=
                    profileVelocity_ * dt + kDoneEpsilonLinear);
        break;
      case Move::Kind::Angle:
        // Same "terminal step already closed the sum" premise as
        // Move::Kind::Distance above, but plannedRemaining reaching the
        // float-noise floor does NOT mean the ramp has actually reached
        // its own tail on a plant with a longer real actuation lag than
        // measure()'s own short (actuationDelay-sized) lookahead assumes
        // (this profile's own PID-disabled, pure first-order WheelPlant
        // response, tau far past actuationDelay). Gating this branch on
        // profileVelocity_ itself being at the rest floor (limits_.
        // landing.settleRestOmega, the SAME floor settleReached() uses to
        // call the MEASURED body "at rest") makes "the terminal step has
        // already closed the sum" true again: this branch now only fires
        // once the commanded ramp has actually decayed to its own last,
        // near-zero step -- not merely once the lookahead-adjusted
        // prediction says so. Measured 2026-08-02 (130-010): without this
        // gate, an isolated 90deg pivot reported `done` at profileVelocity_
        // still ~30 deg/s, roughly 12-17 degrees of true rotation short of
        // where the body eventually settled -- and in a chained tour,
        // where the very next Move immediately overrides the
        // still-spinning wheels, that shortfall is never recovered,
        // showing up as a per-turn undershoot
        // (sim-tour-turn-shaping-undershoots-90-degree-turns.md). The
        // `arrived` event above already covers the fully-honest (measured,
        // at-rest) case for a Twist Angle Move; this gate brings the
        // Wheels-velocityKind case (which `arrived` structurally excludes,
        // see its own guard above) up to the same "commanded ramp has
        // actually finished" standard, without waiting out the plant's own
        // slower physical coast the way the stall backstop would. Unlike
        // Move::Kind::Distance, an Angle Move has no per-tick
        // applyHeadingHold()-style correction running while active for a
        // longer completion window to disturb -- the Distance regression
        // above does not apply here.
        //
        // settleRestOmega <= 0 is treated as unconfigured (the SAME
        // fail-open convention shapeLimits()'s tighten() uses for a <=0
        // ceiling: "contributes nothing," not "zero tolerance"): the
        // rest-floor gate is then a no-op, matching this branch's
        // pre-130-010 behavior for any caller that leaves the field at
        // its structural zero rather than PlannerLimits::Landing's own
        // 0.02 rad/s in-class default.
        done = ((limits_.landing.settleRestOmega <= 0.0f ||
                 std::fabs(profileVelocity_) <= limits_.landing.settleRestOmega) &&
                measured.plannedRemaining <= kDoneEpsilonAngular) ||
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

  // TERMINAL FINE-ALIGN ENTRY (134-003). A Twist Angle Move that got here
  // honestly -- its own arrival or profile-complete event, not a timeout or
  // a stall, and not handing off at speed -- does not complete yet. It is
  // held open in MoveLifecycle::Aligning to trim whatever heading residual
  // its landing left, against the cumulative-intent ledger. The completion
  // ack fires on the way OUT (the `if (done)` block below, reached on a
  // later tick), because a Move is not done until it has landed.
  if (done && !aligning && alignApplies(m, timedOut, stalled)) {
    enterAligning(now);
    alignSettleCommand(dt);  // stages this tick's command (and rolls history)
    return result;           // result.completed stays false
  }

  if (done) {
    // settleReached() is reported truthfully and unconditionally -- the
    // settle-confirm DEFER path that used to sit here (holding a
    // completion back until settled, up to PlannerLimits::settleWindow,
    // via its own `Settling` sub-state) is deleted by 130-008: the
    // `arrived` event above already tests settleReached() directly and
    // completes the instant it holds, so deferring a completion reached
    // via timeout/stall/profile-complete bought nothing that reporting
    // `settled` truthfully at the same tick does not.
    result.completed = true;
    result.moveId = m.id;
    // 134-003: `timedOut` answers "did this Move END VIA ITS BACKSTOP
    // instead of via its stop condition?" -- it is what the wire publishes
    // as kFlagFaultMoveTimeout, a FAULT flag, and what the ledger reads as
    // "the motion was aborted". A Move that reached Aligning already MET
    // its stop condition; the profile landed and only the trim was still
    // running. A wall-clock expiry there is the trim running out of budget,
    // not the motion failing, and reporting it as a move-timeout fault
    // would be a false fault on a Move that drove correctly.
    //
    // The queue is freed either way -- that is what the backstop is for,
    // and it still fires from inside Aligning. What changes here is only
    // the LABEL on a completion, and the label follows the motion.
    // Alignment cost is reported where it belongs: in ticket 004's own
    // per-corner nudge count and wall time, not as a fault bit.
    const bool motionTimedOut = timedOut && !aligning;
    result.timedOut = motionTimedOut;
    result.settled = settleReached(measured, dt);
    // Cumulative-baseline carry (chain-exact accounting): a normally
    // completed Twist Distance/Angle Move hands the next Move BOTH
    // cumulative baselines -- "where the boundary WAS MEANT to be" on its
    // own axis (baseline + this Move's own INTENT on that axis:
    // `threshold` for Distance, requestedThresholdOf() for Angle -- see
    // the Angle branch's own comment below) and its own UNCHANGED baseline
    // on the other axis (a straight leg intends zero heading change; a
    // pivot intends zero path change). Carrying the full ledger across
    // KINDS is what closes a mixed leg/turn tour: each turn targets the
    // cumulative n*90deg and each leg's heading-hold pulls back to the carried
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
      // 134-003: `motionTimedOut`, not the raw `timedOut` -- an expiry
      // inside Aligning is the trim running out of budget, not an aborted
      // motion (see its definition above). Dropping the carry there would
      // re-anchor the chain to wherever the trim gave up, which is exactly
      // the re-anchoring 134-001 removed; the residual is owed to the next
      // corner either way.
      carryValid_ = !motionTimedOut &&
                    m.velocityKind == Move::VelocityKind::Twist &&
                    (m.kind == Move::Kind::Distance ||
                     m.kind == Move::Kind::Angle);
      if (m.kind == Move::Kind::Distance) {
        carryPath_ = active_.baselinePath + linearDirection(m) * m.threshold;
        carryHeading_ = active_.baselineHeading;
      } else {
        carryPath_ = active_.baselinePath;
        // INTENT, not measurement: baseline + what this Move was ASKED to
        // turn. 134-001 restores the projection 130-010 had to give up,
        // and the reason it can is Move::requestedThreshold -- the
        // caller's own request, captured in
        // App::RobotLoop::handleMove() BEFORE that function's
        // rotation-calibration inversion rewrites `threshold` into an
        // actuation-sized command. 130-010 was right that `threshold`
        // could no longer be read as intent; it was wrong only in
        // concluding the intent was unrecoverable, and it carried
        // pose_.heading() instead.
        //
        // Carrying the MEASURED heading makes every landing its own new
        // truth: the residual a corner leaves is never owed to anybody, so
        // it is never repaid. Measured on the sequential planner square
        // tour (docs/bench-reports/motion-planning-lab-2026-08-04.md
        // §1/§2): +1.5 / +5.3 / +6.3 / +10.6 deg of cumulative heading
        // drift across four corners, 64.1 mm closure, against 7.3-11.5 mm
        // for the same firmware with a host-side per-corner correction
        // grafted on. Nothing but this carry differed.
        //
        // With the intent carried, the next Move activates against the
        // CUMULATIVE target (activateNext()'s ledger adoption), so
        // measure()'s own `m.threshold - (heading - baseline) * dir`
        // already starts a debt-carrying turn short by exactly the
        // residual -- the repayment needs no separate mechanism and no new
        // constant. A leg is repaid the same way one corner later:
        // Distance carries its heading baseline through unchanged (below),
        // so a leg's curl shows up as residual at the NEXT corner.
        //
        // 134-003: read the LATCHED target when this Move went through
        // Aligning. It is the SAME expression -- enterAligning() evaluates
        // exactly this line -- but evaluated BEFORE the corrective nudges
        // rewrote active_.move.threshold/omega and active_.baselineHeading
        // to reach planWheels() through the normal path. Recomputing it
        // here would read the last nudge's own baseline and give the
        // ledger the nudge's intent instead of the Move's.
        carryHeading_ = aligning
                            ? align_.target
                            : active_.baselineHeading +
                                  angularDirection(m) * requestedThresholdOf(m);
      }
    }
    active_.occupied = false;
    activateNext(now);
    if (!active_.occupied) {
      // queue-empty, same as the top-of-tick() case: Idle vs. Draining is
      // read off whether the just-completed Move left a nonzero command
      // to ramp down (Distance/Angle/Time) or was already at zero (Stop).
      lifecycle_ = (cmdLeft_ == 0.0f && cmdRight_ == 0.0f)
                       ? MoveLifecycle::Idle
                       : MoveLifecycle::Draining;
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
  // 134-003: whatever the previous Move's terminal fine-align was doing is
  // over the moment we look for a successor. Cleared here rather than at
  // each of the several exits from Aligning because activateNext() is on
  // every one of those paths -- completion, queue-drain, and replace().
  align_ = AlignState{};
  if (pendingCount_ == 0) {
    // GOING IDLE (134-006). This used to be where the cumulative-intent
    // carry died: `carryValid_ = carryValid_ && pendingCount_ > 0`, present
    // since planner v1, read an empty queue as "something else may have
    // moved the robot" and dropped the ledger. The INTENT was right -- a
    // teleop nudge, an estop, or a shove between Moves does invalidate a
    // staged baseline -- but queue occupancy is a false proxy for it: a
    // caller that simply waits for each completion ack before enqueuing the
    // next Move (planner_square_tour.py --sequential, and any GUI or script
    // driven the same way) has an empty queue at EVERY corner, so the
    // ledger was dropped at every corner and 134-001 was inert there.
    // Measured on the same leg+turn, differing only in whether the next
    // Move was queued before completion (docs/bench-reports/
    // sprint-134-004-bench-acceptance-2026-08-05.md): carry live, -0.38 deg
    // mean corner residual, 4/4 corners in tolerance; carry dropped,
    // +1.22 deg, 2/4.
    //
    // So the carry is PRESERVED across the gap here, and the real condition
    // -- did the heading move while nobody was driving? -- is tested at the
    // next activation below, against this latch.
    //
    // Refreshed for as long as the planner is still commanding motion of
    // its own: the post-completion ramp-out (drainToZero(), which this
    // function's callers run immediately after it returns) is the previous
    // Move ending, not an external disturbance, and latching before it
    // finished would charge the planner's own drain to the robot's
    // attacker. It freezes on the first tick with nothing staged; every
    // heading change after that instant is somebody else's.
    if (!idleLatched_ || cmdLeft_ != 0.0f || cmdRight_ != 0.0f) {
      idleHeading_ = pose_.heading();
      idleLatched_ = true;
    }
    return;
  }
  // ACTIVATING OUT OF AN IDLE GAP (134-006): the carry survives only if the
  // robot is still where the last Move left it. `pose_.heading()` is
  // unwrapped (estimation.h) and so is idleHeading_ -- both are readings of
  // the same continuous signal -- so this plain difference is the signed
  // drift with no wrapping to get wrong, the same argument alignStep()
  // makes for its own residual. Wrapping here would be actively unsafe: it
  // would fold a multi-turn disturbance back onto a small number and call
  // it undisturbed.
  //
  // The tolerance is landing.settleEpsilonAngular -- this robot's own
  // "arrived at a heading" resolution (0.035 rad on tovez), already the
  // yardstick for whether a heading reading is meaningfully different from
  // its target. Deliberately NOT a new PlannerLimits field: that path is
  // ~16 files (134-003 traced it) and this is not a new physical quantity.
  //
  // Fails closed by construction -- anything that is not a demonstrably
  // undisturbed gap drops the carry, exactly as the old proxy did. estop()
  // and a replace=true preemption clear carryValid_ at their own call
  // sites; this covers the external shove.
  //
  // The pipelined path (a successor already queued when its predecessor
  // completed) never sets idleLatched_, so it does not reach this check at
  // all and behaves exactly as it did before 134-006 -- that arm measured
  // better 3/3 in the same bench report and is not being touched.
  if (idleLatched_) {
    const float idleDrift = pose_.heading() - idleHeading_;  // [rad] signed
    if (std::fabs(idleDrift) > limits_.landing.settleEpsilonAngular) {
      carryValid_ = false;
    }
    idleLatched_ = false;
  }
  const Move next = pending_[0];
  for (int i = 1; i < pendingCount_; ++i) pending_[i - 1] = pending_[i];
  --pendingCount_;

  active_.occupied = true;
  active_.move = next;
  active_.activationTime = now;
  active_.closingIssued = false;
  active_.stallRemaining = 0.0f;
  active_.stallTicks = 0;
  // Initial lifecycle_ for this Move (tick()'s own doc comment has the
  // full transition table): Stopping for a Kind::Stop entry, Breakaway for
  // a Distance/Angle entry (arrival/stall detection starts disarmed --
  // stallApplies is always true at this instant, since activeBoundary_ is
  // reset to 0 a few lines below), Tracking for a Kind::Time entry (which
  // stallApplies never applies to -- nothing to break away from).
  lifecycle_ = next.kind == Move::Kind::Stop
                   ? MoveLifecycle::Stopping
                   : (next.kind == Move::Kind::Distance ||
                      next.kind == Move::Kind::Angle)
                         ? MoveLifecycle::Breakaway
                         : MoveLifecycle::Tracking;
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
  active_.shape = shapeOf(next, limits_.plant.trackWidth);
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
          limits_.plant.trackWidth;
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
      (right_.velocity() - left_.velocity()) / limits_.plant.trackWidth;  // [rad/s]
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
  const float delay = limits_.plant.actuationDelay * 0.001f;  // [s]
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
      (predictRight - predictLeft) / limits_.plant.trackWidth;               // [rad]

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
    return std::fabs(measured.bodyVelocity) <= limits_.landing.settleRestVelocity &&
           std::fabs(measured.omega) <= limits_.landing.settleRestOmega;
  }
  if (m.velocityKind != Move::VelocityKind::Twist) return false;
  switch (m.kind) {
    case Move::Kind::Stop:
      return false;  // unreachable: handled above
    case Move::Kind::Time:
      return false;  // nothing physical to confirm
    case Move::Kind::Distance:
      return std::fabs(measured.anchoredRemaining) <=
                 limits_.landing.settleEpsilonLinear &&
             std::fabs(measured.bodyVelocity) <= limits_.landing.settleRestVelocity;
    case Move::Kind::Angle:
      return std::fabs(measured.anchoredRemaining) <=
                 limits_.landing.settleEpsilonAngular &&
             std::fabs(measured.omega) <= limits_.landing.settleRestOmega;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Terminal fine-align (134-003, MoveLifecycle::Aligning).
//
// WHAT THIS IS. The per-corner heading trim, moved off the host and into
// the Move. Measured on `tovez` by the host-side graft it reproduces
// (docs/bench-reports/motion-planning-lab-2026-08-04.md §5.2): planner
// square-tour closure 25.8 -> 9.4 mm at a 1.0 deg tolerance, ~2 s/corner,
// with nothing else changed. §1 is why it works at all -- square-tour
// closure is propagated per-corner cumulative-heading residual
// (corr = +0.95), so the only work that moves closure is work that reduces
// that residual.
//
// WHY IT NEEDS BOTH ITS SIBLINGS. 134-001 gives it an honest target: with
// Move::requestedThreshold the ledger can project baseline + direction *
// INTENT again, instead of re-anchoring to wherever the corner stopped.
// 134-002 gives it a deliverable actuator: applySpeedFloor() no longer
// boosts planner-shaped commands, so a ~29 mm/s corrective pivot reaches
// the wheels as itself rather than as a v_min-sized lunge.
//
// THE LOOP, per iteration: settle (command drained to zero and the body
// reading at rest for kAlignRestWindow) -> measure the residual against
// the latched ledger target -> if it exceeds alignTol, emit one bounded
// low-speed pivot nudge through the ordinary planWheels() path -> re-settle
// -> re-measure. Capped at alignMaxNudges, then complete regardless.
//
// WHY RE-SETTLE EVERY TIME, not just before the first nudge: report §6
// measured the next command landing in the SAME control cycle as the
// previous DONE ack, with the plant still moving in 3/5 trials. Measuring
// heading on a body that is still turning reads noise, and a nudge sized
// from noise is a nudge spent against a residual that was never there --
// and the budget is only alignMaxNudges deep.

bool Planner::alignApplies(const Move& m, bool timedOut, bool stalled) const {
  // Unconfigured is OFF, on either field -- the same fail-open convention
  // the rest of this surface uses for a <=0 bound. Every direct caller that
  // never sets these (the ctests, the sim harnesses, the ctypes bench
  // harness) therefore keeps its pre-134-003 behavior exactly.
  if (!(limits_.landing.alignTol > 0.0f) || limits_.landing.alignMaxNudges <= 0) {
    return false;
  }
  // No configured angular ceiling means no nudge to size (alignNudgeOmega()
  // is a fraction of it), and a zero-speed "pivot" has no shape at all --
  // fail closed rather than burn the whole budget commanding nothing.
  if (!(alignNudgeOmega() > 0.0f)) return false;
  // Events 1 and 2 (tick()'s own doc comment) abort: a timed-out or stalled
  // Move has not delivered its motion, so there is no honest landing to
  // trim -- and the ledger deliberately carries no intent for it either.
  if (timedOut || stalled) return false;
  // TWIST ANGLE only, and never handing off at speed. The same guard set
  // the `arrived` event uses, and for the same reason: the ledger's own
  // carryValid_ is gated on VelocityKind::Twist, so a Wheels-velocity Move
  // has no heading intent to align against, and a Move with a positive
  // boundary is not supposed to come to a stop at all.
  //
  // The activeBoundary_ half is a REAL LIMITATION, not merely a guard: in
  // a PIPELINED tour every corner but the last hands off at speed and
  // therefore gets no trim. That is the right scope -- the measured
  // 25.8 -> 9.4 mm result came from a SEQUENTIAL tour
  // (planner_square_tour.py --sequential --trim), and a pipelined tour is
  // a different arm with a different number (report §2 finding 3: the
  // shipped pipelined tour's 26 mm is two errors cancelling, not a near
  // miss). Trimming a corner the caller deliberately asked to flow through
  // would also silently convert its at-speed hand-off into a full stop.
  return m.velocityKind == Move::VelocityKind::Twist &&
         m.kind == Move::Kind::Angle && activeBoundary_ <= 0.0f;
}

void Planner::enterAligning(uint32_t now) {
  lifecycle_ = MoveLifecycle::Aligning;
  align_ = AlignState{};
  align_.settleStart = now;
  // Latch the cumulative-intent target NOW, before any nudge rewrites the
  // Move it is derived from. Byte-identical to the expression tick()'s own
  // completion block uses for carryHeading_ (and to what measure() has been
  // driving this Move against): baseline + direction * INTENT, where intent
  // is the caller's own pre-calibration request (134-001).
  align_.target = active_.baselineHeading +
                  angularDirection(active_.move) *
                      requestedThresholdOf(active_.move);
}

bool Planner::alignStep(uint32_t now, float dt, const Measurement& measured) {
  if (align_.nudging) {
    // A corrective pivot is in flight. It ends when its own profile has
    // finished commanding -- the same test Move::Kind::Angle's
    // profile-complete branch uses, requiring the commanded ramp itself to
    // have decayed to the rest floor, not merely the lookahead-adjusted
    // prediction to have crossed zero -- or when its bounded window
    // expires.
    //
    // The window is not belt-and-braces. Report §3 measured the low-speed
    // corrective pivot as BIMODAL: 26% of 333 nudges delivered under
    // 0.25 deg, i.e. never broke away at all. Those move the residual by
    // nothing, so "has it landed?" never answers yes on its own and the
    // alignment would sit here until the Move's own timeout.
    //
    // settleReached() is deliberately NOT the test: it compares against
    // settleEpsilonAngular, an ARRIVAL tolerance sized to this robot's
    // creep resolution (0.035 rad on tovez) and therefore WIDER than a
    // typical nudge's whole threshold -- it would report every nudge
    // already arrived on its first tick and the pivot would never run.
    const bool profileDone =
        (limits_.landing.settleRestOmega <= 0.0f ||
         std::fabs(profileVelocity_) <= limits_.landing.settleRestOmega) &&
        measured.plannedRemaining <= kDoneEpsilonAngular;
    const bool expired = static_cast<float>(now - align_.nudgeStart) >=
                         align_.nudgeWindow;
    if (!profileDone && !expired) {
      planActive(now, dt, measured);  // the ordinary path, no bypass
      return false;
    }
    align_.nudging = false;
    align_.restTicks = 0;  // this nudge's own re-settle starts from zero
    align_.settleStart = now;
  }

  // SETTLING. Two independent conditions, deliberately kept apart:
  //
  //   DWELL, on the COMMAND. The staged pair must have been at zero for
  //   kAlignRestWindow. This is the noise-immune half -- the command is a
  //   number this class wrote, not a measurement -- and it is what actually
  //   gives the plant time to coast to a physical stop and land a fresh
  //   encoder sample.
  //
  //   BODY QUIET, on the MEASUREMENT, instantaneously. The filtered angular
  //   rate must be inside the robot's own rest floor (settleRestOmega, the
  //   same floor settleReached() and the stall backstop already call
  //   "stopped"). This is the physical confirmation, and it refuses to read
  //   heading off a body that is demonstrably still turning.
  //
  // The dwell counter keys off the command rather than off the conjunction
  // on purpose. Keyed off the measurement, a single noisy sample would
  // restart the whole window, and the two robots' configured floors sit
  // only a little above their measured rest noise (tovez: floor 0.16 rad/s
  // against a ~0.11 rad/s raw noise band) -- close enough that a flicker is
  // ordinary, not exceptional. Restarting there could stall the phase until
  // the Move's own timeout on nothing but noise. As written, a flicker
  // costs one tick.
  const bool commandQuiet = cmdLeft_ == 0.0f && cmdRight_ == 0.0f;
  align_.restTicks = commandQuiet ? align_.restTicks + 1 : 0;
  const bool dwelt = dt > 0.0f && static_cast<float>(align_.restTicks) * dt >=
                                      kAlignRestWindow;
  const bool bodyQuiet =
      std::fabs(measured.omega) <= limits_.landing.settleRestOmega;
  if (!dwelt || !bodyQuiet) {
    // Bounded, so a plant whose measured omega never quiets below its own
    // rest floor cannot hold the Move here forever. The Move `timeout` is
    // the outer backstop, but it is OPTIONAL on the wire (0 = none), and
    // "no timeout" must not mean "may wedge the queue" -- the identical
    // argument plannedStopWindow() makes for a Kind::Stop entry. Giving up
    // completes the Move where it stands, which is exactly what happens
    // today without this phase at all.
    if (static_cast<float>(now - align_.settleStart) >= alignSettleWindow()) {
      return true;
    }
    alignSettleCommand(dt);
    return false;
  }

  // MEASURING. pose_.heading() is unwrapped (estimation.h) and so is the
  // latched target, so this plain difference is the signed residual with no
  // wrapping to get wrong.
  const float residual = align_.target - pose_.heading();  // [rad] signed
  if (std::fabs(residual) <= limits_.landing.alignTol) return true;
  if (align_.nudges >= limits_.landing.alignMaxNudges) return true;

  // STOP IF THE LAST NUDGE MADE IT WORSE. The corrective pivot has a
  // coarse quantum of its own -- report §3 measured a median 1.72 deg
  // delivery against a 1.0 deg tolerance -- so a plant whose nudges
  // consistently over-deliver cannot land inside the tolerance and will
  // instead oscillate across it, spending the whole budget and ending no
  // better than it started. That is the same "a nudge fires at a residual
  // it cannot resolve, and the corner gets WORSE" failure §3 measured when
  // the tolerance was tightened; here it is detected directly, from the
  // mechanism's own delivered result, instead of being assumed away.
  //
  // "Worse", not "no better": a nudge that delivers NOTHING is the
  // measured 26% no-breakaway case, and retrying it is exactly what earns
  // the 94% convergence at 1.3 nudges/corner -- stopping there would throw
  // that away. Only a residual that grew by more than a few measurement
  // quanta (kStallEpsilonAngular, the same "is this change real or is it
  // noise?" threshold the stall backstop uses) counts.
  if (align_.residualBefore >= 0.0f &&
      std::fabs(residual) > align_.residualBefore + kStallEpsilonAngular) {
    return true;
  }

  startAlignNudge(now, residual);
  // Re-measure: startAlignNudge() just rewrote the Move's threshold and
  // baseline, and `measured` was taken against the previous pair.
  planActive(now, dt, measure(now));
  return false;
}

void Planner::startAlignNudge(uint32_t now, float residual) {  // [ms] [rad]
  ++align_.nudges;
  align_.nudging = true;
  align_.nudgeStart = now;
  align_.residualBefore = std::fabs(residual);

  // Re-arm the ACTIVE Move as a bounded low-speed pivot of exactly the
  // residual and let it flow through planActive()/planWheels() like any
  // other Angle Move. That is the point of doing it this way rather than
  // writing cmdLeft_/cmdRight_ directly: the ratio lock, the discrete-exact
  // terminal step, the per-wheel accel ceiling and the decel latch all
  // apply to a nudge exactly as they apply to the corner it is trimming,
  // and there is no second actuation path to keep in sync.
  //
  // Rewriting the Move in place is safe because enterAligning() already
  // latched align_.target: nothing downstream still needs the Move's
  // original threshold or baseline.
  const float magnitude = std::fabs(residual);
  const float omega = alignNudgeOmega();  // [rad/s] magnitude
  Move& move = active_.move;
  move.threshold = magnitude;
  // A PURE pivot, even when the Move being aligned was an arc: v_x/v_y are
  // zeroed so a heading trim can never add path length to a leg it has no
  // business touching. requestedThreshold is deliberately left alone -- the
  // ledger target is already latched, and rewriting it would make the
  // Move's own record of what the caller asked for a lie.
  move.v_x = 0.0f;
  move.v_y = 0.0f;
  move.omega = residual > 0.0f ? omega : -omega;

  active_.baselineHeading = pose_.heading();
  active_.closingIssued = false;
  active_.decelLatched = false;
  active_.phase = MovePhase::Idle;
  active_.stallTicks = 0;
  active_.stallRemaining = 0.0f;
  active_.shape = shapeOf(move, limits_.plant.trackWidth);
  active_.wheelLimits = shapeLimits(active_.shape, limits_);
  if (active_.shape.valid && active_.wheelLimits.aDecel > 0.0f) {
    lastShapeDecel_ = active_.wheelLimits.aDecel;
  }
  // axisPerLambda for a pivot, derived the same way activateNext() derives
  // it: heading advances by the wheel difference over the track.
  active_.axisPerLambda = 1.0f;
  if (active_.shape.valid) {
    active_.axisPerLambda =
        std::fabs(active_.shape.unitRight - active_.shape.unitLeft) /
        limits_.plant.trackWidth;
  }
  if (!(active_.axisPerLambda > 0.0f)) active_.axisPerLambda = 1.0f;

  // Start the profile from rest -- alignStep() has just confirmed the body
  // IS at rest and the staged command IS zero, so there is no carry to
  // preserve and nothing for profileStep() to misread as a coast.
  profileVelocity_ = 0.0f;
  profileAccel_ = 0.0f;
  lastAxis_ = Axis::Angular;

  align_.nudgeWindow = alignNudgeWindow(magnitude, omega);
}

void Planner::alignSettleCommand(float dt) {  // [s]
  rollCommandHistory();
  // Ramp out at the shape's own decel ceiling, the same one drainToZero()
  // uses, so a nudge's terminal step always fits inside exactly one step
  // here (drainToZero()'s own doc comment has the reasoning).
  rampCommandsToZero(drainDecel(dt));
  // profileVelocity_ is in the Move's own axis units -- [rad/s] for an
  // Angle Move -- so it decays against the ANGULAR ceiling, not
  // drainToZero()'s body-frame linear one. That difference is the only
  // reason this is not simply a call to drainToZero().
  profileVelocity_ =
      std::max(0.0f, profileVelocity_ - limits_.ceilings.alphaDecel * dt);
  profileAccel_ = 0.0f;
  activeBoundary_ = 0.0f;
  active_.phase = MovePhase::Decel;
}

float Planner::alignNudgeOmega() const {  // [rad/s]
  return limits_.ceilings.omegaMax * kAlignNudgeOmegaFraction;
}

float Planner::alignNudgeWindow(float threshold,
                                float omega) const {  // [rad] [rad/s] -> [ms]
  // Derived from the pivot itself, not chosen: the time to sweep
  // `threshold` at `omega`, plus the ramp up and the ramp down at the
  // configured angular ceilings, plus one plant-coast allowance. An
  // unconfigured ceiling simply contributes nothing to the sum (the same
  // fail-open convention used elsewhere here) rather than making the
  // window infinite or zero.
  if (!(omega > 0.0f)) return kAlignSettleAllowance;
  float window = 1000.0f * threshold / omega;  // [ms] sweep at cruise
  if (limits_.ceilings.alphaMax > 0.0f) {
    window += 1000.0f * omega / limits_.ceilings.alphaMax;
  }
  if (limits_.ceilings.alphaDecel > 0.0f) {
    window += 1000.0f * omega / limits_.ceilings.alphaDecel;
  }
  return window + kAlignSettleAllowance;
}

float Planner::alignSettleWindow() const {  // [ms]
  // The dwell this phase actually asks for, plus the derived worst case for
  // reaching rest at all -- plannedStopWindow(), which is already "the
  // worst-case decel ramp from vMax plus one settle allowance" and is
  // already the answer to this same question for a Kind::Stop entry. Reused
  // rather than re-derived so the two cannot drift into disagreeing about
  // how long stopping takes on this robot.
  return 1000.0f * kAlignRestWindow + plannedStopWindow();
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
  // 134-003: a Move in its terminal fine-align never hands off at speed.
  // The whole phase is "come to rest, read the heading, trim" -- with a
  // compatible Move already pending, an unguarded lookahead would plan a
  // nonzero landing speed for a corrective nudge and the loop would never
  // reach the rest it has to measure from. It also keeps activeBoundary_
  // pinned at 0 for the duration, which is what alignApplies()' own
  // "not handing off at speed" guard assumes about the state it enters.
  if (lifecycle_ == MoveLifecycle::Aligning) return 0.0f;
  if (pendingCount_ == 0) return 0.0f;
  // Fail closed: never plan a hand-off we have no configured authority to
  // brake out of.
  if (limits_.ceilings.aDecel <= 0.0f) return 0.0f;

  const Move& next = pending_[0];
  const MoveShape nextShape = shapeOf(next, limits_.plant.trackWidth);
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
        ? limits_.ceilings.alphaDecel * 0.5f * limits_.plant.trackWidth
        : limits_.ceilings.aDecel;
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
  const float period = limits_.plant.controlPeriod;  // [ms]

  // M1 terminal-settle creep (DELETED by 130-008): this used to run a
  // closed-loop P creep on the measured residual while a completed Move
  // sat in the deleted `Settling` sub-state, gated on
  // PlannerLimits::requireSettle. Dead code even before this ticket --
  // grep-verified, `active_.settling` was the ONLY thing that ever armed
  // it, and that field is gone -- so deleting it changes no behavior for
  // any caller (requireSettle defaulted false, and no boot config ever
  // turned it on; see planner_types.h's own updated doc comment).

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
    const float accelStep = limits_.ceilings.aMax * dt;
    const float decelStep = limits_.ceilings.aDecel * dt;
    const float vCap = limits_.ceilings.vMax;  // [mm/s] wheel-space ceiling
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
      const float decelStep = limits_.ceilings.aDecel * dt;
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
      const float omegaPrev = (cmdRight_ - cmdLeft_) / limits_.plant.trackWidth;
      const float v = timedRamp(vPrev,
                                std::clamp(m.v_x, -limits_.ceilings.vMax, limits_.ceilings.vMax),
                                sign(m.v_x) * activeBoundary_,
                                limits_.ceilings.aMax * dt, limits_.ceilings.aDecel * dt,
                                ticksLeft);
      const float omega =
          timedRamp(omegaPrev, m.omega, 0.0f, limits_.ceilings.alphaMax * dt,
                    limits_.ceilings.alphaDecel * dt, ticksLeft);
      profileVelocity_ = std::fabs(v);
      const float halfTrack = 0.5f * limits_.plant.trackWidth;
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
  // fall or hold but never rise again -- UNLESS the current tick's own
  // fresh recomputation (`raw` below, from profileStep()'s from-scratch
  // feasibility test, before this block's own override) says the Move is
  // genuinely back in Accel or Hold. 131-006: the latch used to be a pure
  // one-way ratchet -- once Decel/Closing tripped it, EVERY later tick was
  // clamped and forced to Decel for the rest of the Move, regardless of
  // what that later tick's own honest recomputation found. But the
  // Decel/Closing classification that trips the latch is driven by
  // `remaining` (measured.plannedRemaining upstream), a PREDICTION over
  // sample-age and actuationDelay (measure()'s own doc comment) -- not a
  // certainty. A transient under-estimate (e.g. the plant lagging the
  // commanded ramp, so the ZOH predict overstates real progress) could trip
  // the latch on a tick that was never truly braking, and the old
  // unconditional override then forbade recovery even once re-measurement
  // showed real remaining rotation/distance -- directly contradicting
  // profile.cpp's own "brake as hard as allowed and let re-measurement
  // recover" comment at its `!feasible(floor)` branch: profileStep() DOES
  // let re-measurement recover, every tick, on its own; the bug was this
  // caller vetoing that recomputation once latched. Releasing the latch
  // here, rather than adding a new hysteresis/deadband parameter, is the
  // fix: profileStep()'s own from-scratch classification (Accel/Hold means
  // "no, we should not still be braking") IS the "did this materially
  // recover" signal the comment already promises.
  //
  // The never-re-accelerate-once-genuinely-braking guarantee is preserved:
  // the latch still sets on Decel/Closing and still clamps/forces Decel
  // while held, for a Move that really is finishing -- it now ALSO clears
  // the instant a fresh recomputation disagrees, rather than never.
  //
  // The clamp is the BINDING wheel's own previous command translated back
  // to shape space (previousWheel[binding]/magnitude[binding]), the same
  // per-wheel-actual quantity profileStep() was just fed for it -- not the
  // old axisPerLambda-rescaled scalar, for the same reason planWheels()
  // stopped feeding that to profileStep() above.
  StepPhase raw = phase[binding];
  if (active_.decelLatched &&
      (raw == StepPhase::Accel || raw == StepPhase::Hold)) {
    active_.decelLatched = false;
  } else if (active_.decelLatched) {
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
  if (limits_.tracking.headingHoldGain <= 0.0f) return;
  // P on the uncommanded axis, back toward the Move's activation heading.
  const float error = active_.baselineHeading - pose_.heading();  // [rad]
  const float omegaCorrection = limits_.tracking.headingHoldGain * error;  // [rad/s]
  float differential = omegaCorrection * 0.5f * limits_.plant.trackWidth;  // [mm/s]

  // Clamp the CORRECTION, never the profiled velocity: the faster wheel
  // must stay inside vMax, and the mean of the pair -- which is what the
  // odometry integrates as ds, and therefore what the distance accounting
  // depends on -- must come out exactly as profiled.
  const float profiled = 0.5f * (cmdLeft_ + cmdRight_);  // [mm/s]
  const float headroom = std::max(0.0f, limits_.ceilings.vMax - std::fabs(profiled));
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
  profileVelocity_ = std::max(0.0f, profileVelocity_ - limits_.ceilings.aDecel * dt);
}

// The decel ceiling the drain and the planned stop ramp at: the active
// shape's own, when there is one, else the body-frame linear ceiling.
float Planner::drainDecel(float dt) const {  // [s] -> [mm/s] per interval
  if (active_.occupied && active_.shape.valid &&
      active_.wheelLimits.aDecel > 0.0f) {
    return active_.wheelLimits.aDecel * dt;
  }
  if (lastShapeDecel_ > 0.0f) return lastShapeDecel_ * dt;
  return limits_.ceilings.aDecel * dt;
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
  //
  // limits_.plant.controlPeriod (this baked/configured value) and
  // App::Drive's own per-cycle MEASURED state.time.cyclePeriod
  // (drive.cpp's own tick(), Stage B's dt) now agree BY CONSTRUCTION
  // (131-005): App::RobotLoop::cycle()'s trailing pacing block targets an
  // absolute end-of-cycle deadline, so the delivered period converges to
  // App::RobotLoop::kCycle -- the same nominal controlPeriod is baked
  // from (data/robots/*.json's own control_period, now simply "=
  // kCycle") -- rather than the two drifting apart by whatever the
  // pacer's own fixed-gap rounding happened to leak (130-011 measured an
  // 8% gap this way: a rock-stable 54ms delivered vs. a 50ms baked
  // nominal, enough on its own to flip applyHeadingHold() unstable on
  // hardware). This dt is still the BAKED value, not a live read of
  // cyclePeriod -- update() has no access to per-cycle measured timing,
  // only the planner's own configured limits_ -- but the two are no
  // longer an accepted, structural disagreement, just the same number by
  // two different (and now equal) routes.
  const float dt = limits_.plant.controlPeriod * 0.001f;  // [s]
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
  state.command.omega = (stagedRight - stagedLeft) / limits_.plant.trackWidth;

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
      (right_.velocity() - left_.velocity()) / limits_.plant.trackWidth;

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
