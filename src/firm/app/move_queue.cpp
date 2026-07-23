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
//
// kStoppingMarginFactorFinal (pendingCount() == 0) was swept ONCE, at
// sim's original 50ms cycle (118 ticket 004), and re-verified UNCHANGED
// here after sim/firmware cadence parity landed at 40ms (118 ticket 003 --
// SimHarness::kCycleDtUs now equals App::RobotLoop::kCycle exactly, see
// sim_harness.h's own file header): 0.90-1.10 remains a broad, flat
// plateau (worst=0.844deg settle-based at 40ms, against the button-
// acceptance suite's own 3.0deg tolerance -- BETTER than the 50ms
// measurement, not worse). 1.00 (mid-plateau) ships as the default. This
// confirms Drive::stop()'s own real coast is genuinely cadence-independent
// (governed by the motor's own velocity-PID time constants, not by how
// often MoveQueue samples it) -- see kDiscretizationCyclesChain's own
// comment below for the CONTRASTING chain-advance case, which is NOT
// cadence-independent.
//
// kStoppingMarginFactorChain (pendingCount() > 0) is NOT cadence-
// independent, and required real rework at 40ms (118 ticket 003
// resolution, root-caused via move_queue.cpp's own printf-instrumented
// trace and a standalone Motion::VelocityShaper harness comparing dt=
// 0.050s against dt=0.040s): the 50ms value (0.83, "a broad, flat
// plateau 0.82-0.84... worst=2.398deg") measured 4.47-6.28deg at the true
// 40ms cadence -- a real regression, not measurement noise (confirmed by
// A/B-reverting the UNRELATED NezhaMotor write-throttle jitter margin
// ticket 003 also landed this same commit; byte-identical failure with
// or without it, isolating the cadence change itself as the cause). Root
// cause: EVERY tour leg alternates Distance/Angle (TOUR_1/TOUR_2 in
// planner/tour.py, "D ... / RT ..." pairs) -- a chain-advance turn always
// hands off to a Move on the OTHER axis, so `tick()`'s own reset-on-
// completion (below) always zeroes the shared axis's shaper state to a
// hard 0 at the handoff instant. This is a genuine commanded STEP (not a
// smooth taper-to-zero), and the REAL plant coasts some residual angle
// afterward exactly as it does after Drive::stop() -- but UNLIKE the
// final-move case, this coast is only PARTIALLY visible to the ack-instant
// reading (the next leg's own motion continues immediately, so how much
// of the coast lands "during" this leg vs bleeds into the next one is
// itself a function of exactly which tick the step happens on) -- making
// the achieved reading sensitive to per-cycle quantization in a way the
// final-move case is not.
//
// An extensive re-sweep at 40ms (~90 builds: kStoppingMarginFactorChain
// alone over [0.20, 1.10]; jointly with a per-cycle discretization term,
// see kDiscretizationCyclesChain below, over a 2-D grid; and a structural
// variant that made the reset-on-completion conditional on pendingCount()
// -- see tick()'s own comment for why that variant was NOT kept) found NO
// genuinely broad plateau under the tour-closure gate's 2.5deg band: the
// achievable worst-case error jumps discontinuously (e.g. 2.596deg at
// chain=0.80 vs 4.474deg at chain=0.81) because different turns' own
// error-vs-coefficient curves cross zero at slightly different points
// (TOUR_1/TOUR_2 command a genuine variety of angles -- 90/124/146/
// 215/217 degrees, both directions), so ANY single global coefficient's
// own "worst across all turns" envelope is a max over several offset
// curves, not one smooth curve. The values shipped here (0.60 chain
// factor + a 0.53-cycle discretization term, see below) are the BEST
// point found in that search -- worst=2.323deg at 40ms, verified passing
// -- but this is honestly reported as a narrow pocket (neighbors 0.02-0.03
// away measure 3.7-4.5deg), not the broad plateau this project's own
// convention otherwise requires. Escalated to the team-lead alongside
// this commit (118 ticket 003's own exception resolution) with the full
// sweep data; revisit if a genuinely robust fix (e.g. sub-tick crossing
// interpolation, rather than a per-cycle-sampled threshold) is ever
// invested in.
constexpr float kStoppingMarginFactorChain = 0.60f;  // dimensionless
constexpr float kStoppingMarginFactorFinal = 1.00f;  // dimensionless

// kDiscretizationCyclesChain -- CHAIN-ONLY (see landAtZero()'s own use:
// gated on pendingCount() > 0, matching kStoppingMarginFactorChain).
// [cycles] per-cycle discretization allowance: epsilonRemaining also grows
// by |commandedSpeed| * dt * kDiscretizationCyclesChain, budgeting how far
// the axis can travel in roughly this many MORE control cycles at the
// current rate before the next decision point -- the physically-motivated
// term the 40ms re-sweep above tested per the team-lead's own suggestion.
// dt is this Move's own actual elapsed time since its last shaped tick
// (tick()'s own local computation, the SAME baseline shapeAndStage() uses)
// -- not a compile-time cadence constant -- so the term is honest about
// real (possibly jittered) cycle timing and transfers unchanged to any
// control period, including hardware's. Deliberately NOT applied to the
// final-move case (kStoppingMarginFactorFinal's own comment above): that
// regime's plateau was already broad and cadence-robust without it: adding
// it there only shrank real margin for no benefit (measured regression:
// test_managed_angle_preset[-90] went from a clean pass to a 3.07deg miss
// against its 3.0deg tolerance when this term was applied unconditionally).
constexpr float kDiscretizationCyclesChain = 0.53f;  // [cycles]

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
bool MoveQueue::landAtZero(float pathLength, float theta, float dt) const {
  if (active_.velocityKind != msg::Move::VelocityKind::TWIST) return false;

  // See this file's own anonymous-namespace comment (kStoppingMarginFactorChain/
  // kStoppingMarginFactorFinal/kDiscretizationCyclesChain) for the full
  // derivation of why this predicate needs two different margins: whether
  // THIS completion hands off to an already-queued Move (pendingCount() >
  // 0, chain-advance -- Drive::stop() never runs, so only the ack-instant
  // decision matters) or drains the queue to a genuine stop (pendingCount()
  // == 0 -- Drive::stop() runs for real and the plant's own residual speed
  // coasts further before rest).
  float marginFactor =
      pendingCount_ > 0 ? kStoppingMarginFactorChain : kStoppingMarginFactorFinal;
  // The per-cycle discretization allowance (118 ticket 003 resolution) is
  // CHAIN-ONLY -- see the anonymous-namespace comment for why: the
  // final-move regime's own kStoppingMarginFactorFinal=1.00 plateau was
  // already broad and verified robust (worst=1.189deg settle-based)
  // without it; adding it there too pushed that ALREADY-solved case's own
  // firing point earlier for no benefit, costing real margin instead
  // (measured regression: test_managed_angle_preset[-90] went from
  // comfortably passing to a 3.07deg miss against its 3.0deg tolerance).
  float discretizationCycles = pendingCount_ > 0 ? kDiscretizationCyclesChain : 0.0f;

  if (active_.kind == Motion::StopCondition::Kind::Distance) {
    bool linearShaping =
        shaperLimits_.aMax > 0.0f && shaperLimits_.aDecel > 0.0f && shaperLimits_.jMax > 0.0f;
    if (!linearShaping) return false;  // no taper -- the backstop is the only completion path
    float remaining = active_.threshold - std::fabs(pathLength - active_.activationPathLength);
    // "Have we already entered our own braking envelope for our CURRENT
    // commanded speed" PLUS a per-cycle discretization allowance -- see the
    // anonymous-namespace comment for kDiscretizationCyclesChain.
    float cmd = shaperVX_.commandedSpeed();
    float epsilonRemaining =
        (cmd * cmd) / (2.0f * shaperLimits_.aDecel) * marginFactor +
        std::fabs(cmd) * dt * discretizationCycles;
    return remaining <= epsilonRemaining;
  }

  if (active_.kind == Motion::StopCondition::Kind::Angle) {
    bool angularShaping = shaperLimits_.alphaMax > 0.0f && shaperLimits_.alphaDecel > 0.0f &&
                          shaperLimits_.yawJerkMax > 0.0f;
    if (!angularShaping) return false;
    float remaining = active_.threshold - std::fabs(theta - active_.activationTheta);
    float cmd = shaperOmega_.commandedSpeed();
    float epsilonRemaining =
        (cmd * cmd) / (2.0f * shaperLimits_.alphaDecel) * marginFactor +
        std::fabs(cmd) * dt * discretizationCycles;
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

  // dt since this Move's own last shaped tick -- the SAME quantity
  // shapeAndStage() computes below (from the same lastShapeNow_ baseline),
  // read here first (read-only, no mutation) so landAtZero() can fold in
  // its own per-cycle discretization term (118 ticket 003 resolution --
  // see landAtZero()'s own doc comment). shapeAndStage() recomputes and
  // mutates lastShapeNow_ itself on the Continue path below; duplicating
  // the read here is cheaper and clearer than threading a second output
  // parameter back out of shapeAndStage().
  float dt = static_cast<float>(now - lastShapeNow_) / 1.0e6f;  // [us] -> [s]
  if (dt < 0.0f) dt = 0.0f;  // clock-monotonicity defense, same posture as StopCondition's own

  // Backstop (threshold/timeout) is always-armed and evaluated first --
  // "first to fire wins" (move_queue.h's own tick() doc comment). Only
  // when it does NOT already end the Move this cycle (Continue) is the
  // land-at-zero completion path (118 ticket 004) checked as an
  // ADDITIONAL way for the Move to end -- treated identically to
  // StopConditionMet (never TimedOut): the taper decided this Move is
  // done, not the timeout backstop.
  Motion::StopCondition::Outcome outcome = sc.tick(now, pathLength, theta);
  if (outcome == Motion::StopCondition::Outcome::Continue && landAtZero(pathLength, theta, dt)) {
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
  //
  // 118 ticket 003 resolution: tested making this reset conditional on
  // pendingCount() == 0 (skip it on a chain-advance, letting the next
  // Move's own accel-ramp-toward-cruise decay any residual naturally
  // instead of a hard step to 0) on the theory that TOUR_1/TOUR_2's own
  // alternating Distance/Angle leg structure gives a same-axis Move
  // several seconds to decay before its axis is reused anyway, so the
  // corruption this reset guards against couldn't occur in THOSE tours
  // regardless. Measured against the 40ms closure gate: no improvement
  // (still no broad plateau across a re-swept kStoppingMarginFactorChain,
  // best worst-case 2.932deg, itself just as fragile) -- reverted. Kept
  // unconditional, since it is the more conservative, generally-correct
  // choice and the conditional variant bought nothing.
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
