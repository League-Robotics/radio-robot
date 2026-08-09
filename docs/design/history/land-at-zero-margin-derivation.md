# Land-at-zero margin derivation (design history)

**Preserved 2026-07-31, sprint 128 ticket 014**, when
`Motion::MoveQueue`/`Motion::WheelSink`/`Motion::StopCondition`/
`Motion::VelocityShaper` were deleted outright as dead code (zero
callers — `Motion::Planner::update()` already writes
`Types::RobotState::Wheel::cmdVelocity` directly and `RobotLoop::cycle()`
already consumes it directly; see SUC-002 / sprint 128's Decision 1).
This document exists solely to preserve the empirical/analytical work
behind the three `kStoppingMarginFactor*` constants and
`kDiscretizationCyclesChain` — the "land-at-zero completion predicate" —
that lived in `src/firm/motion/move_queue.cpp`'s anonymous namespace before
deletion. **None of this code is live.** It is not consumed by
`Motion::Planner` or any other current component; it is kept here purely
as a record of the tuning methodology and the sweep data, in case a
future planner-side completion predicate needs the same kind of
empirical derivation and wants to avoid re-discovering the same pitfalls
(narrow-pocket vs. broad-plateau selection, cadence sensitivity,
same-axis-chain vs. orthogonal-chain vs. final-move boundary kinds).

The text below is copied verbatim from `move_queue.cpp` as it stood
immediately before deletion (original authorship spans tickets 118-004,
118-003, 119-005, and 121-003, as cited inline).

## Verbatim derivation (former `move_queue.cpp` anonymous-namespace comment)

```cpp
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
// THREE values, not one (121 ticket 003, land-at-zero-at-orthogonal-
// chain-boundaries.md: a THIRD value, kStoppingMarginFactorOrthogonal,
// NARROWS the original two-value "is a chain-advance imminent" criterion
// below) -- chosen by which of three boundary kinds this completion is:
//   - a SAME-AXIS COMPATIBLE chain boundary (the incoming pending Move
//     continues the ending axis the SAME direction -- MoveQueue::
//     sameAxisCompatible() below, e.g. two Distance legs both forward)
//     ships kStoppingMarginFactorChain (+ kDiscretizationCyclesChain);
//   - the queue drains to a genuine stop (pendingCount() == 0) ships
//     kStoppingMarginFactorFinal;
//   - an ORTHOGONAL chain boundary (pendingCount() > 0 but the incoming
//     pending Move does NOT continue this axis -- turn->straight,
//     straight->turn) ships kStoppingMarginFactorOrthogonal, a THIRD,
//     independently-swept constant, structurally shaped like the FINAL
//     branch (no discretization term) but NOT numerically equal to it
//     (0.67, not 0.92 -- see "SCOPE NARROWED" below for why reusing
//     kStoppingMarginFactorFinal verbatim measurably fails), landing the
//     ending axis at zero exactly like an unchained Move.
// The ORIGINAL two-value split (pre-121-003, kept verbatim below for the
// sweep history it documents) existed because the two measurement
// conventions this project's own acceptance suites use for "did the turn
// land" disagree about what "coast" even means:
//   - test_tour_closure_gate.py's own per-turn accuracy check reads sim
//     ground truth at the completion-ack INSTANT (TurnCheck, this file's
//     own `_run_tour_capture()`), because a tour leg's own next Move is
//     already queued (SUC-003 one-leg lookahead) and starts driving the
//     SAME cycle -- there is no settle window between legs to coast into,
//     so this reading never sees whatever the real motor/PID does after
//     Drive::stop() would have run (NO chain-advance calls it, same-axis
//     or orthogonal alike -- Drive::stop() is strictly a pendingCount()==0
//     event; only the MARGIN, not the stop()-vs-no-stop() physical
//     regime, changes at an orthogonal boundary).
//   - test_gui_button_acceptance.py's own preset/SEG checks read pose after
//     genuine quiescence (`settle_pose()`, a real quiet-window poll) --
//     because each button press is its OWN Move with nothing queued behind
//     it, the robot actually reaches Drive::stop() and its real velocity-PID
//     coasts the remaining residual speed to zero, and settle_pose()
//     faithfully captures that coast as part of "where the robot ended up."
//
// kStoppingMarginFactorFinal (pendingCount() == 0) was swept ONCE, at
// sim's original 50ms cycle (118 ticket 004), and re-verified UNCHANGED
// after sim/firmware cadence parity landed at 40ms (118 ticket 003 --
// SimHarness::kCycleDtUs equals Core::RobotLoop::kCycle exactly, see
// sim_harness.h's own file header): 0.90-1.10 was a broad, flat plateau
// (worst=0.844deg settle-based at 40ms, against the button-acceptance
// suite's own 3.0deg tolerance), AT THAT SCHEDULE (118's asymmetric
// drive_.tick() staging). 1.00 (mid-plateau) shipped as the default.
//
// RE-SWEPT (119 ticket 005, straight-leg-crab actuation-staging fix): the
// SAME drive_.tick() hoist that fixed the actuation skew (see
// robot_loop.cpp's own comment) also shifts the average commanded-to-duty
// latency -- previously R's own duty write lagged its own freshly-staged
// target by 0 cycles and L's lagged by 1 (asymmetric, averaging 0.5); now
// BOTH lag by 1 (symmetric, but averaging a FULL cycle more than before).
// This is a real, measured, systematic effect on the land-at-zero
// predicate's own timing, independent of the actuation-skew fix's own
// accuracy benefit: the OLD 1.00 value re-measured a genuine -3.267deg/
// +3.178deg UNDERSHOOT on an isolated +/-90deg managed turn (settle-based,
// pendingCount()==0 -- test_gui_button_acceptance.py's own
// test_managed_angle_preset[+-90]/test_managed_seg_0_cdeg_turn[+-90]),
// over their own 3.0deg gate -- caught by re-running the FULL gate set
// this ticket's own acceptance criteria require, not anticipated in the
// original plan. A fresh sweep over kStoppingMarginFactorFinal in
// [0.50, 1.00] (settle-based +/-90deg turn, matching the failing tests'
// own measurement convention) found a genuinely broad plateau at
// [0.88, 0.96] -- worst=0.316deg throughout (identical at every 0.01-0.02
// sample in that range: 0.88/0.89/0.90/0.92/0.93/0.96 all measure the
// SAME 0.316deg, a real plateau, not sampling noise) -- with sharp cliffs
// on both sides (0.87 asymmetric at 2.909deg; 0.97 back to the old
// 3.267deg undershoot). 0.92 (mid-plateau, matching this file's own
// mid-plateau convention for kStoppingMarginFactorChain above) ships as
// the new default, replacing 1.00 -- 2.68deg of margin under the
// button-acceptance suite's own 3.0deg gate. Full sweep data and the
// standalone measurement script referenced in ticket 119-005's own file.
// This confirms Drive::stop()'s own real coast is still governed by the
// motor's own velocity-PID time constants (not a NEW cadence sensitivity
// this ticket introduced) -- see kDiscretizationCyclesChain's own comment
// below for the CONTRASTING chain-advance case, which is NOT
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
// genuinely broad plateau under the tour-closure gate's 2.5deg band, AT
// THAT SCHEDULE (118's asymmetric drive_.tick() staging -- see robot_loop.cpp's
// own history): the achievable worst-case error jumped discontinuously
// (e.g. 2.596deg at chain=0.80 vs 4.474deg at chain=0.81) because different
// turns' own error-vs-coefficient curves crossed zero at slightly different
// points (TOUR_1/TOUR_2 command a genuine variety of angles -- 90/124/146/
// 215/217 degrees, both directions), so ANY single global coefficient's own
// "worst across all turns" envelope was a max over several offset curves,
// not one smooth curve. 0.60 (a narrow pocket, neighbors 0.02-0.03 away
// measuring 3.7-4.5deg) shipped from that search -- escalated to the
// team-lead alongside that commit (118 ticket 003's own exception
// resolution) with the full sweep data.
//
// RE-SWEPT (119 ticket 005, straight-leg-crab actuation-staging fix):
// hoisting drive_.tick() to the top of cycle() (same-generation L/R
// actuation, see robot_loop.cpp's own comment) changes the plant's exact
// per-cycle response, which shifted this narrow pocket -- the OLD 0.60
// value re-measured 3.457deg worst-case at the new schedule (TOUR_2/ideal
// turn 10), over the 2.5deg gate. A fresh 1-D sweep over
// kStoppingMarginFactorChain in [0.20, 1.10] against the SAME tour-closure
// gate (TOUR_1/TOUR_2 x ideal/realistic, worst |turn error| across all 4)
// at THIS schedule:
//   0.20: 4.111deg   0.30: 2.852deg   0.35: 2.852deg   0.38: 2.852deg
//   0.40: 2.357deg   0.42: 2.357deg   0.45: 2.481deg   0.48: 2.218deg
//   0.50: 2.342deg   0.52: 2.521deg   0.55: 2.748deg   0.60: 3.457deg
//   0.65: 6.660deg   0.70: 7.266deg   0.80: 10.294deg  0.90: 12.378deg
//   1.00: 14.255deg  1.10: 15.066deg
// UNLIKE 118-003's own finding, this IS a genuinely broad plateau --
// [0.40, 0.50] holds comfortably under the 2.5deg gate (worst 2.481deg at
// the 0.45 sample; both edges just outside, 0.38 at 2.852deg and 0.52 at
// 2.521deg, are smooth degradation, not a discontinuous cliff). 0.48
// (worst=2.218deg, 0.282deg of margin) ships as the new default, replacing
// 0.60 -- broad-plateau-or-escalate per this project's own convention, and
// this time a genuine plateau, not another narrow-pocket escalation.
// kDiscretizationCyclesChain (below) was NOT re-swept -- the 1-D
// chain-factor sweep alone already found an adequate broad plateau, so the
// 2-D joint sweep 118-003 needed was not necessary here. Full sweep data
// and per-tour/per-profile breakdown recorded in ticket 119-005's own file.
//
// SCOPE NARROWED (121 ticket 003, land-at-zero-at-orthogonal-chain-
// boundaries.md, 2026-07-23 -- stakeholder decision: stop sweeping this
// constant further, split completion semantics by axis relationship
// instead). kStoppingMarginFactorChain/kDiscretizationCyclesChain above
// now govern ONLY a SAME-AXIS COMPATIBLE chain boundary (the incoming
// pending Move continues the ending axis the SAME direction -- e.g. two
// Distance legs, both forward; MoveQueue::sameAxisCompatible(), this
// file's own landAtZero()) -- sprint 122's own deferred velocity-carry
// case, UNCHANGED by this ticket. An ORTHOGONAL chain boundary
// (turn->straight, straight->turn -- the incoming Move does NOT continue
// this axis) now selects a THIRD, dedicated constant,
// kStoppingMarginFactorOrthogonal (below), instead: there is no velocity
// worth carrying across an axis the incoming Move doesn't command
// (stakeholder decision; a beat of corner dwell for exactness is
// accepted), so this branch is structurally the SAME shape as the FINAL
// branch (no discretization term) -- just its own independently-swept
// margin value, not literally kStoppingMarginFactorFinal.
//
// The plan's own default proposal -- reuse kStoppingMarginFactorFinal
// (0.92) verbatim for the orthogonal case -- was VERIFIED against the
// closure gate (test_tour_closure_gate.py, TOUR_1/TOUR_2 x
// ideal/realistic) per this ticket's own mandate, NOT assumed, and
// measurably FAILS: worst |turn error| 8.043deg (ideal) / 7.863deg
// (realistic), against the shaped-band gate's 2.5deg -- TOUR_1/TOUR_2
// alternate Distance/Angle unconditionally (every leg boundary in both
// tours is orthogonal), so reusing 0.92 there is equivalent to reusing
// the settle-based drain margin for an ack-instant, never-settles
// chain-advance -- the SAME mismatch of measurement convention that
// originally justified kStoppingMarginFactorChain's own existence,
// recurring one level down. A dedicated 1-D sweep of a THIRD constant,
// kStoppingMarginFactorOrthogonal, in [0.00, 1.00] (no discretization
// term, matching the FINAL branch's own structural shape) against the
// SAME gate found a genuinely broad plateau at [0.665, 0.674] (both
// ideal AND realistic worst |turn error| held constant across that
// entire sampled range) -- 0.67 (mid-plateau) ships. Full table (worst
// |turn error| across TOUR_1+TOUR_2, ideal | realistic):
//   0.10: 4.052 | 4.517   0.30: 4.052 | 4.517   0.48: 3.032 | 4.111
//   0.60: 2.314 | 2.852   0.655-0.662: 2.314 | 2.487
//   0.665-0.674: 2.314 | 2.100  (the shipped plateau)
//   0.676-0.69:  2.195-2.314 | 2.549   0.70: 2.306 | 6.587*
//   0.80: 3.237 | 3.086   0.92: 8.043 | 7.863
// (*0.70's realistic=6.587 was measured with the discretization term
// ALSO reinstated in an earlier experiment, not the shipped no-
// discretization shape; the no-discretization curve degrades smoothly
// through 0.70-0.80 without that spike -- recorded here for the
// record, not as a live data point.)
//
// HONEST RESIDUAL (ticket 121-003's own "if a residual remains" clause):
// even at the shipped 0.67, the SPRINT's own aspirational targets --
// straight-following-turn gain <=0.3deg, turn |error| <=0.5deg, TOUR_1
// net heading 540+-1deg -- are NOT met. Measured at 0.67 against
// test_tour_closure_gate.py's own worst-case reporting: turn |error|
// 2.314deg (ideal, TOUR_1)/2.100deg (realistic, TOUR_2); straight-leg
// cruise |delta| 4.104deg (ideal, TOUR_2 leg 13)/9.852deg (realistic,
// TOUR_2 leg 9); TOUR_1/ideal net heading closure residual ~+21deg over
// the 540deg commanded (from heading_delta_deg=-158.95deg, wrapped
// against the expected +-180deg point). This is COMPARABLE TO, not
// dramatically better than, the PRE-ticket baseline (chain=0.48 +
// discretization applied uniformly, since 100% of TOUR_1/TOUR_2
// boundaries are orthogonal): turn 2.195/2.218, cruise 4.254/9.307, net
// closure ~+17.9/+34.2 -- i.e. this ticket's margin-only mechanism
// measurably AVOIDS THE DISASTROUS naive-reuse regression (8.043/
// 7.863deg) and keeps every EXISTING hard gate passing with real margin,
// but does NOT deliver the sprint's own hoped-for cruise/closure
// improvement. Root cause (not fitted-constant-fixable): the residual
// omega/v_x that "decays into the next Move" is the REAL PLANT's own
// post-reset momentum (tick()'s own unconditional shaperOmega_.reset()/
// shaperVX_.reset() zeroes the KINEMATIC shaper target instantly, but the
// physical wheel/velocity-PID plant that had been tracking the
// PREVIOUS nonzero target does not stop instantly) -- a SEPARATE
// physical effect from "how much of the taper's own v^2/(2*a) remaining
// distance is left," which is all a marginFactor scale on that formula
// can ever adjust. Confirmed by sweeping the WHOLE [0.00,1.00] range:
// low margins minimize cruise-leak (best measured 2.459-2.682deg
// ideal) but blow the raw-backstop-driven OVERSHOOT up past the 2.5deg
// turn-error gate; high margins minimize net-heading closure (crosses
// through ~0 around 0.85-0.90) but blow turn-error up past 8-14deg via
// a large systematic UNDERSHOOT that a following straight leg's own
// compensating overshoot happens to cancel -- a two-wrongs-cancel
// artifact, not a genuine fix, and rejected on that basis. Closing this
// residual properly needs the issue's own analytic
// `remaining <= |omega_measured| * (kCycle/2 + tauPlant)` form using an
// ACTUALLY MEASURED velocity (not this predicate's own kinematic `cmd`)
// and an independently-characterized `tauPlant` (the real plant's own
// settling time constant, measured via an isolated step-response
// characterization, NOT fitted against this same closure gate) -- both
// are new capability beyond this ticket's authorized scope (landAtZero()
// has no measured-velocity input today) and are flagged here for a
// follow-up ticket rather than rushed under time pressure into a second
// fitted constant.
constexpr float kStoppingMarginFactorChain = 0.48f;  // dimensionless
constexpr float kStoppingMarginFactorFinal = 0.92f;  // dimensionless
constexpr float kStoppingMarginFactorOrthogonal = 0.67f;  // dimensionless

// kDiscretizationCyclesChain -- SAME-AXIS-COMPATIBLE-CHAIN-ONLY (see
// landAtZero()'s own use: gated on sameAxisCompatible(pending_[0]) &&
// pendingCount() > 0, matching kStoppingMarginFactorChain's own
// narrowed scope above -- 121 ticket 003). [cycles] per-cycle
// discretization allowance: epsilonRemaining also grows by
// |commandedSpeed| * dt * kDiscretizationCyclesChain, budgeting how far
// the axis can travel in roughly this many MORE control cycles at the
// current rate before the next decision point -- the physically-motivated
// term the 40ms re-sweep above tested per the team-lead's own suggestion.
// dt is this Move's own actual elapsed time since its last shaped tick
// (tick()'s own local computation, the SAME baseline shapeAndStage() uses)
// -- not a compile-time cadence constant -- so the term is honest about
// real (possibly jittered) cycle timing and transfers unchanged to any
// control period, including hardware's. Deliberately NOT applied to the
// final-move case OR an orthogonal chain boundary (kStoppingMarginFactorFinal's
// own comment above, and kStoppingMarginFactorOrthogonal's own sweep
// paragraph above -- re-adding this term to the orthogonal branch was
// tried during that sweep and made every candidate margin WORSE, not
// better, see that paragraph): the final-move regime's own plateau was
// already broad and cadence-robust without it: adding it there only
// shrank real margin for no benefit (measured regression:
// test_managed_angle_preset[-90] went from a clean pass to a 3.07deg
// miss against its 3.0deg tolerance when this term was applied
// unconditionally).
constexpr float kDiscretizationCyclesChain = 0.53f;  // [cycles]
```

## Verbatim application (former `MoveQueue::landAtZero()`)

How the derivation above was actually consumed, per cycle, to decide
whether a `Move` had reached its land-at-zero completion point:

```cpp
// landAtZero -- see move_queue.h's own tick() doc comment for the full
// contract. TWIST moves only: a WHEELS Move's own linearly-shaped axes
// (v_left/v_right) have no stop_kind-matched pairing the way a TWIST
// Move's v_x/omega do (shapeAndStage()'s own per-kind breakdown above), so
// ticket 004's scope -- TWIST Angle/Distance stops only -- excludes WHEELS
// structurally, via the velocityKind check below, not via a second
// remaining/epsilon derivation for wheel-space axes.
bool MoveQueue::landAtZero(float pathLength, float theta, float dt) const {
  if (active_.velocityKind != msg::Move::VelocityKind::TWIST) return false;

  bool sameAxisChainBoundary = pendingCount_ > 0 && sameAxisCompatible(pending_[0]);
  bool orthogonalChainBoundary = pendingCount_ > 0 && !sameAxisChainBoundary;
  float marginFactor = sameAxisChainBoundary
                            ? kStoppingMarginFactorChain
                            : (orthogonalChainBoundary ? kStoppingMarginFactorOrthogonal
                                                        : kStoppingMarginFactorFinal);
  float discretizationCycles = sameAxisChainBoundary ? kDiscretizationCyclesChain : 0.0f;

  if (active_.kind == Motion::StopCondition::Kind::Distance) {
    bool linearShaping =
        shaperLimits_.aMax > 0.0f && shaperLimits_.aDecel > 0.0f && shaperLimits_.jMax > 0.0f;
    if (!linearShaping) return false;  // no taper -- the backstop is the only completion path
    float remaining = active_.threshold - std::fabs(pathLength - active_.activationPathLength);
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
```

## Status as of deletion

This predicate and its constants were dead code at deletion time —
`Motion::MoveQueue` had zero callers (`Motion::Planner::update()` writes
`Types::RobotState::Wheel::cmdVelocity` directly, bypassing this whole
generation path). The "HONEST RESIDUAL" section above should be read as
a candid account of where the tuning effort landed, not as a claim that
the mechanism was ever fully satisfactory — a future planner-side
completion predicate inheriting this problem should expect to need the
measured-velocity/characterized-`tauPlant` extension flagged there, not
just a fourth fitted constant.
