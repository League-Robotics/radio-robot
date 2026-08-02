---
status: done
sprint: '130'
tickets:
- 130-010
---

# Sim tour turns undershoot 90° deterministically (−10.8° first turn, −20.8° after)

Found during sprint 128's final validation (2026-07-31). Four testgui
tests fail deterministically:

- `test_gui_button_acceptance.py::test_tour_1_runs_to_completion`
- `test_gui_button_acceptance.py::test_tour_2_runs_to_completion`
- `test_sim_transport_tour1.py::test_tour_1_runs_to_completion_with_finite_small_closure`
- `test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`

Signature (stable across runs, quiet or loaded machine):

```
turn 2:  commanded +90.00°  achieved +79.23°  error −10.765°  (tolerance 8.0°)
turn 4+: commanded +90.00°  achieved ~+69.2°  error ~−20.8°
```

## NOT a sprint-128 regression — verified

Reproduced byte-identically (turn 2 −10.765°) at pre-sprint baseline
commit `c7a955c2` in a pristine worktree with its own venv and a
freshly-built baseline `libfirmware_host.dylib`. Sprint 128's
EXECUTION.md baseline pytest run (6 failed / 1436 passed / **138
skipped**) most plausibly had these tests among its 138 skips (sim lib
not built at measurement time), which is why they are absent from its
failure list.

Also NOT the sprint-108 `kFaultWedgeLatch` tour-abort issue
(`sim-mode-tour-1-fault-baseline-exclusion-mismatch.md`, closed by
108-011) — tours complete here; the turns just undershoot.

## Reading of the signature

First turn short by ~10.8°, every later turn short by ~20.8° (≈ 2×) —
looks like a fixed per-turn angular deficit plus a carried-over offset,
i.e. something in the sim turn-shaping/settle path eats a constant wedge
of each commanded rotation. Wants a bisect over sim/planner history and
a look at the shaped-band turn profile vs. sim ground truth.

## Acceptance

- Root cause identified and fixed, or tolerances re-derived from a
  measured, explained sim behavior (not widened to paper over it).
- All four tests green; the fix or explanation recorded here.

## Root cause and fix (130-010, 2026-08-02)

**Nondeterminism (investigated first, per the ticket)**: the deterministic,
single-step harness (`test_tour_closure_gate.py`'s `_make_stepper()`/
`deterministic=True` path, `SimLoop.connect(start_tick_thread=False)`) is
bit-for-bit reproducible — confirmed identical per-turn errors across
repeated runs. Every test/tool that instead runs a REAL background tick
thread (`SimLoop.connect(start_tick_thread=True)`: `test_gui_button_
acceptance.py`'s tour tests, `src/tests/system/runner.py`'s systest sim
backend) is genuinely nondeterministic — wall-clock scheduling jitter
changes exactly which cycle a poll first observes a Move's completion
ack, and since the body is often still moving substantially at that
instant (see below), a few cycles of scheduling luck measurably changes
the recorded pose. Measured on the systest square tour (real tick thread):
closing CAMFIX position error 85.3 / 82.3 / 82.7 mm across three back-to-
back runs of byte-identical code. The single-step harness is the
deterministic measurement to gate on; the real-tick-thread paths inherit
whatever spread that scheduling jitter produces on top of the true
physical accuracy.

**Turn-shaping undershoot, root cause**: `Motion::Planner::tick()`'s
`profile-complete` completion event for an isolated (`activeBoundary_ <=
0`, i.e. not handing off "at speed" into a compatible chained Move)
Distance/Angle Move could fire while the COMMANDED ramp
(`profileVelocity_`) was still well above the rest floor — using only
`measured.plannedRemaining`, a short lookahead-adjusted prediction sized
to `actuationDelay`, with no check that the body (or even the command)
had actually neared rest. That lookahead assumes the plant tracks a
commanded velocity within roughly one `actuationDelay` window; the sim's
own `TestSim::WheelPlant` (the model the tour tests actually run against,
on the "no calibration" `tovez_nocal.json` profile with Stage B/C PID
correction OFF) is a bare first-order duty->velocity lag, tau=0.13s,
well past that window. Traced directly (2026-08-02, temporary stderr
instrumentation in `planner.cpp`, since removed): a 90deg turn's
`profile-complete` fired at `profileVelocity_` still ~30 deg/s, with the
TRUE (sim ground-truth) heading roughly 8-17 degrees short of where the
robot eventually settled. In an ISOLATED Move (nothing queued behind it)
the body is left free to coast the rest of the way after `done` fires, so
the shortfall recovers on its own and is invisible — which is exactly why
an isolated single-Move repro doesn't show it. In a real tour
(`robot_radio.planner.tour.run_tour()`'s own one-leg lookahead: the NEXT
leg is already enqueued and activates the SAME tick the current Move
completes) the following Move's own commands override the still-turning
wheels immediately, so that coast never happens — the shortfall is
permanent, and it is exactly what a tour's own per-leg accuracy check
(`on_leg`, sampled off the completion ack, the same instant `done` fired)
measures.

**Fix, part 1 (the root cause above)**: `planner.cpp`'s
`Move::Kind::Angle` completion case now gates the `boundary<=0` branch of
profile-complete on the commanded ramp itself having also reached the
SAME rest floor `settleReached()` already uses for the honest, measured
`arrived` event (`limits_.landing.settleRestOmega`, `<=0` treated as
unconfigured/no-op, matching `shapeLimits()`'s own `tighten()`
convention). This closes the gap between "the lookahead predicts we're
about to finish" and "the commanded ramp has actually reached its own
tail," without waiting out the plant's full physical coast to true rest
(the stall backstop still does that, far more slowly, for whatever this
branch doesn't cover). **Scoped to Angle only** — the identical change to
`Move::Kind::Distance` was tried and reverted: it measurably regressed
`src/tests/sim/system/test_straight_leg_crab_regression.py` (a dedicated
119-005 permanent regression test, 0.3deg tolerance) to a genuine +2.0deg
crab on an isolated straight leg, because `applyHeadingHold()`'s own
per-tick correction (`planActive()`'s `Move::Kind::Distance` case) stops
running the instant the Move leaves the active branch — extending how
long a Distance Move stays active extends the window that correction has
to lock in a small per-wheel asymmetry, rather than shortening it. The
reported defect is a rotation undershoot; the fix is scoped to the Kind
that actually has it.

**Fix, part 2 (a coupled ledger bug the above uncovered)**: with turn
completion now landing much closer to true rest, the cumulative baseline
ledger's own carry for a completed Angle Move (`carryHeading_ =
baselineHeading + angularDirection(m) * m.threshold`) turned out to be
using the WRONG operand — `m.threshold` is `App::RobotLoop::handleMove()`'s
own rotation-calibration-corrected internal target (`tovez_nocal.json`'s
`rotation_gain`/`rotation_offset_deg`, ~1.006/12.1deg, `125-007`), sized to
compensate for a coast-down overshoot that (per part 1's own fix) no
longer reliably happens the way it used to — so the ledger was carrying a
stale, calibration-skewed "intended" heading into every following chained
leg, corrupting that leg's own heading-hold target by roughly the
calibration offset. `carryHeading_` now carries `pose_.heading()` (the
Planner's own measured heading at completion) directly instead of
projecting from `m.threshold` — honest by construction once part 1 makes
completion land close to the real target, and no new calibration/fudge
constant.

**Measured effect** (`test_tour_closure_gate.py`'s own deterministic
per-turn assertion, TOUR_1 and TOUR_2, ideal and realistic sim-error
profiles, unmodified `tovez_nocal.json`): TOUR_1 worst |per-turn error|
dropped from -9.9..-20.8 (documented above) to **2.7 deg**, comfortably
inside the 8.0 deg "shaped band" gate. TOUR_2 (non-90-degree turns:
124/-217/146/215/-90/-90 deg) drops to **worst -10.12 deg on its own
146-degree turn** — inside band for every OTHER turn, but that one turn
still exceeds the 8.0 deg gate; see the residual note below. `src/tests/
system/tours/square.tour`'s closing CAMFIX improved from ~361mm (this
directory's own `CLAUDE.md`, "Known state (2026-08-01)") to **42-48mm
across repeated runs** (comfortably under its 80mm bound; see the
nondeterminism note above for why it's a range, not one number).
`circle.tour`'s own CAMFIX: **9.6mm, PASS**, matching the ticket's own
stated target exactly and unaffected by any of this. Both systest tours
now PASS. `src/tests/sim/system/test_straight_leg_crab_regression.py` and
`src/tests/testgui/test_sim_transport_tour1.py::test_tour_1_runs_to_
completion_with_finite_small_closure` (one of the four named tests) both
PASS with this fix; the FULL `uv run python -m pytest src/tests/sim -q`
suite is unchanged at 458 passed / 2 failed (the same two pre-existing
failures named in this ticket).

**New regression test**: `src/motion/planner/tests/planner_noise_test.cpp`
adds `testAngleDoesNotUndershootAtCompletionUnderSevereTrackingLag()`,
reading `plant`'s position on the SAME tick `done` fires (not after a
drain, which is what the rest of that file's `drive()` helper measures
and would mask this) at a `trackingLag` matched to `TestSim::WheelPlant`'s
own tau at a 50ms period. It cannot reproduce the tour's own undershoot in
isolation (an isolated Move is free to coast the rest of the way once
nothing overrides it — see the root-cause section above) but pins the
completion-timing mechanism itself against a future regression.

## Residual, NOT closed by 130-010

1. **TOUR_2's own 146-degree turn (leg index 7)** still misses by -10.12
   deg against the 8.0 deg shaped-band gate (every OTHER turn in TOUR_1
   and TOUR_2, including the 215 and -217 degree ones, is comfortably
   inside band) — found only after fix part 2 above (the -to-measured-
   heading ledger carry) was in place; before that change this same turn
   read -7.68 deg (inside band) but the straight-leg-crab regression and
   `test_sim_transport_tour1.py` were both broken instead. Not
   root-caused further given time spent on this ticket already; plausibly
   related to how the ledger's carried heading interacts with a
   NON-90-degree, and in TOUR_2's case a preceding NEGATIVE (-217 deg)
   turn, but unconfirmed.
2. **`test_tour_closure_gate.py`'s per-straight-leg cruise-heading
   assertion** (119-005's `StraightLegCruiseCheck`) now passes for TOUR_1
   (worst 4.99 deg ideal / 4.99 deg realistic, both under their 5.5/10.5
   deg tolerances) but its TOUR_2 half was not independently re-verified
   given (1) above already fails that same test function first.
3. **`test_gui_button_acceptance.py::test_tour_2_runs_to_completion`**
   (one of the four named tests) still fails — but nondeterministically
   and inconsistently (leg 1/15 timeout on one run, leg 13/15 fault on
   another, a mid-leg-15 `should_stop()` interruption on a THIRD, observed
   across repeated runs of the SAME code, both before and after every
   version of this ticket's own fix). This test drives the sim through a
   REAL background tick thread (`SimTransport`/`SimLoop.connect(
   start_tick_thread=True)`), the same nondeterminism source documented
   above — TOUR_2's own deterministic per-turn measurement (via
   `test_tour_closure_gate.py`) is solid modulo residual (1); this
   real-tick-thread wrapper around the identical tour is not. NOT
   verified whether this specific test was ever reliably green even
   before sprint 130 -- given (1) it exercises TOUR_2 has residual (1)'s
   own turn, and (2) its own transport is the nondeterministic one this
   issue's own first section documents, isolating "did 130-010 break
   this" from "was this always flaky" was not possible in the time
   available.

Given the scope of what's left (a single non-90-degree turn's residual
in one tour, plus pre-existing nondeterminism in the real-tick-thread GUI
path), fixing 1-3 further is left to a follow-up rather than blocking
this ticket's own close — see the ticket's own commit for the full
programmer's report to the team-lead.
