---
id: '006'
title: Release decelLatched on re-measured rise + large-angle/chained/transient planner_tests
status: done
use-cases:
- SUC-131-006
depends-on:
- '005'
github-issue: ''
issue: A-tour2-146-degree-turn-still-undershoots-after-130-010.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Release decelLatched on re-measured rise + large-angle/chained/transient planner_tests

## Description

Ticket 130-010 fixed the general turn-shaping undershoot (worst per-turn
error -20.8 deg -> 2.72 deg), but TOUR_2's 146-degree turn still misses by
-10.12 deg, and the residual does NOT scale with angle — suggesting a
second, additive mechanism rather than more of the one 130-010 fixed.

Leading hypothesis (confirmed by direct read of `planner.cpp:1232-1240`):
`active_.decelLatched` is a ONE-WAY trap. Once any tick's `profileStep()`
returns Decel/Closing, the latch clamps every later tick's lambda to
non-increasing and forcibly overrides the phase to Decel — permanently,
for the rest of the Move, regardless of what a LATER tick's own fresh
recomputation says. But the Closing/Decel classification that sets the
latch is driven by `plannedRemaining`, a PREDICTION over sample-age +
actuationDelay (`planner.cpp:756-824`), not a certainty. One transient
under-estimate trips the latch; the latch then forbids recovery even when
re-measurement shows real remaining rotation — directly contradicting
`profile.cpp:102-104`'s own comment, "let re-measurement recover." The
0.5s stall backstop then completes the Move wherever it parked,
`settled=false`. This is angle-independent and additive, matching the
TOUR_2 residual's signature exactly.

Fix: let `active_.decelLatched` clear when the CURRENT tick's own
freshly-recomputed `raw` phase (`phase[binding]`, computed fresh every
tick from the current re-measured `remaining` — before the latch's
override) is `StepPhase::Accel` or `StepPhase::Hold`. `profileStep()`
already recomputes feasibility from scratch every tick; its own
classification IS the "did this materially recover" signal the comment
already promises. The bug is that the outer latch in `planner.cpp` vetoes
that recomputation unconditionally once tripped — removing the veto (not
adding a new hysteresis parameter) is the fix. Keep the existing
never-re-accelerate-once-genuinely-braking guarantee for a Move that is
truly finishing: the latch still sets on Decel/Closing and still clamps
while it holds; it now ALSO clears when the fresh recomputation
disagrees.

In the same ticket, add the missing `planner_tests` coverage that let
this residual exist unnoticed: no Angle scenario above 90 degrees exists
today, nothing exercises a chained leg -> large-turn -> leg sequence under
lag, and nothing exercises a transient misprediction against the latch
directly.

## Acceptance Criteria

- [x] `active_.decelLatched` clears when the current tick's own
      recomputed `raw` (from `phase[binding]`, before the latch's
      override) is `StepPhase::Accel` or `StepPhase::Hold`; it remains
      set (continuing to clamp/force Decel) otherwise — the
      never-re-accelerate-once-genuinely-braking guarantee is preserved
      for a Move that is truly finishing.
- [x] New `planner_tests` scenario: an Angle Move of 146 degrees with
      `PerfectPlant` completes exact, at the same tolerance as the
      existing 90-degree (`testAngleExact()`) test.
- [x] New `planner_tests` scenario: the same 146-degree Angle Move under
      `NoisyPlant` (lag/gain/quantization) reproduces a transient
      misprediction that would have tripped the pre-fix latch, and
      confirms the Move recovers (completes via a genuine
      profile-complete/settle event, not the 0.5s stall backstop) rather
      than completing short.
- [x] New `planner_tests` scenario: a chained leg -> 146-degree-turn ->
      leg sequence under `NoisyPlant` lag completes each segment within
      the existing per-axis exactness tolerance.
- [ ] Sim-tier: TOUR_2's 146-degree turn lands within the same error band
      as its 90-degree turns (single-digit degrees, matching the ~2.72
      deg worst-case already achieved for other turns), measured via the
      SINGLE-STEP harness (`test_tour_closure_gate.py`), NOT the
      background-tick-thread harness
      (`test_gui_button_acceptance.py` — mixes ~30 mm of harness-induced
      spread into a ~10-degree signal). `test_tour_2_runs_to_completion`
      passes on at least 5 repeated runs, not once.
      **NOT MET — see "Fingerprint check" and "Residual" in Completion
      Notes below.** Measured bit-for-bit IDENTICAL per-turn numbers on
      `test_tour_closure_gate.py` before and after this ticket's fix, on
      both tours and both error profiles. Instrumented every single
      `planWheels()` call across all four tour/profile combinations
      (2351 ticks, both Distance and Angle Kind moves): the release
      condition's own trigger (`raw` recomputing `Accel`/`Hold` while
      `decelLatched` is held) never once fires. The mechanism this ticket
      targets is real and demonstrated (see `planner_tests` below), but
      is NOT what is producing the current shaped-band gate failures —
      see Completion Notes for the actual observed mechanism.
- [ ] No regression: TOUR_1 worst per-turn error stays <= 2.72 deg,
      `square.tour` stays <= 80 mm (41.7/43.0 mm today), `circle.tour`
      stays at 9.6 mm PASS, full sim suite stays at 460 passed / 0 failed
      (or the baseline at ticket start, restated if it has moved).
      **Baseline restated, per the escape clause.** The 2.72 deg figure
      is stale (130-010's own measurement); the sprint's own ticket
      dispatch already documents TOUR_1 turn 4 sitting at -13.080 deg,
      unmoved across all five prior 131 tickets — i.e. since before this
      sprint touched anything. The ACTUAL ticket-start baseline
      (independently re-measured before making any change) is worst
      -24.699 deg (TOUR_1 turn 12). This ticket's fix reproduces that
      SAME baseline exactly, bit-for-bit — no regression against the
      real ticket-start state. `square.tour`/`circle.tour`/the full sim
      suite were not independently re-run (out of this ticket's given
      verification scope — planner_tests + the turn gate + the new
      targeted tests only; see Completion Notes).
- [x] If the new test scenarios reveal the release condition needs a
      deadband/hysteresis beyond a bare Accel/Hold check (e.g. chattering
      near the boundary), that finding is recorded in the ticket's own
      completion notes as an explicit, justified design refinement — not
      silently patched without explanation, and not left unaddressed if
      it causes a measurable regression.
      No chattering observed: `testAngle146DegreesRecoversFromTransient
      DecelLatchTrip` shows one clean latch/release cycle (Hold -> Decel
      -> Accel -> Decel -> Closing) with no oscillation between latched
      and released states. A bare Accel/Hold check is sufficient; no
      hysteresis/deadband was needed.

## Testing

- **Existing tests to run**: full `planner_tests` standalone build,
  `test_tour_closure_gate.py` (single-step harness), TOUR_1/square/circle
  sim-tier gates, full `src/tests/sim` suite.
- **New tests to write**:
  - `planner_tests`: 146-degree Angle exactness (`PerfectPlant`).
  - `planner_tests`: 146-degree Angle under `NoisyPlant` lag, transient
    misprediction vs. latch recovery.
  - `planner_tests`: chained leg -> 146-degree-turn -> leg under
    `NoisyPlant` lag.
- **Verification command**: the standalone, Python-free `planner_tests`
  build's own runner; `uv run python -m pytest
  src/tests/sim/system/test_tour_closure_gate.py` (or equivalent) for the
  sim-tier TOUR_2 check, run repeatedly (>=5x) per the acceptance
  criteria above.

## Implementation Plan

**Approach**: In `Planner::planWheels()` (around `planner.cpp:1231-1240`),
change the logic from "monotonic set, never cleared" to: compute `raw =
phase[binding]` as today; if `active_.decelLatched` is true AND `raw` is
`Accel` or `Hold`, clear `active_.decelLatched` and let the fresh `raw`
stand (skip the clamp-and-override for this tick); else if
`active_.decelLatched` is true, keep the existing clamp/override
behavior; else (not latched), keep the existing "set latch on
Decel/Closing" behavior. Add the three new test scenarios to
`src/motion/planner/tests/` using the existing `PerfectPlant`/
`NoisyPlant`/`cycle()` scaffolding already in `test_support.h` — no new
plant infrastructure needed. Re-run `test_tour_closure_gate.py`
(single-step harness) to confirm the sim-tier TOUR_2 fix; do NOT use
`test_gui_button_acceptance.py`'s background-tick-thread harness to
measure this (it is not bit-reproducible and its own ~30mm of spread
would drown the ~10-degree signal this ticket fixes).

**Files to modify**:
- `src/motion/planner/planner.cpp` (`decelLatched` release condition)
- `src/motion/planner/tests/planner_scenarios_test.cpp` (or a new test
  file if scope warrants) — new Angle-above-90/chained/transient
  scenarios
- `src/motion/planner/tests/test_support.h` — only if a chained-scenario
  helper is needed beyond what already exists

**Testing plan**: as listed above.

**Documentation updates**: `planner.cpp`'s comment block around
`decelLatched` (currently describing it as a one-way ratchet) updated to
describe the release condition and reference this ticket;
`profile.cpp`'s existing "let re-measurement recover" comment gets a
cross-reference note that this promise is now honored at the caller
level too, not just within `profileStep()` itself.

## Completion Notes (programmer, 2026-08-03)

**Fingerprint check, done BEFORE writing any fix code, per this ticket's
own instruction.** Instrumented every `Planner::planWheels()` call
(temporary stderr printf, `#ifdef`-gated, never committed) with
`active_.move.kind`, `active_.decelLatched`, the fresh `raw` phase
(`phase[binding]`, before the latch's own override), and `lambda`.
Ran `test_tour_closure_gate.py`'s own `_run_tour_capture()` helper
directly (not through pytest) for TOUR_1 and TOUR_2, both ideal and
realistic error profiles — 2351 instrumented ticks total, covering every
Distance and Angle Move either tour issues. Result: **zero** ticks where
`decelLatched` was held AND the fresh `raw` recomputed `Accel`/`Hold` —
the exact trigger this ticket's fix releases on never fires in these
runs. Every Angle Move's phase trace is a clean, monotonic
Accel -> Hold -> Decel -> Closing with no reversal, in EVERY turn
examined (both the ones inside the 8 deg shaped band and the ones badly
outside it). The originating issue's own suggested fingerprint ("a
stall-event completion, with no timeout") is also absent: every turn's
own completion event fires via the ordinary Closing/profile-complete
branch (the planner's own `plannedRemaining` lands within float noise
of zero at completion, in every turn, good or bad) — never the 0.5s
stall backstop.

**Conclusion: the hypothesis, as it explains the CURRENTLY OBSERVED
`test_tour_closure_gate.py` shaped-band failures, is not confirmed.**
Per this ticket's own instruction ("if the fingerprint is absent, say so
and investigate rather than implementing a fix for a mechanism you have
not demonstrated"), this is reported plainly rather than forced.

**What IS actually happening** (found by comparing the Planner's own
internal `remaining` accounting, which lands near-exact at completion on
EVERY turn, against `test_tour_closure_gate.py`'s ground-truth
achieved-vs-commanded delta, which does not): TOUR_1's `straight_leg_
cruise_headings` (the file's own 119-005 per-leg TRUE-heading-drift
check) shows leg 3 (the straight leg between turn 2 and turn 4) carrying
a max true-heading drift of 6.45 deg under the IDEAL (zero sensor-noise)
profile alone — and 130-010's own cumulative-baseline ledger carries a
completed Distance leg's baseline **unchanged** (`carryHeading_ =
active_.baselineHeading`, `planner.cpp`'s `tick()`, the `Move::Kind::
Distance` completion case), i.e. it does NOT fold in whatever the
straight leg itself actually drifted by. The FOLLOWING turn then targets
a heading that is `baselineHeading + 90deg` relative to a baseline that
predates the straight leg's own drift, so it physically rotates
`(90 - drift)` degrees from its own true starting heading to reach that
fixed target — an apparent "undershoot" that is arithmetically explained
by the preceding leg's own uncaptured drift, not by anything wrong in
the turn's own execution. This reads as the mirror image of the
already-known, already-accepted "axis-drop coast at chain boundaries"
defect (119-002, `test_tour_closure_gate.py`'s own `StraightLegCruise
Check` doc comment) — turn-into-straight coast is documented; this
appears to be straight-into-turn drift NOT being captured by the carry.
It does not fully close the arithmetic on every turn checked (leg 3's
6.45 deg max drift does not by itself account for turn 4's full -13.08
deg), so there may be a second contributing factor as well — this was
not chased further, per the time already spent confirming/refuting the
ticket's own named hypothesis, which is the actual scope here.

**The fix is still implemented, as directed, because**: (1) it is a
real, independently-demonstrated defect (see the two paragraphs above),
even though it doesn't happen to be triggered by the specific sim
scenarios this suite currently exercises; (2) it is safe by
construction — the new branch only ever WIDENS what the profiler is
allowed to command relative to the old one-way ratchet, never restricts
further, and every existing `planner_tests` scenario (8/8) plus the
new ones pass unchanged; (3) it is the explicit architectural direction
recorded in `sprint.md` and this ticket; (4) sanity-checked directly:
reverted the fix in a scratch copy and re-ran the new
`testAngle146DegreesRecoversFromTransientDecelLatchTrip` scenario against
the OLD code — it fails exactly as expected (`sawRecovery` false, the
old unconditional latch never lets `raw` recover), confirming the new
test is a real regression guard for the mechanism this ticket targets,
not a vacuous pass.

**Constructing a repro for the `planner_tests` scenarios.** The
ORGANIC sim-tour dynamics do not trip the latch prematurely (see the
fingerprint check above), so the transient-misprediction test
(`testAngle146DegreesRecoversFromTransientDecelLatchTrip`,
`planner_noise_test.cpp`) deliberately engineers one, the same way
130-010's own `testAngleDoesNotUndershootAtCompletionUnderSevereTracking
Lag` deliberately used an extreme `trackingLag` beyond what any real
profile in this tree configures, "to characterize the mechanism, not to
match a specific measured profile." Here: `NoisyPlant.sampleDivisor = 8`
(a fresh encoder sample only every 8th cycle — beyond `dirtyPlant()`'s
own every-OTHER-cycle convention, but the same knob, already documented
in this file's own header as existing "to prove the planner survives
repeats") makes the sample-age term in `measure()`'s ZOH predict swing
far enough, during a slow (1 rad/s) 146-degree cruise, to transiently
fail `profileStep()`'s own feasibility test mid-cruise — nowhere near
the real end of the Move. Measured directly (temporary instrumentation,
same technique as the fingerprint check): pre-fix, the robot's command
FREEZES at the wrongly-tripped, clamped speed for 3 ticks once a fresh
sample proves there was plenty of remaining rotation left, before the
`Closing` branch's own exact-landing math (which is always computed
against the TRUE, measured `anchoredRemaining`, regardless of the
clamp) eventually forces it back down to zero anyway — landing at 2.34
deg error in that specific repro. Post-fix, the same tick sees the
latch release, the command climbs back toward cruise for two ticks, and
a second, genuine decel lands the Move at 7.19 deg error in that SAME
repro — a somewhat LARGER numeric error in this one constructed case
(re-accelerating then re-braking costs a little accuracy the frozen
crawl doesn't), because this specific repro's "frozen" case never runs
long enough to hit the actual stall backstop the originating issue
describes — but it is the mechanistically correct behavior (a Move that
genuinely still has 90+ degrees left must not sit at a nearly-stalled
crawl for arbitrary tick counts just because one early tick
mis-predicted), and the test asserts the MECHANISM (a clean recovery
happens, no chatter, no timeout, bounded final error) rather than a
"smaller number wins" comparison against one hand-picked repro. See the
test's own comments for the full parameter rationale.

**Per-turn numbers, before and after this ticket's fix** (`test_tour_
closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_
the_shaped_band`, both measured on the single-step harness) — IDENTICAL,
bit-for-bit, in every cell:

TOUR_1 / ideal: turn 2 -2.567, turn 4 -13.080, turn 6 -12.419,
turn 8 -11.959, turn 10 -13.482, turn 12 -24.699 (deg).
TOUR_1 / realistic: turn 2 -0.119, turn 4 -18.558, turn 6 -4.605,
turn 8 -14.602, turn 10 -16.075, turn 12 -13.450.
TOUR_2 / ideal: turn 2 -2.567, turn 4 -13.287, turn 6 -14.327,
turn 8 (146deg) +5.192, turn 10 -16.667, turn 12 -8.153, turn 14 +12.778.
TOUR_2 / realistic: turn 2 -0.119, turn 4 -18.660, turn 6 -2.902,
turn 8 (146deg) +14.074, turn 10 -20.893, turn 12 -13.586, turn 14
+14.378.

Note the 146-degree turn (TOUR_2 turn 8) is now OVERSHOOTING
(+5.19/+14.07 deg), not undershooting (-10.12 deg) as the originating
issue recorded — some prior 131 ticket already shifted this number's
sign; whichever mechanism is actually responsible is likely
angle/direction-dependent (matches the "straight-leg-drift-into-turn"
theory above: the preceding leg's own drift direction, not the turn's
own magnitude, sets the sign).

**Recommended follow-up** (not filed as a formal issue by me — I do not
have issue-authoring scope as the executing programmer for this ticket,
and this is the sprint's last ticket, so I'm recording it here for the
team-lead to file): re-open investigation into
`A-tour2-146-degree-turn-still-undershoots-after-130-010.md` (or split
a fresh issue) targeting the cumulative-baseline ledger's Distance-leg
carry (`planner.cpp`'s `tick()`, `Move::Kind::Distance` completion case)
not folding in the leg's own actual heading drift — the mirror image of
the already-known 119-002 "axis-drop coast" defect, on the OTHER side of
a straight/turn chain boundary. `straight_leg_cruise_headings` (already
instrumented in `test_tour_closure_gate.py`) is the existing tool to
measure it; per-leg max drift already correlates in direction and rough
order of magnitude with the following turn's own error, though not
exactly (leg 3's 6.45 deg vs. turn 4's -13.08 deg), so there may be a
second, uninvestigated contributor too.

**Verification actually run**: `planner_tests` standalone build, all
9 executables (8 pre-existing + this ticket's additions folded into
existing files), 100% pass. `test_tour_closure_gate.py`, full file, run
directly (not the three full suites — out of this ticket's given
scope): 1 failed (the pre-existing, unchanged shaped-band gate, restated
above), 4 xfailed (pre-existing, untouched), 1 xpassed (`test_two_
compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_
level`, `strict=False` so non-blocking either way — this test uses a
REAL background tick thread, not the deterministic stepper, so its
outcome is not attributable to this ticket's change with confidence;
not independently re-verified against the pre-fix tree). `square.tour`/
`circle.tour`/the three full suites (`src/tests/sim`, `src/tests/unit`,
`src/tests/testgui`) were NOT run, per this ticket's own explicit
verification scope ("Do not run the three full suites — I run those at
the boundary").
