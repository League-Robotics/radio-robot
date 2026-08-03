---
id: '006'
title: Release decelLatched on re-measured rise + large-angle/chained/transient planner_tests
status: open
use-cases: [SUC-131-006]
depends-on: ['005']
github-issue: ''
issue: A-tour2-146-degree-turn-still-undershoots-after-130-010.md
completes_issue: true
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

- [ ] `active_.decelLatched` clears when the current tick's own
      recomputed `raw` (from `phase[binding]`, before the latch's
      override) is `StepPhase::Accel` or `StepPhase::Hold`; it remains
      set (continuing to clamp/force Decel) otherwise — the
      never-re-accelerate-once-genuinely-braking guarantee is preserved
      for a Move that is truly finishing.
- [ ] New `planner_tests` scenario: an Angle Move of 146 degrees with
      `PerfectPlant` completes exact, at the same tolerance as the
      existing 90-degree (`testAngleExact()`) test.
- [ ] New `planner_tests` scenario: the same 146-degree Angle Move under
      `NoisyPlant` (lag/gain/quantization) reproduces a transient
      misprediction that would have tripped the pre-fix latch, and
      confirms the Move recovers (completes via a genuine
      profile-complete/settle event, not the 0.5s stall backstop) rather
      than completing short.
- [ ] New `planner_tests` scenario: a chained leg -> 146-degree-turn ->
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
- [ ] No regression: TOUR_1 worst per-turn error stays <= 2.72 deg,
      `square.tour` stays <= 80 mm (41.7/43.0 mm today), `circle.tour`
      stays at 9.6 mm PASS, full sim suite stays at 460 passed / 0 failed
      (or the baseline at ticket start, restated if it has moved).
- [ ] If the new test scenarios reveal the release condition needs a
      deadband/hysteresis beyond a bare Accel/Hold check (e.g. chattering
      near the boundary), that finding is recorded in the ticket's own
      completion notes as an explicit, justified design refinement — not
      silently patched without explanation, and not left unaddressed if
      it causes a measurable regression.

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
