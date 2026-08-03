---
id: '003'
title: 'Speed floor: ratio-preserving scale, not common-mode-only (REVISED)'
status: done
use-cases:
- SUC-131-003
depends-on:
- '002'
github-issue: ''
issue: A-speed-floor-snaps-the-planner-differential.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Speed floor: ratio-preserving scale, not common-mode-only (REVISED)

## Revision notice

**This ticket supersedes its own prior shipped implementation
(commit `29578345`, "floor the common-mode wheel speed only,
differential passes through").** That implementation was measured, by
the team-lead's boundary verification, to REGRESS turn accuracy: TOUR_1
90-degree turns went from -2.68/-13.19 deg (pre-sprint baseline) to
-10.38/-20.07 deg (post-003) on
`test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
(single-step harness), bisected commit-by-commit and isolated to this
ticket (001/002 are neutral). Root cause and the new design are recorded
in full in `sprint.md`'s Architecture section, under "Revision — Ticket
003's speed-floor semantics" — read that section before implementing.
This ticket's Description/Acceptance Criteria/Implementation Plan below
are REWRITTEN to match the new design; the old "Completion Notes"
section from the superseded implementation is kept at the bottom of this
file, relabeled, as the historical record of what was tried and why it
didn't work — do not delete it.

## Description

`Drive::applySpeedFloor()` (`drive.cpp`) boosts any nonzero commanded
speed below `vMin` up to `±vMin`, sign preserved. The ORIGINAL fix
(this ticket's first attempt) ran it on a `commonMode`/`differential`
decomposition of the two wheels' `cmdVelocity`, flooring only the common
mode. That is mathematically fine but physically wrong for a pure
rotation: `Planner::planWheels()` (every Angle-kind Move, i.e. every
90-degree turn in the tour) emits `cmdLeft_ = lambda*unit[0]`,
`cmdRight_ = lambda*unit[1]`, and for a pure pivot `unit[0]=-1`,
`unit[1]=+1`, so the common mode is EXACTLY zero for the entire turn —
the floor never engages at any point in the turn's profile, and the
turn's own accel ramp-in / decel ramp-out commands a sub-breakaway duty
the actuator cannot deliver, undershooting badly. Full mechanism in
`sprint.md`'s Revision section.

**Fix (revised): a ratio-preserving scale, computed from the raw wheel
pair directly — no common-mode/differential decomposition.**

```
dominantMag = max(|cmdVelocityLeft|, |cmdVelocityRight|)
if dominantMag > 0 and vMin > 0 and dominantMag < vMin:
    scale = vMin / dominantMag
    speedLeft  = cmdVelocityLeft  * scale
    speedRight = cmdVelocityRight * scale
else:
    speedLeft  = cmdVelocityLeft   # unchanged
    speedRight = cmdVelocityRight  # unchanged
```

This preserves the commanded RATIO between the two wheels while ensuring
the dominant (larger-magnitude) wheel reaches the actuator's minimum
sustained speed whenever it would otherwise be sub-floor. It matches the
pre-131-003 per-wheel-independent floor exactly for a symmetric pivot
(both wheels equal magnitude — the case that regressed), matches the
131-003-shipped floor's own intended, tested behavior for a differential
riding on an already-above-floor common mode (the dominant wheel already
clears `vMin`, so no scaling applies — raw pair passes through
unchanged), and is a strict improvement over both for an asymmetric arc
(preserves curvature instead of flattening it toward 1:1 or leaving it
unboundedly distorted). It also has the useful property that any wheel
commanded EXACTLY `0.0f` stays exactly `0.0f` after scaling
(`0 * scale == 0` in IEEE-754), so "stop is stop" and the Stage A/Stage B
commanded-zero guards (131-002) hold without any special-case reasoning.

This is the same dominant-wheel/ratio-lock pattern
`Planner::planWheels()` already uses internally for its own accel-ceiling
tie-break (`planner.cpp`, the `dominant`/`other`/tie-break logic around
line 1175) — reuse of a precedented idiom, not a new one.

**This ticket is semantics-only. Re-fitting `vMin`/`biasMax` or any other
floor constant is explicitly OUT OF SCOPE and MUST NOT happen here** —
same constraint as the original ticket, unchanged: the current 99.7 mm/s
figure is n=3, unloaded, LOW-CONFIDENCE, and the real loaded-actuation
floor needs the robot translating under its own weight
(`clasi/issues/A-next-physical-bench-session-checklist.md` item 4),
unreachable this sprint. Do not retune `vMin`/`biasMax`/the deadband
while touching this code — only the shape of the floor's application
changes. Likewise still out of scope: giving the planner its own `vMin`
awareness (review C1) and re-deriving `settle_epsilon_linear` — both
wait on the same bench measurement. The revision explicitly confirmed
this scope boundary still holds (see sprint.md Revision, point 6): the
ratio-preserving scale needed no planner-side change to fix the
regression.

**Acknowledged, deliberate behavior change from the original 131-003
shipped code (not a silent regression — see sprint.md's Revision
section for the full reasoning):** a real differential trim riding on an
exactly-/near-zero common-mode component (e.g. `applyHeadingHold()`'s
correction during a Distance Move's terminal approach) is now ALSO
scaled up toward `vMin` rather than passed through unfloored, reverting
to the same behavior the pre-131-003 code had for this one sub-case.
This is accepted because (a) it is physically honest — a sub-breakaway
differential is undeliverable regardless of whether it's "a correction"
or "the whole motion," and `Drive` cannot and should not try to tell the
two apart from the wheel pair alone; (b) it is unreachable on any
currently shipped robot profile — no robot JSON combines a nonzero
`wheel_v_min` with a nonzero `heading_hold_gain` (`tovez.json`:
99.7/0.0; `tovez_nocal.json`/`togov.json`: 0.0/2.0); (c) heading-hold is
independently disabled/tracked as unstable, a separate defect.

## Acceptance Criteria

- [x] Sim test: a 3 mm/s differential correction with a >= `vMin`
      common-mode command (dominant wheel already at/above `vMin`)
      produces per-wheel commands differing by approximately the
      differential amount, not a floor-magnitude jump — no scaling
      applies because the dominant wheel already clears the floor.
      (Equivalent to the original ticket's AC#1 — reuse/adapt
      `scenarioDifferentialTrimAtFloorBoundaryStaysProportional()`
      from the superseded implementation; its expected numbers are
      unchanged by this revision.)
- [x] Sim test: a symmetric pivot (equal-and-opposite wheel commands —
      the shape `Planner::planWheels()` emits for an Angle Move) with
      both wheels below `vMin` is boosted to exactly `(-vMin, +vMin)` —
      bit-identical to what the pre-131-003 per-wheel-independent floor
      produced for this case. This is the DIRECT regression-fix proof at
      the unit level; write it fresh (the superseded implementation had
      no equivalent, since its own design made this case impossible to
      get right).
- [x] Sim test: an asymmetric pair below `vMin` (e.g. 20/80 mm/s at a
      test-local `vMin`=100) is scaled to preserve its ratio exactly
      (25/100 mm/s), not flattened toward 1:1 (what the OLD
      pre-131-003 per-wheel-independent floor would have produced:
      100/100) and not left arbitrarily distorted (what the superseded
      131-003 common-mode-only floor could produce depending on sign).
      New scenario, not present in either prior implementation.
- [x] Sim test: both wheels commanded exactly `0.0f` -> output exactly
      `(0.0f, 0.0f)` ("stop is stop"). Reuse/adapt
      `scenarioRawZeroWheelMayDriveNonzeroButGenuineFullStopStaysZero()`'s
      sub-case B from the superseded implementation; sub-case A (a
      raw-zero wheel from vector cancellation while the other wheel is
      driven) should be RE-DERIVED under the new algorithm — expect the
      raw-zero wheel to now stay at exactly zero (not driven nonzero as
      the superseded implementation intentionally produced), since
      `0 * scale == 0` always. Update the scenario's own expected values
      and doc comment accordingly; this is a simplification, not a loss
      of coverage.
- [x] **HARD NON-REGRESSION GATE, the primary acceptance criterion for
      this ticket**:
      `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
      (single-step harness, NOT `test_gui_button_acceptance.py`'s
      background-tick-thread harness — that one mixes ~30mm of
      harness-induced spread into a ~10-degree signal and is not a valid
      instrument for this measurement). Rebuild
      `src/sim/build/libfirmware_host.dylib` first (`just build-sim`) —
      a stale `.dylib` silently tests the wrong `drive.cpp`. Measure
      TOUR_1 turn 2 and turn 4 error magnitude directly (add
      instrumentation/logging if the test doesn't already expose the
      per-turn numbers) and confirm: turn 2 <= 2.68 deg, turn 4 <=
      13.19 deg (the measured pre-sprint baseline — matching or beating
      the 001/002 baseline of -2.57/-13.08 is the real target, not just
      clearing the outer bound). Turn 4's pre-existing ~13 deg residual
      is ticket 006's `decelLatched` territory and is NOT this ticket's
      job to close further — the gate here is "003 must not make it
      worse," not "003 must fix it."
      **MEASURED: turn 2 = -2.567 deg, turn 4 = -13.080 deg** — matches
      the 001/002 baseline to three decimal places and clears both the
      pre-sprint (-2.68/-13.19) and 001/002 (-2.57/-13.08) bars. Confirms
      the ratio-preserving scale fixes the regression; turns 6-12's
      larger residuals are the pre-existing, out-of-scope `decelLatched`
      defect (ticket 006), reproduced identically (see Completion Notes).
- [x] No regression in the square-tour/circle-tour closure gates at the
      sim tier (square <= 80mm, circle 9.6mm) — no sim-tier automated
      test exercises these exact bench-measured figures (bench/playfield
      gate only); use `test_tour_closure_gate.py`'s own `position_delta`
      closure check as the sim-tier proxy, same as the superseded
      implementation did.
      **MEASURED directly (see Completion Notes)**: TOUR_1/ideal
      17.4mm, TOUR_1/realistic 77.8mm, TOUR_2/ideal 170.1mm,
      TOUR_2/realistic 321.7mm — all far under the 600mm sim-tier
      closure ceiling (`_CLOSURE_POSITION_MAX_MM`).
- [x] `Drive::AdaptationBounds::vMin`'s numeric value (and `biasMax`, and
      the output deadband) are byte-for-byte unchanged by this ticket —
      confirm via `git diff -- data/robots/*.json` (must be empty) and by
      reading the full `drive.h`/`drive.cpp` diff (every `vMin`/`biasMax`
      touch must be prose/logic-shape only, never a numeric literal
      change).
      **CONFIRMED**: `git diff -- data/robots/` is empty; the full
      `drive.h`/`drive.cpp` diff was reviewed line by line — every
      touched `vMin`/`biasMax`/deadband reference is prose or
      logic-shape (the `<= 0.0f` no-op guards, structurally identical to
      the prior code), no numeric literal changed.
- [x] Full sim suite stays green: every `src/tests/sim/unit/*` harness
      that links `drive.cpp` (8 files per the superseded implementation's
      own accounting), the entire `src/tests/sim/system/` directory, and
      the `src/tests/testgui/` tour-adjacent files. Team-lead runs the
      complete suite at the ticket boundary; this ticket runs the
      directly-affected subset itself and records results in Completion
      Notes.
      **RAN** (this ticket's own directly-affected subset — see
      Completion Notes for the full list and counts): all green except
      the one known pre-existing `test_tour_closure_gate.py` failure
      (matches baseline exactly, ticket 006's territory). Team-lead runs
      the three full suites at the sprint boundary per standing process.
- [x] Explicitly acknowledged in Completion Notes, not silently dropped:
      the near-zero-common-mode differential-trim sub-case (superseded
      131-003's own AC#2) now reverts to boosted-toward-`vMin` behavior.
      State plainly that this is intentional per sprint.md's Revision
      section, not an oversight.
      **ACKNOWLEDGED** — see Completion Notes below.

## Testing

- **Existing tests to run**: `App::Drive` Stage A/B/C harness (floor
  scenarios, both the reused ones and the newly-written ones above),
  sim-tier tour/closure tests, full `src/tests/sim` suite.
- **New/revised tests to write**:
  - Reuse (numbers unchanged): differential-trim-with-above-floor-common-mode
    stays proportional.
  - Revise (expectation flips, document why): differential-with-near-zero-common-mode
    now gets boosted, not passed through unfloored.
  - New: symmetric-pivot-below-floor boosts to exactly `(-vMin, +vMin)`.
  - New: asymmetric-pair-below-floor preserves ratio (not flattened,
    not left distorted).
  - Revise: raw-zero-wheel-from-cancellation now stays exactly zero
    (simpler than the superseded implementation's "intentionally driven
    nonzero" case).
  - Run and report: `test_tour_closure_gate.py`'s
    `test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
    with actual turn 2/turn 4 numbers, compared against the baseline
    table in sprint.md's Revision section.
- **Verification command**: `uv run python -m pytest src/tests/sim` plus
  the specific `test_tour_closure_gate.py` test named above.

## Implementation Plan

**Approach**: In `Drive::tick()`, replace the `commonMode`/`differential`
decomposition (and its two `applySpeedFloor()`-mediated recombination
lines) with the ratio-preserving scale described above, computed
directly from `state.wheelLeft.cmdVelocity`/`state.wheelRight.cmdVelocity`.
Decide whether `applySpeedFloor()` itself (currently a single-value
helper) should be repurposed into a two-wheel helper returning the scaled
pair, or whether `tick()` should compute the scale factor inline and
`applySpeedFloor()` is retired/kept only for reference — either is fine
as long as `bounds_.vMin`'s "0 = uncalibrated, no-op" and the exact-zero
"stop is stop" guard are preserved with the same meaning. Update both
`drive.cpp`'s inline doc comments (the `tick()` doc comment and whatever
remains of `applySpeedFloor()`'s own) and `drive.h`'s file-header
paragraph describing the floor to match the new algorithm — do not leave
prose describing the superseded common-mode-only design in place.

**Files to modify**:
- `src/firm/app/drive.cpp` — `tick()`'s floor call site; retire or
  repurpose `applySpeedFloor()` as decided above.
- `src/firm/app/drive.h` — file-header doc comment describing the floor.
- `src/tests/sim/unit/app_drive_harness.cpp` — revise/add the scenarios
  listed under Testing above.

**Testing plan**: as listed above.

**Documentation updates**: the floor's doc comment in `drive.cpp`/
`drive.h` updated to state the ratio-preserving-scale semantics, with a
cross-reference to `sprint.md`'s "Revision — Ticket 003's speed-floor
semantics" section for the full history (original decision, the
measurement that invalidated it, why the replacement works, and the
acknowledged near-zero-common-mode tradeoff).

## Superseded Implementation Notes (commit `29578345`, common-mode-only floor — kept for history, DO NOT reuse this design)

**What shipped, and why it's being redone.** `Drive::tick()` computed
`commonMode`/`differential` from the two wheels' `cmdVelocity`, floored
only `commonMode`, and recombined. This passed all of its own written
acceptance criteria and the full reachable sim suite (17/17 harness
scenarios, 7/8 relevant unit files + 1 pre-existing xfail, 28/30 system
tests + pre-existing xfail/xpass, 8/8 TestGUI tour/closure tests) — but
the team-lead's own boundary verification, run on
`test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
specifically (a test this ticket's own implementer flagged as failing
but attributed entirely to the pre-existing `decelLatched` defect), found
that 003 itself made that same test measurably WORSE (-10.38/-20.07 vs
-2.57/-13.08 for ticket 002) — the implementer's own before/after
comparison used a smaller, less complete manual bisection (stash-based,
one data point) than the team-lead's later full commit-by-commit
rebuild-and-measure, and concluded the wrong thing about attribution.
Full root-cause analysis: `sprint.md`'s Revision section.

**Constants were, and remain, unchanged.** The superseded implementation
correctly did not touch `vMin`/`biasMax`/`data/robots/*.json` — this
constraint carries forward unchanged into the revised implementation.

**Original completion notes (2026-08-02), preserved verbatim below for
history:**

> `Drive::tick()` (`src/firm/app/drive.cpp`) now computes
> `commonMode = 0.5*(cmdVelocityLeft + cmdVelocityRight)` and
> `differential = 0.5*(cmdVelocityRight - cmdVelocityLeft)` (matching
> `Planner::applyHeadingHold()`'s own sign convention exactly), floors
> ONLY `commonMode` via the existing, byte-for-byte-unchanged
> `applySpeedFloor()` body, then recombines: `speedLeft = flooredCommon -
> differential; speedRight = flooredCommon + differential`. [...]
> **One pre-existing, unrelated failure found and characterized — NOT
> caused by this ticket.**
> `test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
> fails: TOUR_1's 90 deg turns land 10-25 deg short (tolerance 8 deg). I
> confirmed this is unrelated to my change by `git stash`-ing my
> `drive.cpp`/`drive.h`/harness edits [...]: it fails identically
> (deterministically, same numbers every run) with a comparable
> worst-case error (-24.7 deg vs. my change's -22.0 deg) — i.e. already
> broken before my ticket, not made worse by it. [...] **This
> attribution was WRONG** — see sprint.md's Revision section for the
> team-lead's later, more complete bisection that isolated the
> regression to this ticket specifically.

(Full original notes, including the complete regression-testing
breakdown, remain in this file's git history at commit `29578345` if
needed for reference.)

## Completion Notes (2026-08-02, revised implementation)

**What changed.** `Drive::applySpeedFloor()` (`drive.cpp`/`drive.h`) is no
longer a single-value helper called on a common-mode/differential
decomposition. It is now a two-wheel helper —
`applySpeedFloor(rawLeft, rawRight, speedLeft&, speedRight&)` — that
computes `dominantMag = max(|rawLeft|, |rawRight|)` and, if nonzero and
below `vMin`, scales BOTH raw wheel values by `vMin / dominantMag`,
writing the scaled pair into `speedLeft`/`speedRight`; otherwise the raw
pair passes through unchanged. `Drive::tick()`'s call site was rewritten
to call this directly on `state.wheelLeft.cmdVelocity`/
`state.wheelRight.cmdVelocity` — the `commonMode`/`differential`
decomposition and recombination lines are gone entirely. Doc comments in
both `drive.h` (file header + the `applySpeedFloor()` declaration) and
`drive.cpp` (the function's own doc comment + `tick()`'s doc comment)
were rewritten to describe the new algorithm and cross-reference
sprint.md's Revision section; no numeric literal was touched anywhere in
either file.

**Test harness.** `src/tests/sim/unit/app_drive_harness.cpp`'s
131-003 section (previously 3 scenarios) is now 5 scenarios:
- `scenarioDifferentialTrimAtFloorBoundaryStaysProportional()` — reused,
  numbers unchanged (97/103 raw, dominant wheel 103 already clears
  vMin=100, no scaling), doc comment updated to explain in terms of the
  new algorithm.
- `scenarioSymmetricPivotBelowFloorBoostsToExactVMin()` — NEW. A
  symmetric pivot (-40, +40) at vMin=100 boosts to exactly (-100, +100).
  This is the direct regression-fix proof at the unit level.
- `scenarioAsymmetricPairBelowFloorPreservesRatio()` — NEW. Raw (20, 80)
  at vMin=100 scales to exactly (25, 100) (scale = 100/80 = 1.25),
  preserving the 4:1 ratio exactly.
- `scenarioDifferentialWithNearZeroCommonModeNowBoostedTowardVMin()` —
  REVISED (was `...PassesThroughUnfloored`). Sub-case A's expectation
  flipped: raw (-3, +3) now boosts to exactly (-100, +100) instead of
  passing through at (-3, +3) — documented in-line as the accepted,
  currently-unreachable tradeoff. Sub-case B (a plain sub-floor common
  command, no differential) is unchanged.
- `scenarioRawZeroWheelStaysZeroUnderRatioPreservingScale()` — REVISED
  (was `...MayDriveNonzeroButGenuineFullStopStaysZero`). Sub-case A
  re-derived: raw (0, 50) at vMin=100 now yields (0, 100) — the raw-zero
  wheel stays exactly zero (`0 * 2.0 == 0`) while the other wheel is
  genuinely boosted, proving the scale ran. Sub-case B (genuine full
  stop, both wheels raw zero) is unchanged.

All 17 scenarios in the harness pass (verified by compiling and running
the binary directly, and via `uv run python -m pytest
src/tests/sim/unit/test_app_drive.py`).

**HARD NON-REGRESSION GATE — measured.**
`test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
(after rebuilding `src/sim/build/libfirmware_host.dylib`):

| commit | turn 2 | turn 4 |
|---|---|---|
| `22a3c368` pre-sprint base | -2.68 deg | -13.19 deg |
| `841a018b` ticket 001 | -2.57 deg | -13.08 deg |
| `0cfa5f1f` ticket 002 | -2.57 deg | -13.08 deg |
| `29578345` ticket 003 (superseded) | -10.38 deg | -20.07 deg |
| **this implementation (131-003 revised)** | **-2.567 deg** | **-13.080 deg** |

Matches the 001/002 baseline to three decimal places and clears the
regression entirely. The test itself still FAILS overall (as expected
and documented in the ticket) because turns 6/8/10/12 exceed the 8 deg
shaped-band tolerance (-12.4/-12.0/-13.5/-24.7 deg) — this is the
pre-existing `decelLatched` defect, ticket 006's territory
(`A-tour2-146-degree-turn-still-undershoots-after-130-010.md`), not
introduced or worsened by this ticket.

**Square/circle sim-tier proxy — measured directly** (via a one-off
script calling `_run_tour_capture()` for all four tour/profile
combinations, since the failing per-turn assertion above prevents the
test's own `position_delta` check from being reached in the same run):

| tour/profile | `position_delta` |
|---|---|
| TOUR_1/ideal | 17.4 mm |
| TOUR_1/realistic | 77.8 mm |
| TOUR_2/ideal | 170.1 mm |
| TOUR_2/realistic | 321.7 mm |

All far under `_CLOSURE_POSITION_MAX_MM` = 600 mm.

**Constants diff.** `git diff -- data/robots/` is empty. The full
`drive.h`/`drive.cpp` diff was reviewed line by line: every `vMin`/
`biasMax`/deadband touch is prose or a logic-shape guard
(`bounds_.vMin <= 0.0f` no-op check, structurally identical to the prior
`applySpeedFloor()`'s own guard) — no numeric literal changed.

**Directly-affected subset run by this ticket** (team-lead runs the
three full suites at the sprint boundary):
- `src/tests/sim/unit/test_app_drive.py` — 1 passed (17/17 scenarios).
- `src/tests/sim/unit/test_app_comms.py`,
  `test_app_robot_loop.py`, `test_app_robot_loop_dedup.py`,
  `test_app_robot_loop_replace.py`, `test_app_telemetry.py`,
  `test_config_gate.py`, `test_sim_harness_configure.py`,
  `src/tests/sim/system/test_scripted_twist_demo.py`, `test_sim_api.py`,
  `test_composition_root_parity.py`, `test_move_protocol.py`,
  `test_robot_loop_tlm.py`, `test_straight_twist.py`,
  `faults/test_fault_knobs.py` — 15 passed, 1 xfailed (pre-existing),
  0 failed.
- `src/tests/testgui/test_tour_closure_gate.py`,
  `test_sim_transport_tour1.py`, `test_tour1_geometry.py`,
  `test_tour_stop.py` — 16 passed, 4 xfailed, 1 xpassed, 1 failed (the
  known pre-existing `decelLatched`-driven failure above, matching
  baseline exactly — not a new failure).

Note: every test harness in the directly-affected subset above other
than `test_app_drive.py`'s own new/revised scenarios and
`sim_api_harness.cpp` (which sets `vMin = 0.0f` explicitly) either
configures no `AdaptationBounds.vMin` at all, or runs through the
composition root's default boot config, which bakes in `vMin = 99.7`
(`boot_config.cpp`) — so the floor is genuinely engaged, not a no-op,
for the testgui/tour-adjacent tests above, and the "15 passed" sim-tier
result is a real exercise of the new algorithm, not an accidental
pass-through.

**Acknowledged tradeoff (not silently dropped).** The superseded
implementation's own AC#2 scenario — a real differential trim (e.g.
`Planner::applyHeadingHold()`'s correction) riding on an exactly-/
near-zero common-mode component — is now ALSO scaled up toward `vMin`
rather than passed through unfloored, reverting to the pre-131-003
"lurch" behavior for this one sub-case. This is intentional, per
sprint.md's Revision section: (a) it is the physically honest answer —
`Drive` cannot tell a zero-common-mode differential apart from a
symmetric pivot from the raw wheel pair alone, and both are equally
sub-breakaway; (b) it is currently unreachable on any shipped robot
profile — no robot JSON combines a nonzero `wheel_v_min` with a nonzero
`heading_hold_gain` (`tovez.json`: 99.7/0.0; `tovez_nocal.json`/
`togov.json`: 0.0/2.0); (c) heading-hold is independently disabled/
tracked as unstable. This is not an oversight.

**For ticket 005/006:** No planner-side change was needed to fix the
regression — `Motion::Planner` was not touched. Ticket 006's own
`decelLatched` fix operates on the same tour/turn-accuracy measurement
instrument (`test_tour_closure_gate.py`); the baseline table above
(turn 2/4 now at -2.567/-13.080) is what 006 inherits as its own
starting point for turns 2/4, and turns 6/8/10/12 (-12.4 to -24.7 deg)
are what 006 is expected to close. Ticket 005 (absolute end-of-cycle
deadline pacing) touches `RobotLoop`'s pacing block, not `Drive` or the
speed floor — no interaction expected, but note that this ticket's own
sim-tier measurements above were taken against the sim's own fixed
`kCycle`-per-step advance, unaffected by 005's change either way.
