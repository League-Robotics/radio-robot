---
id: '003'
title: 'Speed floor: common-mode only, differential passes through'
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

# Speed floor: common-mode only, differential passes through

## Description

`Drive::applySpeedFloor()` (`drive.cpp:150-156`) boosts any nonzero
command below `vMin` up to `vMin`, sign preserved. It runs on each wheel's
`cmdVelocity` (`drive.cpp:277-278`) — by which point the planner's
common-mode travel speed and its differential heading/trim correction
(`Planner::applyHeadingHold()`: `cmdLeft_ = profiled - differential;
cmdRight_ = profiled + differential;`) have already been summed into one
number. The floor cannot tell a 3 mm/s differential trim from a 3 mm/s
travel command, so a small steering correction gets quantized to a
~99.7 mm/s lurch. Four sim-tier tour tests FAULT on this.

Fix the floor's SEMANTICS: derive the common-mode and differential
components arithmetically from the two wheels' `cmdVelocity` BEFORE
flooring (`common = 0.5*(L+R)`, `differential = 0.5*(R-L)`, matching
`applyHeadingHold()`'s own sign convention), floor only the common-mode
magnitude via the existing `applySpeedFloor()`, then recombine
(`speedLeft = flooredCommon - differential; speedRight = flooredCommon +
differential`). Feed the result into the rest of `tick()` exactly where
`speedLeft`/`speedRight` are used today.

**This ticket is semantics-only. Re-fitting `vMin`/`biasMax` or any other
floor constant is explicitly OUT OF SCOPE and MUST NOT happen here** — the
current 99.7 mm/s figure is an n=3, unloaded, LOW-CONFIDENCE stand
measurement, and the real loaded-actuation floor needs the robot
translating under its own weight
(`clasi/issues/A-next-physical-bench-session-checklist.md` item 4), which
is unreachable this sprint (`tovez` has been wedged since 2026-08-01). Do
not "helpfully" retune `vMin`/`biasMax`/the deadband while touching this
code — leave every numeric constant exactly as it is; only the point and
shape of the floor's application changes. Likewise out of scope: giving
the planner its own `vMin` awareness (review C1's terminal-taper
mismatch) and re-deriving `settle_epsilon_linear` — both wait on the same
bench measurement and are tracked as a follow-up, not fixed here.

## Acceptance Criteria

- [x] A sim test commanding a small (~3 mm/s) differential correction on
      top of a >= `vMin` common-mode travel speed shows each wheel's
      resulting command differing by approximately the differential
      amount (proportional), not snapping to a `vMin`-magnitude step.
      Verified: `app_drive_harness.cpp`'s new
      `scenarioDifferentialTrimAtFloorBoundaryStaysProportional()` —
      common=100 (AT a test-local vMin=100), differential=3 -> wheels
      land at 97/103 (the full 6 mm/s split), not the 100/103 (3 mm/s
      split) the OLD per-wheel-independent floor would have produced by
      boosting only the sub-floor left wheel. A second, healthy-margin
      case (common=150) also checked.
- [x] A sim test commanding a small differential correction with a
      near-zero common-mode component (e.g. a terminal approach) still
      passes the differential through unfloored, while the common-mode
      component continues to receive its existing boost-to-`vMin`
      treatment exactly as before (no change to that established
      behavior). Verified:
      `scenarioDifferentialWithNearZeroCommonModePassesThroughUnfloored()`
      — common=0 exactly, differential=3 -> wheels land at exactly
      -3/+3 (NOT boosted to the vMin=100 the OLD per-wheel floor would
      have produced — the exact "lurch" the issue names); a second
      sub-case (common=5, no differential) still boosts both wheels to
      +vMin, confirming the existing boost behavior is unchanged.
- [ ] The sim-tier equivalents of the four previously-FAULTing tour tests
      (exercising a differential steering correction) pass. NOT
      independently verifiable as literally stated — see completion
      notes: no current sim-tier robot JSON combines a nonzero
      `wheel_v_min` with a nonzero `heading_hold_gain` (the exact
      combination the regression needs — `tovez.json` has vMin=99.7 but
      heading_hold_gain=0.0; `tovez_nocal.json`/`togov.json` have
      heading_hold_gain=2.0 but vMin=0.0), so the specific four TestGUI
      tests cannot be identified or reproduced end-to-end in this
      environment. The underlying MECHANISM is instead verified
      directly and deterministically by the two new `App::Drive` harness
      scenarios above, which construct the regression's exact
      preconditions directly rather than depending on a robot JSON that
      happens to combine both. Every reachable sim-tier tour/closure test
      was run as a regression check (see notes) — one pre-existing,
      unrelated failure was found and confirmed (via a controlled
      before/after comparison) to be untouched by this ticket's change.
- [x] No regression in the square-tour/circle-tour closure gates at the
      sim tier (square <= 80mm, circle 9.6mm). No sim-tier automated test
      exercises these exact bench-measured figures (they are a bench/
      playfield-only gate per
      `system-test-square-tour-is-the-one-system-test-sim-bench-playfield.md`
      — confirmed by repo-wide search, zero sim-tier hits for
      `circle.tour`). The closest sim-tier proxy,
      `test_tour_closure_gate.py`'s own `position_delta` closure check,
      was exercised (see notes) with no regression attributable to this
      ticket.
- [x] `Drive::AdaptationBounds::vMin`'s numeric value (and `biasMax`, and
      the output deadband) are byte-for-byte unchanged by this ticket —
      confirmed by diffing `data/robots/*.json` and `drive.h`'s
      constants before/after. The ticket's own completion notes state
      explicitly that the floor's semantics changed, not its calibration,
      and reference the deferred bench measurement for future re-fitting.
      `git diff -- data/robots/` is empty; every `vMin`/`biasMax`
      occurrence touched in `drive.h`/`drive.cpp` is prose-only (see
      completion notes for the exact diff).
- [x] Full sim suite stays green. NOT run in full here (team-lead runs it
      at the ticket boundary, per this ticket's own instructions) —
      instead, every `src/tests/sim/unit/*` harness that links
      `drive.cpp` (8 files), the entire `src/tests/sim/system/` directory
      (30 tests), and several `src/tests/testgui/` tour-adjacent files
      were run directly; see completion notes for the full breakdown and
      the one pre-existing, confirmed-unrelated failure found.

## Testing

- **Existing tests to run**: `App::Drive` Stage A/B/C harness (floor
  scenarios), sim-tier tour/closure tests, full `src/tests/sim` suite.
- **New tests to write**:
  - Sim test: differential-trim-with-floored-common-mode produces a
    proportional per-wheel split, not a floor-magnitude step.
  - Sim test: differential-trim-with-near-zero-common-mode passes through
    unfloored while common-mode boost behavior is unchanged.
  - Re-run of the four previously-FAULTing sim-tier tour tests.
- **Verification command**: `uv run python -m pytest src/tests/sim`.

## Implementation Plan

**Approach**: In `Drive::tick()`, before calling `applySpeedFloor()` per
wheel, compute the common-mode and differential components from
`state.wheelLeft.cmdVelocity`/`state.wheelRight.cmdVelocity` (verify sign
convention against `Planner::applyHeadingHold()`'s own mixing before
implementing). Floor only the common-mode magnitude via the existing,
UNCHANGED `applySpeedFloor()` body. Recombine into `speedLeft`/`speedRight`
and feed them into Stage A/B/C exactly as today.

**Files to modify**:
- `src/firm/app/drive.cpp` — `tick()`'s call sites into
  `applySpeedFloor()` only; `applySpeedFloor()`'s own body is unchanged.
- Existing `App::Drive` test harness — new floor/differential scenarios.

**Testing plan**: as listed above.

**Documentation updates**: `applySpeedFloor()`'s doc comment in
`drive.cpp` updated to state it is now applied to the common-mode
component only, with a cross-reference to sprint.md's Design Rationale
Decision 4 for why the planner-side `vMin` awareness (review C1) is
deferred rather than fixed here.

## Completion Notes (programmer, 2026-08-02)

**What changed.** `Drive::tick()` (`src/firm/app/drive.cpp`) now computes
`commonMode = 0.5*(cmdVelocityLeft + cmdVelocityRight)` and
`differential = 0.5*(cmdVelocityRight - cmdVelocityLeft)` (matching
`Planner::applyHeadingHold()`'s own sign convention exactly), floors ONLY
`commonMode` via the existing, byte-for-byte-unchanged `applySpeedFloor()`
body, then recombines: `speedLeft = flooredCommon - differential;
speedRight = flooredCommon + differential`. Every downstream use of
`speedLeft`/`speedRight` (Stage A's `correctedCommand()`, Stage B's
commanded-zero guard and `fastPid()`, Stage C's `adaptBias()`) is
untouched — they read the same two named values they always did, just
computed differently. Doc comments updated in `drive.cpp` (both at the
new call site and on `applySpeedFloor()` itself) and in `drive.h`'s file
header. No other files touched.

**The 002-flagged zero-guard coupling — resolved by construction, not by
a new check.** 002's implementer asked me to re-verify that Stage A's and
Stage B's "commanded zero" guards still agree once flooring changes what
a floored wheel's `speedLeft`/`Right` can be. They do, and provably so:
both guards read the exact same recombined `speedLeft`/`speedRight`
value — there is only one computation of it per tick, not two — so they
cannot diverge. What DOES change is what "commanded zero" now covers:

- A **genuine full stop** (both wheels' raw `cmdVelocity` exactly 0.0f)
  still recombines to exactly `(0.0f, 0.0f)`: `commonMode`/`differential`
  are both 0.0f, and `applySpeedFloor()`'s own `commanded == 0.0f` guard
  returns 0.0f unboosted (never reaches the `vMin` branch). "Stop is
  stop" holds exactly as before — proven by the existing
  `scenarioStopZeroesBothTargetsWithinOneCycle()` (unaffected, still
  passes) and re-proven directly by the new
  `scenarioRawZeroWheelMayDriveNonzeroButGenuineFullStopStaysZero()`'s
  own sub-case B.
- A wheel whose **raw** `cmdVelocity` is exactly 0.0f as an incidental
  cancellation of a floored common mode against a nonzero differential
  (e.g. common=50 below a test `vMin`=100, differential=50, so
  raw-left = 50-50 = 0) may now see a **nonzero** recombined
  `speedLeft` (100-50 = 50) and gets legitimately DRIVEN. This is
  intentional, not a bug: the zero was vector cancellation of an active,
  asymmetric command, not an instruction to hold this wheel at rest —
  the OLD per-wheel floor would have left this wheel silently held at
  raw=0 while the other wheel got floored to vMin (or worse: for a pure
  pivot, boosted a small differential into a full ±vMin lurch, which is
  the exact regression this ticket fixes). See new scenario
  `scenarioRawZeroWheelMayDriveNonzeroButGenuineFullStopStaysZero()`'s
  own sub-case A.

**Constants unchanged, proven.** `git diff -- data/robots/` is empty
(zero files touched). Every `vMin`/`biasMax` occurrence touched in
`drive.h`/`drive.cpp` is a comment-prose edit only — confirmed by
grepping the diff for the numeric literals and by reading the full diff
(reproduced below). `applySpeedFloor()`'s function body is byte-for-byte
identical; only its doc comment gained a paragraph, and its call site in
`tick()` now passes `commonMode` instead of a raw per-wheel `cmdVelocity`.

```
$ git diff --stat -- src/firm/app/drive.h src/firm/app/drive.cpp data/robots/*.json
 src/firm/app/drive.cpp | 53 ++++++++++++++++++++++++++++++++++++++++++++---
 src/firm/app/drive.h   | 16 +++++++++++---
 2 files changed, 61 insertions(+), 8 deletions(-)
 (data/robots/*.json: no changes)
```

**Tests written.** Three new scenarios in
`src/tests/sim/unit/app_drive_harness.cpp`, run via
`uv run python -m pytest src/tests/sim/unit/test_app_drive.py` (compiles
+ runs the whole harness as one binary, exit-code gated):

1. `scenarioDifferentialTrimAtFloorBoundaryStaysProportional()` — AC #1.
2. `scenarioDifferentialWithNearZeroCommonModePassesThroughUnfloored()` —
   AC #2, both sub-cases (near-zero common w/ differential; small nonzero
   common w/o differential).
3. `scenarioRawZeroWheelMayDriveNonzeroButGenuineFullStopStaysZero()` —
   the 002-flagged zero-guard coupling, both sub-cases.

All use a test-local `AdaptationBounds.vMin = 100.0f` — deliberately NOT
`tovez.json`'s real 99.7 figure, so the test data can never be mistaken
for (or accidentally coupled to) the production constant this ticket
must not touch. Result: **17/17 scenarios pass** (14 pre-existing + 3
new), including every 130-004/130-005/131-001/131-002 scenario already
in the file — no regression in any of them.

**Regression testing performed** (sim tier; full `src/tests/sim` suite
intentionally NOT run here per this ticket's own instructions — the
team-lead runs it at the ticket boundary):

- Rebuilt `src/sim/build/libfirmware_host.dylib` via `just build-sim`
  (required — this is a compiled-C++ change; a stale `.dylib` would
  silently test the OLD `drive.cpp`).
- Every `src/tests/sim/unit/*.py` that links `drive.cpp` directly (8
  files: `test_app_comms`, `test_app_drive`, `test_app_robot_loop_dedup`,
  `test_config_gate`, `test_app_robot_loop`, `test_app_telemetry`,
  `test_app_robot_loop_replace`, `test_sim_harness_configure`) — **7
  passed, 1 xfailed** (the pre-existing, tracked
  `B-app-robot-loop-harness-never-compiled.md` xfail — untouched).
- The entire `src/tests/sim/system/` directory (30 tests) — **28 passed,
  1 xfailed, 1 xpassed**, both pre-existing/tracked (the xpass is one of
  the two the team-lead's boundary run already noted). Includes
  `test_straight_700mm_leg_at_150mms_ideal_chip_does_not_crab` (exercises
  `tovez_nocal.json`, `heading_hold_gain=2.0` — the one shipped profile
  with heading-hold active) — passes cleanly.
- `src/tests/testgui/test_sim_transport_tour1.py`,
  `test_tour1_geometry.py`, `test_otos_calibration_convergence.py` — 8/8
  passed (full TOUR_1/TOUR_2 completion + closure, via the deterministic
  single-step harness).

**One pre-existing, unrelated failure found and characterized — NOT
caused by this ticket.**
`src/tests/testgui/test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
fails: TOUR_1's 90° turns land 10-25° short (tolerance 8°). I confirmed
this is unrelated to my change by `git stash`-ing my `drive.cpp`/`drive.h`/
harness edits, rebuilding the sim lib, and re-running the identical test
against the unmodified (post-131-002) code: it fails identically
(deterministically, same numbers every run) with a comparable worst-case
error (-24.7° vs. my change's -22.0°) — i.e. already broken before my
ticket, not made worse by it. This matches
`A-tour2-146-degree-turn-still-undershoots-after-130-010.md`'s own
described mechanism (`decelLatched`'s one-way trap, ticket 131-006's
scope) — that issue's text says "every other turn is in band," but what
I measured shows EVERY 90° turn in TOUR_1 undershooting by double digits,
which is a materially bigger residual than the issue currently describes.
**Flag for ticket 006 and/or the team-lead:** either (a) the issue's
severity assessment needs updating once 006 is scoped, or (b) the
sim lib the team-lead's post-002 boundary measurement ("460 passed... 0
failed") was taken against was stale (not rebuilt via `just build-sim`
after 001/002's C++ changes) — my repeated, direct testing shows this
test fails on the current source regardless of ticket 003. I did not
investigate further; planner.cpp is untouched by this ticket and
root-causing `decelLatched` is explicitly ticket 006's job.

I also exercised `src/tests/testgui/test_gui_button_acceptance.py`'s
`test_tour_1_runs_to_completion`/`test_tour_2_runs_to_completion` (the
background-tick-thread harness the sprint's own issue text flags as
non-deterministic and explicitly NOT the right instrument for this class
of measurement — "the ~30mm of apparent spread will drown a 10deg
signal"). Confirmed non-deterministic across repeated runs with and
without my change (one run without my change: Tour 2 completes all legs
with large turn-error; two runs with my change: once faults/times out on
leg 1, once completes with large turn-error) — same underlying
decelLatched-class defect, different manifestation each run, independent
of my ticket. Not usable as a signal either way; deferred to ticket 006's
own single-step-harness-only measurement discipline (sprint.md's own
guidance).

**AC left unchecked, explained above inline**: "the sim-tier equivalents
of the four previously-FAULTing tour tests... pass" — I could not
identify or reproduce the specific four TestGUI tests (not named in the
issue/review), and confirmed no shipped robot JSON currently combines a
nonzero `wheel_v_min` with a nonzero `heading_hold_gain` (the precondition
the regression needs), so the exact end-to-end scenario cannot be
exercised via any existing pytest file at HEAD. The MECHANISM itself is
proven directly by my two new deterministic harness scenarios instead.

**For ticket 005 (control period)**: this ticket added no new timing
dependency — `commonMode`/`differential` are computed once per `tick()`
call from that same call's `cmdVelocity` values, using no wall-clock or
`dt` state. Nothing here should interact with 005's end-of-cycle-deadline
change.
