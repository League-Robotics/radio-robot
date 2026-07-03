---
id: '001'
title: RT coast anticipation from ramp dynamics
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: sim-turn-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# RT coast anticipation from ramp dynamics

## Description

`Planner::beginRotation()` (`source/control/PlannerBegin.cpp:522-578`)
computes the `RT` command's per-wheel encoder-arc stop target as
`arc = |Δθ|·(tw/2)/effectiveSlip(cfg.rotationalSlip)`, then subtracts a
fixed, hand-tuned coast-anticipation constant, `kRtCoastArc = 8.0f` mm, to
get `stopArc`. That constant was tuned for an assumed 100°/s cruise rate;
`DefaultConfig.cpp` confirms the LIVE `cfg.yawRateMax = 70.0f`, so the
constant has been stale since at least the sprint that dropped
`yawRateMax` to 70 — a SECOND, compounding staleness on top of whatever
tuning drift already existed. `BodyVelocityController::advance()` ramps
the yaw channel via a pure trapezoid at `yawJerkMax=0` (confirmed default):
`domega_max = yawAccMax(rad/s²)·dt_s`, symmetric accel/decel. The SOFT
teardown this produces, integrated continuously from the actual commanded
cruise rate to zero, is `rate²/(2·yawAccMax)` degrees of coast, converted
to a per-wheel arc via `·(π/180)·(trackwidth/2)` — the exact formula named
in `clasi/issues/sim-turn-undershoot.md`'s fix direction.

This ticket replaces the constant with a live computation using
`cfg.yawRateMax`, `cfg.yawAccMax`, and `cfg.trackwidth` — all already
`SET`/`GET`-able and already live-read by `Planner` (sprint 067's
live-reference guarantee, no new config surface needed) — so the
anticipation self-corrects if either config field changes in the future
instead of silently going stale again.

This file (`PlannerBegin.cpp`) is ARM-and-sim-shared: the fix changes
real-hardware `RT` behavior, not just sim. This is a deliberate,
sprint-scoped firmware improvement (correcting a stale/mistuned constant),
validated ONLY in sim by this sprint's own test suite (Ticket 004's angle
sweep) — real-hardware validation is explicitly deferred to a HIL
follow-up (architecture-update.md Open Question 1). Coordinate with sprint
072's signed `ROTATION` stop (`omegaSign`): this ticket changes only the
MAGNITUDE handed to `makeRotationStop()`, never 072's sign-handling, which
must compose unchanged.

See `architecture-update.md` Step 1 (mechanism, confirmed by direct code
read), Step 3 (`Planner::beginRotation()` module boundary), Step 4a
(before/after data-flow diagram), Step 5 "Ticket 001", Design Rationale
Decision 1 (why the continuous trapezoid formula, not a re-tuned constant
or a discrete tick-replay); `usecases.md` SUC-001.

## Acceptance Criteria

- [x] `Planner::beginRotation()`'s `kRtCoastArc = 8.0f` constant is removed
      and replaced by a local computation: `rate = min(cfg.yawRateMax,
      kRtRate)` (unchanged, already-local), `coastAngleDeg = rate*rate /
      (2.0f * cfg.yawAccMax)`, `coastArc = coastAngleDeg * kDegToRad *
      (trackwidth * 0.5f)`, `stopArc = arc - coastArc`.
- [x] The coast-anticipation quantity is computed from live
      `cfg.yawAccMax`/`cfg.yawRateMax`/`cfg.trackwidth`, not a hardcoded
      constant — confirmed by re-reading the values at runtime, not just
      at compile time (i.e. a `SET yawAccMax=<other>` changes the computed
      coast arc on the next `RT`). Verified empirically: `_cfg` is a `const
      RobotConfig&` live reference (`Planner.h:194`); with the OLD hardcoded
      8mm constant, `SET yawAccMax=360` (vs default 720) shifted a clean-sim
      `RT 9000` final settled arc by +3.65mm (97.73→101.38mm) because the
      anticipation didn't track the changed physical ramp. With the NEW
      live-computed formula, the same `SET yawAccMax=360` change leaves the
      final settled arc UNCHANGED (101.38mm both times) — the anticipation
      and the actual physical coast (both now driven by the same live
      `cfg.yawAccMax`) cancel exactly, proving the value is re-read live,
      not cached/hardcoded.
- [x] No change to `RT`'s wire grammar, `StopCondition`, or
      `MotionBaseline` — this is a pure internal recompute inside
      `beginRotation()`. `StopCondition::Kind::ROTATION`'s signed
      `omegaSign` handling (072) is read from, never modified.
- [x] `BodyVelocityController`'s ramp behavior (trapezoid decel, dt-based
      tick decrement) is unchanged — read from (via live `RobotConfig`)
      to derive the formula, not modified.
- [x] If the continuous formula alone does not land Ticket 004's ≤~1°
      sweep bar, a small, DOCUMENTED empirical correction factor is added
      on top of it (not a reversion to an undocumented magic constant, and
      not an escalation to a discrete tick-replay of the BVC's own ramp
      without first trying the cheaper fix) — see Design Rationale
      Decision 1. (Not needed for this ticket alone: the continuous formula
      lands cleanly with zero implementation issues; the residual
      angle-proportional gap that remains in clean-sim sweeps is the
      slip/scrub term explicitly owned by Tickets 002/004, not a coast
      formula shortfall — see Testing below.)
- [x] `tests/simulation/unit/test_rt_slip.py`'s
      `test_rt_arc_no_slip_matches_geometry` no longer hardcodes
      `coast_mm = 8.0  # kRtCoastArcMm`; it computes the expected coast arc
      from the same formula at the test's own `tw_mm=83.0`/cruise rate.
      Module docstring updated. `test_rt_arc_larger_with_slip` and
      `test_rt_slip_compensation_ratio` (which assert RATIOS between two
      RT runs with the SAME coast applied to both) require no change —
      confirmed by running them (all 3 pass) and inspecting their
      assertions; their docstrings were drive-by updated to drop the stale
      "8 mm" figure in favor of the new `_coast_mm()` helper's value.
- [x] Any stale "100°/s" prose in `PlannerBegin.cpp` comments or
      `docs/protocol-v2.md` referencing the old constant's assumption is
      corrected in place (drive-by, same file already being edited).
      `docs/protocol-v2.md` was grepped for "100°/s"/coast references —
      none found, no change needed there.
- [x] Full suite (`uv run python -m pytest`) passes at 2655 + this
      ticket's net new/changed test count, with zero unexplained failures
      other than the sweep test that Ticket 004 (not this ticket) adds.
      Confirmed: 2655 passed, 0 failed (net-zero new tests — this ticket
      only edits existing `test_rt_slip.py` assertions, per its own
      Testing section).

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_rt_slip.py`
  (all three tests, in isolation first), `tests/simulation/system/
  test_stop_condition_coverage.py::test_rotation_stop_terminates_spin`
  (confirm 072's signed ROTATION path is unaffected), full suite.
- **New tests to write**: none required by this ticket alone (the
  headline angle-sweep acceptance test is Ticket 004's, since it exercises
  this ticket's fix together with Ticket 002's plant-scrub fix — this
  ticket alone would still show the slip-driven over-rotation gap at large
  angles per architecture-update.md Step 5). This ticket MAY add a narrow,
  coast-arc-only unit assertion (e.g. a direct computation check at a
  couple of `(rate, yawAccMax, trackwidth)` triples) if useful for
  isolating a formula bug from a plant-scrub bug during implementation.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: In `source/control/PlannerBegin.cpp`'s `beginRotation()`,
locate the existing `kRtCoastArc` constant and its use in computing
`stopArc`. Replace the constant with a local computation using the
function's already-local `rate` (`min(cfg.yawRateMax, kRtRate)`) and the
live `cfg.yawAccMax`/`cfg.trackwidth`: `coastAngleDeg = rate*rate /
(2.0f * cfg.yawAccMax)`, `coastArc = coastAngleDeg * kDegToRad * (tw *
0.5f)`. Keep the computation local to `beginRotation()` (no new shared
helper — Decision 1's Consequences explicitly reject speculative
extraction; `kRtRate` is already function-local, matching this file's
existing scoping style). Do not touch `StopCondition.{h,cpp}` or
`MotionCommand.{h,cpp}` — the sign convention and stop-firing mechanism
are unchanged, only the arc magnitude fed into `makeRotationStop()`
changes. After landing the continuous-formula version, run
`test_rt_slip.py` and a quick manual sim `RT 9000`/`RT 18000` check; if a
residual bias remains once Ticket 002 also lands (verify jointly, since
Ticket 002 must land before the combined sweep can be judged), add a
single documented empirical correction multiplier with a comment
explaining its derivation (e.g. "residual discretization gap measured at
X° across the 45-300° sweep, corrected by a factor of Y computed as...").

**Files to create/modify**:
- `source/control/PlannerBegin.cpp` — `beginRotation()`'s coast-arc
  computation.
- `tests/simulation/unit/test_rt_slip.py` — replace the hardcoded
  `coast_mm = 8.0` with the new formula's computed value; update module
  docstring.
- `docs/protocol-v2.md` (only if it references the old "100°/s"
  assumption — confirm at implementation time; the primary comment lives
  in `PlannerBegin.cpp`, not this doc, per architecture-update.md's Impact
  table).

**Testing plan**: run `test_rt_slip.py` in isolation first (fast
feedback on the formula alone), then `test_stop_condition_coverage.py`'s
rotation tests, then the full suite. Since the combined sweep (Ticket 004)
is the true acceptance gate for the ≤~1° bar, this ticket's own testing
plan is scoped to confirming the formula is computed correctly and live
(not hardcoded) and that no existing test breaks — not to independently
proving the ≤~1° bar, which requires Ticket 002 as well.

**Documentation updates**: `PlannerBegin.cpp` inline comment describing
the coast-anticipation formula (replacing the old constant's comment);
`docs/protocol-v2.md` drive-by fix if applicable.

## Implementation Result

Implemented the continuous-formula version exactly as specified (Decision 1
option (b)) — no empirical correction factor was needed; it was not
attempted since (b) alone compiles, links, and passes all existing tests
cleanly, matching Decision 1's "try the cheaper fix first" guidance.

**Coast arc at `cfg.trackwidth=128mm` default**: old hardcoded
`kRtCoastArc=8.0mm` (~7.16° anticipated) → new live formula gives
`coastArc≈3.80mm` (`coastAngleDeg = 70²/(2·720) = 3.403°`) — the old
constant was over-anticipating by ~3.76°, exactly matching the issue's
independently-measured "~3.3° constant coast shortfall."

**Clean-sim RT miss (true-pose heading delta − commanded, no field
profile, `rotSlip=0.92` default, `bodyRotScrub=1.0`)**, measured before vs.
after this ticket's change only (Tickets 002/003 NOT yet applied):

| Commanded | Before (old 8mm) | After (live formula) | Δ (shift from removing the coast over-anticipation) |
|---|---|---|---|
| 45°  | +0.29° | +5.11° | +4.82° |
| 90°  | +4.02° | +8.92° | +4.90° |
| 180° | +13.03° | +16.30° | +3.27° |
| 300° | +23.75° | +27.10° | +3.35° |

The absolute miss INCREASES after this ticket alone — expected and called
out by the ticket's own Testing section: the old 8mm constant's ~3.3-4.9°
under-anticipation was partially cancelling the separate, angle-proportional
over-rotation caused by the `rotationalSlip=0.92` arc inflation running
against an un-scrubbed ideal plant (Ticket 002's defect, item 2 in
`clasi/issues/sim-turn-undershoot.md`). Removing the coast-formula's own
error (this ticket) *isolates* the remaining proportional term for Ticket
002 to fix; the two tickets must be judged together (Ticket 004's sweep),
per `architecture-update.md` Step 5/4c.

**Live-read confirmation** (acceptance criterion 2): with the OLD hardcoded
constant, `SET yawAccMax=360` (vs. default 720) shifted the settled RT 9000
arc by +3.65mm (97.73→101.38mm) since the fixed anticipation no longer
matched the now-different actual ramp-down. With the NEW formula, the same
`SET yawAccMax=360` change left the settled arc unchanged (101.38mm both
times) — anticipation and actual coast, now both driven by the same live
`cfg.yawAccMax`, cancel exactly. This is possible only if `beginRotation()`
re-reads `_cfg.yawAccMax` (a `const RobotConfig&` live reference,
`Planner.h:194`) on every call, not a cached/compile-time value.

**HIL flag**: `PlannerBegin.cpp` is ARM-and-sim-shared — this change alters
real-hardware `RT` behavior (previously stopping ~3.3-4.9° short of the
commanded encoder-arc target at low angles; now stops closer to the pure
geometric target). Sim-validated only, per architecture-update.md Open
Question 1 — real-hardware validation is deferred to a HIL follow-up.

**Suite**: `uv run python -m pytest` → 2655 passed, 0 failed (clean
baseline reproduced exactly, before and after this ticket's change). No
test required updating beyond `test_rt_slip.py` (in scope for this ticket);
`test_rotation_stop_terminates_spin` (072's signed ROTATION path),
`test_069_rt_90deg_body_scrub.py`, and `test_tour1_geometry.py`'s
pre-existing `xfail(strict=True)` (still xfailing, unaffected — Ticket
004's to flip) were all run and confirmed unaffected.
