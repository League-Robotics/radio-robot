---
id: "001"
title: "RT coast anticipation from ramp dynamics"
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ""
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

- [ ] `Planner::beginRotation()`'s `kRtCoastArc = 8.0f` constant is removed
      and replaced by a local computation: `rate = min(cfg.yawRateMax,
      kRtRate)` (unchanged, already-local), `coastAngleDeg = rate*rate /
      (2.0f * cfg.yawAccMax)`, `coastArc = coastAngleDeg * kDegToRad *
      (trackwidth * 0.5f)`, `stopArc = arc - coastArc`.
- [ ] The coast-anticipation quantity is computed from live
      `cfg.yawAccMax`/`cfg.yawRateMax`/`cfg.trackwidth`, not a hardcoded
      constant — confirmed by re-reading the values at runtime, not just
      at compile time (i.e. a `SET yawAccMax=<other>` changes the computed
      coast arc on the next `RT`).
- [ ] No change to `RT`'s wire grammar, `StopCondition`, or
      `MotionBaseline` — this is a pure internal recompute inside
      `beginRotation()`. `StopCondition::Kind::ROTATION`'s signed
      `omegaSign` handling (072) is read from, never modified.
- [ ] `BodyVelocityController`'s ramp behavior (trapezoid decel, dt-based
      tick decrement) is unchanged — read from (via live `RobotConfig`)
      to derive the formula, not modified.
- [ ] If the continuous formula alone does not land Ticket 004's ≤~1°
      sweep bar, a small, DOCUMENTED empirical correction factor is added
      on top of it (not a reversion to an undocumented magic constant, and
      not an escalation to a discrete tick-replay of the BVC's own ramp
      without first trying the cheaper fix) — see Design Rationale
      Decision 1.
- [ ] `tests/simulation/unit/test_rt_slip.py`'s
      `test_rt_arc_no_slip_matches_geometry` no longer hardcodes
      `coast_mm = 8.0  # kRtCoastArcMm`; it computes the expected coast arc
      from the same formula at the test's own `tw_mm=83.0`/cruise rate.
      Module docstring updated. `test_rt_arc_larger_with_slip` and
      `test_rt_slip_compensation_ratio` (which assert RATIOS between two
      RT runs with the SAME coast applied to both) require no change —
      confirm by running them and inspecting their assertions.
- [ ] Any stale "100°/s" prose in `PlannerBegin.cpp` comments or
      `docs/protocol-v2.md` referencing the old constant's assumption is
      corrected in place (drive-by, same file already being edited).
- [ ] Full suite (`uv run python -m pytest`) passes at 2655 + this
      ticket's net new/changed test count, with zero unexplained failures
      other than the sweep test that Ticket 004 (not this ticket) adds.

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
