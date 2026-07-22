---
id: '002'
title: Motion::StopCondition module
status: in-progress
use-cases:
- SUC-050
- SUC-051
- SUC-052
- SUC-054
depends-on:
- '001'
github-issue: ''
issue: protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Motion::StopCondition module

## Description

New module `Motion::StopCondition` (`src/firm/motion/stop_condition.{h,cpp}`
+ `DESIGN.md`) — recreates a `motion/` directory containing only this one
file, mirroring `kinematics/`'s existing small-pure-computation pattern (S1
deleted the *old*, much larger `motion/` wholesale; this is a fresh, tiny
directory, not a partial restoration). Purpose: reports whether one bounded
motion's stop condition or timeout has been met. Boundary: inside — kind
(TIME/DISTANCE/ANGLE) + threshold + the activation-time baselines (clock
time, `Odometry::pathLength()`, `Odometry::theta()`) + the per-cycle
comparison; outside — what happens when it reports true (ticket 005's
`MoveQueue` job), where the readings it compares against come from (passed
into `tick()`, never owned). No dependency on `MoveQueue`, `Drive`, or the
wire types. `theta()` is already verified UNWRAPPED (`theta_ +=
headingDelta`, no modulo anywhere in `odometry.cpp`) — the ANGLE kind
diffs `theta()` against its own activation baseline directly, no wrap
handling needed.

Per sprint.md's Architecture Open Question 1: pin down and test whichever
convention is chosen for zero/negative stop-value thresholds (recommend
mirroring `timeout`'s own `> 0` rule for `distance`/`angle`) — this is a
`StopCondition`-level decision since it governs what `tick()` does with a
degenerate threshold, even though the wire-level `ERR_BADARG` rejection
itself happens in ticket 006's `handleMove()`.

## Acceptance Criteria

- [ ] `Motion::StopCondition` constructed with a kind, threshold, and
      `timeout`; captures activation baselines (now, `pathLength()`,
      `theta()`) at construction/activation.
- [ ] `tick(now, pathLength, theta)` reports the kind-specific
      stop-condition-met outcome and the timeout-met outcome as two
      distinguishable results (the caller needs to tell them apart to set
      `kFlagFaultMoveTimeout` correctly) — not a single collapsed bool.
- [ ] TIME kind fires at/after the commanded elapsed time.
- [ ] DISTANCE kind fires when `|pathLength() - baseline| >= threshold`.
- [ ] ANGLE kind fires when `|theta() - baseline| >= threshold`, no modulo/
      wrap applied.
- [ ] TIMEOUT fires independent of kind whenever elapsed time reaches
      `timeout`, whether or not the kind-specific condition has also fired
      that same cycle (kind-specific takes precedence if both are true the
      same cycle — document and test the tie-break explicitly).
- [ ] Zero/negative stop-value threshold behavior is decided and tested
      (Open Question 1) — not left as untested, implicit behavior.
- [ ] Module has zero dependency on `MoveQueue`, `Drive`, or `msg::*` wire
      types — constructible and testable with hand-fed numbers alone.
- [ ] `src/firm/motion/DESIGN.md` written, matching the boundary
      description above.

## Testing

- **Existing tests to run**: none directly touch this new module; full
  firmware/sim build (`python build.py`) must stay clean.
- **New tests to write**: a new sim unit-test harness (e.g.
  `motion_stop_condition_harness.cpp` + `test_motion_stop_condition.py`,
  following the existing `app_*_harness.cpp`/`test_app_*.py` pairing
  convention in `src/tests/sim/unit/`) covering all 4 outcomes above plus
  the tie-break and threshold-edge-case tests.
- **Verification command**: `python build.py && uv run python -m pytest
  src/tests/sim/unit/test_motion_stop_condition.py`
