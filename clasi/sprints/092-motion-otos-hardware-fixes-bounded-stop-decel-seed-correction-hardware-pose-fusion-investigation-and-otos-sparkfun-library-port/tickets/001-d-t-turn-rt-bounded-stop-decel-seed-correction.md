---
id: '001'
title: D/T/TURN/RT bounded stop-decel seed correction
status: open
use-cases: [SUC-001]
depends-on: []
github-issue: ''
issue: d-t-terminal-reverse-persists-decel-reseed-from-plan-velocity.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# D/T/TURN/RT bounded stop-decel seed correction

## Description

Sprint 089's Ruckig migration reduced but did not eliminate the D/T
terminal reverse-motion bug: `armDistanceStopDecel()`/`armVelocityStopDecel()`/
`armRotationalStopDecel()` (`source/subsystems/planner.cpp:730-770`) seed
their stop-triggered decel-to-rest re-solve from
`Motion::JerkTrajectory`'s own remembered `lastVelocity_` (089 Decision 8's
"never seed from measured state" contract) -- which
`JerkTrajectory::sample()` (`jerk_trajectory.cpp:170-179`) always overwrites
with the PLAN's theoretical velocity, never the measured one. On hardware
the bench-tuned velocity PID tracks loosely enough (measured ~250-310 mm/s
on a commanded 200) that this seed under-estimates the real wheel speed at
the exact stop-arm instant, so the decel-to-rest trajectory commands a
lower velocity than the wheel is actually running, and the PID brakes the
difference into 11-23 mm of reverse creep after `EVT done`
(`clasi/issues/d-t-terminal-reverse-persists-decel-reseed-from-plan-velocity.md`,
089-007 bench measurement).

This ticket implements the architecture-update.md Decision 1-4 design: a
new, narrow entry point on `Motion::JerkTrajectory` that nudges the seed
velocity toward a caller-supplied measured velocity, bounded to a
ticket-owned magnitude cap, invoked ONLY at the three stop-decel-arm call
sites -- never at the routine per-tick `sample()`, never at the goal-start
solve, and never inside the existing divergence-triggered replan
(`maybeReplanDistance()`/`maybeReplanRotational()`, 089 Decision 10, which
structurally cannot reach this failure mode -- see architecture-update.md
Grounding). This is a genuine, one-shot, bounded exception to 089 Decision
8's general seeding contract, not a reopening of it.

**Read `architecture-update.md` Decisions 1-4 and the Grounding section in
full before starting** -- they establish exactly why this must be a new
`JerkTrajectory` entry point (not caller-side math), why the clamp is
symmetric (not one-directional), and why firing exactly once per stop-arm
event (not per tick) is what keeps this from reopening the 087-009
limit-cycle bug class.

## Acceptance Criteria

- [ ] `Motion::JerkTrajectory` gains one new entry point implementing the
      bounded seed correction (exact name/signature is this ticket's own
      implementation decision) -- `solveToRest()`/`solveToVelocity()`/
      `retarget()`/`reanchor()`'s existing signatures and seeding behavior
      are unchanged.
- [ ] `armDistanceStopDecel()`, `armVelocityStopDecel()`, and
      `armRotationalStopDecel()` all use the new entry point (linear
      channel for the first two, rotational channel for all three where
      applicable), passing an averaged measured velocity derived from
      `leftObs`/`rightObs` (mirroring `maybeReplanDistance()`'s existing
      per-wheel-averaging pattern).
- [ ] The correction is a SYMMETRIC magnitude clamp
      (`clamp(measured - believed, -cap, +cap)`), not a one-directional
      (only-correct-upward) rule (Decision 2).
- [ ] **Sim, BLOCKING**: a Planner-level test injects a synthetic
      post-arm observation showing measured velocity persistently faster
      than the channel's own plan-believed velocity at the exact tick a
      stop-triggered decel is armed (mirroring 089-006's synthetic-
      observation pattern in `tests/sim/unit/planner_harness.cpp`) and
      asserts the resulting decel trajectory's sampled velocity (a) never
      reverses sign, and (b) converges MONOTONICALLY to rest -- no
      dip-then-rebound (the 087-009 limit-cycle signature).
- [ ] **Sim, BLOCKING**: a second scenario proves the correction is bounded
      -- an extreme synthetic divergence (far beyond anything physically
      plausible) still produces a seed correction capped at the
      ticket-owned constant, not a value that fully snaps to measurement.
- [ ] **Sim, BLOCKING**: at least one scenario covers the rotational
      channel (`TURN` or `RT`) with the same injected-divergence pattern,
      not just the linear channel.
- [ ] Full `uv run python -m pytest tests/sim` is green.
- [ ] **Bench, BEST-EFFORT**: `D 200 200 1000` and `T 200 200 1000` on the
      stand (direct USB serial -- the relay dongle is unplugged this
      sprint) -- measured reverse encoder motion after `EVT done` is
      re-measured against the 089-007 baseline (11-21 mm / 19-23 mm) and
      recorded, whether or not it is fully eliminated. If the bench step
      cannot be completed (robot wedges/latches, hardware unavailable, or
      a regression surfaces that cannot be resolved in-sprint), record the
      sim result as the completed deliverable and file a fresh
      `clasi/issues/` follow-on instead of blocking on it.
- [ ] If the monotonic-convergence sim scenario CANNOT be made to pass
      cleanly (i.e. the bounded correction cannot be proven safe against
      the 087-009 signature), do not ship a blind control change --
      surface this via the sprint's exception protocol for a stakeholder
      decision between this approach, retuning the velocity PID (issue's
      option (b)), or an accepted terminal-tolerance bar (option (c)).

## Implementation Plan

**Approach**:
1. Read `architecture-update.md` Grounding + Decisions 1-4 in full.
2. Add the new bounded-correction entry point to
   `source/motion/jerk_trajectory.{h,cpp}`, following the `retarget()`/
   `reanchor()` doc-comment style (state the seeding contract exception
   explicitly, cross-reference this ticket/architecture-update.md).
3. Thread `leftObs`/`rightObs` into `armDistanceStopDecel()`/
   `armVelocityStopDecel()`/`armRotationalStopDecel()` (already in scope at
   every call site inside `Planner::tick()`) and compute the averaged
   measured velocity for the affected channel(s).
4. Pick an initial cap value per channel (linear: informed by the 089-007
   bench numbers, e.g. in the 50-110 mm/s range; rotational: a
   correspondingly reasoned rad/s value), named as a ticket-owned constant
   (e.g. `kStopDecelSeedCorrectionCap`/`kRotStopDecelSeedCorrectionCap`),
   justified in a comment, not silently invented.
5. Write the sim tests (synthetic-observation pattern, mirroring 089-006 in
   `tests/sim/unit/planner_harness.cpp`/`test_planner.py`) BEFORE declaring
   done -- these are the blocking acceptance gate.
6. Attempt the bench step; record the outcome honestly regardless of
   result.

**Files to modify**: `source/motion/jerk_trajectory.h`,
`source/motion/jerk_trajectory.cpp`, `source/subsystems/planner.h`,
`source/subsystems/planner.cpp`, `tests/sim/unit/jerk_trajectory_harness.cpp`
(if the new entry point needs its own unit-level coverage),
`tests/sim/unit/planner_harness.cpp`, `tests/sim/unit/test_planner.py`.

**Testing plan**:
- **Existing tests to run**: full `uv run python -m pytest tests/sim`
  (regression guard for 089's own no-reverse trace assertions and every
  existing D/T/TURN/RT scenario).
- **New tests to write**: see Acceptance Criteria's sim bullets above.
- **Verification command**: `uv run python -m pytest tests/sim`.

**Documentation updates**: none expected beyond the new entry point's own
doc comment and this ticket's completion notes recording the chosen cap
value(s) and their justification.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim` (full
  suite).
- **New tests to write**: synthetic-observation divergence-at-stop-arm
  scenarios for `D` (linear) and at least one of `TURN`/`RT` (rotational),
  plus a bounded-correction-cap scenario.
- **Verification command**: `uv run python -m pytest tests/sim`.
