---
id: '004'
title: 'Planner: migrate TIMED/VELOCITY/STREAM (T/R/S) onto JerkTrajectory'
status: open
use-cases:
- SUC-003
depends-on:
- '003'
github-issue: ''
issue: planner-motion-planning-via-vendored-ruckig.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Planner: migrate TIMED/VELOCITY/STREAM (T/R/S) onto JerkTrajectory

## Description

`TIMED` (`T`), `VELOCITY` (`R`, open-loop arc), and `STREAM` (bare `S`)
share the SAME `stageGoal()`/`stopping_`/SMOOTH-ramp-down code path
(`velocityShapedMode()`'s own doc comment) and, unlike `DISTANCE`, have NO
target position known at command time — only a cruise velocity and,
usually, a time/user-supplied stop. This ticket migrates them onto the
two-phase pattern (architecture-update.md Decision 2, "Pattern B": cruise
via velocity-control solve, sustained via Ruckig's own past-duration
hold-at-final-state; a stop-triggered re-solve to rest). Directly fixes the
other half of the hardware-confirmed bug: `T 200 200 1000` reversing ~23 mm
after `EVT done`.

## Implementation Plan

**Approach** (architecture-update.md Decisions 2, 4, 8):
1. `Planner::apply()`'s `TIMED`/`VELOCITY`/`STREAM` cases: instead of
   `stageGoal(v, omega, mode, cmd)` (→ `ramp_.setTarget()`), solve BOTH
   channels' velocity-control-to-cruise (`target_velocity = v` on the
   linear channel, `target_velocity = omega` on the rotational channel;
   `max_velocity` per-call = `min(commandedMagnitude, globalCeiling)` for
   each channel independently, per ticket 002's design). A channel whose
   commanded component is exactly 0 for the whole goal (e.g. `D`-like
   straight `T`'s rotational component) still gets the same solve call
   with `target_velocity = 0` — no special-casing, per Decision 1's
   "always both channels" design.
2. Sampling: each tick, sample both channels' `at_time(elapsed)`. Past each
   channel's own ramp-up duration, sampling holds at the cruise velocity
   for free (Ruckig's own extrapolation, ticket 002) — no additional
   Planner-side "sustain cruise" bookkeeping.
3. Stop-triggered re-solve: when a stop condition fires (`STOP_TIME`, a
   user `stop=` clause, or — for `TIMED` — `duration` elapsing, all
   UNCHANGED, `Motion::evaluateStopCondition()`), re-solve EACH channel's
   velocity-control-to-zero (`target_velocity = 0`) from that channel's own
   current sampled state (Decision 8 seeding — never `leftObs`/`rightObs`)
   and switch to sampling the new trajectory. This is the SAME mechanism
   ticket 003 already built for `DISTANCE`'s own stop-triggered re-solve —
   reuse it, do not reimplement.
4. Extend ticket 003's intermediate goal-kind-aware dispatch check (in
   `tick()`) to also route `TIMED`/`VELOCITY`/`STREAM` through the Ruckig
   channels. `TURN`/`ROTATION`/`GOTO_GOAL` still route through the old
   mechanism until ticket 005 lands — this ticket does NOT yet collapse
   the dispatch to a clean `mode_ == GO_TO` binary (that is ticket 005's
   final cleanup, once nothing but `GOTO_GOAL` is left on the old path).
5. `applyStopAnticipation()`'s `STOP_DISTANCE` branch is already dead
   (ticket 003); this ticket does not exercise or touch its
   `STOP_HEADING`/`STOP_ROTATION` branches, which stay live for `TURN`/
   `ROTATION` until ticket 005. Still do NOT delete
   `applyStopAnticipation()` in this ticket.

**Files to modify**: `source/subsystems/planner.h`, `source/subsystems/
planner.cpp`.

**Testing plan**: extend Planner-level tests to cover `T`/`R`/bare-`S`
goals through cruise AND the stop-triggered re-solve, sampling the full
commanded velocity trace on both channels. Existing `test_motion_commands*
.py`/`test_motion_overshoot_regression.py` `T` assertions must stay green.

**Documentation updates**: `planner.h`'s class comment updated again — now
`TURN`/`ROTATION`/`GOTO_GOAL` are the only remaining goal kinds on the old
mechanism.

## Acceptance Criteria

- [ ] `Planner::apply()`'s `TIMED`/`VELOCITY`/`STREAM` cases stage a
      velocity-control Ruckig cruise solve on both channels instead of
      `ramp_.setTarget()`.
- [ ] Cruise sustains correctly past the ramp-up trajectory's own duration
      (Ruckig's past-duration hold), with no Planner-side "sustain"
      bookkeeping added.
- [ ] A stop condition firing (time-based, user `stop=`, or `T`'s
      `duration`) triggers a re-solve to rest on both channels, seeded from
      each channel's own last sample, converging with no reverse.
- [ ] Sim: a Planner-level test drives a `T` goal through cruise and the
      stop-triggered re-solve, sampling the full commanded velocity trace
      and asserting it is `>= 0` throughout (matching ticket 003's
      assertion style for `D`).
- [ ] `test_motion_overshoot_regression.py`'s existing `D`/`T` bars are not
      regressed (equal or tighter than before this sprint).
- [ ] `TURN`/`ROTATION`/`GOTO_GOAL` code paths and `applyStopAnticipation()`
      remain fully intact and unmodified by this ticket.
- [ ] Full sim suite green; no new xfail introduced.

## Testing

- **Existing tests to run**: `test_motion_commands*.py`,
  `test_motion_overshoot_regression.py`, full `uv run pytest`.
- **New tests to write**: Planner-level `T`/`R`/`S`-goal trajectory-sampling
  tests covering cruise + stop-triggered re-solve (see SUC-003's acceptance
  criteria in `usecases.md`).
- **Verification command**: `uv run pytest tests/sim -k "timed or stream or velocity"`
  then the full `uv run pytest`.
