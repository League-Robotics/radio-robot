---
id: '003'
title: 'Planner: migrate DISTANCE (D) onto JerkTrajectory'
status: open
use-cases:
- SUC-002
depends-on:
- '002'
github-issue: ''
issue: planner-motion-planning-via-vendored-ruckig.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Planner: migrate DISTANCE (D) onto JerkTrajectory

## Description

This is the first `Subsystems::Planner` integration ticket and the
simplest case: `DISTANCE` (the `D` wire verb) is turn-in-place-free
(`omega = 0` always) and has a target known entirely at `apply()` time (no
live pose/observation dependency), so it uses ONLY the position-control
solve-to-rest (architecture-update.md Decision 2, "Pattern A"), with no
separate cruise phase. Fixing this goal kind directly addresses the
hardware-confirmed bug: `D 200 200 1000` overshooting to ~292 mm/s then
reversing ~16 mm after `EVT done`.

## Implementation Plan

**Approach** (architecture-update.md Decisions 2, 4, 8, and Step 5's "What
Changed"):
1. `Planner` gains two `Motion::JerkTrajectory` members (linear,
   rotational) alongside the existing `ramp_` — both are constructed here
   even though only the linear channel does real work for `DISTANCE`; the
   rotational channel gets a trivial (or skipped) zero-target plan, since
   `omega = 0` always for this goal kind.
2. `Planner::apply()`'s `DISTANCE` case: instead of `stageGoal(v, 0.0f,
   DriveMode::DISTANCE, cmd)` (which calls `ramp_.setTarget()`), solve the
   linear channel's position-control-to-rest with `target_position =
   distance`, `max_velocity = min(speed, v_body_max)` (the commanded
   speed, per-call — NOT the global ceiling, Decision 2's revision).
   `copyCallerStops()`/`appendStop()` (the implicit `STOP_DISTANCE`/
   `STOP_TIME` synthesis) are UNCHANGED — this ticket does not touch
   `stops_[]`/`baseline_`/`captureBaseline()` at all (Decision 4: Ruckig
   only shapes velocity, `Motion::evaluateStopCondition()` stays the
   authoritative completion signal).
3. `Planner::tick()`'s dispatch: for the `DISTANCE` goal kind specifically,
   sample the linear channel (`at_time(elapsed)`) instead of calling
   `ramp_.advance()`/`applyStopAnticipation()`. Do NOT yet generalize this
   to a full `mode_ == GO_TO` binary split — that only becomes correct once
   ticket 004 (TIMED/VELOCITY/STREAM) and ticket 005 (TURN/ROTATION) also
   land; until then, this ticket needs a goal-kind-aware (not `mode_`-
   aware) check scoped narrowly to `DISTANCE`. Document this explicitly as
   a KNOWN INTERMEDIATE STATE in a code comment, cleaned up by ticket 005
   (the last goal-kind ticket) once the dispatch can safely collapse to
   `mode_ == GO_TO` vs. not.
4. The `stopping_`/SMOOTH ramp-down branch, for `DISTANCE` specifically: if
   a stop condition fires before the linear channel's own plan has
   naturally converged to rest, re-solve a fresh velocity-control
   decel-to-rest (`target_velocity = 0`) from the channel's own current
   sampled state (Decision 8 — seed from the channel's last sample, never
   `leftObs`/`rightObs`) and switch to sampling that. In the common case
   (the live encoder roughly tracks the plan), the channel has typically
   already converged to rest by the time the stop fires, making this a
   no-op.
5. `applyStopAnticipation()`'s `STOP_DISTANCE` branch becomes DEAD CODE for
   `DISTANCE` goals once this ticket lands (the goal kind no longer routes
   through it) — but do NOT delete the branch or the function yet.
   `TIMED`/`VELOCITY`/`STREAM`/`TURN`/`ROTATION` still call into
   `applyStopAnticipation()` until tickets 004/005 land. Deleting it now
   would break those goal kinds. Ticket 005 is where the function is
   finally deleted in full, once every goal kind has migrated off it.
6. `holdTwistCommand()` is unchanged — still packs `(v, omega)` into
   `msg::BodyTwist3`, now sourced from the `JerkTrajectory` sample for
   `DISTANCE` instead of `ramp_.currentV()`.
7. **[Revision 2, post-stakeholder-design-discussion] Divergence-triggered
   replan for `DISTANCE`'s linear channel** (architecture-update.md
   Decision 10): each tick, while `STOP_DISTANCE`/`STOP_TIME`'s stop
   condition has NOT fired, compare the linear channel's own remembered
   plan-remaining against `Motion::remainingToStop()`'s measured remaining.
   When they diverge beyond `kDivergenceThreshold` (a new, ticket-owned
   constant — value characterized on the bench, ticket 007): if the
   divergence is below `kGrossDivergenceThreshold` (also ticket-owned),
   call `retarget()` with a new remaining computed as
   `target - (measured + v * tau)` (`tau` = `kDeadTime`, the SAME two-pass
   dead-time constant `applyStopAnticipation()` already defines — reuse it,
   do not redefine it under a new name); if the divergence is at or above
   `kGrossDivergenceThreshold`, call `reanchor()` seeded from measured
   position/velocity (acceleration = 0). Enforce, AT THE `Planner` CALL
   SITE (not inside `JerkTrajectory` — ticket 002's boundary decision):
   (a) skip the replan entirely once the stop condition has FIRED; (b) skip
   if the (dead-time-projected) measured remaining is `<= 0` — never solve
   backward, this preserves this ticket's own no-reverse property; (c)
   rate-limit to at most one replan per `kMinReplanInterval` (ticket-owned).

**Files to modify**: `source/subsystems/planner.h`, `source/subsystems/
planner.cpp`. No `msg::*`/proto change.

**Testing plan**: extend/add Planner-level tests exercising `apply()` +
repeated `tick()` calls for a `DISTANCE` goal, asserting the sampled/
commanded velocity trace never goes negative and the goal completes at the
commanded distance. Existing `test_motion_commands*.py`/
`test_motion_overshoot_regression.py` `D`-specific assertions must stay
green (equal or tighter than before). **[Revision 2]** Extend the D-goal
Planner-level test with a divergence-replan scenario: inject (at the
test-fixture level, not via the sim plant, which tracks too closely to
diverge naturally) a synthetic observation showing more remaining than the
plan expects, confirm a retarget (or reanchor, if the injected divergence
is gross) fires and the resulting trace still never reverses; confirm the
guard skips replan once the stop condition has fired, and skips a
would-be-backward replan when the (projected) measured remaining is `<= 0`.

**Documentation updates**: `planner.h`'s class comment gains a note on the
two coexisting motion-generation mechanisms and which goal kinds each
currently serves (updated again by tickets 004/005 as more goal kinds
migrate).

## Acceptance Criteria

- [ ] `Planner::apply()`'s `DISTANCE` case stages a position-control
      Ruckig solve-to-rest on the linear channel instead of
      `ramp_.setTarget()`.
- [ ] `Planner::tick()` samples the linear channel for `DISTANCE` goals
      each tick; `stops_[]`/`baseline_`/`Motion::evaluateStopCondition()`
      are untouched and remain the authoritative completion signal.
- [ ] Sim: a Planner-level test samples the full commanded velocity trace
      for a `DISTANCE` goal and asserts it is `>= 0` throughout (mirrors
      `test_ruckig_smoke.py`'s own no-reverse assertion, against the real
      goal-staging path).
- [ ] A stop firing before the plan's own natural convergence re-solves a
      decel-to-rest trajectory seeded from the channel's own last sample
      (never `leftObs`/`rightObs`) and completes with no reverse.
- [ ] `applyStopAnticipation()` and `Motion::VelocityRamp` are UNCHANGED as
      code — still fully intact and still serving `TIMED`/`VELOCITY`/
      `STREAM`/`TURN`/`ROTATION`/`GOTO_GOAL` goal kinds, none of which are
      touched by this ticket.
- [ ] `test_motion_overshoot_regression.py`'s existing `D` bar is not
      regressed (equal or tighter).
- [ ] Full sim suite green; no new xfail introduced.
- [ ] **[Revision 2]** A divergence beyond `kDivergenceThreshold` (with the
      stop condition not yet fired) triggers a `retarget()` (or
      `reanchor()`, if beyond `kGrossDivergenceThreshold`) call seeded per
      Decision 10/Decision 8's revision; the commanded trace still never
      reverses.
- [ ] **[Revision 2]** The three guards (stop-not-fired, no-reverse-target,
      deadband+rate-limit) are all enforced at the `Planner` call site, not
      inside `JerkTrajectory`.
- [ ] **[Revision 2]** Under a synthetic tracking-lag scenario that would
      otherwise stall the plan short of the target, `D`'s completion mode is
      `STOP_DISTANCE` firing (`reason=dist`), not the `STOP_TIME` safety net
      (sim-level proof here; the bench-level crisp criterion is ticket
      007's).

## Testing

- **Existing tests to run**: `test_motion_commands*.py`,
  `test_motion_overshoot_regression.py`, full `uv run pytest`.
- **New tests to write**: a Planner-level `D`-goal trajectory-sampling
  test (see SUC-002's acceptance criteria in `usecases.md`). **[Revision 2]**
  Plus the divergence-replan/guard scenarios above (synthetic-observation
  based, not sim-plant based).
- **Verification command**: `uv run pytest tests/sim -k "distance or D "`
  then the full `uv run pytest`.
