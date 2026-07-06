---
id: "003"
title: "Planner terminal decel/coast anticipation (phase 2)"
status: open
use-cases: [SUC-001, SUC-002]
depends-on: ["002"]
github-issue: ""
issue: motion-turn-drive-terminal-overshoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Planner terminal decel/coast anticipation (phase 2)

## Description

Phase 2 of the motion-overshoot fix — depends on ticket 002 (the motor-loop
root fix) landing first, per stakeholder-mandated order. `Subsystems::
Planner::pursueSteer()` already anticipates its stop for GOTO/PURSUE
(`planner.cpp:310-334`): it computes `vCap = sqrtf(2.0f * config_.a_decel *
dRemaining)` and clamps commanded speed every tick, so the wheel is already
near zero when `STOP_POSITION` fires. No equivalent exists for `DISTANCE`/
`TURN`/`ROTATION` — those only react once their stop condition fires
(`planner.cpp`'s `tick()`, the `stopping_` branch), handing ticket 002's
fixed motor loop a harder "arrest from full speed" problem than it needs to
solve.

This ticket extends the SAME anticipation pattern to `DISTANCE`/`TURN`/
`ROTATION`, via a new shared query rather than a fourth independent copy of
the geometry (architecture-update.md Design Rationale 2): add a
"remaining-to-stop" query alongside `Motion::evaluateStopCondition()` in
`source/motion/stop_condition.{h,cpp}`, built from the SAME per-kind
geometry `evaluateStopCondition()` already computes for `STOP_DISTANCE`/
`STOP_ROTATION`/`STOP_HEADING` (encoder-average delta, encoder-differential
delta, fused-heading delta respectively). `Planner::tick()`'s
stop-evaluation loop calls this new query once per active stop condition,
alongside (not instead of) the existing `evaluateStopCondition()` call, and
applies a `vCap`-style speed/rate cap while the goal is still running (not
yet in the `stopping_` SMOOTH ramp-down).

`pursueSteer()`'s own `STOP_POSITION`-specific `vCap`/curvature-clamp logic
is left as-is this ticket (GOTO's world-frame-XY geometry is a different
shape from the other three kinds' scalar distance/angle — see Open
Question 2; not unified this sprint).

## Acceptance Criteria

- [ ] `Motion::stop_condition` gains a new, pure, additive query function
      (no change to `evaluateStopCondition()`'s existing signature/behavior)
      that reports remaining distance/rotation/heading-error for
      `STOP_DISTANCE`/`STOP_ROTATION`/`STOP_HEADING`, built from the exact
      same per-kind geometry `evaluateStopCondition()` already computes —
      not a second, independently-derived copy.
- [ ] `Planner::tick()`'s stop-evaluation loop applies a `pursueSteer()`-
      style anticipatory speed (`DISTANCE`) / angular-rate (`TURN`/
      `ROTATION`) cap while the corresponding stop condition is still open,
      using the new query.
- [ ] `pursueSteer()`/GOTO's own behavior is provably unchanged (existing
      GOTO tests still pass unmodified).
- [ ] Ticket 001's regression tests (now passing post-002) show further
      tightened residual/overshoot with this ticket's anticipation added —
      i.e., measurably less "arrest from full speed" work left for the
      motor loop.
- [ ] `tests/sim/unit/test_planner.py` is extended to cover the new
      anticipation behavior for at least one `DISTANCE` and one `TURN`/
      `ROTATION` case.
- [ ] No wire/message-schema change (`msg::PlannerCommand`/`StopCondition`
      untouched) — this ticket is entirely inside `Motion`/`Subsystems::
      Planner`'s existing contracts.

## Implementation Plan

**Approach**: Read `pursueSteer()` (`planner.cpp:310-334`) as the reference
shape. Add the new query to `motion/stop_condition.{h,cpp}` first (host-
testable in isolation via the existing `stop_condition_harness.cpp`-style
unit tests), then wire it into `Planner::tick()`'s stop-evaluation loop for
the three goal kinds that lack anticipation today.

**Files to modify**:
- `source/motion/stop_condition.{h,cpp}` — new additive query function.
- `source/subsystems/planner.{h,cpp}` — apply the anticipatory cap in
  `tick()`'s stop-evaluation loop for `DISTANCE`/`TURN`/`ROTATION`.

**Testing plan**:
- New/extended unit tests for the `stop_condition` query (mirroring
  `stop_condition_harness.cpp`'s existing pattern).
- Extend `tests/sim/unit/test_planner.py` for the anticipation behavior.
- Re-run ticket 001's regression tests to confirm improvement.
- Re-run existing GOTO/pursue tests to confirm no regression there.

**Documentation updates**: None at the wire/protocol level. If
`docs/protocol-v2.md` §10 describes stop behavior in a way this changes
observably (it should not — this is an internal trajectory-shaping change,
not a wire-visible one), note and correct it explicitly rather than leaving
a stale description.
