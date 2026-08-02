---
id: 008
title: tick() explicit move-lifecycle state machine
status: done
use-cases:
- SUC-004
depends-on:
- '007'
github-issue: ''
issue: planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# tick() explicit move-lifecycle state machine

## Description

Redesign `Planner::tick()` (`planner.cpp:451`, ~240 lines) from
interacting booleans (`occupied`, `hasMoved`, `settling`,
`decelLatched`, `closingIssued`, `stallTicks`, `carryValid_`,
`activeBoundary_`) into an explicit lifecycle enum + dispatcher, per
`planner-honesty-pass-50ms-period-tick-state-machine-limits-
reduction.md` item 2: states `Idle` / `Draining` / `Breakaway` /
`Tracking` / `Stopping`, with `MovePhase` (Accel/Hold/Decel) as
`Tracking`'s sub-phase. Events: profile-complete, arrived (epsilon + at
rest), stall-window expiry, timeout, estop, replace, queue-empty.
Completions become transition actions emitting `TickResult`.
`hasMoved`/`settling` dissolve into state identity; `decelLatched` stays
a `MovePhase` latch. The settle-confirm defer path (and its `Settling`
state) is deleted with `requireSettle` (removed properly in ticket 009,
since it's a `PlannerLimits` field — this ticket removes the STATE and
behavior, ticket 009 removes the config fields that drove it).

Sequenced after ticket 007 (duty-stage deletion) so there are fewer
booleans/states to represent, and after the composition-root/50 ms
period work so the timing constants used inside `tick()` are already
correct — avoiding re-tuning the same code twice.

Behavior-preserving except where the source issue says otherwise;
`planner_scenarios_test`/`planner_noise_test` are the gate.

## Acceptance Criteria

- [x] `tick()` dispatches over an explicit lifecycle enum with one
      visible transition table (states: `Idle`, `Draining`, `Breakaway`,
      `Tracking`, `Stopping`; `MovePhase` Accel/Hold/Decel as
      `Tracking`'s sub-phase).
- [x] `hasMoved`/`settling` booleans removed, replaced by state
      identity; `decelLatched` remains a `MovePhase` latch.
- [x] The settle-confirm defer path and its `Settling` state are
      deleted (arrival completion already covers it).
- [x] `planner_scenarios_test` and `planner_noise_test` (all ctest
      suites) green — the exactness gates are the primary behavior
      guard for this rewrite.
- [x] The completion-priority order (previously implicit in code order)
      is documented explicitly as a transition table, not left as
      "whatever order the `if`-chain happens to check first."

## Testing

- **Existing tests to run**: full `planner_tests` ctest suite,
  especially `planner_scenarios_test`/`planner_noise_test`.
- **New tests to write**: state-transition unit tests covering each
  documented event (profile-complete, arrived, stall-window expiry,
  timeout, estop, replace, queue-empty).
- **Verification command**: ctest for `motion_tests`; `uv run pytest`
  for the sim-side scenario suites.

## Implementation Plan

**Approach**: introduce the lifecycle enum and rewrite `tick()` as a
dispatcher over per-state handlers, preserving every completion path
that today's boolean interactions encode — cross-check against the
`planner_scenarios_test` suite line by line before considering the
rewrite complete.

**Files to create/modify**:
- `src/motion/planner/planner.{h,cpp}` (state machine rewrite)
- `src/motion/planner/planner_types.h` (state enum, `TickResult`)
- New state-transition unit tests

**Testing plan**: full `planner_tests` suite; new state-transition
tests per documented event.

**Documentation updates**: `planner.h`'s own header comment gains a
description of the state machine and its transition table.
