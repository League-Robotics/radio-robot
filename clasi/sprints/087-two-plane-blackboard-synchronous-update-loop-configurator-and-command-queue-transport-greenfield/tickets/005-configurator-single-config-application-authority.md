---
id: "005"
title: "Configurator: single config-application authority"
status: open
use-cases: [SUC-002, SUC-003, SUC-005]
depends-on: ["002", "003", "004"]
github-issue: ""
issue: plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Configurator: single config-application authority

## Description

Implement `Configurator` per `architecture-update.md`'s Reference code and
Decision 4: constructed with references to `Drivetrain`, `PoseEstimator`,
`Planner`, and `Hardware` — **the one deliberate exception** to "no
subsystem pointers" in this design. It folds `Rt::ConfigDelta` entries
popped from the Blackboard's `configIn` `WorkQueue` into a per-target
desired-config copy, calls that target's existing `configure()` when
changed, and publishes the resulting current config into the Blackboard's
config state cells (`drivetrainConfig`, `motorConfig[]`, `plannerConfig`,
`odometerConfig`). Exposes `pending(bb)`/`applyOne(bb)` for the loop's
slack phase (ticket 007) and `publish(bb)` for boot-time seeding.

## Acceptance Criteria

- [ ] `Configurator`'s constructor takes exactly `Drivetrain&`,
      `PoseEstimator&`, `Planner&`, `Hardware&`, plus boot-default configs
      (per the Reference code's
      `Configurator configurator(drivetrain, poseEstimator, planner,
      hardware, ...)`), and holds no other subsystem reference.
- [ ] `applyOne(bb)` pops exactly one `ConfigDelta` from `bb.configIn` per
      call (never more), folds it into the addressed target's desired-config
      copy, calls `configure()` on that target only when the fold actually
      changes anything, and writes the resulting current config into the
      matching `bb.*Config` cell.
- [ ] `publish(bb)` seeds all four `bb.*Config` cells from the
      Configurator's current per-target config without requiring a delta to
      have been posted first (boot-time use, per the Reference code's
      `configurator.publish(bb)` call before the loop starts).
- [ ] `pending(bb)` returns true iff `bb.configIn` is non-empty (used by the
      loop's slack `else if` branch).
- [ ] No other component calls `configure()` directly on any subsystem —
      grepping `source/commands/` and `source/runtime/command_router.*` for
      `.configure(` outside `configurator.cpp` returns nothing.
- [ ] A unit test constructs a `Configurator` against real (not mocked)
      `Drivetrain`/`PoseEstimator`/`Planner`/`Hardware` instances, posts a
      `ConfigDelta` for each of the four targets in turn, calls
      `applyOne()` the corresponding number of times, and asserts each
      target's own `config()` now reflects the delta and the matching
      Blackboard cell was published.
- [ ] A unit test posts two deltas for the **same** target back-to-back and
      confirms both fold into the same `configure()` call's worth of
      change when drained in sequence (deterministic FIFO fold order) —
      grounding Decision 3's "current published config, not
      current+pending" validation baseline from the *caller's* (`SET`
      handler's) side, not the Configurator's own internal fold order.

## Implementation Plan

**Approach.** New `source/runtime/configurator.{h,cpp}`. Uses
`Rt::ConfigDelta` (from `blackboard.h`, ticket 002) and calls the four
targets' existing `configure()`/`config()` (`PoseEstimator`/`Hardware`'s
added in ticket 004; confirm during implementation whether `Drivetrain`/
`Planner` already have `configure()`/`config()` today — Grounding did not
explicitly confirm `Planner` does. If either is missing, add it here and
flag the addition as a deviation from this ticket's stated scope, per
sprint 085's own precedent for documenting such deviations.)

**Files to create:**
- `source/runtime/configurator.{h,cpp}`

**Files to modify:** none expected (built entirely on tickets 002-004's
faceplates); see the `Planner`/`Drivetrain` `configure()`/`config()`
contingency above.

**Testing plan:**
- New `tests/sim/unit/configurator_harness.cpp` + `test_configurator.py`,
  constructing real subsystem instances (no mocks, consistent with this
  sprint's testability goal) and exercising the acceptance criteria above
  directly.
- **Verification command**: `uv run pytest tests/sim/unit/test_configurator.py`

**Documentation updates:** none beyond `architecture-update.md`.
