---
id: "002"
title: "Rt::Blackboard state and command planes"
status: open
use-cases: [SUC-001, SUC-006]
depends-on: ["001"]
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

# Rt::Blackboard state and command planes

## Description

Implement `Rt::Blackboard` exactly as specified in `architecture-update.md`'s
Reference code — the aggregate struct owning every state-plane cell
(`motor[kPortCount]`, `drivetrain`, `encoderPose`, `fusedPose`, `planner`,
`otos`/`otosValid`, and the four current-config cells) and every
command-plane queue instance (`statementsIn`, `driveIn`,
`motorIn[kPortCount]`, `configIn`, `poseResetIn`, `motorResetIn[kPortCount]`,
`otosSetPoseIn`). This is pure data — no method computes anything; it holds
**no subsystem pointer of any kind** (SUC-006). Also define the two payload
types the Blackboard's queues carry that don't already exist as `msg::`
types: `Rt::PoseResetCommand` and `Rt::ConfigDelta`.

## Acceptance Criteria

- [ ] `Rt::Blackboard` compiles and default-constructs with every cell
      zero/default-initialized; the header includes only `messages/*.h`,
      `runtime/queue.h`, and `subsystems/hardware.h` (for the `kPortCount`
      constant only, per the Reference code).
- [ ] Every state cell listed in `architecture-update.md`'s Reference code
      is present with the exact `msg::` type named there.
- [ ] Every command-plane queue is present with the exact vehicle
      (`Mailbox` vs. `WorkQueue`) and capacity named there: `statementsIn`
      (`WorkQueue`, 16), `configIn` (`WorkQueue`, 16), `poseResetIn`
      (`WorkQueue`, 4), `driveIn`/`motorIn[i]`/`otosSetPoseIn` (`Mailbox`,
      capacity 1).
- [ ] `Rt::PoseResetCommand` (`kind` enum `{kSetPose, kResetBaseline}` +
      `msg::SetPose pose`) and `Rt::ConfigDelta` (`target` enum
      `{kDrivetrain, kMotor, kPlanner, kOdometer}` + `port` + a field-mask
      placeholder) are defined in `source/runtime/blackboard.h` exactly as
      specified.
- [ ] Grepping `source/runtime/blackboard.h` for any `Subsystems::` type
      used as a pointer/reference member (as opposed to the one
      `kPortCount` constant reference) returns nothing.

## Implementation Plan

**Approach.** New header `source/runtime/blackboard.h`, namespace `Rt`,
built directly from `architecture-update.md`'s Reference code block. No
`.cpp` — pure aggregate, no logic.

**Files to create:**
- `source/runtime/blackboard.h`

**Files to modify:** none.

**Testing plan:**
- New `tests/sim/unit/runtime_blackboard_harness.cpp` — instantiate a
  `Rt::Blackboard`, exercise a representative post/take round-trip on
  `driveIn`, `configIn`, `poseResetIn`, and `motorIn[0]`, and confirm the
  state cells default-construct to zero/default `msg::` values.
- New `tests/sim/unit/test_runtime_blackboard.py` driving the harness.
- **Verification command**: `uv run pytest tests/sim/unit/test_runtime_blackboard.py`

**Documentation updates:** none beyond `architecture-update.md` (already
written).
