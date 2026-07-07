---
id: "004"
title: "Faceplate regularization: PoseEstimator and Hardware blackboard wiring"
status: open
use-cases: [SUC-001, SUC-002, SUC-004, SUC-006]
depends-on: ["002"]
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

# Faceplate regularization: PoseEstimator and Hardware blackboard wiring

## Description

Give `PoseEstimator` and `Hardware` (`NezhaHardware`/`SimHardware`) the
faceplate members `architecture-update.md`'s Step 3/Step 5 identify as
missing today. `PoseEstimator` gains `configure()`/`config()` and a
drainable `poseResetIn` queue consumed inside `tick()`, **reusing** the
existing pending-flag mechanism (`setPose()`/`resetEncoderBaseline()`)
rather than replacing it — the phantom-jump-avoidance logic stays exactly
where it is today (Decision 7: target-drained resets keep the entangled
coherence logic inside the estimator). `Hardware` gains a uniform
`config()`/`state()` (today per-`Hal::Motor`) and consumes a per-port
`Rt::Mailbox<msg::MotorCommand> motorIn[kPortCount]` array (Decision 2)
plus a `bool motorResetIn[kPortCount]` flag array (`ZERO enc`'s
target-drained reset, idempotent, reusing the existing `resetPosition()`
staging) in place of its current addressed-command input path.

## Acceptance Criteria

- [ ] `PoseEstimator::configure(...)`/`config()` exist and round-trip a
      config value. Confirm during implementation whether a
      `PoseEstimatorConfig`-equivalent `msg::` struct already exists under
      `source/messages/`; add one if not (Grounding did not confirm one
      exists — this is the one open item this ticket must resolve, not
      predicted in `architecture-update.md`, which stays at module level).
- [ ] `PoseEstimator::tick()`'s signature gains a
      `Rt::WorkQueue<Rt::PoseResetCommand,4>& poseResetIn` parameter;
      `tick()` drains it (FIFO, all entries each pass) and dispatches
      `kSetPose` to the existing `setPose()`, `kResetBaseline` to the
      existing `resetEncoderBaseline()` — no change to either method's own
      internals or the pending-flag/phantom-jump-avoidance mechanism.
- [ ] `Hardware::tick()`'s signature takes a per-port
      `Rt::Mailbox<msg::MotorCommand>` array and a per-port
      `bool motorResetIn[]` array; consumes each port's mailbox uniformly
      (no addressed-dispatch branch) and applies a pending
      `motorResetIn[i]` flag by calling the existing per-motor
      `resetPosition()`, clearing the flag afterward (idempotent — "reset
      twice = reset once").
- [ ] `Hardware` (both `NezhaHardware` and `SimHardware`) exposes a uniform
      `config()`/`state()` at the `Hardware` faceplate level (not only
      per-`Hal::Motor` as today).
- [ ] `source/subsystems/pose_estimator.h` and
      `hardware.h`/`nezha_hardware.h`/`sim_hardware.h` include only
      `messages/*.h` and `runtime/queue.h` — never `blackboard.h`.
- [ ] Existing `test_pose_estimator.py`, `test_sim_hardware.py`, and
      `test_hardware_seam.py` (and their harnesses) pass with the updated
      signatures, each updated to construct bare `Rt::WorkQueue`/
      `Rt::Mailbox` instances directly — no full `Blackboard` needed
      (SUC-002).
- [ ] A new test confirms `SI`'s re-anchor (`kSetPose`) and `ZERO enc`'s
      re-baseline (`kResetBaseline`) each still avoid the phantom jump:
      posting a reset command and ticking produces the same before/after
      pose relationship the existing `setPose()`/`resetEncoderBaseline()`
      tests already assert — this ticket's queue plumbing must not regress
      that guarantee, even though the SI/ZERO *wire-level routing* itself
      is ticket 006's job.

## Implementation Plan

**Approach.** Modify `pose_estimator.{h,cpp}`, `hardware.h`,
`nezha_hardware.{h,cpp}`, `sim_hardware.{h,cpp}`. Reuse `Rt::PoseResetCommand`
from `blackboard.h` (ticket 002) as the reset-queue payload type. Confirm/add
any missing `msg::` config type for `PoseEstimator`.

**Files to modify:**
- `source/subsystems/pose_estimator.{h,cpp}`
- `source/subsystems/hardware.h`
- `source/subsystems/nezha_hardware.{h,cpp}`
- `source/subsystems/sim_hardware.{h,cpp}`
- `source/messages/odometer.h` (or the correct home for a `PoseEstimator`
  config type — confirm during implementation)
- `tests/sim/unit/pose_estimator_harness.cpp`, `sim_hardware_harness.cpp`,
  `hardware_seam_harness.cpp`, and their `.py` drivers

**Testing plan:**
- Update harnesses to construct bare `Rt::WorkQueue<Rt::PoseResetCommand,4>`
  and a `Rt::Mailbox<msg::MotorCommand>` array directly.
- Add drain-order and idempotent-reset-flag test cases.
- Re-run the phantom-jump-avoidance assertions already present in
  `test_pose_estimator.py` against the new queue-driven entry point.
- **Verification command**: `uv run pytest tests/sim/unit/test_pose_estimator.py tests/sim/unit/test_sim_hardware.py tests/sim/unit/test_hardware_seam.py`

**Documentation updates:** none beyond `architecture-update.md`.
