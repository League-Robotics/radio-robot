---
id: '013'
title: 'Cleanup: delete parked files, reserve retired fields, retire heading gains + governRatio segment path'
status: open
use-cases: [SUC-015]
depends-on: ['012']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Cleanup: delete parked files, reserve retired fields, retire heading gains + governRatio segment path

## Description

Remove the old motion stack now that the new one is bench- and
field-proven (tickets 011/012 both signed off): delete the parked files,
reserve the retired `PlannerConfig` proto fields, and retire the
`governRatio()` segment-mode call path. This ticket only removes code —
it introduces no new capability.

## Acceptance Criteria

- [ ] `source/motion/segment_executor.{h,cpp}`, `segment.h`,
      `motion_baseline.h`, `stop_condition.{h,cpp}` deleted from disk
      (not merely excluded from the build — they have been off the
      active call path since ticket 007).
- [ ] `protos/planner.proto` marks `PlannerConfig`'s retired
      `heading_kp`(13)/`heading_kd`(14) field numbers `reserved`, never
      reassigned (matching the `motion` field-5 precedent already in
      `envelope.proto`) — `source/messages/planner.h` regenerated
      accordingly.
- [ ] The `governRatio()` SEGMENT-MODE call path is retired (the DIRECT/
      escape-hatch path's own `governRatio()` call, for TWIST/WHEELS, is
      UNCHANGED — only the segment-mode invocation, superseded by
      `Drive::`'s own saturate/clamp cascade, is removed).
- [ ] `git grep -w 'segment_executor\|stop_condition\|SegmentExecutor'`
      outside `clasi/` (history/planning docs) returns nothing under
      `source/`.
- [ ] Open Question 1 from `architecture-update.md` is resolved:
      evaluate whether `DrivetrainConfig.v_wheel_max`/`steer_headroom`
      (fields 10/11) should be marked `DEPRECATED` (matching
      `vel_gains`/`min_wheel`'s existing precedent) now that only the old
      `Drivetrain`'s DIRECT-mode `governRatio()`/saturate path remains
      as a consumer. Per `architecture-update.md`'s own Impact analysis,
      DIRECT mode is UNCHANGED and still legitimately uses
      `BodyKinematics::saturate()` with `DrivetrainConfig`'s fields — if
      still in live use, do NOT deprecate; document the finding in
      completion notes either way.
- [ ] The full sim suite (`uv run python -m pytest`) passes after
      deletion.
- [ ] `just build` (firmware) succeeds after deletion; `arm-none-eabi-
      size` shows the expected flash REDUCTION (the parked files' code,
      previously dead-stripped per `architecture-update.md` Decision 3,
      is now not compiled at all) — record the delta in completion
      notes.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`; `just
  build` + `arm-none-eabi-size`.
- **New tests to write**: none — this ticket removes code; existing
  tests (with any now-dead-file-specific tests also removed) are the
  verification.
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: this ticket only deletes/reserves — it introduces no new
capability. Before deleting anything, double-check tickets 011 and 012
both actually recorded a passing sign-off in their completion notes (the
issue's own rule: "delete only after bench sign-off").

**Files to delete**:
- `source/motion/segment_executor.h`, `source/motion/
  segment_executor.cpp`
- `source/motion/segment.h`
- `source/motion/motion_baseline.h`
- `source/motion/stop_condition.h`, `source/motion/stop_condition.cpp`
- Any now-orphaned test file that exclusively exercised the deleted
  files (e.g. `tests/sim/unit/segment_executor_harness.cpp` and its
  pytest wrapper, if no longer referenced)

**Files to modify**: `protos/planner.proto` (reserved fields),
`source/messages/planner.h` (regenerated), `source/subsystems/
drivetrain.{h,cpp}` (governRatio segment-mode call-site removal, if any
residue remains from ticket 007).

**Testing plan**: full sim suite; firmware build + size check.

**Documentation updates**: none required (the `docs/protocol-v3.md`
follow-up remains flagged from ticket 001, still the team-lead's to
schedule).

---
## DEFERRED to sprint 101 (2026-07-13)

Not completed in sprint 100. Bench diagnosis found the DeviceBus firmware's
**heading feedback is broken** (raw OTOS heading frozen during an open-loop
spin; fused heading garbage/resetting; OTOS re-init commands accepted-inert),
so closed-loop turn accuracy cannot be validated or tuned until that is fixed.
That debugging — and the arc/turn accuracy sweeps, camera-verified field runs,
and the parked-file cleanup that depends on field sign-off — is re-scoped into
sprint 101 (debugging). Carried forward, superseded by 101's tickets.
