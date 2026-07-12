---
id: '007'
title: 'THE CUTOVER: wafer adapter, wire admission, host proxy decomposition, golden-TLM regen'
status: open
use-cases: [SUC-009]
depends-on: ['006']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# THE CUTOVER: wafer adapter, wire admission, host proxy decomposition, golden-TLM regen

## Preconditions (execution-order, verify before starting)

1. **Sprint 099 ("Restore pose estimation: OTOS, encoders, and delayed
   camera fixes") must be EXECUTED AND CLOSED.** This ticket's adapter
   consumes `bb.bodyState`/`bb.poseStepped`/`PoseEstimator::
   lastPoseStep()`, all landed by 099. Before starting, re-read
   `clasi/sprints/099-restore-pose-estimation-otos-encoders-and-delayed-camera-fixes/architecture-update.md`'s ACTUAL landed state (not the plan) —
   confirm the exact field names/shapes in the closed sprint's tickets,
   not this document's paraphrase of the plan. If 099 is not yet closed,
   STOP and escalate to the team-lead rather than guessing.
2. **The robot must be USB-attached** for this ticket's HITL smoke test
   (as of this sprint's planning, only the relay dongle is connected).
   Do NOT request USB access until every host-side/tier-1 step below is
   green — front-load everything that does not need hardware first.

## Description

The atomic cutover: rewrite `Subsystems::Drivetrain` into the thin wafer
adapter over `source/drive/`, wire admission for the `segment`/`replace`
arms, host proxy decomposition (`legacy_translate.py`'s
`primitives_for_move()` + the new `SEG` verb), the build-list swap
(parking, not yet deleting, `segment_executor`/`stop_condition`), and
golden-TLM regeneration. This is the single highest-stakes ticket in the
sprint — it is the one point the live firmware call path changes.

## Acceptance Criteria

- [ ] `Subsystems::Drivetrain` (`source/subsystems/drivetrain.{h,cpp}`)
      is rewritten to hold a `Drive::Drivetrain` (immutable config), the
      current `Drive::MotionPlan` value, `Drive::StepState`, plan-start
      timestamp, and `ChainTail` — zero control math anywhere in this
      file (greppable: no Kanayama/IK/saturation math outside `source/
      drive/` after this ticket).
- [ ] Boundary conversions implemented exactly per
      `architecture-update.md` M7: `msg::MotorState` -> `Drive::
      WheelState`; `bb.bodyState` -> `Drive::BodyState`; `bb.poseStepped`
      -> `StepInput.poseStep`/`poseStepTheta`; `Drive::WheelVelocities`
      -> `msg::MotorCommand` via `hardware_.motor(i).apply()` (unchanged
      staging path).
- [ ] `Status` reactions implemented: `REPLAN_DUE` -> call `replan()`,
      swap the held plan; `DONE_*` -> pop next ring segment (seeded from
      the REFERENCE per ticket 005's handoff spec) or neutral the
      motors; `ABORT_*` -> flush ring, re-anchor `ChainTail`, emit a
      populated `EventNotify` (`seg_seq`/`status`/`e_final_pos`/
      `e_final_theta`).
- [ ] Wire admission: a `segment`/`replace` `CommandEnvelope` with
      `primitive=true` converts to a `Drive::Goal`, `admit()`/`plan()`
      run, `Verdict::OK` stages the plan, any other verdict replies a
      typed `ERR` and leaves the queue untouched. `primitive=false` is
      REJECTED after cutover (typed `ERR`, not silently accepted).
- [ ] DIRECT/escape-hatch mode (`setTwist`/`setWheelTargets`/
      `setNeutral`, `governRatio()` for TWIST/WHEELS) is UNCHANGED —
      explicitly verify (e.g. `git diff` review) this code path is not
      touched by this ticket's diff.
- [ ] Host proxy: `host/robot_radio/robot/legacy_translate.py` gains
      `primitives_for_move()` (decomposes a legacy `MOVE` into `<=3`
      `MotionSegment{primitive=true}` primitives; document the exact
      decomposition strategy and any deviation from the old single-
      segment translation, per this file's own "transcribe, don't
      re-derive... document deviations" discipline) and a
      `segment_for_seg()`-style builder for real arcs; `host/robot_radio/
      robot/legacy_verbs.py` registers the new `SEG` verb.
- [ ] Build-list swap: `source/motion/segment_executor.{h,cpp}`/
      `segment.h`/`motion_baseline.h`/`stop_condition.{h,cpp}` are
      removed from the ACTIVE call path (`Subsystems::Drivetrain` no
      longer references them) but stay ON DISK (parked — ticket 013
      deletes them later, gated on bench+field sign-off).
- [ ] Golden-TLM regeneration: the sim's zero-error-path golden TLM
      output is regenerated as an explicit, REVIEWED step — completion
      notes document what changed, why, and confirm the change is
      expected given the cutover (never a silent re-baseline).
- [ ] HITL (robot on the stand, USB-attached): a `segment` command drives
      an arc and a pivot to completion; encoders/`vel=` show plausible,
      direction-correct motion.
- [ ] HITL: an infeasible `segment` (e.g. a pivot with nonzero exit
      speed) NACKs at the wire with the specific `Verdict`; the queue is
      untouched.
- [ ] HITL: a legacy text `MOVE`/`S`/`T`/`D` command (translated
      host-side via `primitives_for_move()`) still drives correctly
      through the new adapter.
- [ ] `uv run python -m pytest` passes (full sim suite, including
      regenerated golden TLM).

## Testing

- **Existing tests to run**: full `uv run python -m pytest`; every prior
  ticket's harnesses (001-006).
- **New tests to write**: tier-1 sim tests for queue precedence, wire
  admission NACK behavior, DIRECT-mode-unchanged regression; the three
  HITL flows above.
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: sequence as (1) rewrite the adapter and get it compiling +
passing sim tests with `source/drive/` wired in, no hardware; (2)
implement host proxy decomposition and test it against the sim; (3)
regenerate and review golden TLM; (4) ONLY THEN request USB access and
run the HITL smoke test. This ordering minimizes the USB-attached
session's length and risk.

**Files to modify**:
- `source/subsystems/drivetrain.{h,cpp}` (rewrite)
- `source/runtime/main_loop.{h,cpp}`/`source/main.cpp`/`tests/_infra/
  sim/sim_api.cpp` (build-list references, as needed)
- `host/robot_radio/robot/legacy_translate.py`
- `host/robot_radio/robot/legacy_verbs.py`

**Files to leave in place but unreferenced**: `source/motion/
segment_executor.{h,cpp}`, `segment.h`, `motion_baseline.h`,
`stop_condition.{h,cpp}`.

**Testing plan**: tier-1 sim tests; golden-TLM regeneration + review;
the three HITL acceptance criteria, run on the stand per
`.claude/rules/hardware-bench-testing.md`.

**Documentation updates**: `docs/protocol-v3.md` (or the current
protocol doc) follow-up is flagged, not performed here
(`architecture-update.md` Open Question 4) — note this explicitly in
completion notes so the team-lead schedules it.
