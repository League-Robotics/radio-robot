---
id: '007'
title: 'robot_loop: single-point telemetry-frame assembly from primary sources; feed
  StateEstimator from sources not frame_'
status: done
use-cases:
- SUC-005
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# robot_loop: single-point telemetry-frame assembly from primary sources; feed StateEstimator from sources not frame_

## Description

Stakeholder-directed (Eric, 2026-07-25) code-quality refactor of
`src/firm/app/robot_loop.cpp`'s telemetry-frame assembly and
state-estimator feeding. Continuous with 123-004 (the telemetry
migration), which last touched this exact code.

`RobotLoop::cycle()` currently:

(a) uses the telemetry `frame_` (`App::Telemetry::Frame`) as a
per-cycle scratchpad, staging encoder/twist/pose/otos/line/color into
it piecemeal across `updateTlm()`, `updateLineColor()`,
`applyOtosSample()`, and the cycle body;

(b) sets telemetry flags via `tlm_.setFlag()` scattered across those
same three functions (robot_loop.cpp ~L200-206, L226-227, L735-736,
L797, L807); and

(c) builds `Motion::StateEstimator::Input` by copying ~16 fields BACK
OUT of `frame_` (L750-768) to feed `stateEstimator_.update()`.

The telemetry frame is a SERIALIZATION artifact, not a data source and
not a scratchpad. The fix:

1. Feed `stateEstimator_.update()` from PRIMARY SOURCES (motor/encoder
   readings from `motorL_`/`motorR_`, `odom_` pose, the OTOS
   `Devices::PoseReading` + presence, the fused body twist from
   `BodyKinematics::forward()`), NOT by reading fields out of `frame_`.
   The `Motion::StateEstimator::Input` struct stays (motion/ can't
   include app/), but it is filled from sources.
2. Assemble `frame_` in ONE block immediately before `tlm_.emit()`,
   from those same primary sources — remove the piecemeal staging
   scattered through the cycle.
3. Set ALL telemetry flags together in that single assembly block — no
   `tlm_.setFlag()` calls scattered elsewhere.

Note for the implementer: the stakeholder does NOT care where
`emit()` sits in the cycle, only that assembly immediately precedes
emit and is sourced from primaries. Existing behavior to PRESERVE: the
119-005 encoder same-generation freshness property (fresh L and R read
the same cycle), the protocol-v4 §7.2 ack-timing contract (enqueue ack
rides this frame, MOVE-completion ack rides next frame), and the
123-004 cycleBusy/cyclePeriod primary-frame timing. Any timing shift
from moving emit must be consciously preserved/verified, not
accidental.

## Acceptance Criteria

- [x] StateEstimator is fed from primary sources; the copy-out-of-frame_
      block (current L750-768) is gone.
- [x] `frame_` is assembled in a single point immediately before emit;
      no piecemeal frame_ field staging remains scattered across
      updateTlm/updateLineColor/applyOtosSample/cycle body.
- [x] All telemetry flags are set in one place; no scattered setFlag
      calls. (Exception, discovered via the harness and documented in
      code: kFlagFaultMoveTimeout/kFlagFaultShapingDisabled depend on
      moveQueue_.tick()'s own per-cycle output, which is not known until
      AFTER assembleFrame()/emit() run (tick() must stay positioned there
      so a completion ack rides the next frame, protocol-v4 §7.2). The
      harness's SUC-054/119-001 scenarios query tlm_.flags() as LIVE
      state and require both bits to already reflect THIS cycle's tick()
      by the time cycle() returns -- deferring them to next cycle's
      assembly (as first attempted) broke both scenarios. These two are
      set via direct tlm_.setFlag() calls immediately after tick(), same
      position/logic as before this ticket -- see assembleFrame()'s own
      doc comment in robot_loop.h for the full explanation.)
- [x] The sim test suite (`uv run python -m pytest`) passes, including
      the app_robot_loop harness scenarios.
- [x] 119-005 encoder same-generation freshness and the protocol-v4
      §7.2 ack-timing behavior are preserved (verify against
      app_robot_loop_harness scenarios / their comments).
- [x] Coding standards honored (units in `// [unit]` comment tags,
      lowerCamelCase functions, etc.).

## Implementation Plan

- *Approach:* Refactor `RobotLoop::cycle()` (and `updateTlm()`,
  `updateLineColor()`, `applyOtosSample()`) so that (1) a single
  primary-sources read feeds both `Motion::StateEstimator::Input` and
  the telemetry frame assembly, and (2) frame assembly + flag-setting
  happens in one block immediately before `tlm_.emit()`. Do not change
  wire schema, message content, or the 123-004 cycleBusy/cyclePeriod
  placement — this is an internal-structure refactor only.
- *Files:* `src/firm/app/robot_loop.cpp` (and its header if any
  signatures need to change for the primary-sources read); no schema
  (`.proto`/`wire.h`) changes expected.
- *Testing:* Full sim suite (`uv run python -m pytest`), with particular
  attention to `app_robot_loop_harness` scenarios covering encoder
  freshness (119-005) and ack timing (protocol-v4 §7.2).
- *Documentation:* Update `src/firm/app/DESIGN.md` only if it currently
  describes the piecemeal staging/flag-setting structure being removed;
  otherwise none.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full sim suite,
  including `app_robot_loop_harness` scenarios).
- **New tests to write**: none required by this ticket unless the
  refactor reveals an untested behavior; this is a structural refactor
  with preserved behavior, verified against existing harness scenarios.
- **Verification command**: `uv run python -m pytest`
