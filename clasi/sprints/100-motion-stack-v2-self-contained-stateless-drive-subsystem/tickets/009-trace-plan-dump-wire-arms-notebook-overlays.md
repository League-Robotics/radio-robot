---
id: '009'
title: Trace/plan-dump wire arms + notebook overlays
status: open
use-cases: [SUC-011]
depends-on: ['007']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Trace/plan-dump wire arms + notebook overlays

## Description

Wire `PlanDumpRequest`/`PlanRecord`/`MotionTrace` (schema declared in
ticket 001, `ERR_UNIMPLEMENTED` since) live through `BinaryChannel`, plus
host-side notebook overlay tooling. Follows the exact "declare then
implement" sequencing this project already used for config/get/stream
(095->096) and pose/otos (095->098/099).

## Acceptance Criteria

- [ ] `BinaryChannel::handlePlanDump()` (new): a `PlanDumpRequest`
      (`CommandEnvelope` arm 18) against a multi-segment ring returns one
      correlated `PlanRecord` (`ReplyEnvelope` arm 10) per ring entry
      (`goal`/`anchor`/`v_eff`/`duration`/`exit_speed`/`entry_speed`/
      `replan_count`), sharing `corr_id`.
- [ ] `StreamControl.trace` (existing declared field) arms `MotionTrace`
      (`ReplyEnvelope` arm 11) emission at the TLM period, sourced from
      `bb.motionTrace` (committed by the adapter each pass since ticket
      007 — verify this commit is already wired; if not, add it here).
- [ ] `MotionTrace` decodes to a valid `TrackRecord` whose `StepInput`
      replays bit-exact at tier 0 (ticket 006's replay harness) — the
      concrete cross-tier interpretability proof.
- [ ] `Telemetry`'s existing ~166B budget is untouched (grep-verifiable:
      no new field added to `message Telemetry` in `protos/
      telemetry.proto`).
- [ ] Host-side notebook overlay tooling (`tests/notebooks/`): plots a
      dumped plan table (`PlanRecord`-derived) against a streamed
      `MotionTrace` on the same axes.
- [ ] HITL or sim: both flows (`PlanDumpRequest` round trip, `MotionTrace`
      stream-and-decode) demonstrated end to end.
- [ ] `uv run python -m pytest` passes.

## Testing

- **Existing tests to run**: `uv run python -m pytest`.
- **New tests to write**: `PlanDumpRequest` round-trip test (multi-
  segment ring -> correlated `PlanRecord`s); `MotionTrace` decode +
  tier-0 replay test.
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: follow the exact "declare in ticket 001, implement live
here" sequencing already established in this tree — `BinaryChannel`
replies `ERR_UNIMPLEMENTED` for these two arms from ticket 001 through
ticket 007; this ticket makes them live.

**Files to modify**:
- `source/commands/binary_channel.{h,cpp}` (new handlers)
- `source/subsystems/drivetrain.{h,cpp}` (verify/add the `bb.motionTrace`
  commit if not already wired by ticket 007)
- `host/robot_radio` (pb2 regeneration + a small plotting helper)

**Testing plan**: tier-1 sim (`PlanDumpRequest` round trip, `MotionTrace`
decode+replay); a notebook demonstrating the overlay.

**Documentation updates**: none beyond the notebook itself.
