---
id: 009
title: Trace/plan-dump wire arms + notebook overlays
status: done
use-cases:
- SUC-011
depends-on:
- '007'
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

- [x] `BinaryChannel::handlePlanDump()` (new): a `PlanDumpRequest`
      (`CommandEnvelope` arm 18) against a multi-segment ring returns one
      correlated `PlanRecord` (`ReplyEnvelope` arm 10) per ring entry
      (`goal`/`anchor`/`v_eff`/`duration`/`exit_speed`/`entry_speed`/
      `replan_count`), sharing `corr_id`.
- [x] `StreamControl.trace` (existing declared field) arms `MotionTrace`
      (`ReplyEnvelope` arm 11) emission at the TLM period, sourced from
      `bb.motionTrace` (committed by the adapter each pass since ticket
      007 — verify this commit is already wired; if not, add it here).
- [x] `MotionTrace` decodes to a valid `TrackRecord` whose `StepInput`
      replays bit-exact at tier 0 (ticket 006's replay harness) — the
      concrete cross-tier interpretability proof.
- [x] `Telemetry`'s existing ~166B budget is untouched (grep-verifiable:
      no new field added to `message Telemetry` in `protos/
      telemetry.proto`).
- [x] Host-side notebook overlay tooling (`tests/notebooks/`): plots a
      dumped plan table (`PlanRecord`-derived) against a streamed
      `MotionTrace` on the same axes.
- [x] HITL or sim: both flows (`PlanDumpRequest` round trip, `MotionTrace`
      stream-and-decode) demonstrated end to end.
- [x] `uv run python -m pytest` passes.

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

## Completion Notes

### Deviations from the ticket's own text (both discovered, not assumed)

1. **`StreamControl.trace` did NOT already exist** — despite this ticket's
   own AC #2 calling it "existing declared field", ticket 100-001 never
   added it (only `PlanDumpRequest`/`PlanRecord`/`MotionTrace` themselves
   were declared; `StreamControl` still had only `binary`/`period`). Added
   it here as field 3 (`protos/envelope.proto`), regenerated
   `source/messages/envelope.h` + `host/robot_radio/robot/pb2/`. This is a
   natural completion of the arm this ticket owns, not a structural
   deviation — flagged per the team-lead's own dispatch instruction to
   verify and add if missing.
2. **`bb.motionTrace` was NOT wired by ticket 007** — ticket 007's own
   completion notes only mention `bb.lastEvent`/`bb.chainTail`; there was
   no `Drive::TrackRecord` capture or `bb.motionTrace` cell at all. Added
   both here: `Subsystems::Drivetrain::lastRecord_`/`lastRecord()`
   (captured every pass `plan_.step()` runs, in `tick()`, right after the
   `step()` call) and `bb.motionTrace`, published every pass by
   `MainLoop::commit()` (mirrors `bb.lastEvent`'s exact publish shape).
3. **`bb.hasActivePlan`/`bb.activePlanRecord`/`bb.planRingGoals`+`Count`
   are new blackboard cells this ticket adds** (not called for verbatim by
   the ticket's file list, which only named `binary_channel.{h,cpp}` and
   `drivetrain.{h,cpp}`, but `blackboard.h`/`main_loop.cpp` are the natural
   surface for them). Design rationale: `BinaryChannel` must never hold a
   `Subsystems::*` reference (SUC-006 — blackboard.h's own file header:
   "holds NO subsystem pointer of any kind"), so `handlePlanDump()` cannot
   reach the adapter's real `ring_`/`plan_` directly the way it reaches a
   THROWAWAY `Drive::Drivetrain` for admission. The adapter instead
   publishes (every pass, for free — no new Ruckig solve): the active
   plan's already-solved `PlanRecord` (query calls only) and a
   non-destructive snapshot of `ring_`'s raw, unsolved `Drive::Goal`
   entries (`queue.h`'s `peek()`/`size()`). `BinaryChannel::handlePlanDump()`
   then preview-SOLVES each queued Goal ON DEMAND, at request time only
   (never every pass — a per-pass preview solve of up to 8 queued Goals
   would be permanent hot-path cost for a rarely-requested diagnostic),
   via a throwaway `Drive::Drivetrain`, the SAME pattern `admitSegment()`
   already established, chained the same way `startNextPlan()` chains a
   real pop. A totally empty dump (no active plan, nothing queued) replies
   a single `Ack{q:0}` rather than zero replies, so a corr_id-correlating
   client always sees something back.

### The plan-dump handler

`source/commands/binary_channel.cpp`'s `handlePlanDump()` (dispatched from
the new `CmdKind::PLAN_DUMP` case) — see the function's own doc comment for
the full design. Verified via `tests/sim/unit/test_plan_dump.py` (3 tests):
a 3-segment ring (queued via `send_no_tick()` before a single `tick_for()`
drains `bb.segmentIn` into `ring_` and pops the first) dumps exactly 3
correlated `PlanRecord`s; an empty ring dumps a single `Ack{q:0}`; a single
active plan (the common case) dumps exactly 1.

### The trace emission wiring

`bb.motionTrace` was NOT already committed by the adapter (see Deviation 2
above) — added in this ticket. `source/telemetry/telemetry_tick.cpp` gained
`telemetryEmitTrace()`, called from `tickTelemetry()` immediately after the
regular `telemetryEmitBinary()` push, gated on the new `bb.telemetryTrace`
(written unconditionally by `BinaryChannel::handleStream()` from
`StreamControl.trace`, mirroring `telemetryBinary`'s own not-gated
assignment). One `tlm` push then, iff armed, one `trace` push, same pass,
same period — never a `Telemetry` extension (AC #4).

### Bit-exact replay proof

`tests/sim/drive/test_motion_trace_replay.py` sends a real arc segment
through a REAL tier-1 sim run, arms `StreamControl.trace`, ticks forward
collecting N `(Telemetry, MotionTrace)` pairs, reconstructs the exact
`Drive::StepInput` sequence `Subsystems::Drivetrain::tick()` actually fed
`step()` (working around one documented subtlety: `StepInput.measured`
pose/twist is read from `bb.bodyState` ONE PASS STALE relative to the
same-pass `Telemetry` push — sample `k`'s `measured` comes from `Telemetry`
sample `k-1`, not `k`; `left`/`right` wheel state IS same-pass-fresh, so it
comes from `Telemetry` sample `k` itself; sample `k=0`'s `measured` is the
analytically-known boot-default rest pose), then replays that sequence
through `tests/_infra/drive/replay.py`'s tier-0 `step()` starting from a
fresh `StepState()`. Verified GENUINELY BIT-EXACT — not merely close: every
field (`ref_x/y/theta/v/omega`, `e_along/cross/theta`, `v_trim/omega_trim`,
`v_cmd/omega_cmd`, `wheel_left/right`, `trim_saturated`, `status`) diffs by
exactly `0.0` across all 4 collected samples (confirmed with a throwaway
diagnostic script before tightening the test's own assertions from
`pytest.approx` to plain `==`) — `source/drive/`'s reference sample +
tracker cascade are pure functions of `(plan, StepInput)` alone, and both
binaries (`libfirmware_host`, the sim; `libdrive_host`, tier 0) compile the
IDENTICAL `source/drive/*.cpp` on the same host/compiler.

### Budget check (`scripts/gen_messages.py`'s own report, after this
### ticket's proto growth — `StreamControl.trace` + nothing else new)

```
CommandEnvelope: ... stream=12B (was 10B, +2B for the new trace bool) ...
  worst=id=162B + non-oneof=6B => total=168B  (unchanged)
ReplyEnvelope:   ... plan=62B, trace=102B ... worst=tlm=165B + non-oneof=6B
  => total=171B
```
Both well under the 186B envelope cap — `plan`≈62B (ticket 001 estimated
~85B — leaner in practice) and `trace`≈102B (within the ~90-120B estimate)
both fit with margin. `protos/telemetry.proto`'s `message Telemetry` is
untouched (grep-verified: zero new fields; `MotionTrace`/`PlanRecord` are
separate `ReplyEnvelope` arms, never a `Telemetry` extension).

### Notebook

`tests/notebooks/plan_dump_trace_overlay.ipynb` (new) — drives a real
tier-1 sim, admits an arc segment, dumps its `PlanRecord`, arms
`StreamControl.trace`, ticks across the whole plan collecting 7-8
`MotionTrace` samples spread start-to-settle, and plots the tier-0
`referenceAt()` table (the dumped plan's full reference polyline) against
the streamed `MotionTrace.ref_x/ref_y` points (land exactly on the curve)
and the sim's actual fused pose (`Telemetry.pose`, showing real tracking
error). Executed headless (`jupyter nbconvert --to notebook --execute
--inplace`), zero errors. Outputs: `tests/notebooks/out/
plan_dump_trace_overlay.png` and three companion CSVs
(`_plandump.csv`/`_trace.csv`/`_reftable.csv`).

### Verify

- `just build-clean` (real MICROBIT firmware): clean, FLASH 348140B/364KB
  (93.40%), RAM 120768B/122816B (98.33%) — no measurable growth beyond the
  ticket's own small `StreamControl.trace` field and the new adapter
  getters (all header-only/inline query calls, no new members beyond one
  `msg::MotionTrace lastRecord_` and two blackboard arrays).
- `just build-sim` / `just build-drive`: both clean.
- Full suite, BLOCKING (`uv run python -m pytest -q`): **1467 passed, 2
  skipped, 4 xfailed, 1 xpassed, ZERO failures** (baseline 1463 passed + 4
  new tests this ticket added: 3 in `test_plan_dump.py`, 1 in
  `test_motion_trace_replay.py`; the 2 skipped/4 xfailed/1 xpassed are
  pre-existing, untouched by this ticket).
- Notebook headless execution: clean, see above.

### Files changed

- `protos/envelope.proto` (`StreamControl.trace`; doc-comment updates
  moving plan_dump/plan/trace from "declared only" to "live since 100-009")
- `source/messages/envelope.h`, `source/messages/wire.{h,cpp}` (regenerated)
- `host/robot_radio/robot/pb2/*` (regenerated)
- `source/subsystems/drive_bridge.h` (`wirePose2D()`, `drivePlanRecord()`,
  `driveMotionTrace()`)
- `source/subsystems/drivetrain.{h,cpp}` (`lastRecord()`/`hasActivePlan()`/
  `activePlanRecord()`/`ringGoals()`, `lastRecord_` capture in `tick()`)
- `source/runtime/blackboard.h` (`motionTrace`, `hasActivePlan`,
  `activePlanRecord`, `planRingGoals`/`planRingCount`, `telemetryTrace`)
- `source/runtime/main_loop.cpp` (publish the five new cells in `commit()`)
- `source/commands/binary_channel.{h,cpp}` (`handlePlanDump()`,
  `sendPlanRecord()`, `PLAN_DUMP` dispatch case, `handleStream()`'s
  `telemetryTrace` write)
- `source/telemetry/telemetry_tick.cpp` (`telemetryEmitTrace()`)
- `tests/_infra/sim/firmware.py` (`drain_reply_store()` Python wrapper for
  the already-existing `sim_drain_reply_store()` C symbol)
- `tests/sim/unit/_binary_envelope.py` (`send_multi()`)
- `tests/sim/unit/test_plan_dump.py` (new, 3 tests)
- `tests/sim/drive/test_motion_trace_replay.py` (new, 1 test)
- `tests/notebooks/plan_dump_trace_overlay.ipynb` (new) +
  `tests/notebooks/out/plan_dump_trace_overlay*` (new outputs)
