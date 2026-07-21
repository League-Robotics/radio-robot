---
id: '003'
title: 'Proto surgery: envelope/config/planner/motion deletions + telemetry.proto
  rewrite'
status: open
use-cases: [SUC-045, SUC-047, SUC-048, SUC-049]
depends-on: ["002"]
github-issue: ''
issue: telemetry-frame-tightening-amendment-to-gut-s1.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Proto surgery: envelope/config/planner/motion deletions + telemetry.proto rewrite

## Description

Rewrites the wire schema to match the now-deleted app-layer subsystems
(ticket 002) and implements the telemetry-frame-tightening amendment's
full spec. This is the data-model-defining ticket for the sprint: every
downstream ticket (004 persisted-tuning, 005 firmware reshape, 006 sim,
007 host decode) depends on the message shapes this ticket produces.
`build.py` regenerates `messages/*.h`/`wire.cpp` and the host `pb2`
modules from these `.proto` files — never hand-edit generated output.

Two proto-level edits are distinct and easy to conflate (verified during
planning): `envelope.proto`'s `ConfigDelta.planner` is the oneof **arm**
referencing `PlannerConfigPatch` (at `envelope.proto:150`); `config.proto`'s
`message PlannerConfigPatch { ... }` (at `config.proto:155-163`) is the
**type itself**. Both are deleted, in different files, for the same
underlying reason (nothing survives S1 that persists/patches planner
config).

## Acceptance Criteria

- [ ] `envelope.proto`: `Move` deleted from the `CommandEnvelope.cmd`
      oneof (was arm 20); `20` added to that oneof's existing `reserved
      2, 3, 4, 5, 7 to 12, 14 to 18;` list. `ConfigDelta.planner`
      (`PlannerConfigPatch planner = 3`) deleted from the `ConfigDelta`
      oneof; `3` added to a `reserved` list on that oneof (new — none
      exists there today). `Twist` (arm 19) is untouched (sprint.md
      Architecture Decision 5).
- [ ] `config.proto`: `message PlannerConfigPatch { ... }` deleted
      wholesale.
- [ ] `planner.proto` and `motion.proto` deleted wholesale (verified
      contents: `planner.proto` defines `DriveMode`, `StopStyle`,
      `Origin`, `CmpOp`, `StopKind`, `HeadingSourceMode` enums and
      `StopCondition`/`VelocityGoal`/`GotoGoal`/`TurnGoal`/
      `DistanceGoal`/`TimedGoal`/`RotationGoal`/`StreamGoal`/
      `PlannerCommand`/`PlannerState`/`PlannerConfig` messages;
      `motion.proto` defines `MotionSegment` + `MotionStatus`). Confirm
      no other `.proto` still `import`s either file before deleting.
- [ ] `telemetry.proto`: full rewrite per
      `telemetry-frame-tightening-amendment-to-gut-s1.md`'s spec —
      `EncoderReading{position, velocity, time}`,
      `OtosReading{x, y, heading, v_x, v_y, omega, time}`, single
      `flags` bit-string (all 16 bits per the amendment's numbering,
      including bit 15 reserved for sprint 116's MOVE-timeout fault —
      declare the bit position now, do not wire it), single
      `ack_corr`/`ack_err` pair (no ring), `Pose2D pose`/`BodyTwist3
      twist` always-present fields, packed `line`/`color` `uint32`
      words, `DriveMode` relocated in from the deleted `planner.proto`.
      Clean renumber (every field ≤ 15, no `reserved` — no external
      client depends on the old numbers). `AckEntry`, `AckStatus`,
      `ExecutorState`, `HeadingSourceStatus` deleted.
      `TelemetrySecondary` untouched (sprint.md Open Questions #3 — do
      not prune `ts_left`/`ts_right` in this ticket).
- [ ] `src/scripts/gen_boot_config.py`: `defaultPlannerConfig()` emission
      and its planner-field helper functions removed (verified present
      today at gen_boot_config.py:642 and several helper docstrings
      earlier in the file).
- [ ] `python build.py` regenerates `messages/telemetry.h`, `wire.cpp`,
      and the host `pb2` modules without error from the edited protos
      (a full green firmware build is NOT expected yet — `robot_loop.cpp`
      etc. don't reference the new shapes until ticket 005; this
      criterion is about the **generator step succeeding**, not the
      whole firmware linking).
- [ ] `gen_messages.py`'s worst-case size table is inspected for the
      rewritten `Telemetry` message and the real measured number is
      recorded in `telemetry.proto`'s own header comment (replacing the
      amendment issue's ~137 B estimate) — sprint.md Open Questions #2.

## Implementation Plan

**Approach**: Edit protos in dependency order — delete `planner.proto`/
`motion.proto` first (after confirming no remaining importer), then
`envelope.proto`/`config.proto` (independent of the telemetry rewrite),
then the `telemetry.proto` rewrite (the biggest, self-contained edit),
then `gen_boot_config.py`. Run `python build.py` after each proto edit
to catch a missed `import`/reference early rather than at the end.

**Files to modify**: `src/protos/envelope.proto`, `src/protos/config.proto`,
`src/protos/telemetry.proto`; delete `src/protos/planner.proto`,
`src/protos/motion.proto`; `src/scripts/gen_boot_config.py`.

**Files NOT to modify**: anything under `src/firm/messages/` (generated —
regenerates from the above).

**Testing plan**: `python build.py` (generator step). No pytest suite
targets protos directly at this ticket's boundary — wire round-trip
tests for the new message shapes are ticket 009's job (once the app
layer and sim harness both consume the new shapes, a round-trip test is
meaningful; writing one now against nothing that constructs the new
messages yet would be premature).

**Documentation updates**: `telemetry.proto`'s own header comment gets
the measured frame-size number (acceptance criterion above). No
`docs/architecture/` doc is touched (sprint.md's own Architecture
section is this sprint's architecture-update artifact).
