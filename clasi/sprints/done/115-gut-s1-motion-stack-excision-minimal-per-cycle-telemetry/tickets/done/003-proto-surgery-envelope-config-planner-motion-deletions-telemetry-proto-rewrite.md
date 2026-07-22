---
id: '003'
title: 'Proto surgery: envelope/config/planner/motion deletions + telemetry.proto
  rewrite'
status: done
use-cases:
- SUC-045
- SUC-047
- SUC-048
- SUC-049
depends-on:
- '002'
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

- [x] `envelope.proto`: `Move` deleted from the `CommandEnvelope.cmd`
      oneof (was arm 20); `20` added to that oneof's existing `reserved
      2, 3, 4, 5, 7 to 12, 14 to 18;` list. `ConfigDelta.planner`
      (`PlannerConfigPatch planner = 3`) deleted from the `ConfigDelta`
      oneof; `3` added to a `reserved` list on that oneof (new — none
      exists there today). `Twist` (arm 19) is untouched (sprint.md
      Architecture Decision 5).
- [x] `config.proto`: `message PlannerConfigPatch { ... }` deleted
      wholesale.
- [x] `planner.proto` and `motion.proto` deleted wholesale (verified
      contents: `planner.proto` defines `DriveMode`, `StopStyle`,
      `Origin`, `CmpOp`, `StopKind`, `HeadingSourceMode` enums and
      `StopCondition`/`VelocityGoal`/`GotoGoal`/`TurnGoal`/
      `DistanceGoal`/`TimedGoal`/`RotationGoal`/`StreamGoal`/
      `PlannerCommand`/`PlannerState`/`PlannerConfig` messages;
      `motion.proto` defines `MotionSegment` + `MotionStatus`). Confirm
      no other `.proto` still `import`s either file before deleting.
- [x] `telemetry.proto`: full rewrite per
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
- [x] `src/scripts/gen_boot_config.py`: `defaultPlannerConfig()` emission
      and its planner-field helper functions removed (verified present
      today at gen_boot_config.py:642 and several helper docstrings
      earlier in the file).
- [x] `python build.py` regenerates `messages/telemetry.h`, `wire.cpp`,
      and the host `pb2` modules without error from the edited protos
      (a full green firmware build is NOT expected yet — `robot_loop.cpp`
      etc. don't reference the new shapes until ticket 005; this
      criterion is about the **generator step succeeding**, not the
      whole firmware linking).
- [x] `gen_messages.py`'s worst-case size table is inspected for the
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

## Completion Notes

- Deleted `src/protos/planner.proto` and `src/protos/motion.proto`
  wholesale (`git rm`) after confirming `telemetry.proto` was the only
  importer of either (`grep -rn 'import "planner.proto"\|import
  "motion.proto"' src/protos/`).
- `envelope.proto`: removed `message Move { ... }` (arm 20) and the
  `move` oneof field, added `20` to `CommandEnvelope`'s `reserved` list;
  removed `ConfigDelta.planner` (field 3) and added `reserved 3;` to
  `ConfigDelta` (new list — none existed there before). `Twist` (arm 19)
  untouched. Also touched up three stale "ack ring" doc-comment
  mentions elsewhere in the file (the ring this same ticket replaces
  with the single ack slot) to keep the header accurate — not a
  behavioral change, comment-only.
- `config.proto`: removed `message PlannerConfigPatch { ... }`
  wholesale; left `ConfigTarget.CONFIG_PLANNER` declared (an enum
  value, not a type — costs nothing, needs no `reserved`).
- `telemetry.proto`: full rewrite per the amendment spec —
  `DriveMode` enum relocated in from the deleted `planner.proto`;
  `EncoderReading`/`OtosReading` per-source reading messages; `Telemetry`
  rebuilt with the single `flags` bit-string (all 16 bits documented,
  bit 15 declared-not-wired per the ticket), single `ack_corr`/`ack_err`
  slot, always-present `pose`/`twist`, packed `line`/`color` words;
  clean renumber, every field ≤ 15, no `reserved`. `AckEntry`,
  `AckStatus`, `ExecutorState`, `HeadingSourceStatus` deleted.
  `TelemetrySecondary` untouched (ts_left/ts_right redundancy noted but
  not pruned, per scope).
- `gen_boot_config.py`: removed `defaultPlannerConfig()`'s emission and
  its 11 planner-field helper functions (`heading_gains_for_config`,
  `heading_source_for_config` + its `_HEADING_SOURCE_WIRE_NAMES` table,
  `heading_dwell_for_config`, `lead_compensation_for_config`,
  `min_speed_for_config`, `profile_rot_limits_for_config`,
  `arrive_dwell_for_config`, `actuation_lag_for_config`,
  `distance_gains_for_config`, `model_tau_for_config`,
  `motion_limits_for_config`) and their call sites in `generate()`;
  removed the now-unused `import math`; updated the module docstring
  and one now-dangling docstring example (`heading_gains_for_config({})`
  → `vel_gains_for_config({})`) to stop citing deleted functions.
  `boot_config.h`'s own `msg::PlannerConfig defaultPlannerConfig();`
  declaration is untouched (out of this ticket's file scope) and will
  not compile once `msg::PlannerConfig` is gone — expected, ticket 005's
  concern, matching this ticket's own "generator step, not the whole
  firmware linking" framing.
- Verified generator success directly (`uv run python
  src/scripts/gen_messages.py`, `gen_pb2.py`, `gen_boot_config.py`) since
  a full `python build.py` firmware compile is not expected to succeed
  at this ticket's boundary (`robot_loop.cpp` etc. still reference
  deleted headers from ticket 002). All three ran clean; also ran
  `gen_version.py` (build.py's fourth codegen step) for completeness.
  `messages/planner.h`/`motion.h` are gone (glob-discovered, no
  hand-edit needed); `pb2/planner_pb2.py`/`motion_pb2.py` are gone
  (gen_pb2.py wipes and regenerates its whole output dir). Spot-checked
  the regenerated `envelope.h`/`config.h`/`telemetry.h` and the
  regenerated `envelope_pb2`/`telemetry_pb2`/`config_pb2` Python
  bindings by hand (oneof arm lists, field lists) — all match the spec
  exactly.
- **Measured frame size** (`gen_messages.py`'s `kMaxEncodedSize` report,
  computed directly via its own `_worst_case_message_size` for a
  standalone number too): `Telemetry` standalone = **144 B**. Wrapped as
  `ReplyEnvelope.body`'s `tlm` arm (how a primary frame actually
  transmits): 147 B arm contribution, **153 B** `ReplyEnvelope` total —
  33 B margin under the 186-byte envelope budget, vs. the pre-rewrite
  179 B / 7 B margin (spike-003's own measurement). Recorded in
  `telemetry.proto`'s header comment, replacing the amendment's ~137 B
  estimate.
- **Deliberately not touched** (ticket 009's explicit scope per
  sprint.md's ticket table — "Test-suite sweep + green bar... edit
  survivors for the new frame/blob shape"): `src/tests/sim/unit/
  test_gen_boot_config_planner.py` (every test in it calls one of the
  now-deleted helper functions or asserts on `defaultPlannerConfig()`'s
  emitted text — will fail until ticket 009's sweep),
  `test_gen_boot_config_required_keys.py`'s planner-key cases, the
  `boot_config_golden_*.cpp` fixtures, and any wire-codec/differential/
  fuzz test with a hardcoded `Telemetry`/`ConfigDelta`/`CommandEnvelope`
  field dict. This matches the gut issue's own S1 framing ("no
  intermediate state compiles") — S1 does not reach a green `pytest`
  bar until ticket 009.
