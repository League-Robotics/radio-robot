---
id: '001'
title: MotorConfig/MotorState schema additions for the write-path armor
status: open
use-cases:
- SUC-004
depends-on: []
github-issue: ''
issue: armor-motor-write-path-against-reversal-latch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# MotorConfig/MotorState schema additions for the write-path armor

## Description

Foundation ticket for the sprint: add the wire-schema fields every later
ticket reads or writes. No behavior changes yet — this ticket only adds
fields to `protos/motor.proto`, regenerates `source/messages/motor.h` via
`scripts/gen_messages.py`, and documents the new fields' provenance in the
generator's own annotation tables. See `architecture-update.md`'s "Message
Schema" section and Design Rationale 2 for the full design and why these
two `MotorConfig` fields use `optional` (`Opt<float>`) rather than the
existing `slew_rate`-style zero-sentinel convention.

**`MotorConfig`** gains two new `optional` fields (after existing fields
1-7):
- `optional float reversal_dwell = 8;` — `[ms]`, hold at commanded-zero on
  any sign change. Unset (`.has == false`) means "use the ship default,"
  not "disabled" — the ship default (100 ms) is applied later, in ticket
  002's `Hal::Motor::configure()`, not here. An explicit value of `0` is a
  valid, meaningful configuration (legacy immediate-reversal, A/B bench
  comparison only — never ship 0 as a default).
- `optional float output_deadband = 9;` — `[-1,1]` fraction. Same
  has/unset-vs-explicit-zero semantics; ship default 0.03 (3%), applied in
  ticket 002.

**`MotorState`** gains three new `optional` fields (after existing fields
1-5):
- `optional bool wedge_suspect = 6;` — motion-qualified wedge signal (see
  architecture-update.md Decision 3).
- `optional uint32 hard_reset_count = 7;` — cumulative; testability/bench
  verification, ported idea from `source_old`'s 064-003
  `hardResetCount()`/`softResetCount()` precedent.
- `optional uint32 soft_reset_count = 8;`

This ticket does **not** touch `source/hal/capability/motor.h`,
`source/hal/nezha/`, `source/commands/dev_commands.cpp`, or
`docs/protocol-v2.md` — those are tickets 002/003. It also does not touch
`source/main.cpp`'s `initDefaultMotorConfigs()` — nothing there is required
to change (the new `MotorConfig` fields default safely to "unset" until
ticket 002 gives that meaning behavior), though a one-line comment is
added there pointing at where the real ship defaults will live, per
architecture-update.md's Impact on Existing Components, so a future reader
does not go hunting for them.

## Acceptance Criteria

- [ ] `protos/motor.proto`'s `MotorConfig` message gains `reversal_dwell`
      (field 8) and `output_deadband` (field 9), both `optional float`,
      with a comment on each documenting units, the ship default that
      ticket 002 will apply, and the "0 is a valid, explicit, legacy-only
      value" semantics.
- [ ] `protos/motor.proto`'s `MotorState` message gains `wedge_suspect`
      (field 6, `optional bool`), `hard_reset_count` (field 7,
      `optional uint32`), `soft_reset_count` (field 8, `optional uint32`),
      each with a one-line doc comment.
- [ ] `scripts/gen_messages.py`'s per-field annotation tables gain entries
      for all five new fields, following the existing style at
      `("MotorConfig", "port")`/`("MotorState", "wedged")` (e.g.
      `("MotorConfig", "reversal_dwell"): "(new field — sprint 078: ..."`).
- [ ] `uv run python3 scripts/gen_messages.py` run and `source/messages/motor.h`
      regenerated; the diff shows only the five new fields (as
      `Opt<T>` members with matching getters/chainable setters, mirroring
      the existing `MotorCommand.feedforward`/`reset_position` shape) —
      no unrelated fields change.
- [ ] `source/main.cpp`'s `initDefaultMotorConfigs()` gains a one-line
      comment pointing at `Hal::Motor::configure()` (ticket 002) as where
      the real ship defaults (100 ms / 0.03) are applied — no functional
      change to `initDefaultMotorConfigs()` itself.
- [ ] `just build` (or the ARM firmware build) still succeeds — the new
      fields are inert (nothing reads them yet), so this must be a clean,
      no-behavior-change build.
- [ ] Generated `source/messages/motor.h` is confirmed NOT hand-edited
      (per project convention — only `protos/motor.proto` and
      `scripts/gen_messages.py` are hand-edited; the header is
      regenerated).

## Implementation Plan

**Approach**: pure schema/codegen ticket. Edit the `.proto`, edit the
generator's annotation tables (documentation only — these tables do not
affect codegen logic, per the existing `("MotorConfig", "port")`-style
entries already in the file), regenerate, confirm the firmware still
builds.

**Files to modify**:
- `protos/motor.proto` — add the five fields described above.
- `scripts/gen_messages.py` — add annotation-table entries for the five
  new fields (documentation-only, no codegen-logic change).
- `source/messages/motor.h` — regenerated output (never hand-edit).
- `source/main.cpp` — one-line comment near `initDefaultMotorConfigs()`.

**Testing plan**:
- **Existing tests to run**: `uv run python -m pytest` (should stay green —
  no `source/` behavior changed); `just build` to confirm the firmware
  still compiles and links with the regenerated messages.
- **New tests to write**: none this ticket (the fields are inert; ticket
  004 covers behavioral testing once ticket 002 gives them meaning).
- **Verification command**: `uv run python3 scripts/gen_messages.py && just build`

**Documentation updates**: none beyond the generator's own annotation
tables (informational, not user-facing docs — `docs/protocol-v2.md` is
ticket 003's responsibility, once the fields are actually wired to `DEV`
commands).
