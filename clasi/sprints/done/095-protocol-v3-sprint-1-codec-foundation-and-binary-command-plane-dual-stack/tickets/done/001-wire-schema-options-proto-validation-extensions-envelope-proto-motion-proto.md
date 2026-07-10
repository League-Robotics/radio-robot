---
id: '001'
title: 'Wire schema: options.proto validation extensions + envelope.proto + motion.proto'
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wire schema: options.proto validation extensions + envelope.proto + motion.proto

## Description

Declare the binary wire contract as proto source — the single source of
truth both the firmware codegen (tickets 003-005) and the host codegen
(ticket 002) build from. This ticket is pure schema: no generated field
tables, no `wire.{h,cpp}`, no runtime behavior. It only needs
`gen_messages.py`'s EXISTING struct-emission path to keep working
unmodified against the new/extended proto files.

1. **Extend `protos/options.proto`** with four new custom `FieldOptions`
   extensions, alongside the existing `units` (50000) and `max_count`
   (50001):
   - `min` (50002, `double`) — inclusive lower bound.
   - `max` (50003, `double`) — inclusive upper bound.
   - `abs_max` (50004, `double`) — `|v| <= abs_max` (speed/twist idiom).
   - `req` (50005, `bool`) — field must be present on the wire.

2. **Add `protos/envelope.proto`**:
   - `CommandEnvelope { uint32 corr_id = 1; oneof cmd { ... } }` — one
     oneof arm per `Rt::Blackboard` command-plane queue plus system verbs.
     **Implemented this sprint** (tickets 003-007 build on these):
     `DrivetrainCommand drive`, `MotionSegment segment`, `MotionSegment
     replace`, `Stop stop`, `Ping ping`, `Echo echo`, `DeviceId id`.
     **Declared only** (schema exists, `BinaryChannel` replies
     `Error{ERR_UNIMPLEMENTED}` — owned by later sprints): `PlannerCommand
     motion` (096/097 — Planner is currently parked, no live consumer),
     `ConfigDelta config` / `ConfigGet get` / `StreamControl stream`
     (096), `SetPose pose` / `OdometerCommand otos` (098).
   - `ReplyEnvelope { uint32 corr_id = 1; oneof body { Ack ok; Error err;
     Telemetry tlm; ConfigSnapshot cfg; Event evt; DeviceId id; } }` —
     `tlm`/`cfg`/`evt` arms are declared for 096/097 forward-compat;
     unused this sprint.
   - `Ack { uint32 q = 1; float rem = 2; }` — mirrors `q=`/`rem=` from the
     text `MOVE`/`MOVER` acks (`motion_commands.cpp` `handleMove`/
     `handleMover`) exactly.
   - `Error { ErrCode code = 1; uint32 field = 2; }` and enum `ErrCode`:
     `ERR_NONE=0, ERR_UNKNOWN=1, ERR_BADARG=2, ERR_RANGE=3, ERR_FULL=4,
     ERR_DECODE=5, ERR_UNIMPLEMENTED=6, ERR_OVERSIZE=7` (Open Question 4 —
     mirrors the text plane's existing `"unknown"/"badarg"/"range"/"full"`
     reply codes plus binary-specific additions; finalize/rename here if a
     better name surfaces during implementation, but keep the numeric
     values stable once `wire.cpp`/tests in later tickets depend on them).
   - Leaf messages: `Ping {}`, `Echo { bytes payload = 1 [(max_count) =
     64]; }`, `ConfigGet { uint32 target = 1; }`, `StreamControl { bool
     binary = 1; uint32 period = 2; }` (both declared-only, owned by 096 —
     keep minimal), `Stop {}` (zero fields — see Decision 3 below),
     `DeviceId { string model = 1; string name = 2; uint32 serial = 3;
     string fw_version = 4; uint32 proto_version = 5; }` used BOTH as the
     empty-bodied request arm (`CommandEnvelope.cmd.id`, field 14 — see
     Decision 4) and the populated reply arm (`ReplyEnvelope.body.id`,
     mirrors `handleId()`'s text reply fields in `system_commands.cpp`
     exactly: `model=NEZHA2 name=... serial=... fw=... proto=...`).

3. **Add `protos/motion.proto`**: `MotionSegment`, field-for-field
   matching `source/motion/segment.h`'s `Motion::Segment` (`distance`,
   `direction`, `final_heading`, `speed_max`, `accel_max`, `jerk_max`,
   `yaw_rate_max`, `yaw_accel_max`, `yaw_jerk_max`, `time`, `v`, `omega`,
   `stream`), in `Motion::Segment`'s OWN native units (mm, rad, mm/s,
   mm/s², mm/s³, rad/s, rad/s², rad/s³, ms) — **not** the text plane's
   centidegree-integer wire convention (Decision 2: the binary plane
   parses real floats natively; cdeg exists only as a hand-tokenizer
   workaround on the text side).

**Design decisions this ticket implements** (see
`architecture-update.md` Step 6 for full rationale — do not re-derive,
just implement):
- **Decision 2**: `MotionSegment` is a NEW proto message, not a
  resurrection of `Motion::Segment` itself as a generated type (094-005's
  reasoning still holds — `Motion::Segment` stays hand-owned by the
  executor). Ticket 007 does the field-by-field translation at the
  `BinaryChannel` boundary.
- **Decision 3**: `Stop` stays a dedicated, zero-field oneof arm (not
  `drive{neutral: BRAKE}`) — a panic-stop must have zero ways to be
  malformed.
- **Decision 4**: `DeviceId id` is added to `CommandEnvelope.cmd` (field
  14) as an empty request, even though the issue's own sketch only shows
  `DeviceId` on the reply side — `sprint.md`'s Scope commits to binary
  `id` this sprint.
- **Decision 5**: every `(min)`/`(max)`/`(abs_max)` bound in this
  schema is TRANSCRIBED from an existing text-handler constant
  (`motion_commands.cpp`), converted to native units, never re-derived.
  Cite the source constant in a comment next to each bound, e.g.:
  - `MotionSegment.distance`: `abs_max = 10000` (mm) — from
    `parseMove`'s/`parseD`'s `distance`/`mm` range check.
  - `MotionSegment.direction`/`final_heading`: `abs_max = 31.416` (rad,
    ≈ ±1800°) — from `parseMove`'s/`parseMover`'s ±180000 cdeg bound
    (RT's own wider relative-angle bound, `motion_commands.cpp`).
  - `MotionSegment.speed_max`: `min=0, max=3000` (mm/s) — from
    `kMoveMaxSpeedMax`.
  - `MotionSegment.accel_max`: `min=0, max=6000` (mm/s²) — from
    `kMoveMaxAccelMax`.
  - `MotionSegment.jerk_max`: `min=0, max=60000` (mm/s³) — from
    `kMoveMaxJerkMax`.
  - `MotionSegment.yaw_rate_max`: `min=0, max=12.566` (rad/s, ≈720°/s) —
    from `kMoveMaxYawRateMaxCdeg` (72000 cdeg/s) converted.
  - `MotionSegment.yaw_accel_max`: `min=0, max=8.727` (rad/s²,
    ≈500000 cdeg/s²) — from `kMoveMaxYawAccelMaxCdeg` converted.
  - `MotionSegment.yaw_jerk_max`: `min=0, max=34.907` (rad/s³,
    ≈2000000 cdeg/s³) — from `kMoveMaxYawJerkMaxCdeg` converted.
  - `MotionSegment.time`: `min=0, max=5000` (ms) — from `parseMover`'s
    `t` bound.
  - `MotionSegment.v`: `abs_max = 3000` (mm/s) — from `parseMover`'s `v`
    bound (SIGNED, unlike `MOVE`'s unsigned `speed_max`).
  - `MotionSegment.omega`: `abs_max = 12.566` (rad/s) — from
    `parseMover`'s `w` bound converted.
  - `DrivetrainCommand` (via `WheelTargets.w[].speed`, reused unchanged
    from `protos/drivetrain.proto` — no new bound needed here): S's
    existing text bound (±1000) is a `motion_commands.cpp` constant, not a
    proto option today; leave as-is, do not add a new bound to an
    existing message as part of this ticket (out of scope — flag as a
    follow-up if it turns out `drive`'s binary arm needs an explicit
    bound the generated struct doesn't already carry).

## Acceptance Criteria

- [x] `protos/options.proto` declares `min`/`max`/`abs_max`/`req` at field
      numbers 50002-50005 (all `optional`, matching the existing
      `units`/`max_count` declaration style), alongside the existing
      `units`/`max_count`.
- [x] `protos/envelope.proto` declares `CommandEnvelope`/`ReplyEnvelope`
      with every oneof arm listed above (7 implemented + 6 declared-only
      on the command side; `ok`/`err`/`id` populated + `tlm`/`cfg`/`evt`
      declared-only on the reply side), plus `Ack{q,rem}`,
      `Error{code,field}`, `ErrCode`, and the six leaf messages
      (`Ping`/`Echo`/`ConfigGet`/`StreamControl`/`Stop`/`DeviceId`).
- [x] `protos/motion.proto`'s `MotionSegment` fields match
      `Motion::Segment`'s fields 1:1 (name mapped snake_case, unit, sign
      convention); every bound is transcribed from the matching
      `motion_commands.cpp` constant per the list above, with a comment
      citing the source constant.
- [x] `python scripts/gen_messages.py --dry-run` succeeds; every
      PRE-EXISTING generated header's emitted content is unchanged byte-
      for-byte (the new options/messages produce two new headers,
      `envelope.h` and `motion.h`, and do not alter any existing one).
- [x] `python scripts/gen_messages.py` (real run) succeeds and produces
      `source/messages/envelope.h` and `source/messages/motion.h` with
      the expected `Opt<T>`/oneof-union/array shapes per
      `gen_messages.py`'s existing generation rules (no generator code
      change needed — verify the EXISTING rules already produce correct
      output for these new messages; if they don't, that is ticket 005's
      job, not this one — flag and stop rather than hand-patch generated
      output).
- [x] `just build-sim` succeeds; the full existing sim suite (58 tests
      baseline) stays green — this ticket adds zero runtime behavior, so
      zero test behavior change is expected.
- [x] Decisions 2/3/4/5 above are implemented as specified (not
      reinterpreted); any point where implementation forces a deviation
      is recorded in this ticket's own completion notes, not silently
      changed.

## Completion Notes (2026-07-10)

**Verification performed** (per the ticket's own verification command):
`python scripts/gen_messages.py --dry-run` → clean, no diffs to any
pre-existing header (verified programmatically by diffing
`generate_headers()`'s in-memory output against every checked-in
`source/messages/*.h`, not just the printed 5-line preview). Real
`python scripts/gen_messages.py` → wrote `envelope.h`/`motion.h`, left
every other header byte-identical (`git status` shows only the two new
files). `just build-sim` → green. `uv run python -m pytest tests/sim -q`
→ **58 passed in 62.92s**, matching the stated baseline exactly.

**Decision 5 bound-transcription check — two corrections found and
applied** (flagged per the ticket's own instruction to verify each cited
constant, not just copy it): `yaw_accel_max` and `yaw_jerk_max`'s cited
bounds (8.727 rad/s², 34.907 rad/s³) do not match the actual conversion
of their cited source constants. `kMoveMaxYawAccelMaxCdeg = 500000.0f`
cdeg/s² (motion_commands.cpp, matching its own "(~5000 deg/s²)" comment)
converts via the SAME `cdeg * pi/18000` factor `yaw_rate_max` already
uses correctly (72000 cdeg/s → 12.566 rad/s, which does match the
ticket) to **87.266 rad/s²**, not 8.727 — a factor-of-10 slip.
Likewise `kMoveMaxYawJerkMaxCdeg = 2000000.0f` cdeg/s³ converts to
**349.066 rad/s³**, not 34.907. Per Decision 5 ("the constant, never
re-derived from first principles, is authoritative"), `protos/motion.proto`
uses the corrected values (87.266 / 349.066) with an inline comment on
each field documenting the discrepancy and the re-derivation, rather than
silently propagating the ticket's arithmetic. Every other cited bound
(`distance` ±10000mm, `direction`/`final_heading` ±31.416rad,
`speed_max` 0..3000, `accel_max` 0..6000, `jerk_max` 0..60000, `time`
0..5000ms, `v` ±3000, `omega` ±12.566, `yaw_rate_max` 0..12.566) was
re-derived independently against the actual `motion_commands.cpp`
constants/literals and matches the ticket exactly.

**Two real, verified `gen_messages.py` generator gaps found — flagged,
NOT hand-patched, per the ticket's explicit instruction** (both
documented inline in `protos/envelope.proto`'s own header comments too):

1. **Cross-file struct references have no matching `#include`.**
   `envelope.proto` is the first `protos/*.proto` file in this tree to
   `import` a subsystem-specific proto file other than `common.proto`/
   `options.proto` (verified: grepped every other file's `import` lines —
   none do). `gen_messages.py`'s `_emit_file()` unconditionally emits
   exactly one `#include "messages/common.h"` for every non-`common.proto`
   header (the `_OTHER_INCLUDE` constant) — it has no per-file
   cross-reference tracking. The generated `envelope.h` therefore
   references `DrivetrainCommand`/`MotionSegment`/`PlannerCommand`/
   `SetPose`/`OdometerCommand` inside `CommandEnvelope`'s union with no
   `#include` for `drivetrain.h`/`motion.h`/`planner.h`/`odometer.h`.
   **Empirically verified**: `g++ -std=c++11 -fsyntax-only` on a bare
   `#include "messages/envelope.h"` fails with "unknown type name" for
   all five cross-file types; supplying the four missing includes by
   hand in a scratch test (not in the checked-in header — the generator
   itself is untouched) then compiles cleanly with zero errors, confirming
   the underlying shape (a union of message-typed oneof arms, including
   `DrivetrainCommand`'s OWN nested `control` oneof living inside the
   outer `cmd` oneof) is sound under C++11 unrestricted unions — this is
   purely a missing-`#include` gap, not a structural shape problem. Does
   NOT affect this ticket's own gates: nothing in `source/` includes
   `envelope.h` yet (schema-only ticket, no runtime wiring). **Flagged
   for ticket 005**: `_emit_file()`/`_emit_message()` must track, per
   message, which OTHER proto files its field types come from and emit
   one `#include` per distinct file referenced (not just `common.h`).
2. **`bytes` + `(max_count)` on a non-repeated field is silently
   ignored.** `Echo.payload` is specified as
   `bytes payload = 1 [(max_count) = 64];` (a single scalar `bytes`
   field, not `repeated`). `_emit_message()`'s `(max_count)` handling
   only triggers on `is_repeated` fields; a singular `bytes` field falls
   through to the generic scalar branch and is emitted as
   `uint8_t payload = 0;` — one byte, not the intended fixed 64-byte
   buffer. Verified directly in the generated `envelope.h`. Substituting
   `repeated uint32 payload` would produce a real fixed array but
   silently changes the wire representation from a raw byte string to a
   packed varint list — not a schema-authoring call this ticket may make
   silently — so the proto stays exactly as specified. **Flagged for
   ticket 004/005**: `TYPE_BYTES` + `(max_count)` on a non-repeated field
   needs the same fixed-array special-case `repeated` + `(max_count)`
   already gets.

**Schema-completeness gap resolved with documented, minimal placeholders**
(Decision 5's "record the discrepancy, don't silently reconcile"
precedent, generalized): `architecture-update.md`'s own envelope sketch
(and the driving issue's) names `ConfigDelta`/`Telemetry`/`ConfigSnapshot`/
`Event` as payload types for four declared-only oneof arms
(`config`/`tlm`/`cfg`/`evt`), but none of the four exists as a usable
proto message today — `ConfigDelta` exists ONLY as `Rt::ConfigDelta`, a
hand-written, differently-shaped C++ struct in
`source/runtime/commands.h` (different namespace, no collision, but no
reusable shape); `Telemetry`/`ConfigSnapshot` don't exist anywhere;
`Event` exists as the hand-authored (explicitly NOT-generated,
`source/messages/event.h`'s own header comment says so) `msg::Event`.
Since the acceptance criteria require these arms to exist and they are
explicitly "declared only... unused this sprint" (096-owned real
design), minimal EMPTY placeholder messages (`ConfigDelta`, `Telemetry`,
`ConfigSnapshot`) are declared in `envelope.proto` itself — this
introduces no shape 096 is bound to, only lets the arm exist. For `evt`,
reusing the name `Event` would generate a SECOND `struct Event` inside
the same `namespace msg` as the hand-authored one — an ODR violation the
moment one translation unit includes both `event.h` and `envelope.h`
(inevitable once `BinaryChannel`, ticket 006+, needs both) — so the
placeholder is named `EventNotify` instead, with a header comment
flagging that 096/097 must reconcile it with `msg::Event`. This is
schema-authoring latitude (empty stub messages, not an architecture
override), documented inline in `envelope.proto` and here, matching the
ticket's own citation of `protos/odometer.proto`'s Pose2D-vs-SetPose
precedent for this exact kind of recorded deviation.

**Decisions 2/3/4/5**: implemented as specified, no reinterpretation
beyond the two flagged items above (the yaw-bound arithmetic corrections
and the four placeholder types). Decision 2 (`MotionSegment` a new,
separate message in `Motion::Segment`'s native units): implemented as
specified in `protos/motion.proto`. Decision 3 (`Stop` a dedicated
zero-field arm): implemented. Decision 4 (`DeviceId id = 14` added as an
empty `CommandEnvelope.cmd` request arm): implemented. Decision 5 (every
bound transcribed from the cited constant): implemented, with the two
corrections above recorded rather than silently propagated.

## Testing

- **Existing tests to run**: `just build-sim`; `uv run python -m pytest
  tests/sim -q` (58 tests, current baseline — must stay green unmodified).
- **New tests to write**: none required by this ticket (schema-only, no
  runtime code) — `python scripts/gen_messages.py --dry-run` and a real
  `gen_messages.py` run are the verification, not a pytest addition. If a
  quick smoke test asserting `envelope.h`/`motion.h` compile stand-alone
  (a trivial `#include` + no-op `main()`) is cheap to add, it's a
  reasonable bonus but not required — ticket 003's `static_assert`s are
  the real structural verification.
- **Verification command**: `python scripts/gen_messages.py --dry-run &&
  python scripts/gen_messages.py && just build-sim && uv run python -m
  pytest tests/sim -q`
