---
id: 008
title: applyGroup() + per-target re-appliability table + boot-only ERR_NOT_LIVE rejection
status: done
use-cases:
- SUC-003
depends-on:
- '007'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# applyGroup() + per-target re-appliability table + boot-only ERR_NOT_LIVE rejection

## Description

Add `Configurator::applyGroup(ConfigTarget, const uint8_t* wire, size_t
len)` — decodes straight into `config_` (no patch, no presence flags, no
merge) using the group's generated wire codec (ticket 002's C++ target
IS the wire codec type — no adapter needed, since `wire.cpp`'s
`decodeInto` already writes through an arbitrary `base + offset`). Add a
per-`ConfigTarget` re-appliability table declaring which groups are
live-configurable (`DRIVE`, `WHEEL_CONTROL`, `MOTORS` (travel_calib only,
guarded), `OTOS`, `ESTIMATOR` — per the issue's own 8-setter audit) vs.
boot-only (`GEOMETRY`, `PLANNER` — `trackWidth`/`vMax`/`omegaMax`/
`controlPeriod`/`actuationDelay`/`landing.*`/`headingHoldGain` have no
post-construction setter per the issue's ~14-value audit). A push to a
boot-only target returns `ERR_NOT_LIVE`, not `OK`.

This ticket wires the dispatch mechanism and the table; it does not need
every `install(target)` branch to be fully correct yet for OTOS/ESTIMATOR
(tickets 009/010 do that) — this ticket's own scope is: decode correctly,
reject boot-only correctly, and call some `install(target)` path for the
live-configurable targets.

## Acceptance Criteria

- [x] `Configurator::applyGroup(ConfigTarget, const uint8_t*, size_t)`
      exists, decodes into `config_` using the generated group codec, and
      validates bounds inline (reusing the existing `wire.cpp`
      decode/validate machinery — no new hand-written parsing).
- [x] A per-`ConfigTarget` re-appliability table exists (a static array
      or switch) declaring at minimum: `DRIVE`/`WHEEL_CONTROL`/`MOTORS`/
      `OTOS`/`ESTIMATOR` = live; `GEOMETRY`/`PLANNER` = boot-only.
- [x] A push to `GEOMETRY` or `PLANNER` returns `ERR_NOT_LIVE`, verified
      by a new test.
- [x] A push to a live target decodes into `config_` and calls
      `install(target)`.
- [x] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: unaffected — this is new dispatch machinery;
  the old `apply(CommandEnvelope)` patch-based method still exists until
  ticket 013.
- **New tests to write**: one test per target confirming live vs.
  boot-only classification; one test confirming a boot-only push returns
  `ERR_NOT_LIVE` without mutating `config_`.
- **Verification command**: `uv run python -m pytest <configurator unit
  test path> -q`.

## Implementation Plan

**Approach**: Add the table and the new method to `Configurator`, reusing
`wire.cpp`'s existing `decodeInto()`-shaped decode path by calling it
directly against `&config_.<group>` as the base pointer.

**Files to modify**: `src/firm/app/configurator.{h,cpp}`.

**Testing plan**: as above.

**Documentation updates**: `configurator.h`'s doc comment gets a table
(or a pointer to one) listing each `ConfigTarget`'s re-appliability and
why.

## Implementation notes (as-built, deviating from the plan above)

**The plan's "Files to modify: `src/firm/app/configurator.{h,cpp}`" was
incomplete.** No wire codec existed for `Geometry`/`Motors`/`Drive`/
`WheelControl`/`Planner`/`Otos`/`Estimator` when this ticket started:
`gen_messages.py`'s `wire.cpp`/`wire.h` emission only builds a `FieldDesc`
table for structs reachable (BFS) from `CommandEnvelope`/`ReplyEnvelope`
(`_compute_layout_check_structs()`), and `SetConfigGroup.body` is a
placeholder `bytes` field (ticket 001's own scope note), not a
message-typed oneof arm that would make the 7 groups reachable. Ticket
002 explicitly said as much ("this ticket does not yet wire the generated
header into anything"). `robot_config.proto`'s own header comment already
anticipated this: "ticket 002 (codegen) and ticket 008/009/012 (firmware
dispatch) decide the generated C++ representation of `bytes`."

So this ticket also touched, beyond `configurator.{h,cpp}`:

- `src/scripts/gen_messages.py` — `_emit_wire_files()` now ALSO builds a
  `FieldDesc`/`MessageTable` (`kFields_<Group>`/`kTable_<Group>`) and a
  generated `Result decode(<Group>&, const uint8_t*, uint16_t)` overload
  (`wire.h` declaration + `wire.cpp` definition) for each of the 7
  robot-config groups — same pattern `decode(Telemetry&, ...)` (124-008)
  already established, same `decodeInto()`/`validateBounds()` engine, no
  new hand-written parsing. The 7 groups are deliberately kept OUT of
  `kMessageTables[]`/`struct_order` itself (nothing nests a message-typed
  field into them), so `struct_order`'s existing reachable-set semantics
  and the 095-003 day-one layout-check gate are untouched.
- `src/scripts/gen_messages.py`'s `validateBounds()` (fixed engine text)
  — added an explicit `v != v` (NaN) reject before the min/max/abs_max
  comparisons. `v < fd.minVal`/`v > fd.maxVal` are both false for NaN, so
  a NaN payload would otherwise pass every bound a field declares (the
  ticket's own "NaN defeats bounds validation" fact). This fixes the gap
  for EVERY decode through `decodeInto()`, not just the 7 new groups —
  the correct scope for an engine-level bug, not a per-message patch.
- `src/protos/envelope.proto` — added two new `ErrCode` values:
  `ERR_NOT_LIVE = 9` (boot-only rejection, this ticket's own headline
  acceptance criterion) and `ERR_BUSY = 10`. `ERR_BUSY` was not in this
  ticket's original scope (ticket 009's plan text says "mapped here" for
  the `MOTORS` guard), but leaving `install(MOTORS)` as a silent-always-OK
  stub would have reproduced, inside this very ticket, the exact "config
  that acks OK and does nothing" disease the ticket exists to prevent —
  so the guard (`App::configureMotor()`'s bool -> `ERR_BUSY`) is fully
  wired here, not stubbed. Ticket 009 needs no further work for `MOTORS`.
- Regenerated `src/firm/messages/{envelope,wire}.h`, `wire.cpp` (the only
  files that actually changed — `robot_config.h`/`robot_config.schema.json`/
  `robot_config_generated.py`/`wire_commands.py` came out byte-identical,
  confirming the generator change is additive and does not touch the
  path ticket 002 already built).

**A correctness bug caught and fixed during self-review, not part of any
plan**: `msg::wire::decode(<Group>&, ...)` unconditionally `memset`s its
`out` argument to zero before decoding (the same full-object-zero
rationale `decode(Telemetry&, ...)` documents). `applyGroup()`'s first
draft decoded straight into `config_.drive` (etc.) directly, per the
issue's "decodes straight into `config_`" phrasing taken literally — but
that would zero the CURRENTLY LIVE group immediately, before it is even
known whether the incoming push is valid, and a decode failure partway
through would leave `config_` half-zeroed/half-new rather than untouched.
Fixed by decoding into a local scratch value of the group's own type and
committing it into `config_` only after a successful `Result` — "decodes
straight into `config_`" is honored at the level that matters (no patch,
no presence flags, no field-by-field merge — the whole group is replaced
in one assignment on success), while actually delivering the "a rejected
push leaves `config_` untouched" property both the ticket and the
NaN/malformed-bytes tests require.

**`install(ESTIMATOR)` returns `ERR_UNIMPLEMENTED`, not `ERR_NONE`.**
There is no live consumer at all today — `App::StateEstimator` was
deleted as dead code (sprint 128 ticket 016) and `Configurator` holds no
replacement reference (ticket 010 adds one). Returning `OK` would
silently reproduce trap 2 (the exact bug this sprint exists to close)
inside the ticket that is supposed to make that class of bug impossible.
`config_.estimator` is still decoded correctly (read-back stays honest);
only the fan-out is honestly reported as unwired. `install(OTOS)` DOES
reach a real consumer (`App::configureOtos()`, 132-007) but keeps the
known trap-3 scale-domain mismatch (`scaleToRegister()` not yet applied
on this path) — explicitly ticket 010's job per its own ticket file, not
silently "fixed" here.

**"Reject incomplete group pushes"** (a group push whose zero-filled
`Drive` section would silently stop the robot via `setDutyPerSpeed`'s
`calibrated_` gate) is NOT implemented by this ticket — it is a
deeper `install(DRIVE)` correctness concern ticket 009 already owns
(depends-on: 008, title "per-wheel Stage-A correction live over the
wire"). The `(req)` proto option exists and IS engine-level implemented
(`decodeInto()`'s `seen`-bitmap completeness check) and could enforce
this, but marking every `Drive`/`WheelControl` field `(req)` would also
reject a legitimate real-protobuf push that omits a field intentionally
left at its documented "0 = off"/"0 = disabled" default (`crawl_pulse`,
every `WheelControl` field) — proto3 implicit presence means a real
protobuf encoder skips zero-valued scalar fields by default, so blanket
`(req)` would be too strict for THIS schema as currently shaped. Left as
a documented, flagged gap for ticket 009 to resolve with the fuller
context it already has (which specific fields actually need to always be
present vs. legitimately default-zero).

## Testing performed

- New: `src/tests/sim/unit/configurator_applygroup_harness.cpp` +
  `test_configurator_applygroup.py` — boot-only classification
  (`GEOMETRY`/`PLANNER` -> `ERR_NOT_LIVE`, including with a well-formed
  payload, and with the check running BEFORE the buffer is even read);
  live classification (`DRIVE`/`WHEEL_CONTROL`/`MOTORS`/`OTOS`/
  `ESTIMATOR` never `ERR_NOT_LIVE`); `DRIVE`/`WHEEL_CONTROL` decode +
  behavioral `Drive::configure()` effect; `MOTORS` at-rest apply and
  in-motion `ERR_BUSY` (including the per-side-independent-guard case:
  one side busy does not block the other side's calibration); `OTOS`
  decode + `configureOtos()` pass-through; `ESTIMATOR` decode-succeeds-
  but-`ERR_UNIMPLEMENTED`; NaN-in-a-bounded-field rejection
  (`ERR_RANGE`) with a preceding good push proving no-partial-commit;
  truncated/malformed bytes rejection (`ERR_DECODE`) with the same
  no-partial-commit proof. No I2C bus, no sim plant — same lightweight
  shape as `configure_entry_points_harness.cpp` (132-007). Test payloads
  are hand-encoded via `WireRuntime`'s PUBLIC primitives (the same
  pattern `wire_test_codec.cpp` already established for `CommandEnvelope`
  encoding) — test-only, does not compete with or bypass the production
  `applyGroup()` decode path.
- Regression: `test_configure_entry_points.py`,
  `test_configurator_loadbaked.py`, `test_composition_root_parity.py`,
  `test_gen_messages_robot_config_emission.py` (all 31 cases) — pass
  unmodified.
- `uv run python -m pytest src/tests/unit src/tests/sim -q` — run for
  broader regressions; one pre-existing collection error
  (`test_sim_boot_config.py`, `ImportError: CalibrationConfig` from the
  ticket-020 pydantic reshape) excluded per this sprint's documented
  scope discipline (tickets 014/017 own it) — confirmed via `git diff`
  that `robot_config.py` has zero changes on this branch and the missing
  name is absent even at `HEAD`, so the failure predates this ticket's
  work.
