---
id: '013'
title: Delete the *ConfigPatch surface, PatchKind, merge accumulator; reshape persisted
  tuning
status: done
use-cases:
- SUC-001
- SUC-005
depends-on:
- 009
- '010'
- '011'
- '012'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete the *ConfigPatch surface, PatchKind, merge accumulator; reshape persisted tuning

## Description

Delete `DrivetrainConfigPatch`/`MotorConfigPatch`/`OtosConfigPatch`/
`EstimatorConfigPatch` (`config.proto`), `ConfigDelta::PatchKind` and its
union (`envelope.h`), and the merge accumulator
(`mergeMotorGainsPatch`/`mergeOtosPatch`, `configurator.cpp`) — the whole
surface the new group/field wire arms (tickets 008-012) replace.
`Configurator::persistedTuning_` (currently
`msg::MotorConfigPatch`/`OtosConfigPatch`-shaped) becomes a snapshot of
`Config::Robot`'s live-reappliable groups (`DRIVE`/`WHEEL_CONTROL`/
`MOTORS`-travel_calib/`OTOS`/`ESTIMATOR` per ticket 008's table),
following today's persistence PRECEDENT — Motor gains/travel_calib and
OTOS scale/offset persist; `EstimatorConfigPatch` never has
(`config.proto`'s own comment: "a reboot always reverts to the baked
JSON default"). Document explicitly which groups persist under the new
shape rather than silently expanding or contracting the set (see
sprint.md Out of Scope).

**This is the point in the sprint where mid-sprint breakage is expected**:
any host code not yet migrated (ticket 014's job) will fail to build/run
against the deleted patch types until that ticket lands. That is
accepted, per stakeholder direction — do not add a compatibility shim to
keep it working in between.

## Acceptance Criteria

- [x] `DrivetrainConfigPatch`, `MotorConfigPatch`, `OtosConfigPatch`,
      `EstimatorConfigPatch` no longer exist in `config.proto` (or the
      file itself is deleted if nothing else lives in it — confirm
      during implementation).
- [x] `ConfigDelta::PatchKind` and its union no longer exist in
      `envelope.h`.
- [x] `mergeMotorGainsPatch`/`mergeOtosPatch` and
      `Configurator::apply(const msg::CommandEnvelope&)` (the old
      patch-based entry point) are deleted.
- [x] `Configurator::persistedTuning_` is reshaped to a
      `Config::Robot`-groups-shaped snapshot; the persistence scope
      (which groups persist) is documented in `configurator.h`'s file
      header, following today's precedent (Motor/Otos persist,
      Estimator does not) unless this ticket's own finding says
      otherwise — in which case the finding and its reasoning are
      recorded in the ticket's completion notes.
- [x] A repo-wide grep for `ConfigPatch`/`PatchKind` inside `src/firm/`
      returns nothing outside git history.
- [x] Compiles under `HOST_BUILD` (firmware side only — host-side
      breakage until ticket 014 is expected and NOT this ticket's
      problem to fix).

## Testing

- **Existing tests to run**: firmware-side tests referencing the OLD
  patch types are expected to need updating or deletion as part of this
  ticket (they test a surface that no longer exists) — update/delete
  them, do not leave them referencing dead types.
- **New tests to write**: a persistence round-trip test for the reshaped
  `persistedTuning_` (save, reload, confirm the same live-reappliable
  groups come back).
- **Verification command**: `uv run python -m pytest <firmware-side sim
  test paths, host-side EXCLUDED this ticket> -q`.

## Implementation Plan

**Approach**: Delete the old types and the old `Configurator::apply()`
entry point; reshape `persistedTuning_`'s storage type and its save/load
serialization to match `Config::Robot`'s live-reappliable groups.

**Files to modify**: `src/protos/config.proto` (or delete),
`src/firm/messages/envelope.h`, `src/firm/app/configurator.{h,cpp}`,
`src/firm/config/persisted_tuning.{h,cpp}`.

**Files to delete**: any firmware test file that exists solely to test
the deleted patch surface (confirm none is testing something else
incidentally before deleting).

**Testing plan**: as above.

**Documentation updates**: `configurator.h`'s file header rewritten to
describe the new group/field-based ownership model in full (superseding
its current "three things it used to own" description, which describes
the old patch-merge design).

## Completion Notes

**What was deleted.** `src/protos/config.proto` deleted outright (nothing
else lived in it — it held only `ConfigTarget`, `BoundMotorSide`, and the
four curated `*ConfigPatch` messages). `ConfigDelta` (and its `PatchKind`
union) deleted from `src/protos/envelope.proto`; `CommandEnvelope.config`
(field 6, kept — same "reshape an arm's payload, keep its number"
precedent `STOP`'s own meaning change already set) now carries
`SetConfigGroup` instead, dispatched by `RobotLoop::routeCommand()` to
`Configurator::applyGroup()` (this was flagged in robot_config.proto's own
comment as "ticket 013's job" — wiring it was necessary for
`Configurator::apply(const msg::CommandEnvelope&)`'s deletion to leave a
working CONFIG arm at all, not optional polish). `mergeMotorGainsPatch`/
`mergeOtosPatch`/`applyMotorConfigPatch`/`applyOtosPatch`/`apply()` all
deleted from `configurator.{h,cpp}`, replaced by a new
`persistIfEligible(target)` private method called from `applyGroup()`/
`applyField()`'s own success paths. Regenerated `src/firm/messages/*.h`
via `gen_messages.py` and `src/host/robot_radio/robot/pb2/*.py` via
`gen_pb2.py` (mechanical, schema-driven — `config_pb2.py` is now gone
too).

**`ConfigGroupTarget` → `ConfigTarget` rename: declined.** Deleting
`config.proto` frees the shorter `ConfigTarget` spelling as documented,
but a repo-wide grep found 154 occurrences of `ConfigGroupTarget` across
21 files spanning `src/firm/` (configurator.cpp/.h alone: 60), `src/tests/
sim/unit/` C++ harnesses (~40), host Python (`protocol.py`, the
`robot_config_pb2` bindings), and `src/scripts/`. This ripples well
outside `src/firm/` into files this ticket is explicitly not supposed to
touch (host `protocol.py` — ticket 014's own surface) and into the C++
test harnesses already touched for other reasons in this same ticket.
Per the ticket's own escape valve ("if that ripples too far, say so and
leave it"), left as `ConfigGroupTarget`.

**Persisted-tuning reshape — the numbers.** `TuningSnapshot` is now flat
plain data: 3 per-GROUP `bool` presence flags (`wheelControlTuned`/
`motorsTravelCalibTuned`/`otosTuned` — needed so `reapplyPersistedTuning()`
never stomps an untouched group's fresh baked default with a
zero-initialized snapshot field) + 12 raw `float`s (WheelControl's 5
gains, Motors' 2 travel_calib sides, Otos' 5 scale/offset fields) — no
more per-field `Opt<T>` presence (that merge-accumulator shape is gone
with the patch surface). **`kBlobSize` = 51 bytes** (3 presence bytes +
12×4 float bytes), down from the old shape's 85 bytes. **`kPayloadBytes`
(version + blob) = 55**, **`kNumChunks` = ceil(55/32) = 2**, down from 3 —
well under the ARM-only `static_assert(kNumChunks <= 4)` ceiling, with 2
full chunks of headroom now instead of 1. `kConfigSchemaVersion` bumped
2 → 3 (a curated-field-set AND layout change, per that constant's own
documented bump policy) — this wipes any pre-132-013 flash blob at next
boot rather than misreading it, and does not stack an unbumped
meaning-change on top of the tracked prior gap.

Persistence scope, following today's precedent exactly (neither expanded
nor contracted): **WHEEL_CONTROL persists in full** (5 fields — the direct
successor of the old `MotorConfigPatch`'s kp/ki/kff/i_max/kaw, which
already persisted). **MOTORS persists `travel_calib_left`/
`travel_calib_right` only** (matches the old `MotorConfigPatch.travel_calib`
precedent — the one `MotorConfig` field `App::configureMotor()` still
live-applies). **OTOS persists in full** (5 fields, mirrors the old
`OtosConfigPatch`'s scale/offset fields exactly; its 6th field, `init`,
was a fire-and-forget trigger with no `Config::Robot`-shaped successor and
was never persisted either — losing the OI/init wire trigger entirely is a
real, known gap this ticket does not restore; flagged for ticket 014/a
follow-up, not fixed here). **DRIVE (Stage A per-wheel correction, this
sprint's own headline new capability) and ESTIMATOR do NOT persist** —
Stage A never had a wire arm before this sprint (no precedent to honor),
Estimator explicitly never persisted even when it did have a wire arm.
GEOMETRY/PLANNER are boot-only and were never candidates.

**Envelope/line-length constants, after this change.**
`kCommandEnvelopeMaxEncodedSize` **234** (up from 55 — wiring
`SetConfigGroup`'s ~220 B `body` into `CommandEnvelope.config` is what
grew this, not a regression: the old `ConfigDelta` arm never exceeded
~50 B, and this capacity was already allocated by ticket 001's own schema,
just unwired until now). `kReplyEnvelopeMaxEncodedSize` unchanged at
**232**. `kMaxEnvelopeBytes` **234** (was 232 — `config` is now the
overall worst-case arm, edging out `cfg`'s 228+4). `kMaxCrcPayloadBytes`
**236** (was 234). `kFramedMaxBytes` **238, unchanged** (a hand-picked
literal; still satisfies its own `static_assert(>= cobsEncodedMaxLength
(kMaxCrcPayloadBytes))`, but that margin shrank from 3 bytes to 1 — flagged
in `comms.h`'s own doc comment for whichever ticket next grows either
envelope's worst-case arm). `kMaxCommandPrefixBytes` unchanged at 11
(`GET_CONFIG:` is still the longest verb). **`kMaxLineBytes` unchanged at
249** — 1 byte under `Com::SerialPort::kTxBufferCapacity` (250), same
margin as before this ticket. Net: this ticket's deletions did NOT free
line-length budget (the ticket's own expectation) because wiring
`SetConfigGroup` — required to leave a working CONFIG arm at all —
consumes exactly the capacity the deletions freed; the constants are
reported precisely above so the next ticket that touches either envelope's
worst-case arm knows the 1-byte margins are real, not stale.

**Remaining references swept.** `src/firm/` (the acceptance-criterion
scope) is grep-clean for `ConfigPatch`/`PatchKind`, including its four
`DESIGN.md` files, `boot_config.{h,cpp}`, and `devices/motor.h`. Also
swept, though outside the strict `src/firm/` scope: `docs/design/design.md`
(the current, living architecture doc), and the firmware-side C++ test
harnesses/Python drivers under `src/tests/sim/` (see Testing below) —
`justfile`, `CLAUDE.md`, `.clasi/` (outside this sprint's own docs), and
`src/tests/bench/` were already clean, no action needed. Left
UNTOUCHED as archival/historical record (rewriting history would be
wrong): every `clasi/sprints/done/*` ticket/architecture-update, and
`docs/protocol-v3.md`/`docs/protocol-v4.md`/`docs/architecture/
architecture-update-*.md`/`docs/code_review/*.md` (all superseded
snapshots). **Two OPEN, live `clasi/issues/` files reference now-stale
facts and were NOT edited** (out of this ticket's write scope — issue
management is a team-lead/MCP action, not a programmer one): `clasi/
issues/C-cruft-ledger-sweep-zero-consumer-code.md` asks to fix
`EstimatorConfigPatch`'s silent-acked-0 behavior — already resolved by
132-010/132-013 (`install(ESTIMATOR)` now permanently returns
`ERR_UNIMPLEMENTED`, a loud rejection). `clasi/issues/
A-next-physical-bench-session-checklist.md` line ~126 references
`Configurator::applyMotorConfigPatch`, which this ticket deleted (the
live-reappliable path is now `Configurator::install(WHEEL_CONTROL)` →
`Drive::setControlGains()`). Flagging both for the team-lead to update/
close via the proper CLASI tooling.

**Testing.** New: rewrote `persisted_tuning_harness.cpp` (`src/tests/sim/
unit/`) end to end for the new flat/per-group-presence shape — round-trip
(fully-tuned, fresh/all-default, one-group-tuned-leaves-others-untouched),
`shouldWipe()`, and a `kBlobSize == 51` regression pin;
`test_persisted_tuning.py` passes. Updated to compile/pass against the new
`SetConfigGroup` wire shape: `wire_codec_harness.cpp` (its own
`ConfigDelta` round-trip scenario replaced with a `SetConfigGroup`
equivalent; the now-inapplicable reserved-watchdog-field scenario
deleted), `config_gate_harness.cpp` and `move_protocol_harness.cpp` (their
shared `armorMotorConfig(Patch)Command()` helpers retargeted to push a
`WHEEL_CONTROL` `SetConfigGroup` instead of a `MOTOR` `ConfigDelta`, same
`pid_kp`-lands-on-Drive assertion), `wire_differential_harness.cpp` +
`_wire_diff_driver.py` + `test_wire_differential.py` (the whole
`ConfigDelta`-specific differential/boundary corpus deleted — replacement
coverage is the lighter-weight scenarios above, a full differential suite
for the new group surface is not this ticket's job — plus the
`config_pb2`-dependent builders deleted, a new `env_config_group()`
helper added for `test_wire_fuzz.py`'s CONFIG-arm fuzz seed).
`src/host/robot_radio/robot/protocol.py`'s top-level `import config_pb2`
line removed (NOT the method bodies that use it) — this was necessary
because `robot_radio.robot/__init__.py` eagerly imports `protocol.py`, so
leaving the import broken would raise `ImportError` on every firmware-side
test that merely imports `robot_radio.robot.pb2`, not just on an actual
call to a broken method; every `config_pb2`/`ConfigDelta`-referencing
method body in `protocol.py` is left exactly as broken as before (raises
`NameError`/`AttributeError` if called) — this is the one host-file touch
in this ticket, scoped to an import statement, not the migration itself.

Verified GREEN: `src/tests/sim/unit/` (423 passed, 1 xfailed — the xfail
is `test_app_robot_loop.py`, confirmed PRE-EXISTING and unrelated:
`app_robot_loop_harness.cpp` has been `xfail(strict=False)` since 125-006
for an unrelated `App::RobotLoop` constructor-arity break; it never
reaches its own CONFIG-dispatch scenarios). `test_wire_fuzz.py` (265
passed). `test_move_protocol.py` (1 passed). `test_sim_fidelity.py`
(passed).

**Known broken — ticket 014's job, not fixed here.** `src/tests/sim/
system/` has 11 failing tests (plus 5 more in `src/tests/sim/`'s
top-level scripts, 16 total) — ALL trace to the exact same single root
cause: `NezhaProtocol.set_config()`/`.config()` (`protocol.py`) still
building `envelope_pb2.ConfigDelta(...)`/`config_pb2.DrivetrainConfigPatch
(...)`, which no longer exist, raising `AttributeError`. Every one of
these 16 tests reaches that code through `SimLoop.configure_from_robot()`
(`src/host/robot_radio/io/sim_loop.py`)'s Tier-1 push — itself unmodified
by this ticket, since fixing it means building the real
`set_config_group()`/`set_config_field()`-based replacement, i.e. the
actual host-consumer migration sprint.md assigns to ticket 014. Affected:
`test_sim_boot_config_parity.py` (4), `test_sim_configure_from_robot.py`
(3), `test_sim_wire_loopback.py` (3), `test_straight_leg_crab_regression.py`
(1), `test_motor_primitive.py` (2), `test_pathplan_goto_convergence.py`
(3). None of these overlap the sprint's own accepted 7-failure baseline
(different files entirely). `src/tests/unit/`+`src/tests/testgui/`
(host-side) not run, per this ticket's own scope — expected broken the
same way, same root cause.
