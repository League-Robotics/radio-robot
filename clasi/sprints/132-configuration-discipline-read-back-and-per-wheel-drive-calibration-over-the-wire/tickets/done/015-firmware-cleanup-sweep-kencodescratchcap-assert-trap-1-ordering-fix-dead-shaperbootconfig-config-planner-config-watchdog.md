---
id: '015'
title: Firmware cleanup sweep (kEncodeScratchCap assert, trap 1 ordering fix, dead
  ShaperBootConfig/CONFIG_PLANNER/CONFIG_WATCHDOG)
status: done
use-cases:
- SUC-003
depends-on:
- '013'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware cleanup sweep (kEncodeScratchCap assert, trap 1 ordering fix, dead ShaperBootConfig/CONFIG_PLANNER/CONFIG_WATCHDOG)

## Description

Sweep of firmware-side cleanups the issue's audit surfaced, none touching
the wire's live behavior:

1. Add a `static_assert` guarding `kEncodeScratchCap = 220`
   (`wire.cpp:684`) against the largest nested message this schema now
   declares, so an oversized message fails to COMPILE instead of
   silently returning `0` from `encode()` at runtime.
2. Fix **trap 1** — `main.cpp:166` calls `loadPersistedTuning()` before
   `robotLoop().boot()` at `:171`, and every `RealOtos` setter no-ops
   until `begin()` sets `initialized_`. Reorder so persisted OTOS tuning
   is applied AFTER `begin()` runs (either by moving the
   `loadPersistedTuning()` call, or restructuring `boot()` so persisted
   tuning routes through `install(OTOS)` at the correct point in the
   sequence — implementer's call, but the reordering must be provable,
   not just asserted).
3. Delete dead `ShaperBootConfig` (`boot_config.h:125-137`) — confirm
   zero live consumer first with a fresh grep before deleting (the issue
   claims it is read by no code; verify, don't re-derive). Note: by this
   point in the sprint `ShaperBootConfig` may already be gone if ticket
   002/005 didn't carry it forward into `Config::Robot`'s generated
   shape — confirm its actual status before assuming it still needs
   deleting here.
4. Remove `CONFIG_PLANNER`/`CONFIG_WATCHDOG` from the `ConfigTarget`
   enum if confirmed dead — `CONFIG_WATCHDOG`'s documented routing
   target (`config_commands.cpp`/`BinaryChannel`) is confirmed ABSENT
   from the current tree by this round's grep. If the stream watchdog
   window has NO other live configuration path, flag this explicitly as
   a gap (Open Question 4 in sprint.md) rather than silently deleting a
   capability — do not delete `CONFIG_WATCHDOG` without first confirming
   there is truly no live consumer of the capability it was meant to
   serve, not just of the enum value's name.

## Acceptance Criteria

- [x] A `static_assert` on `kEncodeScratchCap` exists, sized against the
      largest nested message this schema declares.
- [x] The assert is demonstrated to fire on a deliberately oversized
      message in a throwaway branch, reverted before this ticket lands
      (completion notes record the demonstration).
- [x] `loadPersistedTuning()`'s OTOS-tuning application happens AFTER
      `begin()`/`initialized_` is true, verified by a new test: persist
      an OTOS scale, reboot (sim), confirm the persisted value — not the
      baked default — is what the chip-level setter receives (trap 1's
      own regression test, matching sprint.md's Success Criteria).
- [x] `ShaperBootConfig` is deleted ONLY after a fresh grep in this
      ticket's own implementation confirms zero live consumers; if a
      consumer is found (or it's already gone by this point), the
      ticket's completion notes say so.
- [x] `CONFIG_PLANNER` is removed from `ConfigTarget` if confirmed dead
      (it already has no wire consumer per the 115-003 planner-stack
      deletion).
- [x] `CONFIG_WATCHDOG` is either removed (with a documented replacement
      configuration path for the stream watchdog window, if one exists)
      or explicitly left in place with a completion note stating the gap
      is real and unresolved, per Open Question 4 — not silently deleted
      without checking.
- [x] Compiles under `HOST_BUILD` and ARM.

## Completion Notes

**PRIORITY 1 — wire budget (stated project emergency).** Root cause was
`SetConfigGroup.body`/`ConfigSnapshot.body`'s `(max_count) = 220` in
`src/protos/robot_config.proto`, tied to `kEncodeScratchCap` (a different,
unrelated ceiling) instead of the real largest group. Re-sized to 140 per
the stakeholder's explicit 2026-08-04 unblock ("no big deal with there
being multiple configuration messages per subsystem"); the proto's own
header comment now states the rule ("split the group, don't grow this
cap") for future readers. Measured before/after (`gen_messages.py`'s own
size-report, regenerated):

| constant | before | after | ceiling |
|---|---:|---:|---:|
| `kCommandEnvelopeMaxEncodedSize` (`wire.h`) | 234 | **154** | 240 |
| `kReplyEnvelopeMaxEncodedSize` (`wire.h`) | 232 | **192** | 240 |
| `kFramedMaxBytes` (`comms.h`) | 238 | **200** | — (own static_assert) |
| `kMaxLineBytes` (`comms.h`, computed) | 249 | **211** | `kTxBufferCapacity` = 250 |

`kFramedMaxBytes`/`kMaxLineBytes` are hand-picked literals, not derived
from the generated envelope constants — shrinking `robot_config.proto`
alone would NOT have improved `kMaxLineBytes`'s 1-byte TX-ring margin
(the actual hardware risk the ticket called urgent); `kFramedMaxBytes`
was re-picked from 238 to 200 (computed minimum 195 + 5B headroom, same
margin convention as the codebase's own "200 chosen pre-132-011"
precedent) so that margin actually widens, from 1 byte to 39 bytes.
`comms.h`'s own prose was rewritten in place (not just the literals) so
the numbers in the comments match reality again.

The dead-code sweep (Priority 4, below) additionally deleted `Planner`'s
6 dead `shaper_*` fields, dropping its own worst-case size 117→81 B
(Motors, 72 B, is now the largest group) — this doesn't move the four
constants above (they're bounded by `(max_count)`, not by a group's real
content), but it widens the 140 B cap's actual headroom to ~59 B.

**PRIORITY 2 — `kEncodeScratchCap` assert.** Added
(`gen_messages.py`'s new `_worst_case_nested_message()`/
`_render_encode_scratch_cap_assert()`, emitted into `wire.cpp` right
after `kEncodeScratchCap`'s own fixed-engine-text declaration): computes
the largest message reached via a `kMessage`/`kOneofMessage` field
anywhere below `CommandEnvelope`/`ReplyEnvelope` (`struct_order` minus
the two roots) and asserts `kEncodeScratchCap >= kWorstCaseNestedMessageSize`.
Current worst case: `Telemetry` at 185 B (kEncodeScratchCap stays 220,
35 B margin) — NOT `DeviceId`, which the pre-existing doc comment
incorrectly claimed; that comment is corrected in the same edit.

**Demonstrated firing**, per the acceptance criterion: temporarily
changed the fixed literal `kEncodeScratchCap = 220` to `100` in
`gen_messages.py`, regenerated, and rebuilt `libfirmware_host` — got a
real compiler error (`static assertion failed ... note: expression
evaluates to '100 >= 185'`), confirming a schema growing past the cap now
fails to COMPILE instead of `encodeInto()` silently returning `false` and
the frame being dropped at runtime. Reverted the literal, regenerated,
rebuilt clean (0 errors) before proceeding.

**PRIORITY 3 — trap 1 (persisted OTOS tuning silently discarded).**
Confirmed live: `main.cpp` called `graph.loadPersistedTuning()` (which
reaches `Devices::RealOtos::setLinearScalar()`/`setAngularScalar()`/
`setOffset()` via `Configurator::install(OTOS)` → `App::configureOtos()`)
BEFORE `graph.robotLoop().boot()` — every `RealOtos` setter no-ops until
`begin()` sets `initialized_ = true`, and `begin()` only ever runs inside
`boot()`'s own `Preamble::step()` loop. Fix: reordered `main.cpp` so
`boot()` runs first, then `loadPersistedTuning()`/`markConfigured()`
(previously split across the `boot()` call, now both after it — `boot()`
itself never reads either, so this changes nothing about boot's own
behavior). `begin()` cannot be re-run to "fix" a too-early apply instead
(zeroes the chip's tracked pose, kicks IMU bias calibration) — the
ordering itself had to move.

`main.cpp` is ARM-only (never compiles under `HOST_BUILD`), so the
regression test exercises the identical underlying call sequence through
`TestSim::SimHarness` instead (the same composition root, `RealOtos`
included, driven over a scripted I2C bus that DOES honor real
firmware writes to the linear/angular scalar registers —
`TestSim::OtosPlant`/`sim_plant.cpp`'s own `handleOtosWrite()`).
`TestSim::SimHarness`'s constructor gained an optional
`Config::TuningStore*` parameter (default `nullptr`, every pre-existing
call site unaffected) plus a `loadPersistedTuning()` passthrough, both
additive. New test:
`src/tests/sim/unit/trap1_persisted_otos_ordering_harness.cpp` +
`test_trap1_persisted_otos_ordering.py`, two scenarios against a seeded
`Config::TuningStore` double: (1) `loadPersistedTuning()` AFTER `boot()`
(the fix) lands the persisted scale on `OtosPlant`'s own chip-level
register; (2) `loadPersistedTuning()` BEFORE `boot()` (the old order)
leaves the register at the baked default — proving the ORDERING itself,
not just "the call happens somewhere," is what matters. Both pass.

**PRIORITY 4 — dead code sweep.**
- `Config::ShaperBootConfig`/`defaultShaperConfig()` — fresh grep
  confirmed zero live consumers (as the struct's own doc comment already
  claimed); deleted from `boot_config.h`/`.cpp` and
  `gen_boot_config.py`'s `shaper_config_for_config()` (which had exactly
  two consumers — this one and the Planner group's `shaper_*` fields
  below — both deleted together). The mirroring
  `msg::Planner.shaper_a_max…shaper_yaw_jerk_max` schema fields
  (`robot_config.proto`, ticket 001 had kept them pending this sweep)
  deleted too, field numbers 17-22 marked `reserved` (matching this
  schema's existing "reserved, not reused" wire-stability convention,
  envelope.proto/telemetry.proto).
- `Config::defaultDriveConfig()`/`Config::defaultWheelControllerConfig()`
  and the three free functions `App::installShaperLimits()`/
  `installDriveCalibration()`/`installWheelController()`
  (`boot_calibration.{h,cpp}`) — fresh grep confirmed zero call sites
  (superseded by `Configurator::install()`'s own inline fan-out since
  132-006/132-009); deleted, along with `Config::DriveBootConfig`/
  `WheelControllerBootConfig` (their only remaining reason to exist).
- `config_parity_capi.cpp`'s hand-written Planner offset table updated
  to match (it isn't generated — an `offsetof()` on a deleted field
  would have failed to compile, which is the file's own designed
  safety net).
- `CONFIG_PLANNER`/`CONFIG_WATCHDOG` (`config.proto`'s `ConfigTarget`
  enum): **already gone** — ticket 013 deleted `config.proto` wholesale,
  taking the whole enum with it, before this ticket started (confirmed:
  `src/protos/` has no `config.proto`; only stale prose comments mention
  the enum by name). Both acceptance criteria are trivially satisfied by
  that prior deletion. Per Open Question 4's own instruction, investigated
  whether `CONFIG_WATCHDOG`'s underlying capability (the stream/serial
  watchdog window) has any live replacement configuration path today:
  grepped for `streamWatchdogWindowIn`/`StreamWatchdog`/any
  watchdog-window mechanism across `src/firm` — **zero hits**. Protocol
  v5's own documented safety contract (`src/tests/CLAUDE.md`) confirms
  this is not an oversight: "there is no session-wide serial-silence
  watchdog to widen/restore at all — every `Move` carries its own bounded
  `timeout` safety backstop by construction." **This is a real, confirmed,
  unresolved gap** — flagging for the team-lead to mark sprint.md's Open
  Question 4 as answered (not resolved: no live path exists, only a
  different mechanism that serves a related but not identical purpose).

**Two pre-existing, unrelated test breaks found and fixed opportunistically**
(both directly adjacent to files this ticket already touches, both
one-line, zero-risk): `test_gen_boot_config_planner.py`'s own
`"DriveBootConfig defaultDriveConfig()" in content` assertion (now
correctly asserts its absence — the other 2 of that file's 3 failures are
confirmed pre-existing/unrelated, a `planner.heading_hold_gain`
value drift nothing to do with this ticket) and
`test_robot_config_proto_parses.py::test_protoc_parses_the_full_proto_set`'s
stale `assert "config.proto" in names` (config.proto was deleted by
ticket 013; flipped to `not in`).

**Testing.** New:
`src/tests/sim/unit/trap1_persisted_otos_ordering_harness.cpp` +
`test_trap1_persisted_otos_ordering.py` (2 scenarios, both pass).
Updated: `test_gen_boot_config_robot_groups.py` (shaper/drive/
wheel-controller absence assertions), `test_robot_config_proto_parses.py`
(REQUIRED_FIELD_HOMES shaper entries removed; config.proto assertion
fixed), `test_gen_boot_config_planner.py` (stale DriveBootConfig
assertion removed). Verified GREEN: `src/tests/unit/` (excluding 3 files
with PRE-EXISTING collection errors from ticket 013's config.proto
deletion — `test_protocol_binary_client.py`, `test_protocol_config.py`,
`test_sim_boot_config.py`, none touched by this ticket) — 715 passed, 6
failed, all 6 confirmed pre-existing/unrelated (3 × already-known
`test_gen_boot_config_planner.py`, 1 × `test_serial_conn_binary_plane.py`
ConfigDelta/config.proto, 2 × `test_calibration_kwargs.py` legacy
text-protocol `SET`/`OI`/`OL`/`OA` drift). `src/tests/sim/`: 445 passed
(444 baseline + this ticket's new test), 16 failed (unchanged — the
known `ConfigDelta`/`config_pb2` family, ticket 014's job, not touched),
3 xfailed (unchanged). HOST_BUILD (`libfirmware_host`) rebuilt clean, 0
warnings, throughout. ARM: `main.cpp` and every other touched `src/firm`
file syntax-checked with the real ARM compile command (extracted from
`build/compile_commands.json`, `-fsyntax-only`) — 0 errors (only
pre-existing vendor-header warnings, unrelated to any touched file).
A full ARM link build was not run (would trigger `dotconfig version
bump`, out of scope per this sprint's version-bump cadence — see
`.claude/rules/git-commits.md`).

**Left deliberately alone** (out of this ticket's explicit scope):
`Motion::PlannerLimits`/`PlannerBootConfig`/`defaultPlannerLimits()` (a
DIFFERENT, live surface — not touched); `App::configurePlanner()` (has
no production call site either, but is a newer 132-007 entry point with
its own test harness, not named by this ticket); the 16 known
`ConfigDelta`/`config_pb2` sim failures (014's job); the pre-existing
`test_gen_boot_config_planner.py`/`test_calibration_kwargs.py`/
`test_serial_conn_binary_plane.py` failures not directly adjacent to
this ticket's own edits; scattered historical prose comments elsewhere
in the tree (`drive.h`, `sim_ctypes.cpp`, `sim_harness.h`,
`composition_root_parity_harness.cpp`, `move_protocol_harness.cpp`,
`config_gate_harness.cpp`, `duty_sweep.py`, `sim_boot_config.py`) that
mention the now-deleted symbol names in past tense — confirmed
comment-only (no compiled/executed code), not chased further.

## Testing

- **Existing tests to run**: not required to run the full sim suite per
  this ticket's own gate (heavy verification is tickets 018/019), but
  the specific tests this ticket touches (trap-1 regression,
  `kEncodeScratchCap` assert demonstration) must pass.
- **New tests to write**: as listed in Acceptance Criteria.
- **Verification command**: `uv run python -m pytest <affected test
  paths> -q`; a separate ARM build check (`just build` or equivalent)
  for the static_assert and any ARM-only code path.

## Implementation Plan

**Approach**: Four largely independent sub-fixes; tackle in the order
listed (assert first — cheapest, no behavior change; trap 1 next — a
real ordering bug; dead-code deletions last, since they require fresh
verification before removal).

**Files to modify**: `src/firm/messages/wire.cpp` (generated — confirm
whether the assert belongs in generator engine-text per
`gen_messages.py`, or is added directly if it's genuinely fixed
text), `src/firm/main.cpp`, `src/firm/app/boot_wiring.{h,cpp}` or
`configurator.{h,cpp}` (wherever the OTOS-tuning-after-`begin()`
reordering ends up living), `src/firm/config/boot_config.h`
(`ShaperBootConfig` deletion, if confirmed dead and still present),
`src/protos/config.proto` or `robot_config.proto` (`ConfigTarget` enum
cleanup).

**Testing plan**: as above.

**Documentation updates**: none beyond inline comments explaining each
fix.
