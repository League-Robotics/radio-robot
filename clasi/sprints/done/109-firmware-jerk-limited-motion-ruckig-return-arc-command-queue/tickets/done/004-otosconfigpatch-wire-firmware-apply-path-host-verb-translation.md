---
id: '004'
title: OtosConfigPatch wire + firmware apply path + host direct-patch send
status: done
use-cases:
- SUC-005
depends-on:
- '003'
github-issue: ''
issue: otos-calibration-config-message.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OtosConfigPatch wire + firmware apply path + host direct-patch send

## Description

**Note (2026-07-17, Architecture Revision 1 on ticket 002):** step 3
below originally read "restore the `binary_bridge`/`NezhaProtocol`
translation arm for `OL`/`OA`/`OI`". That assumed
`binary_bridge.translate_command()` was a working path that merely needed
an OTOS-specific arm re-enabled. It is not — it is a universal stub for
every verb on every transport (`legacy_render`/`legacy_verbs` were
deleted wholesale in sprint 104 ticket 002 and never rebuilt), and this
sprint does not resurrect it (stakeholder's 2026-07-10 "firmware stays
pure binary" decision stands). Step 3 is revised below accordingly: `OL`/
`OA`/`OI` construct and send an `OtosConfigPatch` directly, via the same
direct-patch-send mechanism ticket 002 establishes/reuses, uniformly
across hardware and Sim transports. See sprint.md's Architecture
Revision 1 for the full narrative.

`Devices::Otos` already has `setLinearScalar()`/`setAngularScalar()`
(`source/devices/otos.h:217-218` per the issue) but they're only ever
called once at boot from baked `boot_config`. There is no runtime path.
This ticket adds one, restoring live OTOS recalibration on hardware
without a reflash, and is a hard prerequisite for ticket 007's sim
fidelity work (the sim needs a calibration patch to correct against a
modeled raw error) and for this sprint's SUC-005.

Ordered after ticket 003 (not because of a functional dependency, but so
both tickets' `protos/*.proto`/`ConfigDelta` edits land serially rather
than concurrently, avoiding merge friction in a single-agent serial
execution model).

1. Add `OtosConfigPatch` (linear_scale, angular_scale, offset_x/y/yaw —
   mirror `OtosBootConfig`'s fields) to the `ConfigDelta` oneof in
   `protos/envelope.proto` (or `config.proto`, matching wherever
   `DrivetrainConfigPatch`/`MotorConfigPatch`/`PlannerConfigPatch` live),
   plus the `ConfigTarget`/patch_kind plumbing those three already have.
   Regenerate via `scripts/gen_messages.py`.
2. `RobotLoop::handleConfig` (`source/app/robot_loop.cpp:145` per the
   issue — note the actual current path may have moved under the single-
   loop rebuild; locate the live equivalent before assuming the old line
   number) gains a case that applies the OTOS patch via the existing
   `Otos::setLinearScalar()`/`setAngularScalar()` (and offset/pose
   setters as needed), the same way the MOTOR patch is already live-
   applied — this is additive, not a rewrite of `handleConfig`.
3. Host: `OL`/`OA`/`OI` construct and send an `OtosConfigPatch`
   `ConfigDelta` directly, using ticket 002's direct-patch-send mechanism
   (the same non-`translate_command()` path `MotorConfigPatch` already
   uses) — not via `binary_bridge.translate_command()`, which stays dead.
   `binary_bridge.py`'s `_OTOS_DEVICE_VERBS` currently renders these
   "nodev / requires sprint 098"; replace that gate with the new
   direct-send call rather than routing it through the legacy-verb
   translation layer.

## Acceptance Criteria

- [x] `OtosConfigPatch` added to `ConfigDelta` oneof + `ConfigTarget`
      plumbing, regenerated via `scripts/gen_messages.py` (no hand edits).
- [x] `RobotLoop::handleConfig` applies the OTOS patch via
      `Otos::setLinearScalar()`/`setAngularScalar()` (and offset/pose
      setters), live (no reflash required) — matching how `MotorConfig`
      is already live-applied.
- [x] `binary_bridge.py`'s `_OTOS_DEVICE_VERBS` gate is replaced (not
      restored) with a direct `OtosConfigPatch` construct-and-send call
      using ticket 002's mechanism; `OL <scale>` / `OA <scale>` / `OI`
      reach the firmware over the real wire on hardware, and via
      `SimLoop.inject_command()` on Sim — with no code path through
      `binary_bridge.translate_command()`.
- [x] `src/firm/devices/DESIGN.md` updated if `Otos`'s public interface
      or invariants changed (setters were already public; note the new
      runtime call path exists now, if the doc describes call sites);
      `src/firm/DESIGN.md` updated if `handleConfig`'s patch-kind set
      changed in a way the root doc's Interfaces section documents.
- [~] Bench: `OL`/`OA`/`OI` issued from the TestGUI over the real serial
      link visibly change subsequent OTOS reads (per
      `.claude/rules/hardware-bench-testing.md` — round-trip over the
      real link). **Deferred this session** per explicit team-lead
      direction: hardware USB deploy is currently broken (registry
      inconsistency documented in ticket 003; only one `mbdeploy probe`
      attempt authorized this session, not spent chasing a known-broken
      deploy path). Verified instead against the REAL compiled firmware
      simulator end to end (`src/tests/testgui/test_transport.py`'s new
      `test_o{l,a,i}_round_trips_against_the_real_sim_firmware` tests —
      these run `RobotLoop::handleConfig`'s actual OTOS case, not a mock)
      — the decisive gate this sprint is Sim (sprint.md's own framing);
      the bench round-trip is a follow-up once USB deploy is fixed.

## Implementation Notes

- **Schema**: `OtosConfigPatch` (`protos/config.proto`) mirrors
  `Config::OtosBootConfig`'s 5 fields (`linear_scale`/`angular_scale`/
  `offset_x`/`offset_y`/`offset_yaw`, all `optional float`/`Opt<T>`) PLUS a
  6th, deliberately non-optional `bool init` trigger field (OI has no
  `OtosBootConfig` counterpart — it is a fire-and-forget action, not a
  value). Added as `ConfigDelta.otos = 5` (`envelope.proto`) and
  `ConfigTarget.CONFIG_OTOS = 5` (`config.proto`) — a fresh, never-before-
  used field number, per the project's reserved-field-number discipline.
  `python build.py`'s `kMaxEncodedSize` report: `CommandEnvelope` total
  stays 115B (config=109B was already the worst-case oneof arm before this
  patch — `DrivetrainConfigPatch`'s 8 optional floats already dominated;
  `OtosConfigPatch` did not push `config` past that ceiling), well inside
  the 186-byte envelope budget.
- **Firmware**: `RobotLoop::handleConfig` (`src/firm/app/robot_loop.cpp`)
  gains an `OTOS` branch, checked BEFORE the existing MOTOR-only gate
  (additive, not a rewrite). `setLinearScalar()`/`setAngularScalar()` are
  called directly when present; the offset triple is READ first
  (`Otos::getOffset()`) then merged with whatever fields are present and
  written back as a whole (`setOffset()` always writes x/y/heading
  together) — mirroring the MOTOR patch's own gains-merge pattern one
  section above it in the same function. `init` fires `Otos::init()`
  unconditionally when true, independent of the patch's other fields.
  **Single-loop bus ownership discrepancy, resolved in favor of
  `otos.h`**: the launching brief's invariant framing suggested a config
  apply needing an I2C write must be STAGED and executed from the leaf's
  own `tick()` slot. `otos.h`'s own doc comment for exactly these four
  primitives (`resetTracking()`/`setLinearScalar()`/`setAngularScalar()`/
  `setOffset()`/`init()`) already documents them as issuing their I2C write
  IMMEDIATELY, "matching the OI/OR/OL/OA wire-command shape" — i.e.
  anticipating direct calls from a live command handler, not staging.
  `handleConfig()` runs synchronously inside `RobotLoop::cycle()` (via
  `processMessage()`), so this is still "the loop's own cycle" doing the
  bus traffic per DESIGN.md §3 — a rare, command-triggered transaction
  sandwiched into the existing schedule, not a new per-cycle bus consumer,
  not a violation of the invariant. Followed `otos.h` per this ticket's own
  explicit "if it conflicts with otos.h's own documented register
  behavior, trust otos.h" direction; no architecture exception thrown.
- **Host**: `NezhaProtocol.otos_config()` (`protocol.py`) is a NEW method
  alongside `config()` (not folded into `_ALL_SET_KEYS`) — OL/OA/OI were
  never flat `SET key=value` text verbs, so there was no existing flat
  wire-key vocabulary to extend. `binary_bridge.py`'s `translate_command()`
  intercepts `OL`/`OA`/`OI` at the very top, BEFORE the
  `_LEGACY_TRANSLATION_AVAILABLE` short-circuit, so they work independent
  of that permanently-dead layer; `SimTransport._handle_otos_patch()`
  (`transport.py`) mirrors `_handle_config_set()`'s existing shape for Sim,
  reusing `otos_config()` for envelope construction and
  `_SimConfigConn.poll_ack()` for ack correlation (matching ticket 002's
  established SET/GET precedent exactly — hardware polls via
  `NezhaProtocol.wait_for_ack()`, Sim via its own `poll_ack()`, envelope
  construction shared). `OV`/`OP`/`OR` keep rendering `nodev` (no
  direct-patch-send equivalent — no wire arm for a raw position write/
  query, and OR/Kalman-reset has no `ConfigDelta` field).
- **Drive-by restoration**: `push.py`'s `calibration_commands()` had
  DROPPED the `OI`/`OL`/`OA` push entirely (2026-07-16, out-of-process,
  citing this exact ticket in its own comment: "re-add these... once
  ticket 004... restores a runtime OTOS-config path"). Restored, using
  `scale_to_int8()` (`calibration/helpers.py`, unchanged) to encode the
  raw multiplier into the chip's raw int8 register scalar — this directly
  fulfills the issue's own closing statement ("the TestGUI's connect-time
  OTOS-calibration push works on hardware again"). `test_calibration_push_
  on_connect.py` updated accordingly (was asserting OI/OL/OA are NEVER
  pushed; now asserts they ARE, with the correct encoding and ordering).
- **`ConfigTarget.CONFIG_OTOS`** has no live GET/`ConfigSnapshot` reader
  this ticket (same as the pre-existing `CONFIG_WATCHDOG`'s own partial
  wiring) — the enum slot exists for a future extension, not exercised by
  any test beyond its own presence in the generated header.

## Testing

- **Existing tests to run**: existing `handleConfig`/`ConfigDelta`
  firmware/sim tests (confirm MOTOR patch behavior unregressed);
  `wire_test_codec.cpp` round-trip tests.
- **New tests to write**: `OtosConfigPatch` round-trip test (host encode →
  firmware/sim decode → `Otos::setLinearScalar()`/`setAngularScalar()`
  called with the right values); `binary_bridge` translation test for
  `OL`/`OA`/`OI` → `OtosConfigPatch`.
- **Verification command**: `uv run python -m pytest` (host-side
  translation tests) plus the firmware/sim config-patch test target.

## Implementation Plan

**Approach**: Purely additive — one new patch kind alongside the three
that already exist, following their exact pattern (proto addition,
`handleConfig` case, host translation arm). No architectural decision
beyond "do it the way MOTOR already does it."

**Files to modify**:
- `protos/envelope.proto` (or `config.proto`) — `OtosConfigPatch` +
  oneof/`ConfigTarget` entries
- `src/firm/app/robot_loop.cpp` (or the current `handleConfig` location —
  locate via `grep -rn "handleConfig"`) — new case
- `host/.../binary_bridge.py` — `_OTOS_DEVICE_VERBS` gate replaced with a
  direct-patch-send call (ticket 002's mechanism), not a translation arm
- `src/firm/devices/DESIGN.md`, `src/firm/DESIGN.md` (if call-site
  documentation needs updating)

**Testing plan**: as above.

**Documentation updates**: `src/firm/devices/DESIGN.md` /
`src/firm/DESIGN.md` per the acceptance criteria above.
