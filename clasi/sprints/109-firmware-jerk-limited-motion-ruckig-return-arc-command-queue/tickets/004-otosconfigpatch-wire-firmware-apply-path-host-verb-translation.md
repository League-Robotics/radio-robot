---
id: '004'
title: OtosConfigPatch wire + firmware apply path + host direct-patch send
status: in-progress
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

- [ ] `OtosConfigPatch` added to `ConfigDelta` oneof + `ConfigTarget`
      plumbing, regenerated via `scripts/gen_messages.py` (no hand edits).
- [ ] `RobotLoop::handleConfig` applies the OTOS patch via
      `Otos::setLinearScalar()`/`setAngularScalar()` (and offset/pose
      setters), live (no reflash required) — matching how `MotorConfig`
      is already live-applied.
- [ ] `binary_bridge.py`'s `_OTOS_DEVICE_VERBS` gate is replaced (not
      restored) with a direct `OtosConfigPatch` construct-and-send call
      using ticket 002's mechanism; `OL <scale>` / `OA <scale>` / `OI`
      reach the firmware over the real wire on hardware, and via
      `SimLoop.inject_command()` on Sim — with no code path through
      `binary_bridge.translate_command()`.
- [ ] `src/firm/devices/DESIGN.md` updated if `Otos`'s public interface
      or invariants changed (setters were already public; note the new
      runtime call path exists now, if the doc describes call sites);
      `src/firm/DESIGN.md` updated if `handleConfig`'s patch-kind set
      changed in a way the root doc's Interfaces section documents.
- [ ] Bench: `OL`/`OA`/`OI` issued from the TestGUI over the real serial
      link visibly change subsequent OTOS reads (per
      `.claude/rules/hardware-bench-testing.md` — round-trip over the
      real link).

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
