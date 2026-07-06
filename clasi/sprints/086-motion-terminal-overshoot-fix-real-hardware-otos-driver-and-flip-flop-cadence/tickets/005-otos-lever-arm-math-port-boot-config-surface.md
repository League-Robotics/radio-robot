---
id: "005"
title: "OTOS lever-arm math port + boot-config surface"
status: open
use-cases: [SUC-005, SUC-006]
depends-on: []
github-issue: ""
issue: nezha-hardware-otos-driver-for-new-source-tree.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS lever-arm math port + boot-config surface

## Description

Foundational ticket for the real-hardware OTOS driver — pure math and
config plumbing, no `Hal::Odometer` leaf yet, so this ticket is entirely
host-testable.

**Known hardware quirk (do NOT re-derive): the OTOS `REG_OFFSET` register
is unwritable on this hardware** — verified in `source_old` (`OtosSensor::
begin()`'s own comment: "the write ACKs but the register reads back 0").
The lever-arm (mounting-offset) compensation must be applied host-side, not
via the chip's own offset register. `source_old/hal/capability/
OtosLeverArm.h`'s `sensorToCentre()`/`centreToSensor()` already implement
this correctly — pure, stateless, dependency-free functions — and must be
ported (concept/math, not necessarily byte-identical C++) rather than
re-derived. Note the same-instant-heading requirement documented there: a
past regression (commit `db11b7c`, pre-rebuild tree) produced ~433 mm of
phantom translation on a pure spin because the offset rotation used a
LAGGED heading — do not reintroduce that bug.

**Config surface**: `data/robots/robot_config.schema.json` already defines
`geometry.odometry_offset_mm` (`$defs/OffsetXYYaw`: x/y/yaw_rad) and
`calibration.otos_linear_scale`/`otos_angular_scale` (doc comment:
"Programmed into the OTOS chip at boot from the baked default... via
OL/OA, not SET"), and `data/robots/tovez.json` already carries real values
(`x: -47.7, y: 3.5, yaw_rad: 0.0`). `scripts/gen_boot_config.py` today only
maps `geometry.trackwidth` — it has no OTOS-facing generator. This is
deliberately boot-time-baked, NOT a new live `SET`/wire surface
(architecture-update.md Design Rationale 4) — sprint 085 (ticket 085-005)
explicitly found and removed a dead host-side push of `SET odomOffX/Y/Yaw`
because no such key exists in `config_commands.cpp`; do not reintroduce a
live surface here.

## Acceptance Criteria

- [ ] The lever-arm compensation math (`sensorToCentre()`/`centreToSensor()`
      equivalent) is ported as a pure, stateless, dependency-free header
      (working location `source/hal/lever_arm.h`) with the SAME-instant-
      heading contract preserved and documented (i.e., the function's own
      doc comment states the caller must pass the heading read in the same
      burst as the sensor position, not a stale/previous-tick one).
- [ ] Unit test(s) prove the math is the exact inverse pair (`sensorToCentre`
      / `centreToSensor` round-trip to within floating-point tolerance) and
      exercise at least one non-zero offset + non-zero heading case (not
      just the degenerate zero-offset case).
- [ ] `scripts/gen_boot_config.py` gains a new generator function mapping
      `geometry.odometry_offset_mm` and `calibration.otos_linear_scale`/
      `otos_angular_scale` into a boot-time C++ struct, additive to the
      existing `trackwidth` mapping (no existing mapping touched).
- [ ] `source/config/boot_config.{h,cpp}` gains the corresponding new struct/
      accessor (mirroring `defaultMotorConfigs()`'s existing shape).
- [ ] No new live `SET`/wire verb is added for the offset or scalars.
- [ ] No `Hal::Odometer` leaf, `NezhaHardware`, `main.cpp`, `dev_loop.cpp`,
      or `otos_commands.{h,cpp}` change in this ticket — those are ticket
      006's scope.

## Implementation Plan

**Approach**: Port the math header first (host-testable in isolation, zero
dependencies), then extend the boot-config generator using
`gen_boot_config.py`'s existing `trackwidth` mapping as the template for
the new JSON-to-constant path.

**Files to create/modify**:
- `source/hal/lever_arm.h` (new, working name) — ported math.
- `scripts/gen_boot_config.py` — new generator function for the OTOS boot
  struct.
- `source/config/boot_config.{h,cpp}` — new struct/accessor, additive.

**Testing plan**:
- Host-clean unit tests for the lever-arm math (round-trip + non-degenerate
  case).
- A generator-level test or manual invocation confirming
  `gen_boot_config.py` correctly reads `tovez.json`'s
  `odometry_offset_mm`/`otos_linear_scale`/`otos_angular_scale` values into
  the new struct.
- Full existing `tests/sim/unit/` suite re-run to confirm no regression
  (this ticket should not touch any existing path).

**Documentation updates**: None required — no wire/protocol change. If
useful, a brief doc comment cross-referencing this ticket/issue at the new
generator function, matching the style of nearby generator comments.
