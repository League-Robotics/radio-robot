---
id: "005"
title: "OTOS lever-arm math port + boot-config surface"
status: done
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

- [x] The lever-arm compensation math (`sensorToCentre()`/`centreToSensor()`
      equivalent) is ported as a pure, stateless, dependency-free header
      (working location `source/hal/lever_arm.h`) with the SAME-instant-
      heading contract preserved and documented (i.e., the function's own
      doc comment states the caller must pass the heading read in the same
      burst as the sensor position, not a stale/previous-tick one).
- [x] Unit test(s) prove the math is the exact inverse pair (`sensorToCentre`
      / `centreToSensor` round-trip to within floating-point tolerance) and
      exercise at least one non-zero offset + non-zero heading case (not
      just the degenerate zero-offset case).
- [x] `scripts/gen_boot_config.py` gains a new generator function mapping
      `geometry.odometry_offset_mm` and `calibration.otos_linear_scale`/
      `otos_angular_scale` into a boot-time C++ struct, additive to the
      existing `trackwidth` mapping (no existing mapping touched).
- [x] `source/config/boot_config.{h,cpp}` gains the corresponding new struct/
      accessor (mirroring `defaultMotorConfigs()`'s existing shape).
- [x] No new live `SET`/wire verb is added for the offset or scalars.
- [x] No `Hal::Odometer` leaf, `NezhaHardware`, `main.cpp`, `dev_loop.cpp`,
      or `otos_commands.{h,cpp}` change in this ticket — those are ticket
      006's scope.

## Completion Notes

Implemented exactly per plan, host-tested, no hardware involved.

- **`source/hal/lever_arm.h`** (new) — `namespace LeverArm` (matching this
  tree's existing pure-math-helper precedent: `MotorSlew` in
  `source/hal/nezha/motor_slew.h`, `BodyKinematics` in
  `source/kinematics/body_kinematics.h`), with `sensorToCentre()`/
  `centreToSensor()` ported from `source_old/hal/capability/OtosLeverArm.h`
  — identical math, renamed parameters to drop the embedded-unit suffixes
  (`sensorHrad`/`centreHrad` -> `sensorHeading`/`centreHeading`, tagged
  `// [rad]`) per `.claude/rules/coding-standards.md`. The doc comment states
  the same-instant-heading contract explicitly and cites the db11b7c
  ~433 mm phantom-translation regression as the failure mode to avoid.
- **Tests**: `tests/sim/unit/lever_arm_harness.cpp` +
  `tests/sim/unit/test_lever_arm.py`, following this tree's established
  harness-compiles-and-runs-via-subprocess pattern (mirrors
  `stop_condition_harness.cpp`/`test_stop_condition.py` — header-only, no
  `.cpp` to link). Four scenarios: (1) zero-offset identity in both
  directions; (2) exact-inverse round-trip using tovez.json's real mounting
  offset (x=-47.7, y=3.5) and a non-zero heading (0.9 rad), recovering the
  original centre pose within 1e-3 tolerance; (3) the same round-trip swept
  across 2 offset pairs x 5 headings (including negative and >pi/2 values)
  to rule out a single-angle coincidence; (4) a regression guard proving a
  *lagged* heading (the db11b7c failure mode) leaves a real (>1mm) residual
  — i.e. the same-instant contract is load-bearing, not just a comment. All
  four scenarios pass (`OK: all LeverArm scenarios passed`).
- **`scripts/gen_boot_config.py`** — new `otos_boot_config_values(cfg)`
  function reading `geometry.odometry_offset_mm.{x,y,yaw_rad}` and
  `calibration.otos_{linear,angular}_scale`, falling back to identity
  defaults (zero offset, 1.0 scale) when absent — same fallback-to-firmware-
  default idiom every other mapping in this file already uses. `generate()`
  now additionally emits a `Config::defaultOtosBootConfig()` function,
  appended after `defaultDrivetrainConfig()`; the pre-existing
  `defaultMotorConfigs()`/`defaultDrivetrainConfig()` bodies are untouched
  (confirmed by `git diff` on the regenerated `boot_config.cpp`: a pure
  20-line addition, zero lines removed/changed).
- **`source/config/boot_config.{h,cpp}`** — new `Config::OtosBootConfig`
  struct (`offsetX`/`offsetY`/`offsetYaw` `[mm]`/`[mm]`/`[rad]`,
  `linearScale`/`angularScale` dimensionless multipliers) +
  `defaultOtosBootConfig()` accessor, additive after the existing
  `defaultDrivetrainConfig()`. Deliberately named `linearScale`/
  `angularScale` (the 1.0-based JSON multiplier) rather than reusing
  `Hal::Odometer::setLinearScalar()`'s "scalar" vocabulary (the OTOS chip's
  raw int8 register domain, per `docs/protocol-v2.md` §11's OL/OA spec) —
  ticket 006's leaf is expected to convert multiplier -> register scalar
  once at `begin()` (mirroring `source_old/hal/real/OtosSensor.cpp::begin()`'s
  `scaleToInt8()`), not at every `OL`/`OA` wire call. This ambiguity
  resolution is called out explicitly in the struct's doc comment so ticket
  006 doesn't have to re-derive it.
- Regenerated `source/config/boot_config.cpp` against the currently-active
  robot config (`data/robots/tovez_nocal.json`, via
  `data/robots/active_robot.json`): `offsetX=-47.7f, offsetY=3.5f,
  offsetYaw=0.0f, linearScale=1.0f, angularScale=1.0f` (nocal's `calibration`
  object is empty, so the scale multipliers fall back to the 1.0 identity
  default; the offset is present in nocal's `geometry` block, matching
  tovez.json). Separately verified (not committed) that pointing
  `ROBOT_CONFIG` at `data/robots/tovez.json` directly produces
  `linearScale=1.067, angularScale=0.987` — the calibrated values.
- **Generator test**: `tests/unit/test_gen_boot_config_otos.py` (in-process,
  mirrors `tests/unit/test_gen_messages_no_getters.py`'s pattern) — asserts
  `otos_boot_config_values()` reads tovez.json's real values, falls back to
  identity defaults on an empty config, and that `generate()`'s output gains
  `defaultOtosBootConfig()` additively (the pre-existing generated functions
  are still present, unmodified in shape).
- **No live wire surface added** — grepped `source/commands/config_commands.cpp`
  and `source/commands/otos_commands.cpp`; neither was touched, and no new
  `SET`/`GET` key was introduced anywhere.
- **Full host suite**: `uv run python -m pytest -q` — 619 passed (615
  pre-existing + 4 new), 0 regressions, ~149s.
- Files touched: `source/hal/lever_arm.h` (new),
  `scripts/gen_boot_config.py`, `source/config/boot_config.h`,
  `source/config/boot_config.cpp` (regenerated),
  `tests/sim/unit/lever_arm_harness.cpp` (new),
  `tests/sim/unit/test_lever_arm.py` (new),
  `tests/unit/test_gen_boot_config_otos.py` (new).
- Nothing surprising for the stakeholder to verify on the stand — this
  ticket is pure host-side math/config plumbing with zero hardware surface;
  ticket 006 is where the real I2C driver and HITL verification happen.

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
