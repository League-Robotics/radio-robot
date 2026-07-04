---
id: '002'
title: Proto accuracy pass (motor/drivetrain/gripper/ports/sensors) and message regen
status: done
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Proto accuracy pass (motor/drivetrain/gripper/ports/sensors) and message regen

## Description

Bring `protos/motor.proto` up to the accuracy the new hardware tier needs
(port identity, per-motor PID/slew config, per-mode capability booleans,
`reset_position`), field-check the other subsystem protos against
`source_old` reality while the generator is being re-run anyway, and
regenerate `source/messages/*.h`. This ticket produces no new behavior by
itself — it produces the wire types ticket 3 (`NezhaMotor`) and ticket 4
(`Drivetrain`) implement against.

## Acceptance Criteria

- [x] `protos/motor.proto` updated exactly per the issue's locked schema:
  - `MotorCommand`: existing `oneof control` (`duty_cycle`, `voltage`,
    `velocity`, `position`, `neutral`) unchanged; add `optional bool
    reset_position = 7;` (zero the encoder this tick; rides beside any
    other arm). `feedforward` (field 6) unchanged.
  - `MotorConfig`: add `uint32 port = 7;` (Nezha motor port 1..4 — identity
    lives in Config, not baked into a class name), `Gains vel_gains = 3;`
    (per-motor velocity loop), `float min_duty = 5;` (stiction floor /
    integrator-freeze threshold), `float slew_rate = 6;` (duty slew limit,
    `MotorSlew` semantics). Keep existing `travel_calib` (field 1) and
    `fwd_sign` (field 2). Add `float vel_filt_alpha = 4;`.
  - `MotorCapabilities`: replace `onboard_position`/`has_encoder` with one
    bool per control mode: `duty_cycle`, `voltage` (false on Nezha),
    `velocity` (true on Nezha), `position` (true on Nezha, via onboard
    0x5D), plus keep `has_encoder`.
- [x] `protos/drivetrain.proto`: add a comment on `DrivetrainConfig.vel_gains`
      and `DrivetrainConfig.min_wheel` marking them deprecated/superseded by
      per-motor `MotorConfig.vel_gains`/`min_duty` (the velocity loop moved
      into the motor). Do not delete the fields (wire-shape stability for any
      host tooling still reading the old shape). Keep `sync_gain` — it is now
      the ratio governor's knob. Verify `DrivetrainCommand`/`DrivetrainState`
      still fit a minimal two-wheel Drivetrain (twist + wheels + neutral
      arms) — no structural change expected, just confirm.
  - [x] `protos/gripper.proto`, `protos/ports.proto`, `protos/sensors.proto`:
      field-check each against `source_old` reality (do the fields still
      match what the corresponding `source_old` device/controller actually
      reads/writes?). Correct any drift found; if none, note "no drift
      found" in the PR description. These generate the capability headers
      ticket 3 writes (as headers only, unimplemented except motor).
- [x] `python scripts/gen_messages.py` runs clean against the updated protos
      and regenerates `source/messages/motor.h` and any other changed
      headers with no manual post-edits required.
- [x] `python scripts/gen_messages.py --emit-inventory` refreshes
      `docs/design/message-inventory.md` to reflect the updated message set.
- [x] Regenerated `msg::MotorConfig` exposes `port`, `travel_calib`,
      `fwd_sign`, `vel_gains`, `vel_filt_alpha`, `min_duty`, `slew_rate`
      with chainable setters (per `gen_messages.py`'s existing
      Command/Config setter convention).
- [x] Regenerated `msg::MotorCapabilities` exposes `duty_cycle`, `voltage`,
      `velocity`, `position`, `has_encoder` as plain bools.
- [x] `python build.py --clean` still succeeds (messages regen is part of
      the build; confirm the new fields don't break the C++11 POD codegen
      constraints — no STL containers, no heap, no exceptions, no RTTI).

## Field-check results (gripper/ports/sensors)

- **gripper.proto**: no drift found. `ServoController`/`Servo` are stateless
  command wrappers with no config surface to compare against; `has_gripper`,
  `gripper_offset`, `min`, `max` are pre-existing (sprint-056) forward-looking
  additions already documented as such in `message-inventory.md`, consistent
  with `Servo(pin, maxDegrees)` and the robot-JSON `has_gripper`/
  `gripper_offset_mm` identity fields. Left unchanged.
- **ports.proto**: drift found and corrected. `PortConfig.direction` (a
  per-port direction bitmap) has no hardware counterpart —
  `source_old/hal/real/PortIO.{h,cpp}` shows every RJ11 port always exposes
  BOTH a digital line (S2) and an analog line (S1) simultaneously; CODAL's
  `MicroBitPin` switches electrical direction per call
  (`setDigitalValue`/`getDigitalValue`), so there is no persistent per-port
  mode to configure. Field removed; `lag_ports` kept (matches
  `RobotConfig::lagPorts`).
- **sensors.proto**: drift found and corrected in both configs.
  - `LineSensorConfig`: `threshold` and `channel_map` had no backing anywhere
    in `LineSensor.{h,cpp}` (no binarization step, no channel remap table) —
    removed. `norm_min`/`norm_max` were single scalars, but the real
    calibration bounds are per-channel (`LineSensor::_calMin[4]`/`_calMax[4]`
    via `captureCalibMin()`/`captureCalibMax()`) — replaced with per-channel
    `cal_min`/`cal_max`. Added `filt_alpha`: the real driver has an EMA
    smoothing coefficient (`_alpha`/`setSmoothingAlpha()`) with no config
    field at all. `lag_line` kept (matches `RobotConfig::lagLine`).
  - `ColorSensorConfig`: `cal_r`/`cal_g`/`cal_b` had zero grounding — no RGBC
    scaling of raw counts exists anywhere in `source_old`
    (`readRGBC()`/`pollRGBC()` return raw register values unmodified) —
    removed. `integration`/`gain` kept: they correspond to real APDS9960
    registers (`ATIME` 0x81, `CONTROL` 0x8F) currently hardcoded in
    `ColorSensor::initApds()`; comment added noting they apply only to the
    APDS9960 fallback chip variant (the primary alt/PlanetX chip at 0x43 has
    no equivalent registers in this driver). `lag_color` kept (matches
    `RobotConfig::lagColor`).
- Also refreshed two stale `**MISSING**` entries surfaced by the
  `--emit-inventory` regen for `DrivetrainState.otos_status`/
  `otos_fusion_blocked` (074-004 fields that predate this ticket and were
  never added to `_INVENTORY_MAP`). Coverage is now 210/210 mapped.

## Testing

- **Existing tests to run**: None in `tests/` yet (ticket 006 creates the
  new tree). If any host-side proto/codegen check exists outside `tests/`
  (e.g., a standalone `gen_messages.py --dry-run` sanity check), run it.
- **New tests to write**: None required at this ticket (no runtime code
  consumes these fields yet — that's tickets 3/4). If convenient, a
  dry-run diff of `source/messages/motor.h` before/after can be attached to
  the PR for reviewer sanity, but this is not a blocking test.
- **Verification command**: `python scripts/gen_messages.py --emit-inventory`
  followed by `python build.py --clean`.
