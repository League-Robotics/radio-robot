---
id: '002'
title: ConfigDelta live-apply (motor gains) + inner velocity-PID resonance taming
status: open
use-cases:
- SUC-025
depends-on:
- '001'
github-issue: ''
issue:
- host-planner-design-lessons-from-drive-v2-review.md
- heading-loop-output-clamp-and-velocity-resonance.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# ConfigDelta live-apply (motor gains) + inner velocity-PID resonance taming

## Description

Two findings from this sprint's own architecture pass (Step 1), confirmed
against the live tree, block resonance work outright: (1)
`RobotLoop::cycle()`'s `CmdKind::CONFIG` case unconditionally acks
`ERR_UNIMPLEMENTED` — `ConfigDelta` decodes but is never applied — and (2)
`Devices::NezhaMotor` has NO runtime gain mutator at all, only a
constructor-time `const MotorConfig&`. The resonance issue's own prescribed
bench method (`SET pid.kp` on the stand) is pre-P4 text-protocol vocabulary
that no longer exists on the wire, so binding requirement #9 ("everything
tunable live") is currently unmet at the firmware boundary.

This ticket (a) gives `Devices::NezhaMotor` a live gain-apply method taking
ONLY `Devices`-local types (`Devices::Gains`, plain floats) — never the wire
`MotorConfigPatch` type, preserving `source/devices/`'s standing isolation
invariant (never `#include "messages/..."`); (b) wires `RobotLoop::cycle()`'s
`CONFIG` case to decode a `MotorConfigPatch`, translate its present fields
(`kp`/`ki`/`kff`/`i_max`/`kaw`/`travel_calib`) into that method's parameters
(the app-layer `RobotLoop` is the one legitimate translation boundary between
the wire type and the device-local type, mirroring `config.proto`'s own
documented `BinaryChannel` precedent for the SAME `MotorConfigPatch`), and
ack `OK` instead of `ERR_UNIMPLEMENTED` for that ONE patch type —
`DrivetrainConfigPatch`/`PlannerConfigPatch` stay `ERR_UNIMPLEMENTED`,
unchanged, deliberately out of scope (`PlannerConfigPatch`'s
`heading_kp`/`heading_kd` target `Motion::SegmentExecutor`, deleted post-102;
`DrivetrainConfigPatch`'s EKF fields have no on-robot fusion consumer this
sprint); and (c), with live tuning restored, characterizes and tames the
~140 mm/s inner-velocity-PID resonance (`heading-loop-output-clamp-and-
velocity-resonance.md` Part 2) using the on-stand step harness, EXHAUSTING
the already-wire-tunable `kp`/`ki`/`kff`/`iMax`/`kaw` surface first against
the `<~10%` overshoot bar (rise time preserved) before considering promoting
`velFiltAlpha` (currently reflash-only) to wire-tunable or adding a notch
filter — see `architecture-update.md` Decisions 2 and 4.

## Acceptance Criteria

- [ ] `Devices::NezhaMotor` gains a live gain-apply method whose parameters
      are exclusively `Devices`-local types (`Devices::Gains`/plain floats)
      — confirmed by inspection that no `messages/...` include is added to
      `source/devices/`.
- [ ] `RobotLoop::cycle()`'s `CONFIG` case decodes a `MotorConfigPatch`,
      applies every PRESENT field to both bound motors via the new method
      (matching `config.proto`'s own documented "applied to BOTH bound
      motors unconditionally" convention for `kp`/`ki`/`kff`/`i_max`/`kaw`,
      and per-side for `travel_calib`), and acks `ACK_STATUS_OK`.
- [ ] `DrivetrainConfigPatch`/`PlannerConfigPatch` continue acking
      `ERR_UNIMPLEMENTED`, unchanged — confirmed by a test that a
      `ConfigDelta{drivetrain: ...}` or `{planner: ...}` still gets that
      error code.
- [ ] A `config()` call carrying `pid.kp`/`ki`/`kff`/`iMax`/`kaw` measurably
      changes the robot's live step response on the SAME boot, with no
      reflash — bench-verified.
- [ ] On-stand velocity-step harness (drive-arm step at 70/140/250 mm/s, per
      `heading-loop-output-clamp-and-velocity-resonance.md`'s own
      methodology) shows `<~10%` step overshoot across that range with rise
      time preserved, superseding the interim `vel_kp=0.0014` detuning
      currently shipped in `data/robots/tovez.json`.
- [ ] If constants-only tuning cannot hit the bar, Completion Notes document
      the attempt and either (a) promote `velFiltAlpha` to a new
      live-tunable `MotorConfigPatch` field (the smallest of the three
      original candidates), or (b) explicitly flag a fast-follow ticket —
      an empirical decision, not pre-assumed by this ticket.
- [ ] Full project test suite green; bench-verified per
      `.claude/rules/hardware-bench-testing.md`.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`; the existing
  `devices_motor_harness.cpp`-family unit tests
  (`tests/sim/unit/test_app_drive.py` and neighbors) that exercise
  `Devices::NezhaMotor`/`Devices::MotorVelocityPid`, since this ticket adds
  a new mutator to a class they already cover.
- **New tests to write**: a sim/unit test proving the new `NezhaMotor`
  method changes subsequent PID output (mirroring
  `devices_motor_harness.cpp`'s existing scenario 6 pattern — "PID-on chases
  a velocity target"); a `RobotLoop`-level test proving `CONFIG{motor:...}`
  acks `OK` and applies, while `CONFIG{drivetrain:...}`/`{planner:...}`
  still ack `ERR_UNIMPLEMENTED`.
- **Verification command**: `uv run python -m pytest`, plus the on-stand
  step-response bench sweep (manual, per
  `.claude/rules/hardware-bench-testing.md`).

## Implementation Plan

**Approach**: Add a `Devices`-local gain-apply method to
`Devices::NezhaMotor` (e.g. taking a `const Gains&` plus an optional
`travelCalib` float) that mutates `config_`'s relevant fields directly — no
I2C side effect required, since `MotorVelocityPid::compute()` already reads
`config_.velGains` fresh every tick. Wire `RobotLoop::cycle()`'s
`CmdKind::CONFIG` case: when the decoded `ConfigDelta`'s patch is
`MotorConfigPatch`, build a `Devices::Gains` from the PRESENT optional wire
fields (falling back to each motor's current `config_` value for any absent
field — the same `Opt<T>`-presence convention `config.proto` already
documents), call the new method on both `motorL_`/`motorR_` for
`kp`/`ki`/`kff`/`i_max`/`kaw`, and on the side-selected motor only for
`travel_calib`; ack `OK`. Leave the `Drivetrain`/`Planner` patch arms exactly
as today. Then run the on-stand velocity-step harness (drive-arm step,
`tests/bench/` — check `pid_hold_speed.py`/`velocity_chart.py` first for
reuse before writing anything new) against the now-live `config()` path
instead of a reflash loop; iterate `kp`/`ki`/`kff`/`iMax`/`kaw` against the
`<~10%` bar; update `data/robots/tovez.json` if the shipped tuning changes.

**Files to modify**:
- `source/devices/nezha_motor.{h,cpp}` — new live gain-apply method.
- `source/app/robot_loop.cpp` — `CONFIG` case's `MotorConfigPatch` handling.
- `source/devices/velocity_pid.{h,cpp}` — ONLY if constants-only tuning
  proves insufficient (Open Question 1); gain/filter VALUES, not a
  control-law shape change, unless the fallback notch/feedforward path is
  reached.
- `data/robots/tovez.json` — updated gain values if the shipped tuning
  changes.

**Files to create**: a bench step-response sweep script/extension under
`tests/bench/`, ONLY if no existing script already fits the P4 binary plane
for this purpose.

**Testing plan**: sim/unit coverage for the new `NezhaMotor` method and the
`RobotLoop` `CONFIG` dispatch (both patch-applied and still-unimplemented
paths); the on-stand step-response sweep is the real acceptance evidence,
captured numerically in Completion Notes, not merely asserted.

**Documentation updates**: `heading-loop-output-clamp-and-velocity-
resonance.md` updated with the final tamed numbers (or Part 2 marked
resolved) once the bar is met; Completion Notes record whichever of the two
outcomes in the last Acceptance Criterion above actually occurred.
