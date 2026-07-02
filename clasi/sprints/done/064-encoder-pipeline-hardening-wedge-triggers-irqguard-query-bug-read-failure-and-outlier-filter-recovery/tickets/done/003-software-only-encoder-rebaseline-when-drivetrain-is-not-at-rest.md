---
id: '003'
title: Software-only encoder rebaseline when drivetrain is not at rest
status: done
use-cases:
- SUC-003
depends-on: []
github-issue: ''
issue: encoder-reset-while-moving-latches-readback.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Software-only encoder rebaseline when drivetrain is not at rest

## Description

`Robot::resetEncoders()` → `MotorController::resetEncoderAccumulators()` →
`Motor::resetEncoder()` ×2 fires 3 atomic 0x46 reads + a readback-verify per
wheel (6+ atomic transactions) regardless of whether the wheels are
currently rotating. `Robot::distanceDrive()` (`D`-preemption) is the primary
trigger — stress-matrix arm 3 (2026-07-02 stand session) produced 13
transient latches / 10 cycles, persistent at ~80, from exactly this
mechanism.

A second, independent call site was found during planning:
`Planner::beginDistance()` (`source/control/PlannerBegin.cpp:261`) calls
`_mc_ctrl.resetEncoderAccumulators()` directly, before `Robot::
distanceDrive()`'s own call — every `D` command currently fires the full
hardware burst *twice*. A third call site, `SystemCommands.cpp`'s
`handleZero` (`ZERO enc`), goes through `Robot::resetEncoders()`.

## Acceptance Criteria

- [x] `IVelocityMotor` (`source/hal/capability/IVelocityMotor.h`) gains
      `virtual void rebaselineSoft() = 0;` (pure — both current
      implementers are updated in this ticket) and `virtual uint32_t
      hardResetCount() const { return 0; }` / `virtual uint32_t
      softResetCount() const { return 0; }` (default-returning-zero, so any
      other implementer outside `Motor`/`SimMotor` keeps compiling
      unmodified).
- [x] `Motor::rebaselineSoft()`: folds the already-tick-cached
      `_lastPositionMm` (obtained by the normal per-tick 0x46 read, NOT a
      new atomic burst) back into raw tenths-of-degrees and adds it to
      `_encOffset` — **issues no I2C transaction** — then zeros the cache
      exactly as `resetEncoder()`'s success path already does (keeps
      `Motor`'s own `positionMm()` in lockstep with the host-side baselines
      `Robot::resetEncoders()`/`Drive::resetEncoders()` zero
      unconditionally afterward — a mismatch here would look like a fresh
      outlier-filter freeze).
- [x] `Motor::resetEncoder()` increments a new `_hardResetCount`;
      `rebaselineSoft()` increments a new `_softResetCount`; both exposed
      via the new `IVelocityMotor` accessors.
- [x] `MotorController` gains two new members, `_lastVelMmsL/R`, refreshed
      each `controlTick()` call from `inputs.velMms[]` *after* that tick's
      per-wheel ZOH velocity update runs.
- [x] `MotorController::resetEncoderAccumulators()` (unchanged signature)
      computes an at-rest decision internally: commanded component
      (`_cmds->tgtMms[0]==0.0f && _cmds->tgtMms[1]==0.0f`) AND measured
      component (`|_lastVelMmsL| < kAtRestVelEpsilonMms &&
      |_lastVelMmsR| < kAtRestVelEpsilonMms`, default epsilon 5 mm/s). At
      rest: call `resetEncoder()` on both wheels (unchanged hardware
      re-prime). Not at rest: call `rebaselineSoft()` on both wheels.
- [x] No call-site signature changes anywhere: `Robot::distanceDrive()`,
      `SystemCommands::handleZero`, and `Planner::beginDistance()` all get
      the new behavior automatically through the single
      `resetEncoderAccumulators()` choke point.
- [x] `SimMotor::rebaselineSoft()` performs the same effect
      `resetEncoder()` already does in sim (zero the reported accumulator
      via `_mut.resetReportedEncoder()`) — sim has no I2C timing race to
      avoid. `SimMotor` also implements `hardResetCount()`/
      `softResetCount()`, incremented from `resetEncoder()`/
      `rebaselineSoft()` respectively.
- [x] New sim hooks in `tests/_infra/sim/sim_api.cpp`:
      `sim_get_motor_hard_reset_count_l/r`, `sim_get_motor_soft_reset_count_l/r`.
- [x] `uv run --with pytest python -m pytest -q` is green (2 known-baseline
      failures allowed, no new failures).

**Implementation note (found during this ticket, not previously called
out):** `IVelocityMotor` had a THIRD implementer beyond `Motor`/`SimMotor` —
`NoopVelocityMotor` (`source/hal/NoopDevices.h`), used by `ReplayHAL`/
`Hardware.h` default HAL slots. Since `rebaselineSoft()` is pure virtual,
this class needed a no-op override (matching its existing no-op
`resetEncoder()`) to keep compiling. Added; no behavior change (Noop stays a
do-nothing stub). `hardResetCount()`/`softResetCount()` needed no override
there (safely defaulted, per the interface's own design).

## Testing

- **Existing tests to run**: full default suite; `test_encoder_reset.py`,
  `test_d_distance_baseline_race.py`, `test_motion_command.py` in
  particular (these exercise the existing reset-on-D-start behavior that
  must stay correct when the drivetrain genuinely IS at rest).
- **New tests to write**: a sim test reproducing stress-matrix arm 3 —
  start a `D` command, let the wheels reach nonzero velocity, issue a
  second `D` before the first completes (preemption), and assert:
  - `sim_get_motor_soft_reset_count_l/r()` incremented (software rebaseline
    was used).
  - `sim_get_motor_hard_reset_count_l/r()` did NOT increment during the
    preemption.
  - The encoder baseline after the mid-motion reset does not jump
    (`sim_get_enc_l/r()` immediately after the reset is close to the
    pre-reset `positionMm()`, not a large spurious value).
  Also test the at-rest path is unchanged: issue a `D` from idle and assert
  `hardResetCount` increments (hardware re-prime still happens when
  genuinely at rest).
- **Verification command**: `uv run --with pytest python -m pytest -q`
