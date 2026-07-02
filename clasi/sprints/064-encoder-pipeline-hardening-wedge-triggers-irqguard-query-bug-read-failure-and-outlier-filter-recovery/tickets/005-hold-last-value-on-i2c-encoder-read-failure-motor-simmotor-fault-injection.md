---
id: '005'
title: Hold-last-value on I2C encoder read failure (Motor + SimMotor fault injection)
status: open
use-cases: [SUC-006]
depends-on: []
github-issue: ''
issue: encoder-integrity-i2c-failures-and-outlier-filter-recovery.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Hold-last-value on I2C encoder read failure (Motor + SimMotor fault injection)

## Description

CR-03 (`clasi/issues/encoder-integrity-i2c-failures-and-outlier-filter-recovery.md`):
`Motor::collectEncoder()`, `readEncoderAtomic()`, `readEncoderMmFSettle()`,
and `requestEncoder()` (`source/hal/real/Motor.cpp`) never check
`_i2c.read()`/`_i2c.write()` return codes. On failure the response buffer
stays `{0,0,0,0}`, so the computed position becomes `0 - _encOffset` — a
jump to a large, arbitrary value. `Motor::readSpeedRaw()` (same file)
already shows the correct pattern: check both calls' return codes, return a
sentinel/hold on failure.

## Acceptance Criteria

- [ ] `Motor` gains `mutable int32_t _lastGoodRawEnc` (raw ticks,
      offset-applied — the same domain `collectEncoder()`/
      `readEncoderAtomic()` already return), updated on every successful
      read.
- [ ] `collectEncoder()`, `readEncoderAtomic()`, `readEncoderMmFSettle()`
      check the I2C return code(s) and return/derive from
      `_lastGoodRawEnc` on failure instead of computing from a zeroed
      buffer.
- [ ] `requestEncoder()`'s write status is cached (e.g. `mutable bool
      _pendingEncRequestOk`); `collectEncoder()` treats either half (the
      phase-1 write or its own phase-2 read) failing as a combined failure
      — a failed request means the phase-2 response, even if its own
      `read()` call reports OK, is for a stale prior request.
- [ ] `readEncoderMmF()` is verified to already delegate to
      `collectEncoder()` (no separate change needed there) — confirm this
      during implementation and note it in the ticket's completion notes if
      it does not.
- [ ] `resetEncoder()`'s median-of-3 + readback-verify + retry loop is
      **not** given new explicit failure-tracking (relies on the fix above
      plus its own existing retry loop — see architecture-update.md Design
      Rationale 4). Do not add new state here beyond what already exists.
- [ ] `SimMotor` (`source/hal/sim/SimMotor.{h,cpp}`) gains
      `setReadFailure(bool)`, mirroring the existing
      `SimOdometer::setReadFailure` / `sim_set_otos_read_failure` pattern:
      when injected, `tick()` does not promote a fresh `reportedEncMm()`
      (holds `_lastPositionMm`), and `readEncoderMmFSettle()` /
      `readEncoderMmFAtomic()` / `collectEncoder()` likewise hold their
      last cached value.
- [ ] New sim hook `sim_set_motor_read_failure(h, int side, int fail)` in
      `tests/_infra/sim/sim_api.cpp` (side: 0=left, 1=right, other=both,
      matching the existing `sim_set_motor_slip` convention).
- [ ] `uv run --with pytest python -m pytest -q` is green (2 known-baseline
      failures allowed, no new failures).

## Testing

- **Existing tests to run**: full default suite; `test_motor_controller*.py`,
  `test_drive_subsystem.py`, `test_ekf*.py` in particular (these exercise
  the pipeline the fault-injection model feeds into).
- **New tests to write**: a new sim test (e.g.
  `tests/simulation/unit/test_encoder_read_failure.py` or added to
  `test_drive_subsystem.py`) that:
  1. Starts an active drive command (`D` or `TURN`).
  2. Injects a read failure on one wheel via `sim_set_motor_read_failure`
     for N ticks.
  3. Asserts the fused pose (`sim_get_pose_x/y/h`) does not jump beyond the
     tolerance used by existing pose-stability tests (this is the issue's
     own stated acceptance criterion).
  4. Clears the failure and asserts normal tracking resumes.
  This exercises the full downstream pipeline (`Drive::_runOutlierFilter` →
  `MotorController::controlTick` → `Odometry`/EKF) even though `Motor.cpp`'s
  own new I2C-status-check lines are not directly reachable from
  `HOST_BUILD` — see the note below.
- **Known testability gap (pre-existing, not introduced by this ticket)**:
  `source/hal/real/Motor.cpp` is excluded from `HOST_BUILD`
  (`tests/_infra/sim/CMakeLists.txt`), so the real `Motor`'s new
  status-check lines cannot be unit-tested directly. Verify by close
  pattern-matching against `readSpeedRaw()` (an already-shipped, already
  -reviewed template for this exact class of fix) during code review. The
  `SimMotor` fault-injection test above validates the *consuming pipeline's*
  contract, which is the acceptance-critical behavior.
- **Verification command**: `uv run --with pytest python -m pytest -q`
