---
id: '002'
title: "Add |\u0394PWM| slew cap to Motor::setSpeed"
status: in-progress
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: encoder-reset-while-moving-latches-readback.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add |ΔPWM| slew cap to Motor::setSpeed

## Description

`Motor::setSpeed()` (`source/hal/real/Motor.cpp`) exempts a stop (`pct==0`)
and a direction reversal from its 40 ms write-rate throttle, writing either
*immediately* — and for a reversal, with the **full** requested swing (a
−100 → +100 command is a 200-point step) in one 0x60 transaction while 0x46
traffic may be in flight. The 2026-07-02 stand session's stress-matrix arm 5
(`S +400` → `S −400`, guard ON, no resets involved) reproduced a persistent
encoder-readback latch from this trigger alone. See
`docs/knowledge/2026-07-01-encoder-wedge-boundary-latch-flavor.md`.

This ticket bounds the magnitude of any single 0x60 write.

## Acceptance Criteria

- [x] New header `source/hal/real/MotorSlew.h`: a small, dependency-free
      (no CODAL/MicroBit include) helper, e.g.
      `int8_t clampStep(int8_t lastWritten, int8_t target, uint8_t
      maxDelta)` — returns `target` unchanged if `|target - lastWritten| <=
      maxDelta`, else steps `lastWritten` by `maxDelta` toward `target`.
- [x] `Motor::setSpeed()` uses this helper for every write **except** the
      `pct == 0` stop path, which stays a full, immediate write (safety
      exemption — do not change stop behavior).
- [x] `_lastWrittenPct` is updated to the *clamped* value actually written,
      not the caller's requested `pct` — so the existing write-on-change
      guard (`if (pct == _lastWrittenPct) return;`) causes a large reversal
      to converge over several consecutive `setSpeed()` calls instead of one
      instant slam. The existing reversal exemption from the 40 ms *rate*
      throttle is unchanged (a reversal is still written every tick,
      un-throttled) — only the *magnitude* per write is now bounded.
- [x] `kMaxDeltaPwmPerWrite = 25` (of the ±100 range), declared as a named
      constant with a comment marking it a design-time estimate pending
      bench confirmation (matching the existing `// BENCH-CONFIRM`-style
      convention used elsewhere in `Motor.cpp`, e.g. `readSpeed()`'s
      `kUnitFactor`).
- [x] `SimMotor::setSpeed()` is explicitly **not** modified — no slew
      logic added in sim (see architecture-update.md Design Rationale 3:
      the golden-TLM canary and other tests assume instant PWM application
      in sim; the hazard this cap defends against is real-I2C-only).
- [x] `uv run --with pytest python -m pytest -q` is green (2 known-baseline
      failures allowed, no new failures) — this includes confirming
      `test_golden_tlm.py` and any other sim test that commands a raw
      reversal is unaffected, since `SimMotor` is untouched.

## Testing

- **Existing tests to run**: full default suite, with particular attention
  to `test_golden_tlm.py`, `test_033_005_wedge_hardening.py`, and any test
  that commands rapid `S`/raw reversals — these must be byte-for-byte
  unaffected since `SimMotor` is not modified.
- **New tests to write**:
  - `MotorSlew.h`'s `clampStep()` is pure and CODAL-free, so it compiles
    into `libfirmware_host`. Add a new `sim_motor_clamp_slew(current,
    target, maxDelta)` C-ABI hook in `tests/_infra/sim/sim_api.cpp`
    (mirroring the existing `sim_parse_schema` hook pattern) and a new
    `tests/simulation/unit/test_motor_slew.py` exercising: no clamp needed
    when within cap; clamp toward target when exceeding cap in either
    direction; multi-call convergence (calling repeatedly with the same
    target eventually reaches it); `pct==0` is not part of this pure
    function's contract (document that the *caller*, `Motor::setSpeed`,
    special-cases stop — the pure helper itself has no stop concept).
  - `Motor::setSpeed()` itself is not reachable from `HOST_BUILD`
    (`source/hal/real/` is excluded from the sim library — see
    `tests/_infra/sim/CMakeLists.txt`); verify the call site by code review
    against this ticket's diff, cross-checked with the `MotorSlew.h` unit
    test above. Do not attempt to make `Motor.cpp` HOST_BUILD-reachable —
    that is out of scope and a pre-existing, accepted constraint on all of
    `hal/real/`.
- **Verification command**: `uv run --with pytest python -m pytest -q`
