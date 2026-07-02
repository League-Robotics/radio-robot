---
id: '004'
title: Remove wedge-detector blind spots, add TLM wedge= field, auto re-prime at idle
status: done
use-cases:
- SUC-004
- SUC-005
depends-on:
- '003'
github-issue: ''
issue: encoder-reset-while-moving-latches-readback.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Remove wedge-detector blind spots, add TLM wedge= field, auto re-prime at idle

## Description

`MotorController::controlTick()`'s per-wheel wedge detector has two
structural blind spots that together caused it to miss **all** ~18 observed
wedge episodes in the 2026-07-01/02 sessions:

1. **Target==0 reset**: the stuck counter zeros every tick the wheel's
   target is 0 — including the transient tick(s) at a command's own
   deceleration/stop boundary, exactly where the latch onsets.
2. **Arming grace** (033-005d, `_hasMovedL/R`): counting does not start
   until the wheel has moved at least once since the current command
   started — a wheel that enters a *new* command already frozen never
   "moves," so counting never starts.

This ticket also adds a `wedge=` TLM field (the detector's only current
output, `EVT enc_wedged`, is a latched one-shot event that a relay/radio
link can drop) and an optional one-shot auto re-prime when a wedge is
detected while the drivetrain is at rest (reusing ticket 003's at-rest
decision).

## Acceptance Criteria

- [x] `controlTick()`'s per-wheel comparison becomes unconditional: an
      identical consecutive raw reading increments `_stuckCountW`; a
      changed reading resets it. Both the `tgtW != 0.0f` branch and the
      `_hasMovedW` gate are **removed** from the counting logic.
- [x] `_hasMovedL/R` fields are deleted, along with their clearing in
      `startDriveClean()`/`startDrive()`/`stop()`.
- [x] `kWedgeThreshold` (10) and the latched, single-shot `EVT enc_wedged`
      line are unchanged in format and firing semantics (still fires once
      per episode, still includes the raw-read + bus-diagnostics fields).
- [x] `RobotTelemetry::buildTlmFrame()` (`source/robot/RobotTelemetry.cpp`)
      gains `wedge=<L>,<R>` (0/1 per wheel, L-then-R wire order matching
      `enc=`/`vel=`), reading `drive.state().wheel_wedged()[1]` (L) and
      `[0]` (R). **Emitted unconditionally**, not gated by
      `config.tlmFields` (see architecture-update.md Design Rationale 2 —
      the bitmask's `uint8_t` has all 8 bits already assigned).
- [x] `Drive::tickUpdate()`'s existing wedge-push step (STEP 3) gains: if
      `anyWedged` and the drivetrain is at rest (reuse ticket 003's at-rest
      concept — do not duplicate the epsilon/decision logic) and no
      re-prime has been attempted for this latch episode, call
      `_mc.resetEncoderAccumulators()` and set a new one-shot flag. The
      flag clears when `anyWedged` next goes false (mirror the existing
      `_prevAnyWedged` pattern already in `tickUpdate`). One shared flag
      (not per-wheel) is sufficient.
- [x] `uv run --with pytest python -m pytest -q` is green (2 known-baseline
      failures allowed, no new failures) — **including updating
      `tests/simulation/unit/test_golden_tlm.py`'s captured expected TLM
      frame(s)** to include the new unconditional `wedge=` field (this WILL
      change frame content/length; regenerate the fixture rather than
      trying to preserve the old one).

## Testing

- **Existing tests to run**: full default suite;
  `tests/simulation/system/test_033_005_wedge_hardening.py` and
  `test_golden_tlm.py` specifically (the latter needs its fixture
  regenerated per the acceptance criteria above).
- **New tests to write**:
  - Detector-fires-on-frozen-from-start: inject a frozen wheel via
    `sim_set_motor_offset`/frozen `SimMotor` state (or the read-failure
    hook from ticket 005 if landed first) at the START of a new command
    (not mid-command), and assert `sim_get_wheel_wedged_l/r()` becomes true
    within `kWedgeThreshold` ticks — reproducing Episode A (RT turn frozen
    for 14 frames, zero EVT under the old logic).
  - Detector-survives-target-zero-boundary: freeze a wheel during the tail
    of a `D` command's deceleration (target crossing to 0), and assert the
    stuck streak is NOT reset at the boundary — it continues accumulating
    into the next command.
  - `wedge=` field presence: `SNAP`/one TLM frame while both wheels healthy
    → `wedge=0,0`; force a wedge → next frame shows `wedge=1,0` or
    `wedge=0,1`.
  - Auto re-prime: force a transient wedge while the drivetrain is at rest,
    advance sim ticks, and assert `sim_get_motor_hard_reset_count_l/r()`
    incremented exactly once (not repeatedly) and the wedge latch clears if
    the injected fault is also cleared. Also assert re-prime does NOT fire
    while `anyWedged` is true but the drivetrain is still moving.
- **Verification command**: `uv run --with pytest python -m pytest -q`
