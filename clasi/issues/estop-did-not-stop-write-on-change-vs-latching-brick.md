---
status: pending
priority: critical
---

# ESTOP did not stop the robot — write-on-change vs. a latching motor controller

**This one caused a real runaway on 2026-07-31.** 13 ESTOPs, a `WHEELS(0,0)`,
and an nRF52 reset all failed to stop the wheels. The stakeholder stopped the
robot by cutting power.

## Root cause

`Devices::NezhaMotor::write()` suppressed any duty write equal to the last one
it had *attempted*:

```cpp
if (pct == lastWrittenPct_) return;
```

`lastWrittenPct_` records the attempt, not the landing. The Nezha brick
**latches its last commanded speed**, and an nRF52 reset does not reach it —
only power does. So a single lost zero write is permanent: the host believes
the stop was sent, the motor keeps its last nonzero speed forever, and every
subsequent ESTOP is suppressed as a no-op because `lastWrittenPct_` already
says 0.

The two facts have to be held together to see it. Write-on-change is safe with
a stateless actuator; latching plus a lossy bus makes it a one-way door.

## Fix that was verified on hardware (v0.20260731.12, then abandoned)

1. `nezha_motor.cpp` — never suppress a zero write while the wheel is still
   moving:
   ```cpp
   const bool stopNotTaken = pct == 0 && fabsf(velocity()) > kStopConfirmVelocity;
   if (pct == lastWrittenPct_ && !stopNotTaken) return;
   ```
   with `kStopConfirmVelocity = 8.0f` in `nezha_motor.h`.

2. `drive.cpp`/`drive.h` — re-assert a commanded stop for `kStopEnforceTicks`
   (30) cycles after `estop()`, and unconditionally while either wheel reports
   motion above `kRestVelocity` (8.0 mm/s). A stop is asserted until it is
   *observed*, not until it is *sent*.

Verified on the stand: `vel (0,0)`, `enc (0,0)` held for 3 s after ESTOP.

## Why it matters beyond this bug

Every "halt now" path — geofence, Ctrl-C, panic — depends on this. Per
`.claude/rules/playfield-testing.md`, `estop()` is the only correct halt verb;
that rule is now only true again once this fix is back in.

## Acceptance

- Drive at 150 mm/s, `estop()` mid-leg, confirm encoders stop advancing within
  0.15 s and stay stopped for 3 s with nothing else commanding.
- Repeat 10x consecutively without a power cycle — the original defect needed a
  *lost* write to appear, so a single pass proves nothing.
- Add a firmware unit test: a motor whose write is dropped still re-asserts zero
  on the next tick while velocity is nonzero.

Related: [[unmanaged-drive-lease-expiry-and-terminal-pivot]]
