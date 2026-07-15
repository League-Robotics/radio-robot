---
status: in-progress
sprint: '103'
tickets:
- 103-002
---

# NezhaMotor write-path hardening: commit state only on successful write; no busy-spin clearances

From the 2026-07-13 code review (docs/code_review/2026-07-13-devices-drive-review.md,
Part 1 findings **C1** and **M1**). Both defects live in the `Devices::NezhaMotor` /
`Devices::I2CBus` leaves, which the single-loop rebuild KEEPS — these fixes survive the
rebuild and are scheduled inside its P3 ("fix C1 and M1 in the port"). Filed separately
so they are tracked if the rebuild slips, and because C1 is a live safety defect in the
firmware running today.

## C1 (critical): a NAK'd duty write is latched as written; write-on-change suppresses all retries

`writeRawDuty` (source/devices/nezha_motor.cpp:325-372) ignores `bus_.write()` status —
`lastWrittenPct_`/`lastWriteTimeUs_` update unconditionally, and the write-on-change
check (line 333) then suppresses every retry of the same value. Failure scenario: a
transient NAK on a **stop** write (pct==0) is permanently lost — the watchdog's
"re-asserts Neutral every cycle" robustness is defeated (`armoredWrite(0)` early-returns
forever on `pct == lastWrittenPct_ == 0`), `appliedDuty()` reads 0.0 while the wheel
physically drives, which also blinds `wedgeSuspect()`. Wheels keep spinning with no
signal and no recovery until a nonzero duty is commanded.

**Fix:** commit `lastWrittenPct_`/`lastWriteTimeUs_` only on `status == kOk`; treat a
failed pct==0 write as must-retry-next-tick (stop is already throttle-exempt).

## M1 (major): inter-transaction clearance is a hard busy-spin that blocks the scheduler

`i2c_bus.cpp:67-68, 112-113`: clearance waits are `while(clockUs()<deadline){}` with no
yield — ~4 ms of scheduler-blocking spin nearly every cycle (motor duty write stamps
readyAt=+4ms, next brick txn follows immediately), ~32 ms per motor during preamble
hardReset, back-to-back spins on every OTOS write-then-read pair. Violates the
loop-must-yield rule (radio/serial fibers starve).

**Fix:** in the single-loop design, the required gaps become explicit `runAndWait`
blocks / `sleepUntil` calls owned by the loop; the I2CBus per-device readyAt stamps
remain only as a sleep-not-spin safety net that raises a telemetry fault bit when it
fires (it should never fire if the loop schedule is right).
