---
status: pending
review: docs/code_review/2026-07-01-full-codebase-review.md
findings: CR-02, CR-03
severity: high
sprint: '064'
---

# Encoder integrity: I2C read failures fabricate jumps, and the outlier filter has no recovery path

## Problem

Two compounding defects in the encoder pipeline — together the best candidate
for the residual "encoder odometry wrong / robot freaks out" reports.

**(a) I2C reads ignore failure (CR-03).** `collectEncoder()`
([Motor.cpp:305-325](../../source/hal/real/Motor.cpp)),
`readEncoderMmFSettle()` (336-354, the per-tick path), and
`readEncoderAtomic()` (215-263) never check `_i2c.read()`/`write()` return
codes. On failure the response buffer stays `{0,0,0,0}` so the "position"
becomes `0 − _encOffset` — a jump to a large arbitrary value.
`readSpeedRaw()` (462-478) shows the correct pattern (checks both, returns a
sentinel). Worst case: `resetEncoder()`'s median-of-3 uses atomic reads, so
three consecutive failures produce a confidently-wrong offset. The
`EVT ROTSTOP` diagnostic in MotionCommand.cpp exists because garbage reads
have corrupted turn baselines on the bench — this is the untreated source.

**(b) Outlier filter freezes permanently after one large divergence (CR-02).**
`Drive::_runOutlierFilter`
([Drive.cpp:394-444](../../source/subsystems/drive/Drive.cpp)) rejects any
per-tick delta > max(40 mm, 0.2·target) and holds the previous value; retries
accept a fresh read only if it lands near the **same stale baseline**.
`_filterRejectStreakL/R` are incremented but **never consumed** —
`kFilterRejectStreakThreshold = 3` is still declared
([Drive.h:148](../../source/subsystems/drive/Drive.h)) but the legacy
streak-based rebaseline was lost in the sprint-060 ordered-tick cutover. The
filter also runs only `if (driving)`, so `_hw.encMm[]` is never refreshed
while idle.

**Failure scenario:** operator lifts/rolls the robot while idle (wheels move
> 40 mm) → next VW/TURN/G starts → every fresh read rejected forever →
encoders frozen → `Odometry::predict` sees zero deltas → heading/pose frozen
→ TURN/G spin at commanded ω until the TIME net expires (freakout-shaped).
Only `D` escapes, because `distanceDrive` calls `resetEncoders()`.

## Fix direction

- Check I2C return codes in all encoder read paths; on failure report
  "no reading" and hold the last value (plus a failure counter for telemetry)
  rather than fabricating `−offset`.
- Restore streak recovery: after `kFilterRejectStreakThreshold` consecutive
  rejections, rebaseline `_hw.encMm` to the fresh reading.
- Refresh the baseline (without integrating) while idle or on the
  idle→driving transition.

## Acceptance / tests

- Sim test: jump the plant encoders while idle (hand-reposition analogue),
  then command a TURN — odometry must track the turn (filter recovers).
- Sim test: injected encoder-read failure for N ticks (needs a small sim hook
  analogous to `sim_set_otos_read_failure`) — pose must not jump.
