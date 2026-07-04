---
status: done
tickets:
- NONE
---

# OZ / SI never re-anchor the bench OTOS — second tour in a session is corrupted

## Problem

In bench mode the EKF fuses the BenchOtosSensor's absolute pose every
tick, but no wire command can re-anchor that sensor:

- `OZ` → `OtosCommands` `c->otos->setPositionRaw(0,0,0)`. `OtosCtx.otos`
  is bound **once in the Robot constructor** to the cached `robot->otos`
  reference (the real chip), so it never sees the bench swap — and even
  if it did, `BenchOtosSensor::setPositionRaw()` is a **no-op stub**
  ([BenchOtosSensor.cpp:136](source/hal/real/BenchOtosSensor.cpp#L136)).
- `SI` (`handleSetPose`, SystemCommands.cpp ~755) calls
  `robot->otos.setWorldPose(...)` — again the cached construction-time
  reference to the real chip, never the bench sensor.

`SimOdometer` got exactly this fix for the sim in ticket 063-006
(`setPositionRaw` re-references the accumulator); the bench sensor was
never given the equivalent, and the 031-002 "reference reseating" fix
only covered `Robot::otosCorrect` (now dead code — the live path is
`Drive::tickUpdate` STEP 5, which does read `hal.otos()` live since
074-002).

Consequence: the **first** tour after boot works (accumulators start at
zero), but after any motion, the GUI's "Set Robot @ 0,0" reset
(`STOP; ZERO enc; OZ; SI 0 0 0`) zeroes the EKF but leaves the bench
accumulators in the old frame — and the EKF is immediately dragged back
to the stale pose. Every tour after the first is corrupted from step 1.

## Hardware evidence (2026-07-03, tovez on stand, fw 0.20260703.19)

After a bench-mode Tour 1 ended at bench pose `(305,-130,-42.9°)`, the
GUI origin-reset sequence was sent, then `DBG OTOS` sampled:

- Bench accumulators after `OZ` + `SI 0 0 0`: unchanged
  `ideal=305,-129,-4296 otos=305,-130,-4288` at +0.5 s, +1.0 s, +2.0 s.
- SNAP 3.5 s after the reset: `pose=304,-129,-4287` — the fused pose was
  dragged from the freshly-set `(0,0,0)` back to the stale bench frame.
  (`encpose=-67,78,-15125` kept its own reset frame.)

## Proposed fix

1. Implement `BenchOtosSensor::setPositionRaw(x,y,h)` to re-reference
   both accumulators (scale parity with the real chip's LSBs, mirroring
   SimOdometer 063-006). `setWorldPose` already exists and works.
2. Route `OZ`/`OI`/`OR` and `SI`'s OTOS re-anchor through the **live**
   `hal.otos()` (as Drive does since 074-002) instead of
   construction-bound references, so whichever odometer is active gets
   the fix.
