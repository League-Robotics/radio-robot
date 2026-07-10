---
status: pending
---

# OTOS REG_OFFSET bench re-test deferred — confirm whether the chip honors chip-native mounting-offset compensation

> **Parked to `later/` 2026-07-09 (stakeholder triage).** The OTOS command
> family (`OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`) is unregistered on the gutted
> post-093/094 wire surface and nothing fuses the odometer while
> `Subsystems::PoseEstimator` is unticked — the re-test has no wire path to
> run through. `Hal::OtosOdometer` (with the ported `setOffset()`/
> `getOffset()`) is still constructed at boot by `NezhaHardware`, so the
> test becomes runnable again the moment the OTOS/pose path is restored
> ([[restore-goto-pursuit-with-pose-estimator]]) — do it then.

## Context

Sprint 092 ticket 003 ported the SparkFun OTOS library faithfully into
`Hal::OtosOdometer` (`source/hal/otos/otos_odometer.{h,cpp}`), adding
`setOffset()`/`getOffset()` — a working, sim-tested I2C read/write of
`REG_OFFSET` (0x10-0x15) sharing the exact same write/read path and int16
scaling (`kPosMmPerLsb`/`kHdgRadPerLsb`) that `REG_POSITION` (0x20) already
uses successfully. Ticket 004 was supposed to bench-re-test whether *this
robot's actual chip* honors a `REG_OFFSET` write (the prior "unwritable
register" claim in `source_old/hal/real/OtosSensor.cpp` was suspect —
see `clasi/sprints/092-.../issues/otos-lever-arm-necessity-and-library-port.md`)
and, based on that verdict, finalize `source/hal/lever_arm.h`'s disposition
into exactly one end state (DELETE if the chip honors the write cleanly,
FOLD host-side compensation into `Hal::OtosOdometer` otherwise).

**The bench could not be reached this sprint.** The robot's serial port
(`/dev/tty.usbmodem2121102`) was held by an unrelated local process (a VS
Code extension-host, per `lsof` — the same blocker ticket 092-002 hit
independently the same session, see
`clasi/issues/poseestimator-fused-pose-fix-pending-otos-connected-bench-confirmation.md`),
and the radio relay dongle was unplugged. No bench evidence either way
exists.

Per `architecture-update.md` Decision 7, the default disposition when the
bench cannot be run or is inconclusive is **FOLD, never DELETE** —
deleting a possibly-still-needed compensation on an unconfirmed assumption
risks a live-hardware regression (the `db11b7c` ~433mm phantom-translation
signature). Ticket 092-004 applied that default: `source/hal/lever_arm.h`
no longer exists as a standalone file; its two functions
(`sensorToCentre()`/`centreToSensor()`) are now **private methods of
`Hal::OtosOdometer`** (`source/hal/otos/otos_odometer.{h,cpp}`), with
identical behavior — a pure relocation, not a fix. `setOffset()`/
`getOffset()` remain unused from `begin()`; host-side compensation is
still the live path on every `tick()`/`setPose()` call.

## Ask

Once the bench is reachable again (free serial port or working relay),
re-run the re-test 092-004 deferred:

1. On the stand (wheels off the ground, safe to spin —
   `.claude/rules/hardware-bench-testing.md`), write `REG_OFFSET` with the
   real mounting offset via `Hal::OtosOdometer::setOffset()`.
2. Read it back via `getOffset()`. Non-zero readback consistent with what
   was written = the chip's register I/O honors the write; readback of
   zero (or garbage) = it does not.
3. Drive a pure in-place spin and check for the lever-arm phantom-
   translation arc (the `db11b7c` signature `sensorToCentre()`'s own doc
   comment documents, `otos_odometer.h`). Arc absent + non-zero readback =
   the chip compensates internally; arc still present = it does not,
   regardless of what the register reads back.
4. Record the verdict explicitly (register readback value, spin-test
   translation magnitude, in mm).

## Acceptance

- A clean bench verdict, either way, recorded here or in a fresh ticket.
- **If the chip HONORS `REG_OFFSET`** (clean positive confirmation): open a
  follow-up ticket to delete `Hal::OtosOdometer::sensorToCentre()`/
  `centreToSensor()` and their call sites in `tick()`/`setPose()`; make the
  offset a one-time device write in `begin()` (calling the already-ported
  `setOffset()`); remove the private methods' dedicated coverage in
  `tests/sim/unit/otos_odometer_harness.cpp` (the
  `scenarioLeverArm*()` scenarios and `testSensorToCentre()`/
  `testCentreToSensor()` added by 092-004).
- **If the chip does NOT honor it, or the re-test is inconclusive again**:
  close this issue — 092-004's FOLD disposition stands as the project's
  final answer, not a placeholder.

## References

- `clasi/sprints/092-motion-otos-hardware-fixes-bounded-stop-decel-seed-correction-hardware-pose-fusion-investigation-and-otos-sparkfun-library-port/tickets/004-otos-reg-offset-bench-re-test-and-lever-arm-disposition.md`
  (this issue's completion notes have the full disposition trace)
- `clasi/sprints/092-.../architecture-update.md` Decision 7
- `clasi/sprints/092-.../issues/otos-lever-arm-necessity-and-library-port.md`
  (original hypothesis, completed by ticket 004 via the FOLD default)
- `source/hal/otos/otos_odometer.h`'s own "092-004 update" file-header
  paragraph and `sensorToCentre()`/`centreToSensor()` declaration comments
- `clasi/issues/poseestimator-fused-pose-fix-pending-otos-connected-bench-confirmation.md`
  (same session, same serial-port blocker, independently hit)
