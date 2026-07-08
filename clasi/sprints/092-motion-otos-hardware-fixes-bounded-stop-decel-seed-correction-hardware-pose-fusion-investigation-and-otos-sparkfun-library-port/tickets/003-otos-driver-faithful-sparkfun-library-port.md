---
id: '003'
title: 'OTOS driver: faithful SparkFun library port'
status: done
use-cases:
- SUC-003
depends-on: []
github-issue: ''
issue: otos-lever-arm-necessity-and-library-port.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS driver: faithful SparkFun library port

## Description

`source/hal/otos/otos_odometer.{h,cpp}` already implements the OTOS
register map, matching scaling constants, and 086-007's hard-won
bus-clearance/rate-limiting safety fixes -- but it is a partial port: there
is no `setOffset()`/`getOffset()` (the mounting-offset `REG_OFFSET`
register, 0x10-0x15, is never written today), and other upstream primitives
(full signal-process config detail, IMU calibration parity, product-ID
check parity) are unexercised or approximate rather than a faithful,
near-line-by-line port of the upstream SparkFun reference implementation
(`clasi/issues/otos-lever-arm-necessity-and-library-port.md`).

This ticket is the CODE-DOABLE, sim-testable half of that issue: extend
`Hal::OtosOdometer` to the full upstream primitive surface, in place (not a
rewrite -- see `architecture-update.md` Decision 6 for why 086-007's
bus-safety fixes must not be touched). This ticket does NOT resolve whether
the chip actually honors `REG_OFFSET`, and does NOT decide `lever_arm.h`'s
fate -- that is ticket 004's job, which depends on this one for
`setOffset()`/`getOffset()` to exist.

**Note**: this ticket does not, by itself, complete the linked issue (hence
`completes_issue: false`) -- ticket 004 finalizes the lever-arm disposition
and is the one that closes the issue out.

## Acceptance Criteria

- [x] `Hal::OtosOdometer` gains `setOffset()`/`getOffset()` writing/reading
      `REG_OFFSET` (0x10-0x15), using the SAME `writeReg8()`/`readReg8()`-
      style helpers and `kBusClearance` discipline every existing register
      access in this class already uses (Decision 6) -- not a
      second, independently-written I/O path.
- [x] Any other genuinely-missing upstream primitive the port finds (full
      signal-process config, IMU calibration parity, product-ID check
      parity) is added, conforming to `.claude/rules/naming-and-style.md`
      (CamelCase types / lowerCamelCase functions, no units in identifiers,
      wire/register token names exempt).
- [x] 086-007's existing bus-clearance (`kBusClearance`) and rate-limiting
      (`kReadPeriod`) behavior is verified unchanged -- no regression to
      the CODAL `NRF52I2C::waitForStop()` stall fix that sprint spent real
      stand time eliminating.
- [x] **Sim, BLOCKING**: new unit coverage for the ported register surface
      (register scaling round-trips for `setOffset`/`getOffset` at the
      `Hal::Odometer` interface level, via the sim leaf/mock bus).
- [x] Full `uv run python -m pytest tests/sim` is green.

## Implementation Plan

**Approach**:
1. Read the upstream SparkFun OTOS library (Arduino C++ reference,
   <https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library/>) and
   diff its primitive surface against the current `Hal::OtosOdometer`.
2. Add `setOffset()`/`getOffset()` and any other missing primitives,
   in place, reusing existing helpers (`writeReg8()`/`readReg8()`/
   `writeXYH()`-style burst writes) and existing bus-safety constants.
3. Add unit coverage for the new surface.
4. Verify no regression to the existing 086-007 safety behavior (read the
   existing bus-clearance/rate-limit tests, confirm they still pass
   unchanged).

**Files to modify**: `source/hal/otos/otos_odometer.h`,
`source/hal/otos/otos_odometer.cpp`,
`tests/sim/unit/otos_odometer_harness.cpp` (or wherever existing OTOS
driver unit coverage lives), corresponding `test_*.py`.

**Testing plan**:
- **Existing tests to run**: full `uv run python -m pytest tests/sim`,
  paying particular attention to any existing OTOS-driver / bus-clearance
  tests.
- **New tests to write**: register scaling round-trip for
  `setOffset`/`getOffset`.
- **Verification command**: `uv run python -m pytest tests/sim`.

**Documentation updates**: update `otos_odometer.h`'s file header comment
to reflect the ported surface (it currently states `REG_OFFSET` is
"deliberately NEVER written" -- that statement becomes false once this
ticket lands and must be corrected, independent of ticket 004's later
bench verdict on whether the chip actually HONORS the write).

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim` (full
  suite).
- **New tests to write**: `setOffset`/`getOffset` register round-trip
  coverage.
- **Verification command**: `uv run python -m pytest tests/sim`.

## Completion Notes

**Upstream reference used**: fetched the actual current upstream C++ driver
(`sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library`, `src/sfTk/sfDevOTOS.{h,cpp}`)
via `gh api` rather than relying on memory, to get the exact register map,
helper structure, and scaling constants right.

**Primitives added to `Hal::OtosOdometer`** (`source/hal/otos/
otos_odometer.{h,cpp}`), all additive, none replacing existing behavior:

- `setOffset(const msg::Pose2D&)` / `getOffset()` -- `REG_OFFSET` (0x10),
  confirmed against upstream to share the EXACT SAME `writePoseRegs()`/
  `readPoseRegs()` helper and int16 scaling (`kMeterToInt16`/`kInt16ToMeter`,
  `kRadToInt16`/`kInt16ToRad`) that `REG_POSITION` (0x20) uses -- exactly the
  issue's own claim. Implemented by factoring `setPose()`'s former inline
  clamp+scale+write tail into a new shared private helper `writePoseMm()`
  (used by both `setPose()` and `setOffset()`, mirroring upstream's own
  single shared `writePoseRegs()`), and adding a new private `readXYH()`
  (a plain 6-byte int16-triple burst read, narrower than `tick()`'s own
  combined 12-byte `readPositionVelocity()`, which is untouched). No lever-
  arm/mounting-yaw transform is applied in `setOffset()`/`getOffset()` --
  they write/read the mounting-offset VALUE itself, not a world-frame pose.
- `setSignalProcessConfig(uint8_t)` / `signalProcessConfig()` --
  `REG_SIGNAL_PROCESS_CFG` (0x0E) raw get/set; `init()` now calls
  `setSignalProcessConfig(0x0F)` instead of a raw `writeReg8()` (identical
  transaction, `kBeginTxnCount` unchanged at 8).
- `imuCalibrationSamplesRemaining()` -- `REG_IMU_CALIBRATION` (0x06)
  read-back, the non-blocking counterpart to `init()`'s existing
  fire-and-forget calibration kick-off (mirrors upstream's
  `calibrateImu()`/`getImuCalibrationProgress()` split without introducing
  any blocking wait).
- Product-ID check parity was already present (`begin()`'s `kExpectedProductId
  == 0x5F` matches upstream's `kProductId` exactly) -- no change needed.

**092-003 finding, deliberately NOT fixed in this ticket** (documented in
`otos_odometer.h`'s `kPosMmPerLsb` comment and `tick()`'s velocity-conversion
comment): upstream uses a DIFFERENT LSB scale for the VELOCITY registers
than for position/offset (`kMpsToInt16 = 32768/5`, a 5 m/s full range, vs.
position/offset's 10 m full range; angular rate similarly differs from
heading's scale). This driver's `tick()` applies `kPosMmPerLsb`/
`kHdgRadPerLsb` to the velocity burst too -- a pre-existing behavior, not
introduced by this ticket. Fixing it would change live EKF-fusion twist
magnitudes on real hardware and needs its own bench-verifiable ticket, not a
sim-only port pass (Decision 6 -- add primitives, do not alter existing
`tick()` math without dedicated verification). Flagged explicitly for the
team-lead/stakeholder to decide whether to spin up a follow-on issue.

**086-007 regression check**: `kBusClearance` (4000us) and `kReadPeriod`
(20ms) are untouched; `writeReg8()`/`readReg8()`/`readPositionVelocity()`
are untouched (only a NEW sibling `readXYH()` was added, `tick()`'s own hot
path still calls `readPositionVelocity()` exclusively). Scenario 1
(`begin()` transaction count == 8) and scenario 8 (rate-limit behavior)
both still pass unchanged, proving no regression.

**Driver extended in place, not rewritten**: every existing method's
external behavior and every existing private helper (`writeReg8`,
`readReg8`, `readPositionVelocity`, `writeXYH`) is unchanged; `setPose()`'s
observable behavior is unchanged (its tail was factored into `writePoseMm()`
verbatim, same clamp/round/write sequence, same txn count -- proven by
scenario 7 still passing at "one write").

**Unit tests added** (`tests/sim/unit/otos_odometer_harness.cpp`, run via
the existing `test_otos_odometer.py` harness-compile-and-run wrapper -- no
new pytest file needed, matching this project's established one-pytest-
test-wraps-one-harness-binary convention, so the pytest-collected test
COUNT is unchanged at 311 even though C++ scenario coverage grew from 8 to
10 scenarios with substantially more assertions):

- `scenarioNeverInitializedEverySetterIsNoop` (extended): `setOffset()`/
  `getOffset()`/`setSignalProcessConfig()`/`signalProcessConfig()`/
  `imuCalibrationSamplesRemaining()` are all no-ops (zero bus traffic,
  zero/default return) when never initialized.
- `scenarioSetterTxnCounts` (extended): `setOffset()`/
  `setSignalProcessConfig()` each issue exactly one write.
- `scenarioSetOffsetGetOffsetScalingRoundTrip` (NEW): `setOffset()` writes a
  known mm/mm/rad offset (one write); a scripted read-back encoded with the
  identical LSB scaling is decoded by `getOffset()` (one write + one 6-byte
  read) and round-trips within one LSB on x/y/h -- the BLOCKING
  register-scaling round-trip this ticket's acceptance criteria require.
- `scenarioSignalProcessConfigAndImuCalibrationProgressReads` (NEW):
  `signalProcessConfig()`/`imuCalibrationSamplesRemaining()` each issue
  exactly one write + one read and return the raw scripted byte unmodified.

**Test/build results**:
- `otos_odometer_harness` (compiled directly, `-std=c++20 -Wall -Wextra
  -DHOST_BUILD`): all 10 scenarios pass, 0 failures.
- `uv run python -m pytest tests/sim`: **311 passed, 2 xfailed** (matches
  the stated baseline exactly -- unchanged count, by the harness-wrapping
  convention above; not a sign nothing changed).
- `just build` (full ARM firmware, `arm-none-eabi-g++`): builds and links
  clean (`Built target MICROBIT`/`MICROBIT_hex`), confirming the new
  primitives compile under the real CODAL/non-`HOST_BUILD` path too (no
  new warnings introduced; memory usage unchanged in any concerning way).
- `just build-sim`: host sim library (`libfirmware_host`) builds clean
  (this target doesn't compile `otos_odometer.cpp` at all -- `SimOdometer`
  is the sim-mode leaf -- so it's a build-system sanity check only, not
  coverage of this ticket's changes).

**Deferred to ticket 004 (per this ticket's own scope boundary,
`completes_issue: false`)**: whether `begin()`/hardware actually SWITCHES to
chip-native `REG_OFFSET` compensation (retiring the host-side lever arm) --
that depends on ticket 004's real bench re-test of whether THIS chip honors
the write, and its own Decision-7-governed DELETE-vs-FOLD disposition for
`source/hal/lever_arm.h`. This ticket only proves the register I/O
primitives work correctly in sim; `lever_arm.h` is untouched, `begin()`
still applies mounting-offset compensation host-side exactly as before.
