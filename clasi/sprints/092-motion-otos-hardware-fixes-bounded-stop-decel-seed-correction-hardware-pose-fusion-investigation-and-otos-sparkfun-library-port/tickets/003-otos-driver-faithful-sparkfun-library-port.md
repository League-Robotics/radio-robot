---
id: '003'
title: 'OTOS driver: faithful SparkFun library port'
status: open
use-cases: [SUC-003]
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

- [ ] `Hal::OtosOdometer` gains `setOffset()`/`getOffset()` writing/reading
      `REG_OFFSET` (0x10-0x15), using the SAME `writeReg8()`/`readReg8()`-
      style helpers and `kBusClearance` discipline every existing register
      access in this class already uses (Decision 6) -- not a
      second, independently-written I/O path.
- [ ] Any other genuinely-missing upstream primitive the port finds (full
      signal-process config, IMU calibration parity, product-ID check
      parity) is added, conforming to `.claude/rules/naming-and-style.md`
      (CamelCase types / lowerCamelCase functions, no units in identifiers,
      wire/register token names exempt).
- [ ] 086-007's existing bus-clearance (`kBusClearance`) and rate-limiting
      (`kReadPeriod`) behavior is verified unchanged -- no regression to
      the CODAL `NRF52I2C::waitForStop()` stall fix that sprint spent real
      stand time eliminating.
- [ ] **Sim, BLOCKING**: new unit coverage for the ported register surface
      (register scaling round-trips for `setOffset`/`getOffset` at the
      `Hal::Odometer` interface level, via the sim leaf/mock bus).
- [ ] Full `uv run python -m pytest tests/sim` is green.

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
