---
id: '004'
title: OTOS REG_OFFSET bench re-test and lever-arm disposition
status: open
use-cases: [SUC-004]
depends-on: ['003']
github-issue: ''
issue: otos-lever-arm-necessity-and-library-port.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS REG_OFFSET bench re-test and lever-arm disposition

## Description

Ticket 003 ports `setOffset()`/`getOffset()` into `Hal::OtosOdometer`. This
ticket bench-re-tests whether the chip actually HONORS a `REG_OFFSET`
write (the prior "unwritable register" claim in
`source_old/hal/real/OtosSensor.cpp` is suspect -- see
`clasi/issues/otos-lever-arm-necessity-and-library-port.md`'s own argument
that it used the identical write path/scaling this driver's existing
position-register writes already prove work), and finalizes
`source/hal/lever_arm.h`'s architecture into exactly one end state.

Per `architecture-update.md` Decision 7: **the default disposition, if the
bench cannot be run or is inconclusive, is FOLD (keep host-side
compensation, but no longer standalone) -- never DELETE without a clean,
positive bench confirmation.** Deleting host-side compensation on an
unconfirmed assumption risks a live-hardware regression (the `db11b7c`
phantom-translation signature `lever_arm.h` documents); folding on an
inconclusive result costs only a small amount of code a later sprint can
clean up once the bench is achievable again. The two error costs are not
symmetric -- do not treat an unreachable bench as license to delete.

## Acceptance Criteria

- [ ] **Bench, BEST-EFFORT**: on the stand (wheels off the ground, safe to
      spin -- `.claude/rules/hardware-bench-testing.md`), write
      `REG_OFFSET` with the real mounting offset via the new
      `setOffset()`, read it back via `getOffset()`, then drive a pure
      in-place spin and check for the lever-arm phantom-translation arc
      (the `db11b7c` signature `lever_arm.h` documents). Record the
      verdict (register reads back non-zero and phantom arc absent =
      chip honors it; register reads back zero, or the phantom arc is
      still present = chip does not honor it) explicitly in this ticket's
      completion notes.
- [ ] **Code, BLOCKING, either bench outcome:**
      - If the chip HONORS `REG_OFFSET` (clean positive confirmation):
        delete `source/hal/lever_arm.h` and all host-side lever-arm
        compensation; the offset becomes a one-time device write in
        `OtosOdometer::begin()`; `tick()`/`setPose()` drop their
        `LeverArm::` calls.
      - Otherwise (chip does not honor it, OR the bench could not be run,
        OR the result is inconclusive): FOLD `LeverArm::sensorToCentre()`/
        `centreToSensor()` directly into `OtosOdometer` as private methods
        (its one production consumer) -- `lever_arm.h` is deleted as a
        STANDALONE file either way; the only question is whether its math
        survives (folded) or is removed entirely (deleted with the
        compensation itself).
      - Either way, `source/hal/lever_arm.h` does NOT exist standalone at
        the end of this ticket.
- [ ] `tests/sim/unit/lever_arm_harness.cpp`/`test_lever_arm.py` are
      removed; their coverage is either subsumed by the chip-native path
      (if deleted) or folded into `otos_odometer_harness.cpp`'s own
      assertions (if folded) -- not simply dropped.
- [ ] **Sim, BLOCKING**: full `uv run python -m pytest tests/sim` is
      green.
- [ ] If the bench step cannot be completed this sprint (hardware
      unavailable, robot wedges/latches, or the re-test cannot be
      completed cleanly), record that explicitly, apply the FOLD default
      (per Decision 7 -- do NOT leave the decision unresolved), and file a
      fresh `clasi/issues/` follow-on carrying the `REG_OFFSET` re-test
      forward with fresh evidence for a future sprint, rather than
      blocking sprint close.

## Implementation Plan

**Approach**:
1. Confirm ticket 003 has landed (`setOffset()`/`getOffset()` exist).
2. Attempt the bench re-test (see Acceptance Criteria) -- best-effort.
3. Based on the bench outcome (or its absence), apply Decision 7's
   disposition rule: DELETE only on a clean positive confirmation, FOLD
   otherwise (the conservative default).
4. Update `tests/sim/unit/` accordingly (remove `lever_arm_harness.cpp`/
   `test_lever_arm.py`; fold their assertions into
   `otos_odometer_harness.cpp` if the FOLD path is taken).
5. Update `otos_odometer.h`'s file header (it currently states the
   register is "deliberately NEVER written" and that lever-arm compensation
   is host-side for that reason -- both statements need updating to
   reflect this ticket's actual outcome, with fresh evidence, not a
   silent carry-over of the superseded `source_old` note).

**Files to modify/create**: `source/hal/lever_arm.h` (deleted),
`source/hal/otos/otos_odometer.h`, `source/hal/otos/otos_odometer.cpp`,
`tests/sim/unit/lever_arm_harness.cpp` (deleted),
`tests/sim/unit/test_lever_arm.py` (deleted),
`tests/sim/unit/otos_odometer_harness.cpp` (extended if folding).

**Testing plan**:
- **Existing tests to run**: full `uv run python -m pytest tests/sim`.
- **New tests to write**: if folding, the former lever-arm assertions
  re-homed inside `otos_odometer_harness.cpp`'s own coverage.
- **Verification command**: `uv run python -m pytest tests/sim`.

**Documentation updates**: `otos_odometer.h`'s file header (see above);
this ticket's completion notes must state the final disposition and bench
evidence (or its absence) plainly.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim` (full
  suite).
- **New tests to write**: folded lever-arm assertions (if the FOLD path is
  taken) inside `otos_odometer_harness.cpp`.
- **Verification command**: `uv run python -m pytest tests/sim`.
