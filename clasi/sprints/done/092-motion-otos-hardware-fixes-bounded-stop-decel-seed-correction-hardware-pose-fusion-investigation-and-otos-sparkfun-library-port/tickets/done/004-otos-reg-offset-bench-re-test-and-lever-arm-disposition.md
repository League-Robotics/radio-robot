---
id: '004'
title: OTOS REG_OFFSET bench re-test and lever-arm disposition
status: done
use-cases:
- SUC-004
depends-on:
- '003'
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

- [~] **Bench, BEST-EFFORT**: on the stand (wheels off the ground, safe to
      spin -- `.claude/rules/hardware-bench-testing.md`), write
      `REG_OFFSET` with the real mounting offset via the new
      `setOffset()`, read it back via `getOffset()`, then drive a pure
      in-place spin and check for the lever-arm phantom-translation arc
      (the `db11b7c` signature `lever_arm.h` documents). Record the
      verdict (register reads back non-zero and phantom arc absent =
      chip honors it; register reads back zero, or the phantom arc is
      still present = chip does not honor it) explicitly in this ticket's
      completion notes.
      **DESCOPED to a follow-on issue this sprint** -- the bench was
      physically unreachable this session (see completion notes and the
      last acceptance item below). Best-effort, not blocking.
- [x] **Code, BLOCKING, either bench outcome:**
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
      **FOLD path taken** (bench unreachable -- Decision 7 default). See
      completion notes.
- [x] `tests/sim/unit/lever_arm_harness.cpp`/`test_lever_arm.py` are
      removed; their coverage is either subsumed by the chip-native path
      (if deleted) or folded into `otos_odometer_harness.cpp`'s own
      assertions (if folded) -- not simply dropped.
- [x] **Sim, BLOCKING**: full `uv run python -m pytest tests/sim` is
      green.
- [x] If the bench step cannot be completed this sprint (hardware
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

## Completion Notes

**Bench: could not be run this session (best-effort, descoped).** The
robot's serial port (`/dev/tty.usbmodem2121102`) was held open by an
unrelated local process (a VS Code extension-host — the same blocker
ticket 092-002 hit independently the same session, see
`clasi/issues/poseestimator-fused-pose-fix-pending-otos-connected-bench-confirmation.md`),
and the radio relay dongle was unplugged. Per the team-lead's own
pre-dispatch check, neither path was reachable, so no flash/drive/serial
attempt was made this ticket (per instruction, to avoid interfering with a
possible concurrent session on that port). **No bench evidence either way
exists for whether this chip honors `REG_OFFSET`.**

**Disposition: FOLD (Decision 7's default for an unreachable/inconclusive
bench).** `source/hal/lever_arm.h` is deleted as a standalone file.
`LeverArm::sensorToCentre()`/`centreToSensor()` are folded, verbatim
(same formulas, same call sites, same same-instant-heading contract), into
`Hal::OtosOdometer` as new **private static methods**
`sensorToCentre()`/`centreToSensor()` (`source/hal/otos/otos_odometer.h`/
`.cpp`). `tick()`'s and `setPose()`'s calls were repointed from
`LeverArm::sensorToCentre(...)`/`LeverArm::centreToSensor(...)` to the
unqualified (in-class) `sensorToCentre(...)`/`centreToSensor(...)` — no
other change to either method. This is a pure relocation: host-side
compensation still runs on every `tick()`/`setPose()` call, identically to
before. `setOffset()`/`getOffset()` (ticket 003) remain available, tested
primitives not called from `begin()` — the FOLD path does not switch to
chip-native compensation.

`source/config/boot_config.h`'s `OtosBootConfig` doc comment and
`otos_odometer.h`'s file header (both 092-003 paragraphs referencing the
then-still-standalone `lever_arm.h` and "ticket 004's job, not this one's")
are updated with a fresh "092-004 update" paragraph stating the actual
outcome plainly: bench unreachable, FOLD applied per Decision 7, chip's
honoring of `REG_OFFSET` remains UNCONFIRMED, carried forward by a fresh
issue rather than left unresolved.

**Tests migrated, not dropped.** `tests/sim/unit/lever_arm_harness.cpp` and
`test_lever_arm.py` are deleted. Their four scenarios (zero-offset
identity, non-degenerate round-trip, round-trip across a spread of
headings/offsets, and the `db11b7c` lagged-heading regression guard) are
ported into `tests/sim/unit/otos_odometer_harness.cpp` as
`scenarioLeverArmZeroOffsetIsIdentity()` /
`scenarioLeverArmRoundTripNonDegenerate()` /
`scenarioLeverArmRoundTripAcrossHeadings()` /
`scenarioLeverArmLaggedHeadingLeavesResidual()`, checked against a local
`testSensorToCentre()`/`testCentreToSensor()` oracle (an independent
re-implementation of the exact same two formulas) — because the folded
methods are now `private`, and the harness compiles as a separate
translation unit from `otos_odometer.cpp` (it links the real
`otos_odometer.cpp` object, it does not `#include` it), so they are no
longer callable directly from outside the class. This mirrors this same
harness file's own pre-existing convention of duplicating `kPosMmPerLsb`/
`kHdgRadPerLsb` as an independent test oracle. The pre-existing
`scenarioTickLeverArmOnlyTransform()` / `scenarioTickMountingYawRotationOnlyTransform()`
scenarios (unchanged in intent, only repointed from `LeverArm::` to
`testSensorToCentre()`) independently prove the real, private, folded
method is correctly wired into `tick()`'s end-to-end path. `test_otos_odometer.py`'s
docstring updated to drop the stale `lever_arm.h` mention and note the
092-004 fold.

**Sim: green.** `uv run python -m pytest tests/sim` — 310 passed, 2
xfailed (baseline was 311 passed/2 xfailed; the one test lost is
`test_lever_arm.py`'s single collected test, deleted along with the file
it tested — no coverage was dropped, it moved into
`otos_odometer_harness.cpp`, one C++ binary run per `test_otos_odometer.py`
as before). The harness itself (14 scenarios, up from 10) was also
compiled and run standalone (`c++ -std=c++20 -Wall -Wextra -DHOST_BUILD`)
with zero warnings and all scenarios passing, before running it through
pytest.

**ARM build: clean link.** `just build` — MICROBIT.hex built successfully
(FLASH 90.96%, RAM 98.33% used, unchanged from before this ticket's
change), `libfirmware_host` (HOST_BUILD sim lib) also built clean. No new
warnings from the fold.

**Follow-on issue filed**: `clasi/issues/otos-reg-offset-bench-retest-deferred.md`
— carries the `REG_OFFSET` bench re-test forward (write/readback +
phantom-arc spin test), references this ticket, ticket 003's `setOffset()`/
`getOffset()` primitives, and records that FOLD was chosen as the safe
default (Decision 7) because the bench was unreachable this sprint, not
because the chip was shown not to honor the register.

**Deviations from the plan**: none of substance. The plan's "Documentation
updates" step (update `otos_odometer.h`'s file header) also touched
`boot_config.h`'s `OtosBootConfig` doc comment, `tick()`'s and the
092-003-additions doc comments, and `otos_odometer.cpp`'s two call-site
comments — all one coherent sweep of every remaining `lever_arm.h`/
`LeverArm::` reference in `source/`, not scope creep (a stale doc pointer
to a deleted file is worse than no pointer at all).
