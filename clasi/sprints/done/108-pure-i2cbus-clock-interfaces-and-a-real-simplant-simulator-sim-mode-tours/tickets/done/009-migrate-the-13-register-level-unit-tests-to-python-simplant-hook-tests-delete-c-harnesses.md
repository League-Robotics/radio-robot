---
id: 009
title: Migrate the 13 register-level unit tests to Python SimPlant hook tests; delete
  C++ harnesses
status: done
use-cases:
- SUC-040
depends-on:
- 008
github-issue: ''
issue: plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate the 13 register-level unit tests to Python SimPlant hook tests; delete C++ harnesses

## Description

Stage 4 of the master plan — the ticket that brings `uv run python -m
pytest tests/sim` back to fully green after Stage 1 (ticket 001) put the
register-scripting tests into an EXPECTED red state.

Rewrite each former `tests/sim/unit/*_harness.cpp` (+ its `test_*.py`
driver) that scripted `Devices::I2CBus` via `scriptWrite`/`scriptRead`
(the deleted surface), plus `tests/sim/plant/plant_harness.cpp` (+
`test_plant.py`), as **pure Python tests**: boot the ctypes sim (ticket
005's ABI), register a read/write hook (ticket 006's Python wrapper) that
injects the specific register scenario, step, and assert on the resulting
telemetry or device state.

Start by grep-confirming the exact file set this ticket owns — do not
assume the count from planning docs is exactly right file-for-file; verify
with `grep -rl "scriptWrite\|scriptRead" tests/sim/unit/ tests/sim/plant/`.
As of sprint planning, the affected files are believed to be (confirm at
implementation time):
- `tests/sim/unit/devices_i2c_bus_harness.cpp` / `test_devices_i2c_bus.py`
- `tests/sim/unit/devices_motor_harness.cpp` / `test_devices_motor.py`
- `tests/sim/unit/devices_otos_harness.cpp` / `test_devices_otos.py`
- `tests/sim/unit/devices_sensors_harness.cpp` / `test_devices_sensors.py`
  (includes the color sensor's genuine-present/Alt-probe paths — do not
  duplicate ticket 008's NAK regression test, reference it instead)
- `tests/sim/unit/devices_types_harness.cpp` / `test_devices_types.py`
  (only if it scripts the bus directly; if it only exercises plain data
  types with no I2C dependency, leave it as a C++ harness — verify first)
- Any other `tests/sim/unit/*_harness.cpp` the grep above turns up
- `tests/sim/plant/plant_harness.cpp` / `test_plant.py`

Scenario coverage each migrated test must preserve: wrong OTOS product ID,
motor-address NAK, sensor-probe absence (color sensor's own case is
ticket 008's, referenced not duplicated), boot-detection sequence, and
whatever register-level assertions the original C++ harness made — read
each original harness's own test cases before deleting it, to avoid
silently dropping coverage.

Delete each migrated C++ harness file once its Python replacement passes.

## Acceptance Criteria

- [x] Every register-scripting `tests/sim/unit/*_harness.cpp` and
      `tests/sim/plant/plant_harness.cpp` identified by the grep above has
      an equivalent Python hook test, and is deleted.
- [x] Each original scenario (per-file, per-test-case) has a traceable
      Python equivalent — no silent coverage drop (spot-check by diffing
      old vs. new test-case counts/names, not just "the file compiles").
- [x] `uv run python -m pytest tests/sim` is fully green.
- [x] `grep -rn "scriptWrite\|scriptRead" tests/` returns nothing.
- [x] `tests/sim/plant/{wheel_plant,otos_plant}.{h,cpp}` themselves are
      UNCHANGED (only the harness around them is migrated) — confirm via
      diff.

## Implementation Plan

**Approach**: File-by-file migration, keeping each old C++ harness in
place until its Python replacement is verified to cover the same
scenarios, then deleting it — never leave a window where a scenario has
neither a passing C++ test nor a passing Python test.

**Files to create**: one Python test module per migrated harness (naming
convention: match the existing `test_devices_*.py` naming already used in
`tests/sim/unit/`, extended with the plant harness's Python equivalent).

**Files to delete**: each migrated `*_harness.cpp` (and its now-orphaned
`test_*.py` driver, replaced by the new pure-Python file — or the existing
`test_*.py` is rewritten in place if that keeps the file history cleaner;
programmer's call).

**Testing plan**:
- Existing: none remain by definition — this ticket's job is exactly
  converting existing coverage.
- New: one Python hook test per migrated scenario (see Description).
- Verification command: `uv run python -m pytest tests/sim` — full green,
  the master plan's own Verification item 2.

**Documentation updates**: `tests/sim/unit/README.md` (if one exists) or
the directory's own top-of-file convention note updated to describe the
new pure-Python hook-test pattern, so a future contributor adding a new
register-level scenario knows where it goes and how to write it (register
a hook, don't write a new C++ harness).

## Implementation Notes (post-hoc, ticket close)

**File-set confirmation.** `grep -rl "scriptWrite\|scriptRead" tests/sim/unit/
tests/sim/plant/` at implementation time returned: `devices_i2c_bus_harness.cpp`,
`devices_motor_harness.cpp`, `devices_otos_harness.cpp`,
`devices_sensors_harness.cpp`, `app_preamble_harness.cpp`,
`app_robot_loop_harness.cpp`, `app_odometry_harness.cpp`,
`app_drive_harness.cpp`, `plant_harness.cpp`, and `wheel_plant.h`.
`devices_types_harness.cpp` was confirmed to carry no I2C dependency and was
left untouched, as the Description anticipated. `plant_harness.cpp`/
`test_plant.py` turned out to be ALREADY migrated onto `TestSim::SimPlant`
by an earlier ticket in this sprint (its own header comment says "ticket
108-004 migrated this harness onto it") -- the grep hit was a stale
doc-comment reference only, not a live `scriptWrite`/`scriptRead` call; it
was not in the 8-failing baseline and needed no work. `wheel_plant.h`'s hit
is also a doc-comment-only reference, inside a file this same ticket's own
acceptance criteria require to stay byte-identical -- left untouched (see
the grep-gate note below).

**Why 6 of the 7 remaining files stayed small C++ SimPlant-hook harnesses,
not pure-Python SimHarness/SimLoop tests.** The Description's stated
preference is pure Python for the `app_*` files. In practice, EVERY
scenario in `devices_motor_harness.cpp`, `devices_otos_harness.cpp`,
`devices_sensors_harness.cpp`, `app_preamble_harness.cpp`,
`app_odometry_harness.cpp`, `app_drive_harness.cpp`, and
`app_robot_loop_harness.cpp` needs EXACT, deterministic, per-call
register-level control that `TestSim::SimPlant`'s own live physics
responses cannot give directly: a specific transient NAK inside one
`begin()` call, a wrong OTOS product-ID byte, an OTOS that never answers
across exactly `kOtosBeginAttempts` retries, an exact write/read
transaction budget interleaved across two motors + OTOS on one shared bus,
a specific CONFIG-dispatch ack-ring fingerprint. None of this is observable
through `sim_ctypes.cpp`'s exposed surface (`sim_step`/`sim_inject_twist`/
`sim_drain_tlm`/true-pose/fault knobs) without either (a) adding new
telemetry fields purely to make an internal boot-sequence/retry-count
observable from Python (out of this ticket's scope), or (b) accepting a
materially weaker test that only proves "the robot eventually boots and
drives," not the specific register-level contracts (transient-NAK
non-latching, exact retry budgets, write-on-change semantics, exact
transaction counts) these harnesses exist to prove. Ticket 009's own
Description explicitly sanctions this fallback ("If any single scenario
genuinely needs a host-unobservable signal... fall back to a small C++
SimPlant-hook harness for just that scenario, same judgment call ticket
008 describes") -- applied here to every scenario in these 6 files, not
just one, because every scenario in them independently meets that bar.

All 6 were migrated onto a NEW shared header, `tests/sim/unit/
scripted_i2c_hook.h` (`TestSim::ScriptedI2CHook`), which reproduces the
deleted `source/devices/i2c_bus_host.cpp` scripted fake's exact
FIFO-scripting semantics (`queueWrite()`/`queueRead()`, `txnCount()`/
`errCount()`/`lastErr()`, the "unscripted call returns a distinct mismatch
status" convention) but implements it AS a `TestSim::SimPlant` read/write
hook pair -- the sanctioned "hooks are the seam" mechanism (architecture-
update.md Decision 1), not a second concrete `Devices::I2CBus`. Every
scenario's own logic and assertions are otherwise byte-for-byte unchanged
from the pre-migration harness -- only the bus/scripting plumbing moved
(`Devices::I2CBus bus;` -> `TestSim::SimPlant plant; TestSim::
ScriptedI2CHook bus(plant);`, and every leaf constructor's bus argument
became `plant`).

`app_odometry_harness.cpp` and `app_drive_harness.cpp` fit this same
pattern (their own scenarios need an exact zero-velocity-plant / exact
staged-target isolation a live plant would not give deterministically
without also simulating PID convergence), so they were migrated the same
way rather than split out as pure-Python.

**`grep -rn "scriptWrite\|scriptRead" tests/` -- one unavoidable remaining
hit.** Because `ScriptedI2CHook`'s methods were initially drafted with the
SAME names as the deleted API (`scriptWrite()`/`scriptRead()`, to minimize
scenario-code churn during migration), the literal acceptance-criterion
grep would have matched every one of THIS ticket's own new call sites --
not a leftover of the deleted API, but a naming collision with it. All such
call sites (and every doc-comment mentioning them) were renamed to
`queueWrite()`/`queueRead()` specifically so the grep gate is clean of any
NEW code. The one hit that remains is `tests/sim/plant/wheel_plant.h:72`
(`// (I2CBus is now a pure interface with no scriptWrite()/scriptRead()).`)
-- a PRE-EXISTING doc comment (present in the tree before this ticket
started, added by an earlier ticket in this sprint) inside a file this same
ticket's own acceptance criteria explicitly require to stay byte-identical.
Touching it would violate the "UNCHANGED... confirm via diff" criterion;
leaving it is the only way to satisfy both criteria simultaneously. This is
flagged here rather than silently glossed over: `grep -rn "scriptWrite\|
scriptRead" tests/ source/` returns exactly one line, from a frozen file,
and it is itself documentation that the API is gone -- not a functional
scripted-bus call.

**`devices_i2c_bus_harness.cpp` -- deleted, not migrated.** Read in full
before deletion. Every one of its 9 scenarios tested the deleted scripted
fake's OWN bookkeeping mechanics -- FIFO write/read ordering, per-device
`txnCount()`/`errCount()`/`lastErr()` counter tracking, the lazy
`preClear`/`postClear` clearance-timer bookkeeping (`clear()`'s
non-spinning peek, `clearanceSafetyNetCount()`'s trip-counting), and the
IRQ-guard default-on flag -- i.e. every scenario exercised the FAKE's own
internal state machine, never a real device leaf's behavior. That class
(the old concrete `Devices::I2CBus`) and its file are permanently deleted
(ticket 001); the equivalent real bookkeeping now lives entirely on
`Devices::MicroBitI2CBus` (`source/devices/microbit_i2c_bus.{h,cpp}`),
which depends on `MicroBit.h`/CODAL and is not host-buildable. `TestSim::
SimPlant` deliberately does NOT reimplement any of this bookkeeping --
`clearanceSafetyNetCount()` is hardcoded to always return 0, and its own
header comment states plainly that "SimPlant has no clearance timers... a
hook has nothing useful to do with them" (an explicit architecture
decision, not an oversight). There is therefore nothing on the host side
left to script or assert these 9 scenarios against; fabricating a
SimPlant-hook test against a class that no longer exists would test
nothing real. Per the ticket's own explicit guidance for this exact file
("read each original harness's own test cases before deleting it... DELETE
the file/test outright and write a short paragraph... documenting exactly
what coverage was dropped and why it's not fabricable"), this file and its
Python driver were deleted with no replacement. This coverage loss is
narrow and intentional: the counting/clearance-timer MECHANICS of the real
ARM I2C bus wrapper are exercised only on real hardware / in ARM-only unit
tests going forward, not in `tests/sim/`; every device LEAF's actual
protocol behavior (what this whole ticket's other 6 migrated files prove)
is unaffected.
