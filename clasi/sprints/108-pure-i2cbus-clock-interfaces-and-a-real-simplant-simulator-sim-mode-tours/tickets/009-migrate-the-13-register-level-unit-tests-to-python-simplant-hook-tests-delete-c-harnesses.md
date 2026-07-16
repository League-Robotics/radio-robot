---
id: "009"
title: "Migrate the 13 register-level unit tests to Python SimPlant hook tests; delete C++ harnesses"
status: open
use-cases: ["SUC-040"]
depends-on: ["008"]
github-issue: ""
issue: "plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md"
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

- [ ] Every register-scripting `tests/sim/unit/*_harness.cpp` and
      `tests/sim/plant/plant_harness.cpp` identified by the grep above has
      an equivalent Python hook test, and is deleted.
- [ ] Each original scenario (per-file, per-test-case) has a traceable
      Python equivalent — no silent coverage drop (spot-check by diffing
      old vs. new test-case counts/names, not just "the file compiles").
- [ ] `uv run python -m pytest tests/sim` is fully green.
- [ ] `grep -rn "scriptWrite\|scriptRead" tests/` returns nothing.
- [ ] `tests/sim/plant/{wheel_plant,otos_plant}.{h,cpp}` themselves are
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
