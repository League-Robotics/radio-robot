---
id: '003'
title: "Retire DeviceBus/handles.h \u2014 leaves become the loop's direct dependency"
status: done
use-cases:
- SUC-003
depends-on: []
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Retire DeviceBus/handles.h — leaves become the loop's direct dependency

## Description

Delete `Devices::DeviceBus` (`source/devices/device_bus.{h,cpp}`) and
`source/devices/handles.h`. Per architecture-update.md Decision 1: the
archived plan's main-loop sketch constructs `Devices::I2CBus`/
`Devices::NezhaMotor`/`Devices::Otos`/etc. directly ("no fiber, no
handles, no staging layer"), which is incompatible with `DeviceBus`'s
already-narrowed (sprint 102) but still handle-mediated
`runPreamble()`/`runCycleOnce()` surface — `runCycleOnce()` owns its own
internal settle-sleep, which cannot host the archived plan's `runAndWait`
interleaving (`Comms::pump()`/`Telemetry::emit()` borrowing the settle
window) without restructuring `DeviceBus` into something that is no longer
really `DeviceBus`. This ticket executes that decision: delete the class,
delete its dedicated test harness, and confirm nothing else in the tree
references it.

This is a pure deletion ticket with no new logic — it should land before
tickets 004-007 so there is never a commit with two competing
device-access patterns (`DeviceBus`'s handles AND the new `app/` modules'
bare-leaf access) coexisting.

## Acceptance Criteria

- [x] `grep -rn "DeviceBus\|#include \"devices/handles.h\"\|#include \"devices/device_bus"
      source/ tests/sim/unit/` returns nothing outside the files being
      deleted, confirmed BEFORE deleting (a pre-deletion grep, recorded in
      completion notes, to catch any caller this ticket's description
      missed).
- [x] `source/devices/device_bus.h`, `source/devices/device_bus.cpp`,
      `source/devices/handles.h` deleted.
- [x] `tests/sim/unit/device_bus_cycle_harness.cpp` and
      `tests/sim/unit/test_device_bus_cycle.py` deleted (no port — see
      Description; `DeviceBus`'s responsibilities are redistributed across
      tickets 004-007, not ported 1:1).
- [x] `CMakeLists.txt` updated if it lists either deleted `.cpp`/harness
      file by name.
- [x] A post-deletion `grep -rn "DeviceBus\|handles\.h"` under `source/`
      returns nothing.
- [x] `devices_*` leaf unit tests (which exercise the leaves directly, not
      through `DeviceBus`) stay green — confirming the leaves themselves
      are untouched by this deletion.
- [x] `tests/bench/{rig_dev,rig_soak,device_bus_bringup}.py` and
      `tests/unit/test_device_bus_bringup_bench.py` are explicitly left
      untouched this ticket (their rewrite is sprint 104 scope) — note
      this in completion notes so it reads as a deliberate choice, not an
      oversight.

## Completion Notes

**Pre-deletion grep** (`grep -rn "DeviceBus\|#include \"devices/handles.h\"\|#include
\"devices/device_bus" source/ tests/sim/unit/`) — every hit outside the
five files this ticket deletes was a **comment**, not a code reference:
`source/devices/{motor_armor.h, nezha_motor.{h,cpp}, interpolation.h,
color_sensor.h, i2c_bus.h, line_sensor.h, otos.h, clock.h}` and
`tests/sim/unit/{devices_clock_harness.cpp, devices_motor_harness.cpp}`
only *mentioned* `DeviceBus`/`handles.h` in design-rationale prose. No
actual `#include`, instantiation, or symbol reference existed outside
`device_bus.{h,cpp}`, `handles.h`, `device_bus_cycle_harness.cpp`, and
`test_device_bus_cycle.py` — confirming the archived plan's "no handles"
framing was safe to act on for this tree state.

**Deleted** (`git rm`): `source/devices/device_bus.h` (293 lines),
`source/devices/device_bus.cpp` (329 lines), `source/devices/handles.h`
(237 lines), `tests/sim/unit/device_bus_cycle_harness.cpp` (636 lines),
`tests/sim/unit/test_device_bus_cycle.py` (111 lines). No port — per the
Description, `DeviceBus`'s responsibilities (preamble probe logic, cycle
constants, `neutralizeAllMotors()`) are redistributed across tickets
004-008, not carried into this ticket.

**CMakeLists.txt**: no build-graph reference existed (`source/` firmware
sources are discovered via `RECURSIVE_FIND_FILE` glob, not listed by
name, so no target/list edit was needed). One stale illustrative comment
(line 263, `#include "devices/device_bus.h"` used as an example of a
path-qualified include) was updated to reference a file that still
exists (`devices/otos.h`) instead.

**Post-deletion cleanup beyond the literal delete list**: the acceptance
criterion's post-deletion grep (`grep -rn "DeviceBus\|handles\.h"` under
`source/`) checks comment text too, not just code references. Rewrote
the ~20 comment mentions of `DeviceBus`/`handles.h` in
`source/devices/{motor_armor.h, nezha_motor.{h,cpp}, interpolation.h,
color_sensor.h, i2c_bus.h, line_sensor.h, otos.h, clock.h}` to describe
the new architecture ("the loop constructs/drives this leaf directly")
instead of the retired class, preserving each comment's original
technical content (freshness-gate rationale, time-seam ownership,
rename history, etc.) — no design rationale was dropped, only the
now-wrong class name. `tests/sim/unit/{devices_clock_harness.cpp,
devices_motor_harness.cpp}` had 3 similar comment mentions (outside the
`source/` grep's scope, so not required by the acceptance criterion) —
updated anyway for consistency, same technique.
`grep -rn "DeviceBus\|handles\.h" source/` now returns nothing.

**Deliberately untouched** (sprint 104 scope, per this ticket's
Description): `tests/bench/rig_dev.py`, `tests/bench/rig_soak.py`,
`tests/bench/device_bus_bringup.py`, `tests/unit/test_device_bus_bringup_bench.py`
still reference `DeviceBus`/`device_bus`/`handles.h` by design — a
repo-wide grep (excluding `clasi/` archives and worktrees) confirms
these four are the *only* remaining hits anywhere in the tree.

**Test results**: `uv run python -m pytest tests/sim/unit/` — 333 passed,
0 failed (down from the pre-ticket collection count by exactly the 2
retired `test_device_bus_cycle.py` scenario(s); every `devices_*` leaf
harness — clock, motor, sensors, measurement ring — stayed green,
confirming the leaves themselves are untouched by this deletion).

**Firmware build**: `just build-clean` — clean CMake configure + full ARM
build succeeded, `MICROBIT.hex` produced at v0.20260714.8. FLASH
27.84% used, RAM 98.33% used (expected/by-design for this project, not a
regression). Confirms the stub `main()` never depended on
`DeviceBus`/`handles.h` and the firmware build is unaffected by this
deletion, as anticipated.

**No surprises**: the ticket's premise held exactly — every non-comment
reference to `DeviceBus`/`handles.h` lived inside the five deleted files,
and the leaf classes (`NezhaMotor`, `Otos`, `ColorSensorLeaf`,
`LineSensorLeaf`, `I2CBus`, `MotorArmor`, `MotorVelocityPid`,
`MeasurementRing`) were already independently constructible — this was a
pure deletion plus comment cleanup, no logic changes.

## Implementation Plan

**Approach**: Grep first, delete second, grep again. This ticket is
mechanical (no new code) but the FIRST grep is the load-bearing step — it
is what proves the archived plan's "no handles" framing is actually safe
to act on for THIS specific tree state, not just plausible from reading
the plan.

**Files to create/modify** (all deletions):
- `source/devices/device_bus.h`
- `source/devices/device_bus.cpp`
- `source/devices/handles.h`
- `tests/sim/unit/device_bus_cycle_harness.cpp`
- `tests/sim/unit/test_device_bus_cycle.py`
- `CMakeLists.txt` (remove references to the above, if present)

**Testing plan**:
- Existing tests to run: the full `devices_*` unit suite (unaffected by
  this deletion — they test leaves directly).
- New tests to write: none (pure deletion).
- Verification command: a clean CMake configure + build of the host-buildable
  test targets (`HOST_BUILD`) to confirm nothing fails to link after the
  deletion; `uv run python -m pytest tests/sim/unit/` minus the two removed
  test files.

**Documentation updates**: none beyond this ticket's own completion notes
(the pre/post-deletion grep results).
