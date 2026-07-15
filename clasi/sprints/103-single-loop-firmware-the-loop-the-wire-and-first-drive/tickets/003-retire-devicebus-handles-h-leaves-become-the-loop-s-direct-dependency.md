---
id: '003'
title: "Retire DeviceBus/handles.h — leaves become the loop's direct dependency"
status: open
use-cases: [SUC-003]
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

- [ ] `grep -rn "DeviceBus\|#include \"devices/handles.h\"\|#include \"devices/device_bus"
      source/ tests/sim/unit/` returns nothing outside the files being
      deleted, confirmed BEFORE deleting (a pre-deletion grep, recorded in
      completion notes, to catch any caller this ticket's description
      missed).
- [ ] `source/devices/device_bus.h`, `source/devices/device_bus.cpp`,
      `source/devices/handles.h` deleted.
- [ ] `tests/sim/unit/device_bus_cycle_harness.cpp` and
      `tests/sim/unit/test_device_bus_cycle.py` deleted (no port — see
      Description; `DeviceBus`'s responsibilities are redistributed across
      tickets 004-007, not ported 1:1).
- [ ] `CMakeLists.txt` updated if it lists either deleted `.cpp`/harness
      file by name.
- [ ] A post-deletion `grep -rn "DeviceBus\|handles\.h"` under `source/`
      returns nothing.
- [ ] `devices_*` leaf unit tests (which exercise the leaves directly, not
      through `DeviceBus`) stay green — confirming the leaves themselves
      are untouched by this deletion.
- [ ] `tests/bench/{rig_dev,rig_soak,device_bus_bringup}.py` and
      `tests/unit/test_device_bus_bringup_bench.py` are explicitly left
      untouched this ticket (their rewrite is sprint 104 scope) — note
      this in completion notes so it reads as a deliberate choice, not an
      oversight.

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
