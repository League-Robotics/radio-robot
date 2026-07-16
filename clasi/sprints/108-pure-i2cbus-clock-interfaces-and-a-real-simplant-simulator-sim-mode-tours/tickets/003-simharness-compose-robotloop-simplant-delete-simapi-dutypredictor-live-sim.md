---
id: "003"
title: "SimHarness: compose RobotLoop + SimPlant; delete SimApi/DutyPredictor/live_sim"
status: open
use-cases: ["SUC-041"]
depends-on: ["002"]
github-issue: ""
issue: "plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md"
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# SimHarness: compose RobotLoop + SimPlant; delete SimApi/DutyPredictor/live_sim

## Description

Stage 2 (part b) of the master plan. Build the composition root that wires
the REAL `App::RobotLoop` firmware graph to `SimPlant` (ticket 002), and
delete the superseded harnesses.

Create `tests/_infra/sim/sim_harness.h`: `class SimHarness` (or free
functions, whichever is more consistent with the existing
`tests/sim/support/fake_transport.h` style — match that file's own
composition idiom) that:

- Constructs the real `App::RobotLoop` graph (the same construction shape
  `main.cpp` uses, minus the ARM-only pieces) with a `SimPlant` in the
  `Devices::I2CBus&` slot instead of `MicroBitI2CBus`.
- `boot()`: run whatever one-time boot sequence the real loop needs before
  its first cycle.
- `step(n)`: for `n` cycles, `plant.tick(dt)` then one `robotLoop.cycle()`
  (or equivalent) — the plant is stepped BEFORE the loop reads it each
  cycle, so a cycle's I2C reads see this cycle's physics, not last cycle's.
- Command injection via the existing `serialLink`/`armor*Command` pattern
  (reuse `tests/sim/support/fake_transport.h`, `wire_test_codec.*`
  unmodified) — do not reinvent wire injection.
- Telemetry draining and a "true pose" accessor (reads `SimPlant`'s owned
  `OtosPlant`/`WheelPlant` ground truth directly, bypassing any sensor
  noise) for test assertions.

Delete the superseded harnesses:
- `tests/sim/support/sim_api.{h,cpp}` (the `SimApi` class + its
  `DutyPredictor` — the desync-prone predictor this sprint replaces).
- `tests/_infra/sim/live_sim.h`, if present. **Note**: as of sprint
  planning, `tests/_infra/sim/` does not exist in the tree at all (deleted
  wholesale by commit `72d8be7e`) — if `live_sim.h` genuinely does not
  exist by the time this ticket runs, there is nothing to delete; do not
  treat this as a blocker, just confirm via `find tests/_infra -iname
  live_sim.h` and note the result in the PR/commit.
- The `Responder` seam — already removed from `i2c_bus.h` by ticket 001;
  confirm no residue remains anywhere else (grep `Responder` across
  `source/` and `tests/`).

## Acceptance Criteria

- [ ] `tests/_infra/sim/sim_harness.h` exists, constructs the real
      `App::RobotLoop` with a `SimPlant` in the bus slot, and exposes
      `boot()`, `step(n)`, command injection, telemetry drain, and a true-
      pose accessor.
- [ ] `tests/sim/support/sim_api.{h,cpp}` (and `DutyPredictor`) are
      deleted.
- [ ] `grep -rn "DutyPredictor\|SimApi\b" tests/` returns nothing except
      historical references inside issue/architecture markdown.
- [ ] `grep -rn "Responder" source/ tests/` returns nothing.
- [ ] `tests/_infra/sim/live_sim.h` confirmed absent or deleted.

## Implementation Plan

**Approach**: `SimHarness` is a thin composition root — it must not
contain any simulation logic (that lives in `SimPlant`) or any firmware
dispatch logic (that lives in the real `App::RobotLoop`, unmodified). Model
it directly on how `main.cpp` constructs the graph, substituting only the
bus.

**Files to create**:
- `tests/_infra/sim/sim_harness.h`

**Files to delete**:
- `tests/sim/support/sim_api.h`
- `tests/sim/support/sim_api.cpp`
- `tests/_infra/sim/live_sim.h` (if present)

**Testing plan**:
- No pytest wiring yet (ticket 005 builds the CMake project + ctypes ABI
  that makes this buildable/runnable from Python) — this ticket's own
  verification is a standalone host-compiled smoke driver: boot the
  harness, inject a simple twist via `fake_transport`, step a few cycles,
  confirm telemetry drains and true pose moves in the expected direction.
  Keep this driver; ticket 004 builds on it for the straight-twist check.
- Confirm `tests/sim/support/fake_transport.h`/`wire_test_codec.*` are
  used unmodified (diff against their pre-ticket state).

**Documentation updates**: `sim_harness.h`'s own file-header doc comment
describing the "tick plant, then run one loop cycle" ordering and why it
matters (a cycle's reads must see this cycle's physics).
