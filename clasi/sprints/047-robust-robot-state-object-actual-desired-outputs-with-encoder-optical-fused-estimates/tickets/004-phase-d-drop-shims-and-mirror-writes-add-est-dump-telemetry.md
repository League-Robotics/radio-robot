---
id: '004'
title: "Phase D — Drop shims and mirror-writes, add EST dump telemetry"
status: open
use-cases: [SUC-047-001, SUC-047-002, SUC-047-003]
depends-on: ['003']
github-issue: ''
issue: robot-state-object-proposed-structure-for-review.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Phase D — Drop shims and mirror-writes, add EST dump telemetry

## Description

With all consumers migrated in Phase C, this ticket completes the migration
by removing the temporary scaffolding:

1. Drop `source/state/StateShims.h` (delete file; no consumer should reference it).
2. Remove the legacy mirror-writes from `Odometry.cpp` (`predict()` no longer writes
   `s.poseX`; `correctEKF()` no longer writes `s.fusedV` etc.).
3. Remove now-unused `HardwareState`, `MotorCommands`, and `TargetState` struct
   definitions from `source/types/Inputs.h` (or move them to a deprecated header
   if any external tool still needs them — confirm by grep).
4. Implement `dumpEstimates()` in `source/state/EstimateDump.h` (or companion `.cpp`)
   and wire a `DBG EST` command in `DebugCommandable` to emit the three `EST` lines.

This leaves the codebase with the clean three-group state structure, no dead shim
layer, and a working `EST enc/otos/fuse` telemetry dump.

## Files to Modify

- `source/state/StateShims.h` — **delete**. Run a grep first (`grep -r StateShims source/`) to confirm no remaining references.
- `source/control/Odometry.cpp` — remove legacy mirror-write lines in `predict()` and `correctEKF()` (the lines that write `s.poseX = ...`, `s.fusedV = ...`, etc. after writing to `actual.*`).
- `source/types/Inputs.h` — remove `HardwareState`, `MotorCommands`, `TargetState` struct definitions (after confirming by grep that no file still uses them directly). Keep the `#include` guards and `ValueSet`/`defaultInputs()`.
- `source/state/EstimateDump.h` / optional `source/state/EstimateDump.cpp` — implement `dumpEstimates(const ActualState& a, uint32_t now_ms, EstimateDump out[3])`:
  ```cpp
  out[0] = { "enc",  a.encoder.pose, a.encoder.twist, age(a.encoder.stamp, now_ms), a.encoder.stamp.valid };
  out[1] = { "otos", a.optical.pose, a.optical.twist, age(a.optical.stamp, now_ms), a.optical.stamp.valid };
  out[2] = { "fuse", a.fused.pose,   a.fused.twist,   age(a.fused.stamp,   now_ms), a.fused.stamp.valid  };
  ```
  where `age()` is an inline helper: `return stamp.valid ? (now_ms - stamp.lastUpdMs) : UINT32_MAX;`
- `source/app/commands/DebugCommandable.cpp` (and `.h`) — add `DBG EST` subcommand handler that calls `dumpEstimates(robot->state.actual, robot->systemTime(), dump)` and emits:
  ```
  EST enc   x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
  EST otos  x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
  EST fuse  x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
  ```

## Key Implementation Notes

- **Pre-delete grep**: before removing `HardwareState`/`MotorCommands`/`TargetState`, run:
  `grep -rn "HardwareState\|MotorCommands\|TargetState" source/ tests/` — any remaining references must be fixed first. If any exist, fix them before deleting the structs.
- **`hal.tick(now_ms, const MotorCommands&)` virtual method on `Hardware`**: this signature is on the `Hardware` base class and is implemented by `NezhaHAL`, `MecanumHAL`, and `SimHardware`. Update all three overrides to `tick(uint32_t, const OutputState&)`. Update the base class virtual declaration in `source/io/Hardware.h`. Update all call sites in `LoopTickOnce.cpp` and `sim_api.cpp`. This is the concrete breaking-change risk identified in the architecture review.
- **`DBG EST` format**: use `snprintf` into a stack-local `char buf[128]`, then reply via the existing `replyFn` — no heap.
- **`age()` helper**: safe for uint32 wraparound since both `now_ms` and `stamp.lastUpdMs` are `uint32_t` and subtraction is modular; guard `!stamp.valid` with `UINT32_MAX`.

## Acceptance Criteria

- [ ] `source/state/StateShims.h` does not exist; no file `#include`s it.
- [ ] `source/types/Inputs.h` does not define `HardwareState`, `MotorCommands`, or `TargetState`.
- [ ] `Odometry::predict()` and `correctEKF()` contain no legacy mirror-write lines (no writes to `s.poseX`, `s.fusedV`, `s.fusedOmega`, etc.).
- [ ] `dumpEstimates()` is implemented and fills all three `EstimateDump` slots with correct source labels, pose, twist, age, and validity.
- [ ] `DBG EST` command emits three `EST enc/otos/fuse` lines in the documented format.
- [ ] `Hardware::tick(uint32_t, const OutputState&)` is the updated virtual signature; all HAL implementations (NezhaHAL, MecanumHAL, SimHardware) updated accordingly.
- [ ] **Differential build compiles clean** (`python build.py --clean`): zero errors.
- [ ] **Mecanum build compiles clean**: zero errors.
- [ ] **Sim unit suite green**: `uv run --with pytest python -m pytest tests/simulation/ -q` — no Python test edits required.

## Implementation Plan

1. Grep for all remaining `HardwareState`/`MotorCommands`/`TargetState`/`StateShims` references. Fix any stragglers found.
2. Update `Hardware.h` `tick()` virtual signature; update `NezhaHAL`, `MecanumHAL`, `SimHardware` overrides.
3. Update all `hal.tick(now_ms, state.commands)` call sites to `hal.tick(now_ms, state.outputs)`.
4. Remove legacy mirror-writes from `Odometry.cpp`.
5. Delete `StateShims.h`.
6. Remove `HardwareState`, `MotorCommands`, `TargetState` from `Inputs.h`.
7. Implement `dumpEstimates()` in `EstimateDump.h`.
8. Add `DBG EST` handler in `DebugCommandable`.
9. Build both variants; run sim suite; confirm clean.

## Testing Plan

- **Sim suite**: `uv run --with pytest python -m pytest tests/simulation/ -q` — must pass green.
- **Build test**: `python build.py --clean` (differential and mecanum).
- **Manual check**: run `sim_command(h, "DBG EST", ...)` from Python and verify three `EST` lines appear in the output.
- **No new Python tests required**: the fusion-validation test is in ticket 005.

## Documentation Updates

Architecture update sections B (mirror-write removal), C (dump surface), and F note this cleanup. No new docs needed.
