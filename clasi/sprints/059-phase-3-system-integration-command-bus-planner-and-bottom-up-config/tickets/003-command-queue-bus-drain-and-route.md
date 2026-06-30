---
id: '003'
title: Command-queue bus drain and route
status: open
use-cases:
- SUC-003
depends-on:
- 059-001
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Command-queue bus drain and route

## Description

Implement the command-queue bus drain+route layer: a bounded function that takes a
`msg::CommandBatch` returned by a subsystem `tick()` and routes each `OutCommand`
to the appropriate subsystem `apply()` call or enqueues it for the
`CommandProcessor` verb router. This is the mechanism that connects the Planner's
returned `DrivetrainCommand{twist}` to `Drive2::apply()` each tick.

The bus drain is also the enforcement point for:
- **Bounded cascade**: max 8 routing iterations per tick (prevents infinite loops).
- **Safety priority**: `OutCommand`s with `priority=true` are enqueued via
  `CommandQueue::push_front` rather than appended.

This ticket is additive: the function exists but is not yet called from
`loopTickOnce` (that happens in ticket 005). It can be called from the planner
test shim (ticket 002) to verify routing correctness in isolation.

## Acceptance Criteria

- [ ] `source/robot/BusDrain.h` declares:
  ```cpp
  // Route every OutCommand in batch to its target subsystem.
  // Bounded: stops after kBusDrainMaxIters commands per call.
  // priority=true OutCommands are routed via queue.push_front.
  // Returns number of commands routed (for telemetry/EVT).
  uint8_t drainCommandBatch(
      const msg::CommandBatch& batch,
      subsystems::Drive2&    drive2,
      MotionController2&     planner,
      CommandQueue&          queue,
      CommandProcessor&      cmd);
  ```
- [ ] `source/robot/BusDrain.cpp` implements `drainCommandBatch()`:
  - Iterates over `batch.cmds_[0..batch.cmds_count-1]`.
  - For each `OutCommand`: dispatches by `verb_id` to the correct subsystem `apply()`.
    - Verb ID for `DrivetrainCommand::twist` → `drive2.apply(DrivetrainCommand{twist})`.
    - Verb ID for `PlannerCommand` → `planner.apply(PlannerCommand)`.
    - Any verb ID recognized by `CommandProcessor` → route via `queue.push_back` (or
      `push_front` if `priority==true`).
  - Enforces `kBusDrainMaxIters = 8`; if exceeded, returns immediately (caller is
    responsible for emitting EVT if needed).
- [ ] `kBusDrainMaxIters` is a named compile-time constant (not a magic number).
- [ ] A `priority=true` `OutCommand` is routed via `queue.push_front()`.
- [ ] `push_front` failure (queue full) is handled: the count is returned and the
  caller can emit `EVT bus_overflow` if needed (overflow of safety command is
  treated as EVT not assertion).
- [ ] Unit test in `tests/simulation/unit/test_059_bus_drain.py`:
  - `test_twist_command_routed_to_drive2` — build a `CommandBatch` with one
    `DrivetrainCommand{twist}`, call `drainCommandBatch`, verify `drive2.state()` reflects
    the applied command on the next `tickUpdate/tickAction`.
  - `test_priority_command_uses_push_front` — build a `CommandBatch` with one
    `priority=true` `OutCommand`, call `drainCommandBatch`, verify the command is at
    the head of the queue.
  - `test_bounded_cascade_stops_at_max_iters` — build a `CommandBatch` with 10
    commands (> 8); verify `drainCommandBatch` returns 8 and does not process beyond.
- [ ] `python build.py --clean` zero errors.
- [ ] `uv run python -m pytest -x --tb=short -q` at 2380/2 plus new tests.

## Implementation Plan

### Approach

The verb-ID dispatch table inside `drainCommandBatch` maps `OutCommand::verb_id`
values to subsystem `apply()` calls. The verb IDs are compile-time constants defined
in `source/messages/common.h` or a new `source/messages/verb_ids.h` header.

`OutCommand` is already defined in `source/messages/common.h`:
```cpp
struct OutCommand {
    uint32_t verb_id;
    float args[4];
    uint8_t argc;
    bool priority;
};
```

The routing table needs to reconstruct the typed message from `verb_id` and `args`.
For `DrivetrainCommand{twist}`: pack `args[0]` → `vx_mmps`, `args[1]` → `vy_mmps`,
`args[2]` → `omega_rads`. This packing convention must match what `MotionController2::tick()`
uses when it packs the `OutCommand`.

Define verb ID constants (e.g., `kVerbDrivetrainTwist = 1`, `kVerbPlannerCommand = 2`)
in a shared header. `MotionController2::tick()` uses the same constants when packing.

The `CommandProcessor` verb routing for ASCII verbs (VW, TURN, etc.) is handled by
`cmd.dequeueOne(queue)` in the existing loop — the bus drain enqueues into `queue`
and the existing `dequeueOne` path dispatches it. This avoids duplicating the verb
dispatch table.

### Files to Create

- `source/robot/BusDrain.h` — declaration
- `source/robot/BusDrain.cpp` — implementation
- `source/messages/verb_ids.h` — `kVerbDrivetrainTwist`, `kVerbPlannerCommand`, etc.
- `tests/simulation/unit/test_059_bus_drain.py` — unit tests

### Files to Modify

- `CMakeLists.txt` — add `BusDrain.cpp` to firmware and host-sim source lists

### Testing Plan

```bash
python build.py --clean
uv run python -m pytest tests/simulation/unit/test_059_bus_drain.py -v
uv run python -m pytest -x --tb=short -q
```

### Documentation Updates

Add a comment block at the top of `BusDrain.h` explaining the verb-ID dispatch
convention and the bounded cascade policy.
