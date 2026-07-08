---
status: in-progress
sprint: 091
tickets:
- 091-001
---

# Rename `MainLoop::emergencyNeutralize()` to `estop()`

## Context

Stakeholder FIXME noted in-progress on 2026-07-07 (a `// FIXME rename to "estop"`
comment on the `emergencyNeutralize()` declaration in
[source/runtime/main_loop.h](source/runtime/main_loop.h)). The loop's
immediate motor-neutralize entry point (called on watchdog fire / safety stop)
should be named `estop()` — shorter, the conventional term for an emergency stop.

Captured as a standalone issue so the intent survives independently (the raw
working-tree edit was stashed to keep sprint 088's tree clean). **Out of scope for
sprint 088** (that sprint is testing-focused); schedule separately.

## Scope

- Rename `Rt::MainLoop::emergencyNeutralize()` → `estop()` (declaration
  [main_loop.h](source/runtime/main_loop.h) + definition/call sites in
  `source/runtime/main_loop.cpp` and anywhere else it is invoked).
- Update comments that reference "emergencyNeutralize" by name.
- Pure rename; no behavior change. Lower-camelCase, no units — conforms to the
  naming rules already.

## Acceptance

- No `emergencyNeutralize` identifier remains; `estop()` builds and all tests pass;
  behavior unchanged.
