---
id: '002'
title: Subsystem contract scaffold and fluent-builder pattern
status: done
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Subsystem contract scaffold and fluent-builder pattern

## Description

The message-based subsystem architecture (issue `message-based-subsystem-architecture.md`)
defines a structural convention: every subsystem exposes 3 message types
(`Command`, `State`, `Config`) and 4 verbs (`apply`, `tick`, `state`, `configure`)
plus `capabilities()`. The primary call-site form is a fluent builder:
`subsystem.newCommand().setX(v).apply()` — where `newCommand()` returns a ref to a
reused internal `Command` instance, each `setX()` mutates and returns `*this`, and
`.apply()` commits the staged command.

This ticket creates the documentation header (`SubsystemContract.h`) and the
`SensorsState` aggregate type (needed by ticket 003 before `Sensors` can be written).
It does NOT implement a full subsystem — that is tickets 003 and 004.

## Approach

1. Create `source/subsystems/SubsystemContract.h` — a header-only documentation file
   (no compiled symbols). Contents:
   - Comment block describing the 3-message / 4-verb structural convention, verbatim
     from `message-based-subsystem-architecture.md` §"The subsystem contract".
   - The documented call-site fluent-builder form with C++ examples.
   - The `CommandBatch`-return model for `tick()` / `tickAction()`.
   - Explicit note: this is a DOCUMENTED STRUCTURAL CONVENTION, not a virtual base
     class and not a C++20 concept. C++11 / no-RTTI / no-exceptions.
   - The two-phase Drive split (`tickUpdate` / `tickAction`) rationale.
   - The `apply()` staging semantics (no execution, no hardware, no emission until tick).

2. Create `source/subsystems/sensors/SensorsState.h` — new POD aggregate:
   ```cpp
   #pragma once
   #include "messages/sensors.h"   // msg::LineSensorState, msg::ColorSensorState
   namespace subsystems {
   struct SensorsState {
       msg::LineSensorState  line  = {};
       msg::ColorSensorState color = {};
   };
   }
   ```
   This is the type returned by `Sensors::state()` in ticket 003.

3. Verify `python build.py --clean` still passes (new headers added, no `.cpp` files,
   no new include in any existing TU yet — should be zero-risk).

## Files to Create/Modify

- `source/subsystems/SubsystemContract.h` — NEW documentation header
- `source/subsystems/sensors/SensorsState.h` — NEW aggregate POD type

## Acceptance Criteria

- [x] `source/subsystems/SubsystemContract.h` exists and documents the 3-message /
      4-verb convention, fluent-builder form, `CommandBatch`-return tick, two-phase
      Drive split, and apply-stages-only semantics.
- [x] `source/subsystems/sensors/SensorsState.h` exists with `subsystems::SensorsState`
      holding `msg::LineSensorState line` and `msg::ColorSensorState color`.
- [x] Both headers compile under `-std=c++11 -fno-rtti -fno-exceptions` (verified by
      `python build.py --clean`).
- [x] No existing tests broken (`uv run python -m pytest` baseline green).

## Testing Plan

- **Regression**: `uv run python -m pytest` — full suite green (no new test files
  needed for pure documentation + type headers).
- **Compile check**: `python build.py --clean` zero errors. This proves the new headers
  are syntactically valid C++11.
- **Manual review**: `SubsystemContract.h` should be human-readable as a reference
  for the ticket 003/004 implementers.

## Verification Command

`python build.py --clean && uv run python -m pytest`
