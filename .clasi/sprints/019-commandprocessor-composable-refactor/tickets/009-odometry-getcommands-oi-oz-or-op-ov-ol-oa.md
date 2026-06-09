---
id: "009"
title: "Odometry::getCommands() — OI, OZ, OR, OP, OV, OL, OA"
status: open
use-cases:
- SUC-001
depends-on:
- '008'
github-issue: ""
issue: ""
completes_issue: false
---

# Odometry::getCommands() — OI, OZ, OR, OP, OV, OL, OA

## Description

Implement `Odometry::getCommands()` returning descriptors for the seven OTOS/odometry
commands: OI (OTOS init), OZ (zero pose), OR (read pose), OP (set pose), OV (set
OTOS velocity scale), OL (set OTOS linear scale), OA (set OTOS angular scale). Add
`OdomCtx {Odometry*, OtosSensor*}` to `Odometry.h`. `Odometry` inherits `Commandable`
(declare in header).

The old switch still handles all OI/OZ/OR/OP/OV/OL/OA commands. No behavior change.

## Acceptance Criteria

- [ ] `source/control/Odometry.h` declares `class Odometry : public Commandable`; includes `CommandTypes.h`; adds `virtual int getCommands(CommandDescriptor* buf, int max) const override`; defines `OdomCtx { Odometry* odo; OtosSensor* otos; }`
- [ ] `source/control/Odometry.cpp` implements `getCommands()` returning descriptors for: `"OI"`, `"OZ"`, `"OR"`, `"OP"`, `"OV"`, `"OL"`, `"OA"`
- [ ] All seven handlers produce wire output identical to the corresponding old switch cases
- [ ] Context is `OdomCtx*` — stored as a value member in `Odometry`, populated by `Robot` at wiring time (or passed via `Robot*` if simpler)
- [ ] `python3 build.py` passes with no errors
- [ ] OTOS commands continue to work correctly: `OZ` (zero), `OL <val>` (set linear scale), `OA <val>` (set angular scale) via `uv run rogo`

## Implementation Plan

### Approach

Read all OI/OZ/OR/OP/OV/OL/OA switch cases in `CommandProcessor.cpp` before writing
handlers. Note which cases need `OtosSensor*` (OI, OL, OA) versus just `Odometry*`
(OZ, OR, OP). The `OdomCtx` holds both.

`Commandable` inheritance: add `#include "CommandTypes.h"` to `Odometry.h`; add
`: public Commandable` to the class declaration; add `getCommands()` declaration.
Context struct stored as `_odomCtx` value member set by a `setContext(OdomCtx)` method
or passed in a constructor parameter (consult `Robot.h` to determine which fits the
existing construction order).

### Files to Modify

- `source/control/Odometry.h` — Commandable inheritance, OdomCtx, getCommands() declaration
- `source/control/Odometry.cpp` — getCommands() implementation

### Testing Plan

- Build: `python3 build.py` must pass.
- Bench: `OZ` zeros pose; `OR` reads back zero; `OL 1.05` sets linear scale; verify
  `OK OL 1.05` response via `uv run rogo`.
