---
id: '055'
title: Phase 0 - Reorganize source/io into hal, com, robot
status: done
branch: sprint/055-phase-0-reorganize-source-io-into-hal-com-robot
use-cases:
- SUC-001
issues:
- reorganize-source-io-into-hal-comms-robot-configs.md
test-command: uv run python -m pytest
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 055: Phase 0 - Reorganize source/io into hal, com, robot

## Goals

Rename `source/io/` to `source/hal/`, extract the two concrete robot-config
classes (`NezhaHAL`, `MecanumHAL`) to `source/robot/`, and extract the five
communication transports (`Communicator`, `Radio`, `RadioChannel`, `SerialPort`,
`I2CBus`) to a new `source/com/`. Update all path-prefixed `#include "io/..."` to
`#include "hal/..."`, and update both build files to use the new paths.

This is the foundational rename for the upcoming message-based subsystem
architecture (Phase 1+). It is independently shippable and involves no behavior
or logic changes.

## Problem

`source/io/` mixes three unrelated concerns under one misnamed directory: a
Hardware Abstraction Layer, concrete robot configurations (CODAL-dependent
subclasses of `Hardware`), and communication transports (serial, radio, I2C bus)
that have no robot dependency. The name "I/O" is misleading — the actual HAL
primitives belong under `hal/`, while the robot configs and transports belong in
separate modules.

## Solution

Three atomic file moves using `git mv` to preserve history:

1. `git mv source/io source/hal` — rename the HAL directory.
2. Move `NezhaHAL.{h,cpp}` and `MecanumHAL.{h,cpp}` from `source/io/real/` to
   `source/robot/` (before or after the rename — either order works).
3. Move `Communicator`, `Radio`, `RadioChannel`, `SerialPort`, `I2CBus`
   (`.h`+`.cpp`) from `source/io/real/` to a new `source/com/`.

Then: rewrite ~52 path-prefixed `#include "io/..."` lines to `#include "hal/..."`,
update three regex exclusions in `CMakeLists.txt`, and update glob paths, include
dirs, and CODAL-exclusion filters in `tests/_infra/sim/CMakeLists.txt`.

## Success Criteria

- `uv run python -m pytest` exits with exactly "2361 passed, 2 failed" (the two
  pre-existing failures are not newly introduced or changed).
- `python build.py --clean` compiles and links the device firmware with no
  missing-header or duplicate-symbol errors.
- No source file under `source/` retains a `#include "io/..."` include.
- No `CMakeLists.txt` file retains an `io/` path reference.

## Scope

### In Scope

- `git mv source/io source/hal` (entire directory rename)
- Extract `NezhaHAL.{h,cpp}`, `MecanumHAL.{h,cpp}` to `source/robot/`
- Create `source/com/`; extract `Communicator`, `Radio`, `RadioChannel`,
  `SerialPort`, `I2CBus` (`.h`+`.cpp`) to it
- Sweep all `#include "io/..."` → `#include "hal/..."` across `source/`
- Update `CMakeLists.txt` (device firmware): three path-based exclusion regexes
- Update `tests/_infra/sim/CMakeLists.txt`: glob paths, include dirs, two new
  CODAL-exclusion FILTERs for `NezhaHAL.cpp` / `MecanumHAL.cpp`, add
  `source/com` include dir

### Out of Scope

- Any `.cpp` logic changes — this sprint is path strings and file locations only
- Message-based subsystem interfaces (Phase 1)
- Protobuf message definitions (Phase 2)
- System integration and top-level init changes (Phase 3)
- Moving `I2CBusRawAccess.cpp` or `MotorBusDiagnostics.cpp` (they stay in
  `hal/real/` and resolve `I2CBus.h` transitively through the include path)

## Test Strategy

All verification is build- and test-run-based. No behavior changes exist to
exercise at runtime.

- **Sim/host test suite**: `uv run python -m pytest` — must produce exactly
  "2361 passed, 2 failed". This catches include-path breaks and
  `tests/_infra/sim/CMakeLists.txt` errors.
- **Device firmware clean build**: `python build.py --clean` — catches
  CMakeLists.txt exclusion-regex errors and any missed `#include "io/..."` lines
  in firmware-only translation units.

## Architecture Notes

The consolidated architecture (`architecture-034.md`) already documents the
target layout using `source/hal/`, `source/com/`, and `source/robot/`. This
sprint makes the physical directory tree match the architecture docs.

The device firmware build globs `.cpp` files recursively and adds every
header-containing directory to the include path, so bare-filename includes
survive the rename untouched. Only path-prefixed includes (`"io/..."`) break on
rename.

## GitHub Issues

None.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Atomic source/io rename, file moves, include sweep, and build-file updates | — |

Tickets execute serially in the order listed.
