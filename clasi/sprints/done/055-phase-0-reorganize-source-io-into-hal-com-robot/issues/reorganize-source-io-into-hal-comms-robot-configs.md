---
status: in-progress
sprint: '055'
tickets:
- 055-001
- 055-002
---

# Reorganize `source/io/` into HAL + Comms + Robot-configs

> **Phase 0 of the message-based subsystem architecture program.** This is the
> foundational module reorg that
> [message-based-subsystem-architecture.md](message-based-subsystem-architecture.md)
> builds on — its target layout (`source/hal/`, `source/com/`, `source/robot/`,
> `source/subsystems/`) assumes this move has landed. Mechanical and independently
> shippable; do it first. Per the process note below, this is a good `/oop` or
> single-ticket sprint.

## Context

`source/io/` is misnamed and mixes three unrelated concerns. "I/O" really means a
Hardware Abstraction Layer, and the directory currently also holds (a) two concrete
robot configurations and (b) low-level communication transports that have nothing to
do with the robot. This refactor separates them so each directory has one clear job.
It is a **structural move + rename only** — no behavior changes, no logic edits.

The user's intent (three moves):

1. **Rename the HAL.** `source/io/` → `source/hal/` (it's a Hardware Abstraction Layer).
2. **Robot configs out to `robot/`.** `MecanumHAL` and `NezhaHAL` are concrete
   `Hardware` subclasses (robot configurations / subsystem combinations), not HAL
   primitives. Move them to `source/robot/` (a temporary home; a dedicated "subsystem
   combination" location comes later).
3. **Comms out to `source/com/`.** `Communicator`, `Radio`, `RadioChannel`,
   `SerialPort`, and `I2CBus` are transports with no robot dependency (they include
   only `MicroBit.h`). Move them to a new `source/com/`. (I2CBus included per the
   user's decision, even though device drivers sit on it.)

Everything else (device drivers, capability interfaces, sim, replay, the `Hardware`
base) stays in the HAL.

## Why this is mostly mechanical

- The **device firmware build** (`CMakeLists.txt`) globs `*.cpp` **recursively** and
  adds **every header-containing directory** to the include path. So moving files
  doesn't break the firmware build, and bare-filename includes
  (`#include "NezhaHAL.h"`, `#include "I2CBus.h"`) keep resolving regardless of which
  directory the file lives in.
- Only **path-prefixed includes** (`#include "io/capability/..."`, `"io/NoopDevices.h"`)
  break on the rename — 52 lines, all rewritten `io/...` → `hal/...`.
- The **sim/host test build** (`tests/_infra/sim/CMakeLists.txt`) does NOT glob
  recursively — it lists dirs/files explicitly and must be edited.

## Target layout

```
source/hal/                 (renamed from source/io/)
  Hardware.h  NoopDevices.h  Sensor.h  ReplayHAL.{h,cpp}
  capability/   (all I*.h, Pose2D.h — unchanged)
  real/         Motor, OtosSensor, ColorSensor, LineSensor, Servo, PortIO,
                MotorBusDiagnostics, I2CBusRawAccess, BenchOtosSensor
  sim/          (unchanged contents; uses relative ../Hardware.h includes)

source/com/                 (new)
  Communicator.{h,cpp}  Radio.{h,cpp}  RadioChannel.{h,cpp}
  SerialPort.{h,cpp}  I2CBus.{h,cpp}

source/robot/               (existing — add two files)
  + NezhaHAL.{h,cpp}  MecanumHAL.{h,cpp}
```

## Step 1 — Move files (use `git mv` to preserve history)

- Comms → `source/com/`: `Communicator`, `Radio`, `RadioChannel`, `SerialPort`,
  `I2CBus` (`.h` + `.cpp`), from `source/io/real/`.
- Robot configs → `source/robot/`: `NezhaHAL`, `MecanumHAL` (`.h` + `.cpp`), from
  `source/io/real/`.
- Rename the rest: `git mv source/io source/hal`. (Do the two extractions first, then
  rename the remaining tree, or rename first then extract — either order works.)

`I2CBusRawAccess` and `MotorBusDiagnostics` stay in `hal/real/`; they keep
`#include "I2CBus.h"`, which now resolves to `com/I2CBus.h` via the include path.

## Step 2 — Rewrite path-prefixed includes (`io/` → `hal/`)

Mechanical sweep over all of `source/` of the 52 `#include "io/..."` lines:

- `io/capability/...` → `hal/capability/...` (the bulk; ~30 files incl.
  `robot/Robot.h`, `control/*`, `subsystems/*`, `state/*`, `commands/DebugCommands.h`)
- `io/NoopDevices.h` → `hal/NoopDevices.h`

Bare-filename includes need no change. Sim files' relative `../Hardware.h` includes
are unaffected (the `hal/sim` → `hal/` relationship is preserved).

Representative command (review the diff after):
`grep -rl 'include "io/' source | xargs sed -i '' 's|include "io/|include "hal/|g'`

## Step 3 — Update the device firmware build (`CMakeLists.txt`)

Three exclusion regexes near lines 287–314 reference old paths:

- `.*/io/sim/.*`            → `.*/hal/sim/.*`
- `.*/io/ReplayHAL\.cpp$`   → `.*/hal/ReplayHAL\.cpp$`
- `.*/io/real/MecanumHAL\.cpp$` → `.*/robot/MecanumHAL\.cpp$`  (MecanumHAL moved to robot/)

## Step 4 — Update the sim/host build (`tests/_infra/sim/CMakeLists.txt`)

- Source globs/paths: `source/io/sim/*.cpp` → `source/hal/sim/*.cpp`;
  `source/io/ReplayHAL.cpp` → `source/hal/ReplayHAL.cpp`;
  `source/io/real/BenchOtosSensor.cpp` → `source/hal/real/BenchOtosSensor.cpp`.
- Include dirs: `source/io` → `source/hal`, `source/io/capability` →
  `source/hal/capability`, `source/io/real` → `source/hal/real`, `source/io/sim` →
  `source/hal/sim`; **add** `source/com` (so `I2CBus.h` and other transport headers
  resolve transitively).
- **Critical:** `ROBOT_SOURCES = glob(source/robot/*.cpp)` will now pick up the moved
  `NezhaHAL.cpp` and `MecanumHAL.cpp`, which are CODAL-dependent and must NOT compile
  host-side. Add two `FILTER EXCLUDE` lines mirroring the existing
  `WedgeTest.cpp` / `LoopScheduler.cpp` exclusions:
  - `list(FILTER ROBOT_SOURCES EXCLUDE REGEX ".*/NezhaHAL\\.cpp$")`
  - `list(FILTER ROBOT_SOURCES EXCLUDE REGEX ".*/MecanumHAL\\.cpp$")`

## Notes

- `DebugCommands.cpp` (`#include "NezhaHAL.h"`, downcasts to `NezhaHAL*`) and
  `main.cpp` (`#include "NezhaHAL.h"`, instantiates it at line 159) keep their bare
  includes — these resolve once `source/robot/` is on the include path (it already is
  in both builds). No edits needed there beyond Step 2's `io/`→`hal/` sweep (neither
  uses an `io/`-prefixed include for the HALs).
- No `.cpp` logic changes anywhere — only `#include` path strings and file locations.

## Critical files

- Moves: `source/io/real/{Communicator,Radio,RadioChannel,SerialPort,I2CBus,NezhaHAL,MecanumHAL}.{h,cpp}`; `git mv source/io source/hal`
- Include sweep: ~30 files under `source/` (see Step 2)
- Build: [CMakeLists.txt](CMakeLists.txt) (lines ~287, 288, 314), [tests/_infra/sim/CMakeLists.txt](tests/_infra/sim/CMakeLists.txt) (globs ~41–58, include dirs ~110–134)

## Verification

1. **Host/sim build + tests** (fast, catches include + sim-CMake errors):
   build `tests/_infra/sim` and run the Python simulation test suite — must pass.
2. **Device firmware build, clean** (incremental builds go stale silently on
   `/Volumes`): `python build.py --clean` — must compile and link with no missing-header
   or duplicate-symbol errors. Confirms Step 3 exclusions are correct.
3. **Optional mecanum config sanity:** configure-only with `-DROBOT_DRIVETRAIN=...`
   (or whatever selects MecanumHAL) to confirm `MecanumHAL.cpp` is reachable from its
   new `robot/` location when needed.
4. Behavior is unchanged by construction (pure move/rename); no runtime/bench test
   required beyond a successful build, though a smoke flash is cheap insurance.

## Process note

This is a real code change in a CLASI repo. Execution should go through `/oop` (it's a
contained mechanical refactor) or a small single-ticket sprint, per the team-lead
workflow.
