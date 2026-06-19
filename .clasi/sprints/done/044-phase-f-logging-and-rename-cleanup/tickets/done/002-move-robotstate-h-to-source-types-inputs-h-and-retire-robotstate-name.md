---
id: '002'
title: Move RobotState.h to source/types/Inputs.h and retire RobotState name
status: done
use-cases:
- SUC-002
depends-on:
- 044-001
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 044-002: Move RobotState.h to source/types/Inputs.h and retire RobotState name

## Description

`source/control/RobotState.h` defines the shared state structs (`HardwareState`,
`MotorCommands`, `TargetState`, `RobotStateContainer`, `defaultInputs`). Its
location in `control/` and the name "RobotState" are legacy from before the FRC
Elite Architecture migration. The §5 target layout places this at
`source/types/Inputs.h` (matching the "Inputs struct" concept from the migration
issue §6).

This ticket moves the file verbatim — struct names and all field layouts are
unchanged. All include paths are updated. The "RobotState" filename is retired
(the struct names inside are kept for now to avoid callsite churn).

**Depends on 044-001** because T1 confirms golden-TLM green before this
mechanical include-path churn.

## Files to Create

**`source/types/Inputs.h`** — verbatim copy of `source/control/RobotState.h`.
No content changes: same structs, same field names, same `defaultInputs` function.

## Files to Delete

**`source/control/RobotState.h`** — deleted after all include paths updated.

## Files to Update (include-path rewrite)

The programmer must grep for all `#include.*RobotState` occurrences and update
each one. Expected files (not exhaustive — grep is the authority):

- `source/control/Odometry.h` — `#include "RobotState.h"` → `#include "types/Inputs.h"`
- `source/control/StopCondition.h` — same
- `source/control/MotionCommand.h` — same
- `source/control/MotorController.h` — same (or via Odometry.h transitively; check)
- `source/control/LoopTickOnce.cpp` — same
- `source/state/PhysicalStateEstimate.h` — `#include "Odometry.h"` may transitively pull it; check direct includes
- `source/robot/Robot.h` — check direct include
- `source/robot/RobotTelemetry.cpp` — check direct include
- `source/superstructure/Superstructure.h` — check
- `source/superstructure/MotionController.h` — check
- `source/io/sim/WorldView.h` and `WorldView.cpp` — currently: `#include "control/RobotState.h"`
- `tests/_infra/sim/sim_api.cpp` — check

Command to find all occurrences:
```
grep -rn "RobotState.h" source/ tests/_infra/
```

Also check for the `#include <` variant and for `"control/RobotState.h"` (qualified path).

**CMake build files** — if any CMakeLists.txt references `RobotState.h` explicitly
(unlikely since they use globs), update those too.

## Acceptance Criteria

- [x] `source/types/Inputs.h` exists with identical content to former `RobotState.h`.
      (`git mv`, 0 insertions/0 deletions — byte-identical, history preserved.)
- [x] `source/control/RobotState.h` does not exist.
- [x] Zero `#include "RobotState.h"` or `#include "control/RobotState.h"` occurrences
      in maintained source files. (19 include sites repointed: 16 → `Inputs.h`, 3 → `types/Inputs.h`.)
- [x] `grep -rn "RobotState\.h" source/ tests/_infra/` returns empty for maintained
      source. (Remaining hits are stale generated dep files under `build/` and
      `build_coverage/`, regenerated on rebuild; the legacy name was also retired
      from 6 prose comments. `RobotStateContainer` — a canonical struct name — is kept.)
- [x] Host build green (immediate compile error if any include missed). (Host/sim lib
      rebuilt clean during the pytest run.)
- [x] ARM firmware build green: `python3 build.py --fw-only` → 0 errors. Then
      `git checkout -- source/robot/DefaultConfig.cpp`. (0 `error:`, `MICROBIT.hex`
      produced; DefaultConfig.cpp restored.)
- [x] Golden-TLM canary passes byte-exact. (`test_golden_tlm.py` 1 passed.)
- [x] Full simulation tier green: `uv run --with pytest python -m pytest -q` >= 2001 passed, 0 errors.
      (2001 passed in 31.71s, 0 errors. Includes the dependency-rule fence
      `test_estimate_dependency_rule.py`, whose allowed-set was updated
      `RobotState.h` → `Inputs.h`.)

## Implementation Plan

1. Create `source/types/Inputs.h` with verbatim content of `source/control/RobotState.h`.
   Keep all includes inside `Inputs.h` as they are (it includes `Config.h`,
   `Protocol.h`, `MotionEventSink.h`).

2. Run the grep command above to enumerate all include-path update targets.

3. Update each file's `#include "RobotState.h"` or `#include "control/RobotState.h"`
   to the appropriate relative path to `types/Inputs.h`. The correct relative path
   depends on the including file's location:
   - From `source/control/*.h`: `#include "../types/Inputs.h"` or `#include "types/Inputs.h"`
     (check which include roots the build uses; typically `source/` is on the include
     path so `#include "types/Inputs.h"` works from any `source/` subdirectory)
   - From `tests/_infra/sim/sim_api.cpp`: use the path that matches what the sim
     CMakeLists.txt sets as include directories.

4. Delete `source/control/RobotState.h`.

5. Compile immediately: `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -q`.
   Fix any missed include before proceeding.

6. Run full simulation tier: `uv run --with pytest python -m pytest -q`.

7. Run ARM build gate: `python3 build.py --fw-only`. Then
   `git checkout -- source/robot/DefaultConfig.cpp`.

## Testing Plan

- **Primary:** Full simulation tier (`uv run --with pytest python -m pytest -q`).
- **Golden-TLM:** `test_golden_tlm.py` byte-exact.
- **ARM build:** `python3 build.py --fw-only`.
- **No new tests** — this is a pure mechanical rename; the existing suite is the gate.

## Notes

- Struct names inside `Inputs.h` are NOT changed in this ticket (`HardwareState`
  is not renamed to `Inputs`; the file name changes but the type names stay). This
  avoids callsite churn that would require touching hundreds of `HardwareState`
  references. The §5 spec says `Inputs.h (=HardwareState)` — the equality sign
  means "this is where HardwareState lives," not "rename HardwareState to Inputs."
- The `defaultRobotConfig()` field-pin test (canary) confirms `RobotConfig` layout
  is unchanged; field-pin diff must be empty.
