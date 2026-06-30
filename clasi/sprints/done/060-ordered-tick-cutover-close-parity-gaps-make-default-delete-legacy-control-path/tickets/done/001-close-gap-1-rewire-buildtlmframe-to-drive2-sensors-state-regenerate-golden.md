---
id: '001'
title: 'Close gap 1: rewire buildTlmFrame to Drive2/Sensors state; regenerate golden'
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: make-ordered-tick-the-default-close-parity-gaps.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Close gap 1: rewire buildTlmFrame to Drive2/Sensors state; regenerate golden

## Description

Parity gap #1: `buildTlmFrame` (`source/robot/RobotTelemetry.cpp`) currently reads
encoder, pose, velocity, twist, OTOS, line, and color fields from `robot.state.actual`
(a `HardwareState`). In the ordered-tick path, `drive.periodic()` is called in step 1
solely to keep `state.actual` populated for TLM — this is the only thing preventing
full cutover.

This ticket:
1. Rewires `buildTlmFrame` to read drive-related fields from `robot.drive2.state()`
   (a `msg::DrivetrainState` value) and sensor fields from `robot.sensors.state()`
   (a `SensorsState` value).
2. Removes the `drive.periodic()` call from the ordered-tick branch of `LoopTickOnce.cpp`
   (step 1 comment block, line 190).
3. Regenerates `tests/_infra/golden_tlm_capture.json` using the recipe in
   `tests/simulation/unit/test_golden_tlm.py` (lines 22-36). The diff is committed
   and the change is reviewed by the stakeholder. The new values are the accepted baseline.

Field mapping from `state.actual` to subsystem state:

| TLM field | Old source | New source |
|-----------|-----------|-----------|
| `encL`, `encR` | `state.actual.encMm[1]`, `[0]` | `drive2.state().get_encL()`, `get_encR()` (or equivalent DrivetrainState accessors) |
| `pose_x/y/h` | `estimate.getPose(state.actual, ...)` | `drive2.state()` pose fields (DrivetrainState carries fused pose) |
| `velL`, `velR` | `state.actual.velMms[1]`, `[0]` | `drive2.state()` velocity fields |
| `fusedV`, `fusedOmega` | `estimate.getVelocity(state.actual, ...)` | `drive2.state()` fused twist fields |
| `otos.valid`, `optical.pose` | `state.actual.otos.*`, `state.actual.optical.*` | `drive2.state()` OTOS/optical fields |
| `line[4]`, `lineVS` | `state.actual.line[]`, `state.actual.lineVS` | `sensors.state()` line fields |
| `colorRGBC`, `colorVS` | `state.actual.colorR/G/B/C`, `colorVS` | `sensors.state()` color fields |
| `ekf_rej` | `estimate.ekfRejectCount()` | `drive2.state()` EKF reject count (or keep reading `estimate.ekfRejectCount()` directly — both are acceptable since `estimate` is still a Robot member) |

The `motionController.mode()` call (mode char field) is unchanged for this ticket
(mode is read from the legacy `MotionController`; rename happens in ticket 006).

After this ticket: the ordered-tick path no longer calls `drive.periodic()`.
`robot.state.actual` is no longer written by the tick loop under `USE_ORDERED_TICK`.

## Acceptance Criteria

- [x] `buildTlmFrame` does not read `robot.state.actual` for encoder/pose/vel/twist/otos/line/color fields.
- [x] `drive.periodic()` is not called from the ordered-tick branch in `LoopTickOnce.cpp`.
- [x] `tests/_infra/golden_tlm_capture.json` is regenerated with the regen recipe from `test_golden_tlm.py`.
- [x] The diff of `golden_tlm_capture.json` is committed as part of this ticket and is visible for review.
- [x] `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py` passes with the new capture.
- [x] Full host suite passes: `uv run python -m pytest` — green except the 2 known-baseline config-golden failures.

## Implementation Plan

### Approach

Mechanical field-by-field rewire of `RobotTelemetry.cpp`. Do NOT change the TLM
wire format (field names, ordering, snprintf calls) — only change the C++ source
variables that supply the values.

Inspect `msg::DrivetrainState` (in `source/messages/drivetrain.h`) and `SensorsState`
(in `source/subsystems/sensors/SensorsState.h`) to find the correct accessor names
before writing any code.

### Files to modify

- `source/robot/RobotTelemetry.cpp` — rewire `buildTlmFrame` field reads.
- `source/robot/LoopTickOnce.cpp` — remove `drive.periodic()` from the `#else USE_ORDERED_TICK` branch (step 1 block, ~line 190).
- `tests/_infra/golden_tlm_capture.json` — regenerate using recipe.

### Files to read first

- `source/messages/drivetrain.h` — `msg::DrivetrainState` struct layout.
- `source/subsystems/sensors/SensorsState.h` — `SensorsState` struct layout.
- `source/robot/RobotTelemetry.cpp` (full) — understand every read from `state.actual`.
- `source/robot/LoopTickOnce.cpp:161-279` — ordered-tick branch, confirm step 1.

### Regen recipe (from test_golden_tlm.py lines 22-36)

```
python3 -c "
import sys, json
sys.path.insert(0, 'tests/_infra/sim')
from firmware import Sim
s = Sim()
s.send_command('SET sTimeout=60000')
s.send_command('STREAM 50')
frames  = s.tick_collect_tlm(total_ms=200, step_ms=10)
s.send_command('T 100 100 10000')
frames += s.tick_collect_tlm(total_ms=500, step_ms=10)
s.send_command('X')
frames += s.tick_collect_tlm(total_ms=100, step_ms=10)
print(json.dumps(frames, indent=2))
" > tests/_infra/golden_tlm_capture.json
```

The sim build must be rebuilt before running the regen: `cd tests/_infra/sim && python3 build.py`.

### Testing plan

1. Run regen recipe; capture the diff (`git diff tests/_infra/golden_tlm_capture.json`).
2. Run `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py -v` — must pass.
3. Run `uv run python -m pytest` — must be green except the 2 known-baseline failures.

### Documentation updates

None required. The architecture-update.md for this sprint already documents the gap
and the fix.
