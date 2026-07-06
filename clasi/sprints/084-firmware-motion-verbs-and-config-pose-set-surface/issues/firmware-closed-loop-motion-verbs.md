---
status: in-progress
sprint: 084
tickets:
- 084-001
- 084-002
- 084-003
- 084-004
- 084-005
- 084-009
---

# Firmware closed-loop motion verbs for the new source/ tree

## Context

The new `source/` firmware has only open-loop velocity control
(`DEV DT VW`/`WHEELS`). TestGUI's command rows and pre-programmed tours need
**closed-loop motion** — "drive N mm", "turn to/by N degrees", "go to XY" — which
lived in the parked production stack (`source_old`) and was deliberately not
carried into the greenfield tree. This issue restores that motion surface on top
of sprint 082's `Subsystems::Drivetrain` + `Subsystems::PoseEstimator`.

## Scope

- **Motion executor** above `Drivetrain`: port `source_old/superstructure/{Planner,
  Superstructure,PlannerConfig}.*` and `source_old/control/{BodyVelocityController,
  HaltController,MotorController,VelocityController}.*`, adapted to the new
  HAL/Drivetrain and command-plane discipline (handlers stage into the
  `DevLoopState` outbox; `devLoopTick` drains — sprint 079).
- **Verbs** (`source_old/commands/MotionCommands.*` + `messages/planner.h`),
  restored as **top-level** verbs to match `docs/protocol-v2.md` and TestGUI's
  existing `commands.py`/`drive.py` (082 already set the top-level precedent with
  `STREAM`/`SNAP`): `D` (distance), `T` (timed), `R` (arc), `TURN` (absolute
  heading), `RT` (relative turn), `G` (go-to XY), `S` (streaming drive), `STOP`,
  plus the `stop=<kind>:<args>` clauses.
- **`mode=` state machine**: extend 082's minimal `I`/`S` to the full
  `I/S/T/D/G/...` set so TestGUI's tour runner can detect motion completion
  (`mode=I` = idle) over `TLM`/`SNAP`.

## Acceptance (sketch)

Against the sim: `D 200 200 500` moves true pose ~500 mm; `RT 9000` rotates ~90°
(within plant tolerance); `stop=` clauses honored; `mode=` returns to `I` at
completion. Hardware bench gate: closed-loop drive/turn on the stand, encoders
proportional, round-trip over serial.

## Dependencies

Depends on 082 (pose estimate for goal closure + `mode=`/`TLM`). Pairs with
[[firmware-config-and-pose-set-surface]] (SET tunes motion params). Unlocks
tours + command rows in [[host-testgui-full-revival]].
