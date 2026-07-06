---
status: in-progress
sprint: 084
tickets:
- 084-006
- 084-007
- 084-008
- 084-009
---

# Firmware config + pose-set surface for the new source/ tree

## Context

TestGUI's connect sequence pushes per-robot calibration (`SET ml/mr/tw/rotSlip…`,
OTOS init/lever), and its Operations panel does Sync-Pose (`SI`), Zero-Encoders
(`ZERO enc`), and Set-Origin (`OZ`+`SI`). None of these verbs exist in the new
`source/` tree — the config registry and OTOS command surface were parked in
`source_old`. This issue restores them so TestGUI can configure the robot/sim and
anchor pose.

## Scope

- **`SET`/`GET` config registry**: port `source_old/commands/ConfigCommands.*` +
  `source_old/robot/ConfigRegistry.*` (and boot defaults from `DefaultConfig.cpp`),
  wired to the new `msg::DrivetrainConfig` and `PoseEstimator::configure()` so a
  live `SET` re-propagates (trackwidth, wheel calibration, PID, slip, EKF noise).
  Keep wire keys stable per `.claude/rules/coding-standards.md`.
- **Pose-set**: `SI` (set internal/fused pose), `ZERO enc` (rezero encoders +
  `PoseEstimator` accumulator) — back TestGUI's Sync-Pose / Set-Origin.
- **OTOS command surface**: port `source_old/commands/OtosCommands.*` (`OZ`/`OI`/
  `OL`/`OA`) — against the sim's `SimOdometer` this sprint; on real hardware these
  return `ERR nodev` until the deferred real-OTOS driver lands
  ([[nezha-hardware-otos-driver-for-new-source-tree]]).

## Acceptance (sketch)

`SET tw=...` then `GET` round-trips and visibly changes drivetrain behavior;
`SI x y h` teleports the fused pose; `ZERO enc` rezeroes `enc=`/`encpose=`;
OTOS verbs ack against the sim, `ERR nodev` on hardware (no crash). Sim + bench
verified.

## Dependencies

Depends on 082 (config feeds `PoseEstimator`). Pairs with
[[firmware-closed-loop-motion-verbs]] (same firmware sprint candidate). Unlocks
calibration-push / Sync-Pose / Set-Origin in [[host-testgui-full-revival]].
