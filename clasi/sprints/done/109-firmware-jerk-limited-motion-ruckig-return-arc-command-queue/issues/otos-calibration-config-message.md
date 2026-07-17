---
status: in-progress
sprint: '109'
tickets:
- 109-003
- 109-004
- 109-005
- 109-006
- 109-009
---

# Restore a runtime OTOS-calibration config message (protocol + firmware)

## Problem

The OTOS chip's linear/angular calibration scalars (`OL`/`OA` verbs, and OTOS
init `OI`) have **no path over the current binary wire**. The firmware's
`Devices::Otos` HAS the setters — `source/devices/otos.h:217-218`
`setLinearScalar(); // OL`, `setAngularScalar(); // OA` — but they are only
called once at boot from `boot_config` (`source/devices/otos.cpp:40-41`, baked
from the robot JSON's `calibration.otos_*`). There is no runtime path to them.

Concretely: `protos/envelope.proto`'s `ConfigDelta` carries only THREE patch
kinds — `DrivetrainConfigPatch`, `MotorConfigPatch`, `PlannerConfigPatch`
(envelope.proto:104-106) — and `RobotLoop::handleConfig`
(`source/app/robot_loop.cpp:145`) only APPLIES the MOTOR patch. There is no
OTOS config message at all. So `OL 67` / `OA -13` / `OI` are dead on the wire
on BOTH the real robot and the sim (the TestGUI still emits them on connect,
vestigial from the pre-binary text era; they render "not supported").

## Direction

Add an OTOS config path to the binary protocol and firmware:
1. `protos/config.proto` / `envelope.proto`: an `OtosConfigPatch` (linear_scale,
   angular_scale, offset_x/y/yaw as needed — mirror `OtosBootConfig`) added to
   the `ConfigDelta` oneof, plus the `ConfigTarget`/patch_kind plumbing.
2. `RobotLoop::handleConfig`: a case that applies the OTOS patch via the
   existing `Otos::setLinearScalar()` / `setAngularScalar()` (and offset/pose
   setters), the same way the MOTOR patch is live-applied today.
3. Host: `binary_bridge`/`NezhaProtocol` translation for `OL`/`OA`/`OI` → the
   new OtosConfigPatch envelope (restore the arm that was deferred at the
   text->binary cutover; `binary_bridge.py`'s `_OTOS_DEVICE_VERBS` currently
   renders them "nodev / requires sprint 098").

Then the TestGUI's connect-time OTOS-calibration push works on hardware again
(and, via [[sim-honors-otos-calibration]], in the sim).

## Notes
- This benefits REAL hardware (live OTOS recalibration without a reflash). It
  does NOT by itself change sim OTOS behaviour — see the sibling issue
  [[sim-honors-otos-calibration]] for the sim half.
- Filed out-of-process 2026-07-16 during the TestGUI Sim-mode revival.
