# Resetting robot heading to 0 requires `OZ`, not just `SI`

**Date:** 2026-07-01 (sprint 063, ticket 004)

## Symptom
"Set Robot @ 0,0" sent `ZERO enc` + `SI 0 0 0` but the robot's heading did not stay
at 0 — it snapped to 0 then drifted back to a stale non-zero heading.

## Why
There are three heading/pose sources in the firmware, and `SI` only touches two:

- `SI x y h` → `DrivetrainCommand` POSE → `Drive.cpp` `_est.resetPose(...)` →
  `PhysicalStateEstimate::resetPose` → `Odometry::setPose`
  ([source/control/Odometry.cpp:182](../../source/control/Odometry.cpp)). `setPose`
  overwrites the **fused/EKF** pose and the **encoder** snapshot (incl. heading) —
  but it does **NOT** touch the **OTOS** sensor.
- The OTOS retains its own absolute heading (it survives micro:bit reset/reflash;
  only a power cycle or an explicit position write clears it). Every tick,
  `Odometry::correctEKF` fuses the raw OTOS heading back in:
  `s.poseHrad += alphaYaw * wrapπ(θ_otos − s.poseHrad)`
  ([source/control/Odometry.h:145](../../source/control/Odometry.h)). So the fused
  heading converges back to the (stale) OTOS heading within a few ticks.

## Fix
Also send **`OZ`** — "OTOS zero: set position to 0,0,0" via `setPositionRaw(0,0,0)`
([source/commands/OtosCommands.cpp](../../source/commands/OtosCommands.cpp) handleOZ;
[source/COMMANDS.md](../../source/COMMANDS.md) row `OZ`). This is exactly what the
firmware runs at boot ([source/hal/real/OtosSensor.cpp:51](../../source/hal/real/OtosSensor.cpp))
to align the OTOS with the freshly-zeroed encoders. `OZ` re-references the OTOS
position AND heading to the robot's **current physical orientation** = origin.

**Full "reset to origin, heading 0" sequence (host-side, e.g. `_set_origin` in the
testgui):**
```
ZERO enc      # zero encoders
OZ            # zero OTOS position + heading (re-reference to current orientation)
SI 0 0 0      # snap fused/EKF pose to (0,0,0)
```
`OZ` takes no args. The robot must be physically at the intended origin orientation
when this is issued (the whole point: tell the system "here is 0,0,0").

## Related OTOS verbs
`OI` (re-init + IMU calibrate — robot must be still), `OR` (reset Kalman tracking),
`OV x y h` (set OTOS pose to arbitrary values via `setPositionRaw`), `OP` (report
cached OTOS pose). For a plain heading/pose zero-at-current-orientation, `OZ` is the
one to use.
