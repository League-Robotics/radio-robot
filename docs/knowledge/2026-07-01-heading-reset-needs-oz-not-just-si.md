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

## Update (2026-07-01, ticket 063-006): now reproducible and regression-tested in sim

Previously the sim's `SimOdometer` did not reproduce this bug: `setPositionRaw`
(invoked by `OZ`/`OV`) only wrote the raw-register shadow (`_rawX/_rawY/_rawH`), never
the `_odomX/_odomY/_odomH` accumulator that `readTransformed()` actually returns to
the EKF. So in sim, `OZ` was a no-op on the fused pose and `SI 0 0 0` alone didn't
visibly "drift back" either — there was no re-referencing step to drift back FROM.
Separately, `SimOdometer::begin()` (required by `Sensor::is_initialized()` gates on
every OTOS command and on `Robot::otosCorrect()`) was only reachable via
`sim_set_otos_fusion` / `set_field_profile(fuse_otos=True)`, so a bare `tests/simulation`
fixture calling the OTOS command surface directly saw `ERR nodev`.

Fixed in `source/hal/sim/SimOdometer.cpp`'s `setPositionRaw` (now re-references
`_odomX/_odomY/_odomH` using the same LSB scale as the real `OtosSensor` chip) plus a
narrow `sim_begin_otos()` / `Sim.begin_otos()` harness hook
(`tests/_infra/sim/sim_api.cpp`, `tests/_infra/sim/firmware.py`) mirroring
`drive_api_begin_otos()`. Both the bug (SI-alone drifts back) and the fix
(ZERO enc + OZ + SI holds at 0) are now regression-tested in
`tests/simulation/unit/test_sim_otos_heading_reset.py`.

**Gotcha hit while writing that test:** the firmware's stop command is `X`
(`MotionCommands.cpp`, "stop / soft stop"), NOT `S` — `S` is "set wheel speeds" and
takes required args; sending it with none replies `ERR badarg` and the robot keeps
spinning. Using `S` as a bare stop silently left the sim robot still turning, which
looked exactly like an unfixed heading-reset bug (fused heading kept climbing) until
traced back to the wrong stop command in the test helper.
