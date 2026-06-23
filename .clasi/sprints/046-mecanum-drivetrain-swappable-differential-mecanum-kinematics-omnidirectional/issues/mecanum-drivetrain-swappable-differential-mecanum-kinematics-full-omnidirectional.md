---
status: in-progress
sprint: '046'
tickets:
- 046-001
---

# Mecanum Drivetrain — Swappable Differential/Mecanum Kinematics (full omnidirectional)

## Context

A second robot is now on the bench: a **4-wheel mecanum** chassis (Nezha motors, ports **FR=1, FL=2, BR=3, BL=4**), a color sensor, an **OTOS** mounted **51.5 mm behind centerline** (robot frame **+X = front, +Y = left** → `odomOffX=-51.5, odomOffY=0, odomYawDeg=0`), and **no line sensor**. The firmware today hard-assumes a **2-wheel differential** drivetrain (the existing robot, `tovez`).

**Goal:** make the firmware build for **either** the differential robot **or** a mecanum robot, selected **at compile time from the robot config**, so the repo can be cloned, pointed at a robot, and run on different hardware. For the mecanum variant, deliver **full omnidirectional motion** (forward + strafe + turn). The **differential build must stay byte-identical** (golden-TLM oracle + 2093-test sim suite stay green). Work happens on the `mecanum` branch (already created).

**Operator decisions captured:**
- **Full omnidirectional now** (strafe in this pass, not deferred).
- **Scaffold config with placeholders; calibrate geometry/signs/scalars on the stand** (as we did for `tovez`).
- **Robot config name = the 5-char micro:bit announcement name**, read after first flashing the chassis.
- **OTOS-led odometry** — not all four wheels are encodered; lean on the OTOS for pose **and** lateral velocity, encoders secondary.

## Why OTOS-led simplifies omnidirectional
A holonomic (strafing) robot breaks the current 5-state, non-holonomic EKF assumption (no lateral motion). But the **OTOS directly observes the full planar state** — `x, y, θ` and body velocity `vx, vy, ω`. So instead of forcing 4-wheel encoder kinematics into a 6-state EKF, we make the OTOS the primary odometry source for the mecanum build and feed encoder-derived forward velocity (from whichever wheels are encodered) as a secondary check. This keeps the differential EKF untouched and makes lateral velocity a measured quantity, not an estimated one.

## Recommended approach

### 1. Compile-time drivetrain select (mirror `ROBOT_RUN_MODE` / `PRODUCTION_BUILD`)
- Add `identity.drivetrain_type` (`differential`|`mecanum`, default `differential`) to [data/robots/robot_config.schema.json](data/robots/robot_config.schema.json) with a `firmware` mapping to a new read-only `RobotConfig::drivetrain` enum (for `ID`/telemetry + a boot assertion that the compiled variant matches the loaded JSON; **not** a live `SET` key).
- [build.py](build.py) (which already runs `gen_default_config.py` before CMake and sets `-DROBOT_RUN_MODE=REAL`) reads `drivetrain_type` from the resolved active robot and passes `-DROBOT_DRIVETRAIN=<type>`.
- [CMakeLists.txt](CMakeLists.txt) mirrors the `ROBOT_RUN_MODE` block (lines 270–299): default `differential`, `message(...)`, and on `mecanum` → `add_definitions(-DROBOT_DRIVETRAIN_MECANUM)` + `list(FILTER SOURCE_FILES EXCLUDE REGEX ".../NezhaHAL.cpp")`; else exclude `MecanumHAL.cpp`. Mirror in [tests/_infra/sim/CMakeLists.txt](tests/_infra/sim/CMakeLists.txt) (default differential). **Source guards use `#ifdef ROBOT_DRIVETRAIN_MECANUM`** (per the in-repo "#ifdef not #ifndef" convention at CMakeLists.txt:293).

### 2. Kinematics abstraction — compile-time namespace alias (no vtable)
- New `source/control/IKinematics.h`: `namespace Kinematics = BodyKinematics` (N=2) or `= MecanumKinematics` (N=4) under the macro, plus `constexpr int kWheelCount`. A documented static contract, consistent with how the codebase already treats [BodyKinematics](source/control/BodyKinematics.h) (free-function namespace, not an object) — avoids a vptr in the per-tick control loop on the Cortex-M4.
- New types in [source/io/capability/Pose2D.h](source/io/capability/Pose2D.h): `BodyTwist3 { float vx_mmps, vy_mmps, omega_rads; }` and a POD `RobotGeometry { float halfTrackMm, halfWheelbaseMm; }` so kinematics never reaches into `RobotConfig`.
- [BodyKinematics](source/control/BodyKinematics.cpp): **keep** the existing 2-arg `inverse/forward/saturate` verbatim; **add** array-form overloads (`wheels[2]`, `vy` ignored). New `source/control/MecanumKinematics.{h,cpp}`: 4-wheel X-roller maps, wheel order **[FR, FL, BR, BL]**, `k = halfTrack + halfWheelbase`:
  - inverse: `FR=vx−vy−kω, FL=vx+vy+kω, BR=vx+vy−kω, BL=vx−vy+kω`
  - forward: `vx=Σ/4`, `vy=(−FR+FL+BR−BL)/4`, `ω=(−FR+FL−BR+BL)/(4k)`
  - `saturate(wheels[N], …)`: uniform scale on max|wheel| (preserves twist direction).

### 3. Thread 3-DOF through the control/command stack
- [BodyVelocityController](source/control/BodyVelocityController.cpp): add a **third profiled channel** `vy` (trapezoid/S-curve mirroring the forward channel; new `vyBodyMax/aMaxY/jMaxY` in config). Replace the inverse→saturate→forward chain (~lines 121–131) with `Kinematics::` calls over a `float _wheels[kWheelCount]` scratch member + a `RobotGeometry _geom` built once from config.
- [MotorController](source/control/MotorController.cpp): generalize `setTarget(sL,sR)` → `setTarget(const float* wheels, int n)` (keep a 2-arg shim). In the **mecanum build** hold `IMotor* _motor[4]` + `VelocityController _vc[4]`; the differential build keeps exactly two (macro-gated). **Disable the 2-wheel sync-coupling** in the mecanum build (independent per-wheel PI — there's no clean 4-wheel analog).
- [source/types/Inputs.h](source/types/Inputs.h): widen `MotorCommands`/`HardwareState` to `tgt[ ]/pwm[ ]/enc[ ]/vel[ ]` arrays **in the mecanum build only**, with `L/R` scalar names aliased to `[0]/[1]` so shared differential code compiles unchanged.
- Command grammar [MotionCommandHandlers.cpp](source/app/MotionCommandHandlers.cpp): extend `VW` to accept optional `vy=` (back-compatible; absent ⇒ 0) and add a strafe-capable verb (`OMNI vx vy omega` and/or `STRAFE vy [t=…|dist=…]`) with `parse*/handle*` pairs feeding a new `vy` field on the goal/`VW` request.

### 4. OTOS-led odometry + lateral velocity
- Extend the OTOS velocity read ([IOdometer](source/io/capability/IOdometer.h) `readVelocityTransformed`, impl [OtosSensor.cpp](source/io/real/OtosSensor.cpp)) to surface **`vy`** (it currently collapses to forward+ω). The existing in-chip mounting-offset transform already handles `-51.5 mm` — **no offset code change**.
- For the mecanum build, carry `vy` in the fused body twist sourced from the OTOS; add a lateral-velocity path (a 6th EKF state **or** a simpler OTOS-trusting lateral channel — pick the minimal one that passes bench verification). Encoder-derived forward velocity (via `MecanumKinematics::forward` over the encodered wheels) is a secondary observation; pose `x,y,θ` is OTOS-led. The **differential** [Odometry](source/control/Odometry.cpp)/EKF path stays untouched.

### 5. HAL — `MecanumHAL` sibling (4 motors)
- New `source/io/real/MecanumHAL.{h,cpp}`: a copy of [NezhaHAL](source/io/real/NezhaHAL.cpp) with four `Motor` members constructed `(_bus, 1, fwdSignFR)`, `(_bus, 2, fwdSignFL)`, `(_bus, 3, fwdSignBR)`, `(_bus, 4, fwdSignBL)`. Reuse `motorL()/motorR()` for the **front pair** (FL/FR) and add `motorBR()/motorBL()`. OTOS/color/portIO/gripper identical; the line sensor is constructed but its `begin()` probe fails gracefully (no line sensor) — nothing in drive logic consumes it.
- [Hardware.h](source/io/Hardware.h): **additive** default-Noop `motorBR()`/`motorBL()` + `motorCount()` (=2). Refactor `NoopVelocityMotor` out of [ReplayHAL.h](source/io/ReplayHAL.h) into a small shared `source/io/NoopDevices.h` for reuse.
- `main.cpp`: one `#ifdef ROBOT_DRIVETRAIN_MECANUM` selecting `MecanumHAL` vs `NezhaHAL` (both subclass `Hardware`, so `Robot` binding is unchanged).

### 6. Config schema + the new robot JSON
- Schema/`RobotConfig`/[gen_default_config.py](scripts/gen_default_config.py) additions, all **optional with defaults** so existing differential JSON yields a **byte-identical** `DefaultConfig.cpp` (verify via `git diff` — only additive constant lines): `drivetrain_type`; per-wheel `mm_per_wheel_deg_{fr,fl,br,bl}` + `fwd_sign_{fr,fl,br,bl}`; geometry `half_track_mm`, `half_wheelbase_mm`; `vy` profile limits. New generator `kind: enum_drivetrain`; constant-default lines for the new fields; 4-wheel `mmPerDeg` fallback from `wheel_diameter_mm`.
- New `data/robots/<5-char-name>.json` (name from the micro:bit announcement after first flash): `drivetrain_type: mecanum`, OTOS offset `(-51.5, 0, 0)`, color present, line absent, `encoder_count` = actual, and **MEASURE/CALIBRATE placeholders** for geometry/signs/scalars. Switch by editing the one-line [data/robots/active_robot.json](data/robots/active_robot.json) pointer (or `ROBOT_CONFIG` env).
- Host [robot_config.py](host/robot_radio/config/robot_config.py): add `drivetrain_type` + optional new fields (stays drivetrain-agnostic for motion).

## Implementation order (safe build-up that still delivers omni)
1. **Config/build select** (schema, `RobotConfig`, generator, CMake, build.py). Gate check: differential `DefaultConfig.cpp` diff is additive-constants only; sim suite green.
2. **Kinematics math** (`BodyKinematics` array overloads + `MecanumKinematics` + `BodyTwist3` + `RobotGeometry`) with **host unit tests** (inverse↔forward round-trip; known-vector strafe/rotate/forward).
3. **HAL** (`MecanumHAL`, `Hardware` Noop additions, `NoopDevices.h`, `main.cpp` select).
4. **First flash of the chassis** (basic firmware) → read the **5-char micro:bit announcement name** → finalize the robot JSON filename/identity. Bench: spin each of the 4 motors, confirm direction → set `fwd_sign_*`.
5. **N-wheel control** (`MotorController`/`MotorCommands` arrays, `BodyVelocityController` 3-channel, `VW`/`OMNI`/`STRAFE` verbs).
6. **OTOS-led odometry + `vy`** (OTOS velocity read extension, lateral state).
7. **Telemetry** (`vy` in `twist=`; per-wheel `vel=` for the mecanum build).
8. **Bench + playfield bring-up & calibration** (measure geometry, per-wheel `mmPerDeg`, OTOS scalars); camera-verify forward/turn/**strafe**.

## Critical files
- Kinematics seam: [source/control/BodyKinematics.{h,cpp}](source/control/BodyKinematics.cpp), new `MecanumKinematics.{h,cpp}` + `IKinematics.h`, [Pose2D.h](source/io/capability/Pose2D.h).
- Control stack: [BodyVelocityController.cpp](source/control/BodyVelocityController.cpp), [MotorController.{h,cpp}](source/control/MotorController.cpp), [Inputs.h](source/types/Inputs.h).
- Commands: [MotionCommandHandlers.cpp](source/app/MotionCommandHandlers.cpp).
- Odometry/OTOS: [Odometry.cpp](source/control/Odometry.cpp), [IOdometer.h](source/io/capability/IOdometer.h), [OtosSensor.cpp](source/io/real/OtosSensor.cpp).
- HAL: new `source/io/real/MecanumHAL.{h,cpp}`, [Hardware.h](source/io/Hardware.h), new `source/io/NoopDevices.h` (refactored from [ReplayHAL.h](source/io/ReplayHAL.h)), `main.cpp`.
- Config/build: [robot_config.schema.json](data/robots/robot_config.schema.json), [Config.h](source/types/Config.h), [gen_default_config.py](scripts/gen_default_config.py), [CMakeLists.txt](CMakeLists.txt), [build.py](build.py), [tests/_infra/sim/CMakeLists.txt](tests/_infra/sim/CMakeLists.txt), new `data/robots/<name>.json`, [active_robot.json](data/robots/active_robot.json), [host/robot_radio/config/robot_config.py](host/robot_radio/config/robot_config.py).

## Verification
- **Differential unchanged:** build `tovez` firmware; run the sim suite (`uv run --with pytest python -m pytest tests/simulation -q`, expect 2093 pass); confirm regenerated `DefaultConfig.cpp` diff is additive-constant lines only; golden-TLM oracle unchanged.
- **Mecanum math:** host unit tests for `MecanumKinematics` inverse/forward round-trip + saturation.
- **Mecanum on hardware:** point `active_robot.json` at the new JSON, build (`-DROBOT_DRIVETRAIN=mecanum`), flash. On the stand: each wheel spins in the correct commanded direction (sets `fwd_sign_*`); forward + turn via existing verbs; **strafe** via `OMNI`/`STRAFE`; `SNAP` reports non-zero `vx/vy/ω` and a bounded pose. Then **playfield, camera-verified** (extend `tests/bench/playfield_camera_run.py` with a strafe leg): drive forward, turn, and strafe; confirm camera ground-truth matches commanded direction.

## Open items / operator-supplied (bench calibration)
- The **5-char micro:bit name** (after first flash) → robot config filename + announcement name.
- **Geometry:** `half_track_mm`, `half_wheelbase_mm`, `wheel_diameter_mm` (MEASURE).
- **Per-wheel `fwd_sign_{fr,fl,br,bl}`** (VERIFY on the bench — mecanum sign errors make it spin instead of strafe).
- **Per-wheel `mm_per_wheel_deg_*`** for encodered wheels (CALIBRATE).
- **OTOS `linear/angular` scalars** (CALIBRATE vs the camera).
- **`encoder_count` + which wheels are encodered** (OTOS-led, so secondary — but needed to know which wheels feed forward-velocity).
- Decision to confirm during impl: **6-state EKF vs simpler OTOS-trusting lateral channel** for `vy` (pick the minimal one that verifies on the bench).

(Note: this is a sizable multi-phase change; it could be run as a CLASI sprint with tickets per the implementation order above if you want the process tracking — say the word and I'll set that up. Otherwise I'll execute the order directly on the `mecanum` branch.)
