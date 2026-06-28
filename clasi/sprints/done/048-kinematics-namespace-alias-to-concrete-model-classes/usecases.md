---
status: ready
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Use Cases — Sprint 048: Eliminate `#ifdef ROBOT_DRIVETRAIN_MECANUM` (differential-only)

## SUC-048-001: Clean firmware compilation without the drivetrain macro

**Actor:** Build system / developer  
**Goal:** `python build.py` (or `build.py --clean`) produces a firmware binary with
zero references to `ROBOT_DRIVETRAIN_MECANUM` anywhere in the compiled source tree
— no `-D` flag passed, no `#ifdef` evaluated, no conditional block reachable.

**Preconditions:** The macro-elimination edits are applied to `CMakeLists.txt`,
`tests/_infra/sim/CMakeLists.txt`, and `build.py`.

**Success criteria:**
- `grep -rn ROBOT_DRIVETRAIN_MECANUM source tests CMakeLists.txt build.py` returns
  zero matches.
- `python build.py` completes without errors or warnings from the preprocessor.
- Decoded `MICROBIT.hex` confirms a live build (not a stale incremental).

---

## SUC-048-002: Single unconditional differential control path

**Actor:** Firmware runtime  
**Goal:** All control, superstructure, odometry, and telemetry code follows a single
unconditional differential code path — no branch at run time or compile time that was
formerly guarded by `#ifdef ROBOT_DRIVETRAIN_MECANUM`.

**Preconditions:** Mecanum branches stripped from `BodyVelocityController`,
`MotorController`, `Superstructure`, `MotionController`, `MotionCommands`,
`Odometry`, `PhysicalStateEstimate`, `Robot.cpp`, `RobotTelemetry`, `Config.h`.

**Success criteria:**
- Existing differential behavior (forward drive, spin, arc, stop-condition,
  telemetry) is unchanged as confirmed by `uv run pytest` green.
- No `_vy` lateral-channel state, no rear-motor wiring, no mecanum DesiredState
  publish in the compiled binary.

---

## SUC-048-003: Documented top-level mecanum re-introduction point

**Actor:** Future developer building a mecanum robot  
**Goal:** A future developer can restore full mecanum support by making deliberate,
guided edits at exactly two top-level locations — `main.cpp` (HAL selection) and
`IKinematics.h` (kinematics selection) — without needing to grep the codebase for
scattered `#ifdef` sites.

**Preconditions:** `main.cpp` unconditionally instantiates `NezhaHAL`; `IKinematics.h`
unconditionally aliases `BodyKinematics`; both carry a comment marking them as the
re-introduction points.

**Success criteria:**
- `IKinematics.h` contains no `#ifdef` and a clear "edit here for mecanum" comment.
- `main.cpp` HAL block is unconditional with an equivalent comment.
- `MecanumKinematics.{h,cpp}` and `MecanumHAL.cpp` remain in-tree and continue to
  compile cleanly in the host/sim build.

---

## SUC-048-004: Simulation test suite fully green on differential-only build

**Actor:** CI / developer  
**Goal:** `uv run pytest` passes on the single-config (differential) sim build after
removing the mecanum-specific integration tests and the dual-config `build_mecanum/`
sim build.

**Preconditions:** `test_046_007_mecanum_tlm_format.py`, `test_046_006_otos_lateral_vy.py`,
and `test_mecanum_vw_bvc.py` deleted; `tests/WheelTestMain.cpp` deleted;
`test_mecanum_kinematics.py` retained; `build_mecanum/` dir removed from sim CMakeLists.

**Success criteria:**
- `uv run pytest` completes with zero failures.
- `test_mecanum_kinematics.py` passes (pure-math, no `#ifdef` dependency).
- Bench scripts `wheel_test.py`, `teleop.py`, `playfield_camera_run.py` run against
  the differential robot without errors (adjusted only if they hard-require mecanum).
