---
status: pending
sprint: 048
tickets:
- 048-001
- 048-002
- 048-003
- 048-004
- 048-005
---

# Eliminate `#ifdef ROBOT_DRIVETRAIN_MECANUM` Everywhere

## Context

`#ifdef ROBOT_DRIVETRAIN_MECANUM` is a compile-time drivetrain switch that has
metastasized to **81+ sites across ~15 files** — far beyond kinematics. It gates the
HAL choice, wheel count (2 vs 4), the entire lateral `vy` channel, rear-motor
state/control, OTOS `vy` fusion, telemetry fields, and method arities. The
abstraction it was meant to localize ([IKinematics.h](source/kinematics/IKinematics.h),
sprint 046) leaked: the control stack still branches everywhere.

Sprint **048** ("kinematics namespace alias → concrete classes") is ticketed but not
executed and only *partially* removes the macro (it keeps it in the selector header and
the BVC constructor, by its own design). The stakeholder wants the macro gone
**completely**, with two decisions locked in:

1. **Supersede sprint 048** — don't do the partial refactor then immediately redo it.
2. **Compile differential-only now.** Delete all mecanum `#ifdef` branches; keep the
   standalone mecanum math classes in-tree but unwired; remove the build-flag plumbing.
   Re-introducing a mecanum robot becomes a deliberate, top-level future edit ("an amount
   of editing, but a separate thing") — git history preserves the deleted integration.

**Outcome:** zero `#ifdef ROBOT_DRIVETRAIN_MECANUM` anywhere; a single, unconditional
differential code path; robot-model selection consolidated to a documented top-level
point (`main.cpp` HAL line + `IKinematics.h`).

## Process (CLASI)

This is a CLASI project and the stakeholder asked for "a new issue."

1. **Create the new issue** `clasi/issues/eliminate-ifdef-robot-drivetrain-mecanum.md`
   (via the `/issue` skill / `mcp__clasi`) capturing the goal, the supersede-048
   decision, and the differential-only end-state below.
2. **Retire sprint 048.** It is ticketed-but-unexecuted and its directory is untracked
   (`?? clasi/sprints/048-...`). Fold its intent into the new issue and remove/abandon the
   048 sprint so the two don't produce conflicting edits to the same files.
3. Plan + execute a sprint from the new issue (team-lead → sprint-planner → programmer).

## Elimination pattern

Apply uniformly at every site. For each occurrence:

- `#ifdef MECANUM ... #else ... #endif` → **keep the `#else` (differential) body**, delete
  the mecanum body and the three preprocessor lines.
- `#ifdef MECANUM ... #endif` (mecanum-only addition, no `#else`) → **delete the whole
  block**.
- Comments referencing the macro → update to reflect the single differential path.

Leave `RobotConfig` mecanum fields (`halfTrackMm`, `halfWheelbaseMm`, `vyBodyMax`,
`aMaxY`, `jMaxY`, `fwdSign{FR,FL,BR,BL}`, `otosAlphaVy`) in place — harmless, baked from
JSON, and useful when mecanum returns. Config cleanup is explicitly out of scope.

## Files to modify

**Build plumbing (remove the flag entirely):**
- [CMakeLists.txt](CMakeLists.txt) (~302–329): delete the `ROBOT_DRIVETRAIN` block and
  `add_definitions(-DROBOT_DRIVETRAIN_MECANUM)`; always exclude `MecanumHAL.cpp` from
  firmware; keep `MecanumKinematics.cpp` compiling (pure math, retained).
- [tests/_infra/sim/CMakeLists.txt](tests/_infra/sim/CMakeLists.txt) (~32–43): delete the
  mecanum macro def and dual-config logic. Remove the generated `build_mecanum/` dir.
- [build.py](build.py): remove `_read_drivetrain_type()` (79–103) and every
  `-DROBOT_DRIVETRAIN=` arg (197–201, 229).

**Model-selection consolidation (the one top-level edit point for future mecanum):**
- [source/main.cpp](source/main.cpp) (3–7, 167–171, 179–186): keep `NezhaHAL`; drop the
  HAL `#ifdef`, the mecanum HAL instantiation, and the `WHEEL_TEST_MAIN` mecanum
  diagnostic. Add a comment here marking this + `IKinematics.h` as the places to edit to
  build a mecanum robot.
- [source/kinematics/IKinematics.h](source/kinematics/IKinematics.h): collapse to the
  differential branch — `namespace Kinematics = BodyKinematics; constexpr int kWheelCount = 2;`
  — no `#ifdef`. Keep the filename (DesiredState/ActualState/OutputState include it for
  `kWheelCount`).

**Control / superstructure (strip mecanum branches, keep differential):**
- [source/control/BodyVelocityController.cpp](source/control/BodyVelocityController.cpp)
  and `.h`: remove `setTarget(…, vy)`, the mecanum `advance()` branch, `_vy*` members &
  accessors, `RobotGeometry` include/member, and the mecanum DesiredState-publish branch
  (keep the `0.0f` form).
- [source/control/MotorController.cpp](source/control/MotorController.cpp) (17 blocks) and
  `.h`: remove rear-motor pointers/state/methods (`bindRearMotors`, 4-wheel `setTarget`,
  `getEncoderPositions[4]`, `_motorBR/_motorBL`, `_vcBR/_vcBL`, BR/BL enc state).
- [source/superstructure/MotionController.h](source/superstructure/MotionController.h),
  [Superstructure.h](source/superstructure/Superstructure.h)/`.cpp`: drop the `vy_mms`
  `GoalRequest` field and the 8-arg `beginVelocity` overload (keep 7-arg).
- [source/commands/MotionCommands.cpp](source/commands/MotionCommands.cpp) (~808–873,
  885, 1120, 1151): remove mecanum blocks.

**State / odometry / telemetry / robot wiring:**
- [source/control/Odometry.h](source/control/Odometry.h) (222–228, 277–284): remove
  `setOtosAlphaVy`/`fusedVy`/`_fusedVy`/`_otosAlphaVy`.
- [source/state/PhysicalStateEstimate.h](source/state/PhysicalStateEstimate.h)/`.cpp`:
  remove `setOtosAlphaVy` forward + call.
- [source/robot/Robot.cpp](source/robot/Robot.cpp) (98–103, 124–127, ~284–296): remove
  `bindRearMotors`, `setOtosAlphaVy` init, OTOS 3-DOF `vy` read.
- [source/robot/RobotTelemetry.cpp](source/robot/RobotTelemetry.cpp) (53, 85, 105):
  remove mecanum telemetry fields.
- [source/types/Config.h](source/types/Config.h): update the macro-referencing comments
  (line ~48 and the `drivetrain` field note).

**Retain (do NOT delete) — standalone for future mecanum:**
- [source/kinematics/MecanumKinematics.{h,cpp}](source/kinematics/MecanumKinematics.cpp)
  — pure math, no `#ifdef`, keeps compiling in host/sim.
- [source/io/real/MecanumHAL.cpp](source/io/real/MecanumHAL.cpp) — kept in-tree, just
  unwired (always excluded from the firmware build).

**Tests:**
- Keep `tests/simulation/unit/test_mecanum_kinematics.py` (pure-math; survives).
- Remove the tests that exercise the deleted integrated path / require a mecanum build:
  `test_046_007_mecanum_tlm_format.py`, `test_046_006_otos_lateral_vy.py`,
  `test_mecanum_vw_bvc.py`.
- Delete `tests/WheelTestMain.cpp` (mecanum-only per-wheel diagnostic).
- Verify bench scripts that mention mecanum
  (`tests/bench/{wheel_test,teleop,playfield_camera_run}.py`) still work for differential
  (they branch on robot config — adjust only if they hard-require mecanum).

## Verification

- `grep -rn "ROBOT_DRIVETRAIN_MECANUM" source tests CMakeLists.txt build.py` returns
  **zero** matches (the success gate).
- `python build.py` (firmware) compiles clean with no warnings; decode `MICROBIT.hex` to
  confirm the live build (per the stale-incremental-build gotcha — `build.py --clean`).
- `uv run pytest` green (differential single-config), with the mecanum-integration tests
  removed and `test_mecanum_kinematics.py` still passing.
- Smoke the differential path in sim (drive forward / spin) to confirm BVC + MotorController
  behave identically to before the refactor.
