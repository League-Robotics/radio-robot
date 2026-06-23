---
status: final
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 046 Use Cases

## SUC-001: Differential build is byte-identical after mecanum additions

- **Actor**: Developer building the firmware for tovez (differential robot).
- **Preconditions**: `active_robot.json` points at `tovez.json`
  (`drivetrain_type` absent / `"differential"`); all mecanum schema fields are
  optional with defaults; mecanum source files are excluded from the build.
- **Main Flow**:
  1. Developer runs `python build.py` (no `-DROBOT_DRIVETRAIN` flag; build.py
     reads `tovez.json`, sets `-DROBOT_DRIVETRAIN=differential`).
  2. `gen_default_config.py` regenerates `DefaultConfig.cpp`; the diff vs the
     pre-sprint file contains only additive constant lines for new fields.
  3. CMake excludes `MecanumHAL.cpp`, `MecanumKinematics.cpp`; includes
     `NezhaHAL.cpp`.
  4. Firmware compiles and links with no mecanum-conditional code active.
  5. `uv run --with pytest python -m pytest tests/simulation -q` reports
     2093 passed.
  6. Golden-TLM oracle (`tests/simulation/test_tlm_oracle.py`) is unchanged.
- **Postconditions**: Differential firmware is byte-identical to the pre-sprint
  build; no behavior change on tovez.
- **Acceptance Criteria**:
  - [ ] `git diff source/robot/DefaultConfig.cpp` (after regen for tovez)
        contains only additive constant lines — no deletions, no changed values.
  - [ ] Sim suite: `2093 passed` (no regressions).
  - [ ] Golden-TLM oracle unchanged.
  - [ ] Firmware size delta for tovez is zero or negligible (no mecanum code
        linked in).

---

## SUC-002: Developer selects mecanum build from a single robot JSON

- **Actor**: Developer (or team-lead) targeting the mecanum robot.
- **Preconditions**: Mecanum robot JSON exists at `data/robots/<name>.json`
  with `drivetrain_type: "mecanum"`; `active_robot.json` (or `ROBOT_CONFIG`
  env) points at it.
- **Main Flow**:
  1. Developer edits `active_robot.json` to point at the mecanum robot JSON,
     or sets `ROBOT_CONFIG=data/robots/<name>.json`.
  2. `python build.py` reads the robot JSON, passes
     `-DROBOT_DRIVETRAIN=mecanum` to CMake.
  3. CMake sets `-DROBOT_DRIVETRAIN_MECANUM`; excludes `NezhaHAL.cpp`;
     includes `MecanumHAL.cpp` and `MecanumKinematics.cpp`.
  4. Firmware compiles and links with 4-motor HAL, mecanum kinematics,
     3-channel BVC, and `OMNI`/`STRAFE` verbs active.
  5. Host sim library also builds with `ROBOT_DRIVETRAIN_MECANUM`.
- **Postconditions**: A distinct `MICROBIT.hex` for the mecanum robot is
  produced from the same source tree.
- **Acceptance Criteria**:
  - [ ] `python build.py` (with mecanum robot active) exits 0; `MICROBIT.hex`
        produced.
  - [ ] Sim library builds cleanly under `ROBOT_DRIVETRAIN_MECANUM`.
  - [ ] Switching back to tovez and rebuilding restores the differential hex
        (SUC-001 still holds).

---

## SUC-003: Mecanum robot drives forward and turns

- **Actor**: Team-lead operating the mecanum robot via radio relay.
- **Preconditions**: Mecanum firmware flashed; robot on bench or playfield;
  radio relay connected.
- **Main Flow**:
  1. Team-lead sends `VW 200 0` (200 mm/s forward, 0 yaw); robot moves
     forward.
  2. Team-lead sends `VW 0 30` (0 forward, 30 deg/s yaw); robot turns
     in place.
  3. Team-lead sends `VW 150 20` (combined); robot arcs.
  4. `SNAP` returns `twist= vx=NNN vy=0 omega=MMM` with non-zero vx or
     omega values.
- **Postconditions**: Robot reached commanded motion within one second;
  OTOS-led odometry reports a coherent pose trajectory.
- **Acceptance Criteria**:
  - [ ] `VW 200 0` produces measurable forward motion (camera or ruler).
  - [ ] `VW 0 30` produces measurable rotation.
  - [ ] `SNAP` `twist=` shows non-zero `vx` for forward, non-zero `omega`
        for yaw.
  - [ ] No `EVT enc_wedged` during a 5-second forward run.

---

## SUC-004: Mecanum robot strafes laterally

- **Actor**: Team-lead operating the mecanum robot via radio relay.
- **Preconditions**: SUC-003 passing; mecanum firmware flashed.
- **Main Flow**:
  1. Team-lead sends `STRAFE 150` (150 mm/s lateral) or
     `OMNI 0 150 0` (vx=0, vy=150, omega=0).
  2. Robot translates sideways (no rotation, no forward motion).
  3. `SNAP` returns `twist= vx=0 vy=NNN omega=0` (approx).
  4. Team-lead sends `OMNI 100 80 10` (combined forward + strafe + yaw);
     robot executes omnidirectional move.
- **Postconditions**: Lateral motion is confirmed by camera ground-truth.
- **Acceptance Criteria**:
  - [ ] `STRAFE 150` / `OMNI 0 150 0` produces observed lateral motion
        confirmed by the overhead camera.
  - [ ] `SNAP` `vy` is non-zero during lateral command.
  - [ ] Forward motion during a pure strafe command is less than 10% of the
        commanded lateral speed (camera-measured).
  - [ ] `OMNI vx vy omega` verb accepted; combined motion plausible.

---

## SUC-005: Bench and playfield calibration of the mecanum robot

- **Actor**: Team-lead with overhead camera and bench measurement tools.
- **Preconditions**: SUC-003 and SUC-004 passing; mecanum robot JSON has
  placeholder geometry values.
- **Main Flow**:
  1. Team-lead measures `half_track_mm`, `half_wheelbase_mm`,
     `wheel_diameter_mm` with a ruler; updates the robot JSON.
  2. Team-lead calibrates per-wheel `mm_per_wheel_deg_{fr,fl,br,bl}` by
     driving known distances and reading encoders.
  3. Team-lead calibrates OTOS `linear_scale` and `angular_scale` vs the
     playfield camera (existing calibration protocol).
  4. `tests/bench/playfield_camera_run.py` is extended with a strafe leg;
     camera verifies forward / turn / strafe ground-truth within tolerance.
  5. Robot JSON is committed with measured values.
- **Postconditions**: Robot JSON contains calibrated geometry and sensor
  scalars; camera test passes for all three motion primitives.
- **Acceptance Criteria**:
  - [ ] Robot JSON updated with measured `half_track_mm` and
        `half_wheelbase_mm`.
  - [ ] OTOS scalars calibrated; camera reports pose error under 10mm / 5deg
        after a 1m forward run.
  - [ ] `playfield_camera_run.py` strafe leg executes and camera confirms
        lateral direction.
  - [ ] All three motion primitives (forward, turn, strafe) verified
        camera-accurate in one playfield session.
