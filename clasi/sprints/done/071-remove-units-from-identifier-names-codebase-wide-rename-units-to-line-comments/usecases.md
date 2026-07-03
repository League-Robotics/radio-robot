---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 071 Use Cases

This sprint is a **pure identifier rename** in `source/` (C++ firmware/sim
library): unit suffixes (`Mm`, `Mms`, `Deg`, `Dps`, `Ms`, `Us`, `Pct`, `Hz`)
are stripped from struct fields, locals, parameters, and member variables;
the unit moves to a standard leading `// [unit]` comment tag. No firmware
behavior changes, no wire format changes (see `architecture-update.md`'s
Wire-Compatibility Exclusion Table), no `RobotConfig` default-value changes.

Because nothing observable changes, every SUC below is framed as
**preservation**: the existing top-level UC's behavior, wire output, and
test coverage must be byte-identical after the rename. SUC-007 is the one
genuinely new capability this sprint adds — a documented, grep-able
unit-comment convention — and is parented to a newly proposed UC (there is
no existing UC for source-code documentation conventions).

**Scope note (host Python deferred):** `host/robot_radio/`'s own
unit-suffixed identifiers (`_mm`, `_mms`, `_deg`, `_dps`, `_pct`, `_hz`
snake_case suffixes, incl. `read_ms`) are **not** covered by this sprint's
SUCs — see `architecture-update.md` Decision 1 for the scope-split
rationale (mirrors sprint 070's own issue-1-to-071 split). `tests/simulation/`
and `tests/_infra/` Python files that mirror a renamed C++/proto identifier
(mock `RobotConfig`-shaped test doubles, kwargs, docstrings quoting a field
name) move in lock-step with their corresponding ticket as a **mechanical
consequence** of keeping the suite green — this is not host-Python cleanup,
it is dragging test fixtures along with the C++ names they exercise.

UC-020 and UC-021 were proposed (not yet consolidated into `docs/usecases.md`)
by sprint 069; this sprint's new use case is numbered UC-022 to avoid
collision.

---

## SUC-001: Preserve Runtime-Tunable Calibration Parameter Behavior Across Identifier Rename
Parent: UC-014 (narrows)

- **Actor**: Developer / calibration tooling (Python host via `SET`/`GET`)
- **Preconditions**: Firmware or sim is running with a `RobotConfig` whose
  fields include the units-suffix FIXME set (`trackwidthMm`, `minWheelMms`,
  `rotationOffsetDeg[Neg]`, `arriveTolMm`, `tlmPeriodMs`, `lagOtosMs`,
  `halfTrackMm`) and their unit-suffixed peers (`lagLineMs/ColorMs/PortsMs`,
  `minSpeedMms`, `tickMs`, `sTimeoutMs`, `controlPeriodMs`,
  `mmPerDegL/R/FR/FL/BR/BL`, `odomYawDeg`, `halfWheelbaseMm`).
- **Main Flow**:
  1. Host sends `SET <key>=<value>` using the existing wire key string
     (e.g. `SET tw=130`, `SET minWheelMms=25`, `SET arriveTol=10`) — every
     key string is unchanged by this sprint (see Wire-Compatibility
     Exclusion Table).
  2. `ConfigRegistry::handleSet` commits the value to the `RobotConfig`
     field the key is bound to — same field, new C++ identifier (e.g. the
     `"tw"` key now binds to `RobotConfig::trackwidth` instead of
     `RobotConfig::trackwidthMm`).
  3. `GET <key>` reads back the committed value; every subsystem that
     consumes the field (`Planner`, `Drive`, `MotorController`, `OtosSensor`,
     `BodyVelocityController`) observes it exactly as before.
- **Postconditions**: `SET`/`GET` wire behavior, defaults, validation, and
  every consumer's runtime behavior are byte-identical to pre-071. Only the
  C++ source-level field name changed.
- **Acceptance Criteria**:
  - [ ] Every wire key string in `ConfigRegistry.cpp`'s `CFG_*` table rows
        and `data/robots/robot_config.schema.json`'s `firmware.set_key`
        values is unchanged (diffed against pre-071).
  - [ ] `tests/simulation/unit/test_config_registry.py` and
        `tests/_infra/default_config_golden.json` pass/match unchanged
        (after updating only Python identifiers that mirror the renamed
        C++ field names, not the wire keys).
  - [ ] None of the 7 Config.h FIXME-tagged fields, nor their peers, embed
        a unit suffix in their C++ identifier after this sprint.
  - [ ] `grep -rn "FIXME" source/types/Config.h` returns zero results.

---

## SUC-002: Preserve Dead-Reckoning, EKF Fusion, and External Pose Re-Anchoring Behavior Across Identifier Rename
Parent: UC-006, UC-007 (narrows)

- **Actor**: Developer (Python host via `TLM`/`DBG EST`/`SI`/`OV`/`ZERO pose`)
- **Preconditions**: `Odometry`, `EKF`/`EKFTiny`, and `PhysicalStateEstimate`
  (070-003's freshly de-threaded, per-call-explicit API: `encLeftMm`,
  `encRightMm`, `v_otos_mmps`, `omega_otos_rads`, `vy_otos_mmps`, `x_mm`,
  `y_mm`, `h_cdeg`, `now_ms`, `trackwidthMm`) contain unit-suffixed
  parameter/field names.
- **Main Flow**:
  1. Firmware/sim ticks; `Drive::tickUpdate()` feeds encoder and OTOS
     readings into `PhysicalStateEstimate`'s (renamed) observation methods.
  2. `TLM`'s `enc=`/`pose=`/`otos=`/`encpose=` fields and `DBG EST`'s three
     pose-source lines are emitted exactly as before (these wire tokens are
     already unit-free strings — confirmed by direct read, nothing to
     rename there).
  3. A host `SI`/`OV`/`ZERO pose` command re-anchors the pose through the
     same (renamed) `resetPose`/`zero` methods.
- **Postconditions**: Every `TLM`/`DBG EST` byte and every EKF-fused pose
  value is identical to pre-071 for the same command sequence.
- **Acceptance Criteria**:
  - [ ] `tests/_infra/golden_tlm_capture.json` requires no regeneration
        (byte-identical `TLM` output — no wire field changed).
  - [ ] `Odometry`, `EKFTiny`, `PhysicalStateEstimate` method signatures and
        member/local names carry no unit suffix; every one carries a
        `// [unit]` comment per SUC-007's convention.
  - [ ] `tests/simulation/unit/test_070_003_physicalstateestimate_dethreading.py`
        and the EKF/Odometry unit-test tiers pass unchanged in behavior
        (Python identifiers mirroring the renamed C++ signatures updated
        mechanically).

---

## SUC-003: Preserve Drive-Command Target-State Behavior Across Identifier Rename
Parent: UC-001, UC-002, UC-003 (narrows)

- **Actor**: Developer (Python host issuing `S`/`T`/`D`/`VW`)
- **Preconditions**: `DesiredState`/`TargetState` (`wheelMms`,
  `targetSpeedMms`, `distanceTargetMm`, `deadlineMs`) and
  `OutputState`/`MotorCommands` (`tgtMms`) contain unit-suffixed fields.
- **Main Flow**:
  1. Host issues a drive command; `MotionCommandHandlers` populates
     `DesiredState`'s (renamed) fields.
  2. `BodyVelocityController` profiles and `MotorController` executes,
     writing `OutputState`'s (renamed) per-wheel targets.
  3. The commanded motion (speed, duration, distance, stop conditions)
     plays out identically to pre-071.
- **Postconditions**: Motor commands and stop timing are byte-identical;
  `source/state/DesiredState.h`'s whole-struct FIXME marker is resolved.
- **Acceptance Criteria**:
  - [ ] `grep -rn "FIXME" source/state/DesiredState.h` returns zero results.
  - [ ] `DesiredState`/`OutputState` fields carry no unit suffix and each
        has a `// [unit]` comment.
  - [ ] `tests/simulation/unit/test_body_velocity_controller.py` and the
        S/T/D/VW system/unit test tiers pass with unchanged assertions
        (values, not identifiers).

---

## SUC-004: Preserve Arc-to-Goal Navigation Behavior Across Identifier Rename
Parent: UC-015 (narrows)

- **Actor**: Developer (Python host issuing `G`)
- **Preconditions**: `Planner`/`PlannerBegin.cpp` locals (`arcMm`, `rateDps`,
  `currentAngleDeg`, `kRtRateDps`, `kRtCoastArcMm`, etc.) and the
  proto-generated `msg::PlannerConfig`/`msg::DrivetrainConfig` snake_case
  fields (`mm_per_deg_l/r`, `arrive_tol_mm`, etc.) contain unit suffixes.
- **Main Flow**:
  1. Host issues `G <forward_mm> <left_mm>`.
  2. `Planner` computes the arc, pre-rotate/pursue transition, and arrival
     gate using the same math with renamed locals and renamed
     `msg::`-struct field accessors.
  3. `EVT done G` fires at the same tolerance as before.
- **Postconditions**: Arc geometry, pre-rotate threshold, and arrival
  tolerance are byte-identical to pre-071.
- **Acceptance Criteria**:
  - [ ] `protos/planner.proto`, `protos/drivetrain.proto`,
        `protos/motor.proto` field names carry no unit suffix (renamed
        with `scripts/gen_messages.py` regenerated in the same ticket).
  - [ ] `docs/design/message-inventory.md` regenerated and consistent.
  - [ ] `tests/simulation/unit/test_pursuit_arc_steering.py`,
        `test_planner_subsystem_smoke.py`, `test_rt_slip.py`, and the `G`/
        `RT`/`TURN` system-test tier pass with unchanged numeric assertions.

---

## SUC-005: Preserve OTOS Sensor Calibration and Transformation Behavior Across Identifier Rename
Parent: UC-012, UC-013 (narrows)

- **Actor**: Developer (Python host via `OL`/`OA`/OTOS-related `SET` keys)
- **Preconditions**: `OtosSensor` (real HAL) and `Motor`'s chip-calibration
  math (`mmPerDeg` local, `_lastPositionMm`) contain unit-suffixed
  identifiers.
- **Main Flow**:
  1. Firmware reads the OTOS chip and applies LSB→mm/rad conversion,
     mounting rotation, and lever-arm offset using renamed locals/fields.
  2. `Motor::readEncoderMmF()`/`readSpeed()` convert raw ticks using the
     renamed calibration factor (`mmPerDegL/R` → a name describing the
     calibration factor itself, e.g. `wheelTravelCalibL/R`, per the issue's
     derived-unit-name guidance).
- **Postconditions**: OTOS pose/velocity readings and encoder-derived speed
  are numerically identical to pre-071.
- **Acceptance Criteria**:
  - [ ] `OtosSensor.{h,cpp}` and `Motor.{h,cpp}` carry no unit-suffixed
        identifier; each carries a `// [unit]` comment.
  - [ ] OTOS and motor-calibration unit tests pass with unchanged numeric
        assertions.

---

## SUC-006: Preserve Simulator Error-Model Wire Surface Across Identifier Rename
Parent: UC-020 (narrows — sim-only wire surface introduced by sprint 069)

- **Actor**: Developer / fit tooling / TestGUI operator (via `SIMSET`/`SIMGET`)
- **Preconditions**: `PhysicsWorld`, `SimOdometer`, `SimSetters`, and
  `SimCommands`'s internal function/field names (not its `kSimRegistry[]`
  key strings) contain unit suffixes (`driftPerTickMm`, `sigmaMm`,
  `otosLinDriftMmS`/`otosYawDriftDegS` as C++ function names, etc.).
- **Main Flow**:
  1. Host/TestGUI sends `SIMSET <key>=<value>` using the existing wire key
     string (e.g. `SIMSET trackwidthMm=99.0`, `SIMSET otosLinDriftMmS=0.5`)
     — every `kSimRegistry[]` key string is unchanged by this sprint.
  2. `SimCommands` dispatches to the (renamed) internal setter function;
     `PhysicsWorld`/`SimOdometer`'s internal state updates identically.
  3. `SIMGET <key>` reads back the value.
- **Postconditions**: `SIMSET`/`SIMGET` wire behavior is byte-identical;
  only internal C++ function/field names changed.
- **Acceptance Criteria**:
  - [ ] Every `kSimRegistry[]` key string in `SimCommands.cpp` is unchanged
        (diffed against pre-071).
  - [ ] `tests/simulation/unit/test_simset_profile_chunking.py`,
        `test_sim_commands_registry.py`, and `test_069_knob_telemetry_sweep.py`
        pass unchanged.
  - [ ] `PhysicsWorld`/`SimOdometer`/`SimSetters` internal identifiers carry
        no unit suffix and each carries a `// [unit]` comment.

---

## SUC-007: Maintain Grep-able Physical-Unit Documentation on Renamed Identifiers
Parent: UC-022 (new — "Maintain Consistent, Unit-Free Identifier Naming with Grep-able Unit Documentation")

- **Actor**: Developer reading or modifying `source/` (and, in a future
  sprint, `host/`)
- **Preconditions**: None — this is the standard every other SUC's renames
  conform to.
- **Main Flow**:
  1. Developer opens a declaration this sprint renamed (e.g.
     `float trackwidth;`).
  2. The declaration's trailing (or block) comment begins with a
     `// [unit]` tag (e.g. `// [mm] wheel-to-wheel geometry`), stated once,
     in a standard, uniform position.
  3. Developer runs `grep -rn "// \[mm/s\]" source/` (or any other unit) to
     find every quantity of that physical unit across the codebase, without
     needing to know any identifier's spelling.
- **Postconditions**: Every renamed declaration's unit is discoverable by a
  single, uniform grep pattern; the convention itself is documented in
  `docs/coding-standards.md` for future (including host-Python) use.
- **Acceptance Criteria**:
  - [ ] `docs/coding-standards.md` exists and documents the `// [unit]` /
        `# [unit]` convention with the worked example from the issue
        (`tgtMms` → `tgtSpeed  // [mm/s]`).
  - [ ] Every identifier renamed by this sprint's tickets carries a
        `// [unit]` comment at its declaration (spot-checked per ticket's
        acceptance criteria, not exhaustively enumerated here).
  - [ ] The convention explicitly covers: dimensionless/boolean fields (no
        tag needed), compound units (`mm/s`, `mm/s^2`/`mm/s²`, `deg/s`,
        `rad^2/s`), and the ambiguity-resolution rule (ticket authors must
        choose a descriptive replacement, not a bare strip, when removing a
        unit suffix would collide two previously-distinguished names).
