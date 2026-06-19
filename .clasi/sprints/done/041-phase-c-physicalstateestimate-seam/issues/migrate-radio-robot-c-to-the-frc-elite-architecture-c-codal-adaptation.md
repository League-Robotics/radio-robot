---
status: in-progress
sprint: '041'
tickets:
- 038-001
- 041-001
- 041-002
- 041-003
---

# Migrate radio-robot-c to the FRC Elite Architecture (C++/CODAL adaptation)

## Context

**Why.** Re-organize the firmware to adopt the *FRC Elite Architecture* (`frc-code-scout/knowledge/build-spec/`) — its three structural seams, its vendor/transport confinement rules, and its sim/test discipline — to make the code cleaner, more testable, and future-proof.

**The central adaptation.** frc-code-scout describes a **Java/WPILib** architecture; radio-robot-c is **C++/CODAL firmware on a micro:bit/nRF52** (which cannot run WPILib). So this is an *in-place adaptation of the patterns into the existing C++ code*, **not** a port to Java. Two explicit choices replace the WPILib-specific parts:
- **Capability-typed devices** (`frc-code-scout/knowledge/alternatives/01-capability-typed-devices.md`) replace the HAL — interfaces named by *what a device does*, vendor/CODAL types confined to impls, one Hardware object that constructs + injects.
- **Physical-plant simulation** (`frc-code-scout/knowledge/alternatives/02-physical-plant-simulation.md`) replaces the welded mock-sim — a first-class, **settable** ground-truth plant, separate from the estimate, with observation (sensor-error) models layered on top.

**The good news.** The codebase is already ~80% this shape: `Hardware` is an abstract device factory with `NezhaHAL`/`MockHAL` REAL/SIM impls; `Odometry`+`EKF` is the estimator; `MockMotor`/`ExactPoseTracker`/`BenchOtosSensor` are proto-plants; `HardwareState` is a de-facto inputs/logging struct; `loopTickOnce` is the shared periodic orchestrator. **This is a rename-by-capability + leak-sealing + seam-naming + sim-untangling job — not a rewrite.**

**Decisions locked (this session):** full reorganization (all seams) · full spec-style directory layout (reached incrementally via alias shims) · start now, pausing the encoder-calibration mission (the migration is behavior-preserving, so baked calibration carries through and the cal can be finished after).

**Hard constraints (project memory):** preserve calibration values exactly; every phase keeps the **1954 host tests green** + passes a **hardware bench smoke**; structural changes only — **no behavior changes** (no new in-loop recovery, no "improvements" to wedge/off-table/EKF logic) folded into a restructure; zero-heap, single-threaded, deterministic, `loopTickOnce` stays shared firmware↔sim.

---

## Target architecture (mapped to this firmware)

| Elite seam | This firmware today | Migration |
|---|---|---|
| **Seam 1 — IO layer** (XxxIO + Inputs + XxxIO<device> + XxxIOSim) | `IMotor`/`IOtosSensor`/… + `Motor`/`OtosSensor`/… + `MockMotor`/… ; `HardwareState` is the inputs struct | → **capability-typed devices** (§1) + **physical-plant sim** (§2); `HardwareState` formalized as the inputs/logging contract |
| **Seam 2 — state estimate** (one fused belief; observations in, pose out) | `Odometry` + `EKF` writing pose into `HardwareState` fields | → **`PhysicalStateEstimate`** (§3) — consolidate, observations-in API |
| **Seam 3 — Superstructure** (goal → setpoints, one guarded transition) | `MotionController` (S/T/D/G/VW state machines) + scattered safety in `loopTickOnce` | → **thin `Superstructure`** (§4) — goal enum + one guarded `requestGoal`, centralize keepalive/halt/bounds |
| Logging contract (inputs struct logged each cycle) | `buildTlmFrame`/`telemetryEmit` over `HardwareState` | keep — TLM **is** the inputs-struct log (§6) |
| Run-mode select (REAL/SIM/REPLAY) | implicit: which `Hardware` subclass `main.cpp` vs `sim_api.cpp` builds | → explicit `RobotMode` + CMake `ROBOT_RUN_MODE` (§6) |

---

## §1 — Capability-typed device layer (replaces the HAL)

**Taxonomy** (new `source/io/capability/*.h`; units are mm/mm·s/rad/rad·s; neutral enums; **no vendor/CODAL/transport type in any signature**):

| Today (named by device) | New (named by capability) | Action |
|---|---|---|
| `IMotor` (9 methods; leaks Nezha split-phase + `RobotConfig&`) | **`IVelocityMotor`** (drive wheel: `setOutput`, `positionMm`, `velocityMmps`, `resetPosition`, `setNeutralMode`) **+ `IPositionMotor`** (on-chip move-to-position — the old 0x70/0x5D) | **split into two capabilities** — one `Motor` impl implements both |
| `IOtosSensor` (vendor-named; 18 methods leak int16 LSBs + `cfg`) | **`IOdometer`** — ONE interface (pose + velocity + calibration); NOT split | rename + seal the LSB/`cfg` leaks (drops the vendor "OTOS") |
| `IServo` | **`IPositionMotor`** | fold in — same capability (the `Servo` impl implements `IPositionMotor` only) |
| `ILineSensor` | `ILineSensor` | **keep** (name is fine) |
| `IColorSensor` | `IColorSensor` | **keep** (name is fine) |
| `IPortIO` | `IPortIO` | **keep** (name is fine) |
| (n/a — `MotorController` includes `MicroBit.h` + names `I2CBus`) | `IBusDiagnostics` (`errorCount`/`reentryViolations`/`lastError`) | **new** — seals the one real control-layer vendor leak |

**Vendor confinement** — five "vendors" stay below the device line, enforced by a CI grep gate (no `MicroBit`/`I2CBus`/Nezha `0x46`-class bytes/`int16…Raw`/`microbit_random` above `source/io/`):
- **CODAL/micro:bit core** → device impls, `I2CBus`, `NezhaHAL`, `main.cpp`, the loop driver only.
- **Nezha I2C frame + split-phase timing** → the `Motor` impl only (move the split-phase request/collect state machine *out of* `Robot::controlCollectSplitPhase` and *into* `Motor`, driven by `Hardware::tick(now)`; `positionMm()` becomes a cheap accessor). Keep the I2C-wedge write-throttle inside the impl — **bytes on the wire unchanged**.
- **SparkFun OTOS registers / LSBs** → the `OtosSensor` impl only (the `IOdometer` impl; raw-register/scalar access stays as `IOdometer` methods used by the `O*`/`DBG OTOS` handlers — int16 LSBs never cross the interface).
- **`I2CBus`** → owned by `NezhaHAL`, exposed upward only as `IBusDiagnostics` (a small `MotorBusDiagnostics` adapter bound to addr 0x10). `MotorController.h` drops `#include "MicroBit.h"` and the `I2CBus*`.
- **RNG** (`microbit_random` / `std::mt19937`) → already confined to the sim/bench impls; keep.

**The Hardware object** (`source/io/Hardware.h`, kept + retyped): capability-typed accessors; the `Motor` impl exposes both `IVelocityMotor&` and `IPositionMotor&` (one device, two capabilities — exactly the doc's `TalonFXPositionMotor implements PositionMotor, VelocityMotor`); a `RobotMode` factory selects **REAL=`NezhaHAL`** / **SIM=`SimHardware`** (§2) / **REPLAY=`ReplayHAL`** (stub). Zero-heap value-member ownership preserved.

Critical files: `source/hal/IMotor.h` (→ `IVelocityMotor`+`IPositionMotor`), `source/hal/IOtosSensor.h` (→ `IOdometer`), `source/hal/Hardware.h`, `source/hal/Motor.cpp` (impl of `IVelocityMotor`+`IPositionMotor`), `source/hal/OtosSensor.cpp` (impl of `IOdometer`), `source/hal/Servo.cpp` (impl of `IPositionMotor`), `source/control/MotorController.h` (the leak).

---

## §2 — Physical-plant simulation (replaces the welded mock-sim)

Untangle the three models that are currently fused (`MockMotor` mixes plant+slip+noise; `BenchOtosSensor` mixes ideal-truth+errored-reading; `ExactPoseTracker` is a third oracle):

- **`PhysicsWorld`** (`source/io/sim/PhysicsWorld.{h,cpp}`) — the **one** ground truth, integrated by **one** `update(dt)`. Owns true chassis pose, true per-wheel travel/velocity, true line/color/port values, the OTOS mount transform, and dynamics-error extras (motor lag/momentum). **Two ways in:** `setActuators(pwmL,pwmR)`+`update(dt)` (evolve) and `setTruePose`/`setTrueWheelTravel`/… (**set truth directly** for isolation tests). Canonical midpoint-arc integration (the formula currently triplicated). **Slip moves here** — to the chassis-integration step (wheel travel is real; body rotation is reduced by `effectiveSlip`), *not* the encoder reading — validated numerically against the field-024 fixture so `sim_field_profile` behavior is preserved.
- **Observation models** (`source/io/sim/Sim*.{h,cpp}`) — own the *error*, read `const PhysicsWorld&`, implement the capability interfaces: `SimMotor` (quantization + noise + **frozen/wedge dropout**; `setOutput` forwards PWM → `plant.setActuators`), `SimOdometer` (noise + yaw drift + mount transform + **LIFT/invalid-status + read-failure dropout**; replaces *both* BenchOtos `_otos` and MockOtos `_odom`), `SimLineSensor`/`SimColorSensor`/`SimPortIO`. Every error setter defaults to no-op ⇒ a fresh sensor is **perfect** (the "fidelity dial" at zero).
- **`SimHardware`** owns the plant + constructs each observation model against it; its `tick(now,cmds)` is the one ordered `plant.update(dt)`.
- **Control law on one side** (Case B): the per-wheel PI+FF stays in `MotorController` **above** the device line; `SimMotor` only stores PWM — **no second controller**. Fixes a latent bug (today BenchOtos integrates *commanded* velocity while ExactPose uses *true* velocity — they can disagree; the plant has one truth driven by actual PWM).
- **`WorldView` adapter** + `sim_get_true_*` ABI → `sim.estimation_error()` makes **estimate-vs-truth assertable** (impossible on real hardware). Enables the doc's test matrix: plant-only · observation-only · estimator-only · subsystem-control · whole-robot.
- **ABI / tests**: existing `sim_api.cpp`/`firmware.py` surface kept (back-compat for the ~25 sim-driven tests), routed to the plant; `sim_set_enc_l/r` *fixed* to set true travel (today it lies); `sim_get_exact_pose_*` aliased to `sim_get_true_pose_*`; new settable-truth + error-layer + `sim_set_perfect()` hooks. The ~45 pure-Python mirror tests are untouched; the estimator mirrors (`test_ekf.py`, `test_otos_fusion.py`) can later assert the *real* C++ EKF against plant truth. Deterministic stepped time + fixed-seed LCG preserved.

Critical files: `source/hal/mock/MockHAL.{h,cpp}`, `source/hal/mock/MockMotor.{h,cpp}`, `source/hal/BenchOtosSensor.{h,cpp}` (firmware bench device stays, gated by `BENCH_OTOS_ENABLED`), `tests/sim/sim_api.cpp`, `tests/sim/firmware.py`, `tests/conftest.py`.

---

## §3 — Seam 2: `PhysicalStateEstimate` (the fused belief)

Consolidate `Odometry`+`EKF` into one belief object (`source/state/PhysicalStateEstimate.{h,cpp}`; `EKF.{h,cpp}` moves under `state/`). It is the estimate dual of the sim's `PhysicsWorld` (truth). **Composition-first: move the bodies verbatim, change no numerics.**

- **Observations in:** `addOdometryObservation` (=`Odometry::predict`), `addOtosObservation` (=`correctEKF`), `addCameraObservation`/`resetPose` (=`setPose`, the external camera fix arriving via the radio `SI` verb). Observation structs are plain PODs — never a device ref.
- **Belief out:** `getPose()`, `getVelocity()`.
- **Dependency rule:** imports `<stdint.h>`/`<math.h>`/`EKF`/PODs only — **no** CODAL, device handles, `Protocol.h`, or `Commandable`. The OTOS-tuning verbs (`OI/OZ/OR/OV/OL/OA/OP`) move to an app-layer handler set.
- **Transition safety:** keep publishing the fused pose back into `HardwareState.pose*`/`fused*` so existing readers (`buildTlmFrame`, `MotionController::getPoseFloat`) work byte-identically; repoint readers to `getPose()` in the cleanup phase.

Critical files: `source/control/Odometry.{h,cpp}`, `source/control/EKF.{h,cpp}`, `source/control/RobotState.h`.

---

## §4 — Seam 3: thin `Superstructure` (coordination)

Honestly thin (differential drive + one optional gripper, no mechanism-vs-mechanism interlock). Value = **one guarded entry point** + consolidating today's scattered safety. `source/superstructure/Superstructure.{h,cpp}`:
- `enum class Goal { IDLE, STREAM, TIMED, DISTANCE, GOTO, TURN, ROTATE, VELOCITY, ARC, ESTOP }`; `requestGoal(GoalRequest)` is the single transition function the verb handlers route through (instead of calling `motionController.beginX()` directly); `MotionController` moves under `superstructure/` as the goal executor.
- **Centralize** the keepalive-needs-watchdog decision, the SAFE one-shot re-arm, and the `ESTOP`/`X` path (driven by `HaltController`) — today spread across `loopTickOnce`. **Pre-cut** a `goalAllowed()` world-bounds hook (no new behavior now — just the seam, so the off-table fence can later live in one place).
- Foundation tier only — a `switch`-over-`Goal`. **Do not** build a state-graph/transition-table (D2 L3–4).

Critical files: `source/control/MotionController.{h,cpp}`, `source/control/LoopTickOnce.cpp`, `source/control/HaltController.{h,cpp}`.

---

## §5 — New directory layout (full spec layout, reached incrementally)

```
source/
  io/                       # was hal/  — the device seam
    capability/  IVelocityMotor.h IPositionMotor.h IOdometer.h
                 ILineSensor.h IColorSensor.h IPortIO.h IBusDiagnostics.h
    real/        NezhaHAL.cpp Motor.cpp OtosSensor.cpp
                 LineSensor.cpp ColorSensor.cpp PortIO.cpp Servo.cpp
                 I2CBus.cpp  BenchOtosSensor.cpp        # firmware bench device
    sim/         SimHardware.cpp PhysicsWorld.cpp SimMotor.cpp
                 SimOdometer.cpp SimLineSensor.cpp SimColorSensor.cpp
                 SimPortIO.cpp WorldView.cpp
    Hardware.h   ReplayHAL.cpp(stub)
  subsystems/    drive/Drive.*  gripper/Gripper.*  sensors/{Line,Color,Ports}.*
  state/         PhysicalStateEstimate.*  EKF.*
  superstructure/ Superstructure.*  MotionController.*  HaltController.*  StopCondition.*
  app/           CommandProcessor.*  MotionCommandHandlers.*  SystemCommands.*
                 ConfigCommands.*  DebugCommandable.*  OtosCommands.*(new)
  robot/         Robot.*  RobotTelemetry.*  ConfigRegistry.*  DefaultConfig.cpp
  types/         Config.h Protocol.h CommandTypes.h Inputs.h(=HardwareState)
  main.cpp

tests/                          # test tiers, by how-much-hardware-is-real — see §7
  simulation/  unit/  system/   # entirely simulated (SIM mode + PhysicsWorld)
  bench/       unit/  system/   # real hardware on a stand, odometry simulated
  field/       unit/  system/   # all real hardware + overhead camera
  _infra/                       # sim build (sim_api/firmware.py/CMake), testkit, calibrate/, tools/
```
Reached via **alias shims** (e.g. `using IMotor = IVelocityMotor;` during transition) so each step compiles green; old `I*.h` headers deleted last.

---

## §6 — Build, logging, calibration

- **CMake `ROBOT_RUN_MODE`** (`REAL`|`SIM`|`REPLAY`) replaces both `list(FILTER … EXCLUDE REGEX ".*/hal/mock/.*")` filters: REAL builds `io/real` (excludes `io/sim` + `LoopScheduler` stays), SIM builds `io/sim` (excludes `io/real/NezhaHAL` + CODAL-only files), REPLAY = SIM source set with no-op feed impls. Keep `PRODUCTION_BUILD`/`BENCH_OTOS_ENABLED` orthogonal; `build.py` dual build + `--fw-only` unchanged.
- **Logging contract:** TLM **is** the inputs-struct log — keep `buildTlmFrame`/`telemetryEmit`; enforce "every subsystem writes its inputs slice in `updateInputs`, no subsystem prints." REPLAY-fed TLM log is the AdvantageKit-replay analogue (seam cut, impl deferred).
- **Calibration carry-through (protect):** `data/robots/tovez.json` → `scripts/gen_default_config.py` → `DefaultConfig.cpp` is **untouched**; `RobotConfig` stays the calibration carrier (read live as an impl member — removing `cfg` from public read *signatures* is the win, keeping it as an impl member is fine). A field-pin test gates every phase.

---

## §7 — Test system (simulation / bench / field tiers)

The reorg makes the test system first-class: tests are organized **by how much hardware is real**, matching the run mode and the existing `make_target(sim|bench|production)` switch in `robot_radio.testkit`.

| Tier | Hardware | Target | Odometry / plant | Covers |
|---|---|---|---|---|
| **simulation** | none — runs **entirely** in SIM mode | `make_target("sim")` (host ctypes lib + `PhysicsWorld`) | fully simulated | logic + whole-robot scenarios, zero hardware |
| **bench** | real robot on a stand; **odometry simulated**, motors + I2C sensors real | `make_target("bench")` (radio/serial → real fw, bench mode) | `BenchOtosSensor` for pose; real encoders/motors/sensors | real-device behavior that can't be honestly faked |
| **field** | **all** real hardware on the playfield + overhead camera | `make_target("production")` | real OTOS + camera fixes | end-to-end navigation / system behavior |

Directory structure — **by tier, with unit/system scope under each**:
```
tests/
  simulation/  unit/      # pure logic + one subsystem on its Sim device — no hardware
               system/    # whole-robot in sim (drive square, GOTO, estimate-vs-truth)
  bench/       unit/      # real-device units: motor drive, encoder read, I2C, OTOS status
               system/    # bench end-to-end on the stand (odometry simulated)
  field/       unit/      # (minimal)
               system/    # field navigation / tours (needs playfield + camera)
  _infra/      # sim build (sim_api/firmware.py/CMake), testkit helpers, calibrate/, tools/
```
- **unit vs system:** *unit* = one thing in isolation (a subsystem on its Sim device, or one real device on the bench); *system* = the whole robot end-to-end.
- **Where a test goes:** testable with no hardware → **simulation** (the default, always-run tier); needs a real device that can't be simulated honestly (I2C timing, real encoder/motor, OTOS lift) → **bench**; full navigation against the camera → **field**. Unit tests live primarily under **simulation** and **bench**; **field** is mostly system tests.
- Subsumes today's flat layout: `tests/unit/` → `simulation/{unit,system}`; `tests/bench/` scripts → `bench/`; `tests/system/` (goto_world, tours) → `field/system/`. The physical-plant work (§2) is what lets the **simulation** tier run "entirely in simulation."

---

## Migration sequence (each phase = one CLASI sprint)

**Per-phase Definition of Done:** **simulation tier green** (all sim unit + system tests — the continuous mandatory gate, ⊇ today's 1954) · **bench tier green** (the bench unit/system tests that apply — runnable now, robot on the stand) · `defaultRobotConfig()` field-pin diff empty · golden-TLM canary unchanged · no new heap/fibers · vendor-confinement grep gate (ratchets tighter each phase). **Field/system tier deferred** — each phase records its pending field checks to run when the robot's on the playfield. Move behavioral bodies **verbatim**.

- **Phase 0 — Safety nets + test tiers (no source moves):** stand up the `tests/{simulation,bench,field}/{unit,system}` tiers (§7) and move the current host suite into `simulation/`; add canaries — vendor-confinement grep gate, `defaultRobotConfig()` field-pin, golden-TLM frame for a fixed command sequence; record the calibration + bench baseline. (`tests/` reorg only — `source/` untouched.)
- **Phase A — Capability devices:** add `io/capability/*` interfaces (as aliases over the current `I*` first), seal the `MotorController` `I2CBus`/`MicroBit.h` leak via `IBusDiagnostics`, move Nezha split-phase into the `Motor` impl + `Hardware::tick`, split `IMotor`→`IVelocityMotor`+`IPositionMotor` and rename `IOtosSensor`→`IOdometer` (fold `IServo` into `IPositionMotor`; seal the OTOS LSB/`cfg` leaks), `hal/`→`io/` + `ROBOT_RUN_MODE`.
- **Phase B — Physical-plant sim:** create `PhysicsWorld` + observation models; collapse `MockMotor`/`ExactPoseTracker`/`BenchOtos`-dual-accumulators into the clean split; re-point `sim_api`/`firmware.py` (settable truth, error layers, `WorldView`); migrate the ~25 sim-driven tests (mostly via aliases).
- **Phase C — `PhysicalStateEstimate` seam:** wrap `Odometry`+`EKF` by composition; repoint the three observation sites; strip `Commandable`; move EKF under `state/`.
- **Phase D — thin `Superstructure`:** `Goal` enum + guarded `requestGoal`; route verb handlers through it; centralize keepalive/SAFE/ESTOP; pre-cut `goalAllowed()`.
- **Phase E — subsystem/periodic:** wrap Drive/Gripper/sensors as subsystems with `periodic()`/`updateInputs()`; `loopTickOnce` calls them **in the same order**.
- **Phase F — logging + rename/cleanup:** repoint TLM readers to `estimate.getPose()`, stop mirroring pose into inputs, split `RobotState.h`→`types/Inputs.h`, retire the "RobotState" name, delete old `I*` headers, finalize REPLAY stub.

---

## Key technical decisions (baked in)

- **Secondary-capability discovery is RTTI-free** — query a motor's position capability via `virtual IPositionMotor* asPositionMotor() { return nullptr; }` accessors (firmware likely `-fno-rtti`), not `dynamic_cast`.
- **`RobotConfig&` stays an impl member** (read live); only removed from public read *signatures*. Not a guardrail violation (neutral value type).
- **`OtosPose`/`OtosVelocity` → `Pose2D`/`BodyTwist` with `using` aliases** to avoid TLM/test churn.
- **`PhysicalStateEstimate`** is the belief's name (the estimate dual of `PhysicsWorld`); the legacy "RobotState" blob name is retired, not reused.
- **One central `PhysicsWorld`** (diff-drive is one coupled body; no federation). Default **kinematic** plant (PWM→velocity map; the real PI+FF still runs above it); motor lag/momentum is an opt-in dynamics-error layer.
- **REPLAY** = stub now (`RobotMode::REPLAY` + empty `ReplayHAL`); implement later.

---

## What does NOT apply (and the substitute)

CommandScheduler/SubsystemBase → the cooperative `loopTickOnce` + `CommandQueue` · AdvantageKit/DogLog/`@AutoLog` → TLM-frame logging + REPLAY mode · PathPlanner/Choreo/swerve/`SwerveDrivePoseEstimator` → diff-drive + hand-rolled EKF · vendor-CAN confinement → **no CODAL/`MicroBit.h`/I2C types above `io/`** · multi-mechanism interlocks/state-graph → thin `switch` Superstructure · `TimeInterpolatableBuffer` latency fusion → discrete radio `SI` re-anchor + EKF Mahalanobis gating · SysId → the existing `tests/calibrate/` → `tovez.json` workflow · `NoXxx` null-object → a `GripperIONull` (gripper is already optional, `has_gripper=false`).

---

## Verification

- **Required every phase — simulation tier (the gate):** `python3 build.py --clean` (REAL + SIM) → run the **simulation** tests (`make_target("sim")`; ⊇ today's 1954) all green, plus the canaries (config field-pin, golden-TLM, vendor grep gate). No phase advances on a red simulation tier.
- **Runnable now — bench tier (robot is on the bench):** flash → run the **bench** tests (`make_target("bench")`, odometry simulated, motors/I2C/sensors real): `VER` matches, encoders count, drive-on-stand + sensor reads behave. Run these each phase where they apply.
- **Deferred — field + system tier (no playfield access now):** the **field** tests (`make_target("production")` — full navigation, GOTO-a-tag, tours against the camera) and any whole-robot system tests **cannot run now**; each phase records them as pending, to run once the robot is back on the playfield.
- **Plant correctness (Phase B):** new isolation tests per the doc's matrix — plant-only (true pose reaches target), observation-only (`setTruePose(known)` → sensor returns truth ± error; verify drift/dropout/frozen-encoder), estimator-only (`estimation_error() < TOL`; bad OTOS rejected), whole-robot (final *true* pose within tolerance of a D/G/TURN plan).
- **Behavior-preservation fences (must stay green untouched):** `test_033_005_wedge_hardening.py`, `test_goto_bounds.py`, `test_incident_scenarios.py`, `test_ekf*.py`, `test_otos_fusion.py`, `test_watchdog_exemption.py`, `sim_field_profile` (slip).
- **Final:** vendor-confinement grep returns zero hits above `source/io/`; the four-file device quartet exists per capability; the three seams are findable; a sim log can be re-fed in REPLAY mode (stub exercised).
