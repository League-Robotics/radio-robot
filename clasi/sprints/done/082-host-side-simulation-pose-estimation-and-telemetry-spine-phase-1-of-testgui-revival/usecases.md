---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 082 Use Cases

`docs/usecases.md`'s UC-006 (query/zero dead-reckoning odometry) and UC-012
(initialize/read OTOS) are the closest production-catalog analogues, but both
describe the OLD `source_old/` wire surface (`SO`/`SZ`, `O`/`OP`/`OR`) reached
through the parked `Odometry`/`OtosSensor` classes. This sprint reaches the
equivalent capability through the NEW tree's dev-bench + telemetry surface
(`TLM`/`STREAM`/`SNAP`, `Subsystems::PoseEstimator`) — referenced as "Parent"
where the underlying capability matches, not as a literal wire-compatible
continuation. Each SUC below maps 1:1 onto one of this sprint's five tickets.

## SUC-001: Fuse encoder and OTOS observations into a bounded pose estimate
Parent: UC-006, UC-012 (capability analogue — new surface)

- **Actor**: Firmware developer verifying the estimation core
- **Preconditions**: `source/hal/capability/odometer.h` declares `Hal::Odometer`
  but has no firmware consumer. `libraries/tinyekf` is vendored and already
  on the root `CMakeLists.txt` include path, but nothing in `source/` includes
  it. `msg::DrivetrainConfig` already carries `ekf_q_xy`/`ekf_q_theta`/
  `ekf_r_otos_xy`/`ekf_r_otos_theta` (and four velocity-channel fields this
  sprint does not use — see architecture-update.md Decision 2).
- **Main Flow**:
  1. Developer ports `source_old/state/EKFTiny.{h,cpp}` into a new
     `Hal::EkfTiny` (`source/estimation/ekf_tiny.{h,cpp}`), trimmed from the
     original 5-state (x, y, theta, v, omega) filter to a 3-state
     (x, y, theta) filter — no velocity-channel update, no Mahalanobis
     gating, no P-inflation gate-recovery (see architecture-update.md
     Decision 2 for the full rationale).
  2. `predict(dCenter, dTheta, thetaBefore, dt)` implements the same
     arc-segment motion model as the old class, minus the velocity
     sub-block.
  3. `updatePosition(xOtos, yOtos)` (2-observation position channel) and
     `updateHeading(thetaOtos)` (scalar heading channel) port the position/
     heading math unchanged in concept.
  4. Developer writes host-side unit tests exercising predict-only sequences
     (pure dead-reckoning drift) and predict+correct sequences (a
     deliberately-offset observation pulls the estimate toward it).
- **Postconditions**: `Hal::EkfTiny` is a pure, host-clean, CODAL-free class
  compiled and tested standalone, ready for `Subsystems::PoseEstimator`
  (SUC-002) to wrap.
- **Acceptance Criteria**:
  - [ ] `Hal::EkfTiny` compiles with no `#include "MicroBit.h"` and no I2C
        dependency; includes only `<math.h>`, `<stdint.h>`, `<tinyekf.h>`.
  - [ ] A synthetic predict-only sequence matches a hand-computed
        arc-integration reference within floating-point tolerance.
  - [ ] A synthetic predict+correct sequence demonstrably pulls the estimate
        toward a deliberately-offset position/heading observation (proves
        the correction step is live, not a no-op).

## SUC-002: Dead-reckon from encoders and correct with the odometer, gracefully degrading when no odometer exists
Parent: UC-006, UC-012 (capability analogue — new surface)

- **Actor**: Firmware developer wiring the estimator into the drivetrain's
  wheel-pair observations
- **Preconditions**: SUC-001's `Hal::EkfTiny` exists. `Hal::Motor::position()`
  already returns calibration-corrected mm (see `nezha_motor.cpp`'s
  `travel_calib`/`fwd_sign` application) — no additional unit conversion is
  needed at this layer. No real-hardware `Hal::Odometer` leaf exists yet
  (only `Hal::SimOdometer`, sprint 081).
- **Main Flow**:
  1. Developer writes `Subsystems::PoseEstimator`
     (`source/subsystems/pose_estimator.{h,cpp}`) — a new Subsystems-tier
     peer of `Subsystems::Drivetrain`, deliberately NOT folded into
     `Drivetrain` itself (see architecture-update.md Decision 1: control-law
     tuning and sensor-fusion-noise tuning change for different reasons).
  2. `configure(const msg::DrivetrainConfig&)` reads `trackwidth`,
     `rotational_slip`, and the four position/heading EKF noise fields it
     uses, applying a zero-as-unset-sentinel default fallback (matching the
     existing `effectiveSlip()` pattern in the ported `Odometry` source) so
     an un-`SET` boot config still produces a numerically sane filter.
  3. `tick(now, leftObs, rightObs, otosObs)` — `leftObs`/`rightObs` are this
     tick's `msg::MotorState` (matching `Drivetrain::tick()`'s existing
     parameter shape); `otosObs` is a nullable `const msg::PoseEstimate*`.
     Computes the encoder position delta since the last tick, midpoint-arc
     integrates it into the encoder-only accumulator (`encoderPose()`), runs
     the EKF predict step, and — only when `otosObs != nullptr` and its
     `stamp.valid` — runs the EKF correct step.
  4. When `otosObs == nullptr` throughout (the real-hardware, no-OTOS-leaf
     case), `fusedPose()` is dead-reckoning only — an honest degradation,
     not a silent wrong answer.
- **Postconditions**: `Subsystems::PoseEstimator` exposes `encoderPose()` and
  `fusedPose()` (both `msg::PoseEstimate`), ready for SUC-003's wiring and
  SUC-004's telemetry frame.
- **Acceptance Criteria**:
  - [ ] Host unit test: encoder-only (no odometer) sequence — `fusedPose()`
        equals `encoderPose()` exactly (no fusion applied when there is
        nothing to fuse).
  - [ ] Host unit test: with a synthetic `otosObs` diverging from the
        encoder-only path, `fusedPose()` differs from `encoderPose()` (proves
        the correction step actually runs when an odometer is present).
  - [ ] A `DrivetrainConfig` with all-zero EKF fields still produces a
        finite, non-degenerate `fusedPose()` after several ticks (the
        sentinel-default fallback works).

## SUC-003: Reach whichever concrete odometer the active hardware owner has, without an #ifdef
Parent: None (infrastructure prerequisite for SUC-004)

- **Actor**: Firmware developer wiring the estimator into the shared dev loop
- **Preconditions**: SUC-002's `Subsystems::PoseEstimator` exists.
  `Subsystems::Hardware` (sprint 081) has no `odometer()` accessor —
  `Subsystems::SimHardware` owns a concrete `Hal::SimOdometer` reachable only
  through its own concrete type, never through the abstract base.
  `Subsystems::NezhaHardware` has no odometer at all.
- **Main Flow**:
  1. Developer adds `virtual Hal::Odometer* odometer() { return nullptr; }`
     to `Subsystems::Hardware` (a defaulted, non-pure virtual — see
     architecture-update.md Decision 3 for why this is a default-body
     virtual, not a second pure interface method every owner must implement).
  2. `Subsystems::SimHardware` overrides it to return `&odometer_`.
     `Subsystems::NezhaHardware` is untouched — it inherits the `nullptr`
     default.
  3. `source/dev_loop.{h,cpp}` gains a `Subsystems::PoseEstimator*` field on
     `DevLoop`, and a new step in `devLoopTick()`: after the second
     `hardware.tick(now)` slice (freshest encoder reads), read
     `hardware.odometer()`, tick it if non-null, sample its `pose()`, and
     call `poseEstimator->tick(now, leftObs, rightObs, otosObsOrNullptr)`
     exactly once per pass (never inside the two-slice hardware-tick
     duplication — see the double-integration hazard sprint 081's
     `SimHardware` dt=0 guard already documents for the identical class of
     bug).
  4. `source/main.cpp` constructs the `PoseEstimator`, configures it from the
     same `msg::DrivetrainConfig` already built for `Drivetrain::configure()`,
     and wires it into `DevLoop`.
- **Postconditions**: Every pass of the shared dev-loop body (ARM and host
  sim alike) advances the pose estimate exactly once, from whichever
  concrete `Hardware` owner is active.
- **Acceptance Criteria**:
  - [ ] `Subsystems::NezhaHardware` requires no source change; it compiles
        and links unchanged, inheriting the `nullptr` default.
  - [ ] A standalone harness (matching `tests/sim/unit/*_harness.cpp`'s
        ad hoc-compile convention) proves `poseEstimator->tick()` is called
        exactly once per `devLoopTick()` pass, not twice.
  - [ ] Hardware bench smoke (`.claude/rules/hardware-bench-testing.md`):
        ARM build behavior is otherwise unchanged — `PING`/`DEV` family
        round-trip exactly as before this ticket.

## SUC-004: Stream and snapshot a telemetry frame carrying pose, encoder, and velocity fields
Parent: None (the sprint's headline deliverable — no wire-level parent;
`docs/protocol-v2.md` §8 is the target field vocabulary, not yet implemented
in the new tree)

- **Actor**: Host/sim developer (the eventual TestGUI consumer, sprint 084)
- **Preconditions**: SUC-003's estimator wiring exists. The new `source/`
  tree has no `TLM`/`STREAM`/`SNAP` verbs at all.
- **Main Flow**:
  1. Developer writes `source/telemetry/tlm_frame.{h,cpp}` — a pure
     frame-formatting function taking `now`, `mode`, `seq`, the bound pair's
     `enc`/`vel` readings, and the estimator's `pose`/`encpose`/`otos`/`twist`
     values (each independently omittable when its source is absent),
     ported from `source_old/robot/RobotTelemetry.cpp`'s formatting logic.
  2. Developer writes `source/commands/telemetry_commands.{h,cpp}`
     registering `STREAM <ms>` (sets period; 0 disables) and `SNAP` (one
     synchronous frame), sharing one `seq` counter between them, and
     capturing the issuing statement's `replyFn`/`replyCtx` as the bound
     telemetry channel (D10-style channel binding) at `STREAM`-command time.
  3. `devLoopTick()` gains a periodic-emission step: when a period is set and
     enough time has elapsed since the last emission, format and send a
     frame on the bound channel.
  4. `mode=` reports `I` when `!drivetrain.active()` and `S` when active
     (mirroring the `S`/`VW` mode character `docs/protocol-v2.md` already
     defines) — no other mode character is emitted this sprint.
  5. `main.cpp` concatenates `telemetryCommands()` into the command table
     alongside `systemCommands()`/`devCommands()`.
- **Postconditions**: `STREAM`/`SNAP` produce `TLM` frames identically from
  the ARM firmware and the sprint-081 host sim.
- **Acceptance Criteria**:
  - [ ] `SNAP` returns one well-formed `TLM` line synchronously with all
        applicable fields present.
  - [ ] `STREAM <ms>` (clamped to a 20 ms floor, matching
        `docs/protocol-v2.md`'s existing documented minimum) emits frames at
        the configured period; `STREAM 0` disables it.
  - [ ] `STREAM` and `SNAP` share one monotonically increasing `seq=` counter.
  - [ ] `otos=` and the fused-EKF-dependent portion of `pose=` are omitted
        (not zero-filled) when `hardware.odometer() == nullptr`.
  - [ ] `mode=` is `I` at rest and `S` during an active `DEV DT VW`/`WHEELS`
        drive.

## SUC-005: Verify the estimate against ground truth in sim, and the sensors on the stand
Parent: None (verification closes out the sprint)

- **Actor**: Any developer running the sim test suite; a bench operator on
  the stand
- **Preconditions**: SUC-001 through SUC-004 are implemented. Sprint 081's
  `libfirmware_host`/`tests/_infra/sim/` harness exists and exposes
  `SimOdometer`'s error knobs and the plant's ground-truth pose reads.
- **Main Flow**:
  1. Developer writes sim tests asserting `pose=`/`encpose=` track the
     ctypes ground-truth pose within the plant's tolerance over a drive
     sequence.
  2. Developer writes a sim test that sets `SimOdometer`'s error knobs
     (noise/scale/drift), confirms `otos=` diverges from ground truth by
     roughly the configured amount, then zeroes the knobs and confirms
     `otos=` re-converges to (matches) ground truth.
  3. Developer writes a `STREAM`/`SNAP` shape test (all documented fields
     present/absent per SUC-004's omission rules; shared `seq=`).
  4. Bench operator deploys to the robot (`mbdeploy deploy --build`) and runs
     the hardware bench gate: encoders alive and incrementing proportionally
     to `DEV DT VW`/`WHEELS` commands, visible and correct over `TLM`'s
     `enc=`/`encpose=`; round-trip over serial.
  5. Bench operator explicitly records that the OTOS-specific bench checks
     (sensor alive, changing values) are **not satisfiable** this sprint —
     no real `Hal::Odometer` leaf exists in the new tree — rather than
     silently skipping them.
- **Postconditions**: The sprint's estimation and telemetry surface is
  verified against ground truth in sim and against real encoders on the
  stand; the OTOS hardware gap is a recorded, known limitation, not a
  silent hole in the report.
- **Acceptance Criteria**:
  - [ ] Sim: `pose=`/`encpose=` tolerance test passes.
  - [ ] Sim: `otos=` divergence/reconvergence test passes.
  - [ ] Sim: `STREAM`/`SNAP` shape test passes.
  - [ ] Bench: encoders alive, incrementing correctly, `TLM` round-trip over
        serial — recorded with the actual command transcript, not just
        asserted.
  - [ ] Bench report explicitly states the OTOS-check gap and why (no real
        leaf yet), rather than omitting OTOS from the report silently.
