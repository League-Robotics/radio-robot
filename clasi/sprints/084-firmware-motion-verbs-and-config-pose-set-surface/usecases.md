---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 084 Use Cases

Sprint-level use cases (SUC-NNN) for the firmware motion-verb and
config/pose-set surface. These are firmware-wire-contract use cases: the
"user" is the host (TestGUI, `robot_radio`, or a bench script) issuing
protocol-v2 verbs over serial/radio, not a human directly.

## SUC-001: Open-loop and bounded velocity driving

- **Actor**: Host (TestGUI command row, `robot_radio`, a bench script).
- **Preconditions**: Firmware booted with a bound drive-pair
  (`DrivetrainConfig.left_port`/`right_port`), `Subsystems::Planner`
  idle.
- **Main Flow**:
  1. Host sends `S <l> <r>`, `T <l> <r> <ms>`, or `D <l> <r> <mm>`,
     optionally with one or more `stop=<kind>:<args>` clauses.
  2. Firmware acks synchronously (`OK drive ...`) and begins driving.
  3. Firmware completes the drive when its built-in stop condition (or an
     early `stop=` clause) fires, and emits `EVT done <verb> reason=<token>`.
  4. Host may send `STOP` at any time to cancel immediately (no `EVT`).
- **Postconditions**: Robot is stopped or driving exactly one bounded/
  streaming motion at a time; `Subsystems::Planner::state()` reflects it.
- **Acceptance Criteria**:
  - [ ] `D 200 200 500` moves true pose ~500 mm and emits
        `EVT done D reason=dist`.
  - [ ] `T`/`S` behave per `docs/protocol-v2.md` §10 (duration/streaming
        watchdog semantics).
  - [ ] A `stop=` clause fires before the built-in stop when satisfied
        first, with the correct `reason=` token.
  - [ ] `STOP` halts immediately with no `EVT`.

## SUC-002: Arc and turn-in-place driving

- **Actor**: Host.
- **Preconditions**: Same as SUC-001; `Subsystems::PoseEstimator` supplies
  a fused heading for the closed-loop verbs.
- **Main Flow**:
  1. Host sends `R <speed> <radius>`, `TURN <heading> [eps=<cdeg>]`, or
     `RT <relAngle>`, optionally with `stop=` clauses.
  2. Firmware acks synchronously and begins the arc/turn.
  3. `TURN`/`RT` close the loop against `PoseEstimator::fusedPose()`'s
     heading and complete within tolerance; `R` runs open-loop (like `S`)
     until stopped or a `stop=` clause fires.
  4. Firmware emits `EVT done <verb> reason=<token>` on completion.
- **Postconditions**: Robot heading matches the commanded target within
  `eps`/tolerance for `TURN`/`RT`; `R` behaves as a constant-curvature
  open-loop drive.
- **Acceptance Criteria**:
  - [ ] `RT 9000` rotates ~90° (within plant tolerance) and emits
        `EVT done RT` with a heading/rotation `reason=` token.
  - [ ] `TURN <heading>` reaches the commanded absolute heading within
        `eps`.
  - [ ] `R <speed> <radius>`'s realized arc curvature matches
        `speed`/`radius` (within plant tolerance).

## SUC-003: Go-to XY navigation

- **Actor**: Host.
- **Preconditions**: Same as SUC-002 (needs fused pose, not just heading).
- **Main Flow**:
  1. Host sends `G <x> <y> <speed>`.
  2. Firmware acks synchronously, optionally pre-rotates in place when the
     bearing error exceeds the turn-in-place gate, then pursues the
     target.
  3. Firmware completes when within `arrive_tol` mm of the goal and emits
     `EVT done G`.
- **Postconditions**: Robot is within `arrive_tol` mm of the commanded
  relative XY point.
- **Acceptance Criteria**:
  - [ ] `G 300 0 200` drives to the relative point and emits
        `EVT done G`.
  - [ ] Pre-rotate engages only when the initial bearing error exceeds
        the configured `turn_in_place_gate`.

## SUC-004: Motion-state observability via `mode=`

- **Actor**: Host (TestGUI's tour runner and command-completion
  detection).
- **Preconditions**: `STREAM`/`SNAP` already implemented (sprint 082).
- **Main Flow**:
  1. Host issues a motion verb, then polls `SNAP` or watches the
     `STREAM`.
  2. `mode=` reflects the active verb family (`I`=idle, `S`=streaming,
     `T`=timed, `D`=distance, `G`=go-to, ... per
     `docs/protocol-v2.md` §8) throughout the drive.
  3. `mode=` returns to `I` at completion, independent of whether the
     corresponding `EVT done` line was received.
- **Postconditions**: A host can infer motion-complete from `mode=I`
  alone, without depending on `EVT` delivery (which the relay transport
  can drop).
- **Acceptance Criteria**:
  - [ ] `mode=` is `I` if and only if `Subsystems::Planner` reports no
        active command.
  - [ ] Every other value corresponds to the currently-active verb
        family per the sprint's documented mode-mapping decision.

## SUC-005: Live calibration via `SET`/`GET`

- **Actor**: Operator or TestGUI's calibration-push sequence.
- **Preconditions**: Firmware booted with boot-config defaults from
  `Config::defaultDrivetrainConfig()`/`defaultMotorConfigs()`.
- **Main Flow**:
  1. Host sends `SET <key>=<value>...`.
  2. Firmware validates all keys atomically; on success, re-propagates
     the change into `Subsystems::Drivetrain::configure()` and/or
     `Subsystems::PoseEstimator::configure()` and replies `OK set ...`.
  3. Host sends `GET [<key>...]` and receives the current values,
     confirming the round-trip.
- **Postconditions**: Live drivetrain/pose-estimation behavior reflects
  the newly-`SET` values with no reflash.
- **Acceptance Criteria**:
  - [ ] `SET tw=130` then `GET tw` round-trips to `130` and visibly
        changes arc/turn geometry.
  - [ ] An unknown key yields `ERR badkey <key>`.
  - [ ] An out-of-range value yields `ERR badval <key>=<value>` with no
        partial application (atomic SET).

## SUC-006: Pose synchronization (`SI` / `ZERO enc`)

- **Actor**: Operator or TestGUI's Sync-Pose / Set-Origin / Zero-Encoders
  actions.
- **Preconditions**: `Subsystems::PoseEstimator` running (sprint 082).
- **Main Flow**:
  1. Host sends `SI <x> <y> <h>` (mm, mm, centi-degrees).
  2. Firmware teleports both `PoseEstimator::encoderPose()` and
     `PoseEstimator::fusedPose()` to the given pose and acks.
  3. Host sends `ZERO enc`; firmware rezeroes the bound pair's hardware
     encoders and `PoseEstimator`'s encoder-baseline accumulator
     together, so the next tick's delta is not a phantom jump.
- **Postconditions**: `pose=`/`encpose=` read back at the commanded
  origin with no discontinuity on the following tick.
- **Acceptance Criteria**:
  - [ ] `SI 1000 500 900` makes the next `SNAP`'s `pose=`/`encpose=`
        read back at (1000, 500, 900).
  - [ ] `ZERO enc` rezeroes `enc=`/`encpose=` to (0,0,0)-relative with no
        phantom-jump discontinuity on the following tick.

## SUC-007: OTOS tuning surface (sim-verified)

- **Actor**: Operator or TestGUI, against the simulator (no real OTOS
  driver this program).
- **Preconditions**: `Hal::SimOdometer` exists (sprint 081);
  `Subsystems::Hardware::odometer()` seam exists (sprint 082).
- **Main Flow**:
  1. Host sends `OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`.
  2. Against the sim: each verb acts on `Hal::SimOdometer` (via the
     shared `Hal::Odometer` interface) and replies `OK ...`.
  3. Against real hardware (`Subsystems::NezhaHardware`, whose
     `odometer()` returns `nullptr`): each verb replies
     `ERR nodev <verb>` with no crash.
- **Postconditions**: The OTOS verb family is fully exercised in sim;
  real hardware fails safely, never crashes.
- **Acceptance Criteria**:
  - [ ] All seven verbs (`OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`) ack against
        the sim.
  - [ ] All seven return `ERR nodev` on hardware (`Subsystems::
        NezhaHardware`).
  - [ ] No verb crashes the firmware regardless of which hardware owner
        is active.

## SUC-008: Hardware bench verification

- **Actor**: The team-lead / a bench operator, per
  `.claude/rules/hardware-bench-testing.md`.
- **Preconditions**: Robot mounted on the stand, wheels off the ground;
  sprint firmware deployed via `mbdeploy`.
- **Main Flow**:
  1. Deploy the sprint's firmware to the robot.
  2. Exercise `D`/`T`/`R`/`TURN`/`RT`/`G`/`S`/`STOP` and confirm the
     wheels drive and encoders increment proportionally in the expected
     direction.
  3. Exercise `SET`/`GET`/`SI`/`ZERO enc` and confirm they take visible
     effect; exercise OTOS verbs and confirm `ERR nodev` (no real OTOS
     driver this program).
  4. Confirm round-trip command/reply behavior over the real serial
     link.
- **Postconditions**: The full motion and config/pose-set surface is
  confirmed working on physical hardware, not only under the host-side
  simulator.
- **Acceptance Criteria**:
  - [ ] Every item above is observed working on the stand.
  - [ ] A sprint is not marked done on sim tests alone (per
        `.claude/rules/hardware-bench-testing.md`).
