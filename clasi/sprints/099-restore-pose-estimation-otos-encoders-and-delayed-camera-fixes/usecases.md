---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 099 Use Cases

## SUC-001: OTOS Ticks Live Without Disturbing Motion Timing
Parent: UC-012 (Initialize and Read OTOS Sensor)

- **Actor**: Firmware (internal — no host command triggers this; it is a
  standing runtime property)
- **Preconditions**: Robot is powered, `NezhaHardware::begin()` has run
  (OTOS product-ID detected or not), motion commands may be arriving
  concurrently over the wire.
- **Main Flow**:
  1. Each `NezhaHardware::tick()` call checks whether the OTOS's own 20ms
     read cadence is due AND no Nezha-motor 0x46 request is outstanding.
  2. If so, this call services the OTOS (one I2C burst) instead of the
     motor flip-flop's own next scheduled action.
  3. Otherwise, the motor flip-flop proceeds exactly as before this
     sprint.
  4. The freshly-read OTOS sample and connection state are committed onto
     the Blackboard every pass.
- **Postconditions**: `bb.otos`/`bb.otosValid`/`bb.otosConnected` reflect
  a live-updating OTOS reading (or a truthful "not connected"). No I2C bus
  hang occurs, ever, regardless of what motion commands arrive
  concurrently.
- **Acceptance Criteria**:
  - [ ] A sustained (>=10 minute) bench session with 0x17 (OTOS) and 0x10
        (Nezha motor) traffic interleaved shows zero bus hangs.
  - [ ] The SAME session, with motion commands (`S`/`MOVE`-equivalent
        binary `drive`/`segment`) running throughout, shows no
        motion-timing regression versus a pre-sprint baseline (the
        098-004 hazard class does not reproduce).
  - [ ] `otosconn=` (via `TLM`) reports `true` when a real chip is
        detected, `false` otherwise, refreshed live.

## SUC-002: Encoder-Only Pose Tracks Live, Re-Anchors on SI/ZERO
Parent: UC-006 (Query and Zero Dead-Reckoning Odometry)

- **Actor**: Python host (via a binary `PoseFix` command), firmware
  (internal dead-reckoning)
- **Preconditions**: `PoseEstimator` is ticked by `MainLoop` every pass.
- **Main Flow**:
  1. As the robot's wheels turn, `PoseEstimator` integrates encoder deltas
     into `encoderPose()` every tick.
  2. A host sends a binary `PoseFix{x, y, h, reset=true}` (the SI
     equivalent) — `encoderPose()`/`fusedPose()` immediately re-anchor to
     the given world pose; the encoder-delta baseline is untouched (wheel
     tracking continues uninterrupted from wherever the wheels physically
     are).
  3. A host sends `PoseFix{zero_encoders=true}` (the ZERO equivalent) —
     the encoder-delta baseline re-synchronizes to the CURRENT encoder
     reading; the believed pose (`encoderPose()`/`fusedPose()`) is
     untouched.
- **Postconditions**: `bb.encoderPose` (and, once fusion lands, `bb.
  fusedPose`) tracks real wheel motion on the stand; a reset/zero applies
  exactly once, immediately, with no phantom jump.
- **Acceptance Criteria**:
  - [ ] On the stand, driving the wheels for a known number of
        revolutions produces an `encpose`-equivalent (`pose=`/`enc=` on
        TLM) displacement consistent with the wheel geometry.
  - [ ] `PoseFix{reset=true}` re-anchors `pose=` to the given (x,y,h)
        within one tick; a subsequent SI does not fabricate a jump.
  - [ ] `PoseFix{zero_encoders=true}` does not move `pose=`/`otos=`; only
        the internal delta baseline changes (observable indirectly: no
        pose jump on the next tick that follows a genuinely time-advancing
        pass).

## SUC-003: Fused Pose Does Not Freeze or Drag When OTOS Disagrees
Parent: UC-006, UC-012

- **Actor**: Firmware (internal fusion)
- **Preconditions**: OTOS fusion is enabled (`otosObs` is passed to
  `PoseEstimator::tick()`, not `nullptr`); the innovation gate exists.
- **Main Flow**:
  1. Each tick, `EkfTiny::predict()` advances the fused belief from
     encoder deltas unconditionally.
  2. When a fresh OTOS reading is present, `EkfTiny::updatePosition()`/
     `updateHeading()` run — but a reading whose innovation exceeds the
     bounded gate is rejected (no state change) rather than blindly
     applied.
  3. A run of consecutive rejections inflates the gate's own covariance so
     a genuinely-shifted (not just momentarily-noisy) OTOS reading is
     eventually re-trusted.
- **Postconditions**: `bb.fusedPose` tracks the robot's true motion even
  when the OTOS is momentarily static/disagreeing (e.g. wheels-off-ground
  bench conditions, per the frozen-fused-pose hazard this closes) — it
  never freezes at a stale value nor snaps to the OTOS's disagreeing
  reading.
- **Acceptance Criteria**:
  - [ ] On the stand (wheels off the ground — OTOS reports near-zero
        translation while encoders accumulate), `fusedPose` does not
        freeze at the OTOS's static reading nor drag toward the origin;
        it tracks the encoder-driven belief (documented expected
        divergence — see sprint.md's bench-gate note).
  - [ ] The sim/unit `ekf_tiny_harness.cpp` characterizes accept/reject/
        streak-recovery behavior against known innovation values, not
        freehand-tuned on the bench.

## SUC-004: Per-Motor Acceleration Is Observable
Parent: UC-005 (Query Encoder Positions)

- **Actor**: Firmware (internal), any consumer reading `MotorState`
- **Preconditions**: A motor is polled (encoder-sampled) each of its own
  flip-flop slots.
- **Main Flow**:
  1. Each time a motor's `tick()` refreshes its filtered `velocity()`, the
     `Hal::Motor` base computes an EMA-filtered acceleration from that
     velocity and the tick's own elapsed time.
  2. `MotorState.acceleration` carries this value on every state read.
- **Postconditions**: `bb.motors[i].acceleration` reflects a plausible,
  changing value while the motor accelerates/decelerates, for all 4
  ports (not only the bound drive pair).
- **Acceptance Criteria**:
  - [ ] On a bench ramp (duty step from 0 to a nonzero value), `bb.
        motors[i].acceleration` rises then settles toward zero as
        velocity plateaus — plausible sign and magnitude.
  - [ ] `bb.drivetrain.acc_left`/`acc_right` (the existing, separate
        Drivetrain-level acceleration) are unchanged by this addition —
        no TLM regression.

## SUC-005: Delayed Camera-Fix Corrects the Fused Pose
Parent: UC-007 (Set Odometry from External Source)

- **Actor**: A camera-equipped host (aprilcam), Python host (clock-sync
  via `PING`)
- **Preconditions**: `PoseEstimator`'s pose-history ring is populated
  (the robot has been ticking for at least a few hundred ms);
  `ekf_r_fix_xy`/`ekf_r_fix_theta` are configured (zero-as-unset sentinel,
  matching the existing four EKF fields).
- **Main Flow**:
  1. Host establishes clock sync with the robot via binary `ping`
     (`Ack.t`).
  2. Host observes the robot's true pose at some past robot-clock time
     `T` (a camera frame captured and processed with latency).
  3. Host sends `PoseFix{x, y, h, t=T}` (reset=false, zero_encoders=false).
  4. `PoseEstimator` interpolates its own dead-reckoning history at `T`,
     rigid-composes the camera's `T`-time observation forward to "now"
     using the (exact, world-frame) encoder delta since `T`, and applies
     the result as an ungated EKF position+heading update.
  5. The applied step is recorded (`bb.poseStepped`) and the corrected
     fused pose is posted so the OTOS chip's own frame is re-anchored to
     match (`otosSetPoseOut`).
- **Postconditions**: `bb.fusedPose` converges by the correctly-composed
  amount; `bb.encoderPose` is untouched; a stale-timestamped fix (older
  than the history ring) produces no jump (dropped, counted).
- **Acceptance Criteria**:
  - [ ] Sim: drive, send a fix with a known offset at a captured robot
        time, assert `fusedPose` converges by the composed amount while
        `encoderPose` stays untouched.
  - [ ] Sim: a fix with `t` older than the ring's oldest entry produces no
        jump (a counted drop, not a crash or a garbage compose).
  - [ ] Bench: a `PoseFix` is accepted (`OK`, not `ERR`) and `pose=`
        visibly converges toward the sent value.
  - [ ] Playfield: the aprilcam end-to-end script demonstrates the full
        path (PING clock-sync -> tag-pose-to-FIX send -> convergence
        check) on the real robot.

## SUC-006: BodyState Is Published for the Next Motion Controller
Parent: UC-006, UC-015 (Drive to Relative XY Position — the future
consumer's own use case)

- **Actor**: Firmware (internal); the (not-yet-built) motion-v2 adapter is
  this cell's one intended consumer.
- **Preconditions**: `PoseEstimator` is ticked; wheel velocities are
  available.
- **Main Flow**:
  1. Each pass, `MainLoop` computes body twist (v, omega) from the two
     directly-read wheel velocities via `BodyKinematics::forward()`.
  2. `bb.bodyState` is committed: fused pose + this twist, one
     authoritative cell.
- **Postconditions**: `bb.bodyState` is always current, in-process, ready
  for a future consumer to read without re-deriving the kinematic
  transform itself.
- **Acceptance Criteria**:
  - [ ] `bb.bodyState.pose` matches `bb.fusedPose.pose` every pass.
  - [ ] `bb.bodyState.twist` matches the same `v`/`omega` TLM's `twist=`
        already reports (same inputs, same trackwidth, same transform) —
        no divergence between the two.
  - [ ] Not on the wire this sprint (verified by grep: no `Telemetry`
        field references `bodyState`).

## SUC-007: PoseStepped Distinguishes a Jump From Drift
Parent: UC-007

- **Actor**: Firmware (internal); the future motion-v2 adapter is this
  cell's one intended consumer (`StepInput.poseStep`).
- **Preconditions**: A `PoseFix{reset=true}` or a delayed fix has just
  been applied.
- **Main Flow**:
  1. `PoseEstimator` records the magnitude of the correction it just
     applied (`‖Δp‖`, `|Δθ|`) for exactly the tick it was applied on.
  2. `MainLoop` commits this to `bb.poseStepped` every pass (zero on every
     tick nothing was applied).
- **Postconditions**: A consumer reading `bb.poseStepped` on the SAME
  tick a correction landed can distinguish "the estimate just jumped
  because of a correction" from "the estimate is smoothly diverging" —
  both of which look identical as a raw pose delta without this signal.
- **Acceptance Criteria**:
  - [ ] `bb.poseStepped` is nonzero on exactly the tick a `reset=true` or
        delayed fix is applied, and zero on every other tick (including
        the tick immediately after).
  - [ ] Sim: `pose_estimator_harness.cpp` asserts the reported magnitude
        matches a hand-computed expectation for a known correction.
