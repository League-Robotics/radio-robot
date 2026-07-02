---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 066 Use Cases

This sprint closes out review findings (CR-07..15) rather than adding
stakeholder-visible features. The use cases below describe the *engineering*
scenarios each ticket must satisfy — the "actor" is a developer or an
automated test relying on the simulator/tooling behaving correctly, not an
end operator of the robot. No parent `UC-` exists for these (they are
internal-quality use cases, matching the pattern of prior review-closure
sprints 064/065).

## SUC-001: Sim OTOS disagrees with encoders under slip, exactly like hardware

- **Actor**: A developer writing or debugging EKF/fusion logic against the
  simulator.
- **Preconditions**: `sim_set_motor_slip()` configures a chassis-truth slip
  in the effective range; `sim.enable_otos_model()` and
  `sim.set_otos_fusion(True)` are active.
- **Main Flow**:
  1. Command a turn or drive that engages the configured slip.
  2. Read `sim.get_enc_pose()`, `sim.get_otos_pose()`, and `sim.get_true_pose()`.
  3. Observe that the OTOS pose tracks true pose while the encoder pose
     diverges from it by the slip amount — the same disagreement pattern a
     bench operator sees on real hardware.
- **Postconditions**: The fused estimate (with fusion enabled) tracks the
  OTOS reading, not the encoder-only accumulator.
- **Acceptance Criteria**:
  - [ ] A sim test demonstrates encoder/OTOS disagreement under configured
        slip (previously impossible — the sim OTOS re-integrated the same
        unslipped truth the encoders did).
  - [ ] With zero slip/noise configured, OTOS pose still equals plant truth
        (regression guard for every existing `set_perfect()` test).

## SUC-002: Sim OTOS lever-arm compensation is exercised end to end

- **Actor**: A developer verifying that a lever-arm regression (like
  `db11b7c`'s 433 mm phantom translation on a pure spin) would be caught in
  CI before reaching hardware.
- **Preconditions**: `odomOffX`/`odomOffY` configured to a nonzero mounting
  offset; sim OTOS model enabled.
- **Main Flow**:
  1. Command a pure in-place spin (no translation).
  2. Read `sim.get_otos_pose()`.
  3. Observe the reported robot-centre translation stays ≈ 0 — the same
     compensation math the real driver runs (`OtosLeverArm.h`, shared by
     both) is exercised by the sim, not bypassed.
- **Postconditions**: A future edit that breaks the shared compensation
  formula fails this sim test before it could reach a bench.
- **Acceptance Criteria**:
  - [ ] New sim test: pure spin → OTOS-derived centre translation ≈ 0.
  - [ ] `OtosSensor` (real) and `SimOdometer` (sim) call the same
        `OtosLeverArm.h` functions — verified by code inspection / no
        duplicated formula remains in `OtosSensor.cpp`.

## SUC-003: A future `PlannerCommand` TIME stop uses a real baseline, not boot time

- **Actor**: A developer finishing the `BusDrain` `PLANNER` verb encoding in
  a future sprint.
- **Preconditions**: `Planner::apply()` is called with a real `now_ms`.
- **Main Flow**:
  1. Stage a TIMED or DISTANCE goal via `apply(cmd, now_ms)`.
  2. Tick the planner forward.
  3. Observe the TIME stop fires `duration_ms` after `now_ms`, not on the
     very next tick.
- **Postconditions**: No "instant TIME-stop" landmine for the next developer
  to inherit silently.
- **Acceptance Criteria**:
  - [ ] Guard test: a `PlannerCommand`-path timed motion runs its full
        duration before the TIME stop fires.

## SUC-004: `OdomTracker`'s world-frame convention matches the camera's world frame

- **Actor**: A developer or bench script consuming `OdomTracker.world_pos`/
  `world_yaw` alongside aprilcam ground truth.
- **Preconditions**: `OdomTracker` anchored at a known pose.
- **Main Flow**:
  1. Feed a synthetic straight-ahead TLM track.
  2. Compare `OdomTracker`'s reported world position/yaw against the
     expected aprilcam-frame (A1-centred, +x east, +y north) coordinates.
- **Postconditions**: The convention is proven correct by test, not assumed.
- **Acceptance Criteria**:
  - [ ] Convention test exists and passes.

## SUC-005: Two sequential simulator instances never corrupt each other's clock

- **Actor**: `SimTransport`'s tick-thread lifecycle (reconnect racing a
  slow-exiting prior thread) and any pytest fixture that creates multiple
  `Sim()` instances across test functions.
- **Preconditions**: A `SimHandle` (or `Sim()`) exists on one OS thread while
  a second is created on another.
- **Main Flow**:
  1. Thread A's `SimHandle` is mid-tick (watchdog/TIME-stop deltas in
     flight).
  2. Thread B calls `sim_create()`, resetting its own clock to 0.
  3. Thread A's clock and in-flight deltas are unaffected.
- **Postconditions**: No cross-instance clock corruption; any accidental
  same-`SimHandle` cross-thread call fails loudly (assert) instead of racing
  silently.
- **Acceptance Criteria**:
  - [ ] Two sequential/concurrent `Sim()` instances on different threads do
        not interact.
  - [ ] A deliberately-cross-thread call against one `SimHandle` asserts.

## SUC-006: `GET CFG` (and any long sim reply) returns complete output

- **Actor**: A developer or test reading a long synchronous reply through
  `SimConnection`.
- **Preconditions**: A command whose reply exceeds 512 bytes (e.g. `GET CFG`).
- **Main Flow**: Send the command; read the full reply.
- **Postconditions**: The reply is not silently truncated.
- **Acceptance Criteria**:
  - [ ] `GET CFG` via `SimConnection` returns the complete, untruncated
        output.

## SUC-007: CR-15 batch — eight small correctness/hygiene fixes land cleanly

- **Actor**: Developers relying on each of: bounded plant heading, device
  discovery over the current relay firmware, visible relay channel/group
  mismatches, an accurate `SimTransport` connection-state flag, correct
  encoder-trace heading integration, no wasted stop-condition slots, a
  sensibly-located color-conversion helper, and correct multi-key release
  behavior while driving.
- **Acceptance Criteria**: see `sprint.md`'s CR-15 item list; each item's
  fix or verification is independently checkable.
  - [ ] All eight items resolved or verified (two verify-only, per
        `architecture-update.md`).

## SUC-008: TestGUI encoder trace survives a reset on slow (relay) telemetry

- **Actor**: An operator driving a scripted tour or sequence of `D` commands
  over the radio relay, watching the TestGUI's encoder trace.
- **Preconditions**: TLM arriving at 1-2 Hz; the robot moves 100-200 mm
  between frames.
- **Main Flow**:
  1. The GUI sends a `D` command (which the firmware answers by zeroing
     encoders).
  2. The next TLM frame arrives well past the old 20 mm reset-detection
     epsilon.
  3. The encoder trace rebaselines correctly (no spurious reverse motion,
     no cancelled heading) because the GUI signalled the reset at
     command-send time, not because the frame's magnitude was small.
- **Postconditions**: The encoder trace continues to follow the robot's
  actual turns after the reset.
- **Acceptance Criteria**:
  - [ ] test_traces: a delayed-TLM reset scenario (first post-reset frame at
        ~150 mm) preserves accumulated heading.

## SUC-009: TestGUI otos/fused traces stay aligned after a mid-session anchor

- **Actor**: An operator who anchors the TestGUI to a camera fix partway
  through a session (firmware heading nonzero at that moment).
- **Preconditions**: `hdg_cdeg` captured in the otos/fused baseline tuples at
  anchor time.
- **Main Flow**:
  1. Anchor mid-session with a nonzero firmware heading.
  2. Drive; observe the otos/fused traces.
- **Postconditions**: The otos/fused traces align with the camera trace,
  instead of appearing rotated by the stale firmware heading.
- **Acceptance Criteria**:
  - [ ] test_traces: anchor with non-zero firmware heading — otos/fused
        traces align with the camera trace.
