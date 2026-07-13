---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 100 Use Cases

## SUC-001: A Developer Inspects the Plan Table Before Anything Moves
Parent: UC-015 (Drive to Relative XY Position)

- **Actor**: A developer/researcher, via a Python REPL against the tier-0
  ctypes ABI (no firmware, no hardware).
- **Preconditions**: `source/drive/` compiles standalone into
  `libdrive_host`; `Drive::Drivetrain::plan()` has produced a `PlanResult`.
- **Main Flow**:
  1. The developer constructs a `Drive::Drivetrain` with a `Limits` value
     and calls `plan(PlanRequest)`.
  2. The developer samples `MotionPlan::referenceAt(t)` across
     `[0, duration()]` and dumps or plots the resulting `RefState` table
     (s, v, a, theta, omega, alpha, x, y) — entirely offline, before any
     `step()` call, before any hardware exists.
- **Postconditions**: The full reference trajectory for a segment is
  visible and plottable without a robot, a sim, or a running firmware
  image.
- **Acceptance Criteria**:
  - [ ] A tier-0 Python test dumps `referenceAt()` over a solved plan's
        duration and asserts monotonic time, feasible velocity/
        acceleration bounds, and a smooth (no discontinuity) closed-form
        `x`/`y` trace.
  - [ ] A notebook under `tests/notebooks/` plots at least one arc and one
        pivot plan table (the "interpretability deliverable at its
        purest," per the issue).

## SUC-002: The Planner and Plan Are Provably Pure Functions
Parent: UC-015

- **Actor**: The tier-0 Python test suite.
- **Preconditions**: `Drive::Drivetrain`/`Drive::MotionPlan` are compiled
  into `libdrive_host`.
- **Main Flow**:
  1. The same `(plan, StepInput, StepState)` triple is fed to `step()`
     twice; the two `StepOutput`s (and the two returned `StepState`
     values) are compared byte-for-byte.
  2. A `StepState` round-trips through the ctypes struct boundary
     unchanged when nothing in the tick would alter it.
  3. Fuzzed `StepInput`/`Limits` combinations (including degenerate ones —
     zero trackwidth avoided, near-zero speeds, boundary curvature) never
     produce `NaN`/`Inf` in any output field.
- **Postconditions**: Any tick recorded at any tier (sim, bench, field)
  can be replayed offline in Python and reproduces the exact same output.
- **Acceptance Criteria**:
  - [ ] A property test asserts determinism: same inputs → identical
        `StepOutput` and identical resulting `StepState`, across at least
        arc, pivot, and velocity-mode plans.
  - [ ] A fuzz test over `StepInput`/config ranges asserts zero `NaN`/`Inf`
        in `StepOutput` across >=1000 generated cases.
  - [ ] Static/source check: no method in `source/drive/` mutates a
        `const` `MotionPlan&` or reads global/static mutable state.

## SUC-003: An Arc or Pivot Segment Is Planned Within Feasibility, or Rejected Explicitly
Parent: UC-015

- **Actor**: The wafer adapter (in-process caller), via `admit()`/`plan()`.
- **Preconditions**: A `Goal` (arc or pivot) and a `ChainTail` (predicted
  queue-tail state) are available.
- **Main Flow**:
  1. `admit(goal, tail)` checks: exit reachable within the arc length; the
     joint wheel-speed step at the queue boundary is within cap; no
     per-wheel sign reversal at nonzero joint speed; the inner-wheel floor
     for arcs entered at speed; pivots require zero exit speed. Returns a
     `Verdict`.
  2. If `Verdict::OK`, `plan(request)` solves the one master jerk-limited
     profile (path length for arcs, heading for pivots) under the
     trim-headroom-folded ceiling `v_eff`, composing and freezing the
     world goal pose into the plan.
  3. If not `OK`, no `MotionPlan` is produced; the caller (the adapter) is
     responsible for surfacing the specific `Verdict` (e.g. as a wire
     NACK — SUC-009).
- **Postconditions**: Every admitted plan's sampled wheel speeds never
  exceed `vWheelMax - headroom` at any `t`; every rejected goal carries a
  specific, distinguishable reason, never a silent no-op.
- **Acceptance Criteria**:
  - [ ] A unit/tier-0 admission-verdict table exercises every `Verdict`
        enumerator with a constructed input that should produce it.
  - [ ] Property test: for any admitted plan, `max(|wheel(t)|)` over a
        dense sample of `referenceAt(t)` never exceeds
        `vWheelMax - headroom` (the `v_eff` fold invariant).
  - [ ] A pivot `Goal` with nonzero `exitSpeed` is rejected
        (`PIVOT_NONZERO_EXIT`), never silently clamped to zero.

## SUC-004: step() Tracks a Segment's Reference to Convergence Against a Realistic Plant
Parent: UC-003 (Drive Robot a Specific Distance)

- **Actor**: The tier-0 closed-loop harness (Python plant model of the
  level-2 velocity servo: first-order lag 120-140ms, stiction, encoder
  staleness ~80ms, quantization, slip).
- **Preconditions**: A `MotionPlan` is solved; a `StepState` is
  initialized to its default (all-timer, not-yet-settling) value.
- **Main Flow**:
  1. Each simulated tick, the plant reports `BodyState`/`WheelState`
     (lagged, noisy, stale per the plant model's knobs).
  2. `step()` samples the reference, computes exact-arc-projection
     path-frame errors, applies the P-only Kanayama trims (clamped to
     `trimVMax`/`trimOmegaMax`; pivot mode forces `v_cmd` to a literal
     0.0f), runs IK, curvature-preserving saturation, and the one-sided
     forward-arc wheel clamp, and emits `WheelVelocities`.
  3. The plant integrates the commanded setpoints through its own lag/
     stiction model; the loop repeats until `Status` is a `DONE_*`/
     `ABORT_*` terminal value.
- **Postconditions**: The tracked path/heading error stays within the
  replan envelopes for a plant within the modeled fault range; the
  tracker never commands a value that would saturate silently without
  `trimSaturated` reporting it.
- **Acceptance Criteria**:
  - [ ] Closed-loop sim (tier 0) converges an arc and a pivot segment to
        `DONE_STOP` within the plant model's lag/stiction range, gains
        from the issue's table (k_θ=6.0, k_c=1.5e-5, k_s=2.0, k_d=0).
  - [ ] `TrackRecord.trimSaturated` is `true` exactly when a trim was
        clamped, verified against a deliberately saturating scenario.
  - [ ] No commanded wheel velocity ever reverses sign within a single
        forward-arc segment (structural: the one-sided clamp).

## SUC-005: A Stop Segment Terminates on Measured State, Never on Prediction
Parent: UC-003

- **Actor**: The tier-0 closed-loop harness.
- **Preconditions**: A stop-segment (`exitSpeed` = 0) plan is active and
  has passed `T_plan` (its nominal duration).
- **Main Flow**:
  1. Once `t >= T_plan`, `step()` enters `Status::SETTLING`: angular trims
     are turned off; the along-track walk-in is banded and one-sided
     (inside tolerance → a literal 0.0f + 150ms dwell start; outside →
     `clamp(k_s*e_along, 50, 100)` mm/s, never negative; overshot →
     0.0f and complete).
  2. Completion requires `|e_along| <= 10-15mm AND |v_hat| <= 15mm/s`,
     held for 150ms, before the emitted setpoint snaps to a literal 0.0f
     (the level-2 PI's integrator-freeze deadband engages only on an
     exact zero).
  3. If `T_plan + 1.5s` elapses without completion, the segment
     completes-with-warning within 2x tolerance, else emits
     `Status::ABORT_TIMEOUT` — never silent.
- **Postconditions**: A stop segment never reports `DONE_STOP` from a
  time-only prediction; it costs the accepted ~280ms settle trade in
  exchange for terminal accuracy that is only ever slow, never wrong.
- **Acceptance Criteria**:
  - [ ] Tier-0 test: a stop segment held short of tolerance by the plant
        model does NOT report `DONE_STOP` before the dwell holds.
  - [ ] Tier-0 test: an overshot approach (plant runs past the target)
        completes at a literal 0.0f setpoint, never a negative (reversed)
        one.
  - [ ] Tier-0 test: a pathological non-convergent plant (stiction pinned
        above the walk-in ceiling) produces `ABORT_TIMEOUT` at
        `T_plan + 1.5s`, not an infinite `SETTLING`.

## SUC-006: A Chained Plan Hands Off at Speed Without Stopping
Parent: UC-015

- **Actor**: The tier-0 closed-loop harness / the wafer adapter (in
  production, SUC-009).
- **Preconditions**: A segment with `exitSpeed != 0` is executing;
  `ChainTail` predicts the next segment's anchor.
- **Main Flow**:
  1. Once the plan is exhausted (`t >= duration()`), `step()` checks the
     handoff envelope: `e_cross <= 30mm`, `|e_theta| <= 5deg`,
     `e_along <= 0.14*vExit + 40mm`.
  2. Within envelope: `Status::DONE_HANDOFF`; the caller plans the next
     segment seeded from the REFERENCE (`entrySpeed = vExit`, `a = 0`),
     never from measured state (measured state gates, never seeds).
  3. Outside envelope: the caller replans the same joint (pure `replan()`
     call); a persistent violation falls back to brake-to-stop + flush +
     `EventNotify` (adapter-level, SUC-009).
- **Postconditions**: A chained arc-to-arc or arc-to-pivot transition
  never inserts an unwanted full stop; velocity continuity (C¹) holds by
  construction across the handoff.
- **Acceptance Criteria**:
  - [ ] Tier-0 test: two chained arc segments (second seeded from the
        first's reference exit state) show no velocity discontinuity at
        the handoff boundary in the recorded `TrackRecord`.
  - [ ] Tier-0 test: a handoff attempted outside the envelope emits
        `Status::REPLAN_DUE`, never a silent `DONE_HANDOFF`.

## SUC-007: A Delayed Pose-Fix Correction Is Absorbed or Triggers a Bounded Replan
Parent: UC-007 (Set Odometry from External Source)

- **Actor**: The wafer adapter, forwarding sprint 099's
  `PoseEstimator::lastPoseStep()` into `StepInput.poseStep`/
  `poseStepTheta`.
- **Preconditions**: A segment is executing; a `PoseFix` (or SI reset) was
  applied by `PoseEstimator` on the immediately-prior tick (099's
  `bb.poseStepped`).
- **Main Flow**:
  1. `step()` receives a nonzero `StepInput.poseStep`/`poseStepTheta` for
     exactly the tick the correction landed.
  2. A step `<= 30mm/3deg` is absorbed by the ordinary trim law; the
     sustain timers reset (the correction is treated as ordinary
     tracking error, not a discontinuity).
  3. A step `> 30mm/3deg` bypasses the sustain filter and emits
     `Status::REPLAN_DUE` immediately (rate limit and `N-max` replan
     count still apply); a step arriving during terminal dwell does not
     interrupt completion (the segment completes on its pre-step basis
     and reports honestly).
- **Postconditions**: A camera correction never causes a silent, unbounded
  jump in commanded velocity; it is either absorbed smoothly or triggers
  exactly one bounded replan.
- **Acceptance Criteria**:
  - [ ] Tier-0 test: a small (`<=30mm`) injected `poseStep` does not
        trigger `REPLAN_DUE` and resets the sustain timer.
  - [ ] Tier-0 test: a large (`>30mm`) injected `poseStep` triggers
        `REPLAN_DUE` on the same tick, bypassing the sustain filter.
  - [ ] Tier-0 test: a `poseStep` injected while `Status::SETTLING`'s
        dwell is counting does not reset or extend the dwell.

## SUC-008: The Subsystem's Isolation Is Structurally Enforced
Parent: UC-015

- **Actor**: A grep-based unit test (CI-run, no hardware).
- **Preconditions**: `source/drive/` exists with at least `types.h`.
- **Main Flow**:
  1. The test greps every file under `source/drive/` for `msg::`,
     `Hal::`, `Subsystems::`, `MicroBit`, `kOutputHops`, `kDeadTime`, and
     any `#include` outside `source/drive/`, libc/libm, or
     `libraries/ruckig`.
  2. Any match fails the test with the offending file/line.
- **Postconditions**: `source/drive/` never silently regains a dependency
  on the rest of the firmware tree, a dead-time compensator, or the
  retired output-hop mechanism, at any point in this sprint or after.
- **Acceptance Criteria**:
  - [ ] The grep test exists, runs in the standard `pytest`/CI gate, and
        fails loudly (naming the file) on an introduced violation.
  - [ ] The test passes against the actual `source/drive/` tree as landed
        by ticket 002 and stays passing through every subsequent ticket
        in this sprint.

## SUC-009: A Segment Executes on the Real Robot Through the Wafer Adapter
Parent: UC-015 (also re-implements UC-001, UC-002, UC-003)

- **Actor**: A host client (`robot_radio`), the robot on the stand.
- **Preconditions**: The cutover ticket has landed: `Subsystems::
  Drivetrain` is the thin wafer adapter over `source/drive/`;
  `segment_executor`/`stop_condition` are removed from the active build
  list (parked on disk).
- **Main Flow**:
  1. A wire `segment`/`replace` `CommandEnvelope` arrives with
     `primitive=true` (arc/pivot fields populated).
  2. The adapter converts wire fields to a `Drive::Goal`, calls
     `admit()`/`plan()` through the held `Drive::Drivetrain`, and on
     `Verdict::OK` stages the plan; on any other verdict, replies a typed
     `ERR` and leaves the queue untouched.
  3. Each pass, the adapter converts `bb.bodyState` → `Drive::BodyState`
     and `bb.poseStepped` → `StepInput.poseStep`/`poseStepTheta`, calls
     `step()`, and stages the returned `WheelVelocities` via
     `hardware_.motor(i).apply()` — unchanged from today's staging path.
  4. On `Status::REPLAN_DUE`, the adapter calls `replan()` and swaps the
     held plan value; on `DONE_*`, it pops the next ring segment or
     neutrals the motors; on `ABORT_*`, it flushes the ring, re-anchors
     `ChainTail`, and emits a populated `EventNotify`.
- **Postconditions**: The robot drives arcs and pivots on the real plant
  through the new subsystem with the old stack fully retired from the
  live call path; `cmd_vel`/`vel` telemetry are both live and distinct.
- **Acceptance Criteria**:
  - [ ] HITL (robot on the stand, USB-attached): a `segment` command
        drives an arc and a pivot to completion; encoders/`vel=` show
        plausible, direction-correct motion.
  - [ ] HITL: an infeasible `segment` (e.g. a pivot with nonzero exit
        speed) NACKs at the wire with the specific `Verdict`, and the
        queue is untouched.
  - [ ] Sim (tier 1): the golden-TLM zero-error path is bit-exact after
        regeneration (an explicit, reviewed step, not silently
        re-baselined).
  - [ ] A legacy text `MOVE`/`S`/`T`/`D` command (translated host-side via
        `primitives_for_move()`) still drives correctly through the new
        adapter.

## SUC-010: An Operator Drives by Deadman-Velocity Teleop
Parent: UC-001 (Drive Robot at Continuous Speed)

- **Actor**: An operator via a gamepad/host teleop client (`MOVER`).
- **Preconditions**: The cutover has landed; `planVelocity()` is wired
  into the adapter's `replaceIn` path.
- **Main Flow**:
  1. The client streams `MOVER`-equivalent commands (`v`, `omega`,
     deadman duration) at the teleop rate.
  2. The adapter calls `Drivetrain::planVelocity(target, deadman,
     current)`, producing a velocity-mode `MotionPlan` with no pose goal;
     each new `MOVER` replaces the held plan via `replaceIn` (latest-wins,
     matching the existing `Mailbox<Motion::Segment>` semantics).
  3. If the deadman window elapses with no fresh `MOVER`, the plan's own
     terminal machine (SUC-005) decelerates to a literal 0.0f — no
     separate watchdog logic duplicated in the adapter.
- **Postconditions**: Teleop driving works end-to-end through the new
  subsystem; releasing the deadman always results in a commanded stop,
  never a runaway.
- **Acceptance Criteria**:
  - [ ] HITL: a streamed `MOVER` sequence drives the robot smoothly on
        the stand at commanded (v, omega); releasing the deadman brings
        the robot to a literal-zero setpoint within the terminal
        machine's dwell.
  - [ ] Tier-1 sim: `stream=true` on the (unrelated) `segment`/`MOVE` arm
        still replies `ERR` (BLEND is out of scope this sprint) while
        `MOVER`'s own path is unaffected.

## SUC-011: A Developer Dumps and Plots the Live Plan and Track Over the Wire
Parent: UC-015

- **Actor**: A developer via `robot_radio`, plotting against
  `tests/notebooks/out/`.
- **Preconditions**: The cutover has landed; `PlanDumpRequest`/
  `PlanRecord`/`MotionTrace` wire arms exist.
- **Main Flow**:
  1. The developer sends `PlanDumpRequest` (`CommandEnvelope` arm 18); the
     robot replies one `PlanRecord` (arm 10) per ring entry, sharing a
     `corr_id`, each carrying goal/anchor/`v_eff`/duration/exit_speed/
     entry_speed/replan_count (~85B each).
  2. The developer arms `StreamControl.trace`; the robot emits
     `MotionTrace` (arm 11, ~90-120B) at the TLM period, carrying a
     serialized `TrackRecord` including the `StepInput` replay fields.
  3. The developer's tooling overlays the dumped plan table against the
     streamed `MotionTrace` on a notebook plot.
- **Postconditions**: A developer can see both "what the robot intends to
  do" and "what the robot is actually doing, per tick" without any
  firmware code change, and can replay any `MotionTrace` row at tier 0.
- **Acceptance Criteria**:
  - [ ] HITL/sim: a `PlanDumpRequest` against a multi-segment ring returns
        one correlated `PlanRecord` per entry.
  - [ ] HITL/sim: `MotionTrace` frames, once armed, decode to a valid
        `TrackRecord` whose `StepInput` replays bit-exact at tier 0
        (SUC-002).
  - [ ] `Telemetry`'s existing ~166B budget is untouched (`MotionTrace` is
        a new reply arm, never a `Telemetry` extension).

## SUC-012: The Tracker Holds Up Under Realistic Plant Faults
Parent: UC-015

- **Actor**: The tier-1 sim fault-knob matrix.
- **Preconditions**: The cutover has landed; the sim's `motor_lag`,
  `enc_slip`, `stiction`, `trackwidth`, `scrub` knobs are available.
- **Main Flow**:
  1. Each fault knob is exercised individually (and the sim's
     `motor_lag` is set to 120-140ms as the default for every
     tracker/replan scenario — the zero-lag path is reserved for
     golden-TLM bit-exactness only).
  2. `enc_slip`/scale faults are checked against `true_pose` convergence;
     `stiction` is checked for terminal walk-in with no premature `DONE`
     and no reversal; `trackwidth` error is checked for cross-gain
     correction of the resulting radius error; an infeasible ask is
     checked for a typed `ERR` with the queue untouched.
- **Postconditions**: No fault-knob combination in the matrix produces a
  silent wrong-answer (a `DONE_STOP` that isn't actually stopped, a
  reversal, or a hang); every failure mode surfaces as a typed status.
- **Acceptance Criteria**:
  - [ ] Sim test matrix covers `motor_lag`(120-140ms)/`enc_slip`/
        `stiction`/`trackwidth`-error/`scrub`, each asserting against
        `true_pose`, never `bb.fusedPose`, per the plant-model convention.
  - [ ] No scenario in the matrix reproduces the 2026-07-11 false-green
        (zero-lag-only validation) failure class.

## SUC-013: The Real Robot Executes Arc/Pivot/Chain Grids Within Tolerance On the Bench
Parent: UC-015

- **Actor**: A developer running `tests/bench/arc_sweep.py` against the
  robot on the stand.
- **Preconditions**: The cutover, MOVER cutover, and trace/plan-dump
  tickets have landed; the robot is USB-attached (flashed) or reachable
  over the radio relay (capture).
- **Main Flow**:
  1. `arc_sweep.py` (the `turn_sweep.py`-pattern dual-transport capture)
     runs a grid of arcs, pivots, and chains, recording `MotionTrace` CSVs
     into `tests/notebooks/out/`.
  2. The measured wheel-speed plateau is re-measured and pinned into
     `tovez.json` as `v_wheel_max`.
  3. The 098 pivot acceptance grid (angle x speed, both directions) is
     re-run against the new stack.
- **Postconditions**: The new subsystem matches or exceeds the old
  stack's ±1deg pivot accuracy on hardware, with arcs and chains landing
  within the envelope tolerances from the issue's control-law table.
- **Acceptance Criteria**:
  - [ ] `arc_sweep.py` grid results land within the issue's terminal
        tolerance (`|e_along| <= 10-15mm`) on every cell.
  - [ ] The re-run 098 pivot grid lands >=100% within +/-1deg (matching
        or improving on 098's own bench-verified result) or the delta is
        explicitly analyzed and accepted, not silently regressed.
  - [ ] `v_wheel_max` in `tovez.json` reflects a bench-measured plateau,
        not the sim's 400mm/s ceiling.

## SUC-014: A Camera-Verified Field Chain Closes the Full Pose-to-Tracker Loop
Parent: UC-007 (also UC-015)

- **Actor**: A developer running a playfield chain script (the
  `playfield_camera_run.py` pattern) with live `PoseFix` corrections.
- **Preconditions**: Bench acceptance (SUC-013) has passed; the robot is
  on the camera-covered playfield, geofenced per project convention.
- **Main Flow**:
  1. The script commands a multi-segment world-frame chain.
  2. The camera observes the robot's true pose and sends `PoseFix`
     corrections mid-chain (sprint 099's mechanism); the adapter forwards
     `bb.poseStepped` into `StepInput.poseStep` each pass (SUC-007).
  3. The script overlays the planned `RefState` polyline against the
     fused pose and the camera ground truth.
- **Postconditions**: The full camera → EKF → tracker loop closes
  end-to-end; a `PoseFix` mid-chain measurably improves (not degrades)
  world-frame tracking accuracy versus dead reckoning alone.
- **Acceptance Criteria**:
  - [ ] A camera-verified chain completes without leaving the geofenced
        playfield area.
  - [ ] The plan-vs-actual overlay shows the fused/camera traces
        converging, not diverging, across at least one mid-chain
        `PoseFix`.

## SUC-015: Retired Motion Code Is Fully Removed After Bench Sign-Off
Parent: (internal — hygiene, not a stakeholder-facing use case)

- **Actor**: The cleanup ticket's programmer.
- **Preconditions**: Bench (SUC-013) and field (SUC-014) acceptance have
  both passed; the parked files
  (`segment_executor.*`/`segment.h`/`motion_baseline.h`/`stop_condition.*`)
  have been off the active build list since the cutover ticket.
- **Main Flow**:
  1. The parked files are deleted from disk (not merely excluded from the
     build).
  2. Retired proto fields (`PlannerConfig.heading_kp`/`heading_kd`, the
     `governRatio` segment-mode call path) are reserved (`reserved N;`),
     never silently reused for a different meaning.
  3. The full sim suite is re-run to confirm no latent reference to a
     deleted file remains.
- **Postconditions**: The repository contains exactly one motion stack;
  no dead code, no unreachable dual path.
- **Acceptance Criteria**:
  - [ ] `git grep` for `segment_executor\|stop_condition\|SegmentExecutor`
        outside `clasi/`/history returns nothing under `source/`.
  - [ ] `protos/planner.proto` marks the retired `heading_kp`/
        `heading_kd` field numbers `reserved`, not reassigned.
  - [ ] The full sim suite (`uv run python -m pytest`) passes after
        deletion.
