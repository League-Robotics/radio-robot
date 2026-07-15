---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Use Cases — Sprint 106: Host trajectory planner: profiled twists, straights and turns

Continues SUC numbering from the single-loop-firmware arc (103: SUC-001..010,
104: SUC-011..017, 105: SUC-018..023). This sprint is the first sprint that
builds NEW capability on top of that arc rather than completing it.

## SUC-024: Firmware loop-cadence fix — telemetry paces to its own design budget

- **Actor**: Firmware/host engineer relying on telemetry as the planner's
  only feedback channel.
- **Preconditions**: `source/app/robot_loop.cpp`'s `cycle()` schedules three
  hardware-mandated 4ms windows (`kSettle`, `kClear`, `kSettle` — encoder
  settle ×2, duty-write clearance ×1) and then paces the whole cycle to
  `kCycle=16ms` via `sleepUntil(cycleStart, kCycle)`. Ticket 105-004's
  virtual-cycle-timing diagnostic proved this is a **scheduling defect, not
  merely an environmental slowdown**: with a fake `Devices::Clock` that never
  advances mid-cycle, the SAME code deterministically requests
  `4+4+4+16=28ms` of virtual sleep per cycle — 12ms over the 16ms design
  target — entirely because the final `sleepUntil`'s 16ms figure was sized
  as if it were the WHOLE cycle's budget, not the remainder left after the
  three settle/clearance windows. Sprint 104's own bench measurement showed
  ~36ms/cycle real (~28 Hz would be optimistic even at 36ms; the ack-ring
  issue's own bench figure is ~13.87 Hz / ~72ms — a further, currently
  unreconciled gap this ticket must re-measure fresh, not assume). Telemetry
  emits once per cycle, so this bounds the ONLY feedback rate the host
  planner's heading loop and profile executor will ever see.
- **Main Flow**: `kSettle`/`kClear`/`kCycle` (and/or the `sleepUntil`
  accounting itself) are retargeted so the schedule is internally
  consistent — the three mandatory windows are absorbed into, not stacked
  on top of, the stated per-cycle budget — at a new, explicitly-chosen
  target (~40ms / ~25 Hz, not the original aspirational 16ms, which never
  had room for the three windows plus real per-cycle work in the first
  place). `tests/sim/support/sim_api.h`'s virtual-cycle-timing diagnostic is
  promoted from observational to a hard pytest assertion on the new schedule
  shape (guards this regression permanently). The robot is deployed and the
  real per-cycle TLM emission rate is re-measured on the bench.
- **Postconditions**: The firmware's own scheduled cycle time is a
  documented, internally-consistent number (constants sum to the stated
  target, not silently over it); sim asserts the schedule shape; bench
  confirms the achieved real cadence and reconciles it against both the
  105-004 figure (~36ms) and the ack-ring issue's figure (~72ms).
- **Acceptance Criteria**:
  - [ ] `kSettle + kClear + kSettle` is `<=` the new `kCycle` target, and the
        final `sleepUntil` call's accounting is proven (by the sim's
        zero-real-time-cost virtual clock) to actually pad to that target,
        not add a fresh, unabsorbed increment on top.
  - [ ] `tests/sim/system/` asserts the new virtual per-cycle schedule
        (replacing 105-004's diagnostic-only report) — a regression in this
        schedule fails `uv run python -m pytest`, not just a bench session.
  - [ ] Deployed to the bench rig (`.claude/rules/hardware-bench-testing.md`)
        and the real TLM cadence is measured (e.g. `relay_telemetry_rate.py`
        or an equivalent seq-gap capture) and recorded, explicitly
        reconciling it against both prior figures (~36ms 104, ~72ms
        ack-ring issue) rather than reporting a bare new number.
  - [ ] No behavior change to the command-dispatch/telemetry CONTENT — only
        pacing constants and `sleepUntil` accounting change.

## SUC-025: Inner velocity-PID resonance tamed; motor gains apply live over the wire

- **Actor**: Host planner streaming profiled twists; bench engineer tuning
  gains without a reflash.
- **Preconditions**: `heading-loop-output-clamp-and-velocity-resonance.md`
  Part 2 — the kept `Devices::MotorVelocityPid` has a documented ~140 mm/s
  step-response resonance (+24–33% overshoot at the interim
  `vel_kp=0.0014` tuning) that will ring directly in this sprint's own
  accel/decel telemetry traces. Separately (a Step 1 finding of this
  sprint's own architecture pass, not previously flagged): the P4 wire's
  `ConfigDelta` decodes successfully but is **never applied** —
  `RobotLoop::cycle()`'s `CmdKind::CONFIG` case unconditionally acks
  `ERR_UNIMPLEMENTED` (confirmed live in `source/app/robot_loop.cpp` and in
  `NezhaProtocol.config()`'s own docstring) — and `Devices::NezhaMotor` has
  **no runtime gain mutator at all**, only a constructor-time
  `const MotorConfig&`. Binding requirement #9 ("everything tunable live")
  is therefore currently unmet at the firmware boundary for the P4 image:
  the pre-P4 bench method this issue describes (`SET pid.kp` on the stand)
  no longer exists on the wire.
- **Main Flow**: `Devices::NezhaMotor` gains a live gain-apply capability;
  `RobotLoop::cycle()`'s `CONFIG` case decodes a `MotorConfigPatch` and
  applies present `kp`/`ki`/`kff`/`i_max`/`kaw`/`travel_calib` fields to
  both bound motors, acking `OK` instead of `ERR_UNIMPLEMENTED`.
  `DrivetrainConfigPatch`/`PlannerConfigPatch` remain `ERR_UNIMPLEMENTED`
  (out of this sprint's scope — nothing in 106 applies them on-robot, since
  heading correction is host-side). With live gain apply working, the
  on-stand velocity-step harness (`tests/bench/`) characterizes the
  resonance across the speed range and tames it — first by exhausting the
  now-live-tunable `kp`/`ki`/`kff`/`iMax`/`kaw` surface, only adding a new
  wire-tunable field (e.g. promoting `velFiltAlpha`, currently reflash-only)
  if constants alone cannot hit the bar.
- **Postconditions**: A `SET`-equivalent live tuning path exists again on
  the P4 wire (via `config()`), and the inner velocity loop meets the
  issue's own acceptance bar without sacrificing the endpoint accuracy the
  interim detuning traded away.
- **Acceptance Criteria**:
  - [ ] A `config()` call carrying `pid.kp`/`ki`/`kff`/`iMax`/`kaw` measurably
        changes the robot's live step response on the SAME boot, no reflash.
  - [ ] On-stand velocity-step harness (drive-arm step at 70/140/250 mm/s,
        per the issue) shows `<~10%` step overshoot across that range with
        rise time preserved, superseding the interim `vel_kp=0.0014`
        detuning.
  - [ ] `DrivetrainConfigPatch`/`PlannerConfigPatch` remain explicitly
        `ERR_UNIMPLEMENTED` (unchanged, documented as deliberately
        out-of-scope, not silently broken).
  - [ ] Bench-verified per `.claude/rules/hardware-bench-testing.md` — wheels
        driven on the stand, encoders/telemetry confirmed responsive.

## SUC-026: Sim can observe a closed-loop settle to zero across multiple writes

- **Actor**: Sprint 106 engineer validating a profile's deceleration/stop
  phase in sim before spending bench time.
- **Preconditions**: `sim-api-multi-write-decay-window.md` — `SimApi::
  scriptCycleBusResponses()` provisions exactly one post-command duty write
  per injected command at a single hand-derived `pendingEventCycle_` index;
  a scenario that lets the PID settle toward a reachable target (e.g. a
  profile's own deceleration ramp, or `injectStop()` mid-ramp) issues SEVERAL
  more duty writes as the quantized output un-saturates, desyncing the
  shared `I2CBus` script FIFO past roughly 4 cycles.
- **Main Flow**: `SimApi`'s per-cycle bus-response scripting is generalized
  to detect whether a leaf's `appliedDuty()` actually changed since the
  previous cycle, scripting a write only when it does, rather than assuming
  exactly one transition at one hand-derived index.
- **Postconditions**: A sim scenario can drive a multi-cycle deceleration to
  a full, exact settle (e.g. velocity converging to 0.0, not just "a >50%
  drop within 4 cycles") without FIFO desync artifacts (`connRight` flipping
  false, a false `kFaultWedgeLatch`, `velLeft` freezing at a wrong value).
- **Acceptance Criteria**:
  - [ ] A new or existing sim scenario steps well past the old ~4-cycle safe
        window through a full profile deceleration and asserts convergence
        to (approximately) zero velocity with no bus-error/fault-bit
        artifacts.
  - [ ] `105-006`'s existing scripted-twist STOP-phase scenario (which
        documents and stays within the old bound) is left passing unchanged
        or is updated to assert full convergence now that the bound is
        lifted — implementer's call, documented either way.
  - [ ] `tests/sim/plant`/`tests/sim/support` test suites stay green.

## SUC-027: Pure trapezoidal velocity-profile generator (straight distance, in-place turn angle)

- **Actor**: Host planner code (and its own unit tests) — no robot, no sim,
  no I/O.
- **Preconditions**: No host-side trajectory generator exists yet;
  `host/robot_radio/nav/`'s `navigator.py`/`camera_goto.py` call retired
  blocking verbs (`Robot.go_to()`, `NezhaProtocol.drive()`) that have no P4
  equivalent (`nezha-facade-and-midlayer-dead-verb-residue.md`) and are not
  reused this sprint (their own fate is a separate, future stakeholder
  call). `tests/bench/bench_ruckig_motion_verify.py`'s name notwithstanding,
  no jerk-limited-trajectory library is an actual project dependency today
  — that script targets the retired pre-102 `D`/`T`/`TURN`/`RT` text verbs
  and does not apply to the P4 wire.
- **Main Flow**: A new pure module decomposes (a) a signed straight-line
  distance or (b) a signed in-place turn angle, plus acceleration/
  deceleration (and cruise-speed) limits, into an ordered sequence of
  `(elapsed, v_x, omega)` setpoints sampled at the executor's streaming
  cadence — a classic trapezoidal (accelerate/cruise/decelerate) shape, with
  the final decelerating leg driven to exactly zero at the target, never a
  sign-reversal "creep back." Every input (distance/angle/limits) is
  validated (finite, in-range, non-degenerate) at the boundary before any
  setpoint is generated (binding requirement #5) — an invalid input raises
  immediately rather than producing a garbage or silently-clamped profile.
- **Postconditions**: Given the same inputs, the generator deterministically
  produces the identical setpoint sequence — no I/O, no robot state, no
  wall-clock dependency (a single `elapsed` axis parameterizes the whole
  profile), so it is fully unit-testable and independently reusable by both
  the sim scenario (SUC-026/030) and the real executor (SUC-028).
- **Acceptance Criteria**:
  - [ ] `profile_for_distance(distance, ...)` and `profile_for_turn(angle,
        ...)` (or equivalent names, ticket-time call) each return a
        deterministic setpoint sequence; unit tests assert the accel phase
        never exceeds the configured `a_max`, the cruise phase (if reached)
        holds `v_max`, and the final setpoint lands at exactly zero velocity
        with the commanded sign preserved throughout (never a fabsf-blind
        predicate — binding requirement #1).
  - [ ] A short-distance/short-angle case that never reaches cruise (pure
        triangle profile) is covered and produces a shape distinct from the
        trapezoid case, not a truncated/incorrect trapezoid.
  - [ ] A degenerate/invalid input (zero distance, negative limit,
        non-finite value) raises rather than producing a setpoint sequence.
  - [ ] 100% unit-tested under `tests/unit/`, no hardware/sim dependency.

## SUC-028: Streaming twist executor — paced, safe, binding-requirement-compliant

- **Actor**: Host planner driving the real (or simulated) robot through a
  generated profile.
- **Preconditions**: `NezhaProtocol.twist()`/`.stop()`/`.config()` are
  fire-and-poll (no synchronous per-command reply); `ack-ring-intermittent-
  delivery-gap.md`'s own recommendation is explicit: sprint 106 must NOT
  gate closed-loop control on a bounded, wait-then-give-up `wait_for_ack()`
  call — use continuously-flowing telemetry (`SerialConnection.
  drain_binary_tlm()`) instead, and never send twist commands back-to-back
  with no pacing (a real, sharp reliability cliff, worse over the relay).
- **Main Flow**: The executor walks a profile's setpoint sequence, sending
  one `twist()` per streaming tick at a stated, live-tunable pacing
  interval, re-arming the deadman window on every send, and draining
  telemetry continuously between sends (never a bounded per-command ack
  wait) to track measured progress and detect faults. Every one of the ten
  binding requirements from `host-planner-design-lessons-from-drive-v2-
  review.md` applies directly to this module — see architecture-update.md
  Step 6 for the full per-requirement design mapping. A preemption (a new
  profile arriving, or an explicit stop) always issues `stop()` and
  discards any remaining setpoints from the OLD profile before a new one
  begins; the new profile is built from freshly-measured telemetry state,
  never carried entry speed from the interrupted one.
- **Postconditions**: A profile runs to completion (reaching its own
  distance/angle target within a bounded overshoot) or is cleanly preempted/
  stopped, with every failure mode (validation reject, fault bit observed,
  overshoot bound exceeded, deadman expiry) logged loudly, never silent.
- **Acceptance Criteria**:
  - [ ] The executor's completion check is sign-aware (never `fabsf` on a
        signed measured quantity) and has a bounded outer tolerance in BOTH
        directions (binding requirements #1, #6).
  - [ ] No control decision anywhere in the executor is gated on a bounded
        `wait_for_ack()` — verified by code inspection/grep, per the
        ack-ring issue's own recommendation (binding requirement #2).
  - [ ] The executor uses a single segment-global elapsed-time clock per
        profile run; a preemption starts a fresh clock, never rebasing a
        stale one (binding requirement #3).
  - [ ] Preempting a running profile and starting a new one is unit-tested
        (with a fake transport/telemetry double) to confirm the new profile
        plans from injected "current" state, not carried state from the old
        one (binding requirement #4).
  - [ ] Every `twist()` magnitude is validated against configured ceilings
        immediately before sending, independent of the profiler's own
        validation (binding requirement #5, defense in depth).
  - [ ] The terminal setpoint of any profile is an explicit `stop()` call,
        never reliance on deadman timeout alone to end an intended motion,
        and no phase ever commands a zero-dwell sign reversal (binding
        requirement #7).
  - [ ] Streaming cadence, acceleration/deceleration limits, and every gain
        the executor/profiler use are adjustable at runtime with no code
        redeploy (binding requirement #9) — a config object/file, not
        hardcoded constants.

## SUC-029: Host-side heading-correction loop against streamed encoder pose

- **Actor**: Host planner keeping a straight leg or turn on-heading.
- **Preconditions**: No on-robot heading loop exists post-102 (the pre-102
  `Motion::SegmentExecutor` cascade was deleted). `data/robots/tovez.json`'s
  `geometry.otos_untrusted: true` — the rig's OTOS reading must not be
  trusted for heading feedback; `Telemetry.pose` (firmware-fused,
  encoder-derived) is the correct source this sprint, never `Telemetry.
  otos`. `host/robot_radio/controllers/pid.py`'s `PID` class already
  provides a clamped-output, generic discrete PID plus `normalize_angle()`.
- **Main Flow**: A heading corrector reads `robot_config`'s
  `otos_untrusted` flag to confirm/select the encoder-pose feedback source,
  and on each executor streaming tick computes a clamped omega trim from
  the profile's own commanded heading vs. the latest drained telemetry's
  measured heading, added to the profile's own `omega` setpoint before
  sending. The output clamp directly carries forward the deleted on-robot
  heading loop's own lesson (`heading-loop-output-clamp-and-velocity-
  resonance.md` Part 1: an unclamped heading correction over-drove the
  wheels straight into the ~140 mm/s resonance band) — generalized
  host-side since the original clamp site no longer exists. The achievable
  correction bandwidth over the (now ~25 Hz, post-SUC-024) telemetry link is
  measured empirically on the bench before the gains are treated as final
  (binding requirement #10).
- **Postconditions**: A profiled straight or turn holds within a stated
  heading tolerance, with the correction visibly NOT re-exciting the tamed
  (SUC-025) inner-loop resonance.
- **Acceptance Criteria**:
  - [ ] The corrector reads `otos_untrusted` from the active robot config
        and uses encoder-derived `pose`, never `otos`, when the flag is set
        (unit-testable with a fake config + fake telemetry frame).
  - [ ] The corrector's output is clamped to a stated, live-tunable ceiling
        — unit-tested: a large injected heading error never produces an
        omega trim above the ceiling.
  - [ ] Bench session records the achievable correction bandwidth over the
        real telemetry link (post-SUC-024 cadence) and states the gains
        chosen as a result — an empirical finding, not an assumed number.
  - [ ] A profiled straight leg on the bench stays within a stated
        cross-track/heading tolerance (recorded in the ticket's own
        completion notes).

## SUC-030: Bench-runnable profiled-motion proof — sim-validated first

- **Actor**: Stakeholder / bench engineer verifying this sprint's own
  acceptance bar.
- **Preconditions**: SUC-024..029 exist and are individually verified (sim
  and/or unit-tested). No end-to-end profiled run has ever executed on this
  architecture.
- **Main Flow**: A profiled single straight leg and a profiled single
  in-place turn (an arc leg is a stretch goal if ticket sequencing allows)
  are each first run against `tests/sim/system/`'s `SimApi` (asserting the
  profile completes, telemetry shows the expected ramp shape, no fault bits)
  and then executed for real on the bench rig
  (`.claude/rules/hardware-bench-testing.md`), capturing the full streamed
  telemetry trace (commanded vs. measured velocity/heading over time) to a
  file a human can plot and visually judge.
- **Postconditions**: A captured bench trace exists for both a straight leg
  and a turn, showing clean (non-ringing, bounded-overshoot) acceleration
  and deceleration — the raw material sprint 107's notebook will chart.
- **Acceptance Criteria**:
  - [ ] Sim scenario(s) for a profiled straight and a profiled turn pass
        under `uv run python -m pytest`, exercising the REAL profile
        generator + executor logic against `SimApi` (not a reimplemented
        test-only model).
  - [ ] The same profiled straight and profiled turn are run for real on the
        bench stand; a captured telemetry trace (e.g. CSV/JSON under
        `tests/bench/out/`) records commanded vs. measured velocity and
        heading over the run.
  - [ ] A human reviewing the captured trace can see the acceleration and
        deceleration phases are clean — no visible resonance ringing
        (matching SUC-025's `<~10%` overshoot bar) — recorded as a
        pass/fail judgment in the ticket's own completion notes, since the
        chart itself is sprint 107's deliverable, not this sprint's.
  - [ ] Every device the run touches (motors, encoders, telemetry link) is
        confirmed alive per `.claude/rules/hardware-bench-testing.md`'s
        standing verification gate.
