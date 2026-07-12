---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 098 Use Cases

`docs/usecases.md` predates the `TURN`/`RT`/`MOVE` rotational-channel verb
family and the Ruckig-based `Motion::SegmentExecutor` entirely (it still
describes a `MotorController`/`PurePursuit`/`Stanley` design superseded by
sprint 089's Ruckig migration and sprint 094's segment executor) — it has no
top-level UC for in-place rotation. SUC-001/SUC-002 below are this sprint's
own root use cases (`Parent: N/A`); SUC-005 uses this project's established
"Parent: UC (bench operation / hardware verification)" convention (see
sprint 094's `usecases.md`) for the standing hardware-acceptance gate.

## SUC-001: An in-place turn of any angle, at any speed, terminates on the requested heading

Parent: N/A (no existing top-level UC covers in-place rotation — see note
above).

- **Actor**: Python host / stakeholder issuing `TURN`/`RT`/`MOVE` (a segment
  with a nonzero `direction` and/or `finalHeading`); the 58-turn playfield
  dataset that proved the root cause.
- **Preconditions**: Sprint 094's `Motion::SegmentExecutor` (3-phase
  PRE_PIVOT/TRANSLATE/TERMINAL_PIVOT sequencer) and sprint 097's velocity-
  loop redesign (`real-robot-motion-calibration-undershoot.md` — feedback-
  dominant wheel PID, `kOutputHops` re-measured) are in place and unchanged.
- **Main Flow**:
  1. Host sends a segment whose rotational component (PRE_PIVOT and/or
     TERMINAL_PIVOT) is nonzero.
  2. Each executor tick, the rotational channel's Ruckig-sampled desired
     heading/rate are compared against the ENCODER-DERIVED measured heading/
     rate (`(encR−encL)/trackwidth`, relative to the phase's own baseline);
     the commanded angular rate is corrected by the outer heading PD law
     (`ω_cmd = ω_desired + Kp·(θ_desired−θ_measured) + Kd·(ω_desired−ω_measured)`)
     before being converted to wheel velocity targets and handed to the
     UNCHANGED inner wheel-velocity PID loops.
  3. The phase completes once `|target heading − measured heading| < tol`
     AND `|measured angular rate| < rate_tol`, held for a short dwell — not
     merely when the Ruckig plan's own profile duration/arc-threshold is
     exhausted.
- **Postconditions**: The robot's final heading is within goal tolerance
  (≈±1°) of the commanded heading, for any commanded angle and any
  commanded speed ceiling — the speed-dependent 90° overshoot ridge and the
  σ≈2° run-to-run scatter documented in
  `heading-loop-cascade-control-turns-terminate-on-target.md` are gone.
  Zero commanded terminal reversal at the wheel level beyond what the
  `Hal::Motor` reversal-dwell/deadband armor already absorbs.
- **Acceptance Criteria**:
  - [ ] `tests/sim/unit/segment_executor_harness.cpp` gains PD-loop and
        tolerance-completion scenarios; the full sim regression suite stays
        green (no regression from the pre-sprint 615).
  - [ ] `tests/bench/turn_sweep.py --relay --both`, run on the playfield
        across the existing angle × ceiling grid, lands every cell within
        ≈±1° of target; the 90° ridge (a same-entry-rate turn overshooting
        roughly double a longer turn's overshoot) is gone; repeated trials
        of the same cell no longer scatter by σ≈2°.
  - [ ] A stand session confirms no wedge and no commanded terminal reversal
        beyond the motor armor's own absorbed dwell/deadband window.

## SUC-002: The divergence replan is retired to stall-protection only

Parent: N/A (internal control-correctness concern, no independent
user-visible behavior beyond SUC-001's own acceptance).

- **Actor**: `Motion::SegmentExecutor`'s own `maybeReplanPivot()`; the sim
  test suite; a deliberately stalled/bogged wheel.
- **Preconditions**: SUC-001's PD cascade is live and is now the continuous
  corrector for nominal tracking lag during PRE_PIVOT/TERMINAL_PIVOT.
- **Main Flow**:
  1. Under NOMINAL tracking lag (the kind the PD loop already corrects),
     the pre-existing sub-gross (`kRotDivergenceThreshold`) EXTEND-only
     replan path no longer fires for these two phases — it would otherwise
     double-correct against the PD loop's own continuous correction.
  2. Under a GENUINELY stalled/bogged wheel (the kind no PD gain on `ω_cmd`
     alone can fix if the wheel-level PID/motor cannot achieve it), the
     existing gross-divergence (`kRotGrossDivergenceThreshold`) reanchor
     path still fires, within the same ~2-pass window as today.
- **Postconditions**: A stalled wheel is still caught and re-anchored; a
  normally-tracking turn's plan is never re-solved out from under the PD
  loop.
- **Acceptance Criteria**:
  - [ ] A new sim scenario proves the sub-gross EXTEND path does not fire
        for PRE_PIVOT/TERMINAL_PIVOT under the harness's existing nominal-
        lag conditions (where the pre-sprint code WOULD have fired it).
  - [ ] A new sim scenario proves the gross-divergence reanchor still fires,
        within the same ~2-pass budget, when a wheel is held artificially
        stalled (a fixed/zero encoder reading against a nonzero command).
  - [ ] `maybeReplanTranslate()` (the TRANSLATE/linear-channel replan) is
        untouched — this SUC is scoped to the rotational channel only.

## SUC-003: Heading/velocity gains are tunable live, without a reflash

Parent: N/A (tooling/process concern; ties to
`real-robot-motion-calibration-undershoot.md`'s "Also discovered" runtime-
config gap).

- **Actor**: Bench operator tuning `heading_kp`/`heading_kd` (or an existing
  `vel_*`/drivetrain gain) against the real plant.
- **Preconditions**: SUC-001 has landed and is bench-tunable only via
  reflash today (093/094 removed runtime config-application authority;
  096 built the binary `SET` arm expecting a future consumer that was never
  wired).
- **Main Flow**:
  1. Operator sends a binary `SET` config delta for `heading_kp` (or another
     Configurator-managed field) over serial or the radio relay.
  2. The delta acks into `bb.configIn` as it already does today.
  3. A live `Rt::Configurator`, now constructed and ticked once per pass in
     `main.cpp`, drains the delta, folds it onto its persistent config copy,
     and — for a `kPlanner`-targeted delta specifically — calls
     `Subsystems::Drivetrain::configureMotion()` so the change reaches the
     live `Motion::SegmentExecutor` (today's `kPlanner` fold path only
     folds+publishes to `bb.plannerConfig`; there is no live subsystem left
     to call `configure()` on since `Subsystems::Planner` was relocated out
     of `source/` in ticket 094-002 — `Subsystems::Drivetrain` is the
     correct live target now).
- **Postconditions**: The next turn commanded after the `SET` reflects the
  new gain, with no reflash. Boot config still applies once, directly, at
  construction (unchanged) — the Configurator only handles LIVE deltas
  arriving after boot; this is additive, not a return to 093/094-era full
  runtime config authority.
- **Acceptance Criteria**:
  - [ ] A live `SET` of `heading_kp` measurably changes the commanded twist
        on the very next segment, in sim, without restarting the process.
  - [ ] Boot-time behavior (the config a freshly booted robot runs with) is
        provably unchanged — the Configurator is seeded from the same boot
        defaults already applied directly at construction.
  - [ ] `kMotor`/`kDrivetrain`/`kOdometer` config deltas (already reaching
        their live subsystems via the existing fold paths) are unaffected.

## SUC-004 [Stage 2, optional/deferrable]: OTOS heading, with encoder fallback, immunizes the loop against wheel slip

Parent: N/A; explicitly scoped narrower than sprint 099
(`restore-pose-estimation-otos-encoders-delayed-camera-fixes.md`'s full
pose-fusion restoration) — this SUC needs OTOS *heading* only, not position
or fused pose.

- **Actor**: `Hal::OtosOdometer` (already `begin()`s but is never ticked in
  the live loop today); the executor's heading-measurement step.
- **Preconditions**: SUC-001 (encoder-only heading loop) is bench-verified
  and is the fallback this SUC must never regress below.
- **Main Flow**:
  1. `main.cpp` ticks the OTOS leaf once per pass (a new call, alongside the
     existing `hardware.tick(now)` — OTOS is a separate I2C device, `0x17`,
     outside the Nezha flip-flop sequencer) and commits `bb.otos`/
     `bb.otosConnected` (already-declared, currently-unwritten blackboard
     fields).
  2. `Subsystems::Drivetrain` reads the OTOS leaf's pose/connectedness
     directly (it already holds `Hardware&`) each tick and threads it into
     `Motion::SegmentExecutor::tick()`'s existing (currently always-empty)
     `msg::PoseEstimate` parameter — the seam the executor's stop-condition
     evaluation already routes through today, unused.
  3. The executor's heading-measurement step prefers OTOS heading when
     connected and fresh; falls back to the encoder-derived heading
     (SUC-001's path, unchanged) otherwise.
- **Postconditions**: Turn accuracy is at least as good as SUC-001's
  encoder-only baseline on the SAME hardware; loop-pass timing and radio
  responsiveness are unaffected (the new I2C transaction does not starve
  `uBit.sleep(1)`'s yield).
- **Acceptance Criteria**:
  - [ ] A sim scenario proves OTOS-valid vs. OTOS-invalid source selection
        (injectable in the harness without real hardware).
  - [ ] A sim/bench comparison shows no turn-accuracy regression vs.
        SUC-001's own bench baseline.
  - [ ] A bench session confirms no loop-timing/radio regression (no
        symptom matching `[[radio-needs-loop-yield]]`).

## SUC-005: Hardware acceptance — the standing bench/playfield gate

Parent: UC (bench operation / hardware verification), per sprint 094's own
`usecases.md` convention.

- **Actor**: Bench operator / execution agent with physical access to the
  robot (`.claude/rules/hardware-bench-testing.md`).
- **Preconditions**: `just build-clean` succeeds; the full sim regression
  suite is green.
- **Main Flow**:
  1. Flash the robot over USB (`mbdeploy probe` then `mbdeploy deploy
     --build`), confirming the correct device role per
     `[[verify-microbit-before-flashing]]`.
  2. On the stand (wheels off the ground): confirm encoders/sensors are
     alive, spin the wheels both directions, confirm no wedge and no
     commanded terminal reversal beyond the motor armor's own window.
  3. Move the robot to the playfield (radio relay path) and run
     `tests/bench/turn_sweep.py --relay --both` across the grid.
- **Postconditions**: SUC-001's acceptance criteria are confirmed on real
  hardware, not sim alone; any failure is reported explicitly (which step,
  what was observed) rather than summarized as a single pass/fail.
- **Acceptance Criteria**:
  - [ ] `just build-clean` succeeds; sim suite green immediately beforehand.
  - [ ] Stand check: sensors alive, wheels drive both directions, encoders
        increment as expected, no wedge.
  - [ ] Playfield `turn_sweep.py --relay --both` run recorded, with a
        pass/fail verdict against the ≈±1° goal tolerance stated explicitly.
  - [ ] The USB-reflash-then-playfield-relay two-location dependency is
        called out explicitly in the ticket's own completion notes (this is
        not a same-location, one-session gate).
