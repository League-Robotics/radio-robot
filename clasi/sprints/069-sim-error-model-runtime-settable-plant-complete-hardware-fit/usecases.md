---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 069 Use Cases

Sim-scoped slice only (per sprint.md Scope). The end-to-end hardware
record→fit→replay demo (issue
`sim-error-model-runtime-settable-hardware-fit.md`'s ultimate acceptance) needs
a physical robot and is deferred to a follow-up HIL sprint — see Open
Questions in `architecture-update.md`. SUC-008 below covers only the
sim-to-sim-validated slice of the fit tool.

Two of these SUCs narrow a **new** top-level use case not yet in
`docs/usecases.md` (proposed **UC-020: Configure Simulator Plant/Error Model
at Runtime**, and **UC-021: Fit Simulator Error Parameters to a Recorded
Run**) rather than an existing one — UC-014 ("Tune Calibration Parameters at
Runtime") is about the *robot's* K*/SET-able calibration, a different actor
concern (operating the robot) from configuring the *simulator's* plant/error
model (a test/development-tooling concern with no real-hardware analog).
SUC-001 is the one exception: EKF fusion noise is a genuine firmware
parameter (applies identically on real hardware and sim), so it narrows
UC-014 directly, matching the precedent sprint 067 set. Flagged for the
stakeholder to confirm at consolidation time (mirrors 068's Open Question 4
on the same UC-mint-vs-narrow question).

---

## SUC-001: Tune EKF Fusion Noise at Runtime
Parent: UC-014 (narrows — real firmware parameter, not sim-only)

- **Actor**: Developer / calibration tooling (Python host via `SET`)
- **Preconditions**: Robot firmware (real or sim) is running; `Drive`'s live
  `setNoise()` push path exists (landed sprint 067, currently fed only by the
  one already-registered `ekfRHead` key).
- **Main Flow**:
  1. Host sends `SET ekfQxy=<v>` (or any of the six sibling EKF
     process/measurement-noise keys: `ekfQtheta`, `ekfQv`, `ekfQomega`,
     `ekfROtosXy`, `ekfROtosV`, `ekfREncV`).
  2. `ConfigRegistry::handleSet` commits the field to the live `RobotConfig`
     and, because the key is annotated `"drive"`, calls `Drive::configure()`.
  3. `Drive::configure()` calls `PhysicalStateEstimate::setNoise(...)` with
     all eight live EKF fields — noise-only update, does not reset pose or
     covariance (067's `EKFTiny::setNoise()` contract).
  4. `GET ekfQxy` reads back the committed value.
- **Postconditions**: The named EKF noise parameter is live in the running
  fusion filter without disturbing in-flight pose/covariance.
- **Acceptance Criteria**:
  - [ ] All seven currently-unregistered EKF noise keys (`ekfQxy`,
        `ekfQtheta`, `ekfQv`, `ekfQomega`, `ekfROtosXy`, `ekfROtosV`,
        `ekfREncV`) are `SET`/`GET`-able.
  - [ ] `SET`-ting any one of them changes EKF fusion behavior observably
        (e.g., a test that raises `ekfROtosXy` and shows the fused pose
        trusting OTOS less on the next correction).
  - [ ] No existing EKF-state/covariance disturbance: a `SET` of any of
        these keys mid-mission does not teleport the fused pose.

---

## SUC-002: Configure Per-Wheel Encoder Report Error at Runtime
Parent: UC-020 (new)

- **Actor**: Developer / fit tooling (Python host, via the sim's wire-command
  surface)
- **Preconditions**: Sim is running (`Sim()` / `SimTransport`).
- **Main Flow**:
  1. Host sends `SIMSET encScaleErrL=<f> encScaleErrR=<f>` (or `encSlipL/R`,
     `encNoiseL/R`).
  2. The sim's `SimCommands` registry dispatches each key to the matching
     `PhysicsWorld` setter (`setEncoderScaleError`, `setEncoderSlip`,
     `setEncoderNoise`) — all three already exist (ticket 058-001 / earlier);
     only the wire path and getters are new this sprint.
  3. `SIMGET encScaleErrL` reads back the configured value.
- **Postconditions**: The reported (not true) per-wheel encoder accumulator
  reflects the configured scale error / slip / noise on subsequent ticks.
- **Acceptance Criteria**:
  - [ ] Each of the six per-wheel encoder-report keys is `SIMSET`/`SIMGET`-able.
  - [ ] Setting `encScaleErrL` (only) produces a visible left/right encoder
        divergence in TLM `enc=` without moving `encpose=`/`otos=`/`pose=`
        away from what an unaffected run would show for the true trajectory.

---

## SUC-003: Configure Body/Chassis-Truth Scrub Independent of Encoder Reporting
Parent: UC-020 (new)

- **Actor**: Developer / fit tooling
- **Preconditions**: Sim is running.
- **Main Flow**:
  1. Host sends `SIMSET bodyRotScrub=0.92` (or `bodyLinScrub=<f>`).
  2. `PhysicsWorld` applies the new, independent
     `_bodyRotationalScrub`/`_bodyLinearScrub` factors in sub-step B (chassis
     pose integration) only — the reported/true encoder accumulators
     (sub-step A/A′) are untouched.
  3. A subsequent `RT 9000` (in-place 90° turn) command's true rotation
     reflects the configured scrub.
- **Postconditions**: The plant's true pose genuinely under-rotates/under-
  travels relative to naive wheel-arc kinematics, by the configured factor —
  a capability the plant did not have before this sprint (previously
  `effectiveSlip(<=0)` always clamped to 1.0/no-scrub in practice).
- **Acceptance Criteria**:
  - [ ] `SIMSET bodyRotScrub=0.92` (all else default; `RobotConfig.rotationalSlip`
        at its 0.92 default) → `RT 9000` lands on 90° true pose in sim
        (closing the current ~95.2° gap).
  - [ ] `SIMSET bodyRotScrub=1.0 bodyLinScrub=1.0` (defaults) plus
        `SET rotSlip=1.0` → `RT 9000` lands on exactly 90° true pose.
  - [ ] The 066-001 chassis-truth-slip test
        (`test_turn_with_slip_otos_matches_truth_encoder_diverges`, which
        configures the *existing* `sim_set_motor_slip`/`_rotationalSlip`
        channel) is unaffected — the two mechanisms combine multiplicatively
        and default-neutral.

---

## SUC-004: Configure Simulator Plant Geometry and Actuation Asymmetry at Runtime
Parent: UC-020 (new)

- **Actor**: Developer / fit tooling
- **Preconditions**: Sim is running.
- **Main Flow**:
  1. Host sends `SIMSET trackwidthMm=<f>` (the plant's TRUE physical
     trackwidth, independent of `RobotConfig.trackwidthMm`/`tw`, the
     firmware's belief) or `motorOffsetL=<f>`/`motorOffsetR=<f>` (per-side
     actuation gain asymmetry).
  2. `SimCommands` dispatches to `SimHardware::setTrackwidth()` /
     `PhysicsWorld::setOffsetFactor()` — both already exist.
- **Postconditions**: A mismatch between the plant's true geometry/actuation
  and the firmware's configured belief is reproducible for fit-tool testing.
- **Acceptance Criteria**:
  - [ ] `trackwidthMm`, `motorOffsetL`, `motorOffsetR` are `SIMSET`/`SIMGET`-able.
  - [ ] Setting `trackwidthMm` to a value different from `GET tw`'s reading
        produces a measurable heading-rate discrepancy between the plant's
        true rotation and the firmware's own commanded/expected rotation.

---

## SUC-005: Configure OTOS Sensor Error at Runtime
Parent: UC-020 (new)

- **Actor**: Developer / fit tooling
- **Preconditions**: Sim is running; `SimOdometer`'s sim model enabled.
- **Main Flow**:
  1. Host sends `SIMSET otosLinScaleErr=<f> otosAngScaleErr=<f>
     otosLinDriftMmS=<f> otosYawDriftDegS=<f>` (existing `SimOdometer`
     setters `setLinearScaleError`/`setAngularScaleError`/
     `setDriftPerTickMm`/`setDriftPerTickRad`; drift keys are specified
     per-second on the wire and converted to per-tick internally using the
     sim's control period, matching issue-1's plumbing guidance) — plus the
     already-partially-exposed `otosLinNoise`/`otosYawNoise`.
  2. `SimOdometer`'s accumulator diverges from plant truth by the configured
     amount on subsequent ticks.
- **Postconditions**: OTOS-vs-truth divergence is reproducible and readable
  back.
- **Acceptance Criteria**:
  - [ ] All six OTOS error keys are `SIMSET`/`SIMGET`-able (getters added
        where only a setter existed before).
  - [ ] Setting `otosLinScaleErr` alone changes `otos=`'s reported distance
        relative to `sim.get_true_pose()` without perturbing `encpose=`.

---

## SUC-006: Read Back Current Simulator Error-Model Configuration
Parent: UC-020 (new)

- **Actor**: Developer / fit tooling
- **Preconditions**: Sim is running.
- **Main Flow**:
  1. Host sends `SIMGET` with no arguments.
  2. Every registered `SimCommands` key is dumped, one `SIMCFG key=value…`
     reply line — mirroring `GET`'s no-argument dump-all behavior.
- **Postconditions**: The fit tool (SUC-008) can snapshot the sim's current
  configuration before/after a fit run without needing per-field ctypes
  calls.
- **Acceptance Criteria**:
  - [ ] `SIMGET` with no args dumps all registered sim-error keys.
  - [ ] `SIMGET <key>…` returns only the named keys; an unknown key returns
        `ERR badkey <key>` (mirrors `GET`'s per-key error shape).

---

## SUC-007: Operate the Full Sim-Error Knob Set from the TestGUI
Parent: UC-020 (new)

- **Actor**: TestGUI operator
- **Preconditions**: TestGUI connected in Sim mode.
- **Main Flow**:
  1. Operator opens the "Sim Errors" panel, now showing the existing four
     knobs (encoder noise, turn slip, OTOS linear/yaw noise) plus every
     newly-surfaced knob from SUC-002 through SUC-005, grouped (Encoder
     Report Error / Body-Truth Scrub / Geometry & Actuation / OTOS Error).
  2. Operator edits a field and clicks Apply.
  3. `transport.py::apply_error_profile()` builds and sends one `SIMSET
     k1=v1 k2=v2 …` wire command (not per-field ctypes calls) to the
     connected sim.
- **Postconditions**: Every documented sim error parameter is reachable from
  the GUI without any bespoke ctypes wrapper existing for it.
- **Acceptance Criteria**:
  - [ ] The panel exposes the existing 4 knobs plus all knobs from
        SUC-002/003/004/005.
  - [ ] Apply sends a single `SIMSET` command built from the full profile
        dict; defaults (0.0 error / 1.0 scrub) reproduce today's
        no-op-until-opted-in behavior exactly.
  - [ ] The profile persists to `data/testgui/sim_error_profile.json`
        (existing mechanism, extended with the new keys).

---

## SUC-008: Fit Simulator Error Parameters to a Recorded Run (Sim-to-Sim Validated)
Parent: UC-021 (new)

- **Actor**: Developer running the fit tool
- **Preconditions**: A recorded run exists as a sequence of (timestamp,
  command, `encpose=`/`otos=`/`pose=`) samples — for this sprint, produced by
  a *sim* run with known injected `SIMSET` parameters (real-hardware Tour-1
  recording is out of scope, deferred — see Open Questions).
- **Main Flow**:
  1. Developer runs the fit script against the recorded run, naming the
     deterministic/bias-shaped parameter subset to regress (scale errors,
     drift, body scrub, trackwidth, actuation offset — NOT noise sigmas,
     which don't bias the mean trajectory).
  2. The script replays the recorded command sequence against a fresh,
     zero-error sim instance for each candidate parameter vector, samples
     `encpose`/`otos`/`pose` at the recorded timestamps, and minimizes
     trajectory disagreement (least-squares) against the recorded run.
  3. The script emits a parameter file (`SIMSET`-key → fitted value) the sim
     can load via a batch of `SIMSET` commands.
- **Postconditions**: The fitted parameter file's values are within a stated
  tolerance of the run's true injected parameters.
- **Acceptance Criteria**:
  - [ ] Given a recording produced by a sim run with known injected
        `SIMSET` values (e.g., `bodyRotScrub=0.90`, `encScaleErrL=0.03`), the
        fit recovers each injected parameter within a stated tolerance.
  - [ ] The emitted parameter file, loaded into a fresh sim via `SIMSET`,
        reproduces the recorded trajectory within a stated tolerance.
  - [ ] Real-hardware record→fit→replay (the issue's ultimate acceptance) is
        explicitly out of this sprint — flagged as a follow-up HIL task.
