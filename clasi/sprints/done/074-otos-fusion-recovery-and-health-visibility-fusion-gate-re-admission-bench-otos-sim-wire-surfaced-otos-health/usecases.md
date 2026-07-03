---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 074 Use Cases

Parent reference: `docs/usecases.md` UC-012 (Initialize and Read OTOS
Sensor). UC-012 predates the ordered-tick architecture, the EKF fusion gate,
and the wire telemetry surface — the parent link is for traceability of
intent (the OTOS sensor's readings must be usable to correct the robot's
pose), not literal command-syntax equivalence.

All five use cases below trace to
`clasi/issues/otos-not-used-frozen-pose-ekf-rejects-everything.md`. Two
findings from this sprint's investigation reframe what the issue's own
"investigation pointers" suspected:

- `Robot::otosCorrect()` (the function the issue names as the place to
  check the fusion gate) has had **zero call sites in the live control
  loop** since the ordered-tick cutover (sprint 060). The sole live
  OTOS-read-and-fuse path is `Drive::tickUpdate()` STEP 5 /
  `Drive::_updateOtosFusionGate`. SUC-002 and SUC-003 below target the live
  path; `Robot::otosCorrect()` is addressed only as an Open Question in
  `architecture-update.md` (documentation/cleanup, not a behavior fix).
- The CR-06 warn-persistence gate (sprint 065) already re-admits fusion
  after a run of clean ticks — it does not "latch forever" as the issue
  speculated, and this is already regression-tested
  (`tests/simulation/unit/test_otos_warn_persistence.py::test_clean_streak_readmits_fusion_after_block`).
  The gap this sprint closes is different and more specific: the gate only
  watches the OTOS chip's self-reported STATUS bits, so a reading that is
  readable, self-reports clean, and simply stops changing value sails
  through undetected and gets fused — which is what a climbing `ekf_rej`
  counter alongside a frozen `otos=` actually implies (see SUC-003).

## SUC-001: Bench-mode OTOS simulation tracks commanded motion in both sim and firmware
Parent: UC-012

- **Actor**: Firmware/host developer exercising `DBG OTOS BENCH` (via
  `testkit.make_target("bench")`, `testkit.make_target("sim")` with the
  default `sim_otos=True`, or a physical stand session).
- **Preconditions**: `DBG OTOS BENCH 1` has been sent. The robot (real or
  simulated) is being driven by a motion command.
- **Main Flow**:
  1. The active `Hardware` implementation (`NezhaHAL`/`MecanumHAL` on real
     firmware, `SimHardware` in host-sim) redirects `otos()` to a
     bench-simulated odometer instead of the real/ground-truth one.
  2. Each control tick, the bench-simulated odometer integrates the
     commanded wheel velocities into its own pose accumulator.
  3. Any reader of `hal.otos()` — the live fusion/telemetry path, `DBG
     OTOS`, `SIMGET`-style test hooks — observes the bench sensor's moving
     pose, not a frozen value.
- **Postconditions**: A bench-mode session (real stand or host-sim) shows a
  simulated OTOS pose that visibly advances with commanded motion, matching
  the behavior a physical, correctly-tracking OTOS chip would produce.
- **Acceptance Criteria**:
  - [ ] `SimHardware` gains the same HAL-level bench-otos swap
        (`setOtosBench`/`otos()`/`benchOtosPtr()`) that `NezhaHAL`/
        `MecanumHAL` already implement — reachable uniformly through
        `Hardware`, no per-build `#if`/downcast at the call site.
  - [ ] A sim test: enable `DBG OTOS BENCH 1`, drive, and assert the
        bench-simulated pose is non-zero and tracks the commanded arc
        (matches the noiseless `BenchOtosSensor` ideal-accumulator math).
  - [ ] `DBG OTOS`'s `ideal=`/`otos=` reply fields, currently hardcoded to
        `0,0,0` under `HOST_BUILD`, reflect the real bench-sensor
        accumulators in sim, the same as they do in a `BENCH_OTOS_ENABLED`
        firmware build.

## SUC-002: The live OTOS fusion/telemetry path always reads the currently active odometer
Parent: UC-012

- **Actor**: Firmware control loop (`Drive::tickUpdate()` STEP 5).
- **Preconditions**: `DBG OTOS BENCH` has toggled the active odometer at
  runtime (real hardware) — or, once SUC-001 lands, in host-sim.
- **Main Flow**:
  1. `Drive::tickUpdate()`'s STEP 5 OTOS-correction block resolves the
     active odometer through the same `Hardware`-level indirection every
     tick, rather than a reference captured once when `Drive` was
     constructed at boot.
  2. When bench mode is toggled on or off mid-session, the very next tick's
     read reflects the newly active sensor.
- **Postconditions**: `Drive`'s fused pose and TLM `otos=` track whichever
  odometer `Hardware::otos()` currently reports, matching the contract
  `Robot::otosCorrect()` already implements for its (otherwise dead)
  code path.
- **Acceptance Criteria**:
  - [ ] A sim test (built on SUC-001's bench-otos substrate) toggles `DBG
        OTOS BENCH 1` mid-session and confirms `otos=`/fused pose switch to
        the bench sensor's simulated motion on the next tick — this test
        must fail against the pre-fix `Drive` (which keeps reading the
        sensor bound at construction).
  - [ ] Toggling bench mode back off similarly restores the previously
        active sensor on the next tick.
  - [ ] No behavior change for any session that never toggles bench mode
        (the default-bound sensor is unchanged).

## SUC-003: A readable-but-stuck OTOS reading is excluded from EKF fusion
Parent: UC-012

- **Actor**: Firmware EKF fusion pipeline (`Drive::_updateOtosFusionGate`).
- **Preconditions**: The OTOS chip reports a successful read and a clean
  STATUS byte (`otosStatus == 0`) every tick, but the reported pose does
  not change from one tick to the next while the encoders show the robot is
  moving (the same-value-repeated signature observed in
  `recordings/latest.jsonl`: frozen `otos=`, `ekf_rej` climbing every
  tick — proof that the current gate does not catch this case, since a
  blocked gate would stop feeding the EKF and `ekf_rej` would stop
  climbing).
- **Main Flow**:
  1. Each tick, alongside the existing STATUS-bit warn-streak check,
     `Drive` compares the newly read OTOS pose against the previous tick's
     value while encoder motion is present.
  2. A persistently unchanged reading (over the same window as an ordinary
     STATUS-bit warning) is treated as a warning input to the existing
     `_otosFusionBlocked` state machine — it does not require a new,
     parallel gate.
  3. Once blocked, `addOtosObservation` is skipped (as it already is for a
     STATUS-bit block); `ekf_rej` stops climbing because the EKF is no
     longer being fed the frozen reading.
  4. Once the reading resumes changing for the existing re-admission window
     (`kOtosCleanReadmitN` consecutive ticks), fusion resumes automatically
     — reusing the CR-06 re-admission path unchanged.
- **Postconditions**: A stuck-but-status-clean OTOS reading no longer feeds
  garbage into the EKF indefinitely; the existing re-admission behavior
  (already regression-tested for the STATUS-bit case) also covers this
  case without new re-admission logic.
- **Acceptance Criteria**:
  - [ ] Sim test: inject a fixed (never-updated) OTOS pose via the existing
        `sim_set_otos_pose`/fusion-bypass hooks while driving (STATUS stays
        clean); after the persistence window, `_otosFusionBlocked` becomes
        true and `ekf_rej` stops incrementing.
  - [ ] The same test then updates the injected pose again (simulating
        recovery) and confirms fusion is re-admitted after
        `kOtosCleanReadmitN` ticks, matching the existing STATUS-bit
        re-admission test's shape.
  - [ ] A robot that is legitimately stationary (no encoder motion) with an
        unchanging OTOS reading is NOT flagged as stuck — the check is
        conditioned on encoder-evidenced motion, not on pose value alone.
  - [ ] Existing `test_otos_warn_persistence.py` tests are unaffected
        (the STATUS-bit path is untouched; the new check is an additional
        input to the same state machine).

## SUC-004: OTOS fusion health is visible on the wire independent of `ekf_rej`
Parent: UC-012

- **Actor**: Any TLM consumer (TestGUI, bench script, log analysis).
- **Preconditions**: `STREAM`/`SNAP` telemetry is active.
- **Main Flow**:
  1. Every TLM frame (when the corresponding field bit is enabled — on by
     default) includes the raw OTOS STATUS byte and the current
     `_otosFusionBlocked` state, unconditional on `otos=`'s own freshness
     gate (matching the existing `wedge=` field's "always visible,
     poll-safe" precedent).
  2. A host reading this field can directly answer "is OTOS fusion
     currently blocked, and why" without inferring it from a climbing
     `ekf_rej` counter or a suspiciously static `otos=` value.
- **Postconditions**: The failure mode this sprint's issue describes (an
  operator only able to guess, after the fact, that fusion was silently
  degraded) is diagnosable live and from a recording, without guesswork.
- **Acceptance Criteria**:
  - [ ] A new TLM field is added, gated by a new `TLM_FIELD_*` bit
        (on by default), emitted unconditionally when enabled (no freshness
        gate).
  - [ ] Host `parse_tlm`/`TLMFrame` (`host/robot_radio/robot/protocol.py`)
        parses the new field.
  - [ ] `tests/_infra/golden_tlm_capture.json` is regenerated to include
        the new field; `test_golden_tlm.py` passes against the regenerated
        capture.
  - [ ] A sim test drives the gate into the blocked state (via SUC-003's or
        the existing STATUS-bit injection) and asserts the new field
        reflects it.

## SUC-005: `otos=` telemetry semantics are documented and verified failure-safe
Parent: UC-012

- **Actor**: Firmware maintainer / anyone debugging a future OTOS
  telemetry anomaly.
- **Preconditions**: None — this is a documentation and regression-guard
  use case closing out the issue's fourth investigation pointer ("determine
  what TLM `otos=` actually reflects: raw read vs last-accepted").
- **Main Flow**:
  1. `otos=` is documented (in code comments and this sprint's architecture
     record) as reflecting the most recent RAW, successfully-read pose from
     whichever odometer is currently active — independent of whether that
     reading was admitted into EKF fusion (i.e. `otos=` does not go stale
     or change meaning when `_otosFusionBlocked` is true; SUC-004's new
     field is what tells the host fusion is blocked).
  2. A read failure (`poseOk == false`) clears the freshness envelope
     (`otos.valid = false`) the same tick, so `otos=` disappears from TLM
     (via the existing N8 freshness gate) rather than repeating a stale
     value forever.
- **Postconditions**: No stale-cache-masks-a-read-failure defect exists in
  the raw OTOS telemetry path; this is verified, not assumed.
- **Acceptance Criteria**:
  - [ ] A regression test asserts that once `sim_set_otos_read_failure(1)`
        is set, `otos=` disappears from the next TLM frame after the
        freshness window elapses, and does not reappear with a stale value.
  - [ ] Code comments at the `otos=` emission site
        (`RobotTelemetry.cpp`) and at `Drive::tickUpdate()` STEP 5
        cross-reference this use case and SUC-004's health field.
