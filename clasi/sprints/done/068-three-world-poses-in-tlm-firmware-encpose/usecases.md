---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 068 Use Cases

Parent: `docs/usecases.md` UC-006 "Query and Zero Dead-Reckoning Odometry"
(the dead-reckoning-pose use case, pre-dating protocol v2's `TLM`
streaming) and UC-012 "Initialize and Read OTOS Sensor" (the precedent for
a raw-sensor pose alongside a fused one). Neither UC anticipated three
side-by-side world poses on one wire frame; these SUCs narrow/extend both
rather than mint a new top-level UC (see architecture-update.md Open
Question 4 for the stakeholder call on whether a new UC-020 is warranted
instead).

## SUC-001: Firmware streams an encoder-only dead-reckoned world pose alongside OTOS and fused poses
Parent: UC-006, UC-012

- **Actor**: Python host (TestGUI, bench script, sim test, or the sprint-069
  hardware-fit tooling this sprint unblocks).
- **Preconditions**: Robot firmware (or sim) is running; telemetry streaming
  is active (`STREAM <ms>`, no `fields=` narrowing) or a `SNAP` is issued.
- **Main Flow**:
  1. Every control tick, `Odometry::predict()` updates its encoder-only
     accumulator (`ActualState.encoder`) from wheel deltas alone, using the
     live trackwidth/rotational-slip calibration (sprint 067) — unchanged
     behavior, already true before this sprint.
  2. `Robot::buildTlmFrame()` reads `drive.state().encoder.pose.{x,y,h}`
     and emits `encpose=<x>,<y>,<h>` (mm, mm, centidegrees) whenever
     `TLM_FIELD_ENCPOSE` is subscribed.
  3. The same TLM frame also carries `otos=` (raw OTOS) and `pose=` (fused
     EKF), each computed independently.
  4. The host receives one TLM line carrying all three world poses plus the
     raw `enc=` counts they were derived from.
- **Postconditions**: A single telemetry frame lets a consumer compare
  encoder-only, optical, and fused pose estimates without any host-side
  re-derivation.
- **Acceptance Criteria**:
  - [ ] `TLM` frames (STREAM and SNAP) carry `encpose=`, `otos=`, and
        `pose=` in the same line whenever all three are subscribed and
        their source data is available.
  - [ ] `encpose=` reflects the firmware's own encoder-only accumulator —
        not a host-side approximation — and is NOT corrected by the EKF or
        OTOS.
  - [ ] `encpose=` continues accumulating across a per-drive-command
        encoder-count zeroing (`D`, `ZERO enc`) — it is re-referenced only
        by explicit pose-set commands (`SI`/`OV`/`ZERO pose`), identically
        to how `pose=`/`otos=` are already re-referenced only by those same
        commands.
  - [ ] The default TLM field subscription (`STREAM <ms>` with no
        `fields=` clause — what TestGUI and every existing sim test use)
        includes `encpose=` without any caller having to opt in.
  - [ ] `docs/protocol-v2.md` documents the new field.

## SUC-002: TestGUI plots the encoder trace directly from the wire pose, with no host-side re-integration
Parent: UC-006

- **Actor**: TestGUI operator viewing the four-trace pose comparison
  (camera/encoder/otos/fused) during a manual or `TOUR_1` run.
- **Preconditions**: TestGUI is connected (serial, radio, or sim) and
  receiving `TLM` frames carrying `encpose=`.
- **Main Flow**:
  1. `TraceModel.feed()` receives a parsed `TLMFrame` with `frame.encpose`
     populated.
  2. `TraceModel` treats `encpose=` exactly as it already treats
     `otos=`/`pose=`: an absolute world-frame pose, baselined once, rotated
     into the display frame by the fixed anchor-to-firmware-heading offset.
  3. The `encoder` trace polyline renders, staying continuous across any
     `D`/`ZERO enc` mid-session — with no reset detection, no trackwidth
     mirroring, and no turn-scrub compensation performed host-side.
  4. The Sim Errors panel (encoder noise, turn slip, OTOS noise) and the
     robot-config calibration fields (`tw`, `rotational_slip`) remain
     exactly as they were: user-settable, and consumed only by the
     firmware/simulator — never by `TraceModel`.
- **Postconditions**: The encoder trace is exactly as reliable as the
  otos/fused traces already are — neither more nor less — because all
  three now share one code path.
- **Acceptance Criteria**:
  - [ ] `TraceModel` has no method named `set_trackwidth_mm`,
        `set_turn_scrub_factor`, or `notify_reset_pending`, and no
        `_feed_encoder` re-integration method.
  - [ ] `Transport.on_reset_pending` and `is_reset_inducing_command` are
        removed from `transport.py`; no call site references them.
  - [ ] `Transport.turn_scrub_factor` and the Sim Errors panel's
        apply/read path are unchanged and still pass their existing tests.
  - [ ] A slow-TLM (relay-rate, ~1-2 Hz) session shows a correct,
        un-skipped encoder trace across a `D` command boundary — the
        CR-09 failure mode is structurally impossible (there is no
        reset-detection heuristic left to miss a reset).

## SUC-003: Zero-injected-error regression proves all three wire poses and plant truth agree
Parent: UC-006, UC-014 ("Tune Calibration Parameters at Runtime" — the sim
error-injection knobs this SUC sets to zero)

- **Actor**: CI / any engineer running the default pytest suite.
- **Preconditions**: Sim error injection (encoder noise, turn-slip scrub,
  OTOS noise/drift) is set to zero; the robot drives a representative
  multi-leg maneuver (straight legs and turns, matching the shape of the
  TestGUI's `TOUR_1` sequence).
- **Main Flow**:
  1. Test configures the sim with all injected-error knobs at zero
     (`sim.set_field_profile(slip_turn_extra=0.0, ...)` and equivalent
     zeroing of OTOS noise/drift).
  2. Test drives the maneuver, collecting TLM frames via
     `tick_collect_tlm`.
  3. For each frame, `frame.encpose`, `frame.otos`, and `frame.pose` are
     compared against `sim.get_true_pose()` (plant ground truth) at the
     same tick.
  4. All four agree within a small numeric tolerance throughout the
     maneuver — with zero injected error, there is nothing left for the
     three estimators to disagree about.
- **Postconditions**: A future regression in any of the three pose
  estimators, or in the TLM wire encoding of any of them, fails this test
  specifically — not silently, and not only when noise happens to be
  nonzero.
- **Acceptance Criteria**:
  - [ ] New sim regression test (tier: `tests/simulation/system/`) passes
        with all three wire poses and plant truth in agreement across the
        full maneuver, including at least one turn.
  - [ ] `tests/_infra/golden_tlm_capture.json` is regenerated and
        `test_golden_tlm_unchanged` passes with `encpose=` present in every
        frame of the fixed sequence.
  - [ ] Full default pytest suite stays green (baseline after 067: 2520
        passed, 0 failed, plus this sprint's new/modified tests).

## SUC-004: An operator can exclude `encpose=` from a bandwidth-constrained stream
Parent: UC-019 ("Radio Relay Mode")

- **Actor**: Python host operating over a bandwidth-constrained link
  (radio/relay).
- **Preconditions**: Firmware supports the widened 9-field TLM
  subscription bitmask.
- **Main Flow**:
  1. Host sends `STREAM fields=pose,otos,vel` (or any subset omitting
     `encpose`).
  2. Firmware's `OK stream fields=...` echo confirms `encpose` is not in
     the active subscription.
  3. Subsequent TLM frames omit `encpose=` entirely, saving ~26-29 bytes
     per frame.
- **Postconditions**: Bandwidth-sensitive deployments retain the same
  opt-out control they already have for every other TLM field; nothing
  about `encpose=`'s inclusion is hard-coded or unconditional.
- **Acceptance Criteria**:
  - [ ] `STREAM fields=...,encpose` and `STREAM fields=...` (omitting
        `encpose`) both round-trip correctly through `handleStream`'s
        parse-and-echo path.
  - [ ] `STREAM` with no `fields=` clause (the default used by TestGUI and
        the existing sim test suite) still includes `encpose=` — this SUC
        extends the opt-out mechanism without changing the default
        established in SUC-001.
