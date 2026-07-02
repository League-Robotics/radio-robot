---
status: pending
severity: medium
sprint: 068
---

# TLM must carry three world poses: encoder-only, OTOS, and EKF — add the missing encoder-only pose

## Problem

Telemetry currently transmits two integrated poses — `otos=` (raw OTOS pose)
and `pose=` (fused EKF estimate) — plus raw cumulative wheel distances
(`enc=left_mm,right_mm`). There is **no encoder-only dead-reckoned pose** on
the wire. The robot is the only party that can integrate the encoders
correctly (it owns the trackwidth and slip calibration and sees every sample
at tick rate), yet the integration is currently reconstructed host-side:
`TraceModel._feed_encoder` in
[host/robot_radio/testgui/traces.py](../../host/robot_radio/testgui/traces.py)
re-derives a pose from the two raw counts with its own trackwidth constant, a
reset-detection heuristic, and a turn-scrub knob synced to the *simulator's*
injected error rather than the firmware's calibration.

That host-side integrator is a defect factory:

- 2026-07-01: missed firmware encoder resets cancelled turn headings
  ("encoder track ignores turns").
- CR-09 (sprint 066): the reset heuristic still misses resets on slow TLM
  (relay, ~1–2 Hz).
- 2026-07-02: the GUI scrub knob desynced from the sim's injected slip
  (Sim-Errors Apply reconfigures the sim but not the trace integrator),
  producing a wildly rotated encoder trace with all sim errors set to zero.

## Requirement

The firmware computes and transmits an encoder-only dead-reckoned world pose
in TLM, so telemetry carries **three world poses**:

1. `encpose=` (new) — pose integrated purely from wheel encoders,
   using the firmware's own geometry/slip calibration.
2. `otos=` (exists) — raw OTOS sensor pose.
3. `pose=` (exists) — fused EKF estimate.

Hosts (TestGUI traces, bench scripts, the hardware-fit regression tooling in
[sim-error-model-runtime-settable-hardware-fit.md](sim-error-model-runtime-settable-hardware-fit.md))
become dumb plotters of the three poses. The TestGUI host-side encoder
integrator is deleted — meaning `TraceModel._feed_encoder`'s re-integration
of raw counts, its private trackwidth copy, its internal
`set_turn_scrub_factor` compensation factor, and the reset heuristic. These
are hidden display-layer internals, NOT user-facing controls.

**Explicitly retained (stakeholder requirement):** the user-facing
configuration surfaces are untouched by this deletion —

- the Sim Errors panel (encoder noise, turn slip, OTOS noise) remains the
  way the operator sets the *injected plant errors*; it is expanded, not
  removed, by the sim-error-model issue linked above;
- trackwidth and the robot's scrub *calibration* (`rotational_slip`) remain
  robot configuration values (robot config file / `SET tw`, `SET rotSlip`),
  consumed by the firmware integrator that now produces `encpose=`
  (propagation fix: [set-config-not-propagated-to-planner.md](set-config-not-propagated-to-planner.md)).

## Notes / constraints

- The firmware already has the integration machinery (`Odometry::predict`);
  this is exposing an encoder-only variant of it, not new math.
- The encoder-only pose must NOT be EKF/OTOS-corrected and must survive the
  per-drive-command encoder zeroing (continuous accumulation, re-referenced
  only by explicit pose-set commands like `SI`/`ZERO`).
- Relay bandwidth is tight (~12 msg/s radio); adding a third pose to every
  TLM line must be weighed — options: emit in STREAM TLM only, alternate
  fields between frames, or gate behind a TLM verbosity flag.
- Supersedes the CR-09 host-side fix direction in
  [testgui-trace-correctness-slow-tlm-and-anchor-rotation.md](testgui-trace-correctness-slow-tlm-and-anchor-rotation.md)
  part (a): with `encpose=` on the wire, the reset heuristic ceases to exist.

## Acceptance

- TLM (at least in streamed form) carries encoder-only, OTOS, and EKF poses.
- Sim golden-TLM / protocol tests updated; `parse_tlm` exposes the new field.
- TestGUI encoder trace plots `encpose=` directly; the internal host-side
  integration code (raw-count re-integration, private trackwidth/scrub
  compensation, reset heuristic) is removed. The Sim Errors panel and all
  robot-config calibration values (trackwidth, rotational_slip) are
  unaffected and remain user-settable.
- With zero injected sim error, all three wire poses and plant ground truth
  agree over Tour 1 (regression test in sim).
