---
id: '007'
title: Enable OTOS fusion in PoseEstimator (bench-gated hazard close)
status: open
use-cases: [SUC-003]
depends-on: ['002', '004', '005', '006']
github-issue: ''
issue: restore-pose-estimation-otos-encoders-delayed-camera-fixes.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Enable OTOS fusion in PoseEstimator (bench-gated hazard close)

## Description

Every prerequisite is now in place: OTOS ticks live and safely (002),
`PoseEstimator` runs encoder-only with an accurate joint predict (004,
005), and `EkfTiny`'s innovation gate protects against a momentarily-
disagreeing sensor (006). This ticket is the "one-token flip" the driving
issue itself names it: `MainLoop::tick()`'s `poseEstimator_.tick(...)`
call stops passing a literal `nullptr` for `otosObs` and instead assembles
a real observation, gated on `Hal::Odometer::fusableThisPass()` — the
ONE-SHOT, read-and-clear signal whose own doc comment
(`hal/capability/odometer.h`) already names this exact call site as its
sanctioned caller.

This is the ticket that closes the frozen-fused-pose hazard
(`clasi/issues/poseestimator-fused-pose-frozen-on-hardware.md`) for real —
the ungated-fusion hazard never ships (fusion stays `nullptr` until this
ticket, which itself does not land without ticket 006's gate already in
place, per its `depends-on`).

## Acceptance Criteria

- [ ] `MainLoop::tick()` calls `hardware_.odometer()->fusableThisPass()`
      EXACTLY ONCE per pass (never a second call anywhere else in this
      pass) and reads `hardware_.odometer()->pose()` into a local sample.
- [ ] `otosObs` passed to `poseEstimator_.tick(...)` is `&otosSample` when
      `fusable && otosSample.stamp.valid`, else `nullptr` — matching
      `pose_estimator.cpp`'s own existing consumption gate
      (`otosObs->stamp.valid`) exactly, no redundant re-check.
- [ ] `MainLoop::commit()` gains `bb.otosValid = fusable;` (the value
      captured this pass, before it was consumed) — completing the
      cell architecture-update.md's D1 pseudocode already specifies.
- [ ] `PoseEstimator::configure()`'s existing EKF-noise wiring
      (`ekf_q_xy`/`ekf_q_theta`/`ekf_r_otos_xy`/`ekf_r_otos_theta`, already
      implemented) needs no change — verify by reading, not assuming.
- [ ] Full sim suite passes, including ticket 006's gate-characterization
      harness now exercised end-to-end through `PoseEstimator::tick()`
      with a real (sim) OTOS observation present.
- [ ] **BENCH MANDATORY**: on the stand, `fusedPose` (`pose=` on TLM)
      does NOT freeze at a stale value nor drag toward the origin while
      the robot's wheels are driven (OTOS reports near-zero translation
      on the stand — wheels off the ground — while encoders accumulate;
      `fusedPose` should track the encoder-driven belief, not the static
      OTOS reading, per the gate's design and this sprint's own bench-
      gate note on expected stand-vs-floor divergence).
  - [ ] Document explicitly, in the completion notes, what "sane" meant
        for this specific bench session (expected divergence between
        `fusedPose` and `otos=` on the stand is NORMAL — wheels-off-ground
        is not representative of floor/playfield conditions; the
        acceptance bar is "does not freeze/drag," not "matches OTOS
        exactly").
  - [ ] Confirm no regression in the OTOS-coexistence bus-safety property
        ticket 002 already established (motion running throughout the
        session, no bus hang) — this ticket changes fusion behavior, not
        bus scheduling, but re-verify nothing about the fusion path
        introduces new I2C traffic patterns.

## Implementation Plan

**Approach**: this is a small, surgical change to `MainLoop::tick()`'s
observation-assembly step — the pseudocode already exists in
architecture-update.md's D1 section; implement it verbatim.

**Files to modify**:
- `source/runtime/main_loop.cpp` — the `fusable`/`otosSample`/`otosArg`
  assembly, `poseEstimator_.tick(...)`'s `otosObs` argument, `commit()`'s
  new `bb.otosValid`.

**Testing plan**:
- Full sim suite (exercises ticket 006's gate harness against a live,
  fused `PoseEstimator::tick()` call for the first time).
- Bench session per acceptance criteria — this is a MANDATORY,
  hazard-closing gate; do not skip.

**Documentation updates**: none required (behavior activation, no schema
change).
