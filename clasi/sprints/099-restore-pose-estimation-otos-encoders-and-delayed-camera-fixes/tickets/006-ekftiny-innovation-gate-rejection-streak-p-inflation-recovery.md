---
id: '006'
title: EkfTiny innovation gate + rejection-streak P-inflation recovery
status: open
use-cases: [SUC-003]
depends-on: ['002', '004']
github-issue: ''
issue: restore-pose-estimation-otos-encoders-delayed-camera-fixes.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# EkfTiny innovation gate + rejection-streak P-inflation recovery

## Description

`EkfTiny::updatePosition()`/`updateHeading()` currently apply every
observation unconditionally (no gating beyond a numerical-singularity
guard). Once OTOS fusion is enabled (ticket 007), a momentarily-
disagreeing OTOS reading (e.g. the frozen-fused-pose hazard: OTOS static
while the robot is moving, or vice versa — `clasi/issues/
poseestimator-fused-pose-frozen-on-hardware.md`) would otherwise drag or
freeze `fusedPose` incorrectly.

This ticket adds a **bounded innovation-consistency gate**
(architecture-update.md D4): a Mahalanobis-style chi-square test on the
position channel, a sigma-bound test on the heading channel, each with
its own rejection-streak counter; a run of consecutive rejections
inflates that channel's own `P` diagonal (never a hard reset) so a
genuinely-shifted OTOS is eventually re-trusted rather than permanently
locked out. This ticket is **sim-only** — it lands the gate mechanism,
inert (fusion itself is still disabled, `otosObs = nullptr`, until ticket
007), so it can be reviewed and merged without needing bench time.

**Do not treat the starting constants below as final** — architecture-
update.md's Decision 4 explicitly documents them as bench-evidence-
dependent starting values. Characterize accept/reject/streak-recovery
behavior in the harness against these documented starting values; if
ticket 002's bench session (already landed by the time this ticket
starts, per its dependency) produced concrete `otosconn=`/frozen-pose
evidence, factor it into the final constants and record what informed the
choice in this ticket's completion notes.

## Acceptance Criteria

- [ ] `EkfTiny::updatePosition(xOtos, yOtos)` computes the Mahalanobis
      statistic `d^2 = y^T S^-1 y` (S already computed for the existing
      analytic 2x2 inverse) and rejects (returns without modifying state)
      when `d^2` exceeds a documented starting threshold (2-DOF chi-square
      critical value, e.g. ~9.21 — characterize, do not freehand).
- [ ] `EkfTiny::updateHeading(thetaOtos)` rejects when `|y| > kSigma *
      sqrt(S)` for a documented starting `kSigma` (e.g. 3.0).
- [ ] Each channel gets its own private rejection-streak counter,
      incremented on reject, reset to 0 on accept.
- [ ] When a channel's streak reaches a documented starting threshold
      (e.g. 10 consecutive rejections), that channel's own `P` diagonal
      entries are inflated by a documented starting bump (NOT a full
      reset — a gradual, bounded widening) so a genuinely-shifted sensor
      is eventually re-accepted; the streak counter resets once the gate
      re-opens (a subsequent accept).
- [ ] Public method signatures (`updatePosition(float, float)`,
      `updateHeading(float)`) are UNCHANGED — the gate is entirely
      internal, invisible to `PoseEstimator`.
- [ ] `PoseEstimator`'s delayed-fix update path (ticket 008, not landed
      yet this ticket) is unaffected by this gate by construction — it
      does not call `updatePosition()`/`updateHeading()` at all (it uses
      its own, separate, ungated EKF update path per D5 — verify this
      ticket does not accidentally route the fix path through the gated
      methods when ticket 008 lands).
- [ ] Extended `ekf_tiny_harness.cpp`: accept/reject boundary cases for
      both channels against known innovation values; a synthetic
      rejection streak demonstrates `P`-inflation recovery (a fixed,
      genuinely-shifted observation is rejected initially, then
      eventually accepted after enough streak-triggered inflation, within
      a documented tick bound); a single noisy-but-not-shifted observation
      does not trip the streak counter to inflation.
- [ ] Full sim suite passes; this ticket requires no bench session (gate
      exists but fusion stays disabled until ticket 007).

## Implementation Plan

**Approach**: add the gate/streak/inflation logic as private state and
inline checks inside `EkfTiny::updatePosition()`/`updateHeading()`
(`source/estimation/ekf_tiny.{h,cpp}`) — no public API change. Pick the
documented starting constants from architecture-update.md's Decision 4,
implement them as named `static constexpr` values (mirroring `EkfTiny`'s
existing `kPriorXY`/`kPriorTheta` constant style), and characterize them
in the harness rather than tuning freehand.

**Files to modify**:
- `source/estimation/ekf_tiny.h` — new private gate/streak state, new
  named constants, updated doc comments on `updatePosition()`/
  `updateHeading()`.
- `source/estimation/ekf_tiny.cpp` — the gate/streak/inflation logic.

**Testing plan**:
- Extend `tests/sim/unit/ekf_tiny_harness.cpp` per the acceptance
  criteria above — this IS the characterization work; do not defer
  threshold justification to a later, undocumented tuning pass.
- Full sim suite.

**Documentation updates**: architecture-update.md's Decision 4 already
documents the starting values and the "characterize, don't freehand"
requirement — if the final constants differ from the documented starting
values, note the change and its bench/sim justification in this ticket's
completion notes (a future architecture-consolidation pass reconciles the
doc, not this ticket).
