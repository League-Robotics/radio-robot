---
id: '006'
title: EkfTiny innovation gate + rejection-streak P-inflation recovery
status: done
use-cases:
- SUC-003
depends-on:
- '002'
- '004'
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

- [x] `EkfTiny::updatePosition(xOtos, yOtos)` computes the Mahalanobis
      statistic `d^2 = y^T S^-1 y` (S already computed for the existing
      analytic 2x2 inverse) and rejects (returns without modifying state)
      when `d^2` exceeds a documented starting threshold (2-DOF chi-square
      critical value, e.g. ~9.21 — characterize, do not freehand).
- [x] `EkfTiny::updateHeading(thetaOtos)` rejects when `|y| > kSigma *
      sqrt(S)` for a documented starting `kSigma` (e.g. 3.0).
- [x] Each channel gets its own private rejection-streak counter,
      incremented on reject, reset to 0 on accept.
- [x] When a channel's streak reaches a documented starting threshold
      (e.g. 10 consecutive rejections), that channel's own `P` diagonal
      entries are inflated by a documented starting bump (NOT a full
      reset — a gradual, bounded widening) so a genuinely-shifted sensor
      is eventually re-accepted; the streak counter resets once the gate
      re-opens (a subsequent accept).
- [x] Public method signatures (`updatePosition(float, float)`,
      `updateHeading(float)`) are UNCHANGED — the gate is entirely
      internal, invisible to `PoseEstimator`.
- [x] `PoseEstimator`'s delayed-fix update path (ticket 008, not landed
      yet this ticket) is unaffected by this gate by construction — it
      does not call `updatePosition()`/`updateHeading()` at all (it uses
      its own, separate, ungated EKF update path per D5 — verify this
      ticket does not accidentally route the fix path through the gated
      methods when ticket 008 lands).
- [x] Extended `ekf_tiny_harness.cpp`: accept/reject boundary cases for
      both channels against known innovation values; a synthetic
      rejection streak demonstrates `P`-inflation recovery (a fixed,
      genuinely-shifted observation is rejected initially, then
      eventually accepted after enough streak-triggered inflation, within
      a documented tick bound); a single noisy-but-not-shifted observation
      does not trip the streak counter to inflation.
- [x] Full sim suite passes; this ticket requires no bench session (gate
      exists but fusion stays disabled until ticket 007).

## Completion Notes

**Constants used (all exactly D4's documented starting values — no
deviation):**
- `kChiSquare2Dof99 = 9.21f` (2-DOF chi-square, p=0.99) — position channel
  Mahalanobis gate.
- `kHeadingSigma = 3.0f` — heading channel `|y| > kSigma*sqrt(S)` gate.
- `kRejectStreakThreshold = 10` — consecutive rejections before a widening.
- `kPInflationBumpXY = 50.0f` mm², `kPInflationCapXY = 500.0f` mm² —
  position-channel P-inflation bump/cap (new; D4 did not specify an exact
  bump/cap number, only "documented starting bump" and "bounded" — sized in
  the harness so a ~30mm genuinely-shifted observation against a settled P
  needs exactly two widenings, not one, to recover: a gradual, not abrupt,
  demonstration).
- `kPInflationBumpTheta = 0.001f` rad², `kPInflationCapTheta = 0.01f` rad² —
  heading-channel P-inflation bump/cap, same sizing rationale (a 0.15rad
  genuinely-shifted heading observation needs two widenings to recover).

**Bench evidence check (per the ticket's "factor in ticket 002's evidence"
instruction):** ticket 002 (done, `tickets/done/002-...md`) closed via an
architecture exception — its own bench-mandatory acceptance criteria are
explicitly UNCHECKED/DEFERRED ("Not run this session"). It records no
numeric `otosconn=`/frozen-pose evidence, only the qualitative fact that
OTOS currently reads `connected()==False` on the bench. There is nothing
numeric there to inform this ticket's thresholds, so the constants above
are D4's starting values as-is, characterized in the harness per the
"characterize, don't freehand" mandate rather than tuned against bench
data that does not yet exist. Re-tuning against real bench evidence is
future work once ticket 002's deferred bench session (or ticket 007's) is
actually run.

**Gate/streak/inflation logic summary:** both `updatePosition()` and
`updateHeading()` compute their gate statistic using the SAME S/S⁻¹ already
computed for the Kalman update (S⁻¹ is reused, never recomputed, for
`updatePosition()`'s Mahalanobis `d²`). On reject: the call returns before
touching `ekf_.x`/`ekf_.P`, and that channel's private `rejPosStreak_`/
`rejHeadStreak_` counter increments. Every time the streak crosses a
`kRejectStreakThreshold` multiple (10, 20, 30, ...) that channel's own P
diagonal (`P[0][0]`/`P[1][1]` for position, `P[2][2]` for heading) is
bumped and clamped to its cap — the streak counter is NOT reset by an
inflation event, only by a subsequent accept, per the AC. On accept, the
streak resets to 0 and the normal Kalman update proceeds unchanged from
before this ticket.

**Fix-path (ticket 008) separation — verified by reading the code, not
assumed:** grepped all of `source/` for `updatePosition(`/`updateHeading(`
— today there is exactly ONE call site, `PoseEstimator::tick()`
(`source/subsystems/pose_estimator.cpp:160-163`), gated on
`otosObs != nullptr && otosObs->stamp.valid`. In production `otosObs` is
always `nullptr` (`source/runtime/main_loop.cpp`, per D1/ticket 004 —
OTOS fusion is ticket 007's job), so this gate is currently fully inert on
hardware, exactly as this ticket's Description says. Ticket 008 (open, not
yet landed) is the one that will add the delayed camera-fix; its own plan
(D5) requires an UNGATED update and explicitly must NOT reuse
`updatePosition()`/`updateHeading()` as-is (that would be "gating with
extra steps") — it needs new, separate methods (e.g.
`updatePositionUngated()`/`updateHeadingUngated()`) sharing a private
Kalman-update core, left to ticket 008 to implement. This ticket adds the
gate ONLY inside the two existing gated methods, so by construction it
cannot sit in a future fix-path call site that does not exist yet.

**New harness scenarios (`tests/sim/unit/ekf_tiny_harness.cpp`)**, all
against a shared, hand-derived "settled" filter (`gate::makeSettledFilter()`
— 5 straight predict ticks give `P[0][0]=0.4` exactly, `P[2][2]=0.0001`
exactly, both hand-derivable from the arc-motion Jacobian, not read back
empirically):
- `scenarioPositionGateAcceptRejectBoundary` — 12mm (d²=5.67) accepted,
  20mm (d²=15.75) rejected, against the exact `s00=25.4` boundary
  (chi-square boundary ≈15.29mm).
- `scenarioHeadingGateAcceptRejectBoundary` — 0.05rad accepted, 0.15rad
  rejected, against the exact `s=0.00086` boundary (3-sigma boundary
  ≈0.088rad).
- `scenarioPositionRejectionStreakRecoversViaPInflation` /
  `scenarioHeadingRejectionStreakRecoversViaPInflation` — a fixed
  genuinely-shifted observation (30mm / 0.15rad) is rejected on calls 1-20,
  then ACCEPTED on call 21 (measured, both channels) after two
  streak-triggered widenings. Documented tick bound asserted: recovery must
  land within 25 calls (comfortable margin over the measured 21) and must
  take strictly more than one streak cycle (>10 calls), proving the gate
  neither caves immediately nor never recovers.
- `scenarioSingleAcceptedNoisyObservationDoesNotPreloadStreak` — seeds a
  partial streak (5 rejects, below threshold), feeds ONE small in-gate
  (accepted) observation, then 9 more reject-level observations; asserts
  `P[0][0]` is bit-for-bit unchanged across those 9 — proving the single
  accept correctly reset the streak counter to 0 (a buggy
  non-resetting implementation would have crossed the streak=10 threshold
  partway through and inflated P, which this test would have caught).
- `scenarioPInflationIsBoundedByCap` — a permanently-disagreeing
  (10000mm) observation over 130 calls (13 streak cycles) never pushes
  `P[0][0]`/`P[1][1]` past `kPInflationCapXY`, and is never re-accepted —
  proves the widening is bounded, not unbounded.

**Existing scenarios updated (now correctly gated, not broken):** the
pre-existing `ekf_tiny_harness.cpp` pull-toward-observation scenarios
((b1)/(b2), unconditional-accept era) used 60mm/-45mm and 0.2rad offsets
that are now correctly REJECTED by the new gate — shrunk to 10mm/-8mm and
0.05rad (both comfortably inside the gate, still nonzero, still prove the
correction executes). `pose_estimator_harness.cpp`'s scenarios (b)
(`scenarioOtosDivergesFusedFromEncoder`) and (c)
(`scenarioZeroConfigSentinelKeepsFusionFiniteAndCorrected`) used a
persistent 150mm OTOS-vs-encoder offset that the new gate now correctly
recognizes as a genuinely-shifted-sensor-class disagreement and rejects
every one of their 8 ticks (nowhere near the ~21-tick recovery point) —
shrunk to 60mm, which stays inside the gate at every tick (d² ≤ ~2.7 vs.
9.21) and still produces the required >10mm measurable divergence once the
scenario's one-tick observation lag settles out. Both changes are
necessary consequences of adding the gate the ticket's own AC requires,
not scope creep — verified by hand-deriving the exact P/S values at each
tick (documented inline in the updated comments) before picking the new
offsets, then confirming both harnesses pass.

**Verification:**
- `just build` — firmware hex + host sim lib both compile clean (RAM
  98.33%, expected/normal).
- `ekf_tiny_harness` / `pose_estimator_harness` compiled standalone
  (`c++ -std=c++20 -Wall -Wextra`) and run directly: all scenarios PASS,
  zero warnings beyond pre-existing vendored `tinyekf.h` unused-function
  warnings.
- `uv run python -m pytest` — full suite green, no failures introduced
  (see session for the exact pass/xfail/xpass counts).

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
