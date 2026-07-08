---
id: '001'
title: D/T/TURN/RT bounded stop-decel seed correction
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: d-t-terminal-reverse-persists-decel-reseed-from-plan-velocity.md
completes_issue: false
exception:
  thrown_by: programmer
  thrown_at: '2026-07-08T11:20:57.110166+00:00'
  attempted: 'Implemented architecture-update.md Decisions 1-4 exactly as specified.
    Added Motion::JerkTrajectory::solveToVelocityWithSeedCorrection(targetVelocity,
    maxVelocity, measuredVelocity, cap) -- a new, narrow entry point that nudges lastVelocity_
    by clamp(measuredVelocity-lastVelocity_,-cap,+cap) (a symmetric clamp, Decision
    2) then delegates to the existing solveToVelocity(), mirroring retarget()/reanchor()''s
    precedent (Decision 1). Wired it into armDistanceStopDecel()/armVelocityStopDecel()/armRotationalStopDecel()
    via two new Planner helpers -- measuredLinearVelocity() (sum-average of leftObs/rightObs.velocity,
    mirroring maybeReplanDistance()''s own reanchor()-velocity averaging) and measuredRotationalVelocity()
    (differential-average converted via rotationalArcScale_, mirroring maybeReplanRotational()''s
    own position-domain conversion) -- both falling back to the plain, uncorrected
    solveToVelocity() when velocity.has is false, so no existing sim scenario (none
    populate velocity.has) is touched. Added kStopDecelSeedCorrectionCap=100mm/s /
    kRotStopDecelSeedCorrectionCap=0.6rad/s as ticket-owned constants sized from the
    089-007 bench numbers (Decision 4). Wrote and ran 9 new sim tests across two tiers:
    jerk_trajectory_harness.cpp unit-level (bounded/capped/rotational-channel mechanism
    proofs) and planner_harness.cpp Planner-tier (D/T/RT synthetic-observation scenarios
    proving no-reverse + strictly monotonic post-correction convergence, explicitly
    isolating the new mechanism from the pre-existing Decision-10 divergence-replan
    for the D scenario) -- all pass cleanly, satisfying the ticket''s three blocking
    sim bullets in isolation. But the ticket''s own overarching gate, `uv run python
    -m pytest tests/sim`, then surfaced 3 regressions in PRE-EXISTING, already-tuned
    tests: test_motion_overshoot_regression.py''s D-200-200-500 tolerance test (087-009''s
    own tuned baseline 502.27mm/+0.45% -> 516.29mm/+3.26%, over its 1.5% bar) and
    two test_motion_commands_arc_turn.py TURN absolute-heading tests (~98deg vs 90deg
    commanded, over a 6deg bar). Root-caused both via temporary instrumentation (reverted
    before leaving the tree): (1) armDistanceStopDecel()/armRotationalStopDecel()
    fire, by 087-009/Decision-10''s own design, very close to the plan''s own natural
    convergence, so linear_/rotational_''s remembered lastVelocity_ is already low
    at that exact tick (measured e.g. believed=60mm/s for D, believed=1.22rad/s --
    still full cruise -- for TURN); the sim''s own realistic (non-ideal) PID/motor
    lag reads a meaningfully higher instantaneous velocity at that SAME tick as ordinary
    end-of-decel tracking noise (e.g. measured=164mm/s for D), not genuine sustained
    cruise-speed divergence -- correcting toward it adds real, unwanted extra travel/heading
    on top of a decel Decision 10 + 087-009 had already correctly converged. A cap-sensitivity
    sweep (20/40/60/80/100mm/s) showed the D test passes at cap<=40mm/s and fails
    at cap>=60mm/s, while the confirmed hardware divergence needing correction (089-007)
    is 50-110mm/s -- no single cap value is both large enough to meaningfully address
    the hardware bug and small enough to avoid this sim regression. (2) Independently
    for TURN, rotationalArcScale_ defaults to 1.0f (089-005''s own documented placeholder
    -- TURN''s STOP_HEADING threshold and rotational target are literally the same
    number, no physical trackwidth/2 meaning), so measuredRotationalVelocity()''s
    division by it produced a wildly inflated "rad/s" value (measured=111rad/s against
    believed=1.22rad/s) that saturates the cap unconditionally whenever the wheels
    are moving -- an always-on bias, not a measurement-informed correction. RT (a
    genuine trackwidth/2 conversion, confirmed measured=2.14rad/s against believed=0.36rad/s
    in the same instrumented run) does not exhibit this, and its own full-sim tests
    stayed green.'
  conflict: 'architecture-update.md (092) Decisions 1-3 (a bounded, one-shot correction
    fired at every stop-arm event, no additional gate specified) and this ticket''s
    own blocking acceptance bullet "Full uv run python -m pytest tests/sim is green"
    are mutually unsatisfiable as currently specified: a cap sized to meaningfully
    address the confirmed hardware divergence (50-110mm/s, the issue''s own 089-007
    bench numbers) necessarily also fires on the sim''s own ordinary end-of-decel
    PID/plant noise at the identical stop-arm instant, because the correction as specified
    has no way to structurally distinguish "plan still cruising, the real bug" from
    "plan already converged via Decision 10''s replan + 087-009''s dead-time projection,
    ordinary plant lag" -- both present as "measured > believed at the arm tick."
    Additionally, Decision 1''s implicit reuse of rotationalArcScale_ (mirroring maybeReplanRotational()''s
    position-domain conversion) does not hold in the velocity domain for TURN specifically:
    089-005''s own arcScale=1.0f placeholder (its Grounding: "no conversion needed"
    because TURN''s STOP_HEADING threshold and rotational target are literally the
    same number in the POSITION domain) is not a physical trackwidth/2 conversion,
    so reusing it for a velocity conversion produces an unconditional, always-cap-saturated
    bias rather than a genuine measurement-informed correction. Neither issue is resolvable
    within Decisions 1-4''s own specified scope (a magnitude clamp + a ticket-owned
    cap + one-shot timing) without inventing a new, unauthorized guard (e.g. "only
    correct while the plan is still substantially below its own cruise/decel-start
    point" or "skip the rotational correction whenever rotationalArcScale_ is not
    a validated physical conversion") that would itself need its own sim-validated
    design -- exactly the kind of unproven refinement the ticket''s own Open Question
    1 / exception protocol anticipated needing a stakeholder call on, between this
    approach, issue option (b) (retune the velocity PID), or option (c) (an accepted
    terminal-tolerance bar).'
  surface: user-visible
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# D/T/TURN/RT bounded stop-decel seed correction

## Description

Sprint 089's Ruckig migration reduced but did not eliminate the D/T
terminal reverse-motion bug: `armDistanceStopDecel()`/`armVelocityStopDecel()`/
`armRotationalStopDecel()` (`source/subsystems/planner.cpp:730-770`) seed
their stop-triggered decel-to-rest re-solve from
`Motion::JerkTrajectory`'s own remembered `lastVelocity_` (089 Decision 8's
"never seed from measured state" contract) -- which
`JerkTrajectory::sample()` (`jerk_trajectory.cpp:170-179`) always overwrites
with the PLAN's theoretical velocity, never the measured one. On hardware
the bench-tuned velocity PID tracks loosely enough (measured ~250-310 mm/s
on a commanded 200) that this seed under-estimates the real wheel speed at
the exact stop-arm instant, so the decel-to-rest trajectory commands a
lower velocity than the wheel is actually running, and the PID brakes the
difference into 11-23 mm of reverse creep after `EVT done`
(`clasi/issues/d-t-terminal-reverse-persists-decel-reseed-from-plan-velocity.md`,
089-007 bench measurement).

This ticket implements the architecture-update.md Decision 1-4 design: a
new, narrow entry point on `Motion::JerkTrajectory` that nudges the seed
velocity toward a caller-supplied measured velocity, bounded to a
ticket-owned magnitude cap, invoked ONLY at the three stop-decel-arm call
sites -- never at the routine per-tick `sample()`, never at the goal-start
solve, and never inside the existing divergence-triggered replan
(`maybeReplanDistance()`/`maybeReplanRotational()`, 089 Decision 10, which
structurally cannot reach this failure mode -- see architecture-update.md
Grounding). This is a genuine, one-shot, bounded exception to 089 Decision
8's general seeding contract, not a reopening of it.

**Read `architecture-update.md` Decisions 1-4 and the Grounding section in
full before starting** -- they establish exactly why this must be a new
`JerkTrajectory` entry point (not caller-side math), why the clamp is
symmetric (not one-directional), and why firing exactly once per stop-arm
event (not per tick) is what keeps this from reopening the 087-009
limit-cycle bug class.

## Acceptance Criteria

- [ ] `Motion::JerkTrajectory` gains one new entry point implementing the
      bounded seed correction (exact name/signature is this ticket's own
      implementation decision) -- `solveToRest()`/`solveToVelocity()`/
      `retarget()`/`reanchor()`'s existing signatures and seeding behavior
      are unchanged.
- [ ] `armDistanceStopDecel()`, `armVelocityStopDecel()`, and
      `armRotationalStopDecel()` all use the new entry point (linear
      channel for the first two, rotational channel for all three where
      applicable), passing an averaged measured velocity derived from
      `leftObs`/`rightObs` (mirroring `maybeReplanDistance()`'s existing
      per-wheel-averaging pattern).
- [ ] The correction is a SYMMETRIC magnitude clamp
      (`clamp(measured - believed, -cap, +cap)`), not a one-directional
      (only-correct-upward) rule (Decision 2).
- [ ] **Sim, BLOCKING**: a Planner-level test injects a synthetic
      post-arm observation showing measured velocity persistently faster
      than the channel's own plan-believed velocity at the exact tick a
      stop-triggered decel is armed (mirroring 089-006's synthetic-
      observation pattern in `tests/sim/unit/planner_harness.cpp`) and
      asserts the resulting decel trajectory's sampled velocity (a) never
      reverses sign, and (b) converges MONOTONICALLY to rest -- no
      dip-then-rebound (the 087-009 limit-cycle signature).
- [ ] **Sim, BLOCKING**: a second scenario proves the correction is bounded
      -- an extreme synthetic divergence (far beyond anything physically
      plausible) still produces a seed correction capped at the
      ticket-owned constant, not a value that fully snaps to measurement.
- [ ] **Sim, BLOCKING**: at least one scenario covers the rotational
      channel (`TURN` or `RT`) with the same injected-divergence pattern,
      not just the linear channel.
- [ ] Full `uv run python -m pytest tests/sim` is green.
- [ ] **Bench, BEST-EFFORT**: `D 200 200 1000` and `T 200 200 1000` on the
      stand (direct USB serial -- the relay dongle is unplugged this
      sprint) -- measured reverse encoder motion after `EVT done` is
      re-measured against the 089-007 baseline (11-21 mm / 19-23 mm) and
      recorded, whether or not it is fully eliminated. If the bench step
      cannot be completed (robot wedges/latches, hardware unavailable, or
      a regression surfaces that cannot be resolved in-sprint), record the
      sim result as the completed deliverable and file a fresh
      `clasi/issues/` follow-on instead of blocking on it.
- [ ] If the monotonic-convergence sim scenario CANNOT be made to pass
      cleanly (i.e. the bounded correction cannot be proven safe against
      the 087-009 signature), do not ship a blind control change --
      surface this via the sprint's exception protocol for a stakeholder
      decision between this approach, retuning the velocity PID (issue's
      option (b)), or an accepted terminal-tolerance bar (option (c)).

## Implementation Plan

**Approach**:
1. Read `architecture-update.md` Grounding + Decisions 1-4 in full.
2. Add the new bounded-correction entry point to
   `source/motion/jerk_trajectory.{h,cpp}`, following the `retarget()`/
   `reanchor()` doc-comment style (state the seeding contract exception
   explicitly, cross-reference this ticket/architecture-update.md).
3. Thread `leftObs`/`rightObs` into `armDistanceStopDecel()`/
   `armVelocityStopDecel()`/`armRotationalStopDecel()` (already in scope at
   every call site inside `Planner::tick()`) and compute the averaged
   measured velocity for the affected channel(s).
4. Pick an initial cap value per channel (linear: informed by the 089-007
   bench numbers, e.g. in the 50-110 mm/s range; rotational: a
   correspondingly reasoned rad/s value), named as a ticket-owned constant
   (e.g. `kStopDecelSeedCorrectionCap`/`kRotStopDecelSeedCorrectionCap`),
   justified in a comment, not silently invented.
5. Write the sim tests (synthetic-observation pattern, mirroring 089-006 in
   `tests/sim/unit/planner_harness.cpp`/`test_planner.py`) BEFORE declaring
   done -- these are the blocking acceptance gate.
6. Attempt the bench step; record the outcome honestly regardless of
   result.

**Files to modify**: `source/motion/jerk_trajectory.h`,
`source/motion/jerk_trajectory.cpp`, `source/subsystems/planner.h`,
`source/subsystems/planner.cpp`, `tests/sim/unit/jerk_trajectory_harness.cpp`
(if the new entry point needs its own unit-level coverage),
`tests/sim/unit/planner_harness.cpp`, `tests/sim/unit/test_planner.py`.

**Testing plan**:
- **Existing tests to run**: full `uv run python -m pytest tests/sim`
  (regression guard for 089's own no-reverse trace assertions and every
  existing D/T/TURN/RT scenario).
- **New tests to write**: see Acceptance Criteria's sim bullets above.
- **Verification command**: `uv run python -m pytest tests/sim`.

**Documentation updates**: none expected beyond the new entry point's own
doc comment and this ticket's completion notes recording the chosen cap
value(s) and their justification.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim` (full
  suite).
- **New tests to write**: synthetic-observation divergence-at-stop-arm
  scenarios for `D` (linear) and at least one of `TURN`/`RT` (rotational),
  plus a bounded-correction-cap scenario.
- **Verification command**: `uv run python -m pytest tests/sim`.

## Team-lead disposition (2026-07-08) — DESCOPED, no fix shipped

The programmer hit this ticket's own stakeholder-decision trigger: the
bounded stop-decel seed correction (option (a)) is **proven infeasible in
sim** — no cap value both fixes the real 50–110 mm/s hardware divergence and
avoids regressing 087-009's overshoot bar (passes only at cap ≤ 40 mm/s), and
TURN separately saturates because `rotationalArcScale_` is a 1.0 placeholder,
not a trackwidth/2 conversion. Per the trigger, **no control change was
shipped** — the exception WIP was reverted; master's motion behavior is
unchanged. `completes_issue` set to **false**: the D/T reverse-creep bug is
NOT fixed.

This ticket's investigative deliverable (determine feasibility of option (a),
escalate if infeasible) is complete; it is marked done as **descoped**, not as
a fix. The decision between (a-variant) / PID retune / accepted-tolerance is
escalated to the stakeholder via the fresh pool issue
`clasi/issues/d-t-turn-terminal-reverse-stakeholder-decision.md`. The approach
and its 9 passing monotonic-convergence sim tests are preserved on branch
`spike/092-001-infeasible-bounded-seed-correction` (commit 3559d28e). The
`rotationalArcScale_` placeholder is flagged in that issue as a real latent
defect worth fixing regardless of the linear-channel choice.
