---
status: pending
---

# Wheel-speed servo: measure the trim's load envelope, then decide whether it moves into Drive

## Description

Stakeholder design discussion (2026-07-31): should wheel speed be maintained
by a PID at the Drive level — for heavier loads, and to escape the duty dead
zone "naturally"? Conclusion of the discussion: the system already HAS the
PID — `Motion::WheelTrim`, a per-wheel PI + acceleration-feedforward
controller on wheel-speed error in the velocity domain — but with three
deliberate restrictions (velocity-domain output through Drive's calibrated
map, integrator active only in cruise `Hold`, total authority clamped at
`trimMax` = 80 mm/s / `trimIMax` = 40 mm/s). The open questions are the
restrictions' envelope under load, and where the servo should live.

This issue: (1) measure whether the current envelope actually saturates
under realistic load, (2) if so, widen authority / revisit the integrator
gate, (3) evaluate relocating the servo into `App::Drive` so EVERY
`cmdVelocity` writer gets closed-loop speed — not just the planner.

## Cause

- **Load gap is plausible.** Drive's feedforward map (per-wheel,
  per-direction affine) was measured unloaded on the stand; a loaded
  gearmotor droops, so the map under-delivers. At cruise the trim's
  integrator absorbs the droop only up to its clamps; during accel ramps
  the integrator is frozen (kp + kaff only), so a loaded robot ramps slow
  with nothing accumulating. Distance stays exact (the profiler re-plans
  from measured positions) — the failure mode is sustained speed deficit,
  sluggish legs, and eventually move timeouts.
- **The teleop asymmetry.** The trim lives in `Motion::Planner` and is
  summed only in its `update()`. A `WHEELS` teleop command writes
  `cmdVelocity` through `App::Drive`'s own path — pure open loop, no speed
  maintenance at all. If the actuation contract is "command a wheel
  speed," every writer should get a wheel that holds it.
- **History constrains the solution space.** Duty-domain feedback on this
  plant limit-cycled at 2–3 Hz across a full on-robot tour (2026-07-27,
  plotted), and integrator-driven dead-zone escape produced the
  unacceptable stick-slip sawtooth the same day. ~120–140 ms actuation lag
  (2–3 control periods of dead time) + noisy 20 Hz difference-quotient
  velocity is why. The dead zone is handled deterministically on the
  feedforward side (calibrated intercepts + sprint-114 boost-to-breakaway)
  and must stay there. Whatever this issue changes keeps: velocity domain,
  authority clamp, feedforward-first.

## Proposed fix

### Phase 1 — measure the envelope (bench, no code changes)

Load experiment on the stand: apply controlled drag to the wheels (or a
loaded playfield run) while logging `trimLeft()`/`trimRight()` and
`trimIntegralLeft()/Right()` — the observability accessors already exist.
Sweep cruise speeds; record where (if anywhere) trim saturates against
`trimMax`/`trimIMax` and how much speed deficit remains during ramps.

Exit gate: a plot of trim vs load, and a verdict — envelope sufficient, or
saturating. **If sufficient, stop here and close the issue with the data.**

### Phase 2 — widen the envelope (config-level, only if Phase 1 saturates)

- Raise `trimMax`/`trimIMax` against the measured deficit.
- Revisit the integrator gate: allow integration during ramps when the
  target accel is small (gate on |cmdAccel| rather than phase), with
  anti-windup. Keeps the measured windup-hump protection (integrating
  transient ramp error released a +20%/0.5 s overshoot at cruise entry on
  the duty-domain predecessor).
- Re-run Phase 1's experiment as the A/B.

### Phase 3 — evaluate relocating the servo into Drive (design decision)

Move the velocity-domain servo from `Motion::Planner::stageTrim()` into
`App::Drive`, so planner moves, WHEELS teleop, and any future
`cmdVelocity` writer all get closed-loop speed:

- Blackboard carries `cmdAccel` alongside `cmdVelocity` (kaff needs the
  target accel; the integrator gate derives "ramping vs holding" from
  cmdAccel ≈ 0 instead of the planner's `MovePhase`).
- Planner sheds `stageTrim()` and its trim state — compounds with the
  tick() state-machine redesign
  ([`planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md`](planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md)).
  Note: that issue groups the five trim gains into PlannerLimits'
  `tracking` sub-struct; if the servo moves to Drive the gains move to
  Drive's config surface instead — coordinate the two so the gains are
  reshaped once, not twice.
- Tension to resolve explicitly: Drive sits in the frozen-candidate base
  layer (`src/firm`), and control-law churn belongs in the motion library
  per the two-layer split. The argument FOR: a fixed-structure per-wheel
  speed servo with configured gains is actuator-local plant knowledge,
  the same category as the calibration map Drive already owns. The
  stakeholder decides with the Phase 1/2 data in hand.
- Acceptance: bench A/B (same tour, servo in planner vs servo in Drive) —
  closure and per-leg speed tracking at least as good, and a WHEELS
  teleop command demonstrably holds speed under drag.

## Verification

- Phase 1 produces the trim-vs-load dataset and chart (send the chart).
- Phase 2 (if run): the same experiment shows deficit closed without
  reintroducing the cruise-entry overshoot hump.
- Phase 3 (if adopted): the A/B tour + loaded-WHEELS demonstration above;
  `SET` wire keys for the gains route to wherever the servo now lives, no
  silent no-op.

## Related

- [`planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md`](planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md)
  — deletes the duty stage this issue's history references; coordinate the
  trim-gain reshape (Phase 3 note above).
- [`tests-outside-the-system-test-taxonomy-and-tiers.md`](tests-outside-the-system-test-taxonomy-and-tiers.md)
  — the load experiment is a characterization tool by that issue's
  taxonomy (it produces calibration data, not a standing verdict); its
  script belongs in `src/tools/`.
- Current duty-sweep work (OOP, branch 20260731-testgui) is establishing
  the default duty-per-speed calibration — a better feedforward map
  shrinks the residual the trim must close; Phase 1 should run AFTER that
  calibration lands, or its saturation verdict measures the old map.
