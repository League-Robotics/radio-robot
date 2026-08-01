---
status: pending
---

# Move the wheel-speed controller into Drive: conversion + slow trim + fast PID

## Description

**DECIDED (stakeholder, 2026-08-01): the wheel-speed controller moves into
`App::Drive`.** Not a maybe. One unified controller in the base — the
calibrated conversion map, the slow trim (bounded intercept adaptation),
and a small-authority fast PID — establishing the wheels-solid contract:
you command the wheel speeds, and that's what the wheel speeds are. The
motion planner plans on that expectation and sheds its own wheel control
entirely.

Origin (design discussion 2026-07-31 → 2026-08-01): should wheel speed be
maintained by a PID at the Drive level — for heavier loads, and to escape
the duty dead zone "naturally"? The system already HAS the PID —
`Motion::WheelTrim`, a per-wheel PI + acceleration-feedforward controller
on wheel-speed error in the velocity domain — but planner-side by
evolutionary accident, with an additive-offset structure that cannot
represent the measured affine residual, and invisible to WHEELS teleop.

This issue: (0) population duty sweep → baked defaults + measured
adaptation bounds, (1) baseline measurement of the current trim under
load, (2) the unified algorithm — the trim is bounded adaptation of the
duty↔wheel-speed map's intercept (ONE parameter; Phase 0's sweep
confirms), the PID is a small fast loop that never carries standing
error, (3) the move itself: the whole controller into `App::Drive`, so
EVERY `cmdVelocity` writer (WHEELS teleop included) gets solid wheel
speeds, not just the planner.

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

### Phase 0 — population duty sweep: baked defaults + measured adaptation bounds (stakeholder, 2026-08-01)

The stakeholder hooks up a batch of motors; the duty-sweep tool runs each
one, both directions:

- Fit the affine duty→speed line per motor per direction; capture each
  motor's breakaway duty (dead-zone edge) in the same run.
- **Population mean → the baked default map** (generated constants /
  robot-JSON defaults, replacing single-robot guesses).
- **Population spread (envelope or ±2σ) → the adaptation bounds**: "how
  far a real motor sits from the average" IS the space the adaptation may
  move in. Replaces the ±10%-by-feel number with a measured one.
- **The parallel-lines test decides WHICH parameter adapts** (stakeholder
  hypothesis: intercept, not slope — adapt ONE, not both): if the
  population plots as a family of roughly parallel lines, variation is
  intercept-dominated and intercept-only adaptation is confirmed; fanned
  lines would say slope. Physics favors intercept (constant-torque loads
  — weight, gear friction — droop speed by a constant; viscous drag and
  supply sag are the slope-shifters), and intercept-only is learnable
  from a SINGLE cruise speed, deleting the multi-speed observability
  caveat, with no loop-gain risk.
- Breakaway population spread feeds the deadband constants the same way.
- Artifacts: the per-motor sweep dataset committed (send the chart), plus
  the derived (mean map, bounds, breakaway band) saved as generated
  values the boot config bakes.

Builds on the single-robot duty-sweep calibration already in flight
(OOP, branch 20260731-testgui, commit ab303ee3); same tool, run across
the population.

### Phase 1 — measure the envelope AND the unwind (bench, no code changes)

Load experiment on the stand: apply controlled drag to the wheels (or a
loaded playfield run) while logging `trimLeft()`/`trimRight()` and
`trimIntegralLeft()/Right()` — the observability accessors already exist.
Sweep cruise speeds; record where (if anywhere) trim saturates against
`trimMax`/`trimIMax` and how much speed deficit remains during ramps.
**Also measure the recovery half**: release the load mid-run and measure
the overspeed transient and its decay time (predicted: stored integral
carried into the next leg, unwinding only during Hold at tau = 1/ki
≈ 2.5 s with current gains — the stakeholder's "takes longer to adjust
back" concern, 2026-08-01).

Exit gate: trim-vs-load and unwind-transient plots, and a verdict.

### Drive interface today (review, src/firm/app/drive.h)

The core is the two-method contract, mirrored from the planner:

- `tick(speedLeft, speedRight)` — commanded speed → duty (per-wheel
  calibration → crawl shaping → quiet-at-zero) → leaf writes. Takes the
  speeds as parameters so there is ONE actuation path regardless of which
  subsystem decided the motion.
- `update(Types::RobotState&, now)` — expire an armed WHEELS command,
  publish targets into `Wheel::cmdVelocity` ONLY while Drive owns motion
  (runs after `Planner::update()`).

Around it: the WHEELS lifecycle (`command(vLeft, vRight, duration,
moveId, now)` — always time-bounded — `estop()`, `owns()`,
`takeCompletion(&moveId)`); the fail-closed calibration installers
(`setDutyPerSpeed(l, r)`, `setWheelCorrection(gain/intercept × wheel ×
accel|decel — 8 values)`, `setCrawlPulse(p)`, `calibrated()` — an
uncalibrated Drive refuses to write duty); the baked
`kDutyPerSpeed = 0.001182` (ONE value for both wheels — measured 1.9%
apart; per-wheel pairs invited the circular duty_per_speed-vs-wheel_gain
calibration bug); and observability (`targetLeft()/targetRight()`,
`trackWidth()`). Header line 14 currently reads "There is no controller
here" — the line this redesign deletes.

Interface change the unified controller needs: `tick()` must also see
the measured wheel velocity (+ sampleTime, for fresh-sample gating) and
the commanded accel (for kaff and the steady gate). Proposal:
`tick(const Types::RobotState&)` — Drive already includes robot_state.h;
the planner publishes a new `Wheel::cmdAccel` blackboard field beside
`cmdVelocity`. The one-actuation-path property is preserved.

### Phase 2 — the algorithm: conversion + trim + PID (per wheel, per 50 ms tick)

Evidence for map-parameter adaptation over an additive offset (stakeholder
bench data, commit ab303ee3): the right wheel measured cmd 100→70,
150→115, 250→210, 400→344 — an affine residual, actual ≈ 0.91·cmd − 21.
A stored integral offset can represent one point on that line; a map
correction represents the line.

Signals: `v_cmd` (blackboard), `a_cmd` (new blackboard field), `v_meas`
(encoder velocity, fresh-sample gated), `err = v_cmd − v_meas`. Every
correction lives in the VELOCITY domain and converts to duty through the
map at one point.

**Stage A — conversion (the map; offline timescale):**

```
v_corrected = gain[dir] · v_cmd + intercept[dir] + bias    // [mm/s]
dutyFF      = dutyPerSpeed · v_corrected                   // [duty]
```

`gain`/`intercept` per wheel per direction from calibration (population
mean default, per-robot override). `bias` is the ONE adapted parameter —
the trim — see Stage C. Slope is never adapted online.

**Stage B — the PID (fast timescale, small authority):**

```
p    = kp · err
i   += ki · err · dt     while |a_cmd| < aSteady, else frozen; |i| ≤ iMax
ff   = kaff · a_cmd
pid  = clamp(p + i + ff, ±pidMax)                          // [mm/s]
duty = dutyFF + dutyPerSpeed · pid
```

Because Stages A+C carry all DC content, `i` idles near zero — the
condition under which a PID near stiction does not stick-slip. Reset on
estop().

**Stage C — the trim (slow timescale; adaptation of `bias`):**

```
when steady:  |a_cmd| < aSteady  AND  |v_cmd| ≥ vMin  AND fresh sample
    bias += err · dt / tauAdapt                            // [mm/s]
    clamp bias ∈ [−biasMax, +biasMax]     // biasMax = population spread
```

`tauAdapt` is tens of seconds — load changes are slow; the hard clamp,
not an unwind rate, is what bounds recovery after a loaded period.
Intercept-only adaptation is single-speed observable (no multi-speed
excitation needed) and cannot change the loop gain.

**Stage D — output conditioning (existing, unchanged):** crawl shaper,
quiet-at-zero baseline, MotorArmor write-suppression at the leaf. The
breakaway/dead zone stays calibration-owned feedforward (stick-slip
history) — the adaptive bias trims droop, it does not rediscover the
dead zone.

**Wheels-solid contract enforcement:**

- Speed floor: `0 < |v_cmd| < vMin` → policy decision (round up vs
  refuse — see Open decisions), never a silently stalled wheel.
- Deficit flag: `|err| > deficitThreshold` sustained for `deficitWindow`
  while `bias` AND `pid` are saturated → speed-deficit telemetry fault
  flag (fail-loud, like shaping-disabled).
- Accepted tradeoff: load beyond `biasMax + pidMax` means the robot runs
  slow, loudly; the distance ledger keeps moves accurate. Predictability
  over load rejection.

### Phase 3 — the move: ONE wheel controller in Drive (DECIDED, stakeholder 2026-08-01)

The WHOLE wheel-speed controller unifies in the base — decided, not
proposed. Splitting the feature (adaptation in Drive, PID in the motion
planner) is rejected. The receipt: the sprint-122 two-layer
split charter (CLAUDE.md) defines the firmware base as "buses, devices,
THE VELOCITY PID, the wire, the loop schedule" — the wheel controller was
always chartered base-side; the trim drifted planner-side by evolutionary
accident (it grew inside the planner's duty-stage experiments).

The unified unit is one controller with state at three timescales:

| Timescale | Mechanism | Captures |
|---|---|---|
| offline | calibrated map (Phase-0 population mean, per-robot bake) | what this kind of motor is |
| slow | adapted parameter (intercept, bounded by population spread) | what THIS motor is doing today, under this load |
| fast | small-authority per-cycle PID on wheel-speed error | the residual the other two can't predict |

Conversion + adaptation carry all the DC content, so the fast loop never
holds a standing error — its integrator stays near zero, which is exactly
the condition under which a PID near stiction does not stick-slip. The
2–3 Hz limit cycle happened when one loop was asked to be map,
adaptation, and feedback at once; separated by timescale each piece is
boring. "The trim is just a parameter of the PID" becomes literal: the
adapted intercept is the controller's slowly-managed bias term, sitting
beside its gains.

Remaining work:

- Planner sheds `stageTrim()`/`WheelTrim` and its trim state — compounds
  with the tick() state-machine redesign
  ([`planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md`](planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md)).
  Note: that issue groups the five trim gains into PlannerLimits'
  `tracking` sub-struct; under this redesign the trim gains leave
  PlannerLimits entirely (the adaptation is configured on Drive) —
  coordinate so the fields are reshaped once, not twice.
- The adaptation's inputs are (commanded, measured) wheel-speed pairs —
  Drive already sees the command; the measured wheel velocity reaches it
  via `Types::RobotState` (the blackboard is already the boundary).
- Decide the fate of the per-tick closed loop: a small kp for transient
  damping in Drive, or none (the profiler re-plans from measured
  positions every tick, so trajectory correction already exists at the
  planning level).
- The two-layer-split tension is resolved by the reframe: a bounded
  correction to the actuator's own calibration map is actuator-local
  plant knowledge, the same category as the map itself — base-layer
  Drive is its natural home.
- Acceptance: bench A/B (same tour, old additive trim vs map
  adaptation) — closure and per-leg speed tracking at least as good;
  a WHEELS teleop command demonstrably holds speed under drag; and the
  right-wheel affine residual from ab303ee3's table measurably closed
  across ALL four speeds, not just one.

### Parameters to configure and store

| # | Parameter | Unit | Per | Source | Stored where |
|---|---|---|---|---|---|
| 1 | dutyPerSpeed | duty/(mm/s) | robot (one value) | population sweep mean (today: baked measured constant) | generated constant → runtime-adapted per issue 04 (see Related) |
| 2 | map gain | 1 | wheel × direction (4) | per-robot calibration sweep | robot JSON → boot bake |
| 3 | map intercept | mm/s | wheel × direction (4) | per-robot calibration sweep | robot JSON → boot bake |
| 4 | vMin (speed floor) | mm/s | robot class | population breakaway spread | generated constant |
| 5 | biasMax (trim authority) | mm/s | robot class | population intercept spread | generated constant |
| 6 | tauAdapt | s | robot class | design choice, bench-verified | generated constant |
| 7 | aSteady (steady gate) | mm/s² | robot class | design choice | generated constant |
| 8 | kp | 1 | robot class | bench-tuned | robot JSON, wire-tunable |
| 9 | ki | 1/s | robot class | bench-tuned | robot JSON, wire-tunable |
| 10 | iMax | mm/s | robot class | bench-tuned | robot JSON, wire-tunable |
| 11 | kaff | s | robot class | measured plant tau (~0.23 s) | robot JSON, wire-tunable |
| 12 | pidMax (fast authority) | mm/s | robot class | design choice ≪ biasMax | robot JSON, wire-tunable |
| 13 | deficitThreshold / deficitWindow | mm/s, ms | robot class | design choice | generated constant |
| 14 | crawlPulse | duty | robot | existing per-robot breakaway property | robot JSON (unchanged) |

Runtime state (NOT configuration): `bias` per wheel (possibly ×
direction) and the PID integrator.

Wire surface: the existing `pid.*` CONFIG keys re-point to rows 8–12
(closing the silent-no-op trap — today they tune the parked duty stage);
rows 1–7 are calibration/population values, not live-tunable.

### Open decisions

1. `bias` per-wheel vs per-wheel-per-direction (the map's intercept
   table is per-direction; the adaptation could start coarser).
2. `bias` persistence across boot — recommend NO at first: load varies
   per session; boot from calibration, adapt fresh.
3. Speed-floor policy: round up to vMin vs refuse the command.
4. dutyPerSpeed: stays one value per robot, or per-direction from the
   population data.
5. Reconcile issue 04 (continuous duty-per-speed calibration) — one
   adaptation charter, not two (see Related).

## Verification

- Phase 1 produces the trim-vs-load dataset and chart (send the chart).
- Phase 2 (if run): the same experiment shows deficit closed without
  reintroducing the cruise-entry overshoot hump.
- Phase 3: the A/B tour + loaded-WHEELS demonstration above; `SET` wire
  keys for the gains route to Drive's controller, no silent no-op.

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
- `04-continuous-duty-per-speed-calibration.md` (referenced from
  drive.h's `kDutyPerSpeed` comment) already charters runtime adaptation
  of the baked duty-per-speed value — overlapping intent. Fold it into
  this issue's Stage C / parameter 1, or mark it superseded by this
  charter; do not run two competing adaptation designs.
