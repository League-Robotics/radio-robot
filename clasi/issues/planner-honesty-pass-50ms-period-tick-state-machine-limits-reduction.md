---
status: pending
---

# Planner honesty pass: 50 ms period everywhere, tick() as a real state machine, PlannerLimits 34 → 23

## Description

Three stakeholder directives from the 2026-07-31 planner design review, plus
the precondition decision that unlocks them:

1. **Move the control period to 50 ms, one constant, sim and hardware
   identical.**
2. **Redesign `Planner::tick()` from an implicit state machine (~8
   interacting booleans) into an explicit one.**
3. **Reduce `PlannerLimits` from 34 fields to the set that is actually
   needed, breaking the append-only ctypes constraint once, deliberately.**
0. **Precondition: delete the parked duty stage** (`WheelPid`,
   `Planner::stageDuty()`), reversing sprint 128 Decision 2's PARK now that
   the velocity trim is the proven control law on the bench.

## Cause

- **The "47 ms" period is an overrun artifact, not a design.**
  `RobotLoop::kCycle` is 40; the loop cannot fit in 40 (measured busy
  ~21 ms + vendor bus clearances), so the pacer never sleeps and the
  delivered ~46–48 ms drifts with load. `boot_wiring` hardcodes 47 for the
  planner; the sim steps at the kCycle-derived 40 — three numbers, none of
  them honest. All of the planner's discrete-exact accounting assumes
  dt == controlPeriod, so every mismatch silently degrades exactness.
  The planner's own ctest suites already run at kPeriod = 50.
- **`tick()` (planner.cpp:451, ~240 lines) encodes the move lifecycle in
  interacting booleans** (`occupied`, `hasMoved`, `settling`,
  `decelLatched`, `closingIssued`, `stallTicks`, `carryValid_`,
  `activeBoundary_`) with the completion priority order existing only as
  code order. Every completion path added so far was justified by a
  measured field failure; each new one multiplies interactions.
- **`PlannerLimits` has 34 flat fields with an append-only ABI constraint**
  (planner_harness.py ctypes mirror). Grep-verified: 7 fields feed only the
  parked duty stage, 2 feed the fail-closed-off OTOS heading blend, 2 feed
  the everywhere-false settle-confirm feature.
- **The parked duty stage is a live trap**: the `pid.*` CONFIG wire keys
  route to `applyVelGains()` (configurator.cpp:145), so live-tuning
  `pid.kp` on the bench today tunes a controller that is not running —
  a silent no-op with a plausible name. (The stand-tuning memory that
  `SET pid.*` works dates from when the duty stage ran.)

## Proposed fix

### 0. Delete the duty stage (precondition)

Delete `wheel_pid.{h,cpp}`, `Planner::stageDuty()`, `dutyLeft_`/`dutyRight_`
and their accessors, `wheel_pid_test.cpp`, `planner_duty_scenarios_test.cpp`,
`tests/duty_plant.h`, and the ctest registrations. Repoint or retire the
`pid.*` wire keys: either route them to the trim gains (`applyTrimGains`) or
reject them with a clear error — never a silent no-op. Update
`src/motion/DESIGN.md`'s "wheel control generations" note (generation 2:
deleted; generation 3, WheelTrim, is the law). Git preserves the class for
any future duty-sink revisit.

### 1. One period, 50 ms

- `App::RobotLoop::kCycle` 40 → 50 (the pacer paces again: busy ~21 ms
  < 50, so the delivered period becomes exactly 50 and stable).
- `App::bootPlannerLimits()` `controlPeriod`/`actuationDelay` 47 → 50.
- Sim: `SimHarness::kCycleDtUs` already derives from `kCycle` — verify the
  sim harness's own planner limits use the same derived constant instead of
  a separate literal (40.0f in `simPlannerLimits()` today).
- Telemetry `kPrimaryPeriod` (40 ms): confirm intended behavior at a 50 ms
  cycle (emission is per-cycle gated; document the effective rate).
- Re-verify tuning at the new dt: trim gains and `decelPlanFraction` were
  tuned at ~47 ms. Run the bench square tour before re-blessing numbers.

### 2. Explicit move lifecycle in tick()

Introduce a lifecycle enum and make `tick()` a dispatcher over state
handlers with one visible transition table:

| State | Meaning |
|---|---|
| `Idle` | no active move, command at zero |
| `Draining` | no active move, ramping staged command to zero |
| `Breakaway` | active, not yet left rest (stall backstop disarmed) |
| `Tracking` | profile driving; `MovePhase` (Accel/Hold/Decel) as sub-phase |
| `Stopping` | a `Kind::Stop` entry ramping to rest |

Events: profile-complete, arrived (epsilon + at rest), stall-window expiry,
timeout, estop, replace, queue-empty. Completions become transition actions
emitting `TickResult`. `hasMoved` and `settling` dissolve into state
identity; `decelLatched` stays a `MovePhase` latch. The settle-confirm
defer path (and its `Settling` state) is deleted with `requireSettle`
(below) — arrival completion already covers it. Behavior-preserving except
where this issue says otherwise; the scenario suites
(`planner_scenarios_test`, `planner_noise_test`) are the gate.

### 3. PlannerLimits 34 → 23, grouped

Cut (11): `velKff` `velKp` `velKi` `velIMax` `velKaff` `velIAccelGate`
`dutyFloor` (duty-stage-only, dead with #0); `headingOtosWeight`
`otosStaleness` (OTOS blend off everywhere; estimator-v2 is separate,
tracked work); `requireSettle` `settleWindow` (feature false everywhere —
note `plannedStopWindow()` falls back to its built-in default allowance
once `settleWindow` is gone).

Keep 23, grouped into sub-structs:

- `ceilings`: vMax aMax aDecel omegaMax alphaMax alphaDecel jerkMax yawJerkMax
- `plant`: trackWidth controlPeriod actuationDelay velocityFilterWeight
- `landing`: settleEpsilonLinear settleEpsilonAngular settleRestVelocity
  settleRestOmega decelPlanFraction
- `tracking`: trimKp trimKi trimIMax trimKaff trimMax headingHoldGain

Break append-only ONCE: regenerate the `planner_harness.py` ctypes mirror
and the per-field offset guards (`plannerStructSizes`) against the new
layout. Update every composition root (`boot_wiring`, `simPlannerLimits`,
test `benchLimits()`), and `applyShaperLimits`/`applyTrimGains` signatures
if grouping suggests passing sub-structs.

## Verification

- `planner_tests` (all ctest suites) green after each step; the exactness
  gates in `planner_scenarios_test` are the primary behavior guard for #2.
- Telemetry `cycle_period` on the bench reads 50 ± jitter, stable under
  load (the pacer is pacing, not overrunning).
- `SET pid.kp` over the wire either visibly tunes the trim or returns an
  error — no silent no-op path remains.
- Bench square tour closure at 50 ms is at least as good as the 47 ms
  baseline before re-blessing tuned constants (goldens re-blessed with a
  why, per the golden process).
- Offset guard passes on the regenerated ctypes mirror.

## Related

- [`unify-sim-and-robot-composition-roots.md`](unify-sim-and-robot-composition-roots.md)
  — the one-period constant is that issue's item 4; this issue settles the
  value at 50.
- [`minimal-system-test-one-program-tour-files-one-jsonl-dataset-dbg-fault-injection.md`](system-test-minimal-system-test-one-program-tour-files-one-jsonl-dataset-dbg-fault-injection.md)
  — the `cycle_period` golden is what catches a future period regression.
- `src/motion/DESIGN.md` §6 — the duty-vs-trim open question this issue
  closes (answer: WheelTrim is the law; duty stage deleted).
- `clasi/issues/later/estimator-v2-otos-fusion-sim-first.md` — where the
  cut OTOS-blend fields' concern lives on.
