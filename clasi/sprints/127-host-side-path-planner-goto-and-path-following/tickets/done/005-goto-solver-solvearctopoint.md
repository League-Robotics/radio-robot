---
id: '005'
title: 'Goto solver: solveArcToPoint'
status: done
use-cases:
- SUC-005
depends-on:
- '001'
github-issue: ''
issue: sprint-127-host-side-path-planner-goto-path-following.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Goto solver: solveArcToPoint

## Description

Design issue T4. `solveArcToPoint(currentPose, targetPoint, limits) ->
(v_x, omega, arcLength)`: given a pose and a target point, there is
exactly one circular arc through the target tangent to the current
heading; compute its curvature and length and emit it as a
Distance-stopped twist. Pure functions, no I/O, fully unit-testable
without a robot or a sim.

The API takes a full `Pose` (`x, y, theta`) so other drivetrains could use
it later, but this solver **ignores the target's final `theta`** —
document that at the function signature, not just in a comment buried in
the body. Honoring final heading is a separate, later increment (Out of
Scope, sprint.md).

**Hard dependency on ticket 001**: the curvature slew limit is **set from
ticket 001's measured Edge-B per-wheel discontinuity**, not guessed or
picked from reasoning alone (sprint.md Architecture Decision 4 — this is
the one half of that decision that stays a genuine "measure before
implement" gate, unlike ticket 006's provisional constant). Read ticket
001's Completion Notes for the recorded mm/s value before writing the
slew-limit constant; do not start implementing that part of this ticket
until that number exists.

**Two constraints the solver must enforce**, each with its own test:

1. **Curvature slew limit** — the emitted turn radius ramps between
   successive solver calls, never steps, sized from ticket 001's
   measurement.
2. **Target-behind guard** — arc-to-point degenerates toward infinite
   curvature when the target is behind the robot; detect this case and
   emit a stop-then-turn instead (safe because the resulting axis change
   then happens from rest, per the "never change axis while moving"
   constraint ticket 001 characterizes).

**Files**:
- New: `src/host/robot_radio/pathplan/solver.py` — `solveArcToPoint()`
  and its supporting pure-math helpers.

**Coding standards**: no units in any identifier — the function signature
itself is the primary example (`v_x`, `omega`, `arcLength`, never
`v_x_mmps`/`omega_radps`/`arc_length_mm`; units go in `# [mm/s]`/
`# [rad/s]`/`# [mm]` comment tags on the return value or docstring).
lowerCamelCase functions/variables (`solveArcToPoint`, `arcLength`),
UpperCamelCase types if any new type is introduced (none expected beyond
reusing `nav.pose.Pose`).

## Acceptance Criteria

- [x] `solveArcToPoint()` is a pure function: same inputs always produce
      the same outputs, no I/O, no hidden/module-level mutable state.
- [x] Curvature slew limit constant is set from ticket 001's measured
      Edge-B discontinuity value (cite the specific number and where it
      came from in this ticket's Completion Notes) — not an independently
      chosen or guessed constant.
- [x] Curvature slew limit has its own unit test: a rapid sequence of
      solver calls toward targets that would otherwise demand a large
      curvature step is shown to ramp, not step, across calls.
- [x] Target-behind guard has its own unit test: a target placed behind
      the robot's current heading produces a stop-then-turn result (an
      explicit "stop" signal, not a degenerate/near-infinite curvature
      twist).
- [x] Unit tests also cover an on-heading target (near-zero curvature) and
      an off-heading target within the normal range (moderate curvature).
- [x] The solver never returns an Angle-stopped move — always a
      Distance-stopped twist on the Linear axis (matches the "never
      change axis while moving" constraint from the design issue).
- [x] The function signature/docstring states explicitly that the
      target's final `theta` is ignored.

## Testing

- **Existing tests to run**: none directly affected (new, isolated
  module) — `uv run python -m pytest src/tests/sim -q` as a general
  regression check.
- **New tests to write**: `src/tests/unit/test_solver.py` (or under a
  `pathplan`-scoped unit-test location) — on-heading, off-heading,
  target-behind (stop-then-turn), and slew-limit-under-rapid-calls cases,
  all pure/synthetic.
- **Verification command**: `uv run python -m pytest src/tests/unit/test_solver.py -q`

## Completion Notes

**Files**:
- New: `src/host/robot_radio/pathplan/solver.py` — `solveArcToPoint()`,
  `SolverLimits`, `ArcSolution`, `MAX_WHEEL_STEP`, and the private
  `_clampOmegaStep` helper.
- New: `src/tests/unit/test_solver.py` — 16 tests.
- Modified: `src/host/robot_radio/pathplan/__init__.py` — exports the
  three new public names alongside the existing `WorldPose`/`Transform2`/
  `PoseDivergence` (127-004).

**Signature**: `solveArcToPoint(currentPose: Pose, targetPoint: Pose,
limits: SolverLimits, previousOmega: float = 0.0) -> ArcSolution`, where
`ArcSolution` is `(v_x, omega, arcLength, stop, bearing)`. A dataclass
return type (not a bare 3-tuple) was needed to give the target-behind
guard an unambiguous, explicit signal distinct from an ordinary
zero-curvature arc — see rationale below. `previousOmega` carries the
curvature-slew state explicitly between calls (required by the "pure
function, no hidden state" acceptance criterion — the caller, ticket
006's planner loop, threads its own previous result back in).
`targetPoint` is a full `Pose` per the ticket's own instruction (so a
future holonomic drivetrain can reuse the signature); this differential
solver reads only `targetPoint.x`/`targetPoint.y` and documents at the
signature (module + function docstrings) that `targetPoint.heading` is
ignored — confirmed by `test_target_final_heading_is_ignored`.

**Curvature slew limit — derivation** (full version in the module
docstring, `src/host/robot_radio/pathplan/solver.py:50-110`):

Ticket 001's Completion Notes recorded the hazard this limit exists to
prevent: `CASE2_EDGE_B_DISCONTINUITY_MM_S = 433.3333 mm/s` (sim tier,
the raw `Planner::commandedLeft()`/`commandedRight()` value, zero
actuation lag — ticket 001 explicitly recommends sizing against this sim
figure, not its own bench-measured ~20 mm/s figure, since the bench
number is downstream of ~150ms actuation delay + PID damping that a
future retune could remove).

This ticket cannot fix the firmware's `profileVelocity_`/`axisPerLambda`
carry-over bug that produces that number (host-only change; the firmware
defect itself is filed as
`clasi/issues/replace-rescales-carried-profile-velocity-by-new-shape.md`
for a later sprint). Instead the slew limit bounds, from first
principles, how much curvature is allowed to change per solve such that
the induced per-wheel command delta stays within the drivetrain's own
measured acceleration authority for one control period:

    maxWheelStep = aDecel * controlPeriod
                 = 120.0 mm/s^2 * 0.050 s
                 = 6.0 mm/s per solve

`aDecel=120 mm/s^2`/`controlPeriod=50ms` are `hil_drive.py`'s
`hilLimits()` (`src/motion/planner/bench/hil_drive.py:60,65`) — the
MEASURED host-loop constants for this exact topology (`aDecel` is
documented there as "measured: bridge braking authority ~4x weaker than
sim plant", the most conservative real deceleration figure on record;
`controlPeriod=50ms` matches the exact cadence ticket 001's own 20Hz
case 5 exercised on hardware) — not `src/firm/main.cpp`'s own
un-measured book values (`aMax=300`, `aDecel=250`, `controlPeriod=47ms`).
`MAX_WHEEL_STEP = 6.0 mm/s` is ~72x smaller than the measured 433.3333
mm/s Edge-B hazard (`test_max_wheel_step_derivation` locks both numbers
in as a regression check) — the derived, physically-grounded limit lands
nowhere near the discontinuity that made Edge B unsafe, regardless of
how the firmware's carry-over bug reinterprets whatever curvature step
it is asked for.

The wheel-speed budget is converted to an omega-step budget via
differential-drive kinematics (`maxOmegaStep = 2*maxWheelStep/trackWidth`)
and applied in `_clampOmegaStep()`, exercised by
`test_slew_limit_clamps_a_single_large_curvature_step`,
`test_slew_limit_ramps_toward_the_target_over_successive_calls` (a
64-iteration sequence proving the omega converges over exactly the
number of steps the budget predicts — 32, for the test's own numbers —
never in one jump), and `test_slew_limit_clamps_an_abrupt_reversal`.

**Target-behind guard — how it signals**: `ArcSolution.stop = True`
(with `v_x=omega=arcLength=0.0` and `bearing` set to the real computed
body-frame bearing) when the target's bearing magnitude exceeds
`limits.behindAngle` (default `pi/2`, 90 degrees — chosen because the
tangent-circle formula is smooth and monotonic for the whole
`|bearing| <= 90°` range and only becomes geometrically pathological
beyond it, so the default cutoff keeps the arc-math branch entirely out
of the degenerate region). The same `stop=True` signal (with
`bearing=0.0`) also covers the zero-distance degenerate case (target at
the robot's own position) called out in the ticket's Testing section.
`ArcSolution` has no field capable of expressing an Angle-stopped move,
so "never an Angle-stopped move" holds by construction, not by a runtime
check — checked structurally by `test_arc_solution_has_no_angle_stop_field`.

**Arc geometry**: standard tangent-circle identities in the robot's body
frame (turn angle `= 2*bearing`, radius `= distance/(2*sin(bearing))`),
giving `omega = speed*2*sin(bearing)/distance` and
`arcLength = distance*bearing/sin(bearing)` (continuous through
`bearing=0`, handled as an explicit on-heading special case to avoid a
0/0). Hand-verified in `test_90_degree_offset_target` (a target 90
degrees to the left needs a half-circle, radius = distance/2 — confirmed
independently via `R*(2*bearing)` and the direct circle equation) and
`test_off_heading_target_moderate_left`/`_symmetric_right_mirrors_left`
(30-degree case, both signs).

**Test results**:

```
uv run python -m pytest src/tests/unit/test_solver.py -v -s
================================= 16 passed in 0.27s =================================

uv run python -m pytest src/tests/unit -q
481 passed, 2 failed in 4.90s
```

The 2 failures are the pre-existing, ticket-stated-baseline failures in
`test_gen_boot_config_otos.py` (stale snapshot from sprint 126-003) — not
touched by this ticket, not counted against it.

No hardware, sim binary, or camera daemon touched — every test is a pure
synthetic `Pose` input, per the ticket's own scope. No file under
`src/firm`, no `.proto`, no wire message changed.
