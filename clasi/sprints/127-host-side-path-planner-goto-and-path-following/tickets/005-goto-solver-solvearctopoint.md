---
id: '005'
title: 'Goto solver: solveArcToPoint'
status: open
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

- [ ] `solveArcToPoint()` is a pure function: same inputs always produce
      the same outputs, no I/O, no hidden/module-level mutable state.
- [ ] Curvature slew limit constant is set from ticket 001's measured
      Edge-B discontinuity value (cite the specific number and where it
      came from in this ticket's Completion Notes) — not an independently
      chosen or guessed constant.
- [ ] Curvature slew limit has its own unit test: a rapid sequence of
      solver calls toward targets that would otherwise demand a large
      curvature step is shown to ramp, not step, across calls.
- [ ] Target-behind guard has its own unit test: a target placed behind
      the robot's current heading produces a stop-then-turn result (an
      explicit "stop" signal, not a degenerate/near-infinite curvature
      twist).
- [ ] Unit tests also cover an on-heading target (near-zero curvature) and
      an off-heading target within the normal range (moderate curvature).
- [ ] The solver never returns an Angle-stopped move — always a
      Distance-stopped twist on the Linear axis (matches the "never
      change axis while moving" constraint from the design issue).
- [ ] The function signature/docstring states explicitly that the
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
