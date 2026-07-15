---
id: '004'
title: Pure trapezoidal profile generator (straight distance + in-place turn)
status: done
use-cases:
- SUC-027
depends-on: []
github-issue: ''
issue:
- host-planner-design-lessons-from-drive-v2-review.md
- heading-loop-output-clamp-and-velocity-resonance.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Pure trapezoidal profile generator (straight distance + in-place turn)

## Description

No host-side trajectory generator exists yet. `host/robot_radio/nav/`'s
`navigator.py`/`camera_goto.py` call retired blocking verbs (`Robot.go_to()`,
`NezhaProtocol.drive()`) with no P4 equivalent
(`nezha-facade-and-midlayer-dead-verb-residue.md`) and are NOT reused this
sprint — `nav/`'s own fate is a separate, future stakeholder call, per that
issue's own Direction (`architecture-update.md` Decision 3). No jerk-limited
trajectory library is an actual project dependency today (`bench_ruckig_
motion_verify.py`'s name notwithstanding — it targets retired pre-102 text
verbs).

This ticket builds a new, PURE `host/robot_radio/planner/profile.py` module:
given a signed straight-line distance OR a signed in-place turn angle, plus
acceleration/cruise/deceleration limits, it produces a deterministic ordered
sequence of `(elapsed, v_x, omega)` setpoints — a classic trapezoidal
(accelerate/cruise/decelerate) shape, collapsing to a triangle profile when
the limits never let cruise be reached. The final decelerating leg always
lands at EXACTLY zero velocity at the target — never a sign-reversal
"creep back" (binding requirement #7). Every input is validated (finite,
in-range, non-degenerate) at the boundary before any setpoint is generated
(binding requirement #5). No I/O, no robot/sim dependency, no wall-clock
read — fully unit-testable, and the sprint's most stable, most reusable
module (zero outward dependencies in the architecture's own dependency
graph). No dependency on any other ticket in this sprint.

## Acceptance Criteria

- [x] A function generating a straight-distance profile and a function
      generating a turn-angle profile (names are this ticket's own call,
      e.g. `profile_for_distance(distance, limits)`/`profile_for_turn(angle,
      limits)`) each return a deterministic setpoint sequence.
- [x] Unit tests assert: the acceleration phase never exceeds the configured
      `a_max`; the cruise phase (when reached) holds exactly `v_max`; the
      final setpoint lands at exactly zero velocity with the commanded sign
      preserved throughout the WHOLE sequence (never a `fabsf`-blind
      predicate anywhere in the generator or its tests — binding
      requirement #1).
- [x] A short-distance/short-angle case that never reaches cruise (a pure
      triangle profile) is covered by its own test and produces a shape
      distinct from — not a truncated/incorrect version of — the trapezoid
      case.
- [x] Both a positive and a negative (reverse/CW) input are tested, proving
      the sign is preserved through every setpoint, not just the first/last.
- [x] A degenerate/invalid input (zero distance/angle, a non-positive limit,
      a non-finite value) raises immediately rather than producing any
      setpoint sequence.
- [x] 100% unit-tested under `tests/unit/`, with zero hardware or sim
      dependency — importable and testable with no robot connected.
- [x] Full project test suite green.

## Completion Notes

Implemented exactly per the plan: `host/robot_radio/planner/profile.py`
holds `ProfileLimits`/`ProfileSetpoint` dataclasses, the shared internal
`_scalar_trapezoidal_profile()` timing helper, and the two public entry
points `profile_for_distance()`/`profile_for_turn()`, which map the same
scalar-velocity sequence onto `v_x` (omega=0) and `omega` (v_x=0)
respectively — no duplicated shape math. Validation
(`_validate_scalar`/`_validate_limits`/`_validate_cadence`) runs at the top
of the shared helper before any setpoint is generated, rejecting
non-finite/non-positive limits and zero/non-finite distance/angle/cadence
with `ValueError`. Cadence is a caller-supplied parameter (default
`DEFAULT_CADENCE = 0.05`), not a buried constant — no latency modeling in
this ticket (that is ticket 005's `model.py`/`executor.py`, per its own
Implementation Plan). `planner/__init__.py` re-exports the public API and
documents that this ticket ships only `profile.py` (executor/heading/model
are 106-005) and that the package deliberately does not reuse `nav/`
(architecture-update.md Decision 3).

`tests/unit/test_planner_profile.py` adds 48 tests across six sections:
trapezoid shape (area-under-curve via trapezoidal-rule integration, cruise
plateau, accel-phase slope bound), triangle shape (never reaches
v_max/omega_max, distinct from trapezoid — no plateau), sign preservation
(positive/negative distance and angle, mirror-image check, no
sign-reversal-anywhere check), terminal velocity (parametrized exact-zero
checks), cadence sampling (spacing, monotonic elapsed, first-setpoint-at-
rest, finer-cadence-more-setpoints), and validation (zero distance/angle,
non-positive v_max/a_max, non-finite distance/angle/v_max/a_max/cadence,
each parametrized over inf/-inf/nan as applicable).

Full suite: `uv run python -m pytest` — 617 passed (0:01:25), including the
new 48. No regressions.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` (this ticket
  adds new files only; no regression surface in existing code).
- **New tests to write**: `tests/unit/test_planner_profile.py` — trapezoid
  shape, triangle shape, sign preservation (both directions, straight and
  turn), exact-zero terminal velocity, invalid-input rejection (per each
  Acceptance Criterion above).
- **Verification command**: `uv run python -m pytest tests/unit/
  test_planner_profile.py -v`, then the full suite.

## Implementation Plan

**Approach**: Define a small `ProfileLimits` dataclass (`v_max`, `a_max` —
`j_max`/jerk-limiting explicitly out of scope this sprint, per
`architecture-update.md` Step 1 finding 3) and a `ProfileSetpoint` dataclass
(`elapsed`, `v_x`, `omega`). Implement ONE internal scalar trapezoidal-timing
helper (accelerate/cruise/decelerate over a signed scalar distance, given
`v_max`/`a_max`) — the shape math is identical whether the scalar being
profiled is a linear distance (mm) or an angle (rad); `profile_for_distance()`
maps the resulting scalar-velocity sequence onto `v_x` (omega=0 throughout),
and `profile_for_turn()` maps the SAME scalar-velocity sequence onto `omega`
(v_x=0 throughout) — no code duplication between the two public entry
points. Validate every input at the very top of each public function (raise
`ValueError` on non-finite/non-positive limits, or a zero/non-finite
distance/angle — zero is itself invalid per this ticket's own acceptance
criterion, not silently treated as a no-op profile). The sampling cadence
(how finely `elapsed` is discretized into individual setpoints) is a
PARAMETER the caller (ticket 005's executor) supplies — `profile.py` itself
imports nothing from `planner/model.py`, preserving its zero-outward-edge
boundary from the architecture's own dependency graph.

**Files to create**:
- `host/robot_radio/planner/__init__.py`
- `host/robot_radio/planner/profile.py`
- `tests/unit/test_planner_profile.py`

**Files to modify**: none.

**Testing plan**: exhaustive pure-Python unit tests per the Acceptance
Criteria above — no mocks, no fakes, no hardware/sim needed, since the
module has no I/O.

**Documentation updates**: `profile.py`'s own module and function
docstrings document the trapezoidal/triangle shape, the sign convention, and
the explicit non-goals (no jerk limiting, no cadence opinion, no I/O). No
external doc changes required this ticket (the `host/robot_radio/README.md`
package-layout refresh is deferred, per `architecture-update.md` Step 7 Open
Question 6).
