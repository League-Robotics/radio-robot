---
id: '004'
title: App::Drive setWheels staging path
status: done
use-cases:
- SUC-050
- SUC-051
depends-on: []
github-issue: ''
issue: protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# App::Drive setWheels staging path

## Description

Additive-only extension of `App::Drive` (`src/firm/app/drive.{h,cpp}`):
`setWheels(v_left, v_right)` — a second, independent staging path alongside
the existing `setTwist(v_x, v_y, omega)`, per sprint.md's Design Rationale
Decision 3. Stages raw wheel velocities directly, bypassing
`BodyKinematics::inverse()` entirely — a `MoveWheels` command tells the
robot exactly what wheel speeds it wants, and this sprint keeps that
honest rather than routing it through a forward/inverse round trip that
buys nothing on today's differential base (see the sprint.md rationale for
why: `v_y` already rides `MoveTwist` wire-forward for a future holonomic
base, and the wire's twist-vs-wheels oneof exists specifically because a
future non-differential base's wheels won't always correspond to one body
twist).

`tick()` computes from whichever of `setTwist()`/`setWheels()` was called
most recently (last-wins, a small internal mode flag); `stop()` clears
both paths to zero regardless of which was staged. `setTwist()`/`stop()`'s
existing signatures and behavior for every current caller are unchanged.

Independent of tickets 001/002/003 — no wire or queue dependency, purely a
`Drive` internal addition. Only ticket 005 (`MoveQueue`) actually calls
`setWheels()` at runtime.

## Acceptance Criteria

- [x] `setWheels(v_left, v_right)` added; stages the two values directly
      onto the left/right leaves' targets with no `BodyKinematics::inverse()`
      call in between.
- [x] Last-wins: whichever of `setTwist()`/`setWheels()` was called most
      recently determines what the next `tick()` call computes/stages.
- [x] `stop()` clears both staging paths to zero, regardless of which was
      last active.
- [x] `setTwist()`'s existing behavior (stages through
      `BodyKinematics::inverse()`, `v_y` accepted-and-ignored) is
      unchanged for every existing caller.

## Testing

- **Existing tests to run**: `src/tests/sim/unit/test_app_drive.py` (must
  stay green — every existing `setTwist()`/`stop()`/`tick()` assertion
  unaffected).
- **New tests to write**: `setWheels()` then `tick()` stages the raw
  `v_left`/`v_right` values onto the two leaves unchanged (no `inverse()`
  involvement — verify against a `(v_x, omega)` pair that would NOT
  round-trip to the same wheel speeds if `inverse()` were mistakenly
  applied); `setTwist()` called after `setWheels()` overrides to the
  twist-derived values on the next `tick()`; `setWheels()` called after
  `setTwist()` overrides to the raw wheel values; `stop()` zeroes both
  leaves' targets regardless of which staging path was active beforehand.
- **Verification command**: `uv run python -m pytest
  src/tests/sim/unit/test_app_drive.py`
