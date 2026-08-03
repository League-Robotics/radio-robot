---
id: '003'
title: 'Speed floor: common-mode only, differential passes through'
status: open
use-cases: [SUC-131-003]
depends-on: ['002']
github-issue: ''
issue: A-speed-floor-snaps-the-planner-differential.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Speed floor: common-mode only, differential passes through

## Description

`Drive::applySpeedFloor()` (`drive.cpp:150-156`) boosts any nonzero
command below `vMin` up to `vMin`, sign preserved. It runs on each wheel's
`cmdVelocity` (`drive.cpp:277-278`) — by which point the planner's
common-mode travel speed and its differential heading/trim correction
(`Planner::applyHeadingHold()`: `cmdLeft_ = profiled - differential;
cmdRight_ = profiled + differential;`) have already been summed into one
number. The floor cannot tell a 3 mm/s differential trim from a 3 mm/s
travel command, so a small steering correction gets quantized to a
~99.7 mm/s lurch. Four sim-tier tour tests FAULT on this.

Fix the floor's SEMANTICS: derive the common-mode and differential
components arithmetically from the two wheels' `cmdVelocity` BEFORE
flooring (`common = 0.5*(L+R)`, `differential = 0.5*(R-L)`, matching
`applyHeadingHold()`'s own sign convention), floor only the common-mode
magnitude via the existing `applySpeedFloor()`, then recombine
(`speedLeft = flooredCommon - differential; speedRight = flooredCommon +
differential`). Feed the result into the rest of `tick()` exactly where
`speedLeft`/`speedRight` are used today.

**This ticket is semantics-only. Re-fitting `vMin`/`biasMax` or any other
floor constant is explicitly OUT OF SCOPE and MUST NOT happen here** — the
current 99.7 mm/s figure is an n=3, unloaded, LOW-CONFIDENCE stand
measurement, and the real loaded-actuation floor needs the robot
translating under its own weight
(`clasi/issues/A-next-physical-bench-session-checklist.md` item 4), which
is unreachable this sprint (`tovez` has been wedged since 2026-08-01). Do
not "helpfully" retune `vMin`/`biasMax`/the deadband while touching this
code — leave every numeric constant exactly as it is; only the point and
shape of the floor's application changes. Likewise out of scope: giving
the planner its own `vMin` awareness (review C1's terminal-taper
mismatch) and re-deriving `settle_epsilon_linear` — both wait on the same
bench measurement and are tracked as a follow-up, not fixed here.

## Acceptance Criteria

- [ ] A sim test commanding a small (~3 mm/s) differential correction on
      top of a >= `vMin` common-mode travel speed shows each wheel's
      resulting command differing by approximately the differential
      amount (proportional), not snapping to a `vMin`-magnitude step.
- [ ] A sim test commanding a small differential correction with a
      near-zero common-mode component (e.g. a terminal approach) still
      passes the differential through unfloored, while the common-mode
      component continues to receive its existing boost-to-`vMin`
      treatment exactly as before (no change to that established
      behavior).
- [ ] The sim-tier equivalents of the four previously-FAULTing tour tests
      (exercising a differential steering correction) pass.
- [ ] No regression in the square-tour/circle-tour closure gates at the
      sim tier (square <= 80mm, circle 9.6mm).
- [ ] `Drive::AdaptationBounds::vMin`'s numeric value (and `biasMax`, and
      the output deadband) are byte-for-byte unchanged by this ticket —
      confirmed by diffing `data/robots/*.json` and `drive.h`'s
      constants before/after. The ticket's own completion notes state
      explicitly that the floor's semantics changed, not its calibration,
      and reference the deferred bench measurement for future re-fitting.
- [ ] Full sim suite stays green.

## Testing

- **Existing tests to run**: `App::Drive` Stage A/B/C harness (floor
  scenarios), sim-tier tour/closure tests, full `src/tests/sim` suite.
- **New tests to write**:
  - Sim test: differential-trim-with-floored-common-mode produces a
    proportional per-wheel split, not a floor-magnitude step.
  - Sim test: differential-trim-with-near-zero-common-mode passes through
    unfloored while common-mode boost behavior is unchanged.
  - Re-run of the four previously-FAULTing sim-tier tour tests.
- **Verification command**: `uv run python -m pytest src/tests/sim`.

## Implementation Plan

**Approach**: In `Drive::tick()`, before calling `applySpeedFloor()` per
wheel, compute the common-mode and differential components from
`state.wheelLeft.cmdVelocity`/`state.wheelRight.cmdVelocity` (verify sign
convention against `Planner::applyHeadingHold()`'s own mixing before
implementing). Floor only the common-mode magnitude via the existing,
UNCHANGED `applySpeedFloor()` body. Recombine into `speedLeft`/`speedRight`
and feed them into Stage A/B/C exactly as today.

**Files to modify**:
- `src/firm/app/drive.cpp` — `tick()`'s call sites into
  `applySpeedFloor()` only; `applySpeedFloor()`'s own body is unchanged.
- Existing `App::Drive` test harness — new floor/differential scenarios.

**Testing plan**: as listed above.

**Documentation updates**: `applySpeedFloor()`'s doc comment in
`drive.cpp` updated to state it is now applied to the common-mode
component only, with a cross-reference to sprint.md's Design Rationale
Decision 4 for why the planner-side `vMin` awareness (review C1) is
deferred rather than fixed here.
