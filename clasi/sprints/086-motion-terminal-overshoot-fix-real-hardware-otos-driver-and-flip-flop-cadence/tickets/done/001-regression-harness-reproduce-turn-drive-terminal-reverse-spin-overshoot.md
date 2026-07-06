---
id: "001"
title: "Regression harness: reproduce turn/drive terminal reverse-spin overshoot"
status: done
use-cases: [SUC-001, SUC-002]
depends-on: []
github-issue: ""
issue: motion-turn-drive-terminal-overshoot.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Regression harness: reproduce turn/drive terminal reverse-spin overshoot

## Description

Before any fix lands, reproduce the issue's own root-caused, measured
failure signature as a deterministic sim-level test so tickets 002/003 have
a concrete "before" baseline to flip from failing to passing, instead of
fixing against prose alone.

Two behaviors to reproduce quantitatively, matching the issue's own data:

1. **Turn reverse-spin**: an `RT 9000` (+90°) turn. Sample per-wheel `vel(L,R)`
   via `SNAP` across the completion tick and ~800 ms after. The issue's own
   measured signature: `vel(L,R)` holds at roughly `(-43, +43)` through
   `EVT done rt` (heading correct, `mode=I`), then flips to roughly
   `(+52, -52)` — a reverse-sign magnitude that *exceeds* what it was
   arresting — before settling back over the next several hundred ms with a
   ~4-10° heading backtrack.
2. **Drive overshoot**: a `D 200 200 500` (500 mm at 200 mm/s). Final
   encoder-measured distance overshoots the commanded 500 mm by roughly 7%
   in the issue's own measurement.

This ticket is a pure test addition — no production code changes. It is
expected to fail (or pass only against today's loose tolerances) until
ticket 002 lands; its purpose is to make that failure explicit and
measurable, and to give ticket 002 a concrete regression to flip.

## Acceptance Criteria

- [x] A new sim-level test (e.g. `tests/sim/unit/test_motion_overshoot_regression.py`
      or an addition to an existing motion-commands test file) drives an
      `RT 9000` through `libfirmware_host`, samples per-wheel `vel(L,R)` at
      and after `EVT done rt`, and asserts a **tight** post-completion
      residual-velocity bound (no sustained reverse-sign residual beyond the
      bound) — written so it currently FAILS against today's code, with the
      measured pre-fix magnitude recorded in the test's own docstring/comment
      for traceability.
- [x] A companion case does the same for a `D 200 200 500`, asserting final
      traveled distance within a tight tolerance of commanded (materially
      tighter than the pre-fix ~7%) — currently FAILING against today's code.
- [x] Both tests reference this ticket and the parent issue in their
      docstrings, and state plainly that they are expected to start passing
      once ticket 002 (and, for full tolerance, ticket 003) lands.
- [x] No production code (`source/`) is touched by this ticket.

## Completion Notes (086-001)

Implemented `tests/sim/unit/test_motion_overshoot_regression.py` with two
`xfail(strict=True)` tests, both driving `libfirmware_host` via the `sim`
fixture at 24ms tick resolution:

- `test_rt_9000_settles_without_sustained_reverse_spin_residual`: samples
  `vel(L,R)` via `SNAP` across `RT 9000`'s completion and 800ms after.
  Measured pre-fix (2026-07-06, this commit): `EVT done RT` fires at
  t=864ms elapsed with vel(L,R)=(-93,+93) (still spinning), crosses zero at
  +72ms, then a SUSTAINED reverse-sign residual oscillates ~2-7 mm/s through
  the whole 800ms window (worst case 7.0 mm/s at +264ms after done, vs a
  200-800ms-post-done tight bound of 2.0 mm/s) — asserts FAILS today.
- `test_d_200_200_500_stops_within_tight_tolerance_of_commanded_distance`:
  measures true-pose distance at `EVT done D`. Measured pre-fix: 532.51mm
  (+6.50% over the 500mm target) against a 1.5% (7.5mm) tight-tolerance
  assertion — FAILS today. (Matches the issue's own ~7%/~535mm figure
  closely; a longer trace also shows the same reverse-spin residual then
  rolls the robot backward past 500mm again, down to 486.43mm by +1920ms.)

Both tests xfail cleanly (`2 xfailed`); full `tests/sim` suite: `246 passed,
2 xfailed`. No `source/` changes. `completes_issue: true` on this ticket's
frontmatter is aspirational for the whole issue — the issue itself is only
truly resolved once 086-002 (motor-loop fix), 086-003 (terminal decel
anticipation), and 086-004 land and these two `xfail` markers are removed
(`strict=True` will hard-fail the suite the moment that's not done, forcing
the marker's removal rather than a silent unexpected-pass).

## Implementation Plan

**Approach**: Follow the existing pattern in `tests/sim/unit/
test_motion_commands_arc_turn.py` (drives `libfirmware_host` via
`Sim.command()`/`sim.tick_for()`, reads `SNAP`/sim ground truth) and
`tests/sim/unit/test_planner.py`'s harness conventions. Sample `vel(L,R)`
at a fine enough tick resolution across the stop transition to capture the
issue's own measured shape (sustained-then-reversed), not just a single
post-stop sample.

**Files to create/modify**:
- New test file (or extension of an existing `tests/sim/unit/` motion test
  file) — test-only, no production code.

**Testing plan**:
- Run the new test(s) and confirm they fail against current `main` in the
  documented way (reverse-spin / overshoot magnitude roughly matching the
  issue's own numbers) — capture that failing output in the ticket's
  completion notes so ticket 002's fix has a clear "before" reference.
- Run the full existing `tests/sim/unit/` suite to confirm this ticket adds
  no regressions (it should not — it's additive).

**Documentation updates**: None (test-only ticket).
