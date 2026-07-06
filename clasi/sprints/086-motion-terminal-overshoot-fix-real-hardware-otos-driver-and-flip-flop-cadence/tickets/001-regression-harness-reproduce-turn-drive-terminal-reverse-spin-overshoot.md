---
id: "001"
title: "Regression harness: reproduce turn/drive terminal reverse-spin overshoot"
status: open
use-cases: [SUC-001, SUC-002]
depends-on: []
github-issue: ""
issue: motion-turn-drive-terminal-overshoot.md
completes_issue: true
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

- [ ] A new sim-level test (e.g. `tests/sim/unit/test_motion_overshoot_regression.py`
      or an addition to an existing motion-commands test file) drives an
      `RT 9000` through `libfirmware_host`, samples per-wheel `vel(L,R)` at
      and after `EVT done rt`, and asserts a **tight** post-completion
      residual-velocity bound (no sustained reverse-sign residual beyond the
      bound) — written so it currently FAILS against today's code, with the
      measured pre-fix magnitude recorded in the test's own docstring/comment
      for traceability.
- [ ] A companion case does the same for a `D 200 200 500`, asserting final
      traveled distance within a tight tolerance of commanded (materially
      tighter than the pre-fix ~7%) — currently FAILING against today's code.
- [ ] Both tests reference this ticket and the parent issue in their
      docstrings, and state plainly that they are expected to start passing
      once ticket 002 (and, for full tolerance, ticket 003) lands.
- [ ] No production code (`source/`) is touched by this ticket.

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
