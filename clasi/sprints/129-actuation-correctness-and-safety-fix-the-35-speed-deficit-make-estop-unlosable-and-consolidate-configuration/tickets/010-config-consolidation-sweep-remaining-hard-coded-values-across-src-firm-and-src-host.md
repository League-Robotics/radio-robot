---
id: '010'
title: 'Config consolidation: sweep remaining hard-coded values across src/firm and
  src/host'
status: open
use-cases: [SUC-008]
depends-on: ['009']
github-issue: ''
issue: 02-move-hard-coded-values-to-configuration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Config consolidation: sweep remaining hard-coded values across src/firm and src/host

## Description

Second of the two "mechanical sweep, run last" tickets; sequenced after
009 to keep the two configuration tickets from touching the same files
out of order. Issue 02 generalizes issue 03 (ticket 009) — deliberately
scoped as a separate, non-overlapping sweep of the *rest* of the tree,
not a second overlapping pass over `main.cpp`'s `PlannerLimits` block
(see `sprint.md` Design Rationale, Decision 4, for why these are two
tickets rather than one).

Sweep the tree for inline constants that are really configuration or
tuning values sitting mid-function instead of routed through the owning
config surface. Known starting points (captured during sprint 128, not
exhaustive):

- Host GUI panel parameters: spinbox decimals/thresholds in
  `testgui/__main__.py`, staleness thresholds such as `_STALE_AFTER_S` in
  `telemetry_panel.py`, retry counts/timeouts in halt paths.
- "All similar values": grep for numeric literals in logic across
  `src/firm` and `src/host` and classify each as (a) true config → move
  to config file/registry, (b) named constant → lift to a `k`-constant /
  module-level constant with a `[unit]` tag, or (c) genuinely local math
  — leave alone.

## Acceptance Criteria

- [ ] Each moved value has one declared home (config registry, robot
      JSON, or a named constant) and the code reads it from there.
- [ ] No behavior change: moved values keep their current defaults.
- [ ] The sweep records, per file touched, which values were moved vs.
      left, and why — a short table or note per file, not just a diff.
- [ ] Full clean build + test suite green (excluding the sprint's known
      baseline: 4 sim-tour turn-undershoot failures, 2 standalone-harness
      include-path failures — pre-existing, not this sprint's to fix).

## Testing

- **Existing tests to run**: `just build-clean`, `uv run python -m
  pytest`, `test_gui_button_acceptance.py` for any GUI-touching change.
- **New tests to write**: none required beyond confirming existing
  behavior is unchanged (this is a pure relocation sweep) — a test only
  needs to exist if a moved value previously had no coverage at all and
  the move is a natural point to add a minimal one.
- **Verification command**: `uv run pytest`.

## Implementation Plan

- **Approach**: grep-driven sweep, classify-then-move, one file at a
  time; do not touch `main.cpp`'s `PlannerLimits` region (that's ticket
  009's territory, already landed by the time this ticket starts).
- **Files to modify**: `src/host/robot_radio/testgui/__main__.py`,
  `src/host/robot_radio/testgui/telemetry_panel.py`, other `src/firm`/
  `src/host` files surfaced by the grep sweep.
- **Documentation updates**: the per-file "moved vs. left, and why"
  record required by Acceptance Criteria — keep it in the ticket's own
  completion notes or a short section of this file, since that's the
  sweep's own audit trail.
