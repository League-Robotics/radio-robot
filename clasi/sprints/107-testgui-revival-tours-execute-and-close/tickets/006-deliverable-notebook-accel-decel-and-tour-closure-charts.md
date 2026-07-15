---
id: "006"
title: "Deliverable notebook: accel/decel and tour-closure charts"
status: open
use-cases: [SUC-037]
depends-on: ["005"]
github-issue: ""
issue: ""
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Deliverable notebook: accel/decel and tour-closure charts

## Description

This is the stakeholder's own literal, verbatim, stated deliverable for
the ENTIRE single-loop rebuild arc (2026-07-14): "I want to see charts in
Jupyter Notebooks that show nice acceleration and deceleration on
straights and turns." A new notebook under `tests/notebooks/` (alongside
the established `motion_control.ipynb`/`wheel_motion_trace.ipynb`
precedent) loads ticket 005's captured bench trace files and renders the
accel/decel envelope evidence, heading tracking, and tour closure. Serves
SUC-037.

**CHART QUALITY IS THE ACCEPTANCE BAR HERE, NOT MERELY "A CHART EXISTS."**
Before writing any chart code, the implementer MUST load the project's
`dataviz` skill (`Skill(skill="dataviz")`) and apply its guidance — form
heuristics, the color formula, light/dark consistency, mark specs — to
every chart in this notebook. This is a non-negotiable, explicit
instruction from this ticket's own acceptance criteria, not a suggestion.

## Acceptance Criteria

- [ ] The `dataviz` skill is loaded BEFORE any chart code is written (this
      is checked, not assumed — the implementer's own Completion Notes
      must state that the skill was loaded and name at least the specific
      palette/form choices it drove for this notebook).
- [ ] The notebook loads ticket 005's captured CSV/JSON trace files under
      `tests/bench/out/tour_*` — not synthetic/hand-built data.
- [ ] Chart 1 — commanded-vs-measured velocity for at least one straight
      leg, with visible accel/cruise/decel phases (the "nice acceleration
      and deceleration" evidence, literally).
- [ ] Chart 2 — commanded-vs-measured (angular) velocity for at least one
      turn leg, with visible accel/cruise/decel phases.
- [ ] Chart 3 — heading over time: one straight leg (should hold roughly
      flat) and one turn leg (should track the commanded ramp) shown
      together or clearly paired for comparison.
- [ ] Chart 4 — the tour's own (x, y) path (dead-reckoned from the trace's
      `pose_x`/`pose_y`), with the start point and end point both marked
      and the closure gap (ticket 005's measured closure delta) visually
      called out — not just present in a table.
- [ ] A per-leg summary table: leg index, kind (straight/turn), target,
      measured outcome, error, pass/fail against ticket 005's chosen
      tolerance.
- [ ] Both Tour 1 and Tour 2's captured runs are represented somewhere in
      the notebook (not just one tour) — implementer's call whether as
      separate sections or a combined comparison.
- [ ] The notebook runs top-to-bottom without error (`jupyter nbconvert
      --execute` or equivalent) and is committed WITH its rendered output
      (not just source cells) so it displays correctly on GitHub/without
      re-running.
- [ ] Charts render legibly in both light and dark themes (per the
      `dataviz` skill's own standing requirement) if the notebook's
      rendering path supports theme awareness; at minimum, the chosen
      palette must not rely on a single-theme-only assumption (e.g. pure
      black text on transparent background).

## Implementation Plan

### Approach

1. `Skill(skill="dataviz")` FIRST — read its form heuristics, palette
   formula (`references/palette.md`), and mark-spec guidance before writing
   any plotting code.
2. Load ticket 005's trace CSVs (pandas), one straight leg and one turn
   leg selected as the primary "clean ramp" exhibits (chosen for clarity —
   the FIRST clean, non-outlier run per ticket 005's own Completion Notes,
   not necessarily the first chronological run if an early one had a known
   issue).
3. Build charts per the acceptance criteria using whatever plotting library
   the existing `tests/notebooks/` precedent already uses (check
   `motion_control.ipynb`/`wheel_motion_trace.ipynb` for the established
   convention — likely matplotlib or plotly; match it rather than
   introducing a third library for consistency across the notebook
   family), applying the `dataviz` skill's palette/form guidance
   throughout.
4. Build the closure/path chart from the tour-level trace's `pose_x`/
   `pose_y` columns, annotating start (0,0-relative) and end points and the
   measured closure distance.
5. Build the per-leg summary table (a pandas DataFrame rendered inline, or
   a styled table per the `dataviz` skill's own table guidance if it has
   one).
6. Execute the notebook end-to-end and commit with output cells populated.

### Files to Create

- `tests/notebooks/tour_closure_and_ramps.ipynb` (exact filename,
  implementer's call, following the existing `tests/notebooks/` naming
  convention).

### Testing Plan

- `jupyter nbconvert --to notebook --execute` (or the project's own
  established notebook-verification command, if `tests/notebooks/` has
  one already — check for a `just` recipe or CI step covering the existing
  notebooks before inventing a new verification path) confirms the
  notebook runs without error.
- Visual review against the `dataviz` skill's own checklist before
  considering this ticket done.

### Documentation Updates

- None beyond the notebook's own markdown cells (which should briefly
  explain what each chart shows and cite ticket 005's trace files as the
  data source, per the existing notebook family's own documentation
  convention).
