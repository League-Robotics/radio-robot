---
id: '006'
title: 'Deliverable notebook: accel/decel and tour-closure charts'
status: done
use-cases:
- SUC-037
depends-on:
- '005'
github-issue: ''
issue: ''
completes_issue: true
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

- [x] The `dataviz` skill is loaded BEFORE any chart code is written (this
      is checked, not assumed — the implementer's own Completion Notes
      must state that the skill was loaded and name at least the specific
      palette/form choices it drove for this notebook).
- [x] The notebook loads ticket 005's captured CSV/JSON trace files under
      `tests/bench/out/tour_*` — not synthetic/hand-built data.
- [x] Chart 1 — commanded-vs-measured velocity for at least one straight
      leg, with visible accel/cruise/decel phases (the "nice acceleration
      and deceleration" evidence, literally).
- [x] Chart 2 — commanded-vs-measured (angular) velocity for at least one
      turn leg, with visible accel/cruise/decel phases.
- [x] Chart 3 — heading over time: one straight leg (should hold roughly
      flat) and one turn leg (should track the commanded ramp) shown
      together or clearly paired for comparison.
- [x] Chart 4 — the tour's own (x, y) path (dead-reckoned from the trace's
      `pose_x`/`pose_y`), with the start point and end point both marked
      and the closure gap (ticket 005's measured closure delta) visually
      called out — not just present in a table.
- [x] A per-leg summary table: leg index, kind (straight/turn), target,
      measured outcome, error, pass/fail against ticket 005's chosen
      tolerance.
- [x] Both Tour 1 and Tour 2's captured runs are represented somewhere in
      the notebook (not just one tour) — implementer's call whether as
      separate sections or a combined comparison.
- [x] The notebook runs top-to-bottom without error (`jupyter nbconvert
      --execute` or equivalent) and is committed WITH its rendered output
      (not just source cells) so it displays correctly on GitHub/without
      re-running.
- [x] Charts render legibly in both light and dark themes (per the
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

## Completion Notes

### dataviz skill (AC #1)

`Skill(skill="dataviz")` was loaded FIRST, before any chart code was
written. Specific choices it drove for this notebook:

- **Form**: every commanded-vs-measured chart uses the **emphasis** form
  (`choosing-a-form.md`: "one series is the point, rest are context") —
  the commanded/planned trajectory is context (muted secondary-ink
  `#52514e`, dashed), the measured/actual trajectory is the point (a
  single accent hue, categorical slot 1 blue `#2a78d6`, solid, 2.2px).
  This is used consistently across Charts 1-4 so the reader learns the
  convention once. The tour-path chart (Chart 5) is the one exception —
  there the *runs themselves* are the identity, so it uses the fixed
  categorical order (slots 1/2/3, blue/aqua/yellow) per run instead.
- **Palette**: `references/palette.md`'s reference instance verbatim — no
  eyeballed hex values. Chart chrome (surface `#fcfcfb`, hairline grid
  `#e1e0d9`, muted ticks `#898781`, ink `#0b0b0b`/`#52514e`) applied via a
  shared `style_axes()` helper so every chart is visually identical.
  Status colors (`#0ca30c` good / `#d03b3b` critical) reserved for the
  per-leg table's PASS/FAIL text only, paired with the text label itself
  (never color alone), per the status-color rule.
- **Marks**: 2px+ lines, round joins, hairline recessive
  gridlines, legend always present for the 2-series emphasis charts,
  small-multiples grid (Chart 3) uses one shared figure-level legend
  instead of repeating it per subplot.
- **Light/dark safety**: every figure is rendered with an **opaque**
  `facecolor="#fcfcfb"` (chart card), not a transparent background — so
  the embedded PNG stays legible whether the notebook is viewed on
  GitHub's light or dark theme (an opaque light card renders identically
  in both; a transparent-background dark-ink figure would vanish on a
  dark host page, which is exactly the anti-pattern this AC calls out).
- **Validator**: not re-run here — this notebook's charts are 2-series
  emphasis (1 accent + 1 muted-gray context) or, for Chart 5, a 2-3-run
  categorical set using the palette's own slots 1-3 directly from
  `palette.md`, not a novel palette requiring `validate_palette.js`.

### Data source (AC #2)

Loads every `tour_*.{csv,json}` pair from
`tests/bench/data/tour_traces/` (the curated, committed subset ticket 005
copied out of its own gitignored `tests/bench/out/` — see that
directory's own `README.md`). This ticket's own AC text names
`tests/bench/out/tour_*`, but that directory does not exist on a fresh
checkout (gitignored, session-local) — 005's Completion Notes and its own
`tour_traces/README.md` both explicitly document that ticket 006 must
read `tests/bench/data/tour_traces/` for exactly this reason. Confirmed:
no synthetic/hand-built rows anywhere in the notebook — every DataFrame
comes straight from `pd.read_csv()`/`json.loads()` against these files.

### Chart inventory (AC #3-8, both tours AC)

1. **Chart 1** — straight-leg (TOUR_2 leg 4, 850mm, run
   `20260715T202802Z`) commanded-vs-measured `v_x`: clean 0->200mm/s ramp,
   tight ~197-203mm/s cruise plateau, clean ramp-down.
2. **Chart 2** — turn-leg (same run, leg 5, target -217deg — the largest
   single turn in either tour) commanded-vs-measured `omega` (measured
   derived as `(vel_r - vel_l) / trackwidth`, trackwidth=128mm from
   `data/robots/tovez.json`): clean ramp, no oscillation, even on the
   hardest turn either tour attempts.
3. **Chart 3** (two figures) — population/small-multiples: every straight
   leg of TOUR_1's clean run `20260715T202538Z` (7 legs) and every turn
   leg of TOUR_2's clean run `20260715T202802Z` (7 legs, 90deg to -217deg,
   mixed direction) — same clean-ramp shape throughout, not an artifact of
   picking an easy leg. Both tours represented here on their own.
4. **Chart 4** — heading over time, paired: straight leg 4 (holds within
   ~+/-1.5deg of start) vs. turn leg 5 (tracks the commanded integrated
   ramp closely, converging by leg end).
5. **Chart 5** — tour path: 2 panels (TOUR_1, TOUR_2), every clean
   completion overlaid (recentred to each run's own start pose), start
   (circle) / end (square) markers, closure distance annotated for the
   worst-closure run per tour (TOUR_1 502.8mm, TOUR_2 715.6mm) with a
   dotted end-to-start leader line — the closure gap is visual, not just
   tabular.
6. **Per-leg summary tables** (styled pandas, PASS/FAIL in status color +
   text) for both hero runs (TOUR_1 `20260715T202538Z`, TOUR_2
   `20260715T202802Z`) — target, measured (displacement for distance legs,
   heading delta for turn legs), error, executor's own completion
   pass/fail. Surfaces the -217deg leg's ~-19.4deg landing error
   numerically, matching 005's own "-217deg leg marginal overshoot"
   finding.
7. **Per-run summary table** — all 9 captured runs (both tours: faults,
   overshoots, and clean completions), closure numbers, and pass/fail
   against 005's own chosen tolerances (TOUR_1 600mm / TOUR_2 800mm) —
   6/6 TOUR_1 and 2/2 TOUR_2 clean completions pass position closure;
   faulted/overshoot runs show `pass_position=None` (never reached a
   tour-level closure at all) rather than a false fail.
8. Closing markdown recaps 005's own honest interpretation verbatim in
   substance: turn-heading error compounds across a chained multi-leg
   tour (not a new defect), the reversal-adjacent `kFaultWedgeLatch`
   dwell-widening finding, and that heading-gain retuning is out of scope
   here (measure-and-report, not a fix).

### Execution (AC #9)

`uv run jupyter nbconvert --to notebook --execute --inplace
tests/notebooks/tour_closure_and_ramps.ipynb` — ran clean, ~4s wall time,
no `just`/CI recipe existed for `tests/notebooks/` prior to this (checked
first, per the ticket's own Testing Plan note). Verified programmatically
after execution: all 13 code cells have `outputs` populated (charts as
`display_data` PNG, tables as `execute_result` styled HTML, no `error`
output type on any cell) before committing. The committed `.ipynb` carries
its rendered output inline, so it displays correctly on GitHub without
re-running.

### Full suite

`uv run python -m pytest` — unchanged pass count from ticket 005's own
baseline (1090 passed, 15 skipped); this ticket adds no new
pytest-collected file (a notebook is not pytest-collected), matching its
own Testing Plan.

### Stakeholder WIP untouched

`source/app/robot_loop.{h,cpp}`, `host/robot_radio/io/cli.py`,
`host/robot_radio/io/repl.py` were dirty/untracked at ticket start
(concurrent stakeholder work) and were never read or modified by this
ticket — confirmed via `git status` before and after.
