---
id: '001'
title: 'Population duty-sweep protocol: baked defaults + adaptation bounds'
status: open
use-cases: [SUC-002]
depends-on: []
github-issue: ''
issue:
- wheel-speed-controller-moves-into-drive.md
- duty-sweep-single-wheel-vs-simultaneous-current-limit-129-006.md
- 06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Population duty-sweep protocol: baked defaults + adaptation bounds

## Description

Extend `src/tests/bench/duty_sweep.py` into the population-calibration
protocol `wheel-speed-controller-moves-into-drive.md` Phase 0 calls for,
explicitly constrained by `duty-sweep-single-wheel-vs-simultaneous-
current-limit-129-006.md`'s power-delivery-ceiling finding. Today's
sweep is single-robot, single-wheel-at-a-time, duty 0.04-0.60.

This ticket:

- Extends the sweep to the FULL duty range per motor per direction (not
  just 0.04-0.60), fitting the affine duty-speed line and capturing
  breakaway per motor. 129-006 found the response saturates and
  DECLINES above duty ~0.30 (left) / ~0.40 (right), well below the
  historical ceiling — the restricted-range fit alone is not enough.
- Adds a simultaneous-both-wheels sweep grid (both wheels driven
  together across a duty grid, not just one at a time) to characterize
  whether combined load underperforms single-wheel load — 129-006
  measured vL~104/vR~68 mm/s at a combined 150 mm/s command vs. ~129
  mm/s for the same left wheel driven alone.
- Re-runs against a freshly-charged/verified battery to separate a
  session-specific battery-sag artifact from a hard current-limit
  ceiling, and records an explicit verdict either way.
- Computes: population mean -> baked default map (per-wheel/direction
  gain+intercept, or one shared value if the parallel-lines test says
  variation is intercept-only); population spread (envelope or ±2σ) ->
  adaptation bounds (`biasMax`, `vMin`); breakaway population spread ->
  deadband/floor constants.
- Runs the parallel-lines test: if the population's duty-speed lines
  plot roughly parallel, intercept-only adaptation is confirmed
  (feeds ticket 004's Stage C design and Open Question 1); fanned lines
  would say slope-dominated instead.
- Commits the per-motor dataset (CSV) and chart, plus the derived
  values as generated constants the boot config bakes (coordinate with
  ticket 002's composition-root work and ticket 004's config schema
  changes).

Population testing requires physically swapping motors on the bench —
coordinate with the stakeholder per `.claude/rules/hardware-bench-
testing.md`. If a batch of spare motors is not available in one
session, ship the extended tool + protocol validated against the
currently-mounted robot's own two motors, and flag the full population
campaign as a stakeholder-scheduled follow-on bench session using this
same tool.

## Acceptance Criteria

- [ ] `duty_sweep.py` supports a population mode (multiple motor units,
      both directions, full duty range) and a simultaneous-both-wheels
      grid mode.
- [ ] Per-motor sweep dataset (CSV) and chart committed to the repo.
- [ ] Population mean map, spread-derived `biasMax`/`vMin`, and
      breakaway band recorded as generated values the boot config bakes.
- [ ] The parallel-lines test result stated explicitly (intercept-
      dominated vs. fanned/slope-dominated).
- [ ] An explicit, evidenced verdict on the 129-006 power-delivery
      ceiling (hard limit vs. session/battery artifact), re-run against
      a verified/freshly-charged battery.
- [ ] The L 853.6/R 837.8 mm/s-per-duty figures from
      `06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md` are
      re-verified (or explicitly superseded) against the FULL duty
      range, since 129-006 found saturation/decline above duty ~0.30-0.40
      that the original 0.04-0.60 fit did not capture.

## Testing

- **Existing tests to run**: any existing `duty_sweep.py` unit tests
  (fitting math); `uv run pytest src/tests/bench/` smoke coverage.
- **New tests to write**: unit tests for the fitting/parallel-lines-test
  math against synthetic multi-motor data (no hardware required); a
  dry-run smoke test of the new CLI modes against the one mounted
  robot's two motors.
- **Verification command**: `uv run pytest` plus an on-stand bench run
  per `.claude/rules/hardware-bench-testing.md`.

## Implementation Plan

**Approach**: extend the existing tool rather than write a new one
(closest precedent: `velocity_step_response.py`). Add CLI flags for
population/simultaneous modes; keep `--from-csv`/`--fit-duty-max`
refit-without-redriving support intact.

**Files to create/modify**:
- `src/tests/bench/duty_sweep.py` (population + simultaneous modes)
- `src/tests/bench/duty_sweep.csv` / `.png` (regenerated)
- `data/robots/tovez.json` / `togov.json` / `tovez_nocal.json`
  (population-mean defaults + adaptation-bound keys — coordinate with
  ticket 004's schema changes so this isn't done twice)
- `data/robots/tovez.json`'s `control._drive_calibration_note` history
  (existing convention) gets a new dated entry with the full writeup

**Testing plan**: synthetic-data unit tests for the fit/parallel-lines
math; a dry-run against the mounted robot as a smoke test of the new
modes; the full population campaign itself is a bench session, not a
CI-run test.

**Documentation updates**: sprint 130 cross-reference in the
calibration-note history; note the 129-006 verdict explicitly wherever
`biasMax`/`pidMax` bounds are later consumed (ticket 004).
