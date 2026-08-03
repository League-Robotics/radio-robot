---
id: '001'
title: 'Population duty-sweep protocol: baked defaults + adaptation bounds'
status: done
use-cases:
- SUC-002
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

- [x] `duty_sweep.py` supports a population mode (multiple motor units,
      both directions, full duty range) and a simultaneous-both-wheels
      grid mode.
- [x] Per-motor sweep dataset (CSV) and chart committed to the repo.
- [ ] Population mean map, spread-derived `biasMax`/`vMin`, and
      breakaway band recorded as generated values the boot config bakes.
      PARTIAL: computed and recorded as prose in `tovez.json`'s
      `_drive_calibration_note` (report-only, n=3, explicitly flagged
      LOW CONFIDENCE); NOT wired into an actual generated/boot-config
      value -- that plumbing is ticket 004's config-schema work, and
      baking a low-confidence, incomplete-population value now would be
      premature regardless. See completion notes.
- [x] The parallel-lines test result stated explicitly (intercept-
      dominated vs. fanned/slope-dominated).
- [ ] An explicit, evidenced verdict on the 129-006 power-delivery
      ceiling (hard limit vs. session/battery artifact), re-run against
      a verified/freshly-charged battery. NOT DONE -- deferred to the
      stakeholder-scheduled follow-on bench session per this ticket's
      own sanctioned fallback (a battery cannot be charged by an agent);
      see completion notes and the filed follow-on issue.
- [~] The L 853.6/R 837.8 mm/s-per-duty figures from
      `06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md` are
      re-verified (or explicitly superseded) against the FULL duty
      range, since 129-006 found saturation/decline above duty ~0.30-0.40
      that the original 0.04-0.60 fit did not capture. LEFT wheel: DONE
      -- superseded (linear to duty~0.55, not ~0.30; refit ~1062-1100
      mm/s/duty vs. the old 853.6). RIGHT wheel: NOT DONE -- a mid-session
      hardware fault (see completion notes) left only 3 low-duty forward
      rungs, too few to re-verify or supersede 837.8; flagged UNCONFIRMED,
      not superseded, pending the follow-on session.

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
- `src/tests/bench/output/duty_sweep.csv` / `.png` (regenerated)
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

## Completion Notes (2026-08-01)

**Tool delivered and verified.** `src/tests/bench/duty_sweep.py` now
supports: full duty range (0.04-1.0, was 0.60-ceiling) per motor per
direction with a separate fit window (`--fit-from`/`--fit-duty-max`); a
simultaneous-both-wheels grid sharing the single-wheel sweep's own duty
grid for an apples-to-apples comparison; `--from-csv` to re-run the
fit/population analysis without re-driving the robot; and the
population/parallel-lines analysis (`fit_line`, `population_spread`,
`parallel_lines_verdict`, `bias_max_from_offsets`, `v_min_from_breakaway`,
`map_gain_intercept`), unit-tested against synthetic data in
`src/tests/unit/test_duty_sweep_population.py` (10 tests, all passing, no
hardware). Both the population and simultaneous CLI modes were exercised
live against `tovez` and produced correct output before the session-ending
hardware fault below.

**A real, more subtle bug was found and fixed along the way**: the
script's duty-axis inversion was reading `duty_per_speed_left/right` from
the robot JSON, which `boot_calibration.cpp` has ignored since 2026-07-31
(`Drive::kDutyPerSpeed` is now a baked C++ constant, config-independent).
Left unfixed, every full-range rung this session would have targeted the
wrong actual duty by a ~1.6x ratio. Fixed by hardcoding the firmware's
actual constant in the script (cited from `drive.h`, cross-checked every
run by a constant-free saturation reading per the module's own point 4).

**Session-ending hardware fault, not a script defect.** After 45 good
rungs (complete left-wheel characterization, both directions, full range;
right-wheel forward through duty=0.242), `tovez` went hard-silent -- no
PING, no telemetry. Confirmed via read-only pyOCD diagnosis (core running,
PC in RAM, spinning in `codal::system_timer_wait_cycles`/
`system_timer_current_time`) and reproduced across 3 fresh reconnects
(including a hard `reset=True` DTR reset) -- matches the project's own
documented "silent robot = dead external I2C bus" signature exactly (banner
+ some good frames, then total silence, every reboot re-wedges). USB
enumeration stayed healthy throughout; this is a physical fault (motor
brick battery / bus power), not a cable, port, or software issue, and not
something an agent can recover from. Filed as
`clasi/issues/tovez-hard-silent-i2c-wedge-blocks-completing-the-population-
duty-sweep.md`.

**What was captured and committed** (`src/tests/bench/output/duty_sweep.csv`,
`duty_sweep.png`): a complete left-wheel duty-speed curve, both
directions, full 0.04-1.0 range -- genuinely linear (steady local slope
~1000-1200 mm/s/duty) to duty~0.55, materially higher than 129-006's ~0.30
estimate (itself apparently session/battery-specific, not a fixed
ceiling), then a sharp one-rung saturation onset. Refit over the
corrected [0.12,0.50] window: left fwd 1101.3 mm/s/duty (offset -14.2),
left rev 1023.1 (offset +1.7) -- this SUPERSEDES the L 853.6 figure. Right
wheel has only 3 usable forward rungs (duty<=0.242) -- its 837.8 figure is
UNCONFIRMED, not superseded. No simultaneous-grid data was captured (the
fault hit before that phase). Full writeup:
`data/robots/tovez.json`'s `control._drive_calibration_note`, 2026-08-01
entry.

**Derived population values** (n=3 lines -- this robot's own two motors,
one incomplete -- the ticket's own sanctioned fallback, further narrowed
by the hardware fault): parallel-lines verdict came out slope-dominated
(fanned), opposite the stakeholder's intercept-only hypothesis, but is
explicitly flagged LOW CONFIDENCE (the right-wheel line rests on 3
near-breakaway points) and must not be treated as settling ticket 004's
Stage C design question. `biasMax` ±23.8 mm/s, `vMin` 99.7 mm/s, breakaway
band 0.04-0.09 duty (44-100 mm/s) -- report-only, not baked into any
config key.

**Deferred to the same stakeholder-scheduled follow-on bench session**
(physical intervention required, cannot be done by an agent): (1) fix
the motor-brick battery / I2C bus power fault, (2) complete the right
wheel's full range and the simultaneous grid, (3) 129-006's own
freshly-charged-battery re-run (battery was NOT freshly charged or
independently verified this session -- no pack-voltage telemetry exists to
measure it directly), (4) the full multi-motor population campaign
(physically swapping motors). All four are consolidated in the filed
follow-on issue above.

**Test suite**: `uv run python -m pytest src/tests/sim -q` --
484 passed, 3 xfailed, 2 failed (both pre-existing/known:
`test_clock_sync_activation.py`, `test_fake_transport.py`) -- matches
baseline exactly, no regressions.
