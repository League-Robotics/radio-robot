---
id: '006'
title: 'Bench acceptance: controller A/B, WHEELS-holds-speed-under-drag, +500 spec
  re-verification'
status: in-progress
use-cases:
- SUC-001
depends-on:
- '005'
github-issue: ''
issue:
- wheel-speed-controller-moves-into-drive.md
- 06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench acceptance: controller A/B, WHEELS-holds-speed-under-drag, +500 spec re-verification

## Description

On-stand acceptance per `wheel-speed-controller-moves-into-drive.md`
Phase 3's own acceptance criteria and
`06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md`'s agreed
+500-button acceptance spec. Bench A/B (old additive trim vs. new map
adaptation, same tour); WHEELS teleop demonstrably holds speed under
applied drag; right-wheel affine residual (ab303ee3's measured table)
closed across all four measured speeds; +500 button re-verified against
the full agreed spec.

Note the estop-risk context for this ticket (sprint Risks &
Dependencies): the write-on-change/latching-brick runaway defect
implied by this sprint's own briefing is already fixed and merged
(sprint 129 ticket 001); the only known residual is
`estop-settle-time-floor-is-the-loop-cycle-not-the-write-path.md`
(~0.19 s settle vs. a 0.15 s bound, priority medium, not in this
sprint's scope) — plan bench sessions around that bounded, already-
measured quantity, not an unbounded risk.

## Acceptance Criteria

- [ ] Bench A/B: closure and per-leg speed tracking at least as good as
      the old additive-trim baseline.
- [ ] WHEELS teleop under applied drag holds its commanded speed
      (measured, not asserted).
- [ ] The right-wheel affine residual closes across cmd 100/150/250/400
      mm/s (not just one speed).
- [ ] The +500 button acceptance spec re-verified end to end: rise
      <=0.3 s, plateau at 150 mm/s, ripple <=±10 mm/s, |vL-vR|<=10 mm/s,
      taper over the last 60 mm to the 90 mm/s floor, elapsed ~4 s,
      encoders 500±15 mm, heading <=3°, camera-measured travel
      500±25 mm.
- [ ] Results (data + chart) committed;
      `06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md`
      marked resolved with evidence.

## Testing

- **Existing tests to run**: n/a (bench/HITL verification ticket, not a
  unit-test ticket) — confirm no regression in the existing
  `planner_tests`/`app_drive` suites first.
- **New tests to write**: a committed bench acceptance script (or
  extension of an existing one) capturing the A/B comparison and the
  +500 spec's measured criteria.
- **Verification command**: on-stand bench run per
  `.claude/rules/hardware-bench-testing.md`; `uv run pytest` for any
  new script's own unit-testable pieces (e.g. its scoring logic).

## Implementation Plan

**Approach**: bench/HITL verification; no new production code expected
beyond fixing anything the A/B run reveals as a regression against
ticket 004/005's implementation.

**Files to create/modify**:
- `src/tests/bench/` (new or extended acceptance script — closest
  precedent: the existing `duty_sweep.py`/`velocity_step_response.py`
  bench-script style)
- `data/robots/tovez.json`'s `control._drive_calibration_note` history
  (dated entry recording the verdict)

**Testing plan**: on the stand, per `.claude/rules/hardware-bench-
testing.md`; commit the chart same session (project convention:
"ALWAYS send the chart").

**Documentation updates**: chart + writeup committed; calibration-note
history entry.
