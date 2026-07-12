---
id: '011'
title: 'Bench: arc_sweep grids, plateau to v_wheel_max, envelope/gain tuning, 098 pivot grid re-run'
status: open
use-cases: [SUC-013]
depends-on: ['008', '009', '010']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench: arc_sweep grids, plateau to v_wheel_max, envelope/gain tuning, 098 pivot grid re-run

## Preconditions

Robot USB-attached (flash the cutover+MOVER+trace builds from tickets
007-010) or reachable via the radio relay for capture (per
`turn_sweep.py`'s dual-transport pattern). Robot mounted on the stand,
wheels off the ground, per `.claude/rules/hardware-bench-testing.md`.

## Description

`tests/bench/arc_sweep.py` (the `turn_sweep.py`-pattern dual-transport
capture script) runs arc/pivot/chain grids on the real robot,
re-measures the wheel-speed plateau into `tovez.json`'s (new)
`PlannerConfig.v_wheel_max`, and re-runs the 098 pivot acceptance grid to
confirm the new stack matches or exceeds the old one's ±1° result.

## Acceptance Criteria

- [ ] `tests/bench/arc_sweep.py` exists, follows `turn_sweep.py`'s
      dual-transport (serial + relay) capture pattern, writes
      `MotionTrace`-derived CSVs into `tests/notebooks/out/`.
- [ ] Arc/pivot/chain grid results land within the issue's terminal
      tolerance (`|e_along| <= 10-15mm`) on every cell tested.
- [ ] The measured wheel-speed plateau is re-measured and pinned into
      `data/robots/tovez.json` as the NEW `PlannerConfig.v_wheel_max`
      (NOT `DrivetrainConfig.v_wheel_max` — per `architecture-update.md`
      Decision 2, these are deliberately separate fields), with a dated
      note of the measurement session, mirroring `tovez.json`'s existing
      `_vel_gains_note` precedent.
- [ ] The 098 pivot acceptance grid (angle x speed grid, both
      directions) is re-run against the new stack via `tests/bench/
      turn_sweep.py --relay --both` (or `arc_sweep.py`'s pivot-only
      mode, if consolidated); result lands `>=100%` within `+/-1deg`
      (matching or improving on 098's own bench-verified result) OR the
      delta is explicitly analyzed and reported, not silently regressed.
- [ ] MOVER deadman teleop is exercised on the stand as part of this
      session (co-locating with the arc/pivot grid work, per sprint.md's
      HITL front-loading note).
- [ ] OTOS coexistence soak: a sustained session with OTOS ticking
      (sprint 099) alongside motion commands shows no bus-hang
      regression.
- [ ] Envelope/gain tuning: if bench evidence shows the tier-0-shipped
      starting gains/envelopes need adjustment, the adjusted values are
      pinned into `tovez.json` with a dated note explaining what changed
      and why (mirroring `_heading_gains_note`) — not silently left at
      tier-0 defaults if bench evidence says otherwise.
- [ ] Results (CSVs, plots, the pivot-grid comparison) committed under
      `tests/notebooks/out/` and referenced in completion notes.

## Testing

- **Existing tests to run**: `tests/bench/turn_sweep.py --relay --both`
  (the 098 grid, for direct before/after comparison).
- **New tests to write**: `tests/bench/arc_sweep.py`.
- **Verification command**: `uv run pytest` (host-side regression check
  only — this ticket's real verification is the bench session itself).

## Implementation Plan

**Approach**: this ticket does NOT modify `source/drive/` or the
adapter — a bench-discovered control-law defect reopens ticket 004/005/
007 instead of being patched here. Gain/envelope RETUNING (adjusting a
JSON value, not code) IS in scope, matching sprint 098's own Decision
2 precedent (ship conservative starting values, iterate against the real
plant during the hardware acceptance ticket).

**Files to create**: `tests/bench/arc_sweep.py`.

**Files to modify**: `data/robots/tovez.json` (plateau + any retuned
gains).

**Testing plan**: the bench session itself; re-run of the existing
`turn_sweep.py` 098 grid for direct comparison.

**Documentation updates**: `tovez.json`'s own inline notes (per the
existing convention) document what was measured/changed and when.
