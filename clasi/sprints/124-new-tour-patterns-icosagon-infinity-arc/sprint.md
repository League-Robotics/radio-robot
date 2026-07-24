---
id: '124'
title: 'New tour patterns: icosagon + infinity/arc'
status: roadmap
branch: sprint/124-new-tour-patterns-icosagon-infinity-arc
worktree: false
use-cases: []
issues:
- tour-3-icosagon-and-tour-4-infinity-test-patterns.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 124: New tour patterns — icosagon + infinity/arc

> Re-planned check 2026-07-23 (`clasi/issues/replan-sprints-122-plus-to-close-goal-exact-tours.md`):
> KEPT as detailed, with the stakeholder amendments of 2026-07-23 IN FORCE
> (already reflected in the spine issue, restated here so detail planning
> cannot miss them): (1) board-fit resize — TOUR_3 s=120 mm (R=384), TOUR_4
> L=250 mm (r=144.3, lobe arc 604 mm); (2) NO new arc verb — an arc is a
> regular `move_twist(v_x, omega, stop_distance)`; the only host change is
> `parse_tour()` no longer averaging a left!=right `D` step into a straight.
> Stage bar: goal doc S1 evidence (multiplier patterns). Sequenced after 122's
> analytic completion lands.
>
> Roadmap-level plan (Phase 1). Architecture, use cases, and tickets are
> filled in at detail-planning time, after sprint 121 lands.

## Goals

Add two host-side tours, each designed to stress exactly one thing the box
tours (TOUR_1/TOUR_2) cannot:

- **TOUR_3 (20-gon "circle").** 20 chained (D, RT) pairs = 40 orthogonal chain
  boundaries at a SMALL per-turn angle (18 deg). The density stress for 121's
  land-at-zero: at +-0.3 deg/boundary the figure reads as a clean circle;
  per-boundary residue shows up unmistakably (scalloping, radius spiral, or net
  rotation error x20). Looks like a circle, is actually 20 straights.
- **TOUR_4 (infinity symbol).** Two straights through the center + two
  opposite-hand 240 deg arcs. First-ever coverage of the ARC primitive (a TWIST
  `Move` with BOTH v_x and omega nonzero), the reverse-curve transition (CW arc
  -> straight -> CCW arc), and crossing accuracy at the center.

## Problem / What's new

- No tour today multiplies per-boundary residue enough to see it, and none
  exercises arcs at all. Both are host-only additions — the wire needs nothing
  new (an arc is a regular `move_twist(v_x, omega, stop_distance)`).
- The tour string parser (`parse_tour()`, `src/host/robot_radio/planner/tour.py`)
  currently AVERAGES a `D <left> <right> <mm>` step's two wheel speeds into one
  straight-leg speed, silently discarding a left!=right arc. The minimal fix is
  to the PARSER, not the vocabulary: when left != right, emit an arc leg
  (`v_x = (l+r)/2`, `omega = (r-l)/trackwidth`, distance stop = the step's
  `<mm>` path length), mapped onto the existing `move_twist(...)`.

## Solution (candidate — confirm at detail time)

- **TOUR_3**: `D 200 200 345` entry leg (like TOUR_1), `RT 9000` to face +y,
  then 20 x `[ D 200 200 <s>, RT 1800 ]` (exterior angle 18 deg). Sized to the
  board (stakeholder-authorized): `s = 120 mm` -> R = 384 mm, diameter 767 mm
  (fits the ~1010 x 890 mm playfield). CW vs CCW is the implementer's call;
  ends at polygon closure, no return-home legs.
- **TOUR_4**: sized to the board (stakeholder-authorized) at `L = 250 mm`
  (`alpha = 30 deg`, `r = L*tan(alpha) = 144.3 mm`, lobe sweep 240 deg, lobe arc
  length 604 mm, through-center leg 500 mm). Arc legs realized as regular
  `move_twist` (forward velocity + turn velocity + distance stop). Spelled via
  the fixed `parse_tour()` (e.g. a lobe is `D 111 289 604` and its mirror
  `D 289 111 604`). Geometry is fully derived in the issue — do not re-derive.

## Known gap this will EXPOSE (deliberate, do not silently patch)

`MoveQueue::shapeAndStage()` shapes only the stop-kind axis; an arc's other
commanded axis passes through unshaped, so a Distance-stop arc tapers v_x while
omega holds cruise — curvature TIGHTENS as v drops (kappa = omega/v). TOUR_4's
acceptance must RECORD what this does (curvature/heading error at lobe exit) as
numbers, then give a go/no-go recommendation on whether coordinated two-axis
tapering is worth building. Do NOT special-case it inside the tour; if two-axis
shaping is needed it is its own future issue.

## Success Criteria

- TOUR_3 (sim ground truth, ideal chip, after 121): polygon start/end position
  delta below ~30 mm; net heading 360 deg within ~1 deg; per-vertex TRUE
  heading delta 18 deg +- 0.5 deg; per-side heading gain <= 0.3 deg; traced
  vertex radius constant within ~2% (no spiral).
- TOUR_4 (sim ground truth, ideal chip): both crossings within ~25 mm of
  center; net heading after leg 6 = 0 deg +- 1 deg; end within ~50 mm of start;
  lobe sweeps 240 deg +- 1 deg each, opposite signs; arc-axis shaping gap
  measured and reported with a go/no-go on two-axis tapering.

## Scope

### In Scope

- `TOUR_3`/`TOUR_4` geometry + `parse_tour()` left!=right arc-leg support
  (`src/host/robot_radio/planner/tour.py`), `_move_kwargs_for_leg()` arc
  mapping, TestGUI wiring, and closure-gate coverage.
- Host only; Sim ground-truth acceptance.

### Out of Scope

- Any wire-protocol or firmware change (arcs already expressible via
  `move_twist`).
- Building coordinated two-axis arc shaping (TOUR_4 only MEASURES the gap).

## Dependencies / Sequencing

- **Depends on 121** — TOUR_3's whole value is multiplying per-boundary residue
  by 40, which is noise until land-at-zero lands. Ideally 123 too (heading-hold
  further squares the straights), but not required.
- TOUR_4's `parse_tour()` work is independent and can proceed any time; its
  ACCEPTANCE runs after 121 for the same reason.
- Independent of 122/125/126/127.

## Architecture

Deferred to detail planning. Expected tier: compact — host-side additions to
one module (`planner/tour.py`) plus test/GUI wiring; no new cross-module
dependency, no firmware, no wire change.

## Use Cases

Deferred to detail planning.

## Tickets

Deferred to detail planning.
