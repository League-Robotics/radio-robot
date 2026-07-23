---
status: pending
filed: 2026-07-23
filed_by: team-lead (stakeholder-specified patterns, this session)
related:
- land-at-zero-at-orthogonal-chain-boundaries.md
- chain-advance-reset-defeats-same-axis-compatible-leg-continuity.md
tickets: []
---

# TOUR_3 (20-gon "circle") and TOUR_4 (infinity symbol) — chained-turn and arc test patterns

## Purpose

Two new tours, each designed to stress exactly one thing the box tours cannot:

- **TOUR_3** — a 20-sided polygon driven as 20 chained (D, RT) pairs: 40
  orthogonal chain boundaries at a SMALL per-turn angle (18°). This is the
  density stress for `land-at-zero-at-orthogonal-chain-boundaries.md`: at
  ±0.3°/boundary the figure reads as a clean circle; per-boundary residue
  shows up unmistakably as scalloping, radius spiral, or net rotation error
  ×20. Visually: looks like a circle, is actually 20 straights.
- **TOUR_4** — an infinity symbol built from two straights through the center
  and two opposite-hand 240° arcs. First-ever coverage of the ARC primitive
  (a TWIST Move with BOTH v_x and omega nonzero), the reverse-curve
  transition (CW arc → straight → CCW arc), and crossing accuracy (both
  straights must intersect at the center — a sharp, visible closure signal).

## TOUR_3 — icosagon

Stakeholder spec: start exactly like TOUR_1 (robot facing +x / "right"),
drive the same first leg, turn left 90°, then drive the polygon.

    D 200 200 345          # same entry leg as TOUR_1
    RT 9000                # left 90 -> facing +y
    20 x [ D 200 200 <s> , RT 1800 ]   # exterior angle 360/20 = 18 deg, CCW

Geometry: a regular 20-gon with side `s` has circumradius `R = s / (2 sin(pi/20))
= 3.196 * s`. Pick `s` so the figure fits the playfield from the entry point
(s = 120-150 mm -> diameter 0.77-0.96 m; implementer fits it, CW vs CCW
likewise — whichever keeps the polygon on the board from the TOUR_1 entry
pose). The tour ends at polygon closure (start vertex, heading +90° again);
no return-home legs — closure is measured on the polygon itself.

Acceptance (sim ground truth, ideal chip, after the land-at-zero boundary fix):
- polygon start/end position delta below ~30 mm; net heading 360° within ~1°
  (20 x the per-boundary budget — this is the multiplier that makes small
  residue visible);
- per-vertex TRUE heading delta 18° ± 0.5°, per-side heading gain ≤ 0.3°
  (the same per-leg assertion the closure gate grows in the boundary issue);
- radius of the traced vertices constant within ~2% (no spiral).

## TOUR_4 — infinity symbol

Stakeholder spec, made concrete (all headings from the start pose, facing +x;
`L` = straight length, `alpha` = half-crossing angle = 30°):

    1. RT +3000                        # turn left 30
    2. D 200 200 <L>                   # straight out along +30
    3. ARC right lobe: CW, sweep 240 deg, radius r
    4. D 200 200 <2*L>                 # straight back THROUGH the center to the far side
    5. ARC left lobe: CCW, sweep 240 deg, radius r   # the reverse curve
    6. D 200 200 <L>                   # straight back to the center; net heading 0

Derived geometry (so nobody re-derives it): for the arcs to be tangent to
both crossing straights, the lobe center sits on the crossing bisector and

    r = L * tan(alpha)            # alpha=30: r = 0.577 * L
    sweep per lobe = 180 + 2*alpha = 240 deg
    arc path length = r * (4*pi/3) = 4.189 * r
    half-width of the whole figure = sqrt(3) * L   (total width 3.46 * L)

**Fit warning:** the stakeholder default L=500 mm gives a 1.73 m-wide figure —
almost certainly off the board. L = 250-300 mm (width 0.87-1.04 m) is the
realistic bracket; keep L a parameter and state the value used.

Wire realization of an arc leg: `move_twist(v_x = v, omega = ±v/r,
stop_angle = radians(240), timeout = ...)` — stop on ANGLE (the lobe is
defined by its sweep), timeout as the distance backstop.

### Known gap this tour will expose (deliberate)

`MoveQueue::shapeAndStage()` shapes ONLY the stop-kind axis; an arc's other
commanded axis passes through unshaped (documented in move_queue.cpp's own
shapeAndStage comment as a scope limitation). An Angle-stop arc therefore
tapers omega into the lobe exit but steps v_x; a Distance-stop arc does the
reverse. TOUR_4's acceptance should first RECORD what this does (curvature
error at lobe entry/exit), then the team decides whether coordinated
two-axis tapering is worth building or whether arc entry/exit error is
tolerable. Do not silently "fix" this with a special case inside the tour —
if two-axis shaping is needed, it is its own issue.

### Parser/vocabulary work (host)

`parse_tour()` (`src/host/robot_radio/planner/tour.py`) knows only `D` and
`RT`; TOUR_4 needs an arc verb — e.g. `"A <sweep_cdeg> <radius_mm> <v_mms>"` —
parsed to a `TourLeg(kind="arc", ...)` and mapped by `_move_kwargs_for_leg()`
onto the `move_twist(v_x, omega, stop_angle)` shape above. Signed sweep
carries the hand (CW negative), mirroring `RT`'s own sign convention.

Acceptance (sim ground truth, ideal chip):
- both crossings pass within ~25 mm of the center point;
- net heading after leg 6 = 0° ± 1°; end position within ~50 mm of start;
- lobe sweeps 240° ± 1° each, opposite signs;
- the arc-axis shaping gap is measured and reported (numbers, not adjectives),
  with a go/no-go recommendation on two-axis tapering.

## Sequencing

Build after `land-at-zero-at-orthogonal-chain-boundaries.md` lands — TOUR_3's
whole value is multiplying the per-boundary residue by 40, which is noise
until that fix is in. TOUR_4's parser work is independent and can proceed any
time; its acceptance runs after the boundary fix for the same reason.
