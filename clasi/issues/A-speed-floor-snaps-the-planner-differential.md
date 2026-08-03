---
status: pending
priority: high
---

# `applySpeedFloor()` snaps the planner's differential correction up to vMin — a 3 mm/s trim becomes a ~100 mm/s lurch

## Description

**This file was referenced by three documents and never existed.** The
sprint-130 post-mortem, the sprint-130 knowledge doc, and the
playfield-actuation-floor issue all cite
`sprint-130-regressions-speed-floor-snaps-differential-and-shaper-defaults.md`
as the tracking issue for the only regression pair sprint 130 introduced.
A repo- and history-wide search found nothing (2026-08-02 review, F12, which
calls creating it "the highest-value single file in this review"). Filed
here under a shorter name.

## What is wrong

`Drive::applySpeedFloor()` (`src/firm/app/drive.cpp:150-156`) boosts any
nonzero command below `vMin` up to `±vMin`:

```cpp
float Drive::applySpeedFloor(float commanded) const {
  if (commanded == 0.0f) return 0.0f;   // stop is stop, never boosted
  if (bounds_.vMin <= 0.0f) return commanded;
  const float magnitude = std::fabs(commanded);
  if (magnitude >= bounds_.vMin) return commanded;
  return std::copysign(bounds_.vMin, commanded);
}
```

It runs on each wheel's `cmdVelocity` (`drive.cpp:277-278`) — by which point
the planner's common-mode speed and its differential heading/trim correction
have already been **summed into one number**. The floor therefore cannot tell
a 3 mm/s differential trim from a 3 mm/s travel command, and quantizes the
correction to `vMin` (currently 99.7 mm/s, itself n=3 LOW CONFIDENCE). A
small correction is delivered as a lurch.

Observed consequence: four TestGUI tour tests FAULT on it.

## Why it is not just "lower vMin"

Two independent walls meet here:

1. **The planner has no concept of the floor at all.** `PlannerLimits`
   carries no vMin field, so the jerk-limited terminal taper sweeps 100 → 0
   continuously while the delivered behavior is "hold 99.7, step to 0"
   (2026-08-02 review, C1). Meanwhile `settle_epsilon_linear = 4.0 mm` still
   promises a 4 mm arrival tolerance against ~5.4 mm of travel per 54 ms tick
   at vMin, with `crawl_pulse = 0` shipped so there is nothing below
   breakaway.
2. **vMin itself is unmeasured under load.** The 99.7 figure comes from a
   stand sweep (wheels unloaded, n=3, cut short by the I2C wedge). A measured
   *loaded* floor is likely lower and would reduce the damage independently —
   see [[next-physical-bench-session-checklist]] item 4.

There is also a floor split: firmware `wheel_v_min = 99.7`
(`drive.cpp:150-156`) vs the host taper's `_UNMANAGED_FLOOR = 90.0`
(`testgui/transport.py:200-201`) — one physical quantity, two constants
(review F7).

## Proposed direction (not yet a decision)

Floor the **common-mode** speed and pass the **differential** through:
a correction that is small *because it is a correction* is not the same
thing as a travel command that is small. That requires the differential to
survive as its own quantity down to the floor, rather than being pre-summed
into `cmdVelocity`.

Pair with: the planner learning the floor (a `PlannerLimits` field fed from
the same single source), and the 4 mm settle epsilon waiting on the real
measured actuation floor.

## Verification

- A commanded differential trim of a few mm/s produces a proportional wheel
  response, not a step to vMin.
- The four FAULTing TestGUI tour tests pass.
- No regression in the square-tour closure gate at any tier.
- One owner for the floor constant; the host taper reads it rather than
  carrying its own 90.0.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` — F12 (this
  file's absence), C1 (the planner-side wall), F7 (the constant split).
- `docs/post-mortem/2026-08-02-sprint-130-wheels-solid.md`
- [[next-physical-bench-session-checklist]] — item 4 measures the real
  loaded floor.
