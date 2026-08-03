---
status: pending
priority: medium
---

# Heading hold is disabled, not fixed — it closes the loop on a delayed function of its own output

## Description

Post-mortem **recommendation #4**. Carried over from the closed
`bench-reverify-residuals-...` issue, whose other live half this is.

`Motion::Planner::applyHeadingHold()` is a P-loop at gain 2.0 rad/s per rad
closing on **pure-encoder heading** (the OTOS blend is fail-closed to 0.0
everywhere). Encoder heading is derived from the very wheel differential the
loop commands — so it is closing on a delayed function of its own output, and
it cannot distinguish a real heading error from encoder drift.

## Measured

130-011, bench, `tovez`:

- Full square tour: heading **411.8° vs 360 commanded** (+51.8), closure 80.6 mm
  — where sim showed 41.7/43.0 mm and clean turns.
- Isolated single-leg A/B on the same build: gain 2.0 gave a **sustained ~1 Hz
  wheel reversal**, 11.35 s for a leg that should take ~4 s. Gain 0.0 was clean.
- `App::Drive`'s raw `wheels()` path was unaffected either way — isolating it to
  the planner's profile path, not sprint 130's new controller.

**The gain is the symptom; the input is the problem.** It was stable at the old
44 ms delivered period and went unstable at 54 — see
[[A-nominal-50ms-vs-delivered-54ms]].

## Current state — a mitigation wearing the shape of a fix

`data/robots/tovez.json` sets `heading_hold_gain: 0.0`. That is a disable.

**`togov.json` and `tovez_nocal.json` still carry 2.0** and were never
independently bench-confirmed (no hardware for them that session) — they share
the code path and should be treated as equally suspect. `tovez_nocal.json` is
also the default `--robot-config` for `systest`, so the system test currently
runs with a suspect gain live.

## What to do

Either re-found the loop on an input it can trust, or retire it:

1. **Re-found**: close on a fused heading (OTOS + encoders) rather than encoders
   alone, so the loop has an independent reference. This depends on
   estimator-v2 OTOS fusion, and on `otos.cpp:137-143`'s velocity registers
   being decoded with the right LSB constants — currently they use *position*
   LSBs, so `otos.v_*` on the wire are systematically mis-scaled (post-130
   review, Part 1 MINOR). Fix that first or the fused heading inherits it.
2. **Or re-gain it honestly** at the delivered period with a stated stability
   margin, and say in the config note that it is a band-aid on an encoder-only
   input.
3. **Or delete it** and let the planner's own per-tick re-plan-from-measured
   handle heading, which is what it does today with the gain at zero — and
   which passes the sim square at 41.7 mm.

Option 3 deserves genuine consideration: nothing currently depends on heading
hold, and the tree is better with one fewer correction layer (see the knowledge
doc's "adding a fudge constant is how this got hard").

Whatever lands, reconcile all three robot JSONs — one behavior, not three.

## Verification

- No sustained limit cycle on a straight `move_twist` leg at the delivered
  period, per the 130-011 single-leg A/B protocol.
- Bench square tour heading within band (the sim reference is 41.7/43.0 mm
  closure).
- All three robot JSONs carry the same, justified value with a note that says
  what it was validated against and at what period.
- If kept: a sim test that can actually destabilize it, which today requires
  the sim to step at the delivered period.

## Related

- `docs/post-mortem/2026-08-02-sprint-130-wheels-solid.md` recommendation 4.
- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` Part 6
  ("heading hold off until re-founded on a fused heading").
- [[B-rotation-calibration-vs-live-heading-hold-gain]] — the other half of
  `tovez_nocal.json`'s turn-correction problem.
