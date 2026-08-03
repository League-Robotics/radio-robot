---
status: pending
priority: high
---

# The control period is 54 ms and the config says 50; every gain is tuned against the wrong number

## Description

Post-mortem **recommendation #2**; post-130 review **F7 (MAJOR)**. Carried over
from the closed `bench-reverify-residuals-...` issue, whose live half this is.

Measured 130-011: **54.000 ms ± 0.006 idle**, 54.09 ± 0.38 under load, against
`kCycle = 50`. `cycleBusy` is only 21–23 ms, so this is **not** budget overrun.

**It is not new.** With `kCycle = 40` the delivered period was 44 ms (visible in
TestGUI as `loop 21.3ms / 44.0ms`). Same +4 ms, both times. Sprint 130 moved the
nominal and inherited the offset.

## Where the 4 ms comes from

Structural, not incidental: the loop's four `runAndWait()` gaps sum to `kCycle`,
but real work runs *between* the marks (`robot_loop.cpp:512-534`), plus each
block's `fiber_sleep` rounds up to a whole ms. `robot_loop.h:49-57` asserts
"exactly 50 ms".

## Why it matters more than 4 ms sounds

- **The planner and Drive disagree inside one control path.** Drive integrates
  with measured dt (`drive.cpp:298`); the planner's profile math and `cmdAccel`
  use the baked 50 (`planner.cpp:1347`). That is a systematic **8% disagreement**
  between two halves of the same loop.
- **8% was enough to flip a loop unstable.** 130-011 bench-reproduced
  `applyHeadingHold()` at gain 2.0 as a sustained ~1 Hz wheel-reversal limit
  cycle — 11.35 s for a leg that should take ~4 s — at the delivered 54 ms, on
  a gain that was fine at 44. See
  [[B-heading-hold-closes-on-encoder-only-heading]].
- **The sim cannot represent the difference at all**: one constexpr serves both
  believed and delivered period, so the sim steps at exactly 50 and no sim test
  can ever catch this class. That is the over-unification flaw in an otherwise
  excellent 130-002 composition root — see
  [[B-sim-plant-is-idealized-and-biases-belief-not-motion]].
- Every gain in the robot JSON was tuned against a number that has never been
  true.

## What to do

Pick one, and make the whole tree read it:

1. **Make the loop deliver the nominal** — the final pacing block becomes an
   absolute end-of-cycle deadline (`sleepUntil(cycleStartMark, kCycle)`) rather
   than a relative sleep, so jitter is absorbed instead of accumulated. This is
   the honest fix; the constant then means what it says.
2. **Or make the constant mean the delivered value** and re-derive everything
   from it.

Then feed planner, Drive, and the sim from that one number, and delete the
stale "47 ms" claims left over from the previous generation of this same
confusion (`main.cpp:80`, `boot_wiring.h:36`, `robot_loop.h:52`).

Do not fix the note and leave the number — the note is the *symptom*; a config
that asserts something measurably false is how the heading-hold gain got
shipped.

## Verification

- Measured delivered period equals nominal within the measurement floor, or the
  constant is renamed and every consumer re-derived — stated explicitly, either
  way.
- The planner's profile math and Drive integrate against the same period.
- The sim can be stepped at the measured period (see the sim issue) and a
  heading-hold-style marginal loop can be destabilized there.
- Timing notes in `data/robots/*.json`, `robot_loop.h`, `main.cpp`,
  `boot_wiring.h` all state the same, true number.

## Related

- `docs/post-mortem/2026-08-02-sprint-130-wheels-solid.md` recommendation 2,
  root cause 1 ("the period *was* 50 ms because a constant said so").
- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F7.
- `docs/code_review/2026-08-02-sprint-130-midpoint.md` finding 3 — the 40→50
  cutover also missed three bench scripts; that is
  [[B-bench-scripts-hardcode-the-retired-40ms-cycle]].
