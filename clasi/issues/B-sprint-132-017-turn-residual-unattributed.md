---
status: pending
priority: medium
---

# The 132-017 turn residual is reproducible but unattributed

## Description

Sprint 133 ticket 005 root-caused and fixed the **dominant** step of the
sprint-132 turn-accuracy regression — the sim inheriting `tovez`'s baked
wheel correction, worth about −30°, now fixed via `App::BootOverrides`.

A **second, smaller step remains**, introduced by `63f3039d` (132-017, the
JSON reshape). It is reproducible but not attributed to any field.

Post-fix, `TOUR_1/ideal`'s middle turns sit near **−20°** where pre-132 they
clustered near **−12°**. Worst |error| is 22.10° against pre-132's 24.70°, so
the *headline* number is restored — but the distribution shape is not
identical, and the difference is real rather than noise.

## What has been ruled out, with evidence

Do not re-litigate these; each was tested by rebuild-and-measure, not by
reading:

- **Shaper-ceiling resourcing** (`control.*` → `planner_shaper.*`)
- **Baked rotation calibration**
- **Baked `rotational_slip`**
- **Baked velocity-PID gains**

Also ruled out earlier, by 132-018:

- **Config-value drift** — a 64-field mechanical diff of the reshaped JSON
  against its pre-sprint values came back clean.
- **The `PLANNER_SHAPER` wire push** — output byte-identical with it disabled.
  **Note this one is weaker evidence than it appears**: 133-005 established
  that the push had already become inert, so "disabling it changes nothing"
  was a true observation about a no-op, not evidence about the mechanism.

Ticket 005 stopped at its attempt cap rather than guessing, which was the
right call and is why the remaining candidates are still clean.

## Method that works

133-005's approach is the one to reuse: build a **second git worktree** at the
pre-132 merge-base and run the identical deterministic test in both, comparing
per-turn distributions rather than pass/fail. `63f3039d` is the first runnable
commit after the schema cutover — commits `bb305ff5`…`4125c0b6` cannot run the
test at all mid-sprint, so the range is not finely bisectable and the residual
must be attributed by field, not by commit.

## Verification

- `TOUR_1/ideal`'s per-turn distribution returns to its pre-132 shape
  (middle turns clustering near −12°, not −20°).
- The attribution names a specific field or mechanism, reproduced by changing
  only that value.

## Related

- Sprint 133 ticket 005 — fixed the dominant step; carries the three-way
  distribution tables and the refuted hypotheses.
- [[B-rotation-calibration-vs-live-heading-hold-gain]] — adjacent, may share a
  mechanism.
- Note the shaped-band test has a **pre-existing** failure history and is not
  the target: restoring the distribution is, not making it green.
