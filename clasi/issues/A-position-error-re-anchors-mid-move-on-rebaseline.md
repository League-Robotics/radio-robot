---
status: pending
priority: high
---

# `positionError()` re-anchors to zero mid-move on a pose rebaseline

## Description

`src/tests/sim/system/test_rebaseline_pose.py::test_rebaseline_pose_sanity`
passes at the sprint-133 merge-base and **fails with the `pid_ki 6.0` that
133-004 measured and shipped**. It was left failing, documented, rather than
silenced — the failure is real and the test is right.

`positionError()` re-anchors its reference to zero whenever `positionEpoch`
changes. A software pose rebaseline bumps that epoch mid-move, so the
accumulated position correction is discarded at the rebaseline instant and
the controller resumes from zero error while the wheel is still mid-travel.

Inert before sprint 133 because every shipped robot carried `pid_i_max: 0`,
which disabled the I term entirely. 133-004's tuning made the term live, and
the defect surfaced immediately.

## Why this is a design question, not a tuning knob

`pid_ki 3.0` makes the test pass — and **breaks the imbalance criterion**
(trapezoid L/R falls to 0.9525, against the ~0.99 that 133-004 achieved and
promoted). So the two requirements are in direct tension at the level of a
single gain, which means the gain is the wrong place to resolve it.

The real question is what a position-domain integrator *should* do when the
position reference it accumulates against is redefined underneath it:

- **Re-anchor and forget** (today): correct on a genuine origin change,
  wrong mid-move — it deletes real, still-outstanding travel debt.
- **Re-anchor and carry the outstanding error across the epoch change**:
  preserves the debt, but requires the epoch change to be expressible as a
  known offset, which a rebaseline may not provide.
- **Refuse to re-anchor while a move is active**, deferring to the next
  commanded-zero: simplest, and consistent with the sigma-delta carry, which
  is already discarded only at commanded zero (133-002 §7-C).

The third option is the most consistent with the mechanism 133-002 built and
is where I would start.

## Verification

- `test_rebaseline_pose_sanity` passes with the **shipped** tuning
  (`pid_ki 6.0`, `pid_i_max 60.0`, `pos_err_max 10.0`), not with a reduced
  gain chosen to make it pass.
- The 133-004 imbalance result holds: L/R within ~1% on both the square and
  trapezoid profiles, re-measured on `tovez`.
- A test that a rebaseline arriving mid-move does not discard outstanding
  position error.

## Related

- Sprint 133 ticket 002 — built the position-domain I term and removed the
  encoder-freshness gate deliberately.
- Sprint 133 ticket 004 — measured and promoted the tuning that exposed this,
  and reported it rather than adjusting the gain to hide it.
- [[B-wheel-controller-position-loop-and-tuning]] — the parent work.
