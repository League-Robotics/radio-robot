---
status: done
priority: high
sprint: '133'
tickets:
- 133-005
---

# Sprint 132 introduced a sim turn-accuracy regression, cause unknown

## Description

`src/tests/testgui/test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
was already failing before sprint 132, so the pass/fail tally did not move. **But
its error distribution changed materially**, and the change is real.

| | pre-sprint | post-132 |
|---|---|---|
| turn 2 | −2.6° | **−8.6°** |
| turns 6 / 8 / 10 | ~−12° | **~−21°** |

Early turns got better, later turns got substantially worse. Commanded 90°;
turns 10 and 12 now land at 69.37°.

## Cause

**Not root-caused.** Ticket 132-018 established that it is a genuine regression
rather than variance, and ruled out two hypotheses — but did not find the cause.

Method (worth repeating for whoever picks this up): a **second git worktree was
built at the pre-sprint merge-base commit** and the identical deterministic test
run there against HEAD. The distribution difference is reproducible, not noise.

Ruled out with evidence:
- **Config-value drift** — a 64-field mechanical diff of the reshaped robot JSON
  against its pre-sprint values came back clean.
- **The new `PLANNER_SHAPER` wire push** — output is byte-identical with it
  disabled.

Still open as candidates: the `duty_per_speed` sourcing change (132-009 reversed
the hardcoded `Drive::kDutyPerSpeed` in favour of the robot JSON — behaviour-
preserving for `tovez` by construction, but the *path* changed), the
Planner/PlannerShaper group split (132-017), and the planner-limits reshape.

This is **sim-side** (testgui runs against `SimLoop`), so it does not implicate
the hardware bench result — 132-019 measured straight-line distance fidelity on
`tovez` and it passed. But it is a motion regression the sprint introduced.

## Proposed fix

1. Bisect across sprint 132's own commits using the worktree method 018 used —
   the commit range is small and each commit is coherent.
2. The most suspicious candidates are 132-009 (`duty_per_speed` sourcing) and
   132-017 (group split + planner reshape), because both touch values that feed
   turn geometry.
3. Note that this test has a **pre-existing** failure history, so "make it pass"
   is not the goal — restoring the pre-sprint error distribution is.

## Verification

- The error distribution across turns 1-12 should return to its pre-sprint
  shape (roughly uniform ~−13°), not necessarily to passing.
- Compare against a worktree at the pre-sprint merge-base, the same way 018 did.

## Related

- [[B-rotation-calibration-vs-live-heading-hold-gain]] — adjacent, and may share
  a mechanism.
- Sprint 132 ticket 018's completion notes carry the worktree A/B evidence.
- [[the-configuration-object]] — the design whose implementation introduced this.
