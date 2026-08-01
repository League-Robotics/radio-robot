---
status: pending
---

# Sim tour turns undershoot 90° deterministically (−10.8° first turn, −20.8° after)

Found during sprint 128's final validation (2026-07-31). Four testgui
tests fail deterministically:

- `test_gui_button_acceptance.py::test_tour_1_runs_to_completion`
- `test_gui_button_acceptance.py::test_tour_2_runs_to_completion`
- `test_sim_transport_tour1.py::test_tour_1_runs_to_completion_with_finite_small_closure`
- `test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`

Signature (stable across runs, quiet or loaded machine):

```
turn 2:  commanded +90.00°  achieved +79.23°  error −10.765°  (tolerance 8.0°)
turn 4+: commanded +90.00°  achieved ~+69.2°  error ~−20.8°
```

## NOT a sprint-128 regression — verified

Reproduced byte-identically (turn 2 −10.765°) at pre-sprint baseline
commit `c7a955c2` in a pristine worktree with its own venv and a
freshly-built baseline `libfirmware_host.dylib`. Sprint 128's
EXECUTION.md baseline pytest run (6 failed / 1436 passed / **138
skipped**) most plausibly had these tests among its 138 skips (sim lib
not built at measurement time), which is why they are absent from its
failure list.

Also NOT the sprint-108 `kFaultWedgeLatch` tour-abort issue
(`sim-mode-tour-1-fault-baseline-exclusion-mismatch.md`, closed by
108-011) — tours complete here; the turns just undershoot.

## Reading of the signature

First turn short by ~10.8°, every later turn short by ~20.8° (≈ 2×) —
looks like a fixed per-turn angular deficit plus a carried-over offset,
i.e. something in the sim turn-shaping/settle path eats a constant wedge
of each commanded rotation. Wants a bisect over sim/planner history and
a look at the shaped-band turn profile vs. sim ground truth.

## Acceptance

- Root cause identified and fixed, or tolerances re-derived from a
  measured, explained sim behavior (not widened to paper over it).
- All four tests green; the fix or explanation recorded here.
