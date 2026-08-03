---
status: in-progress
priority: high
sprint: '131'
tickets:
- 131-001
---

# Ownership handover is implemented as `estop()`: every accepted MOVE wipes the wheel controller's learned state

## Description

Found independently by two 2026-08-02 review passes — the sprint-130 midpoint
review (finding #1, MAJOR) and the post-130 review (F5, MAJOR). Verified in the
tree at close.

`src/firm/app/robot_loop.cpp:227` calls `drive_.estop()` on **every**
non-duplicate accepted MOVE:

```cpp
drive_.estop();  // the planner takes over motion (one owner at a time)
const bool accepted = planner_.move(m, move.replace);
```

Since 130-004, `src/firm/app/drive.cpp:43-50` makes `estop()` also zero **both
PID integrators, both Stage C biases, and the deficit latches**.

## Why this defeats the thing sprint 130 was built for

Stage C adapts on a `tauAdapt = 30 s` timescale. Bench-measured convergence
(130-006, `bias_convergence_150.csv`): biasRight climbs 0 → +14.0 mm/s over a
continuous 90 s hold, reaching within ~5 mm/s of setpoint at t ≈ 15–20 s.

- **Between legs**, every Move cold-starts adaptation, so Stage C can never
  converge on the planner path — the navigation surface it exists to serve.
- **Mid-flight**, a chained tour that tops up the queue resets a converged
  30-second-timescale bias in one tick — a non-bumpless ~1–2% duty step.
- Measured consequence: fresh-start delivery is **70–85% of commanded**
  (130-006's short-hold sweep), and it is silent, because the deficit flag that
  would say so cannot fire (see [[B-observability-contract-is-inert-as-shipped]]).

The teleop path (`handleWheels`, `robot_loop.cpp:275`) does **not** do this — it
keeps Drive's bias. The asymmetry is undocumented and backwards: teleop is the
path that could tolerate a reset, the planner is the one that cannot.

## Provenance — why nobody caught it

The takeover `estop()` predates the controller state. It was added at `bd7f75b8`
(2026-07-27) when `estop()` meant only "zero the targets"; 130-004 later added
the learned-state reset to `estop()` without revisiting the takeover call site.
Neither change was wrong on its own.

## What to do

Split the two verbs, which are two different intents wearing one name:

```cpp
// Drive::takeover() -- another subsystem is assuming motion ownership.
// Zero targets, disarm WHEELS. KEEP learned state (bias, integrators):
// the plant did not change, only the writer did.
void takeover();

// Drive::estop() -- safety. Full reset including learned state.
// Reserved for the ESTOP verb and real panic paths.
void estop();
```

Point `robot_loop.cpp:227` at `takeover()`; leave the ESTOP verb on `estop()`.

**Sequence with the sign-aware bias fix** (post-130 review, C3,
`drive.cpp:122`: `copysign(magnitude, desired) + bias`). A forward-learned +14
bias *reduces* reverse magnitude and perturbs pivots until re-learned at
tau = 30 s. That defect is latent **only** because this issue resets the bias so
often; fixing this one alone makes it real. Fix them together — bias per signed
direction, or decay on sign flip.

## Verification

- A chained tour holds a converged bias across leg boundaries; bias does not
  return to 0 at each enqueue (readable in the bench capture).
- The ESTOP verb still performs the full reset — re-run
  `src/tests/bench/estop_unlosable_bench.py`, 10/10 with no relapses.
- A reversal after a converged forward hold does not overshoot: the bias applies
  with the correct sign or has decayed.
- Firmware test covering both verbs' distinct post-conditions.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F5, C3.
- `docs/code_review/2026-08-02-sprint-130-midpoint.md` finding 1 — recommended
  fixing this *before* ticket 012's playfield gate, which did not happen
  (012 was deferred).
