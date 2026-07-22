---
id: '003'
title: App::Odometry pathLength accessor
status: in-progress
use-cases:
- SUC-051
depends-on: []
github-issue: ''
issue: protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# App::Odometry pathLength accessor

## Description

Additive-only extension of `App::Odometry` (`src/firm/app/odometry.{h,cpp}`):
`integrate()` already computes this cycle's `|distance|` internally
(`odometry.cpp:24-26`, currently discarded after feeding `x_`/`y_`) — this
ticket accumulates it into a new running total, exposed via a `pathLength()`
read-only accessor `Motion::StopCondition`'s DISTANCE kind (ticket 002)
baselines against. `integrate()`'s existing outputs (`x()`/`y()`/`theta()`)
are unchanged; no existing caller's behavior changes.

Independent of tickets 001/002/004 — no wire or queue dependency, purely an
`Odometry` internal addition. Only ticket 005 (`MoveQueue`) actually reads
`pathLength()` at runtime.

## Acceptance Criteria

- [ ] `pathLength()` accessor added, returning the cumulative `|distance|`
      accumulated across every `integrate()` call since construction (or
      since whatever `reset()` behavior this ticket decides — see below).
- [ ] `integrate()` accumulates `|distance|` (the value already computed
      internally) into the new running total on every call, unconditionally.
- [ ] `reset(x, y, theta)`'s interaction with `pathLength()` is decided and
      documented explicitly in the header comment: recommend `pathLength()`
      is NOT zeroed by `reset()` (it's a cumulative odometer-style value,
      and `StopCondition` baselines against a snapshot at MOVE activation
      regardless of when the odometer itself last reset, so zeroing on
      `reset()` would be a surprising, undocumented side effect for a
      caller that never asked for it) — pick one, write the rationale in
      the doc comment, and add a test asserting the chosen behavior.
- [ ] Existing `x()`/`y()`/`theta()` outputs and every existing test that
      exercises `integrate()`/`reset()` are unaffected.

## Testing

- **Existing tests to run**: `src/tests/sim/unit/test_app_odometry.py`
  (must stay green — no existing assertion should need to change).
- **New tests to write**: straight-line travel accumulates `pathLength()`
  ≈ true distance traveled; an in-place turn (zero net forward travel)
  contributes ≈0 to `pathLength()`; reverse travel still accumulates
  positively (uses `|distance|`, not net signed displacement, so forward
  then reverse over the same ground adds, it doesn't cancel); the chosen
  `reset()` interaction behavior.
- **Verification command**: `uv run python -m pytest
  src/tests/sim/unit/test_app_odometry.py`
