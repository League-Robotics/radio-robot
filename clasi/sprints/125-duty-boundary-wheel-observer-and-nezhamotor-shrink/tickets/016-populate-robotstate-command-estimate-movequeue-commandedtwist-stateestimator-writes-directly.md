---
id: '016'
title: 'Populate RobotState::Command/Estimate: MoveQueue::commandedTwist(), StateEstimator
  writes directly'
status: open
use-cases:
- SUC-007
depends-on:
- '006'
- '015'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Populate RobotState::Command/Estimate: MoveQueue::commandedTwist(), StateEstimator writes directly

## Description

**Valve-line tail** (Design Rationale Decision 6), the second carried-
over 124 gap. `RobotLoop` wires `MoveQueue::commandedTwist()` (ticket 006
added the method itself) into `state.command.v_x`/`omega` each cycle.
`Motion::StateEstimator::update()` is changed to WRITE directly into
`state.estimate.wheelLeft/wheelRight/body/innovations` (the caller's
`RobotState&`) instead of private members; `wheelAt()`/`bodyAt()`/
`whereAmI()`/`wheelNow()` become FREE FUNCTIONS over `RobotState::
Estimate` data — a caller holding a COPIED `RobotState` extrapolates
with no live `StateEstimator` instance in scope. `StateEstimator` itself
retains only `weights_` (config). See sprint architecture Step 3
(`Motion::StateEstimator`) and Design Rationale Decision 3.

## Acceptance Criteria

- [ ] **[off-hardware]** A sim test drives one `TWIST` and one `WHEELS`
      `Move` in turn and asserts `state.command.v_x`/`omega` is nonzero
      and correct in both cases (wired end-to-end through `RobotLoop`,
      not just unit-tested on `MoveQueue` alone).
- [ ] **[off-hardware]** A test constructs a `RobotState`, calls
      `StateEstimator::update()` once, COPIES the state, and calls the
      free-function `wheelAt()`/`bodyAt()` against the COPY with NO live
      `StateEstimator` in scope — proving the query genuinely dissolved
      into data, not merely relocated.
- [ ] `Motion::StateEstimator`'s only remaining private member besides
      transient computation locals is `weights_`.

## Testing

- **Existing tests to run**: `state_estimator_harness.cpp` (expect
  significant rework — the public query surface changes shape).
- **New tests to write**: the copied-`RobotState`-no-live-instance test
  above (this is the load-bearing proof of Decision 3, not optional); the
  end-to-end `commandedTwist()` wiring test.
- **Verification command**: `uv run pytest` and `cmake --build
  src/motion/build --target motion_tests`.
