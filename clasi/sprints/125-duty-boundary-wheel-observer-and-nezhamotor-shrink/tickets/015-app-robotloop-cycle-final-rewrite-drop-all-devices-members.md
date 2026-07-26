---
id: '015'
title: 'App::RobotLoop::cycle() final rewrite: drop all Devices:: members'
status: open
use-cases:
- SUC-005
- SUC-006
depends-on:
- '013'
- '014'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# App::RobotLoop::cycle() final rewrite: drop all Devices:: members

## Description

**Valve-line tail, the closing piece** (Design Rationale Decision 6). The
one place the tree's ownership reshuffle becomes complete:
`App::RobotLoop::cycle()` restructured so `RobotLoop` holds ONLY
subsystems (`Drive&`, `Sensors&`, `Comms&`, `Telemetry&`) and motion
objects (`MoveQueue&`, `Odometry&`, `StateEstimator&`) — zero
`Devices::*` members. Statement order follows the base-explicit-loop-
sketch's normative sense→observe→decide→act→report VALUE ordering
(temporal adjacency to the bus schedule is explicitly NOT required to
match — the interleaved request/settle/collect timing stays real, per
the sketch's own caveat). See sprint architecture Step 3 (`App::RobotLoop`)
and Use Cases SUC-005/SUC-006.

## Acceptance Criteria

- [ ] `App::RobotLoop`'s header declares zero `Devices::*` members
      (grep-enforceable).
- [ ] `cycle()`'s own top-to-bottom statement order matches sense/
      observe/decide/act/report (documented, reviewable — call out any
      borrowed-window reordering explicitly in the code comment, matching
      today's file's own convention).
- [ ] **[off-hardware]** A sim re-run of the existing pairing-skew/
      straight-leg-crab regression suite passes UNCHANGED post-rewrite.
- [ ] **[stand-required, USB]** The stand bench gate (ticket 012, if not
      already closed, or a follow-up confirmation run) exercises a full
      move under the FINAL rewritten loop with no regression against
      those same invariants, live.

## Testing

- **Existing tests to run**: full sim/unit suite, pairing-skew/straight-
  leg-crab regression suite, `app_robot_loop_harness.cpp`.
- **New tests to write**: none expected beyond confirming existing
  coverage survives — this ticket's own risk is regression, not missing
  coverage.
- **Verification command**: `uv run pytest`; stand bench gate re-run if
  ticket 012 closed before this ticket landed.
