---
id: '003'
title: Fix stale-twist-on-idle in App::Pilot::tick()
status: open
use-cases: [SUC-002]
depends-on: ['001', '002']
github-issue: ''
issue: motion-control-terminal-blips-reconciled-fix-plan.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix stale-twist-on-idle in App::Pilot::tick()

## Description

Step 1 of the driving issue. `App::Pilot::tick()`
(`src/firm/app/pilot.cpp`) only stages `Drive::setTwist()` while
`executor_.state() != Motion::State::kIdle`. On the running→idle
transition — a command completing naturally INSIDE a single `tick()`
call — the current code takes neither branch: `Drive` is left holding
whatever twist was staged the PREVIOUS cycle, forever, until the 300ms
deadman lease (`kPilotDeadmanLease`, `robot_loop.cpp`) expires and
force-stops it. This is the post-completion "shelf." Confirmed present in
the current code (read directly during sprint planning):

```cpp
if (executor_.state() != Motion::State::kIdle) {
    drive_.setTwist(twist.v, omega);
}
```

Fix: capture `executor_.state()` BEFORE calling `executor_.tick()`; if
that captured state was non-idle and the state AFTER `tick()` is
`kIdle`, stage `drive_.setTwist(0, 0)` exactly once. Do NOT zero on a
flush-caused idle transition (a same-cycle raw `TWIST`): `RobotLoop::
handleTwist()`/`handleStop()` call `Pilot::flush()` BEFORE
`Pilot::tick()` runs in the SAME cycle (`processMessage()` runs earlier
in `RobotLoop::cycle()`'s motorR-settle block than `pilot_.tick()` — see
`robot_loop.cpp`), so on a flush cycle `stateBefore` is ALREADY `kIdle`
by the time `tick()` samples it — the "just completed" branch is
naturally never taken, and the raw command's own twist (already staged
by `handleTwist()`) survives untouched. This distinction is why a
before/after snapshot is required rather than a simpler "always zero
once per idle entry" flag.

**Scope discipline**: this ticket touches `pilot.cpp`'s `tick()` method
ONLY. It does not touch `robot_loop.cpp` (cycle order/request-collect
sequencing stays exactly as-is, per this sprint's guardrail), does not
touch `Motion::Executor`, and does not touch the min-speed floor or any
other patch-stack mechanism in `pilot.cpp` (those are sprint 2's scope).

## Acceptance Criteria

- [ ] `Pilot::tick()` captures `Motion::State stateBefore =
      executor_.state();` before calling `executor_.tick(...)`.
- [ ] After computing `twist` and the heading-PD `omega` (unchanged
      logic), the twist-staging decision becomes: if
      `executor_.state() != Motion::State::kIdle`, stage
      `drive_.setTwist(twist.v, omega)` (unchanged, existing behavior);
      else if `stateBefore != Motion::State::kIdle` (a natural completion
      happened inside THIS tick() call), stage `drive_.setTwist(0.0f,
      0.0f)` exactly once; else (already idle before AND after — includes
      the flush-caused-idle case) do nothing, matching today's existing
      "does nothing while kIdle" behavior.
- [ ] A same-cycle flush-then-raw-TWIST is unaffected: existing TWIST/
      STOP preemption behavior (`handleTwist()`/`handleStop()`) is not
      regressed — verify via the existing move-queue/twist sim tests
      (`test_move_queue.py`'s own Scenario 4, "TWIST still preempts...
      STOP still stops the robot immediately").
- [ ] Ticket 001's behavior-lock harness "no nonzero command survives
      past the terminal zero" assertion flips from `xfail` to passing.
      No OTHER assertion in that harness is expected to flip (the
      hump/tail shape assertions are unaffected by this fix — they stay
      `xfail`, unchanged, until sprint 2).
- [ ] `robot_loop.cpp` is not modified.
- [ ] `uv run pytest` is green.

## Implementation Plan

**Approach**: a small, surgical change to `Pilot::tick()`'s twist-staging
logic. No new members, no new parameters — `Motion::State` is already
readable via `executor_.state()` at both points in the method.

**Files to modify**:
- `src/firm/app/pilot.cpp` — `Pilot::tick()`.
- `src/firm/app/pilot.h` — if the doc comment above `tick()` needs a
  one-line update describing the new zero-on-completion behavior (the
  existing comment already documents the "does nothing while kIdle"
  contract; extend it rather than replace it).

**Testing plan**: run ticket 001's harness first in isolation to confirm
the specific assertion flips; then run the existing move/twist/stop sim
tests (`test_move_queue.py`, `test_sim_api.py`'s twist/stop scenarios) to
confirm no regression; then the full suite.

**Documentation updates**: `src/firm/app/DESIGN.md`'s `Pilot` subsection
(§2) currently doesn't describe this specific completion-zeroing
behavior — add one sentence noting that `Pilot::tick()` zeroes `Drive`
exactly once on a natural running→idle transition, distinguishing it
from the flush-caused case, so the doc stays accurate to the fixed
behavior.

## Testing

- **Existing tests to run**: `src/tests/sim/system/test_move_queue.py`,
  `src/tests/sim/system/test_sim_api.py` (twist/stop scenarios), the full
  `uv run pytest` suite.
- **New tests to write**: none new — this ticket is verified by ticket
  001's harness (already written) flipping one assertion from xfail to
  passing.
- **Verification command**: `uv run python -m pytest
  src/tests/sim/system/test_behavior_lock.py -v -s` (confirm the specific
  flip), then `uv run pytest`.
