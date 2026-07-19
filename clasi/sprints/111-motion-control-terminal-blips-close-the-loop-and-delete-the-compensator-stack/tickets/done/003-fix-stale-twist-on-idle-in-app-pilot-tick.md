---
id: '003'
title: Fix stale-twist-on-idle in App::Pilot::tick()
status: done
use-cases:
- SUC-002
depends-on:
- '001'
- '002'
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

- [x] `Pilot::tick()` captures `Motion::State stateBefore =
      executor_.state();` before calling `executor_.tick(...)`.
- [x] After computing `twist` and the heading-PD `omega` (unchanged
      logic), the twist-staging decision becomes: if
      `executor_.state() != Motion::State::kIdle`, stage
      `drive_.setTwist(twist.v, omega)` (unchanged, existing behavior);
      else if `stateBefore != Motion::State::kIdle` (a natural completion
      happened inside THIS tick() call), stage `drive_.setTwist(0.0f,
      0.0f)` exactly once; else (already idle before AND after — includes
      the flush-caused-idle case) do nothing, matching today's existing
      "does nothing while kIdle" behavior.
- [x] A same-cycle flush-then-raw-TWIST is unaffected: existing TWIST/
      STOP preemption behavior (`handleTwist()`/`handleStop()`) is not
      regressed — verify via the existing move-queue/twist sim tests
      (`test_move_queue.py`'s own Scenario 4, "TWIST still preempts...
      STOP still stops the robot immediately").
- [x] **Reconciled, not literally satisfied — see Completion Notes.**
      Ticket 001's "no nonzero command survives past the terminal zero"
      assertion did NOT flip from `xfail` to passing, because it was
      never `xfail`: both `straight_no_command_after_terminal_zero` and
      `pivot_no_command_after_terminal_zero` were already plain, passing
      assertions before this fix (confirmed by re-running the harness
      against the pre-fix `pilot.cpp`). That check reads the DECODED,
      MEASURED wheel-velocity trace, and the ideal sim's own terminal
      decel already drives that trace under the 15mm/s near-zero bar by
      completion, so it cannot see the shelf either way. This ticket adds
      two NEW checks (`straight_shelf_collapsed`/`pivot_shelf_collapsed`,
      `measureShelfCycles()` in `behavior_lock_harness.cpp`) that measure
      the COMMANDED PID target directly (not the measured trace) and
      demonstrate the fix concretely: shelf length goes from 5 cycles
      (pre-fix, rebuilt and measured against the unfixed `pilot.cpp`) to
      0 cycles (post-fix) for both scenarios. No other harness assertion
      flipped — the hump/tail-shape checks stay `xfail`, unchanged.
- [x] `robot_loop.cpp` is not modified.
- [x] `uv run pytest` is green: `uv run python -m pytest` → **1224
      passed, 18 xfailed, 2 xpassed, 0 failed** (baseline 1222/18/2/0 from
      ticket 002 + this ticket's own 2 new genuinely-passing shelf
      checks; zero new failures, zero new xfails).

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

## Completion Notes

**The fix** (`src/firm/app/pilot.cpp`, `Pilot::tick()`): captures
`Motion::State stateBefore = executor_.state();` immediately before the
existing `executor_.tick(...)` call. The twist-staging decision at the
end of the method becomes a 3-way branch: `executor_.state() != kIdle` ->
stage `drive_.setTwist(twist.v, omega)` (unchanged); else if
`stateBefore != kIdle` (a natural running->idle completion happened
INSIDE this call) -> stage `drive_.setTwist(0.0f, 0.0f)` exactly once;
else (already idle before AND after, which includes the same-cycle-flush
case) -> do nothing, matching the pre-existing "does nothing while
kIdle" contract. Exactly the surgical change the ticket specified — no
other logic in `tick()` touched, `robot_loop.cpp`/`Motion::Executor`/the
min-speed floor untouched.

**AC #4 reconciliation (read this before trusting a bare "xfail->pass"
claim).** The ticket's literal acceptance criterion -- ticket 001's
`*_no_command_after_terminal_zero` checks flip from `xfail` to passing
-- does not apply: both checks were confirmed ALREADY plain, non-xfailed
passes before this fix landed (re-compiled and ran
`behavior_lock_harness.cpp` against the unmodified, pre-fix `pilot.cpp`
to confirm directly, not just trusting ticket 001's own completion
notes, which already flagged this). The reason: that check evaluates the
DECODED, MEASURED wheel-velocity trace (`Telemetry::Frame.velLeft/
velRight`), and the ideal sim's own terminal decel already drives that
trace under the 15mm/s near-zero bar by the time a command completes
(both scenarios settle to <5mm/s within one cycle of the DONE ack) --
holding an already-near-zero MEASURED value stale for the ~300ms
deadman-lease window never crosses the bar again, so this check cannot
distinguish fixed from unfixed timing in the ideal sim, regardless of
which pilot.cpp is running.

**What actually demonstrates the fix**: two new harness checks,
`straight_shelf_collapsed`/`pivot_shelf_collapsed`
(`behavior_lock_harness.cpp`'s `measureShelfCycles()`/
`runShelfScenario()`), which measure the COMMANDED PID target instead of
the measured trace -- `SimHarness::driveTargetVelLeft()`/
`driveTargetVelRight()` (new test-only accessors added to
`src/sim/sim_harness.h`, forwarding to the pre-existing
`Devices::Motor::velocityTarget()`, the value `App::Drive::tick()` last
wrote via `setVelocity()`). This has no near-zero headroom to hide
behind: it only reads EXACTLY `0.0f` once something explicitly stages a
zero twist. Measured directly, by recompiling and running the SAME
harness binary against BOTH versions of `pilot.cpp`:

| Scenario | Pre-fix shelf (cycles) | Post-fix shelf (cycles) |
|---|---|---|
| D700 straight | 5 | 0 |
| 360deg pivot | 5 | 0 |

("Shelf" = cycles from the command's own `ACK_STATUS_DONE` to the first
cycle the commanded target reads exactly 0.) 5 cycles at the sim
harness's 50ms/cycle rate is ~250ms, consistent with the ~300ms
`kPilotDeadmanLease` deadman-force-stop mechanism the driving issue
documents; 0 cycles post-fix confirms `Pilot::tick()` now zeroes `Drive`
on the SAME cycle the command completes, exactly as designed. Both new
checks are genuine (non-xfailed) assertions (`shelf <= 2` cycles),
locking this in as a real regression fence going forward. No other named
check in the harness flipped -- `pivot_ramp_bounds`/
`straight_ramp_bounds`/`straight_terminal_bounds`/
`pivot_single_lobe_left`/`pivot_single_lobe_right`/
`pivot_lobes_opposite_sign` remain `xfail`, unchanged, exactly as ticket
001 left them (confirmed by diffing the full `RESULT:` line set
before/after this fix -- identical except the two new shelf checks).

**No-regression**: `test_move_queue.py` (Scenario 4, "TWIST preempts the
Move queue; STOP still stops immediately") and `test_sim_api.py`'s
twist/stop/deadman scenarios both pass cleanly with the fix in place --
the same-cycle-flush branch (`stateBefore` already `kIdle` when sampled,
because `RobotLoop::handleTwist()`/`handleStop()` call `Pilot::flush()`
before `Pilot::tick()` runs) is confirmed to fall through to "do
nothing," leaving the raw command's own `Drive::setTwist()` call
untouched.

**Files changed**: `src/firm/app/pilot.cpp` (the fix),
`src/firm/app/pilot.h` (doc comment extended, not replaced, per the
ticket's own instruction), `src/firm/app/DESIGN.md` (`Pilot` §2 subsection
gained the completion-zeroing sentence), `src/sim/sim_harness.h` (two new
test-only read accessors, `driveTargetVelLeft()`/`driveTargetVelRight()`),
`src/tests/sim/system/behavior_lock_harness.cpp` (`measureShelfCycles()`/
`runShelfScenario()` + two new scenario calls in `main()`),
`src/tests/sim/system/test_behavior_lock.py` (two new test functions,
`test_straight_shelf_collapsed`/`test_pivot_shelf_collapsed`, plus
docstring updates on the two `*_no_command_after_terminal_zero` tests and
the module header documenting the AC #4 reconciliation honestly).

**Full-suite verification**: `uv run python -m pytest` (module-invocation
form, canonical per `.clasi/knowledge/pytest-env-uv-run-gotcha.md`) ->
**1224 passed, 18 xfailed, 2 xpassed, 0 failed** (322s). Accounting:
ticket 002's own established baseline was 1222 passed/18 xfailed/2
xpassed/0 failed; this ticket adds exactly 2 new genuinely-passing tests
(`test_straight_shelf_collapsed`/`test_pivot_shelf_collapsed`) and zero
new xfails/failures -- 1222+2=1224 passed, 18 xfailed unchanged, 2
xpassed unchanged (pre-existing, untouched by this ticket, per ticket
002's own completion notes). Exit code 0. Ran twice independently
(module-invocation form) with identical results.
