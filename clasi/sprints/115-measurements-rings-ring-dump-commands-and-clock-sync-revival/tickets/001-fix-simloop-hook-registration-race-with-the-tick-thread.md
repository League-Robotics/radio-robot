---
id: '001'
title: Fix SimLoop hook-registration race with the tick thread
status: open
use-cases:
- SUC-115-007
depends-on: []
github-issue: ''
issue: sim-loop-hook-registration-race-with-tick-thread.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix SimLoop hook-registration race with the tick thread

## Description

`src/host/robot_radio/io/sim_loop.py`'s `set_read_hook()`/`set_write_hook()`
(via the shared `_set_hook()`) call `sim_set_read_hook()`/
`sim_set_write_hook()` directly on the CALLING thread, unlike every other
`SimLoop` mutator (`twist()`, `stop()`, fault-condition setters,
`set_pid_enabled()`, ...), which all route through `_run_or_enqueue()`/
`_call_on_tick_thread()` so only the tick thread ever touches the raw
ctypes handle — per the module's own "Threading model" docstring. One
segfault was observed under heavy concurrent load
(`clasi/issues/sim-loop-hook-registration-race-with-tick-thread.md`) with a
backtrace landing inside the tick thread's own `sim_step()` call — a
`std::function` member (`SimPlant::readHook_`/`writeHook_`,
`src/sim/sim_ctypes.cpp`, no mutex/atomic) being reassigned from the
test's own thread while the tick thread is concurrently invoking it is a
textbook data race consistent with that crash. This sprint's own bench
gate (dump-and-reconstruct, sim first) leans on `SimLoop` stability, so
this fix belongs directly ahead of that work.

This sprint's arc issue (Related Issues) and the reorder-experiment issue
are unrelated to this ticket; do not conflate them.

## Implementation Plan

- **Approach**: route `set_read_hook()`/`set_write_hook()` (via
  `_set_hook()`) through `_call_on_tick_thread()`, exactly like every
  other `SimLoop` mutator already does — the issue's own suggested
  option (a), and the simplest fix that matches the module's stated
  invariant. Do not add a mutex to `SimPlant` (option (b)) — that
  duplicates a threading discipline the module already has a working
  pattern for.
- **Files to modify**: `src/host/robot_radio/io/sim_loop.py`
  (`_set_hook()`/`set_read_hook()`/`set_write_hook()`).
- **Repro attempt**: before touching the fix, write a tight loop that
  registers/unregisters a read or write hook repeatedly against a busy
  tick thread (e.g. `SimLoop` ticking while looping
  `with sim.read_hook(cb): pass` hundreds of times, ideally under
  induced concurrent load matching the original crash's conditions) and
  run it several times to see whether it reproduces the race at all.
  Record the outcome (reproduced or not) either way — a non-repro is not
  proof of safety, just a documented attempt. Re-run the same repro
  after the fix lands; it must not crash.
- **Files to create**: a new repro test/script under
  `src/tests/testgui/` (alongside `test_sim_loop.py`, the existing home
  of `test_read_hook_fires_and_pass_through_returns_bytes`/
  `test_write_hook_can_swallow_a_command`) — exact name/shape at
  implementer's discretion.

## Acceptance Criteria

- [ ] `set_read_hook()`/`set_write_hook()` route through
      `_call_on_tick_thread()` (or an equivalent synchronous-on-tick-thread
      dispatch), matching every other `SimLoop` mutator's documented
      threading discipline.
- [ ] A tight-loop register/unregister repro attempt against a busy tick
      thread is on record, run both before and after the fix — pass or
      fail, the outcome is documented in this ticket's completion notes.
- [ ] `src/tests/testgui/test_sim_loop.py`'s existing hook tests
      (`test_read_hook_fires_and_pass_through_returns_bytes`,
      `test_write_hook_can_swallow_a_command`) still pass.
- [ ] Full sim suite passes with no new flakiness introduced.

## Testing

- **Existing tests to run**: `src/tests/testgui/test_sim_loop.py`; full
  `uv run python -m pytest` sim/testgui suite.
- **New tests to write**: a tight-loop hook register/unregister repro
  test against a busy tick thread (see Implementation Plan).
- **Verification command**: `uv run pytest`
