---
id: '001'
title: 'Rider: fix SimLoop hook-registration race'
status: done
use-cases:
- SUC-045
depends-on: []
github-issue: ''
issue: sim-loop-hook-registration-race-with-tick-thread.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Rider: fix SimLoop hook-registration race

## Description

`src/host/robot_radio/io/sim_loop.py`'s `_set_hook()` (shared by
`set_read_hook()`/`set_write_hook()`) calls `sim_set_read_hook()`/
`sim_set_write_hook()` directly on the calling thread — unlike every
other `SimLoop` mutator (`twist()`, `stop()`, fault-condition setters,
`set_pid_enabled()`), which route through `_run_or_enqueue()`/
`_call_on_tick_thread()` so only the tick thread ever touches the raw
ctypes handle (the module's own documented threading invariant). One
segfault was observed during 111-002's heavy-load baseline triage,
backtrace landing in `_tick_loop`'s `sim_step()` call — consistent with
a data race against `SimPlant::readHook_`/`writeHook_` (plain
`std::function` members, no mutex, in `sim_ctypes.cpp`'s
`sim_set_read_hook()`/`sim_set_write_hook()` around line 333) being
reassigned from the test's own thread while the tick thread is mid-step
invoking the same hook. This is an independent, pre-existing reliability
issue (sprint-108 vintage) riding this sprint because it touches the
same file family (`sim_loop.py`, adjacent to this sprint's own
`sim_harness.h`/CMake edits) — it has no functional dependency on the
gut itself and can land first.

Per the issue's own "Suggested next step" (a): route
`set_read_hook()`/`set_write_hook()` through the existing
`_call_on_tick_thread()` seam, matching every other mutator, rather than
adding a mutex inside `sim_ctypes.cpp` (that would be a second fix
pattern for the same class of problem this file already solves one way).

## Acceptance Criteria

- [x] `SimLoop._set_hook()` (backing `set_read_hook()`/`set_write_hook()`)
      routes through `_call_on_tick_thread()`, matching `twist()`/
      `stop()`/`set_pid_enabled()`'s existing pattern — no direct
      cross-thread call into `sim_set_read_hook()`/`sim_set_write_hook()`
      remains.
- [x] The module's own "Threading model" docstring (which already states
      the invariant this fix restores) is updated if its wording implied
      an exception for hook registration.
- [x] `src/tests/testgui/test_sim_loop.py`'s
      `test_read_hook_fires_and_pass_through_returns_bytes` and
      `test_write_hook_can_swallow_a_command` still pass.
- [x] No new repro was required to land this fix (none exists — see
      sprint.md Open Questions #5); the fix is justified by matching the
      documented invariant, not by a reproduced crash.

## Implementation Plan

**Approach**: Change `_set_hook()`'s body to enqueue the ctypes call
through `_call_on_tick_thread()` instead of calling
`sim_set_read_hook()`/`sim_set_write_hook()` directly. Confirm the
context-manager `__enter__`/`__exit__` pair (register/unregister) both
go through the same seam, since the race is symmetric (register races a
concurrent `sim_step()`, and so does unregister).

**Files to modify**:
- `src/host/robot_radio/io/sim_loop.py` — `_set_hook()`, and the
  `read_hook()`/`write_hook()` context managers if they call ctypes
  directly rather than through `_set_hook()`.

**Files NOT to modify**: `src/sim/sim_ctypes.cpp` (no mutex added — the
existing single-thread-ownership pattern is preserved instead, per the
issue's own recommended direction (a) over (b)).

**Testing plan**:
- Run `src/tests/testgui/test_sim_loop.py` (existing) — must stay green.
- Optional, not required for acceptance: a tight register/unregister
  loop racing a busy tick thread, as a best-effort repro attempt (the
  issue's own "Suggested next step" note) — if it doesn't reproduce, do
  not treat that as blocking; the original crash was already
  non-reproducible on demand.
- `uv run python -m pytest src/tests/testgui/` green.

**Documentation updates**: none required beyond the docstring check
above; no architecture-doc change (this ticket doesn't touch anything
sprint 115's own Architecture section models as a subsystem).
