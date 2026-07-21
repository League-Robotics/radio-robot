---
title: SimLoop.set_read_hook()/set_write_hook() race the background tick thread --
  observed one segfault under heavy concurrent load
filed: 2026-07-19
filed_by: programmer (111-002, baseline sim-suite triage)
status: done
related: []
tickets:
- 115-001
sprint: '115'
---

# SimLoop hook registration races the tick thread

## What happened

While running the full suite repeatedly during 111-002 (sometimes with
several other heavy `pytest`/C++-compile processes running concurrently
in the same session), ONE run crashed the whole `pytest` process with a
segfault:

```
Fatal Python error: Segmentation fault
Current thread ... (most recent call first):
  File ".../src/host/robot_radio/io/sim_loop.py", line 956 in _tick_loop
  ...threading bootstrap frames...
Thread ... (most recent call first):
  File ".../src/tests/testgui/test_sim_loop.py", line 168 in test_read_hook_fires_and_pass_through_returns_bytes
  ...pytest runner frames...
```

`test_read_hook_fires_and_pass_through_returns_bytes` (and its neighbor
`test_write_hook_can_swallow_a_command`) register a Python callback via
`SimLoop.read_hook()`/`write_hook()` (context managers) while the
background tick thread is running.

**Not reproducible in isolation or under normal load**: re-running the
full suite, and this test file alone, multiple times afterward (including
immediately after the crash, and again later in the same session with no
other concurrent processes) never reproduced it again. The one crash
coincided with unusually heavy concurrent CPU load from this same
session (a C++ compile plus two other full-suite `pytest` runs in
parallel) -- likely widening a narrow race window rather than a
deterministic bug trigger.

## Suspected mechanism (not yet proven, worth a closer look)

`src/host/robot_radio/io/sim_loop.py`'s `_set_hook()` (the shared
implementation behind `set_read_hook()`/`set_write_hook()`) calls straight
into `sim_set_read_hook()`/`sim_set_write_hook()` **directly on the
calling thread** -- unlike every other mutator on `SimLoop` (`twist()`,
`stop()`, the fault-condition setters, `set_pid_enabled()`, ...), which
all route through `_run_or_enqueue()` or `_call_on_tick_thread()` so only
the tick thread ever touches the raw ctypes handle (see the module's own
"Threading model" docstring: "the raw `ctypes` handle ... is NOT
thread-safe for concurrent access, so exactly ONE thread ever touches
it").

`src/sim/sim_ctypes.cpp`'s `sim_set_read_hook()`/`sim_set_write_hook()`
(around line 333) call `SimPlant::setReadHook()`/`setWriteHook()`, which
plainly assign a `std::function` member (`readHook_`/`writeHook_`) with
no mutex/atomic of any kind. If the tick thread is concurrently inside
`sim_step()` invoking that same `std::function` (a real I2C hook fires
mid-step), a `set_read_hook(None)`-on-`__exit__` call from the TEST'S own
thread reassigning/destroying that `std::function` at the same moment is
a textbook data race -- and could explain a segfault inside `_tick_loop`'s
own `self._lib.sim_step(...)` call (sim_loop.py:956, exactly where the
crash's backtrace shows the tick thread).

This predates the pid-debugging WIP entirely -- the hook-wrapper design
is sprint 108 ticket 006 vintage (`sim_loop.py`'s own module docstring);
the one line of the merged WIP that touched this file
(`set_pid_enabled()`, commit `5f5a2ba7`) is unrelated and correctly routed
through `_call_on_tick_thread()`.

## Why not fixed as part of 111-002

- Not reproducible on demand -- there is nothing to verify a fix against
  without inducing the same abnormal concurrent-load conditions that (may
  have) triggered it once.
- Not one of 111-002's own 9 named baseline failures.
- A real fix (routing `set_read_hook()`/`set_write_hook()` through
  `_call_on_tick_thread()`, or adding a mutex around `readHook_`/
  `writeHook_` in `SimPlant`) touches production `sim_loop.py`/
  `sim_ctypes.cpp` -- out of this ticket's triage-only scope.

## Suggested next step

A future sprint should either (a) route `set_read_hook()`/
`set_write_hook()` through `_call_on_tick_thread()` like every other
`SimLoop` mutator (simplest, matches the module's own stated threading
invariant), or (b) add a mutex around `SimPlant`'s `readHook_`/
`writeHook_` in `sim_ctypes.cpp` if same-thread registration must stay
synchronous for some reason. Before landing either, try to get a reliable
repro (e.g. a tight loop of `read_hook()`/`write_hook()` register/
unregister cycles racing a busy tick thread) so the fix can be verified
rather than shipped on faith.
