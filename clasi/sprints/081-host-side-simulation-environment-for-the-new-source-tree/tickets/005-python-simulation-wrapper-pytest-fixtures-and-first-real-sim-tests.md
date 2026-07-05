---
id: '005'
title: Python simulation wrapper, pytest fixtures, and first real sim tests
status: open
use-cases: [SUC-005]
depends-on: ['004']
github-issue: ''
issue: host-side-simulation-environment-for-the-new-tree-design-write-up.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Python simulation wrapper, pytest fixtures, and first real sim tests

## Description

Give Python test-friendly ergonomics over ticket 004's C ABI, replace
`tests/sim/conftest.py`'s documented placeholder with real fixtures, and
write the first substantive sim tests — turning
`uv run python -m pytest tests/sim` from "collects a placeholder" into "a
real, green regression suite."

Depends on ticket 004 (the compiled `libfirmware_host` and its C ABI).

## Acceptance Criteria

- [ ] `tests/_infra/sim/firmware.py`'s `Sim` class wraps the ctypes ABI: a
      context manager, `tick_for(total, step=24)`, and clean teardown
      (`sim_destroy` called exactly once per `Sim` instance).
- [ ] `tests/sim/conftest.py`'s placeholder docstring is replaced with a
      real session-scoped `build_lib` fixture (runs `just build-sim` once
      per test session) and a function-scoped `sim` fixture (fresh `Sim()`
      per test) that issues `DEV WD 3600000` immediately after `sim_create`
      so the 1 s `SerialSilenceWatchdog` does not neutralize motors during
      a long `tick_for`.
- [ ] `host/robot_radio/io/sim_conn.py` is fixed up against ticket 004's
      actual ABI (function names/signatures reconciled; any delta from the
      28-symbol contract it originally expected is resolved here, not
      deferred).
- [ ] First real tests, collected under `tests/sim/unit/` or
      `tests/sim/system/` per `tests/CLAUDE.md`'s domain split:
      - **Plant correctness**: commanded drive/turn geometry matches true
        pose within the plant's own numerical tolerance.
      - **Errored-observation split**: nonzero error knobs make reported
        encoder/OTOS diverge from true by the configured amount; zeroing
        every knob restores bit-for-bit agreement (re-exercises ticket
        003's determinism gate through the Python wrapper, not only at the
        C++ harness level).
      - **Velocity-PID response**: a commanded velocity step's rise time,
        overshoot, and settle fall within the same envelope ticket 001's
        bench pass used — meaningful now that sim and hardware share
        `Hal::MotorVelocityPid`.
      - **Protocol round-trips**: PING, the DEV M/DT family, `ERR unsupported`
        (e.g. `DEV M n POS` against `SimMotor`), and the watchdog path.
      - **Determinism gate**: an identical command script run twice
        produces bit-identical state logs.
      - **Watchdog policy**: one dedicated test lowers the watchdog window
        (overriding the fixture's default `DEV WD 3600000`) and confirms
        `EVT dev_watchdog` fires via `sim_get_async_evts`.
- [ ] `uv run python -m pytest tests/sim` is green and collects
      substantively more than `test_placeholder.py`.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full default run,
  `testpaths = ["tests/sim", "tests/unit"]`) must stay green; existing
  `tests/sim/unit/*_harness.cpp`-backed tests must be unaffected by the new
  fixtures (they run their own ad hoc compile, independent of `build_lib`).
- **New tests to write**: the six categories listed in the acceptance
  criteria above.
- **Verification command**: `uv run python -m pytest tests/sim -q` (and the
  full `uv run python -m pytest` to confirm no cross-domain regression).

## Implementation Plan

**Approach:**

1. Write `tests/_infra/sim/firmware.py`'s `Sim` class — ctypes bindings
   over every `sim_*` function ticket 004 exposes, a context manager
   (`__enter__`/`__exit__` calling `sim_create`/`sim_destroy`), and
   `tick_for(total, step=24)` (repeated `sim_tick` calls advancing `now` by
   `step` each time, matching the design's ~24 ms increment convention).
2. Replace `tests/sim/conftest.py`'s placeholder with `build_lib`
   (session-scoped, autouse, shells out to `just build-sim` or
   `build.build_host_sim()`) and `sim` (function-scoped, yields a fresh
   `Sim()`, sends `DEV WD 3600000` immediately after create, tears down in
   a `finally`).
3. Fix up `host/robot_radio/io/sim_conn.py` against the real ABI from
   ticket 004 — this is the same module the (separately scoped, later)
   TestGUI/SimTransport revival would eventually build on, so keep its
   surface close to `sim_conn.py`'s pre-existing expectations rather than
   inventing a new shape.
4. Write the first real tests, one file/class per acceptance-criteria
   category above.
5. Run the full suite; confirm the default `uv run python -m pytest`
   invocation collects and passes everything.

**Files to create:**
- `tests/_infra/sim/firmware.py`
- New test files under `tests/sim/unit/` and/or `tests/sim/system/` (one
  per category: plant correctness, errored-observation split, velocity-PID
  response, protocol round-trips, determinism, watchdog policy)

**Files to modify:**
- `tests/sim/conftest.py`
- `host/robot_radio/io/sim_conn.py`

**Testing plan:** see "Testing" section above.

**Documentation updates:** none required beyond code/test docstrings — no
wire or architecture change. If `sim_conn.py`'s fix-up changes anything a
future TestGUI revival would need to know, leave a clear docstring/comment
there (not a separate doc file) since that revival is explicitly out of
this sprint's scope (Open Question 5).
