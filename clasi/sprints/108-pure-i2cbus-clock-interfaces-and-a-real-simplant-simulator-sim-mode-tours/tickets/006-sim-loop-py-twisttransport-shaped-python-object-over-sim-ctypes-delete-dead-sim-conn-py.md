---
id: "006"
title: "sim_loop.py: TwistTransport-shaped Python object over sim_ctypes; delete dead sim_conn.py"
status: open
use-cases: ["SUC-040", "SUC-042"]
depends-on: ["005"]
github-issue: ""
issue:
  - "plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md"
  - "sim-api-ctypes-abi-for-sim-mode-tours.md"
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# sim_loop.py: TwistTransport-shaped Python object over sim_ctypes; delete dead sim_conn.py

## Description

Stage 3 (part b) of the master plan. `host/robot_radio/io/sim_conn.py`'s
`SimConnection` resolves to `tests/_infra/sim/build/
libfirmware_host.{dylib,so}` against a ~40-symbol ABI from a deleted
pre-single-loop-rebuild subsystem graph (`Hal::PhysicsWorld`/
`Hal::SimOdometer` — none of which exist in the current tree). It has been
dead code (`SimConnection.connect()` always fails: "Sim library not
found") since commit `72d8be7e`. Delete it and replace it with a fresh
module built against ticket 005's new ABI.

Create `host/robot_radio/io/sim_loop.py`:
- A `TwistTransport`-shaped object — `twist(v_x, v_y, omega)`, `stop()`,
  `read_pending_binary_tlm_frames()` — matching exactly the protocol shape
  `host/robot_radio/planner/tour.py` (sprint 107) already consumes for
  hardware transports. This is the concrete, already-proven consumer
  interface the issue this ticket closes calls for designing against.
- A wall-clock tick thread: steps the sim forward at a real-time rate
  (mirroring `SimTransport`'s existing tick-thread pattern in
  `transport.py`, which this ticket's consumer — ticket 007 — reuses),
  delivering telemetry frames and true-pose updates as they arrive.
- `set_read_hook(cb)` / `set_write_hook(cb)`: Python wrappers around the
  raw ctypes `CFUNCTYPE` registration from ticket 005, handing the
  callback `(addr, buffer)` plus a `pass_through()` helper that calls
  `sim_default_read`/`sim_default_write`. This is the surface ticket 008's
  color-sensor regression test and ticket 009's 13 migrated register tests
  both build on — get its ergonomics right (context-managed
  register/unregister is a reasonable shape to consider).
- Fault-condition setters exposed as plain Python methods, mapped 1:1 onto
  ticket 005's C exports where possible.

Delete `host/robot_radio/io/sim_conn.py` and any test/module that only
exists to support it (grep for `sim_conn` importers first — confirm the
only consumers are `transport.py` (ticket 007 rewires this) and
`test_tour1_geometry.py` (already `_LIB_PRESENT`-guarded and skipping;
ticket 007 or 009 un-skips it against the new module).

## Acceptance Criteria

- [ ] `host/robot_radio/io/sim_conn.py` is deleted.
- [ ] `host/robot_radio/io/sim_loop.py` exists, exposes `twist()`/`stop()`/
      `read_pending_binary_tlm_frames()` satisfying the same shape
      `planner/tour.py` already consumes for hardware transports (verified
      by a direct read of `tour.py`'s own transport protocol, not assumed).
- [ ] A wall-clock tick thread steps the sim and delivers telemetry/truth
      without the caller manually pumping it.
- [ ] `set_read_hook`/`set_write_hook` Python wrappers exist with a working
      `pass_through()` helper; a hook registered from Python is
      demonstrably invoked (unit test).
- [ ] `grep -rn "sim_conn" host/ tests/` returns nothing except this
      ticket's own commit history / historical doc references.

## Implementation Plan

**Approach**: Build `sim_loop.py` to satisfy `tour.py`'s EXISTING protocol
shape — do not modify `tour.py` to accommodate `sim_loop.py` (per this
sprint's architecture-update.md dependency-direction note: the domain
layer's interface is fixed, the new infrastructure adapter conforms to it).

**Files to create**:
- `host/robot_radio/io/sim_loop.py`
- A focused unit test (e.g. `tests/testgui/test_sim_loop.py` or
  colocated with the existing `tests/testgui/` suite) exercising
  twist/stop/telemetry-drain and the hook wrapper against the real ctypes
  library (not a mock — this is exactly the seam that needs a real
  end-to-end check).

**Files to delete**:
- `host/robot_radio/io/sim_conn.py`

**Testing plan**:
- Existing: confirm no other module breaks on `sim_conn.py`'s removal
  (grep-verify import sites first).
- New: `test_sim_loop.py` — twist/stop round-trip, telemetry drain
  non-empty after a twist + step, a registered write hook observably
  changes behavior (e.g. swallows a duty write, wheel doesn't move — same
  assertion shape as ticket 005's own smoke check, now through the nicer
  Python wrapper).
- Verification command: `uv run python -m pytest tests/testgui/test_sim_loop.py`.

**Documentation updates**: `sim_loop.py`'s own module docstring
(mirroring `sim_conn.py`'s own docstring density) documenting the ABI it
targets and the reconciliation from the old (dead) module, so a future
reader has the same "what changed and why" trail `sim_conn.py`'s docstring
used to provide for its own predecessor.
