---
id: '006'
title: 'sim_loop.py: TwistTransport-shaped Python object over sim_ctypes; delete dead
  sim_conn.py'
status: done
use-cases:
- SUC-040
- SUC-042
depends-on:
- '005'
github-issue: ''
issue:
- plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md
- sim-api-ctypes-abi-for-sim-mode-tours.md
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

- [x] `host/robot_radio/io/sim_conn.py` is deleted.
- [x] `host/robot_radio/io/sim_loop.py` exists, exposes `twist()`/`stop()`/
      `read_pending_binary_tlm_frames()` satisfying the same shape
      `planner/tour.py` already consumes for hardware transports (verified
      by a direct read of `tour.py`'s own transport protocol, not assumed).
- [x] A wall-clock tick thread steps the sim and delivers telemetry/truth
      without the caller manually pumping it.
- [x] `set_read_hook`/`set_write_hook` Python wrappers exist with a working
      `pass_through()` helper; a hook registered from Python is
      demonstrably invoked (unit test).
- [x] `grep -rn "sim_conn" host/ tests/` returns nothing except this
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

## Completion Notes

**Built**: `host/robot_radio/io/sim_loop.py` — `SimLoop`, a ctypes wrapper
over `sim_ctypes.cpp`'s 19-symbol ABI. Implements `planner.executor.
TwistTransport` directly (`twist()`/`stop()`/
`read_pending_binary_tlm_frames()`, verified against that Protocol's own
declared methods, not assumed). Owns a background wall-clock tick thread
(`connect(start_tick_thread=True)`, default) that steps `sim_step()` at
real time (one 50ms cycle per tick, matching
`TestSim::SimHarness::kCycleDtUs`), drains `sim_drain_tlm()` each
iteration (dearmor/parse via the same `pb2` codec a real robot's replies
use), and delivers frames to `on_telemetry` / ground truth to `on_truth`.
`suspend_telemetry_reader()`/`resume_telemetry_reader()` mirror
`_HardwareTransport`'s own toggle. `set_read_hook`/`set_write_hook` wrap
`ctypes.CFUNCTYPE` registration with a `cb(addr, buf)` surface plus
`pass_through(addr, buf, length, write)`; `read_hook()`/`write_hook()`
context managers register-then-clear. Fault setters
(`set_wheel_disconnected`/`set_wheel_freeze`/`set_wheel_dropout_rate`/
`set_otos_drift`) and `get_true_pose()` round-trip synchronously onto the
tick thread. `connect(start_tick_thread=False)` + `step(cycles)` gives a
fully synchronous, single-threaded mode for deterministic register-level
tests (ticket 009's own shape).

**Deleted**: `host/robot_radio/io/sim_conn.py` (948 lines, the dead
~40-symbol ABI). Every importer was tracked down (not just the two named
in this ticket's own description) and fixed so `grep -rn "sim_conn" host/
tests/` returns only this ticket's own historical doc trail (this
paragraph and `sim_loop.py`'s module docstring): `host/robot_radio/
__init__.py`'s lazy `SimConnection` re-export (now `SimLoop`),
`host/robot_radio/testkit/target.py`'s `make_target("sim")` branch (raises
`NotImplementedError` with a clear message — that branch needs a
`TwistTransport`-shaped rewrite, not a drop-in import swap, and was
already effectively dead since `sim_conn.py` never connected), and doc-only
mentions in `protocol.py`, `_legacy_tlm_text.py`, `sim_prefs.py`,
`test_traces.py`, `profiled_motion_harness.cpp`, and
`tests/notebooks/wheel_motion_trace.ipynb` (reworded to stay accurate
without the dead module path).

**SimTransport rewire deferred to ticket 007** (explicitly, per this
ticket's own instructions): `testgui/transport.py`'s `SimTransport` still
has its whole pre-108 shape (config-simulation plumbing, `_apply_profile_
to_sim`, `_drain_cmd_queue`, etc.) — none of that maps onto `SimLoop`'s
narrower ABI (no generic wire/config-channel simulation at all, only
command injection + telemetry drain + fault knobs). Minimum-viable fix
applied here: the top-level `from robot_radio.io.sim_conn import
SimConnection` import is removed (so the module — and the whole
`uv run python -m pytest` suite — collects with no `ImportError`), and
`SimTransport._tick_loop()` now fails FAST and cleanly (one log line +
`self._sim_ready_event.set()`) instead of touching the deleted
`SimConnection` name (which would otherwise NameError inside the daemon
thread and leave `connect()` hanging its full 5s timeout). Every other
`SimTransport` method (`_apply_field_profile`, `_apply_profile_to_sim`,
`_deliver_sim_truth`, `_drain_cmd_queue`, `_handle_evt_lines`, `set_true_
pose`, ...) is untouched, dead code until ticket 007 rewires the class
onto `SimLoop`.

**Full-suite verification** (`uv run python -m pytest`, 1118 collected —
1110 pre-existing + 8 new `test_sim_loop.py`): 1095 passed, 5 xfailed, 13
failed, 5 errors — every failure/error accounted for and expected:
  - 8 `tests/sim/unit/test_app_*`/`test_devices_*` harness-compile
    failures: pre-existing, unrelated to this ticket (`required source
    missing: source/devices/i2c_bus_host.cpp` — ticket 001/009 scope, not
    touched here).
  - 5 failed + 5 errored `tests/testgui/test_calibration_push_on_connect.py`,
    `test_transport.py`, `test_error_divergence.py`, `test_traces.py`:
    all fail at the SAME assertion (`SimTransport failed to connect`),
    the direct, documented consequence of the SimTransport stub above —
    deferred to ticket 007.
  - No new/unexplained failures.

**Smoke verification** (throwaway script, removed after use): `SimLoop`
constructed, connected, `twist(150, 0, 300)`; `read_pending_binary_tlm_
frames()` returned 15 frames with advancing `enc`/`pose`; `get_true_pose()`
showed x ≈ 49mm after the twist; a registered read hook fired 18 times at
real wire addresses (0x20 motor, 0x2E OTOS) and `pass_through()` returned
valid bytes. `tests/testgui/test_sim_loop.py` (8 tests, all passing)
covers the same ground formally: TwistTransport shape, twist/stop corr_ids,
telemetry drain, true-pose advance, suspend/resume toggle, a fault setter,
read-hook firing + pass-through, and a write hook observably swallowing a
command (wheel does not move).
