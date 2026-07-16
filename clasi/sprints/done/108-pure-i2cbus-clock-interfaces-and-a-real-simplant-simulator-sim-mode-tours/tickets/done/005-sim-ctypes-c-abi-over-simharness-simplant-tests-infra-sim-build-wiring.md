---
id: '005'
title: sim_ctypes C ABI over SimHarness/SimPlant + tests/_infra/sim build wiring
status: done
use-cases:
- SUC-040
- SUC-042
depends-on:
- '004'
github-issue: ''
issue:
- plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md
- sim-api-ctypes-abi-for-sim-mode-tours.md
completes_issue:
  plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md: false
  sim-api-ctypes-abi-for-sim-mode-tours.md: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# sim_ctypes C ABI over SimHarness/SimPlant + tests/_infra/sim build wiring

## Description

Stage 3 (part a) of the master plan, and the ticket that delivers
`clasi/issues/sim-api-ctypes-abi-for-sim-mode-tours.md`'s core ask: a
ctypes-callable C ABI over the new `SimHarness`/`SimPlant` (note: that
issue originally scoped the ABI over the OLDER `tests/sim/support/
sim_api.h`'s `SimApi` — this sprint's own Stage 2 deletes `SimApi`
outright and replaces it with `SimPlant`/`sim_harness.h`, so the ABI target
is the NEW composition, not the one the issue originally named; this
supersedes that issue's specific target while fully satisfying its intent).

Create `tests/_infra/sim/sim_ctypes.cpp`: `extern "C"` exports over
`SimHarness`/`SimPlant`, mirroring the shape the deleted (pre-`72d8be7e`)
`tests/_infra/sim/sim_api.cpp` used (a fresh implementation, not a restore
— that file targeted a different, pre-single-loop-rebuild subsystem
graph).

Exports needed (design the exact names during implementation; this is the
capability list, not a frozen signature list):
- Lifecycle: create / destroy a `SimHarness` instance (opaque handle).
- Stepping: `step(handle, n)`.
- Command injection: `inject_twist(handle, v_x, v_y, omega)`,
  `inject_stop(handle)` — the two verbs `planner/tour.py`'s
  `TwistTransport`-shaped protocol needs (see ticket 006).
- Telemetry: `drain_tlm(handle, buf, len)` (or equivalent) —
  `read_pending_binary_tlm_frames()`-shaped.
- True pose: `get_true_pose_x/y/h(handle)`.
- Fault-condition setters: thin call-throughs to `SimPlant`'s fault knobs
  (disconnect, freeze, dropout, OTOS noise/drift) — reconcile against
  `sim_prefs.py`'s existing `PROFILE_TO_SIM_SETTER` key names where a
  1:1 mapping makes sense (ticket 006/007's job to wire the Python side;
  this ticket just needs the C exports to exist).
- Hook surface (the master plan's Target architecture, verbatim):
  `sim_set_read_hook(h, cb, ctx)` / `sim_set_write_hook(h, cb, ctx)`
  (register/clear, `cb=NULL` clears) and `sim_default_read(h, addr, buf,
  len)` / `sim_default_write(h, addr, buf, len)` (the pass-through a
  Python callback calls to run `SimPlant`'s real response).

Also stand up `tests/_infra/sim/CMakeLists.txt` (the directory does not
currently exist — deleted wholesale by commit `72d8be7e`) building a shared
library (`libfirmware_host`) from: `sim_ctypes.cpp`, `sim_plant.cpp`,
`sim_harness.h`'s dependencies, the reused `tests/sim/plant/*.cpp` +
`tests/sim/support/{fake_transport,wire_test_codec}.*`, and the REAL
`source/**` graph (the same files the ARM build compiles, built here as a
host binary against the pure interfaces from ticket 001/010). `build.py`'s
existing `build_host_sim()` (lines ~190-210) already targets
`tests/_infra/sim` and is currently a dormant no-op (its own comment says
"self-heals once tests/_infra/sim/... reappears") — verify it picks this
up with NO changes needed, or make the minimal change if its assumptions
have drifted.

## Acceptance Criteria

- [x] `tests/_infra/sim/CMakeLists.txt` exists; `python build.py` (default,
      not `--fw-only`) builds `libfirmware_host` successfully.
- [x] `tests/_infra/sim/sim_ctypes.cpp` exports create/destroy/step/
      inject_twist/inject_stop/drain_tlm/true-pose, the fault-condition
      setters, and the hook-registration + pass-through exports.
- [x] Every export is a thin call-through (no decision logic) — verified
      by code review, not just tests.
- [x] A ctypes smoke test from Python (can be a throwaway script for this
      ticket's own verification, formalized in ticket 006) loads the
      library, creates a harness, injects a twist, steps, and reads back a
      moved true pose.
- [x] A Python-registered write hook (via the raw ctypes call, ahead of
      ticket 006's nicer wrapper) can swallow a duty write and the wheel
      does not move, proving the hook ABI works end-to-end from Python.

## Implementation Plan

**Approach**: Design the export surface against `planner/tour.py`'s
ALREADY-PROVEN `TwistTransport`-shaped consumer interface (`twist()`/
`stop()`/`read_pending_binary_tlm_frames()`) — this is the concrete
validated-consumer target `sim-api-ctypes-abi-for-sim-mode-tours.md`
itself calls out as the thing to design against, now that it exists.

**Files to create**:
- `tests/_infra/sim/sim_ctypes.cpp`
- `tests/_infra/sim/CMakeLists.txt`

**Files to modify**:
- `build.py` (only if `build_host_sim()`'s assumptions have drifted from
  what this ticket actually produces — check first, minimal-diff if so).

**Testing plan**:
- New: a ctypes smoke script exercising every exported symbol at least
  once (lifecycle, step, inject, drain, true-pose, one fault setter, the
  hook pair). This becomes the seed for ticket 006's `sim_loop.py`.
- Verification command: `python build.py && python <smoke_script>.py`.

**Documentation updates**: `sim_ctypes.cpp`'s own file header documenting
the exported symbol list and the hook pass-through contract (mirrors the
deleted `sim_api.cpp`'s own documentation density, per this sprint's
architecture-update.md Decision 2).
