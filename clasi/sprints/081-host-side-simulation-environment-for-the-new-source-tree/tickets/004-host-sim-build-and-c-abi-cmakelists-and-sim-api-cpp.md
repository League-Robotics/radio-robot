---
id: '004'
title: 'Host-sim build and C ABI: CMakeLists and sim_api.cpp'
status: open
use-cases: [SUC-004]
depends-on: ['002', '003']
github-issue: ''
issue: host-side-simulation-environment-for-the-new-tree-design-write-up.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host-sim build and C ABI: CMakeLists and sim_api.cpp

## Description

Stand up `tests/_infra/sim/` — currently absent from the working tree (only
`tests_old/_infra/sim/` and a stale, uncommitted worktree copy exist) — with
a CMake build producing `libfirmware_host` and a C ABI over it. `build.py`'s
`build_host_sim()` and `just build-sim` already point at this directory and
self-heal the moment it exists; `host/robot_radio/io/sim_conn.py` already
expects a ~28-symbol ctypes contract (fixed up in ticket 005).

Depends on ticket 002 (`source/dev_loop.h`'s `DevLoop`/`devLoopTick`) and
ticket 003 (`Subsystems::SimHardware`/`Hal::PhysicsWorld` and their ctypes
hooks).

## Acceptance Criteria

- [ ] `tests/_infra/sim/CMakeLists.txt` defines `add_library(firmware_host SHARED ...)`
      with `-DHOST_BUILD=1 -DROBOT_DEV_BUILD=1`, C++ standard matched to the
      firmware, and an **explicit** source list (no glob-then-filter):
      `kinematics/*.cpp`, `subsystems/drivetrain.cpp`,
      `subsystems/sim_hardware.cpp`,
      `commands/{arg_parse,command_processor,dev_commands,system_commands}.cpp`,
      `dev_loop.cpp`, `hal/sim/*.cpp`, `hal/velocity_pid.cpp`,
      `types/clock_host.cpp`, `com/i2c_bus_host.cpp`, `sim_api.cpp`.
      **Absent**: `com/i2c_bus.cpp`, `subsystems/communicator.*`,
      `subsystems/nezha_hardware.cpp`, `hal/nezha/*.cpp`, `types/clock.cpp`,
      `main.cpp`.
- [ ] `tests/_infra/sim/sim_api.cpp` implements a `SimHandle` owning
      `Subsystems::SimHardware` + `Subsystems::Drivetrain` +
      `CommandProcessor` + `source/dev_loop.h`'s `DevLoop`, plus a reply
      store for synchronous command replies and an async-EVT queue for
      loop-originated output (the watchdog `EVT dev_watchdog`, via
      `DevLoop`'s `defaultReply`/`defaultReplyCtx` — ticket 002).
- [ ] `sim_create`/`sim_destroy`, `sim_tick(h, now)` (calls
      `devLoopTick(loop, now, nullptr)`), and
      `sim_command(h, line, reply, size)` (copies `line` into a
      `DevLoopStatement`, calls `devLoopTick(loop, now, &stmt)` at the
      **same** `now` as the most recent `sim_tick` — the dt=0 synchronous-
      command trick, safe because of ticket 003's re-entry guard) are
      implemented.
- [ ] Ground-truth reads (`sim_get_true_pose_x/y/h` + `exact_pose` legacy
      aliases, `sim_get_true_enc_l/r`, `sim_get_true_vel_l/r`,
      `sim_set_true_wheel_travel`, `sim_set_true_pose`), errored-observation
      reads (`sim_get_enc_l/r`, `sim_get_vel_l/r`, `sim_get_pwm_l/r`,
      `sim_get_otos_x/y/h`), and every error-knob setter forward directly to
      ticket 003's `Hal::` free setter functions (`sim_setters.h`) — **one
      canonical call site per knob**, no duplicated logic between this
      file and `sim_setters.h`.
- [ ] `sim_get_async_evts` drains the async-EVT queue `devLoopTick`'s
      loop-originated replies (e.g. watchdog fire) populate.
- [ ] **No `SIMSET`/`SIMGET` wire command family, ever** — every knob and
      every ground-truth/telemetry read in this file is reachable only via
      a `sim_*` ctypes entry point, confirmed by grepping
      `source/commands/` for any new wire verb this ticket might have
      introduced (it should introduce none).
- [ ] `just build-sim` succeeds and produces
      `tests/_infra/sim/build/libfirmware_host.{dylib,so}`.
- [ ] The ARM build (`python build.py`) is unaffected — confirm it still
      builds `MICROBIT.hex` with no reference to anything under
      `tests/_infra/sim/`.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim tests/unit`
  (must remain green — this ticket adds a new build target, it must not
  regress anything already collected). `python build.py --fw-only` to
  confirm the ARM path is unaffected.
- **New tests to write**: a minimal smoke test (can be Python, calling
  `sim_create`/`sim_tick`/`sim_command("PING")`/`sim_destroy` directly via
  ctypes, ahead of ticket 005's full `Sim` wrapper) proving the library
  loads and the ABI responds — this can be a small script under
  `tests/_infra/sim/` itself, not necessarily pytest-collected yet (ticket
  005 owns the real pytest fixtures).
- **Verification command**: `just build-sim`, then the smoke script above;
  `uv run python -m pytest tests/sim -q`.

## Implementation Plan

**Approach:**

1. Create `tests/_infra/sim/CMakeLists.txt` with the explicit source list
   above — cross-check every entry against `architecture-update.md`'s
   dependency graph (Step 4) before finalizing.
2. Implement `sim_api.cpp`'s `SimHandle` and the lifecycle/loop functions
   (`sim_create`/`sim_destroy`/`sim_tick`/`sim_command`).
3. Implement the ground-truth/errored-observation/error-knob functions,
   each forwarding to a `sim_setters.h`/`PhysicsWorld`/`SimMotor`/
   `SimOdometer` accessor — no new logic invented here beyond ctypes
   marshalling (POD in, POD/pointer out).
4. Implement `sim_get_async_evts` over `DevLoop`'s default-reply sink.
5. Run `just build-sim`; fix any compile errors surfaced by the explicit
   source list (a missing file in the list is a link error, not a silent
   omission — treat any such error as a signal to re-check the dependency
   graph, not just add the file blindly).
6. Write and run the minimal ctypes smoke script.

**Files to create:**
- `tests/_infra/sim/CMakeLists.txt`
- `tests/_infra/sim/sim_api.cpp`
- A minimal ctypes smoke script (e.g. `tests/_infra/sim/smoke_check.py`)

**Files to modify:** none expected — `build.py`/`justfile` already point at
this directory; if either needs a small tweak (e.g. a path assumption that
doesn't hold), note it explicitly in the ticket's closing notes.

**Testing plan:** see "Testing" section above.

**Documentation updates:** none required — this is a build/ABI plumbing
ticket with no wire-visible surface. If the final ABI's function list
diverges from `sim_conn.py`'s current 28-symbol expectation in a way ticket
005 needs to know about, note the delta explicitly in this ticket's closing
notes for the next ticket's implementer.
