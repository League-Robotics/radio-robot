---
id: '004'
title: 'Host-sim build and C ABI: CMakeLists and sim_api.cpp'
status: done
use-cases:
- SUC-004
depends-on:
- '002'
- '003'
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

- [x] `tests/_infra/sim/CMakeLists.txt` defines `add_library(firmware_host SHARED ...)`
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
- [x] `tests/_infra/sim/sim_api.cpp` implements a `SimHandle` owning
      `Subsystems::SimHardware` + `Subsystems::Drivetrain` +
      `CommandProcessor` + `source/dev_loop.h`'s `DevLoop`, plus a reply
      store for synchronous command replies and an async-EVT queue for
      loop-originated output (the watchdog `EVT dev_watchdog`, via
      `DevLoop`'s `defaultReply`/`defaultReplyCtx` — ticket 002).
- [x] `sim_create`/`sim_destroy`, `sim_tick(h, now)` (calls
      `devLoopTick(loop, now, nullptr)`), and
      `sim_command(h, line, reply, size)` (copies `line` into a
      `DevLoopStatement`, calls `devLoopTick(loop, now, &stmt)` at the
      **same** `now` as the most recent `sim_tick` — the dt=0 synchronous-
      command trick, safe because of ticket 003's re-entry guard) are
      implemented.
- [x] Ground-truth reads (`sim_get_true_pose_x/y/h` + `exact_pose` legacy
      aliases, `sim_get_true_enc_l/r`, `sim_get_true_vel_l/r`,
      `sim_set_true_wheel_travel`, `sim_set_true_pose`), errored-observation
      reads (`sim_get_enc_l/r`, `sim_get_vel_l/r`, `sim_get_pwm_l/r`,
      `sim_get_otos_x/y/h`), and every error-knob setter forward directly to
      ticket 003's `Hal::` free setter functions (`sim_setters.h`) — **one
      canonical call site per knob**, no duplicated logic between this
      file and `sim_setters.h`.
- [x] `sim_get_async_evts` drains the async-EVT queue `devLoopTick`'s
      loop-originated replies (e.g. watchdog fire) populate.
- [x] **No `SIMSET`/`SIMGET` wire command family, ever** — every knob and
      every ground-truth/telemetry read in this file is reachable only via
      a `sim_*` ctypes entry point, confirmed by grepping
      `source/commands/` for any new wire verb this ticket might have
      introduced (it should introduce none).
- [x] `just build-sim` succeeds and produces
      `tests/_infra/sim/build/libfirmware_host.{dylib,so}`.
- [x] The ARM build (`python build.py`) is unaffected — confirm it still
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

## Closing Notes (for ticket 005's implementer)

**justfile fix (small tweak, as anticipated by this ticket's own
Implementation Plan).** `just build-sim`'s `gen_default_config.py` step
hard-failed the moment `tests/_infra/sim/` existed for the recipe to reach
its `cmake` steps: that generator writes to `source/robot/DefaultConfig.cpp`,
a directory the 077 greenfield rebuild deliberately does not (yet)
recreate. `build.py` already guards this structurally (its own 077-001
comment, `if os.path.isdir(source/robot/)`); the `justfile` recipe had never
been updated to match. Fixed by mirroring the same guard in `justfile`'s
`build-sim` recipe (`if [ -d source/robot ]; then ...; fi`) — no other
build.py/justfile change was needed.

**ABI symbol list actually exported (40 `sim_*` symbols)** — see
`tests/_infra/sim/sim_api.cpp`:
- Lifecycle/loop (4): `sim_create`, `sim_destroy`, `sim_tick`, `sim_command`.
- Async (1): `sim_get_async_evts`.
- Ground truth (12): `sim_get_true_pose_x/y/h`, `sim_get_exact_pose_x/y/h`
  (legacy aliases for the same true-pose reads), `sim_get_true_enc_l/r`,
  `sim_get_true_vel_l/r`, `sim_set_true_wheel_travel`, `sim_set_true_pose`.
- Errored observation (9): `sim_get_enc_l/r` (PhysicsWorld's REPORTED
  accumulator), `sim_get_vel_l/r` (the two default plant-bound
  `Hal::SimMotor`s' own filtered `velocity()` — ports 1/2, `SimHardware`'s
  documented default LEFT/RIGHT binding), `sim_get_pwm_l/r` (the plant's raw
  commanded actuator value), `sim_get_otos_x/y/h`.
- Error-knob setters (14, one call site each into `hal/sim/sim_setters.h`):
  `sim_set_enc_scale_error`, `sim_set_enc_slip`, `sim_set_enc_noise`,
  `sim_set_stiction`, `sim_set_motor_lag`, `sim_set_trackwidth`,
  `sim_set_body_rotational_scrub`, `sim_set_body_linear_scrub`,
  `sim_set_otos_linear_noise`, `sim_set_otos_yaw_noise`,
  `sim_set_otos_linear_scale_error`, `sim_set_otos_angular_scale_error`,
  `sim_set_otos_linear_drift`, `sim_set_otos_yaw_drift`.

**Delta vs. `host/robot_radio/io/sim_conn.py`'s current (pre-005, stale)
expectation** — that file's ~28-symbol contract is from the OLD
(pre-greenfield-rebuild) tree's `Robot`/`MockHAL`/EKF-fusion model and does
**not** name-match this ABI at all; ticket 005 owns the rewrite. Concretely:
- `sim_conn.py` expects `sim_get_pose_x/y/h` (firmware EKF-fused pose) —
  this tree's dev-bench firmware has no EKF/fusion loop yet (per
  architecture-update.md, `Subsystems::Drivetrain` has no odometry this
  sprint), so there is no equivalent "fused pose" concept to expose; the
  closest concept is `sim_get_true_pose_x/y/h` (ground truth) or
  `sim_get_otos_x/y/h` (the OTOS-only errored estimate).
- `sim_conn.py`'s `sim_set_motor_offset`/`sim_enable_otos_model`/
  `sim_set_otos_fusion`/`sim_get_bench_otos_*`-style knobs have no
  equivalent here (no offset-factor ctypes entry point is wired — see
  `sim_api.cpp`'s comment on `PhysicsWorld::setOffsetFactor()` being
  deliberately left unwrapped by `sim_setters.h`; no OTOS-fusion/bench-OTOS
  concept exists in the new tree at all).
- `sim_conn.py`'s `sim_set_motor_slip`/`sim_set_encoder_noise` (single call,
  both wheels) map onto this ABI's more granular `sim_set_enc_slip`/
  `sim_set_enc_noise` (explicit `side` parameter: 0=left, 1=right, 2=both).
- This ABI additionally exposes `sim_get_true_enc_l/r`/`sim_get_true_vel_l/r`
  (ground truth, no error model) alongside the errored `sim_get_enc_l/r`/
  `sim_get_vel_l/r` — a true/errored split `sim_conn.py` does not have today.
- Not exposed by this ticket (out of scope, no acceptance-criteria line item
  called for it): `Subsystems::SimHardware::rebindPlantPorts()` — if a
  future test needs `sim_get_vel_l/r`/`sim_get_pwm_l/r` to track a rebound
  port pair rather than the fixed default (ports 1/2), a new ctypes entry
  point should be added then.

**CMake source-list surprise: none.** Every file in the explicit list
compiled and linked on the first `just build-sim` attempt with zero
additions or removals needed — the architecture doc's Step 4 dependency
graph was accurate as written. The one build-tooling surprise was entirely
outside the source list itself (the `gen_default_config.py` justfile issue
above, unrelated to which `.cpp` files are compiled).

**Manual verification performed** (beyond the smoke script): `DEV M 1 DUTY
50` followed by ~500 ms of `sim_tick` advances showed true pose/encoders/
velocity/PWM/OTOS all changing plausibly and consistently (reported encoder
== true encoder with all error knobs at their zero defaults, confirming the
zero-error determinism gate holds through this ABI); `DEV M 1 STATE`/
`DEV STOP` round-tripped correctly; the serial-silence watchdog fired
exactly one `EVT dev_watchdog` after simulated silence, correctly drained
by `sim_get_async_evts` and not before. All 14 knob setters were exercised
and did not crash.
