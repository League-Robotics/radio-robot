---
id: '005'
title: Python simulation wrapper, pytest fixtures, and first real sim tests
status: done
use-cases:
- SUC-005
depends-on:
- '004'
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

- [x] `tests/_infra/sim/firmware.py`'s `Sim` class wraps the ctypes ABI: a
      context manager, `tick_for(total, step=24)`, and clean teardown
      (`sim_destroy` called exactly once per `Sim` instance).
- [x] `tests/sim/conftest.py`'s placeholder docstring is replaced with a
      real session-scoped `build_lib` fixture (runs `just build-sim` once
      per test session) and a function-scoped `sim` fixture (fresh `Sim()`
      per test) that issues `DEV WD 60000` immediately after `sim_create`
      so the 1 s `SerialSilenceWatchdog` does not neutralize motors during
      a long `tick_for` (**note**: the ticket text's literal `DEV WD
      3600000` exceeds `dev_commands.cpp`'s own `kDevWdArgs` range clamp of
      `[50, 60000]` ms and is rejected with `ERR badarg window`; `60000` —
      the firmware's own maximum — is used instead; see Closing Notes).
- [x] `host/robot_radio/io/sim_conn.py` is fixed up against ticket 004's
      actual ABI (function names/signatures reconciled; any delta from the
      28-symbol contract it originally expected is resolved here, not
      deferred).
- [x] First real tests, collected under `tests/sim/unit/` or
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
        (overriding the fixture's default `DEV WD 60000`) and confirms
        `EVT dev_watchdog` fires via `sim_get_async_evts`.
- [x] `uv run python -m pytest tests/sim` is green and collects
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

## Closing Notes (for ticket 006's implementer)

**`DEV WD` range clamp (ticket-text correction).** The ticket's own text
(both here and its Implementation Plan) said to send `DEV WD 3600000`.
`dev_commands.cpp`'s `kDevWdArgs` ArgDef clamps the window to `[50, 60000]`
ms — `3600000` is rejected with `ERR badarg window`, silently leaving the
watchdog at its 1 s default (confirmed empirically: a `DEV WD 3600000` +
long `tick_for` run gets neutralized by `EVT dev_watchdog` partway
through). Both `tests/sim/conftest.py`'s `sim` fixture and every test that
constructs its own `Sim()`/`SimConnection` directly use `60000` (the
firmware's own maximum) instead.

**A genuine SimMotor tick-latency artifact — avoid irregular `tick_for()`
steps.** `Hal::SimMotor::tick()` (`source/hal/sim/sim_motor.cpp`) computes
`rawVel = (pos - lastPosition_) / elapsedTime`, where `elapsedTime` is
*this* tick's own interval since the motor's last tick, but
`pos - lastPosition_` reflects the position delta produced by the
*previous* tick's `plant.update(dt_prev)` call — the documented one-tick
sample latency (`sim_hardware.h`'s file header: "each `Hal::SimMotor`
samples the plant's STILL-STALE reported encoder from the previous pass").
When tick intervals are uniform (`dt_prev == elapsedTime`, the "~24 ms
convention"), this is invisible. When they differ — e.g. a `tick_for()`
that ticks a shorter, irregular *remainder* step after a run of uniform
ones — the reading is scaled by `dt_prev / elapsedTime` and spikes
visibly: reproduced directly against `sim_api.cpp` with `DEV M 1 VEL 120`,
a converged, stable `sim_get_vel_l()` reading of ~120 mm/s after a run of
uniform 24 ms ticks jumped to ~192 mm/s the instant a single non-uniform
(8 ms) tick followed. `tests/_infra/sim/firmware.py`'s `Sim.tick_for()`
avoids this structurally: it only ever issues full `step`-sized
`sim_tick()` calls (`steps = total // step`), silently dropping any
remainder rather than ticking a shorter final step — this is a Python-side
test-authoring fix, not a firmware change, and is out of this ticket's
scope to "fix" in `source/`. Ticket 006 (porting legacy error-injection
suites) should keep any of its own tick-advancing helper(s) to this same
uniform-step discipline, or it will intermittently see this same spurious
velocity spike whenever a script's total duration isn't an exact multiple
of its tick step.

**`Sim` wrapper API surface settled this ticket** (`tests/_infra/sim/firmware.py`):
context manager (`__enter__`/`__exit__` → `sim_create`/`sim_destroy`,
`close()` idempotent), `tick_for(total, step=24)`, `command(line) -> str`,
`get_async_evts() -> str`, ground-truth reads (`true_pose()`,
`exact_pose()`, `true_wheel_travel()`, `true_velocity()`,
`set_true_wheel_travel()`, `set_true_pose()`), errored-observation reads
(`enc()`, `vel()`, `pwm()`, `otos_pose()`), and one method per error-knob
setter (`set_enc_scale_error`, `set_enc_slip`, `set_enc_noise`,
`set_stiction`, `set_motor_lag`, `set_trackwidth`,
`set_body_rotational_scrub`, `set_body_linear_scrub`,
`set_otos_linear_noise`, `set_otos_yaw_noise`,
`set_otos_linear_scale_error`, `set_otos_angular_scale_error`,
`set_otos_linear_drift`, `set_otos_yaw_drift`) — all take a `side` int
(0=left, 1=right, 2=both) where the C ABI does. No method name embeds a
unit (units live in trailing `# [unit]` comments), per
`.claude/rules/coding-standards.md`. Ticket 006 should build directly on
this class rather than re-deriving ctypes bindings — every `sim_*` symbol
ticket 004 exports is already bound in `_setup_types()`.

**`sim_conn.py` reconciliation delta** — see that file's own module
docstring for the full list; summary: dropped the "fused pose"
(`sim_get_pose_x/y/h`, no backing symbol — no EKF/fusion loop exists in
`source/` this sprint) from `_snapshot()`/`state_log` in favor of
`true_pose_*`/`otos_*`; `set_motor_offset()`/`set_otos_pose()`/
`enable_otos_fusion()` now raise `NotImplementedError` with a message
pointing at the missing ABI entry point (rather than an opaque
`AttributeError` or a silent no-op); `enable_otos_model()` is now a
documented no-op (`Hal::SimOdometer` always accumulates, no separate
"enable" step exists); `set_slip()`'s `turn_extra` parameter warns (rather
than silently no-op'ing) since it has no ABI backing; `set_enc()`'s
semantics changed to inject TRUE (not reported-only) wheel travel, the
only injection point this ABI exposes.
