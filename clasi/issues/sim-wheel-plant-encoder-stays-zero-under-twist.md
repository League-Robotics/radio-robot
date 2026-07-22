---
status: pending
---

# Sim: WheelPlant encoder/pose stay flat at (0,0) under `SimLoop.twist()`, while OtosPlant shows realistic motion (5 tests failing)

## Description

Found 2026-07-22 while verifying a `protocol.py`/`binary_bridge.py`/
`velocity_pid.cpp` bench fix (calibration push routing, completion-ack
feedback, exact-zero-target creep). NOT caused by that work — see
"Ruled out" below.

`uv run python -m pytest` (full suite) currently fails 5 Sim-mode tests,
all with the same signature: a `SimTransport`/`SimLoop.twist()` (or the
equivalent managed-move path) command is sent, `Telemetry.cmd_vel`
correctly shows the commanded target, `Telemetry.mode` correctly flips
`I`->`V`->`I`, and `Telemetry.otos`/`otos_reading` show a REALISTIC
accelerate-then-decay curve (e.g. `otos.x`: 0 -> 7 -> 17 -> 29 -> 45 ->
79 -> 98 -> 117 -> ... -> 177, `otos_reading.v_x` peaking ~385mm/s then
decaying to 0) — but `Telemetry.enc`/`vel`/`pose` stay EXACTLY `(0, 0)`
/ `(0, 0)` / `(0, 0, 0)` for the entire run (up to 15s tested manually,
well past the drive command's own window).

Failing tests:
- `src/tests/testgui/test_transport.py::test_drive_produces_moving_telemetry`
- `src/tests/testgui/test_traces.py::test_encoder_trace_grows_with_forward_drive_via_dead_reckoning`
- `src/tests/testgui/test_traces.py::test_camera_trace_grows_in_step_with_ground_truth`
- `src/tests/testgui/test_error_divergence.py::test_enc_scale_err_separates_encoder_trace_from_camera_truth`
- `src/tests/testgui/test_sim_transport_tour1.py::test_tour_1_runs_to_completion_with_finite_small_closure`

## Ruled out

- **Not the `set_config_binary()` fire-and-poll fix** (`protocol.py`,
  same session): monkeypatched `NezhaProtocol.set_config_binary` to
  unconditionally return `None` (byte-for-byte the OLD, pre-fix
  behavior) and re-ran `test_drive_produces_moving_telemetry` manually —
  still fails identically. The Tier-1 config push this fix touches has
  nothing to do with `WheelPlant`'s own duty->velocity integration.
- **Not a stale build**: `python build.py` (repo root) rebuilt both the
  firmware hex and `libfirmware_host.dylib` from current source; re-ran
  the same 5 tests against the fresh dylib — identical failures, byte-
  identical `otos` trajectory values (deterministic, not flaky).
- **Not `velocity_pid.cpp`'s exact-zero-target fix** (same session,
  `if (target == 0.0f) return 0.0f;`): every failing scenario commands a
  NONZERO target (200mm/s), never touching that new branch.
- **Not test timing/system load**: manually waited 15s (3x the tests'
  own 5s bound) — encoder/pose never move even once.

## Suspected area

`src/tests/sim/plant/wheel_plant.cpp` (`TestSim::WheelPlant`) vs
`src/tests/sim/plant/otos_plant.cpp` (`TestSim::OtosPlant`) — two
independent plant models compiled into `libfirmware_host.dylib`.
`OtosPlant` clearly integrates SOME representation of commanded motion
correctly (real accel/decel curve); `WheelPlant`'s own duty->velocity
path (feeding `NezhaMotor`'s encoder simulation, hence
`Telemetry.enc`/`vel`/`pose`) appears to never advance from its own
`appliedDuty()`/`SimPlant` I2C hook, or advances but the RESULT never
reaches the Telemetry frame's `enc_left`/`enc_right`. Not root-caused
further this session (out of scope for the calibration/completion-ack
bench fix this was found alongside) — `src/tests/sim/unit/
devices_motor_harness.cpp`'s own direct `NezhaMotor`+`SimPlant` scenarios
(run standalone, not through `SimLoop`/`SimTransport`) all pass, which
narrows this toward the `SimLoop`/`RobotLoop`/`App::Drive` wiring level
rather than `NezhaMotor`/`MotorVelocityPid` themselves.

## Priority

Normal — blocks 5 Sim-mode tests (out of 1302 total; everything else
passes) but does not block real-hardware bench work (`src/tests/bench/`
scripts talk to the real robot directly, bypassing `SimLoop` entirely).
Worth a dedicated session: bisect `git log` for `src/tests/sim/plant/`,
`src/sim/sim_plant.cpp`, `src/firm/app/drive.cpp`,
`src/firm/devices/nezha_motor.cpp` to find when `WheelPlant` last
demonstrably produced a moving `TLMFrame` (the earlier bench-checklist
docs this sprint reference sim closure runs that DID show motion, so
this is plausibly a regression from some point after those).
