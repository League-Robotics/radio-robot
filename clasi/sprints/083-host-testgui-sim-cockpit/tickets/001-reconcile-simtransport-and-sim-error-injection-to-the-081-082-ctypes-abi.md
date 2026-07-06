---
id: '001'
title: Reconcile SimTransport and sim-error injection to the 081/082 ctypes ABI
status: done
use-cases:
- SUC-001
- SUC-004
depends-on: []
github-issue: ''
issue: host-testgui-sim-cockpit.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Reconcile SimTransport and sim-error injection to the 081/082 ctypes ABI

## Description

`host/robot_radio/testgui/transport.py`'s `SimTransport` predates sprints 081
(host ctypes sim) and 082 (pose telemetry) and calls a ctypes/wire surface
that no longer exists:

- Four method calls have no match on `tests/_infra/sim/firmware.py`'s `Sim`
  class: `sim.get_true_pose()` (real name `true_pose()`), `sim.send_command()`
  (real name `command()`), `sim.set_true_velocity()` (no such method — no
  ctypes symbol backs it at all), `sim.set_field_profile()` (no such method —
  the old `_rotationalSlip` test-infra channel it drove is gone). Also
  `sim.tick_for(total, step_ms=...)` — the real keyword is `step`.
- `_apply_profile_to_sim()` builds `SIMSET k=v ...` wire lines. No `SIMSET`
  verb is registered anywhere in `source/commands/` (confirmed by reading
  every `makeCmd`/`makeSchemaCmd` registration — the complete current verb
  set is `PING`/`VER`/`HELP`/`ECHO`/`ID`/`STREAM`/`SNAP`/`DEV M`/`DEV DT`/
  `DEV STATE`/`DEV STOP`/`DEV WD`).
- Bare `except Exception` blocks around these calls currently swallow the
  failures silently — `SimTransport` "connects" today but never actually
  applies an error profile.

This ticket rewrites `SimTransport` to own a `SimConnection`
(`host/robot_radio/io/sim_conn.py`) instead of a raw `Sim`, and rewrites
`_apply_profile_to_sim()`/`sim_prefs.py`'s field-to-wire-key map to call the
sprint-081 ctypes error-knob setters directly — see
`architecture-update.md` Design Rationale Decisions 1 and 4 for the full
reasoning (why `SimConnection`, not raw `Sim`; why the two no-ctypes-backing
fields stay visible with a tooltip rather than being hidden).

## Acceptance Criteria

- [x] `SimTransport`'s tick-thread owns a `SimConnection` instance (not a raw
      `Sim`), constructed/destroyed via its `connect()`/`disconnect()` (it has
      no context-manager protocol).
- [x] All one-way drive/stop commands go through `conn.send_fast(line)`
      (fire-and-forget, no forced time advance).
- [x] Synchronous commands needing a reply go through
      `conn.send(line, read_timeout=0, stop_token=None)` (zero-time-advance
      synchronous reply) — verified this does not tick sim time as a side
      effect of a query.
- [x] The tick-thread's periodic wall-clock advance is one
      `conn.tick(speed_factor * _SIM_TICK_STEP_DURATION)` call per iteration,
      replacing the old manual `sim.tick_for()` + `sim.get_async_evts()` pair.
- [x] `conn.clear_state_log()` is called once per tick-thread iteration so
      `SimConnection`'s internal `_state_log` (built for bounded pytest runs,
      not an open-ended GUI session) never grows unbounded over a long
      interactive session.
- [x] Ground truth is read via `conn.get_true_pose()` (dict `{x,y,h}`) and
      teleported via `conn.set_true_pose()`/`conn.set_true_wheel_travel()`;
      the dead `set_true_velocity()` call is removed (no ctypes backing
      exists — documented, not silently retried).
- [x] `_apply_profile_to_sim()` no longer builds any `SIMSET` wire string. It
      calls the ctypes setters directly through a new
      `sim_prefs.PROFILE_TO_SIM_SETTER` map (field name -> `SimConnection`
      setter method name), replacing `sim_prefs.PROFILE_TO_SIMSET_KEY`.
      `encoder_noise` fans out via `set_enc_noise(2, value)` (side=2 = both,
      one call); `enc_scale_err_l`/`enc_scale_err_r` call
      `set_enc_scale_error(0, ...)`/`set_enc_scale_error(1, ...)`
      independently.
- [x] `motor_offset_l`, `motor_offset_r`, and `slip_turn_extra` keep their
      JSON keys in `sim_prefs.DEFAULT_PROFILE` (existing persisted profile
      files keep loading) but are documented as having no ctypes backing:
      `_apply_profile_to_sim()` skips applying them and logs a one-time
      `[WARN]` when either is set away from its neutral value
      (`1.0`/`1.0`/`0.0` respectively). **Stakeholder decision**: the
      corresponding Sim Errors panel spin boxes (`sim_err_motor_offset_l`,
      `sim_err_motor_offset_r`, `sim_err_slip_turn` in `__main__.py`) stay
      **visible** — do not hide or remove them — each gets a tooltip stating
      it is "not supported in sim" / has no effect against the current
      ctypes ABI.
- [x] Before finalizing the `otos_lin_drift`/`otos_yaw_drift` mapping, read
      `source/hal/sim/physics_world.h` (and/or `sim_setters.h`) to confirm
      whether `sim_set_otos_linear_drift`/`sim_set_otos_yaw_drift` apply a
      one-shot bias or a per-tick rate. The old `SIMSET` wire keys
      (`otosLinDriftMmS`/`otosYawDriftDegS`) were a rate in mm/s and deg/s;
      the new ctypes setters are tagged `[mm]`/`[rad]` in
      `firmware.py`/`sim_conn.py` (a bias). Update `sim_prefs`'s field
      labels/units/scaling to match whatever the implementation actually
      does — do not assume the old unit carries over unchanged.
- [x] `SimTransport.connect()`'s existing "Build required" `QMessageBox`
      warning path (missing `libfirmware_host.*`) is unchanged.
- [x] `turn_scrub_factor` (used by the Sim Errors display) is updated to read
      from whichever field now carries the rotational-scrub concept
      (`body_rot_scrub`), consistent with `slip_turn_extra` no longer having
      a live ctypes effect.

## Testing

- **Existing tests to run**: none yet exist for this module in the rebuilt
  `tests/` tree (the old suite lives in `tests_old/testgui/`, ported in
  ticket 004) — run `uv run python -m robot_radio.testgui` manually
  (`QT_QPA_PLATFORM=offscreen` for a headless smoke import) to confirm no
  import-time errors.
- **New tests to write**: a headless test (no `QApplication` needed) that
  constructs `SimTransport`, calls `connect()`, sends
  `"DEV DT PORTS 1 2"` and `"DEV DT VW 200 0 0"` via `send()`, ticks briefly,
  and asserts a `TLMFrame` with `mode == "S"` and nonzero `vel` arrives via
  `on_telemetry`; a test that calls `apply_error_profile()` with each profile
  field set to a distinctive nonzero value and asserts (via a fake/mocked
  `SimConnection`, or by reading back `sim_get_*` through the real one) that
  the correct setter was invoked with the correct value, including the two
  no-op fields logging a `[WARN]` and not raising; a test that `disconnect()`
  cleanly joins the tick-thread with no lingering handle.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui -k transport` (once ticket 004 creates the directory; until then, run the new test file directly with `uv run pytest <path>`).
