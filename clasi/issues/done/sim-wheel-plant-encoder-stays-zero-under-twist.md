---
status: resolved
---

# Sim: WheelPlant encoder/pose stay flat at (0,0) under `SimLoop.twist()`, while OtosPlant shows realistic motion (5 tests failing)

**RESOLVED 2026-07-22** — root-caused and fixed the same day by a dedicated
systematic-debugging session. This file's original "pre-existing, not caused
by this session's work" conclusion was WRONG (see "Original claims,
corrected" below); the text is preserved for the record with corrections.

## Root cause (verified)

`sim_ctypes.cpp`'s `sim_configure_motor()` (the Tier-2 velFiltAlpha/fwdSign
config-load surface `SimLoop.configure_from_robot()` calls) built its
`Devices::MotorConfig` from a blank `MotorConfig{}` merging ONLY `velGains`
— per a 113-002-era comment claiming the other fields "have no live
behavioral effect through this call." That comment was invalidated by
114-001 Revision 1, which made `MotorArmor::reconfigure()` forward the
WHOLE config to the wrapped `NezhaMotor`. From that revision on, every
un-merged field — `wheelTravelCalib`, `slewRate`, `outputDeadband`,
`velDeadband`, `reversalDwell` — was silently zeroed by the Tier-2 push.

`nezha_motor.cpp` gates the encoder mm decode on
`config_.wheelTravelCalib != 0.0f` (and multiplies raw counts by it), so a
zeroed calibration makes `position()`/`velocity()` read 0 forever:
`Telemetry.enc`/`vel`/`pose` flat, measured velocity 0 → the velocity PID
winds up against full error → duty saturates → the plant (ground truth +
OtosPlant) runs away — the exact "otos moves realistically / accelerates
past the commanded speed while enc stays (0,0)" signature (e.g. otos v_x
peaking ~385mm/s on a 200mm/s command).

## Why it regressed TODAY (not pre-existing)

The clobber only fires when Tier 2 is applied AFTER Tier 1's ConfigDelta.
`configure_from_robot()` runs Tier 1 first, Tier 2 second — but before
`9d9ac074` (2026-07-22), `set_config()`'s Tier-1 push returned without
waiting (Sim's `SimConfigConn.send_envelope()` fires and returns
immediately), so in the GUI/tick-thread path Tier 2's direct ctypes call
landed within microseconds — BEFORE the tick thread's next 50ms cycle
processed the Tier-1 envelopes. Effective application order was Tier 2 →
Tier 1, and Tier 1 healed the clobber one cycle later. `9d9ac074` fixed
`set_config_binary()` to fire-and-poll the ack ring — each Tier-1 envelope
is now fully APPLIED (acked) before `set_config()` returns — flipping the
effective order to Tier 1 → Tier 2 for the first time, exposing the latent
clobber. A/B proof: the same repro (SimLoop + tick thread + configure +
twist) MOVES on a worktree build of `19553d64` (v0.20260722.3, the
morning's code) and stays flat on pre-fix HEAD.

This also explains the original session's misleading A/B: monkeypatching
`set_config_binary` to return `None` is NOT the old behavior — the old
path still fired the envelopes (then returned None); the patched one sent
nothing, so Tier-1 config was never applied, the twist was rejected by the
fail-closed NOT_CONFIGURED gate or ran unconfigured, and the test failed
either way — "fails identically" did not rule the config path out.

## Fix

- `src/firm/devices/nezha_motor.h`: new `config()` full live-config
  readback (whole-config counterpart of `gains()`).
- `src/sim/sim_ctypes.cpp` `sim_configure_motor()`: merge starts from
  `motor.config()` and overwrites only port/velFiltAlpha/fwdSign —
  restoring the function's own documented "cannot clobber what Tier 1
  already pushed" contract for every field. Tier order no longer matters.

Verified: the 5 tests below all pass; headless
`SimTransport.run_unmanaged(+700mm)` (the stakeholder's exact repro) ends
with enc=(706,705)/pose=(706,0)/otos=706; managed D/RT and Tour 1 drive
the sim; full suite 1302 passed, 1 skipped, 9 xfailed, 2 xpassed.

## Original description (2026-07-22, for the record)

`uv run python -m pytest` (full suite) failed 5 Sim-mode tests, all with
the same signature: a `SimTransport`/`SimLoop.twist()` (or the equivalent
managed-move path) command is sent, `Telemetry.cmd_vel` correctly shows
the commanded target, `Telemetry.mode` correctly flips `I`->`V`->`I`, and
`Telemetry.otos`/`otos_reading` show a REALISTIC accelerate-then-decay
curve — but `Telemetry.enc`/`vel`/`pose` stay EXACTLY `(0, 0)` / `(0, 0)`
/ `(0, 0, 0)` for the entire run.

Failing tests (all pass post-fix):
- `src/tests/testgui/test_transport.py::test_drive_produces_moving_telemetry`
- `src/tests/testgui/test_traces.py::test_encoder_trace_grows_with_forward_drive_via_dead_reckoning`
- `src/tests/testgui/test_traces.py::test_camera_trace_grows_in_step_with_ground_truth`
- `src/tests/testgui/test_error_divergence.py::test_enc_scale_err_separates_encoder_trace_from_camera_truth`
- `src/tests/testgui/test_sim_transport_tour1.py::test_tour_1_runs_to_completion_with_finite_small_closure`

## Original claims, corrected

- "NOT caused by that work / pre-existing" — WRONG. It was a same-day
  regression: latent since 114-001 Revision 1, EXPOSED by `9d9ac074`'s
  `set_config_binary()` fire-and-poll fix (see "Why it regressed TODAY").
- "Not the `set_config_binary()` fire-and-poll fix … monkeypatched … still
  fails identically" — the monkeypatch was not behavior-equivalent to the
  old code (see above); the conclusion drawn from it was invalid.
- "Suspected area: `wheel_plant.cpp` vs `otos_plant.cpp`" — the plants were
  both fine; the zeroed state lived in `NezhaMotor::config_` inside the
  firmware graph, written there by the Tier-2 ctypes surface.
