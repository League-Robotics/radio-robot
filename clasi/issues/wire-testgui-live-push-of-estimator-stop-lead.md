---
status: resolved
---

# Wire TestGUI's live connect-time push of estimator.stop_lead_ms (and the other estimator.* fields)

## Description

The turn-prediction campaign (2026-07-22) added `App::MoveQueue`'s own
stop-condition anticipation lead (`move_queue.h`'s `tick()`, config key
`estimator.stop_lead_ms`, wire arm `EstimatorConfigPatch.stop_lead_ms`,
host builder `NezhaProtocol.estimator_config(stop_lead_ms=...)`). It
closes the dominant overshoot mechanism on the managed (Move-queue)
motion path -- verified via `src/tests/testgui/test_tour_closure_gate.py`'s
own sweep against the real firmware: worst per-turn error at
`omega=2rad/s` drops from ~13-23deg (anticipation off) to ~4-7deg
(`stop_lead_ms=90` for the sim fixture robot).

That verification pushes `stop_lead_ms` explicitly, test-locally, via
`NezhaProtocol`/`_SimConfigConn` (mirroring `test_tour_closure_gate.py`'s
own pre-existing `OtosConfigPatch` calibration push). It is NOT wired into
the live TestGUI's own connect-time calibration flow
(`testgui/__main__.py`'s `_push_robot_calibration()`): `robot_radio.
config.robot_config.RobotConfig` has no `estimator` field at all today --
nothing host-side currently reads the robot JSON's `estimator.*` section;
only `gen_boot_config.py` does, at ARM build time. Real hardware is
unaffected (it gets `estimator.stop_lead_ms` from boot config,
unconditionally, on every reboot) -- this gap is Sim-mode-only, and only
for a LIVE TestGUI session (a one-shot script that builds its own
`NezhaProtocol`/`SimLoop` handle, like `test_tour_closure_gate.py` or
`src/tests/bench/turn_prediction_capture.py`, is unaffected -- it can
push the patch itself).

## Why this wasn't closed in the same pass

Wiring this correctly needs, at minimum:
1. A new optional `estimator` section on `RobotConfig` (pydantic model,
   `robot_config.py`) -- `weight_heading_otos`/`weight_omega_otos`/
   `staleness_ms`/`stop_lead_ms`, mirroring `data/robots/robot_config.
   schema.json`'s own `estimator` object.
2. `_push_robot_calibration()` (`testgui/__main__.py`) needs a NEW push
   path -- `EstimatorConfigPatch` is NOT covered by the existing
   `calibration_commands()` text-command list (`calibration/push.py`,
   which never learned an `estimator.*` vocabulary), so this can't reuse
   that mechanism as-is. It also isn't uniform across transports:
   `_HardwareTransport.protocol` is a real `NezhaProtocol` (has
   `estimator_config()` directly); `SimTransport.protocol` is a bare
   `SimLoop` (no `estimator_config()` method at all -- a caller needs to
   wrap it in `NezhaProtocol(_SimConfigConn(loop))`, the SAME construction
   `test_tour_closure_gate.py`'s own `_make_loop()` already does
   test-locally).
3. Once wired, `_button_acceptance_support.py`'s own tolerance-model
   docstring flags `MANAGED_ANGLE_ABS_MARGIN_DEG`/`MANAGED_DIST_ABS_MARGIN_MM`
   (`test_gui_button_acceptance.py`) as the two constants to retighten
   against freshly re-measured numbers once the live push is real -- do
   NOT tighten them before the push exists (see that docstring's own
   caution, added by this same campaign).

Bounded, real work across a config schema, a large/sensitive GUI connect
flow, and a slow (~1-2 minute) acceptance suite -- deliberately deferred
rather than rushed under this campaign's own already-large scope.

## Suggested approach

Smallest coherent path: add the `RobotConfig.estimator` field (all
sub-fields optional, so existing robot JSONs without the section --
should not occur post-117, but defensively -- don't break), then extend
`_push_robot_calibration()` with a small `_push_estimator_config()` helper
dispatched by transport type (real `NezhaProtocol` vs a
`NezhaProtocol(_SimConfigConn(sim_loop))` wrapper for `SimTransport`),
called unconditionally alongside the existing OTOS/Tier-2 pushes. Re-run
`test_gui_button_acceptance.py` (slow) to get fresh MANAGED_* numbers,
then retighten `_button_acceptance_support.py`'s two constants against
them.

## Related

- `src/tests/testgui/test_tour_closure_gate.py` -- the sim sweep that
  measured the ~4-7deg post-fix numbers this issue's own eventual
  retightening should match against (once the live-push path exists,
  re-measure THROUGH the GUI, not just through the direct protocol push,
  in case the two paths diverge for some other reason).
- `src/firm/app/move_queue.h` -- the anticipation-lead mechanism itself
  (`tick()`'s own doc comment).
- `src/tests/notebooks/turn_prediction.ipynb` -- Phase A's own prediction-
  quality notebook this campaign's firmware fix is built on.

## Resolution (2026-07-22, OOP defect remediation)

Closed via the "smallest coherent path" this issue's own Suggested
Approach outlined, plus the sim-parity/acceptance-tightening work the
stakeholder's follow-up complaint ("you're running 1,300 tests and not
testing the thing I want: the tour to look good") demanded:

1. **`RobotConfig.estimator`** (new `EstimatorConfig` pydantic model,
   `src/host/robot_radio/config/robot_config.py`) -- `weight_heading_otos`/
   `weight_omega_otos`/`staleness_ms`/`stop_lead_ms`, mirroring
   `data/robots/robot_config.schema.json`'s own `estimator` object.
2. **`push.estimator_kwargs(config)`** (`src/host/robot_radio/calibration/push.py`)
   -- pure field-selection function reading BOTH `config.estimator.*` and
   `config.control.*` (the shaper limits `a_max`/`a_decel`/`alpha_max`/
   `alpha_decel`/`j_max`/`yaw_jerk_max`, which ride the SAME
   `EstimatorConfigPatch` wire arm per `NezhaProtocol.estimator_config()`'s
   own doc comment).
3. **`__main__.py`'s `_push_estimator_config(transport, cfg)`** -- new
   helper, called unconditionally from `_push_robot_calibration()`
   (connect + every robot-select while connected), for BOTH transports:
   `_HardwareTransport.protocol` (a real `NezhaProtocol`, `wait_for_ack()`
   for the ack) and `SimTransport._config_proto`/`._config_conn` (the SAME
   `NezhaProtocol`/`_SimConfigConn` pair the Tier-1 SET/GET path and
   `_handle_otos_patch()` already share -- no second, redundant wrapper
   constructed). Every push logs an explicit applied/rejected/timed-out
   outcome (`[CAL] pushed N/N estimator/shaper fields ...` or the
   REJECTED/TIMED OUT variant).
4. **Sim-parity verification**: `src/tests/testgui/test_calibration_push_on_connect.py::test_connect_pushes_estimator_config_and_acks_cleanly`
   asserts a live Sim Connect click reports a clean 10/10 apply (no
   readback getter exists -- see this issue's own history -- so ack
   success + downstream behavioral verification, below, stand in for it,
   per this issue's own "read-back if a getter exists; otherwise assert
   behaviorally via the turn landing" fallback).
5. **Acceptance bands tightened** (`test_gui_button_acceptance.py`,
   `_button_acceptance_support.py`): managed +/-90deg presets and `SEG 0
   9000` land within +/-3deg (`MANAGED_ANGLE_90_ABS_MARGIN_DEG`); Tour 1/
   Tour 2 per-leg turn error is captured via `TourLegCapture` (the SAME
   `TurnCheck` instrumentation `test_tour_closure_gate.py`'s own closure
   gate uses, wrapped around the REAL button-driven `_TourRunner`) and
   bounded by `TOUR_TURN_ERROR_MAX_DEG` (5.0deg -- see that constant's own
   comment for why 2.5deg, though achieved by `test_tour_closure_gate.py`'s
   deterministic harness, measured flaky ~30% of the time through the REAL
   threaded GUI tour path, and why 5.0deg still fails hard on a genuine
   connect-push regression). Both bands are the user-visible quality bar
   this fix exists to hold the line on -- widening either requires
   stakeholder sign-off (comment on each constant; also noted in
   `src/tests/DESIGN.md`'s Constraints section).

Real hardware needed no wiring change (boot config always bakes these
fields in on reflash) -- this issue was Sim-mode-only, exactly as
originally scoped.
