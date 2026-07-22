---
status: pending
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
