"""src/tests/sim/system/test_straight_leg_crab_regression.py -- 119 ticket
005's own permanent regression test: the exact scenario
``docs/code_review/2026-07-22-turn-execution-review-scripts/straight_drift_repro.py``
used to root-cause and confirm the straight-leg-crab fix (see
``clasi/issues/straight-leg-crab-118-001-actuation-and-telemetry-pairing-skew.md``
and ``docs/code_review/2026-07-22-turn-execution-review.md`` §9).

118-001's schedule restore (commit ``3189086f``) introduced two coupled
defects in ``src/firm/app/robot_loop.cpp``'s ``RobotLoop::cycle()``:

- **A -- one-cycle L/R actuation skew.** ``drive_.tick()`` sat BETWEEN
  ``motorL_.tick()`` and ``motorR_.tick()``, so L always wrote duty from a
  target staged one cycle older than R's. During any commanded ramp this
  produced a real, physical yaw transient (measured +2.685deg cruise
  heading on a straight leg before the fix) -- decel restored it (final
  heading 0.00deg), so the net signature was LATERAL DISPLACEMENT with
  ZERO final heading error (measured +32.5mm crab over a 700mm straight).
- **B -- telemetry pairs fresh L with stale R.** ``updateTlm()``/``emit()``
  ran between L's own collect and R's, so every outbound frame paired
  THIS cycle's L against LAST cycle's R -- a pairing skew that numerically
  CANCELED the physical skew from (A) (measured host-visible
  ``dL - dR == +0.00`` on every single frame), hiding the crab from every
  host-visible encoder view (``encpose``, ``frame.twist``) even though the
  firmware's own ``pose`` (odometry, computed directly from live motor
  state, not the TLM-paired fields) and OTOS/truth all agreed the robot
  crabbed.

119 ticket 005 restores same-generation actuation staging (``drive_.tick()``
hoisted above BOTH motor selects, restoring the one genuinely good half of
the retired 112-005 hoist) and same-generation telemetry pairing
(``updateTlm()``/``emit()`` moved to the start of the trailing pace block,
after BOTH collects) -- see ``robot_loop.cpp``'s own comments at each call
site for the full mechanism.

This is exactly why an ENDPOINT-ONLY check (final heading, or an
encoder-derived trace) is provably blind to this failure shape: 118-001's
own bug measured final heading error 0.00deg and a perfectly flat
``dL - dR`` trace while the robot's TRUE path crabbed the entire time. This
test asserts truth (``SimPlant`` ground truth, bypassing every sensor/host
path) DURING CRUISE, not just at the end -- see
``test_tour_closure_gate.py``'s own ``StraightLegCruiseCheck``/
``_assert_tour_gate(cruise_heading_tolerance_deg=...)`` for the same
discipline applied to every straight leg of a full tour.

Run with::

    uv run python -m pytest src/tests/sim/system/test_straight_leg_crab_regression.py -v -s

Requires the compiled ``src/sim/build/libfirmware_host.{dylib,so}``
(``python build.py`` or ``cmake --build src/sim/build``) -- skips cleanly if
not present.
"""
from __future__ import annotations

import math
import pathlib
import sys

import pytest

# src/tests/sim/system/test_straight_leg_crab_regression.py -> system -> sim
# -> tests -> src -> repo root (mirrors test_sim_configure_from_robot.py's
# own four-hop convention).
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_SIM_LIB_PATH = _REPO_ROOT / "src" / "sim" / "build" / _LIB_NAME

pytestmark = pytest.mark.skipif(
    not _SIM_LIB_PATH.exists(),
    reason="sim lib not built -- cmake --build src/sim/build (or `python build.py`)",
)

_TRACK_WIDTH = 128.0  # [mm] matches straight_drift_repro.py's own SimLoop construction

# straight_drift_repro.py's own exact scenario: a 700mm straight leg at
# 150mm/s (this project's own tour cruise speed, DEFAULT_V_MAX), ideal chip
# (every sim fidelity knob explicit at its documented no-op default -- no
# injected sensor/encoder error, isolating the actuation-skew mechanism
# itself, matching test_tour_closure_gate.py's own "ideal chip" posture).
_V_X = 150.0            # [mm/s]
_STOP_DISTANCE = 700.0  # [mm]
_TIMEOUT_MS = 15000.0

# 118-001's own measured baseline (before this ticket's fix, for the
# record): cruise heading +2.685deg, final y +32.5mm, final heading
# 0.00deg. Post-fix (measured against this exact scenario while writing
# this ticket): cruise heading 0.000deg for the ENTIRE run (not merely
# "a few tenths of a degree" -- an isolated, from-rest straight leg has no
# chain-advance history to inherit any residual from, so the fix's own
# contribution measures as an exact zero here), final y +0.0mm.
#
# Tolerances still carry real margin over that measured exact-zero result
# (not asserting float equality to 0.0) -- a few tenths of a degree /
# a few mm, per the straight-leg-crab issue's own acceptance wording, in
# case of legitimate float32 accumulation noise on a different platform's
# libm.
_CRUISE_HEADING_TOLERANCE_DEG = 0.3   # "a few tenths of a degree"
_FINAL_Y_TOLERANCE_MM = 3.0           # "a few mm"


def _make_loop():
    """A bare, headless ``SimLoop`` -- deterministic manual stepping
    (``start_tick_thread=False``), mirrors
    ``test_sim_configure_from_robot.py``'s own ``_make_loop()`` convention
    and ``straight_drift_repro.py``'s own construction exactly."""
    from robot_radio.io.sim_loop import SimLoop

    loop = SimLoop(track_width=_TRACK_WIDTH, lib_path=_SIM_LIB_PATH)
    loop.connect(start_tick_thread=False)
    return loop


def test_straight_700mm_leg_at_150mms_ideal_chip_does_not_crab():
    """Permanent regression test for 118-001's straight-leg crab (119
    ticket 005) -- straight_drift_repro.py's own exact scenario, with
    assertions:

    - truth heading, sampled EVERY cycle for the leg's full duration
      (accel+cruise+decel, a superset of "cruise"), stays within
      ``_CRUISE_HEADING_TOLERANCE_DEG`` of zero -- NOT an endpoint-only
      check (118-001's own bug had 0.00deg final heading error while
      crabbing the entire time; this is the exact class of check the
      straight-leg-crab issue's own "Gate addition" section calls for).
    - truth |y| at the end of the leg stays within
      ``_FINAL_Y_TOLERANCE_MM`` of zero (was +32.5mm before this fix).
    """
    from robot_radio.config.robot_config import load_robot_config

    loop = _make_loop()
    try:
        loop.configure_from_robot(load_robot_config(_ROBOTS_DIR / "tovez_nocal.json"))

        # Ideal chip: every sim fidelity knob explicit at its documented
        # no-op default -- isolates the actuation-skew mechanism from any
        # sensor/encoder noise (matches straight_drift_repro.py's own setup
        # and test_tour_closure_gate.py's own "ideal chip" convention).
        loop.set_otos_raw_scale_err(0.0, 0.0)
        for port in (1, 2):
            loop.set_enc_scale_err(port, 0.0)
            loop.set_enc_tick_quant(port, 0.0)
            loop.set_enc_slip(port, 0.0, 0.0)

        loop.step(5)
        loop.drain_pending_tlm()
        t0 = loop.get_true_pose()

        loop.move(v_x=_V_X, stop_distance=_STOP_DISTANCE, timeout=_TIMEOUT_MS,
                  replace=True, id=7)

        max_abs_heading_deg = 0.0
        move_done_cycle: int | None = None
        for i in range(300):
            loop.step(1)
            for frame in loop.drain_pending_tlm():
                if frame.ack is not None and frame.ack.corr_id == 7:
                    move_done_cycle = i

            tp = loop.get_true_pose()
            heading_deg = abs(math.degrees(tp["h"] - t0["h"]))
            max_abs_heading_deg = max(max_abs_heading_deg, heading_deg)

            # A bounded settle window past the completion ack, mirroring
            # straight_drift_repro.py's own "done + 15 cycles" bound --
            # gives the decel taper time to finish before reading the
            # final pose below.
            if move_done_cycle is not None and i >= move_done_cycle + 15:
                break

        assert move_done_cycle is not None, (
            "the MOVE never completed within the bounded cycle budget -- "
            "cannot measure the final pose"
        )

        tp = loop.get_true_pose()
        final_x = tp["x"] - t0["x"]
        final_y = tp["y"] - t0["y"]
        final_heading_deg = math.degrees(tp["h"] - t0["h"])

        print(f"\nSTRAIGHT-LEG-CRAB REGRESSION: max|cruise heading|={max_abs_heading_deg:.4f}deg "
              f"(tolerance {_CRUISE_HEADING_TOLERANCE_DEG}deg); "
              f"final x={final_x:+.1f}mm y={final_y:+.2f}mm "
              f"(tolerance |y|<{_FINAL_Y_TOLERANCE_MM}mm) "
              f"heading={final_heading_deg:+.3f}deg")

        assert max_abs_heading_deg < _CRUISE_HEADING_TOLERANCE_DEG, (
            f"truth heading drifted {max_abs_heading_deg:.4f}deg at some point during the leg "
            f"(tolerance {_CRUISE_HEADING_TOLERANCE_DEG}deg) -- 118-001's own actuation-skew bug "
            f"measured +2.685deg here with a MISLEADING 0.00deg final heading error (endpoint-only "
            f"checks are provably blind to this failure shape) -- see "
            f"clasi/issues/straight-leg-crab-118-001-actuation-and-telemetry-pairing-skew.md"
        )
        assert abs(final_y) < _FINAL_Y_TOLERANCE_MM, (
            f"truth |y| at the end of a {_STOP_DISTANCE:.0f}mm straight leg is {final_y:+.2f}mm "
            f"(tolerance {_FINAL_Y_TOLERANCE_MM}mm) -- 118-001's own bug measured +32.5mm here"
        )
    finally:
        loop.disconnect()


if __name__ == "__main__":
    # -s: don't capture stdout -- see this file's own header for the
    # standalone invocation.
    sys.exit(pytest.main([__file__, "-v", "-s"]))
