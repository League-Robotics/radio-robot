"""src/tests/unit/test_planner_heading.py -- 106-005 (SUC-028/SUC-029).

Covers `robot_radio.planner.heading.HeadingCorrector`: `otos_untrusted`
pose-source selection, the cdeg->rad conversion, the live-tunable output
clamp, degraded-feedback handling (no silent crash/None-propagation), and
gain/clamp live-tunability. No I/O, no hardware, no sim -- a plain
`SimpleNamespace` stands in for `RobotConfig` (duck-typed, per
`HeadingCorrector.__init__()`'s own docstring), and `TLMFrame` instances
are built directly.

Collected under `src/tests/unit/` per `pyproject.toml`'s `testpaths`.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from robot_radio.planner.heading import HeadingCorrector
from robot_radio.planner.model import PlannerParams
from robot_radio.robot.protocol import TLMFrame


def _robot_config(otos_untrusted: bool):
    return SimpleNamespace(geometry=SimpleNamespace(otos_untrusted=otos_untrusted))


# ---------------------------------------------------------------------------
# otos_untrusted source selection
# ---------------------------------------------------------------------------


def test_otos_untrusted_true_selects_encoder_derived_pose():
    corrector = HeadingCorrector(PlannerParams(), robot_config=_robot_config(True))
    assert corrector.source == "pose"


def test_otos_untrusted_false_selects_otos():
    corrector = HeadingCorrector(PlannerParams(), robot_config=_robot_config(False))
    assert corrector.source == "otos"


def test_no_robot_config_defaults_to_otos():
    """Matches GeometryConfig.otos_untrusted's own field default (False)."""
    corrector = HeadingCorrector(PlannerParams(), robot_config=None)
    assert corrector.source == "otos"


def test_measured_heading_reads_selected_source_only():
    frame = TLMFrame(pose=(0, 0, 9000), otos=(0, 0, 4500))  # cdeg

    pose_corrector = HeadingCorrector(PlannerParams(), robot_config=_robot_config(True))
    otos_corrector = HeadingCorrector(PlannerParams(), robot_config=_robot_config(False))

    assert pose_corrector.measured_heading(frame) == pytest.approx(math.pi / 2.0)
    assert otos_corrector.measured_heading(frame) == pytest.approx(math.pi / 4.0)


def test_measured_heading_none_when_frame_none():
    corrector = HeadingCorrector(PlannerParams(), robot_config=_robot_config(True))
    assert corrector.measured_heading(None) is None


def test_measured_heading_none_when_selected_field_absent():
    frame = TLMFrame(otos=(0, 0, 4500))  # no pose
    corrector = HeadingCorrector(PlannerParams(), robot_config=_robot_config(True))
    assert corrector.measured_heading(frame) is None


# ---------------------------------------------------------------------------
# update() -- degraded feedback
# ---------------------------------------------------------------------------


def test_update_returns_zero_trim_and_logs_when_no_measurement(caplog):
    corrector = HeadingCorrector(PlannerParams(), robot_config=_robot_config(True))
    with caplog.at_level("WARNING"):
        trim = corrector.update(commanded_heading=0.5, frame=None, now=0.0)
    assert trim == 0.0
    assert any("no measured heading" in r.message for r in caplog.records)


# ---------------------------------------------------------------------------
# Output clamp
# ---------------------------------------------------------------------------


def test_large_positive_error_clamps_to_positive_ceiling():
    params = PlannerParams(heading_kp=100.0, heading_omega_clamp=0.5)
    corrector = HeadingCorrector(params, robot_config=_robot_config(True))
    # Measured heading far below commanded -> large positive error.
    frame = TLMFrame(pose=(0, 0, 0))
    trim = corrector.update(commanded_heading=math.pi, frame=frame, now=0.0)
    assert trim == pytest.approx(0.5)


def test_large_negative_error_clamps_to_negative_ceiling():
    params = PlannerParams(heading_kp=100.0, heading_omega_clamp=0.5)
    corrector = HeadingCorrector(params, robot_config=_robot_config(True))
    frame = TLMFrame(pose=(0, 0, 0))
    # Deliberately NOT exactly -pi: normalize_angle()'s own (-pi, pi] range
    # canonicalizes an error of exactly -pi to +pi (the equivalent angle),
    # which would flip the expected sign here -- this test wants an
    # unambiguous negative error, not the -pi/+pi boundary case.
    trim = corrector.update(commanded_heading=-(math.pi - 0.1), frame=frame, now=0.0)
    assert trim == pytest.approx(-0.5)


@pytest.mark.parametrize("clamp", [0.1, 0.5, 1.5])
def test_clamp_ceiling_is_never_exceeded_for_any_stated_clamp(clamp):
    params = PlannerParams(heading_kp=1000.0, heading_omega_clamp=clamp)
    corrector = HeadingCorrector(params, robot_config=_robot_config(True))
    frame = TLMFrame(pose=(0, 0, 0))
    trim = corrector.update(commanded_heading=math.pi, frame=frame, now=0.0)
    assert abs(trim) <= clamp + 1e-9


# ---------------------------------------------------------------------------
# Live-tunability -- gains/clamp re-synced from params on every update()
# ---------------------------------------------------------------------------


def test_clamp_mutated_after_construction_takes_effect_next_update():
    params = PlannerParams(heading_kp=100.0, heading_omega_clamp=0.5)
    corrector = HeadingCorrector(params, robot_config=_robot_config(True))
    frame = TLMFrame(pose=(0, 0, 0))

    first = corrector.update(commanded_heading=math.pi, frame=frame, now=0.0)
    assert first == pytest.approx(0.5)

    params.heading_omega_clamp = 1.2  # mutate live, no reconstruction
    second = corrector.update(commanded_heading=math.pi, frame=frame, now=1.0)
    assert second == pytest.approx(1.2)


def test_kp_mutated_after_construction_takes_effect_next_update():
    params = PlannerParams(heading_kp=0.0, heading_omega_clamp=10.0)
    corrector = HeadingCorrector(params, robot_config=_robot_config(True))
    frame = TLMFrame(pose=(0, 0, 0))

    zero_gain = corrector.update(commanded_heading=1.0, frame=frame, now=0.0)
    assert zero_gain == pytest.approx(0.0)

    params.heading_kp = 2.0
    nonzero_gain = corrector.update(commanded_heading=1.0, frame=frame, now=1.0)
    assert nonzero_gain != pytest.approx(0.0)


# ---------------------------------------------------------------------------
# reset()
# ---------------------------------------------------------------------------


def test_reset_clears_pid_history():
    params = PlannerParams(heading_kp=1.0, heading_ki=1.0, heading_kd=1.0,
                          heading_omega_clamp=10.0)
    corrector = HeadingCorrector(params, robot_config=_robot_config(True))
    frame = TLMFrame(pose=(0, 0, 0))

    corrector.update(commanded_heading=1.0, frame=frame, now=0.0)
    corrector.update(commanded_heading=1.0, frame=frame, now=1.0)
    assert corrector._pid.prev_time is not None  # accumulated state exists

    corrector.reset()
    assert corrector._pid.prev_time is None
    assert corrector._pid.integral == 0.0


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
