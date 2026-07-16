"""src/tests/testgui/test_encoder_dead_reckoning.py — 097 (this ticket, Goal 3):
host-side encoder dead reckoning (``traces.py``'s ``EncoderDeadReckoner``)
and its wiring into ``TraceModel.feed()``/``encoder_yaw``.

Qt-free — no QApplication, no sim lib, no PySide6 required. Binary
telemetry carries no ``encpose`` field at all (096-001's permanent trim,
see ``protocol.py``'s ``TLMFrame.from_pb2()`` docstring), so the canvas
avatar would never move (pinned at the anchor) without this host-side
fallback — this is the piece that makes "run a tour in Sim mode and SEE the
avatar drive the tour" possible before sprint 098 wires a live fused pose.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_encoder_dead_reckoning.py -q
"""

from __future__ import annotations

import math

import pytest

from robot_radio.robot.protocol import TLMFrame
from robot_radio.testgui.traces import EncoderDeadReckoner, TraceModel


def _frame(enc=None, encpose=None, otos=None, pose=None) -> TLMFrame:
    return TLMFrame(t=0, enc=enc, encpose=encpose, otos=otos, pose=pose)


# ---------------------------------------------------------------------------
# EncoderDeadReckoner — pure integrator
# ---------------------------------------------------------------------------


def test_first_update_returns_zero_and_only_caches_the_baseline():
    dr = EncoderDeadReckoner(trackwidth=128.0)
    assert dr.update(500.0, 500.0) == (0, 0, 0)


def test_straight_drive_moves_forward_in_x_no_heading_change():
    dr = EncoderDeadReckoner(trackwidth=128.0)
    dr.update(0.0, 0.0)  # baseline
    x, y, h_cdeg = dr.update(100.0, 100.0)
    assert x == pytest.approx(100.0, abs=0.5)
    assert y == pytest.approx(0.0, abs=0.5)
    assert h_cdeg == 0


def test_pure_rotation_in_place_changes_heading_not_position():
    """dL == -dR (equal and opposite wheel travel) is a pure in-place
    pivot: d = (dL+dR)/2 = 0, so x/y do not move even though theta does."""
    trackwidth = 128.0
    dr = EncoderDeadReckoner(trackwidth=trackwidth)
    dr.update(0.0, 0.0)  # baseline
    d_side = 10.0
    x, y, h_cdeg = dr.update(-d_side, d_side)
    assert x == pytest.approx(0.0, abs=0.5)
    assert y == pytest.approx(0.0, abs=0.5)
    expected_dtheta_deg = math.degrees((2.0 * d_side) / trackwidth)
    assert h_cdeg / 100.0 == pytest.approx(expected_dtheta_deg, abs=0.1)


def test_reset_zeroes_pose_and_forgets_the_previous_reading():
    dr = EncoderDeadReckoner(trackwidth=128.0)
    dr.update(0.0, 0.0)
    dr.update(200.0, 200.0)
    dr.reset()
    # Post-reset, the next call is a fresh baseline again (returns zero).
    assert dr.update(9000.0, 9000.0) == (0, 0, 0)
    # And the step after THAT integrates from the new baseline, not the
    # pre-reset accumulated pose.
    x, y, _h = dr.update(9100.0, 9100.0)
    assert x == pytest.approx(100.0, abs=0.5)


def test_set_trackwidth_affects_subsequent_integration_only():
    dr = EncoderDeadReckoner(trackwidth=100.0)
    dr.update(0.0, 0.0)
    dr.set_trackwidth(200.0)
    d_side = 10.0
    _x, _y, h_cdeg = dr.update(-d_side, d_side)
    expected_dtheta_deg = math.degrees((2.0 * d_side) / 200.0)
    assert h_cdeg / 100.0 == pytest.approx(expected_dtheta_deg, abs=0.1)


# ---------------------------------------------------------------------------
# TraceModel.feed() — dead-reckoning fallback wiring
# ---------------------------------------------------------------------------


def test_encoder_trace_grows_from_enc_when_no_encpose_present():
    model = TraceModel(trackwidth=128.0)
    model.feed(_frame(enc=(0.0, 0.0)))
    model.feed(_frame(enc=(100.0, 100.0)))
    model.feed(_frame(enc=(200.0, 200.0)))

    assert len(model.encoder) == 3
    first_x, first_y = model.encoder[0]
    last_x, last_y = model.encoder[-1]
    assert abs(first_x) < 1.0 and abs(first_y) < 1.0
    assert last_x > first_x + 5.0  # moved forward, in cm


def test_real_encpose_takes_priority_over_dead_reckoning_fallback():
    """If a future firmware build ever adds encpose back to the wire, it
    must win over the host-side dead-reckoning fallback -- never silently
    overridden."""
    model = TraceModel(trackwidth=128.0)
    model.feed(_frame(enc=(0.0, 0.0), encpose=(0, 0, 0)))
    # enc says "moved 1000mm forward"; encpose (the authoritative firmware
    # value, if present) says "didn't move" -- encpose must win.
    model.feed(_frame(enc=(1000.0, 1000.0), encpose=(0, 0, 0)))

    last_x, last_y = model.encoder[-1]
    assert last_x == pytest.approx(0.0, abs=0.5)
    assert last_y == pytest.approx(0.0, abs=0.5)


def test_no_enc_and_no_encpose_leaves_encoder_trace_untouched():
    model = TraceModel(trackwidth=128.0)
    model.feed(_frame(enc=None, encpose=None, pose=(0, 0, 0)))
    assert model.encoder == []


def test_encoder_yaw_tracks_dead_reckoned_heading():
    model = TraceModel(trackwidth=128.0)
    model.anchor(0.0, 0.0, 0.0)
    model.feed(_frame(enc=(0.0, 0.0)))
    assert model.encoder_yaw == pytest.approx(0.0)

    d_side = 10.0
    model.feed(_frame(enc=(-d_side, d_side)))
    expected_dtheta = (2.0 * d_side) / 128.0
    assert model.encoder_yaw == pytest.approx(expected_dtheta, abs=0.01)


def test_anchor_and_clear_reset_the_dead_reckoning_integrator():
    """A 'Set Robot @ 0,0' reset (anchor() + clear()) must re-zero the
    host-side dead-reckoning pose, not just the display baseline -- else a
    tour restarting mid-session would inherit stale accumulated drift."""
    model = TraceModel(trackwidth=128.0)
    model.feed(_frame(enc=(0.0, 0.0)))
    model.feed(_frame(enc=(500.0, 500.0)))
    assert len(model.encoder) == 2
    far_x, _far_y = model.encoder[-1]
    assert far_x > 10.0

    model.anchor(0.0, 0.0, 0.0)
    model.clear()
    assert model.encoder == []
    assert model.encoder_yaw is None

    # Next feed() re-baselines from wherever the (already-moved) firmware
    # counters currently sit -- same "first reading after reset is zero"
    # contract as every other trace.
    model.feed(_frame(enc=(500.0, 500.0)))
    first_x, first_y = model.encoder[0]
    assert abs(first_x) < 1.0 and abs(first_y) < 1.0


def test_set_trackwidth_updates_the_model_owned_dead_reckoner():
    model = TraceModel(trackwidth=100.0)
    model.set_trackwidth(200.0)
    model.feed(_frame(enc=(0.0, 0.0)))
    d_side = 10.0
    model.feed(_frame(enc=(-d_side, d_side)))
    expected_dtheta = (2.0 * d_side) / 200.0
    assert model.encoder_yaw == pytest.approx(expected_dtheta, abs=0.01)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
