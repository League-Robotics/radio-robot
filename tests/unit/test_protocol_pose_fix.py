"""tests/unit/test_protocol_pose_fix.py -- 099-009 (SUC-005).

Covers this ticket's host-lib addition to
``host/robot_radio/robot/protocol.py``: ``build_pose_fix_envelope()`` (pure
``CommandEnvelope{pose_fix: PoseFix{...}}`` construction, arm 7) and
``NezhaProtocol.pose_fix()`` (the method
``tests/playfield/pose_fix_convergence.py`` sends the delayed camera fix
through). No hardware/robot/serial port involved anywhere in this file --
pure protobuf construction plus a serialize/parse round trip through
``pb2`` (host/robot_radio/robot/pb2/), the same "host-side codec" check
``tests/sim/unit/test_wire_differential.py``'s own docstring names for this
class of test.

Firmware-side behavior for the ``pose_fix`` arm (BinaryChannel::
handlePose(), PoseEstimator's ring/interpolate/compose/ungated-EKF-update)
is already covered by ``tests/sim/unit/test_pose_fix_reset_zero.py`` and
``tests/sim/unit/test_pose_fix_end_to_end.py`` (tickets 099-004/099-008) --
this file stays scoped to the NEW host-side envelope builder this ticket
adds, not a re-test of firmware dispatch.

Collected under ``tests/unit/`` (host-side unit/tooling check, not
sim/bench/playfield-scoped -- see ``tests/CLAUDE.md``); ``pyproject.toml``'s
``testpaths`` includes ``tests/unit`` so ``uv run python -m pytest`` collects
it.
"""

from __future__ import annotations

import math

import pytest

from robot_radio.robot.pb2 import envelope_pb2
from robot_radio.robot.protocol import NezhaProtocol, build_pose_fix_envelope


# ---------------------------------------------------------------------------
# build_pose_fix_envelope() -- pure construction
# ---------------------------------------------------------------------------

class TestBuildPoseFixEnvelope:
    def test_selects_pose_fix_cmd_arm(self) -> None:
        env = build_pose_fix_envelope(100.0, -200.0, 0.5, 12345)
        assert env.WhichOneof("cmd") == "pose_fix"

    def test_fields_carried_verbatim(self) -> None:
        env = build_pose_fix_envelope(842.5, -317.0, 0.75, 99000)
        assert env.pose_fix.x == pytest.approx(842.5)
        assert env.pose_fix.y == pytest.approx(-317.0)
        assert env.pose_fix.h == pytest.approx(0.75)
        assert env.pose_fix.t == 99000

    def test_default_flags_both_false(self) -> None:
        env = build_pose_fix_envelope(1.0, 2.0, 3.0, 4)
        assert env.pose_fix.reset is False
        assert env.pose_fix.zero_encoders is False

    def test_reset_flag_set(self) -> None:
        env = build_pose_fix_envelope(1.0, 2.0, 3.0, 4, reset=True)
        assert env.pose_fix.reset is True
        assert env.pose_fix.zero_encoders is False

    def test_zero_encoders_flag_set(self) -> None:
        env = build_pose_fix_envelope(1.0, 2.0, 3.0, 4, zero_encoders=True)
        assert env.pose_fix.reset is False
        assert env.pose_fix.zero_encoders is True

    def test_both_flags_set(self) -> None:
        env = build_pose_fix_envelope(1.0, 2.0, 3.0, 4, reset=True, zero_encoders=True)
        assert env.pose_fix.reset is True
        assert env.pose_fix.zero_encoders is True

    def test_negative_and_zero_values_round_trip(self) -> None:
        env = build_pose_fix_envelope(0.0, 0.0, -math.pi, 0)
        assert env.pose_fix.x == pytest.approx(0.0)
        assert env.pose_fix.y == pytest.approx(0.0)
        assert env.pose_fix.h == pytest.approx(-math.pi)
        assert env.pose_fix.t == 0


# ---------------------------------------------------------------------------
# Serialize/parse round trip -- confirms the wire bytes carry every field
# (host-side codec check, mirroring test_wire_differential.py's own posture)
# ---------------------------------------------------------------------------

class TestWireRoundTrip:
    def test_serialize_parse_round_trip(self) -> None:
        original = build_pose_fix_envelope(123.25, -45.5, 1.2345, 654321,
                                           reset=False, zero_encoders=False)
        wire_bytes = original.SerializeToString()

        parsed = envelope_pb2.CommandEnvelope()
        parsed.ParseFromString(wire_bytes)

        assert parsed.WhichOneof("cmd") == "pose_fix"
        assert parsed.pose_fix.x == pytest.approx(123.25)
        assert parsed.pose_fix.y == pytest.approx(-45.5)
        assert parsed.pose_fix.h == pytest.approx(1.2345, rel=1e-5)
        assert parsed.pose_fix.t == 654321
        assert parsed.pose_fix.reset is False
        assert parsed.pose_fix.zero_encoders is False

    def test_round_trip_preserves_reset_flag(self) -> None:
        original = build_pose_fix_envelope(0.0, 0.0, 0.0, 0, reset=True)
        parsed = envelope_pb2.CommandEnvelope()
        parsed.ParseFromString(original.SerializeToString())
        assert parsed.WhichOneof("cmd") == "pose_fix"
        assert parsed.pose_fix.reset is True


# ---------------------------------------------------------------------------
# NezhaProtocol.pose_fix() -- confirms the method builds the SAME envelope
# build_pose_fix_envelope() does, without sending anything (no connection).
# ---------------------------------------------------------------------------

class TestNezhaProtocolPoseFixEnvelopeConstruction:
    """``pose_fix()`` delegates envelope construction to
    ``build_pose_fix_envelope()`` and hands it to ``_send_envelope()`` --
    verified here by capturing the envelope a stub ``_send_envelope``
    receives, with no real ``SerialConnection``/hardware anywhere."""

    def test_pose_fix_method_builds_expected_envelope(self) -> None:
        captured: list[envelope_pb2.CommandEnvelope] = []

        proto = NezhaProtocol.__new__(NezhaProtocol)  # bypass __init__ (no conn needed)
        proto._send_envelope = lambda envelope, read_timeout=500: (  # type: ignore[method-assign]
            captured.append(envelope) or None)

        proto.pose_fix(10.0, 20.0, 0.1, 5000)

        assert len(captured) == 1
        sent = captured[0]
        assert sent.WhichOneof("cmd") == "pose_fix"
        assert sent.pose_fix.x == pytest.approx(10.0)
        assert sent.pose_fix.y == pytest.approx(20.0)
        assert sent.pose_fix.h == pytest.approx(0.1)
        assert sent.pose_fix.t == 5000
        assert sent.pose_fix.reset is False
        assert sent.pose_fix.zero_encoders is False

    def test_pose_fix_method_passes_flags_through(self) -> None:
        captured: list[envelope_pb2.CommandEnvelope] = []

        proto = NezhaProtocol.__new__(NezhaProtocol)
        proto._send_envelope = lambda envelope, read_timeout=500: (  # type: ignore[method-assign]
            captured.append(envelope) or None)

        proto.pose_fix(0.0, 0.0, 0.0, 0, reset=True, zero_encoders=True)

        assert captured[0].pose_fix.reset is True
        assert captured[0].pose_fix.zero_encoders is True
