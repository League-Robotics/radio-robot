"""src/tests/testgui/test_hardware_transport_managed_move.py --
testgui-motion-paths-dead-after-move-cutover fix.

Exercises ``_HardwareTransport``'s new ``D``/``RT``/``SEG 0 <cdeg>`` ->
``NezhaProtocol.move_twist()``/``move_wheels()`` dispatch
(``_dispatch_managed_move()``) and its new ``run_unmanaged()`` -- both added
because, pre-fix, ``_HardwareTransport.send()``/``command()`` routed every
line through ``binary_bridge.translate_command()``'s permanent dead stub
(legacy_render/legacy_verbs deleted, see that module's own docstring) and
``_HardwareTransport`` had no ``run_unmanaged()`` at all (the GUI's own
``hasattr(transport, "run_unmanaged")`` guard silently no-op'd it) -- see
``clasi/issues/testgui-motion-paths-dead-after-move-cutover.md``.

Qt-free, sim-lib-free -- constructs a bare ``SerialTransport`` and pokes its
private ``_conn``/``_proto`` attributes directly with a fake connection
double (mirrors ``src/tests/unit/test_protocol_config.py``'s
``_FakeFastConn`` and ``src/tests/testgui/test_binary_bridge.py``'s
``_FakeConn`` -- both already establish this "poke the private attribute,
never open a real port" pattern for testing this transport layer in
isolation), so it runs with no hardware and no compiled sim library.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_hardware_transport_managed_move.py -v
"""
from __future__ import annotations

import math

import pytest

from robot_radio.robot.pb2 import envelope_pb2, telemetry_pb2
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.testgui import binary_bridge
from robot_radio.testgui.transport import SerialTransport, _UNMANAGED_YAW_RATE

# flags bit 5 (ack_fresh) -- telemetry.proto Telemetry.flags, 115-003 frame
# v2. AckEntry.from_telemetry() does not itself gate on this bit (see that
# method's own docstring) -- set here only for realism/parity with
# test_binary_bridge.py's own _ACK_FRESH_BIT usage.
_ACK_FRESH_BIT = 1 << 5


class _FakeHardwareConn:
    """Minimal fake ``SerialConnection``: ``is_open`` (checked by
    ``send()``/``command()``/``run_unmanaged()`` before dispatching at all)
    plus ``send_envelope_fast()``/``wait_for_ack()`` -- the exact two
    methods ``NezhaProtocol.move_twist()``/``move_wheels()``/
    ``wait_for_ack()`` call on ``self._conn``. Mirrors
    ``test_protocol_config.py``'s own ``_FakeFastConn``."""

    def __init__(self) -> None:
        self.is_open = True
        self.sent: list["envelope_pb2.CommandEnvelope"] = []
        self._next_corr_id = 0
        self.ack_result: "telemetry_pb2.Telemetry | None" = None

    def send_envelope_fast(self, envelope: "envelope_pb2.CommandEnvelope") -> int:
        self._next_corr_id += 1
        envelope.corr_id = self._next_corr_id
        self.sent.append(envelope)
        return self._next_corr_id

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "telemetry_pb2.Telemetry | None":
        return self.ack_result


@pytest.fixture
def transport():
    """A ``SerialTransport`` wired to a fake connection -- never opens a
    real port (``connect()`` is never called)."""
    t = SerialTransport("dummy-port")
    t._conn = _FakeHardwareConn()
    t._proto = NezhaProtocol(t._conn)  # type: ignore[arg-type]
    logs: list[str] = []
    t.on_log = logs.append
    t.logs = logs  # type: ignore[attr-defined] -- test-only convenience handle
    return t


def _ok_ack(conn: _FakeHardwareConn) -> None:
    conn.ack_result = telemetry_pb2.Telemetry(flags=_ACK_FRESH_BIT, ack_corr=1, ack_err=0)


# ---------------------------------------------------------------------------
# D -- left == right -> MoveTwist straight drive
# ---------------------------------------------------------------------------


def test_d_equal_left_right_sends_move_twist_with_distance_stop(transport):
    _ok_ack(transport._conn)

    reply = transport.command("D 150 150 300", read_timeout=500)

    assert reply == "OK move", reply
    assert len(transport._conn.sent) == 1
    sent = transport._conn.sent[0]
    assert sent.WhichOneof("cmd") == "move"
    assert sent.move.WhichOneof("velocity") == "twist"
    assert sent.move.twist.v_x == pytest.approx(150.0)
    assert sent.move.twist.omega == pytest.approx(0.0)
    assert sent.move.WhichOneof("stop") == "distance"
    assert sent.move.distance == pytest.approx(300.0)
    assert sent.move.timeout > 0.0
    assert sent.move.replace is True


def test_d_negative_distance_flips_v_x_sign_not_stop_distance(transport):
    _ok_ack(transport._conn)

    transport.command("D 150 150 -300", read_timeout=500)

    sent = transport._conn.sent[0]
    assert sent.move.twist.v_x == pytest.approx(-150.0)
    # The stop condition's own distance field is always the unsigned
    # |path length| threshold (envelope.proto: "|path arc length| since
    # activation") -- direction is carried by the velocity, never the stop.
    assert sent.move.distance == pytest.approx(300.0)


# ---------------------------------------------------------------------------
# D -- left != right -> MoveWheels (sprint 116 Decision 3: never round-
# tripped through a twist)
# ---------------------------------------------------------------------------


def test_d_unequal_left_right_sends_move_wheels(transport):
    _ok_ack(transport._conn)

    reply = transport.command("D 100 200 300", read_timeout=500)

    assert reply == "OK move", reply
    sent = transport._conn.sent[0]
    assert sent.move.WhichOneof("velocity") == "wheels"
    assert sent.move.wheels.v_left == pytest.approx(100.0)
    assert sent.move.wheels.v_right == pytest.approx(200.0)
    assert sent.move.WhichOneof("stop") == "distance"
    assert sent.move.distance == pytest.approx(300.0)


# ---------------------------------------------------------------------------
# RT / SEG 0 <cdeg> -> MoveTwist(omega) with an angle stop
# ---------------------------------------------------------------------------


def test_rt_sends_move_twist_with_angle_stop_at_unmanaged_yaw_rate(transport):
    _ok_ack(transport._conn)

    reply = transport.command("RT 9000", read_timeout=500)

    assert reply == "OK move", reply
    sent = transport._conn.sent[0]
    assert sent.move.WhichOneof("velocity") == "twist"
    assert sent.move.twist.v_x == pytest.approx(0.0)
    assert sent.move.twist.omega == pytest.approx(_UNMANAGED_YAW_RATE)
    assert sent.move.WhichOneof("stop") == "angle"
    assert sent.move.angle == pytest.approx(math.radians(90.0))


def test_rt_negative_cdeg_flips_omega_sign_not_stop_angle(transport):
    _ok_ack(transport._conn)

    transport.command("RT -9000", read_timeout=500)

    sent = transport._conn.sent[0]
    assert sent.move.twist.omega == pytest.approx(-_UNMANAGED_YAW_RATE)
    assert sent.move.angle == pytest.approx(math.radians(90.0))


def test_seg_0_cdeg_is_translated_to_the_same_move_as_rt(transport):
    _ok_ack(transport._conn)

    transport.command("SEG 0 9000", read_timeout=500)

    sent = transport._conn.sent[0]
    assert sent.move.WhichOneof("velocity") == "twist"
    assert sent.move.twist.omega == pytest.approx(_UNMANAGED_YAW_RATE)
    assert sent.move.WhichOneof("stop") == "angle"
    assert sent.move.angle == pytest.approx(math.radians(90.0))


# ---------------------------------------------------------------------------
# Malformed lines -- badarg, no wire traffic
# ---------------------------------------------------------------------------


def test_malformed_d_line_returns_badarg_and_sends_nothing(transport):
    reply = transport.command("D 150 150", read_timeout=500)  # missing <mm>

    assert reply.startswith("ERR badarg"), reply
    assert transport._conn.sent == []


def test_d_with_zero_speed_returns_badarg_and_sends_nothing(transport):
    reply = transport.command("D 0 0 300", read_timeout=500)

    assert reply.startswith("ERR badarg"), reply
    assert transport._conn.sent == []


def test_malformed_rt_line_returns_badarg_and_sends_nothing(transport):
    reply = transport.command("RT notanumber", read_timeout=500)

    assert reply.startswith("ERR badarg"), reply
    assert transport._conn.sent == []


def test_send_malformed_line_logs_error_and_raises_nothing(transport):
    transport.send("D 150 150")  # missing <mm> -- must not raise

    assert transport._conn.sent == []
    assert any("ERROR" in line for line in transport.logs)


# ---------------------------------------------------------------------------
# Ack outcomes
# ---------------------------------------------------------------------------


def test_ack_timeout_returns_err_unknown_move_timeout(transport):
    transport._conn.ack_result = None  # no matching ack ever arrives

    reply = transport.command("RT 9000", read_timeout=500)

    assert reply == "ERR unknown move-timeout", reply
    assert len(transport._conn.sent) == 1  # the Move was still sent


def test_nak_ack_returns_err_nak_with_err_code(transport):
    transport._conn.ack_result = telemetry_pb2.Telemetry(
        flags=_ACK_FRESH_BIT, ack_corr=1, ack_err=envelope_pb2.ERR_BADARG)

    reply = transport.command("RT 9000", read_timeout=500)

    assert reply.startswith("ERR nak move"), reply
    assert str(int(envelope_pb2.ERR_BADARG)) in reply


# ---------------------------------------------------------------------------
# send() is fire-and-forget -- no ack wait
# ---------------------------------------------------------------------------


def test_send_does_not_wait_for_ack(transport):
    """``send()`` must not call ``wait_for_ack()`` at all -- unlike
    ``command()``, a matched D/RT/SEG line is dispatched and forgotten."""
    transport._conn.ack_result = None  # would make command() time out

    transport.send("D 150 150 300")  # must not raise/hang despite no ack

    assert len(transport._conn.sent) == 1


# ---------------------------------------------------------------------------
# Unrecognized verbs still fall through to binary_bridge's legacy stub
# ---------------------------------------------------------------------------


def test_unrecognized_verb_falls_through_to_legacy_stub(transport):
    reply = transport.command("S 200 200", read_timeout=500)

    assert reply == binary_bridge._LEGACY_UNAVAILABLE_REPLY
    assert transport._conn.sent == []


# ---------------------------------------------------------------------------
# run_unmanaged() -- the hardware counterpart of SimTransport.run_unmanaged()
# ---------------------------------------------------------------------------


def test_run_unmanaged_distance_sends_move_twist_with_distance_stop(transport):
    transport.run_unmanaged(distance_mm=200.0)

    assert len(transport._conn.sent) == 1
    sent = transport._conn.sent[0]
    assert sent.move.WhichOneof("velocity") == "twist"
    assert sent.move.twist.v_x > 0.0
    assert sent.move.WhichOneof("stop") == "distance"
    assert sent.move.distance == pytest.approx(200.0)
    assert sent.move.timeout > 0.0


def test_run_unmanaged_negative_distance_flips_v_x_sign(transport):
    transport.run_unmanaged(distance_mm=-200.0)

    sent = transport._conn.sent[0]
    assert sent.move.twist.v_x < 0.0
    assert sent.move.distance == pytest.approx(200.0)


def test_run_unmanaged_angle_sends_move_twist_with_angle_stop(transport):
    transport.run_unmanaged(angle_deg=360.0)

    sent = transport._conn.sent[0]
    assert sent.move.WhichOneof("velocity") == "twist"
    assert sent.move.twist.omega == pytest.approx(_UNMANAGED_YAW_RATE)
    assert sent.move.WhichOneof("stop") == "angle"
    assert sent.move.angle == pytest.approx(math.radians(360.0))


def test_run_unmanaged_zero_zero_is_a_noop(transport):
    transport.run_unmanaged()

    assert transport._conn.sent == []


def test_run_unmanaged_distance_wins_when_both_given(transport):
    transport.run_unmanaged(distance_mm=100.0, angle_deg=90.0)

    sent = transport._conn.sent[0]
    assert sent.move.WhichOneof("stop") == "distance"


def test_run_unmanaged_not_connected_logs_warning_does_not_raise():
    t = SerialTransport("dummy-port")  # never connected -- _conn/_proto are None
    logs: list[str] = []
    t.on_log = logs.append

    t.run_unmanaged(distance_mm=200.0)  # must not raise

    assert any("WARN" in line for line in logs)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
