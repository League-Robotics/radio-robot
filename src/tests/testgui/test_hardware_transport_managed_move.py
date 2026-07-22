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
import time

import pytest

from robot_radio.robot.pb2 import envelope_pb2, telemetry_pb2
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame
from robot_radio.testgui import binary_bridge
from robot_radio.testgui.transport import SerialTransport, _UNMANAGED_YAW_RATE

# flags bit 5 (ack_fresh) -- telemetry.proto Telemetry.flags, 115-003 frame
# v2. AckEntry.from_telemetry() does not itself gate on this bit (see that
# method's own docstring) -- set here only for realism/parity with
# test_binary_bridge.py's own _ACK_FRESH_BIT usage.
_ACK_FRESH_BIT = 1 << 5
# flags bit 15 (kFlagFaultMoveTimeout) -- docs/protocol-v4.md sec 7.3.
_FAULT_MOVE_TIMEOUT_BIT = 1 << 15


class _FakeHardwareConn:
    """Minimal fake ``SerialConnection``: ``is_open`` (checked by
    ``send()``/``command()``/``run_unmanaged()`` before dispatching at all)
    plus ``send_envelope_fast()``/``wait_for_ack()`` -- the exact two
    methods ``NezhaProtocol.move_twist()``/``move_wheels()``/
    ``wait_for_ack()`` call on ``self._conn``. Mirrors
    ``test_protocol_config.py``'s own ``_FakeFastConn``.

    ``drain_binary_tlm()`` (stakeholder bench fix, 2026-07-22) backs
    ``_await_move_completion()``'s polling loop -- ``tlm_script`` is a list
    of "what the next call returns" entries (each a list of raw
    ``ReplyEnvelope``-shaped objects exposing ``.tlm``), consumed one call
    at a time; an exhausted script returns ``[]`` forever (no more frames
    arriving), matching a real connection's non-blocking drain."""

    def __init__(self) -> None:
        self.is_open = True
        self.sent: list["envelope_pb2.CommandEnvelope"] = []
        self._next_corr_id = 0
        self.ack_result: "telemetry_pb2.Telemetry | None" = None
        self.tlm_script: list[list[object]] = []

    def send_envelope_fast(self, envelope: "envelope_pb2.CommandEnvelope") -> int:
        self._next_corr_id += 1
        envelope.corr_id = self._next_corr_id
        self.sent.append(envelope)
        return self._next_corr_id

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "telemetry_pb2.Telemetry | None":
        return self.ack_result

    def drain_binary_tlm(self) -> list:
        if self.tlm_script:
            return self.tlm_script.pop(0)
        return []


class _FakeTlmReply:
    """A ``ReplyEnvelope``-shaped object exposing exactly what
    ``TLMFrame.from_pb2()`` needs off ``.tlm`` -- a real
    ``telemetry_pb2.Telemetry``, so ``from_pb2()`` runs completely
    unmocked."""

    def __init__(self, tlm: "telemetry_pb2.Telemetry") -> None:
        self.tlm = tlm


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
# Move completion feedback (stakeholder bench fix, 2026-07-22) --
# "verify that you say 'drive 500mm' and it drives for 500mm ... You did
# not test this. It never finishes." command()/send()/run_unmanaged() all
# now start a bounded background poller (_await_move_completion()) that
# watches telemetry for the Move's own completion ack (ack_corr ==
# Move.id, distinct from the enqueue ack) and logs ONE outcome line.
# ---------------------------------------------------------------------------


def _wait_for_log(transport, substr: str, timeout_s: float = 3.0) -> str:
    """Poll ``transport.logs`` for a line containing ``substr``, bounded by
    ``timeout_s`` -- the completion poller runs on a background daemon
    thread, so tests must wait for it rather than assert immediately after
    the dispatching call returns. Fails the test (via assert) if the bound
    is exceeded."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for line in transport.logs:
            if substr in line:
                return line
        time.sleep(0.02)
    raise AssertionError(
        f"no log line containing {substr!r} within {timeout_s}s; got: {transport.logs}")


def _completion_tlm(move_id: int, *, left: float, right: float,
                    timeout_fault: bool = False) -> "telemetry_pb2.Telemetry":
    flags = _ACK_FRESH_BIT | (_FAULT_MOVE_TIMEOUT_BIT if timeout_fault else 0)
    return telemetry_pb2.Telemetry(
        flags=flags, ack_corr=move_id, ack_err=0,
        enc_left=telemetry_pb2.EncoderReading(position=left),
        enc_right=telemetry_pb2.EncoderReading(position=right))


def test_completion_poll_logs_done_line_with_encoder_delta(transport):
    _ok_ack(transport._conn)
    transport._last_tlm = TLMFrame(enc=(1000, 1000))

    reply = transport.command("D 150 150 300", read_timeout=500)
    assert reply == "OK move", reply

    move_id = transport._conn.sent[0].move.id
    assert move_id != 0  # a real, distinct Move.id was assigned
    transport._conn.tlm_script.append(
        [_FakeTlmReply(_completion_tlm(move_id, left=1300.0, right=1298.0))])

    done_line = _wait_for_log(transport, "[DONE]")
    assert "completed" in done_line
    assert "L=+300mm" in done_line
    assert "R=+298mm" in done_line
    assert "|L-R|=2mm" in done_line


def test_completion_poll_timeout_fault_outcome(transport):
    _ok_ack(transport._conn)
    transport._last_tlm = TLMFrame(enc=(0, 0))

    transport.command("D 150 150 300", read_timeout=500)
    move_id = transport._conn.sent[0].move.id
    transport._conn.tlm_script.append(
        [_FakeTlmReply(_completion_tlm(move_id, left=200.0, right=200.0, timeout_fault=True))])

    done_line = _wait_for_log(transport, "[DONE]")
    assert "timeout-fault" in done_line


def test_completion_poll_no_ack_within_bound_logs_warn(transport):
    """A short RT (Move.timeout floors at _MOVE_MIN_TIMEOUT=2000ms) whose
    completion ack never arrives -- e.g. a later Move preempted it before
    it ever activated-and-ended (docs/protocol-v4.md sec 5.3) -- logs a
    WARN, not a hang, once the bound (timeout + margin) elapses."""
    _ok_ack(transport._conn)

    transport.command("RT 100", read_timeout=500)  # never scripts a matching ack

    warn_line = _wait_for_log(transport, "no completion ack observed", timeout_s=4.0)
    assert "WARN" in warn_line


def test_completion_poll_missing_baseline_reports_unavailable(transport):
    """No baseline telemetry captured yet (``_last_tlm`` still ``None``, a
    connection that has not received any frame at all) -- the completion
    line still fires, with an honest "unavailable" instead of a bogus
    delta computed against ``None``."""
    _ok_ack(transport._conn)
    assert transport._last_tlm is None

    transport.command("D 150 150 300", read_timeout=500)
    move_id = transport._conn.sent[0].move.id
    transport._conn.tlm_script.append(
        [_FakeTlmReply(_completion_tlm(move_id, left=300.0, right=300.0))])

    done_line = _wait_for_log(transport, "[DONE]")
    assert "encoder delta unavailable" in done_line


def test_second_dispatch_while_poll_in_flight_skips_new_poller(transport):
    _ok_ack(transport._conn)

    transport.command("RT 100", read_timeout=500)  # starts a poll that will run to its bound
    transport.command("RT 100", read_timeout=500)  # second dispatch -- poll already active

    skip_line = _wait_for_log(transport, "completion poll skipped", timeout_s=1.0)
    assert "INFO" in skip_line


def test_run_unmanaged_starts_completion_poll(transport):
    transport._last_tlm = TLMFrame(enc=(0, 0))

    transport.run_unmanaged(distance_mm=200.0)

    move_id = transport._conn.sent[0].move.id
    transport._conn.tlm_script.append(
        [_FakeTlmReply(_completion_tlm(move_id, left=200.0, right=199.0))])

    done_line = _wait_for_log(transport, "[DONE]")
    assert "unmanaged drive" in done_line
    assert "completed" in done_line


def test_send_starts_completion_poll_without_waiting_for_enqueue_ack(transport):
    """``send()`` is fire-and-forget on the ENQUEUE ack (no
    ``wait_for_ack()`` call -- see ``test_send_does_not_wait_for_ack()``
    above) but still starts the completion poller right after dispatch."""
    transport._conn.ack_result = None  # would make command() time out
    transport._last_tlm = TLMFrame(enc=(0, 0))

    transport.send("D 150 150 300")

    move_id = transport._conn.sent[0].move.id
    transport._conn.tlm_script.append(
        [_FakeTlmReply(_completion_tlm(move_id, left=300.0, right=300.0))])

    done_line = _wait_for_log(transport, "[DONE]")
    assert "completed" in done_line


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
