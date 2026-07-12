"""tests/unit/test_bridge_routing.py -- 097-004 (M5 rogo Translator Proxy).

Exercises ``ProtocolBridge``'s routing layer (``_handle_client_line`` and
its per-verb handlers) plus ``_EvtWatcher``'s state machine, entirely
against a ``_FakeConn`` double -- no real PTY, no background threads (the
bridge under test is never ``start()``-ed), no hardware. This is the
"per-verb envelope differential vs legacy_translate, corr-id round trip,
STOP clears the EVT watch, relay-verb swallow, unknown verb -> typed ERR
with no wire call, SET badkey local, GET fan-out targets, SNAP restores
prior stream state, _EvtWatcher transitions" tier the ticket's testing
plan names -- the PTY-transport end-to-end tier lives in
``test_bridge_pty_e2e.py``.
"""

from __future__ import annotations

import pytest

from robot_radio.io.proxy import ProtocolBridge, _EvtWatcher
from robot_radio.robot import legacy_translate
from robot_radio.robot.pb2 import config_pb2, envelope_pb2, telemetry_pb2


# ---------------------------------------------------------------------------
# Test double -- records every envelope/fast-send the bridge hands the
# underlying connection; canned replies are queued FIFO (one per expected
# round trip, in call order) by the test.
# ---------------------------------------------------------------------------


class _FakeConn:
    def __init__(self):
        self.envelope_calls: list[envelope_pb2.CommandEnvelope] = []
        self.fast_calls: list[str] = []
        self._reply_queue: list[envelope_pb2.ReplyEnvelope | None] = []
        self._tlm_queue: list[envelope_pb2.ReplyEnvelope] = []

    def queue_reply(self, reply: "envelope_pb2.ReplyEnvelope | None") -> None:
        self._reply_queue.append(reply)

    def push_tlm(self, reply: "envelope_pb2.ReplyEnvelope") -> None:
        self._tlm_queue.append(reply)

    # -- SerialConnection surface ProtocolBridge/NezhaProtocol use --------

    def send_envelope(self, envelope: envelope_pb2.CommandEnvelope,
                      read_timeout: int = 500) -> dict:
        self.envelope_calls.append(envelope)
        reply = self._reply_queue.pop(0) if self._reply_queue else None
        return {"sent": envelope, "mode": "direct", "reply": reply}

    def send_fast(self, message: str) -> None:
        self.fast_calls.append(message)

    def drain_binary_tlm(self) -> list:
        # Decoupled from `push_tlm`/`read_binary_tlm`'s queue: production
        # code calls this ONLY to clear STALE frames queued before an
        # arm-wait-disarm sequence starts (see ProtocolBridge.
        # _snap_binary_frame's own docstring); this fake's synchronous
        # `push_tlm()` always represents a frame that arrives DURING the
        # wait, never a stale leftover, so there is nothing for this to
        # drain in any test that uses `push_tlm()`.
        return []

    def read_binary_tlm(self, duration: int) -> list:
        frames, self._tlm_queue = self._tlm_queue, []
        return frames

    @property
    def is_open(self) -> bool:
        return True


def _bridge(**kwargs) -> tuple[ProtocolBridge, _FakeConn]:
    fake = _FakeConn()
    bridge = ProtocolBridge(fake, **kwargs)
    return bridge, fake


def _ack(**kwargs) -> envelope_pb2.ReplyEnvelope:
    return envelope_pb2.ReplyEnvelope(ok=envelope_pb2.Ack(**kwargs))


# ---------------------------------------------------------------------------
# Per-verb envelope differential vs legacy_translate -- the routed envelope
# must be byte-identical to what legacy_translate's OWN builders (one layer
# below legacy_verbs.BINARY_DISPATCH) produce, not just self-consistent
# with whatever legacy_verbs happens to build.
# ---------------------------------------------------------------------------


def test_route_s_drive_matches_legacy_translate():
    bridge, fake = _bridge()
    fake.queue_reply(_ack(q=1))
    line = bridge._handle_client_line("S 200 200 #1")

    expected = envelope_pb2.CommandEnvelope()
    expected.drive.wheels.CopyFrom(legacy_translate.wheel_targets_for_drive(200.0, 200.0))
    assert fake.envelope_calls[-1].SerializeToString() == expected.SerializeToString()
    assert line == "OK drive l=200 r=200 #1"


def test_route_d_distance_matches_legacy_translate():
    bridge, fake = _bridge()
    fake.queue_reply(_ack(q=1))
    line = bridge._handle_client_line("D 200 200 300")

    expected = envelope_pb2.CommandEnvelope(
        segment=legacy_translate.segment_for_distance(200.0, 200.0, 300))
    assert fake.envelope_calls[-1].SerializeToString() == expected.SerializeToString()
    assert line == "OK drive l=200 r=200 mm=300"


def test_route_t_timed_matches_legacy_translate():
    bridge, fake = _bridge()
    fake.queue_reply(_ack(q=1))
    line = bridge._handle_client_line("T 200 200 1000")
    expected = envelope_pb2.CommandEnvelope(
        segment=legacy_translate.segment_for_timed(200.0, 200.0, 1000))
    assert fake.envelope_calls[-1].SerializeToString() == expected.SerializeToString()
    assert line == "OK drive l=200 r=200 ms=1000"


def test_route_rt_matches_legacy_translate():
    bridge, fake = _bridge()
    fake.queue_reply(_ack(q=1))
    line = bridge._handle_client_line("RT 9000 #4")
    expected = envelope_pb2.CommandEnvelope(segment=legacy_translate.segment_for_rt(9000.0))
    assert fake.envelope_calls[-1].SerializeToString() == expected.SerializeToString()
    assert line == "OK rt rot=9000 #4"


def test_route_move_matches_legacy_translate_and_arms_evt():
    bridge, fake = _bridge()
    fake.queue_reply(_ack(q=2, rem=5.0))
    line = bridge._handle_client_line("MOVE 500 9000 9000 v=300 w=4500 s=1")
    expected = envelope_pb2.CommandEnvelope(segment=legacy_translate.segment_for_move(
        500.0, 9000.0, 9000.0, speed_max=300.0, yaw_rate_max=4500.0, stream=True))
    assert fake.envelope_calls[-1].SerializeToString() == expected.SerializeToString()
    assert line == "OK move dist=500 dir=9000 fh=9000 q=2 rem=5"
    assert bridge._evt_watcher.pending is True


def test_route_mover_matches_legacy_translate_and_does_not_arm_evt():
    bridge, fake = _bridge()
    fake.queue_reply(_ack(q=1))
    line = bridge._handle_client_line("MOVER 0 0 0 t=400 v=-300 w=-4500")
    expected = envelope_pb2.CommandEnvelope(replace=legacy_translate.segment_for_mover(
        0.0, 0.0, 0.0, time=400.0, v=-300.0, omega=-4500.0))
    assert fake.envelope_calls[-1].SerializeToString() == expected.SerializeToString()
    assert line == "OK mover t=400 v=-300 w=-4500 q=1"
    assert bridge._evt_watcher.pending is False


def test_route_echo_round_trips_payload():
    bridge, fake = _bridge()
    fake.queue_reply(envelope_pb2.ReplyEnvelope(echo=envelope_pb2.Echo(payload=b"hi there")))
    line = bridge._handle_client_line("ECHO hi there")
    assert fake.envelope_calls[-1].echo.payload == b"hi there"
    assert line == "OK echo hi there"


def test_route_ping_uses_ack_t():
    bridge, fake = _bridge()
    fake.queue_reply(_ack(t=99999))
    line = bridge._handle_client_line("PING #2")
    assert fake.envelope_calls[-1].WhichOneof("cmd") == "ping"
    assert line == "OK pong t=99999 #2"


def test_route_id_and_ver_both_use_id_arm():
    bridge, fake = _bridge()
    device = envelope_pb2.DeviceId(model="NEZHA2", name="GUTOV", serial=5,
                                   fw_version="v1", proto_version=3)
    fake.queue_reply(envelope_pb2.ReplyEnvelope(id=device))
    line = bridge._handle_client_line("ID")
    assert fake.envelope_calls[-1].WhichOneof("cmd") == "id"
    assert line == "ID model=NEZHA2 name=GUTOV serial=5 fw=v1 proto=3"

    fake.queue_reply(envelope_pb2.ReplyEnvelope(id=device))
    line = bridge._handle_client_line("VER")
    assert line == "OK ver fw=v1 proto=3"


# ---------------------------------------------------------------------------
# Corr-id round trip
# ---------------------------------------------------------------------------


def test_corr_id_reattached_to_rendered_reply_not_wire_envelope():
    bridge, fake = _bridge()
    fake.queue_reply(_ack())
    line = bridge._handle_client_line("STOP #77")
    assert line == "OK stop #77"
    # send_envelope() owns corr_id assignment on the real SerialConnection;
    # the proxy never writes the client's id into the outgoing envelope.
    assert fake.envelope_calls[-1].corr_id == 0


def test_no_corr_id_produces_no_hash_suffix():
    bridge, fake = _bridge()
    fake.queue_reply(_ack())
    assert bridge._handle_client_line("STOP") == "OK stop"


# ---------------------------------------------------------------------------
# STOP clears the pending EVT watch
# ---------------------------------------------------------------------------


def test_stop_clears_pending_evt_watch_silently():
    bridge, fake = _bridge()
    fake.queue_reply(_ack(q=1, rem=0.0))
    bridge._handle_client_line("D 200 200 300")
    assert bridge._evt_watcher.pending is True

    fake.queue_reply(_ack())
    line = bridge._handle_client_line("STOP")
    assert line == "OK stop"
    assert bridge._evt_watcher.pending is False


# ---------------------------------------------------------------------------
# Relay-control lines swallowed locally
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("raw", ["!MODE RAW250", "!ECHO OFF", "!GO", "!CG 5", "!P 1", "?"])
def test_relay_control_lines_swallowed_with_ok_comment(raw):
    bridge, fake = _bridge()
    line = bridge._handle_client_line(raw)
    assert line == "# ok"
    assert fake.envelope_calls == []
    assert fake.fast_calls == []


def test_keepalive_plus_forwarded_via_send_fast_no_reply():
    bridge, fake = _bridge()
    line = bridge._handle_client_line("+")
    assert line is None
    assert fake.fast_calls == ["+"]
    assert fake.envelope_calls == []


# ---------------------------------------------------------------------------
# Unsupported verbs -- typed ERR, zero wire traffic, never a hang.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("raw,verb", [
    ("R 200 500", "R"), ("TURN 9000", "TURN"), ("G 100 100 200", "G"),
    ("GRIP 90", "GRIP"), ("QLEN", "QLEN"), ("DEV STATE", "DEV"),
    ("FROBNICATE 1 2 3", "FROBNICATE"),
])
def test_unsupported_verbs_typed_err_no_wire_call(raw, verb):
    bridge, fake = _bridge()
    line = bridge._handle_client_line(raw)
    assert line == f"ERR unsupported {verb}"
    assert fake.envelope_calls == []
    assert fake.fast_calls == []


@pytest.mark.parametrize("raw,verb", [
    ("SI 0 0 0", "SI"), ("OI", "OI"), ("OZ", "OZ"), ("OL 5", "OL"),
])
def test_pose_otos_verbs_gated_unsupported_until_098(raw, verb):
    bridge, fake = _bridge()
    line = bridge._handle_client_line(raw)
    assert line == f"ERR unsupported {verb}"
    assert fake.envelope_calls == []


def test_binary_native_client_line_rejected():
    bridge, fake = _bridge()
    line = bridge._handle_client_line("*Bsomebase64==")
    assert line == "ERR unsupported proxy-is-text-only"
    assert fake.envelope_calls == []


# ---------------------------------------------------------------------------
# Local (non-wire) verbs -- HELLO/HELP
# ---------------------------------------------------------------------------


def test_hello_answers_locally_from_cached_device_id_no_wire_call():
    bridge, fake = _bridge()
    bridge._device_id = envelope_pb2.DeviceId(name="GUTOV", serial=2121102)
    line = bridge._handle_client_line("HELLO")
    assert line == "DEVICE:NEZHA2:robot:GUTOV:2121102"
    assert fake.envelope_calls == []


def test_help_answers_locally_no_wire_call():
    bridge, fake = _bridge()
    line = bridge._handle_client_line("HELP")
    assert line.startswith("OK help ")
    assert fake.envelope_calls == []


# ---------------------------------------------------------------------------
# SET -- badkey local (no wire traffic), good key round trip
# ---------------------------------------------------------------------------


def test_set_badkey_local_no_wire_traffic():
    bridge, fake = _bridge()
    line = bridge._handle_client_line("SET notakey=5 #1")
    assert line == "ERR badkey notakey #1"
    assert fake.envelope_calls == []


def test_set_good_key_round_trips_through_config_binary():
    bridge, fake = _bridge()
    fake.queue_reply(_ack())
    line = bridge._handle_client_line("SET tw=128")
    assert fake.envelope_calls[-1].WhichOneof("cmd") == "config"
    assert fake.envelope_calls[-1].config.drivetrain.trackwidth == pytest.approx(128.0)
    assert line == "OK set tw=128"


# ---------------------------------------------------------------------------
# GET -- fan-out across distinct ConfigTarget groups, merged into ONE line
# ---------------------------------------------------------------------------


def test_get_fans_out_one_round_trip_per_distinct_target_merges_one_line():
    bridge, fake = _bridge()
    dt_snapshot = envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_DRIVETRAIN,
        drivetrain=config_pb2.DrivetrainConfigPatch(trackwidth=128.0))
    motor_snapshot = envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_MOTOR_LEFT,
        motor=config_pb2.MotorConfigPatch(kp=1.5))
    fake.queue_reply(envelope_pb2.ReplyEnvelope(cfg=dt_snapshot))
    fake.queue_reply(envelope_pb2.ReplyEnvelope(cfg=motor_snapshot))

    line = bridge._handle_client_line("GET tw pid.kp #3")

    assert len(fake.envelope_calls) == 2
    assert fake.envelope_calls[0].get.target == config_pb2.CONFIG_DRIVETRAIN
    assert fake.envelope_calls[1].get.target == config_pb2.CONFIG_MOTOR_LEFT
    assert line == "CFG tw=128 pid.kp=1.500 #3"


def test_get_badkey_local_no_wire_traffic():
    bridge, fake = _bridge()
    line = bridge._handle_client_line("GET notakey")
    assert line == "ERR badkey notakey"
    assert fake.envelope_calls == []


# ---------------------------------------------------------------------------
# SNAP -- restores PRIOR stream state, does not blindly stream(0)
# ---------------------------------------------------------------------------


def test_snap_restores_prior_client_stream_period():
    bridge, fake = _bridge()
    bridge._client_stream_period = 100
    bridge._last_upstream_period = 100

    frame = telemetry_pb2.Telemetry(now=1, mode=0, seq=1)
    fake.queue_reply(_ack())  # arm at the 20ms floor
    fake.push_tlm(envelope_pb2.ReplyEnvelope(tlm=frame))
    fake.queue_reply(_ack())  # restore to 100ms

    line = bridge._handle_client_line("SNAP")

    stream_periods = [c.stream.period for c in fake.envelope_calls if c.WhichOneof("cmd") == "stream"]
    assert stream_periods == [20, 100]
    assert line == "TLM t=1 mode=I seq=1"
    assert bridge._client_stream_period == 100  # unchanged by SNAP


def test_snap_restores_zero_when_client_never_armed_stream():
    bridge, fake = _bridge()
    frame = telemetry_pb2.Telemetry(now=1, mode=0, seq=1)
    fake.queue_reply(_ack())
    fake.push_tlm(envelope_pb2.ReplyEnvelope(tlm=frame))
    fake.queue_reply(_ack())

    bridge._handle_client_line("SNAP")
    stream_periods = [c.stream.period for c in fake.envelope_calls if c.WhichOneof("cmd") == "stream"]
    assert stream_periods == [20, 0]


def test_snap_timeout_typed_error():
    bridge, fake = _bridge()
    fake.queue_reply(_ack())
    fake.queue_reply(_ack())
    line = bridge._handle_client_line("SNAP")
    assert line == "ERR unknown snap-timeout"


# ---------------------------------------------------------------------------
# One-shot TLM verb
# ---------------------------------------------------------------------------


def test_tlm_one_shot_ok_wrapped_with_bench_body():
    bridge, fake = _bridge()
    frame = telemetry_pb2.Telemetry(now=5, mode=0, seq=0, active=True,
                                    conn_left=True, conn_right=True)
    fake.queue_reply(_ack())
    fake.push_tlm(envelope_pb2.ReplyEnvelope(tlm=frame))
    fake.queue_reply(_ack())
    line = bridge._handle_client_line("TLM #9")
    assert line.startswith("OK tlm ")
    assert "active=1" in line
    assert line.endswith("#9")


# ---------------------------------------------------------------------------
# STREAM -- clamp + client-stream-armed flag
# ---------------------------------------------------------------------------


def test_stream_clamps_to_firmware_floor():
    bridge, fake = _bridge()
    fake.queue_reply(_ack())
    line = bridge._handle_client_line("STREAM 5")
    assert line == "OK stream period=20"
    assert bridge._client_stream_period == 20


def test_stream_zero_disarms():
    bridge, fake = _bridge()
    fake.queue_reply(_ack())
    bridge._handle_client_line("STREAM 100")
    fake.queue_reply(_ack())
    line = bridge._handle_client_line("STREAM 0")
    assert line == "OK stream period=0"
    assert bridge._client_stream_period == 0


# ---------------------------------------------------------------------------
# _EvtWatcher transitions -- pure state machine, injected `now`.
# ---------------------------------------------------------------------------


def test_evt_watcher_wait_busy_to_busy_to_idle_fires_once():
    w = _EvtWatcher()
    w.arm("D", "1", now=0.0)
    assert w.state == _EvtWatcher.WAIT_BUSY
    assert w.observe(True, now=0.1) is None       # -> BUSY
    assert w.state == _EvtWatcher.BUSY
    line = w.observe(False, now=0.5)               # -> emit, back to IDLE
    assert line == "EVT done D #1 reason=idle"
    assert w.state == _EvtWatcher.IDLE
    # A second "still inactive" observation must NOT re-fire.
    assert w.observe(False, now=0.6) is None


def test_evt_watcher_wait_busy_cap_expiry_fires_anyway():
    w = _EvtWatcher()
    w.arm("T", None, now=0.0)
    assert w.observe(False, now=1.0) is None        # within cap, no active yet
    line = w.observe(False, now=2.1)                # cap (2.0s) expired
    assert line == "EVT done T reason=idle"
    assert w.state == _EvtWatcher.IDLE


def test_evt_watcher_stop_clears_pending_watch_silently():
    w = _EvtWatcher()
    w.arm("MOVE", "9", now=0.0)
    w.clear()
    assert w.state == _EvtWatcher.IDLE
    assert w.pending is False
    # No emission possible after clear -- observe() is a no-op from IDLE.
    assert w.observe(True, now=0.1) is None
    assert w.observe(False, now=0.2) is None


def test_evt_watcher_new_motion_verb_supersedes_pending():
    w = _EvtWatcher()
    w.arm("D", "1", now=0.0)
    w.observe(True, now=0.1)  # -> BUSY for D
    w.arm("RT", "2", now=0.2)  # a new motion verb supersedes the pending D watch
    assert w.state == _EvtWatcher.WAIT_BUSY
    line = w.observe(True, now=0.3)
    assert line is None
    assert w.state == _EvtWatcher.BUSY
    line = w.observe(False, now=0.4)
    assert line == "EVT done RT #2 reason=idle"


def test_evt_watcher_disabled_never_arms():
    w = _EvtWatcher(enabled=False)
    w.arm("D", "1", now=0.0)
    assert w.state == _EvtWatcher.IDLE
    assert w.pending is False


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
