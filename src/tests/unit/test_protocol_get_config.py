"""src/tests/unit/test_protocol_get_config.py — 132-011 (GetConfig/
ConfigSnapshot wire read-back + host get_config()).

``NezhaProtocol.get_config(target)`` is a NEW method (not a resurrection of
104-002's deleted ``get_config()``/``get_config_binary()``, which targeted
the pre-104 reserved ``get`` arm — see ``protocol.py``'s own module
docstring). It sends ``CommandEnvelope{get_config: GetConfig{target}}``
over the BLOCKING ``send_envelope()``/``_send_envelope()`` path (unlike
``move``/``config``/``stop``/``otos_config()``/``estimator_config()``,
which fire-and-poll the ack ring) and parses a genuinely synchronous
``ReplyEnvelope{cfg: ConfigSnapshot}`` reply into a typed, GENERATED
pydantic group model (``robot_config_generated.<Group>``, ticket 002's own
model — never a raw dict).

Uses the same ``send_envelope()``-shaped fake connection double
``test_binary_bridge.py``'s own ``_FakeConn`` established (a queued-reply
``send_envelope()`` returning the decoded ``ReplyEnvelope`` directly,
SimConnection-shaped — see ``NezhaProtocol._send_envelope()``'s own
docstring for why both shapes exist) — duplicated here rather than
imported, matching this project's "each test file is self-contained, no
shared Python-level fixture across files" convention
(``src/tests/CLAUDE.md``).

Collected under ``src/tests/unit/`` — ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by
default.
"""

from __future__ import annotations

import pytest

from robot_radio.config import robot_config_generated
from robot_radio.robot.pb2 import envelope_pb2, robot_config_pb2
from robot_radio.robot.protocol import NezhaProtocol


class _FakeConn:
    """Minimal fake connection: implements the BLOCKING ``send_envelope()``
    shape ``get_config()`` actually uses (unlike most of NezhaProtocol,
    which fires via ``send_envelope_fast()`` + polls the ack ring) —
    queued replies, returned directly (not wrapped in
    ``SerialConnection``'s own ``{"reply": ...}`` dict — ``_send_envelope()``
    normalizes both shapes, so a test can pick whichever is more
    convenient; this file picks the direct shape, matching
    ``test_binary_bridge.py``'s own ``_FakeConn``)."""

    def __init__(self) -> None:
        self.is_open = True
        self.mode: str | None = "serial"
        self.envelope_calls: list["envelope_pb2.CommandEnvelope"] = []
        self._reply_queue: list["envelope_pb2.ReplyEnvelope | None"] = []

    def queue_reply(self, reply: "envelope_pb2.ReplyEnvelope | None") -> None:
        self._reply_queue.append(reply)

    def send_envelope(self, envelope: "envelope_pb2.CommandEnvelope",
                      read_timeout: int = 500) -> "envelope_pb2.ReplyEnvelope | None":
        self.envelope_calls.append(envelope)
        return self._reply_queue.pop(0) if self._reply_queue else None


def _snapshot_reply(target: int, group_msg) -> "envelope_pb2.ReplyEnvelope":
    return envelope_pb2.ReplyEnvelope(
        corr_id=1,
        cfg=robot_config_pb2.ConfigSnapshot(
            target=target, body=group_msg.SerializeToString()))


# ---------------------------------------------------------------------------
# 1. get_config() — envelope construction (the REQUEST side).
# ---------------------------------------------------------------------------


def test_get_config_sends_get_config_envelope_with_target():
    conn = _FakeConn()
    conn.queue_reply(_snapshot_reply(robot_config_pb2.DRIVE, robot_config_pb2.Drive()))
    proto = NezhaProtocol(conn)

    proto.get_config(robot_config_pb2.DRIVE)

    assert len(conn.envelope_calls) == 1
    sent = conn.envelope_calls[0]
    assert sent.WhichOneof("cmd") == "get_config"
    assert sent.get_config.target == robot_config_pb2.DRIVE


def test_get_config_uses_the_blocking_send_envelope_path_not_fast():
    """get_config() must NOT go through send_envelope_fast()/the ack ring
    -- a ConfigSnapshot needs a real synchronous reply body, which the ack
    ring (a bare corr_id/err pair) has no room for."""
    conn = _FakeConn()
    conn.queue_reply(_snapshot_reply(robot_config_pb2.OTOS, robot_config_pb2.Otos()))
    proto = NezhaProtocol(conn)

    # _FakeConn deliberately has no send_envelope_fast() -- if get_config()
    # tried to call it, this test would raise AttributeError.
    result = proto.get_config(robot_config_pb2.OTOS)

    assert result is not None


# ---------------------------------------------------------------------------
# 2. get_config() — reply parsing, per group, into the GENERATED pydantic
#    model (ticket 002's robot_config_generated, not a raw dict).
# ---------------------------------------------------------------------------


def test_get_config_drive_returns_typed_generated_pydantic_model():
    conn = _FakeConn()
    conn.queue_reply(_snapshot_reply(
        robot_config_pb2.DRIVE,
        robot_config_pb2.Drive(duty_per_speed_left=0.512, crawl_pulse=0.08,
                               wheel_gain_left_accel=1.9)))
    proto = NezhaProtocol(conn)

    result = proto.get_config(robot_config_pb2.DRIVE)

    assert isinstance(result, robot_config_generated.Drive)
    assert result.duty_per_speed_left == pytest.approx(0.512)
    assert result.crawl_pulse == pytest.approx(0.08)
    assert result.wheel_gain_left_accel == pytest.approx(1.9)
    # Every OTHER field stays at its schema default (proto3 implicit
    # presence — the pushed message above never set them).
    assert result.duty_per_speed_right == 0.0
    assert result.wheel_intercept_left_accel == 0.0


def test_get_config_geometry_round_trips_a_boot_only_target():
    """GEOMETRY is boot-only for WRITES (a future set_config_group() would
    get ERR_NOT_LIVE) but GET is not gated by re-appliability — this test
    only proves get_config() itself parses a GEOMETRY snapshot correctly;
    the firmware-side "GET still works for boot-only groups" property is
    src/tests/sim/unit/test_configurator_getconfig.py's own job."""
    conn = _FakeConn()
    conn.queue_reply(_snapshot_reply(
        robot_config_pb2.GEOMETRY,
        robot_config_pb2.Geometry(trackwidth=131.5, rotational_slip=0.92)))
    proto = NezhaProtocol(conn)

    result = proto.get_config(robot_config_pb2.GEOMETRY)

    assert isinstance(result, robot_config_generated.Geometry)
    assert result.trackwidth == pytest.approx(131.5)
    assert result.rotational_slip == pytest.approx(0.92)


@pytest.mark.parametrize(
    "target,pb_cls,generated_cls,field_name,value",
    [
        (robot_config_pb2.MOTORS, robot_config_pb2.Motors,
         robot_config_generated.Motors, "travel_calib_left", 0.487),
        (robot_config_pb2.WHEEL_CONTROL, robot_config_pb2.WheelControl,
         robot_config_generated.WheelControl, "pid_kp", 0.11),
        (robot_config_pb2.PLANNER, robot_config_pb2.Planner,
         robot_config_generated.Planner, "v_max", 450.0),
        (robot_config_pb2.OTOS, robot_config_pb2.Otos,
         robot_config_generated.Otos, "linear_scale", 1.03),
        (robot_config_pb2.ESTIMATOR, robot_config_pb2.Estimator,
         robot_config_generated.Estimator, "weight_heading_otos", 0.4),
    ],
)
def test_get_config_every_group_parses_into_its_own_generated_model(
        target, pb_cls, generated_cls, field_name, value):
    conn = _FakeConn()
    conn.queue_reply(_snapshot_reply(target, pb_cls(**{field_name: value})))
    proto = NezhaProtocol(conn)

    result = proto.get_config(target)

    assert isinstance(result, generated_cls)
    assert getattr(result, field_name) == pytest.approx(value)


# ---------------------------------------------------------------------------
# 3. get_config() — failure paths: unknown target, timeout, err reply.
# ---------------------------------------------------------------------------


def test_get_config_unknown_target_returns_none_no_wire_call():
    conn = _FakeConn()
    proto = NezhaProtocol(conn)

    assert proto.get_config(999) is None
    assert conn.envelope_calls == []


def test_get_config_timeout_returns_none():
    conn = _FakeConn()
    conn.queue_reply(None)  # no reply queued -- send_envelope() times out
    proto = NezhaProtocol(conn)

    assert proto.get_config(robot_config_pb2.DRIVE) is None


def test_get_config_err_reply_returns_none():
    conn = _FakeConn()
    conn.queue_reply(envelope_pb2.ReplyEnvelope(
        corr_id=1, err=envelope_pb2.Error(code=envelope_pb2.ERR_BADARG)))
    proto = NezhaProtocol(conn)

    assert proto.get_config(robot_config_pb2.DRIVE) is None


def test_get_config_not_connected_returns_none():
    class _ClosedConn(_FakeConn):
        def __init__(self) -> None:
            super().__init__()
            self.is_open = False

        def send_envelope(self, envelope, read_timeout: int = 500):
            return {"error": "Not connected. Call connect first."}

    proto = NezhaProtocol(_ClosedConn())

    assert proto.get_config(robot_config_pb2.DRIVE) is None


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
