"""src/tests/unit/test_protocol_set_config_field.py — 132-012 (Generic
applyField(target, field, value) setter + SetConfigField wire command).

``NezhaProtocol.set_config_field(target, field_name, value)`` — the
development-mode single-value push ``.claude/rules/configuration-
discipline.md`` carves out for bench tuning sweeps. Resolves ``field_name``
to its wire field NUMBER via the REAL compiled protobuf descriptor for
``target``'s own group message (``<Group>.DESCRIPTOR.fields_by_name[...]
.number``) — a human still types a name, the wire still carries only a
number — then sends ``CommandEnvelope{set_field: SetConfigField{target,
field, value}}`` via the fire-and-poll ack-ring path (``send_envelope_
fast()`` + ``wait_for_ack()``), the SAME shape ``set_config_binary()``/
``config()`` already use for every other CONFIG-arm SET — never the
synchronous ``get_config()`` path.

Uses the SAME ``send_envelope_fast()``/``wait_for_ack()``-shaped fake
connection double ``test_protocol_config.py``'s own ``_FakeFastConn``
established — duplicated here rather than imported, matching this
project's "each test file is self-contained, no shared Python-level
fixture across files" convention (``src/tests/CLAUDE.md``).

Collected under ``src/tests/unit/`` — ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by
default.
"""

from __future__ import annotations

import pytest

from robot_radio.robot.pb2 import envelope_pb2, robot_config_pb2
from robot_radio.robot.protocol import AckEntry, NezhaProtocol


class _FakeFastConn:
    """Minimal fake connection: implements ``send_envelope_fast()`` +
    ``wait_for_ack()`` — the SAME shape ``test_protocol_config.py``'s own
    ``_FakeFastConn`` uses for ``config()``/``set_config_binary()``.
    ``wait_for_ack()`` here returns whatever raw packed ring-entry ``int``
    (``corr_id << 4 | err``) a test scripts, defaulting to ``None`` (a
    bounded-timeout-with-no-match) — ``NezhaProtocol.wait_for_ack()``
    unpacks it into an ``AckEntry`` exactly as it would a real
    ``SerialConnection``'s."""

    def __init__(self) -> None:
        self.sent: list["envelope_pb2.CommandEnvelope"] = []
        self._next_corr_id = 0
        self.ack_result: "int | None" = None

    def send_envelope_fast(self, envelope: "envelope_pb2.CommandEnvelope") -> int:
        self._next_corr_id += 1
        envelope.corr_id = self._next_corr_id
        self.sent.append(envelope)
        return self._next_corr_id

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "int | None":
        return self.ack_result


def _pack_ack(corr_id: int, err_code: int) -> int:
    """Mirrors telemetry.proto's own ``corr_id << 4 | err`` packing (the
    SAME helper shape ``test_protocol_config.py``/``test_twist_stop_ack_
    matcher.py`` already use for scripting a ring entry)."""
    return (corr_id << 4) | err_code


# ---------------------------------------------------------------------------
# 1. set_config_field() — envelope construction: field NAME resolves to the
#    real descriptor's field NUMBER, never a hand-maintained mapping table.
# ---------------------------------------------------------------------------


def test_set_config_field_resolves_field_name_via_real_descriptor():
    conn = _FakeFastConn()
    conn.ack_result = _pack_ack(1, 0)
    proto = NezhaProtocol(conn)

    proto.set_config_field(robot_config_pb2.DRIVE, "wheel_gain_left_decel", 1.043)

    assert len(conn.sent) == 1
    sent = conn.sent[0]
    assert sent.WhichOneof("cmd") == "set_field"
    assert sent.set_field.target == robot_config_pb2.DRIVE
    expected_number = robot_config_pb2.Drive.DESCRIPTOR.fields_by_name[
        "wheel_gain_left_decel"].number
    assert sent.set_field.field == expected_number
    assert sent.set_field.value == pytest.approx(1.043)


def test_set_config_field_never_sends_a_hand_maintained_number_a_schema_change_would_drift_from():
    """The field NUMBER on the wire always comes from the REAL compiled
    descriptor, not a literal this test (or protocol.py) hardcodes --
    resolving the SAME name against the descriptor directly must match
    whatever set_config_field() actually put on the wire, structurally
    closing the pid.kff -> kaff class of bug (a hand-written mapping table
    drifting from what it names)."""
    conn = _FakeFastConn()
    conn.ack_result = _pack_ack(1, 0)
    proto = NezhaProtocol(conn)

    proto.set_config_field(robot_config_pb2.WHEEL_CONTROL, "pid_kp", 0.11)

    sent = conn.sent[0]
    assert sent.set_field.field == robot_config_pb2.WheelControl.DESCRIPTOR.fields_by_name[
        "pid_kp"].number


@pytest.mark.parametrize(
    "target,pb_cls,field_name,value",
    [
        (robot_config_pb2.GEOMETRY, robot_config_pb2.Geometry, "trackwidth", 131.5),
        (robot_config_pb2.MOTORS, robot_config_pb2.Motors, "travel_calib_left", 0.487),
        (robot_config_pb2.DRIVE, robot_config_pb2.Drive, "wheel_gain_left_decel", 1.043),
        (robot_config_pb2.WHEEL_CONTROL, robot_config_pb2.WheelControl, "pid_kp", 0.11),
        (robot_config_pb2.PLANNER, robot_config_pb2.Planner, "v_max", 450.0),
        (robot_config_pb2.OTOS, robot_config_pb2.Otos, "linear_scale", 1.03),
        (robot_config_pb2.ESTIMATOR, robot_config_pb2.Estimator, "weight_heading_otos", 0.4),
    ],
)
def test_set_config_field_every_group_resolves_its_own_field(target, pb_cls, field_name, value):
    conn = _FakeFastConn()
    conn.ack_result = _pack_ack(1, 0)
    proto = NezhaProtocol(conn)

    proto.set_config_field(target, field_name, value)

    sent = conn.sent[0]
    assert sent.set_field.target == target
    assert sent.set_field.field == pb_cls.DESCRIPTOR.fields_by_name[field_name].number
    assert sent.set_field.value == pytest.approx(value)


# ---------------------------------------------------------------------------
# 2. set_config_field() — ack round trip (fire-and-poll, the ack ring, the
#    SAME shape set_config_binary()/config() already use for every other
#    CONFIG-arm SET).
# ---------------------------------------------------------------------------


def test_set_config_field_ok_ack_returns_ack_entry():
    conn = _FakeFastConn()
    conn.ack_result = _pack_ack(1, 0)  # err=0 -> ok
    proto = NezhaProtocol(conn)

    ack = proto.set_config_field(robot_config_pb2.DRIVE, "wheel_gain_left_decel", 1.043)

    assert ack == AckEntry(corr_id=1, ok=True, err_code=0)


def test_set_config_field_err_ack_returns_none():
    """A rejected push (ERR_BADARG/ERR_RANGE/ERR_NOT_LIVE/ERR_BUSY) returns
    None -- mirroring set_config_binary()'s own "return None on any NAK"
    convention exactly, not a distinct per-error-code shape."""
    conn = _FakeFastConn()
    conn.ack_result = _pack_ack(1, 3)  # ERR_RANGE (envelope.proto)
    proto = NezhaProtocol(conn)

    ack = proto.set_config_field(robot_config_pb2.DRIVE, "wheel_gain_left_decel", 0.0)

    assert ack is None


def test_set_config_field_timeout_returns_none():
    conn = _FakeFastConn()
    conn.ack_result = None  # no matching ring entry -- wait_for_ack() times out
    proto = NezhaProtocol(conn)

    ack = proto.set_config_field(robot_config_pb2.DRIVE, "wheel_gain_left_decel", 1.043, read_timeout=50)

    assert ack is None


# ---------------------------------------------------------------------------
# 3. set_config_field() — failure paths that send NO wire traffic at all.
# ---------------------------------------------------------------------------


def test_set_config_field_unknown_target_returns_none_no_wire_call():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    assert proto.set_config_field(999, "trackwidth", 1.0) is None
    assert conn.sent == []


def test_set_config_field_unknown_field_name_returns_none_no_wire_call():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    assert proto.set_config_field(robot_config_pb2.DRIVE, "not_a_real_field", 1.0) is None
    assert conn.sent == []


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
