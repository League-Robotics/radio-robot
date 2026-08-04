"""src/tests/unit/test_protocol_verified_push.py -- ticket 133-006 parts 2
and 3 on the HOST side (A-live-config-push-is-wiped-by-the-next-reconnect.md):

- ``get_config_snapshot()`` carries per-group PROVENANCE off the
  ``ConfigSnapshot`` reply, so a caller can ask "is the robot running what I
  pushed" and get an answer rather than an inference.
- ``set_config_field()``/``set_config_group()`` grow a ``verify=True`` path
  that reads the value back and RAISES ``ConfigNotVerified`` unless the robot
  reports what was sent.

Why raising matters: 132-019 lost a whole bench measurement to a config push
that acked OK and landed nowhere, and caught it only because that script
happened to read back by hand. Returning ``None`` puts the burden on every
caller to check; raising puts it on the one that does not.

Uses the same ``_FakeConn`` shape ``test_protocol_get_config.py`` established,
extended with the ack-ring ``send_envelope_fast()``/``poll_ack()`` pair the SET
path uses -- duplicated rather than imported, per this project's "each test
file is self-contained" convention (``src/tests/CLAUDE.md``).

Collected under ``src/tests/unit/`` -- ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by default.
"""

from __future__ import annotations

import pytest

from robot_radio.robot.pb2 import envelope_pb2, robot_config_pb2
from robot_radio.robot.protocol import (
    ConfigNotVerified,
    ConfigReadback,
    NezhaProtocol,
)


class _FakeAck:
    """Minimal AckEntry stand-in: ``ok`` is all the SET path reads."""

    def __init__(self, ok: bool = True, err_code: int = 0) -> None:
        self.ok = ok
        self.err_code = err_code

    def __repr__(self) -> str:  # pragma: no cover - debugging aid only
        return f"_FakeAck(ok={self.ok}, err_code={self.err_code})"


class _FakeRobot:
    """A fake connection that behaves like a robot holding one config group.

    Holds a real mutable group message per target, so a push and the read-back
    that follows it see the SAME object -- which is what makes the verify
    tests meaningful. ``drop_pushes`` reproduces the actual defect being
    guarded against: the robot acks OK and discards the value.
    """

    def __init__(self, *, drop_pushes: bool = False, ack_ok: bool = True,
                 source: int = robot_config_pb2.CONFIG_SOURCE_BAKED) -> None:
        self.is_open = True
        self.mode: str | None = "serial"
        self.drop_pushes = drop_pushes
        self.ack_ok = ack_ok
        self.source = source
        self.groups: dict[int, object] = {}
        self.sent: list = []
        self.get_config_calls = 0
        self._corr_id = 0

    # -- the group store ------------------------------------------------
    def _group(self, target: int):
        if target not in self.groups:
            name = {
                robot_config_pb2.DRIVE: "Drive",
                robot_config_pb2.WHEEL_CONTROL: "WheelControl",
                robot_config_pb2.OTOS: "Otos",
                robot_config_pb2.PLANNER: "Planner",
            }[target]
            self.groups[target] = getattr(robot_config_pb2, name)()
        return self.groups[target]

    # -- the SET path (ack ring) ---------------------------------------
    def send_envelope_fast(self, envelope) -> int:
        self.sent.append(envelope)
        self._corr_id += 1
        arm = envelope.WhichOneof("cmd")
        if not self.drop_pushes and self.ack_ok:
            if arm == "set_field":
                request = envelope.set_field
                group = self._group(request.target)
                field = group.DESCRIPTOR.fields_by_number[request.field]
                setattr(group, field.name, request.value)
            elif arm == "config":
                request = envelope.config
                group = self._group(request.target)
                group.CopyFrom(type(group).FromString(request.body))
        return self._corr_id

    def poll_ack(self, corr_id: int, timeout: int = 500):
        return _FakeAck(ok=self.ack_ok) if self.ack_ok else None

    # -- the GET path (blocking reply) ----------------------------------
    def send_envelope(self, envelope, read_timeout: int = 500):
        arm = envelope.WhichOneof("cmd")
        assert arm == "get_config", f"unexpected blocking send of {arm}"
        self.get_config_calls += 1
        target = envelope.get_config.target
        return envelope_pb2.ReplyEnvelope(
            corr_id=1,
            cfg=robot_config_pb2.ConfigSnapshot(
                target=target,
                body=self._group(target).SerializeToString(),
                source=self.source,
            ))


# ---------------------------------------------------------------------------
# 1. Provenance on the reply -- get_config_snapshot().
# ---------------------------------------------------------------------------


def test_get_config_snapshot_reports_the_source_off_the_reply():
    conn = _FakeRobot(source=robot_config_pb2.CONFIG_SOURCE_LIVE)
    proto = NezhaProtocol(conn)

    snapshot = proto.get_config_snapshot(robot_config_pb2.DRIVE)

    assert isinstance(snapshot, ConfigReadback)
    assert snapshot.target == robot_config_pb2.DRIVE
    assert snapshot.source == robot_config_pb2.CONFIG_SOURCE_LIVE
    assert snapshot.source_name == "LIVE"


@pytest.mark.parametrize("source,expected", [
    (robot_config_pb2.CONFIG_SOURCE_BAKED, "BAKED"),
    (robot_config_pb2.CONFIG_SOURCE_LIVE, "LIVE"),
    (robot_config_pb2.CONFIG_SOURCE_PERSISTED, "PERSISTED"),
    (robot_config_pb2.CONFIG_SOURCE_UNSPECIFIED, "UNSPECIFIED"),
])
def test_every_source_value_round_trips_with_a_readable_name(source, expected):
    conn = _FakeRobot(source=source)
    proto = NezhaProtocol(conn)

    snapshot = proto.get_config_snapshot(robot_config_pb2.OTOS)

    assert snapshot.source == source
    assert snapshot.source_name == expected


def test_a_pushed_group_reads_live_while_an_untouched_sibling_reads_baked():
    """The case a single global flag would get wrong.

    Two groups, two independent provenances, one connection -- ``DRIVE`` is
    live while ``PLANNER`` is still baked. A robot with one flag would have to
    report the same answer for both.
    """
    class _PerGroupSource(_FakeRobot):
        def send_envelope(self, envelope, read_timeout: int = 500):
            target = envelope.get_config.target
            self.get_config_calls += 1
            source = (robot_config_pb2.CONFIG_SOURCE_LIVE
                      if target == robot_config_pb2.DRIVE
                      else robot_config_pb2.CONFIG_SOURCE_BAKED)
            return envelope_pb2.ReplyEnvelope(
                corr_id=1,
                cfg=robot_config_pb2.ConfigSnapshot(
                    target=target, body=self._group(target).SerializeToString(),
                    source=source))

    conn = _PerGroupSource()
    proto = NezhaProtocol(conn)

    proto.set_config_field(robot_config_pb2.DRIVE, "duty_per_speed_left", 0.42)

    assert proto.get_config_snapshot(robot_config_pb2.DRIVE).source_name == "LIVE"
    assert proto.get_config_snapshot(robot_config_pb2.PLANNER).source_name == "BAKED"


def test_get_config_still_returns_only_the_values_unchanged():
    """``get_config()`` is now implemented on top of ``get_config_snapshot()``;
    its contract must not have moved for the callers that already use it."""
    conn = _FakeRobot()
    conn._group(robot_config_pb2.DRIVE).crawl_pulse = 0.08
    proto = NezhaProtocol(conn)

    values = proto.get_config(robot_config_pb2.DRIVE)

    assert values.crawl_pulse == pytest.approx(0.08)
    assert not isinstance(values, ConfigReadback)


# ---------------------------------------------------------------------------
# 2. verify=True -- set_config_field().
# ---------------------------------------------------------------------------


def test_verified_field_push_returns_normally_when_the_value_lands():
    conn = _FakeRobot()
    proto = NezhaProtocol(conn)

    ack = proto.set_config_field(robot_config_pb2.WHEEL_CONTROL, "pid_kp", 0.37,
                                 verify=True)

    assert ack is not None
    assert conn.get_config_calls == 1, "verify must read back exactly once"
    assert conn._group(robot_config_pb2.WHEEL_CONTROL).pid_kp == pytest.approx(0.37)


def test_verified_field_push_raises_when_the_value_lands_nowhere():
    """The actual 132-019 defect: acks OK, changes nothing."""
    conn = _FakeRobot(drop_pushes=True)
    proto = NezhaProtocol(conn)

    with pytest.raises(ConfigNotVerified) as excinfo:
        proto.set_config_field(robot_config_pb2.WHEEL_CONTROL, "pid_kp", 0.37,
                               verify=True)

    message = str(excinfo.value)
    assert "pid_kp" in message
    assert "0.37" in message
    assert "did NOT land" in message


def test_verified_field_push_raises_on_a_rejected_value():
    """A deliberately-rejected push (NAK/timeout) raises rather than
    returning a falsy value the caller may not check."""
    conn = _FakeRobot(ack_ok=False)
    proto = NezhaProtocol(conn)

    with pytest.raises(ConfigNotVerified, match="REJECTED"):
        proto.set_config_field(robot_config_pb2.WHEEL_CONTROL, "pid_kp", 0.37,
                               verify=True)


def test_verified_field_push_raises_on_an_unknown_field_without_sending():
    conn = _FakeRobot()
    proto = NezhaProtocol(conn)

    with pytest.raises(ConfigNotVerified, match="no such field"):
        proto.set_config_field(robot_config_pb2.DRIVE, "not_a_real_field", 1.0,
                               verify=True)
    assert conn.sent == [], "a push that cannot be built must not reach the wire"


def test_unverified_push_keeps_its_old_return_contract():
    """``verify`` defaults to False so library/sim/REPL callers are unaffected
    -- a rejected push still returns None rather than raising, and a
    successful one costs exactly one round trip."""
    conn = _FakeRobot(ack_ok=False)
    proto = NezhaProtocol(conn)
    assert proto.set_config_field(robot_config_pb2.DRIVE, "crawl_pulse", 0.1) is None

    ok_conn = _FakeRobot()
    ok_proto = NezhaProtocol(ok_conn)
    assert ok_proto.set_config_field(robot_config_pb2.DRIVE, "crawl_pulse", 0.1) is not None
    assert ok_conn.get_config_calls == 0, "an unverified push must not read back"


def test_verify_tolerates_float32_wire_rounding():
    """Config crosses the wire as float32, so a host double never comes back
    bit-identical. Verification must compare with a tolerance sized for that,
    or every verified push of a value like 0.1 would raise."""
    conn = _FakeRobot()
    proto = NezhaProtocol(conn)

    # 0.1 is not representable in binary floating point; the protobuf float
    # field rounds it to float32 and reads back as 0.10000000149011612.
    ack = proto.set_config_field(robot_config_pb2.DRIVE, "crawl_pulse", 0.1,
                                 verify=True)

    assert ack is not None


# ---------------------------------------------------------------------------
# 3. verify=True -- set_config_group().
# ---------------------------------------------------------------------------


def test_verified_group_push_returns_normally_when_every_field_lands():
    conn = _FakeRobot()
    proto = NezhaProtocol(conn)

    ack = proto.set_config_group(robot_config_pb2.OTOS, verify=True,
                                 offset_x=-47.7, linear_scale=1.0275)

    assert ack is not None
    assert conn.get_config_calls == 1
    group = conn._group(robot_config_pb2.OTOS)
    assert group.offset_x == pytest.approx(-47.7)
    assert group.linear_scale == pytest.approx(1.0275)


def test_verified_group_push_raises_when_the_group_lands_nowhere():
    conn = _FakeRobot(drop_pushes=True)
    proto = NezhaProtocol(conn)

    with pytest.raises(ConfigNotVerified) as excinfo:
        proto.set_config_group(robot_config_pb2.OTOS, verify=True,
                               offset_x=-47.7, linear_scale=1.0275)

    assert "did NOT land" in str(excinfo.value)


def test_verified_group_push_raises_on_an_unknown_target_without_sending():
    conn = _FakeRobot()
    proto = NezhaProtocol(conn)

    with pytest.raises(ConfigNotVerified, match="unknown target"):
        proto.set_config_group(999, verify=True, offset_x=1.0)
    assert conn.sent == []


def test_verification_failure_names_the_group_source_it_saw():
    """The error should say what the robot reported, not just that it
    disagreed -- a group reading BAKED after a push is the whole story."""
    conn = _FakeRobot(drop_pushes=True, source=robot_config_pb2.CONFIG_SOURCE_BAKED)
    proto = NezhaProtocol(conn)

    with pytest.raises(ConfigNotVerified, match="BAKED"):
        proto.set_config_field(robot_config_pb2.DRIVE, "crawl_pulse", 0.5, verify=True)


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
