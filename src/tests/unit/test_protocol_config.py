"""src/tests/unit/test_protocol_config.py — 104-001 (host command surface
completed: ``NezhaProtocol.config()``).

Sprint 103 shipped ``twist()``/``stop()`` host builders for the pruned P4
``CommandEnvelope`` schema (``envelope.proto``) but left ``ConfigDelta`` — a
schema-defined oneof arm since 103-001 — without a host-side builder
(103 Step 7 Open Question 3). This ticket adds ``NezhaProtocol.config()``,
mirroring ``twist()``/``stop()``'s fire-and-poll construction style
(``send_envelope_fast()`` + the existing 103-009 ack-ring matcher,
``wait_for_ack()`` — no new matching logic here, per this ticket's own
acceptance criteria).

Firmware dispatch behavior (confirmed directly against the merged 103
tree's ``src/firm/main.cpp``, resolving 103's Step 7 Open Question 3): the
``CmdKind::CONFIG`` case in the main-loop dispatch switch decodes the
envelope successfully but does NOT apply it — it always acks
``ACK_STATUS_ERR``/``ERR_UNIMPLEMENTED`` ("ConfigDelta runtime application
deferred this sprint"). Per this ticket's acceptance criteria, that means
test coverage here asserts the envelope/ack ROUND TRIP only — never config
application (a future ticket's scope) — so the ack-round-trip tests below
script an ``ERR_UNIMPLEMENTED`` ack, matching today's real firmware
behavior, not a hypothetical live-apply Ack.

Collected under ``src/tests/unit/`` — ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by
default.
"""

from __future__ import annotations

import pytest

from robot_radio.robot.pb2 import config_pb2, envelope_pb2
from robot_radio.robot.protocol import AckEntry, NezhaProtocol
from robot_radio.robot.pb2 import telemetry_pb2


class _FakeFastConn:
    """Minimal fake connection: implements ``send_envelope_fast()`` -- the
    same fake ``test_twist_stop_ack_matcher.py`` (103-009) uses for
    ``twist()``/``stop()`` -- plus ``wait_for_ack()`` (104-003: the shared
    matcher now lives on ``SerialConnection``, so ``NezhaProtocol.
    wait_for_ack()`` delegates to ``self._conn.wait_for_ack()``; this fake's
    own ``wait_for_ack()`` just returns whatever ``ack_result`` a test
    scripts, defaulting to ``None`` -- a bounded-timeout-with-no-match).
    ``config()`` calls nothing else on ``self._conn``."""

    def __init__(self) -> None:
        self.sent: list["envelope_pb2.CommandEnvelope"] = []
        self._next_corr_id = 0
        self.ack_result: "telemetry_pb2.AckEntry | None" = None

    def send_envelope_fast(self, envelope: "envelope_pb2.CommandEnvelope") -> int:
        self._next_corr_id += 1
        envelope.corr_id = self._next_corr_id
        self.sent.append(envelope)
        return self._next_corr_id

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "telemetry_pb2.AckEntry | None":
        return self.ack_result


# ---------------------------------------------------------------------------
# 1. config() — schema-level envelope construction, one target group at a
#    time, each asserted against a hand-built reference envelope's encoded
#    bytes.
# ---------------------------------------------------------------------------


def test_config_drivetrain_key_builds_correct_envelope_and_returns_corr_id():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    corr_id = proto.config(tw=128.0)

    assert corr_id == 1
    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(
            drivetrain=config_pb2.DrivetrainConfigPatch(trackwidth=128.0)))
    assert sent.SerializeToString() == expected.SerializeToString()
    assert sent.WhichOneof("cmd") == "config"
    assert sent.config.WhichOneof("patch") == "drivetrain"


def test_config_multiple_drivetrain_keys_land_on_one_patch():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    proto.config(tw=128.0, rotSlip=0.5, ekfQxy=0.1)

    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(
            drivetrain=config_pb2.DrivetrainConfigPatch(
                trackwidth=128.0, rotational_slip=0.5, ekf_q_xy=0.1)))
    assert sent.SerializeToString() == expected.SerializeToString()


def test_config_ml_builds_motor_patch_side_left():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    proto.config(ml=0.487)

    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(motor=config_pb2.MotorConfigPatch(
            side=config_pb2.LEFT, travel_calib=0.487)))
    assert sent.SerializeToString() == expected.SerializeToString()
    assert sent.config.WhichOneof("patch") == "motor"


def test_config_mr_builds_motor_patch_side_right():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    proto.config(mr=0.481)

    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(motor=config_pb2.MotorConfigPatch(
            side=config_pb2.RIGHT, travel_calib=0.481)))
    assert sent.SerializeToString() == expected.SerializeToString()


def test_config_pid_keys_alone_default_to_side_left():
    """pid.* fields are applied to BOTH bound motors server-side
    (MotorConfigPatch.side is meaningless for them, config.proto's own
    comment) -- with no ml/mr in the call, side still needs SOME wire
    value, and defaults to LEFT, mirroring set_config()'s own
    motor_left_patch branch."""
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    proto.config(**{"pid.kp": 1.5, "pid.ki": 0.2})

    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(motor=config_pb2.MotorConfigPatch(
            side=config_pb2.LEFT, kp=1.5, ki=0.2)))
    assert sent.SerializeToString() == expected.SerializeToString()


def test_config_ml_and_pid_keys_combine_on_one_motor_patch():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    proto.config(ml=0.487, **{"pid.kp": 1.5})

    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(motor=config_pb2.MotorConfigPatch(
            side=config_pb2.LEFT, travel_calib=0.487, kp=1.5)))
    assert sent.SerializeToString() == expected.SerializeToString()


def test_config_planner_key_builds_correct_envelope():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    proto.config(headingKp=6.0)

    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(
            planner=config_pb2.PlannerConfigPatch(heading_kp=6.0)))
    assert sent.SerializeToString() == expected.SerializeToString()
    assert sent.config.WhichOneof("patch") == "planner"


def test_config_watchdog_key_builds_correct_envelope():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    proto.config(sTimeout=1000)

    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1, config=envelope_pb2.ConfigDelta(watchdog=1000))
    assert sent.SerializeToString() == expected.SerializeToString()
    assert sent.config.WhichOneof("patch") == "watchdog"


def test_config_each_call_gets_a_fresh_corr_id():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    c1 = proto.config(tw=128.0)
    c2 = proto.config(sTimeout=1000)
    c3 = proto.config(headingKp=6.0)

    assert [c1, c2, c3] == [1, 2, 3]
    assert len(conn.sent) == 3


# ---------------------------------------------------------------------------
# 2. config() — input validation (empty / unknown key / multi-target)
# ---------------------------------------------------------------------------


def test_config_with_no_kwargs_raises_value_error():
    proto = NezhaProtocol(_FakeFastConn())

    with pytest.raises(ValueError):
        proto.config()


def test_config_with_unknown_key_raises_value_error():
    proto = NezhaProtocol(_FakeFastConn())

    with pytest.raises(ValueError):
        proto.config(notARealKey=1.0)


def test_config_spanning_two_targets_raises_value_error():
    """tw= (drivetrain) and pid.kp= (motor) cannot share one ConfigDelta --
    a single ConfigDelta carries only one patch oneof arm."""
    proto = NezhaProtocol(_FakeFastConn())

    with pytest.raises(ValueError):
        proto.config(tw=128.0, **{"pid.kp": 1.5})


def test_config_spanning_drivetrain_and_watchdog_raises_value_error():
    proto = NezhaProtocol(_FakeFastConn())

    with pytest.raises(ValueError):
        proto.config(tw=128.0, sTimeout=1000)


def test_config_invalid_call_sends_nothing():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    with pytest.raises(ValueError):
        proto.config()
    with pytest.raises(ValueError):
        proto.config(badkey=1)
    with pytest.raises(ValueError):
        proto.config(tw=1.0, sTimeout=1)

    assert conn.sent == []


# ---------------------------------------------------------------------------
# 3. config() -> wait_for_ack() round trip (103-009's existing ack-ring
#    matcher; no new matching logic added by this ticket). 104-003 promoted
#    the actual match/timeout algorithm out of NezhaProtocol into
#    SerialConnection.wait_for_ack() -- these tests now script the fake
#    connection's own wait_for_ack() (a raw telemetry_pb2.AckEntry or None)
#    rather than a batch of TLMFrame polls. The algorithm's own scenario
#    coverage (exact match, ring re-delivery tolerance, ring-wrap, bounded
#    timeout) lives in src/tests/unit/test_serial_conn_ack_ring.py.
# ---------------------------------------------------------------------------


def test_config_corr_id_round_trips_through_wait_for_ack():
    """End-to-end shape of a config() call: send, then confirm receipt via
    the SAME ack-ring matcher twist()/stop() already use. Scripts an
    ERR_UNIMPLEMENTED ack -- the confirmed (main.cpp, merged 103 tree)
    real firmware outcome for CONFIG today (runtime apply is deferred,
    not this ticket's scope)."""
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)
    corr_id = proto.config(tw=128.0)

    conn.ack_result = telemetry_pb2.AckEntry(
        corr_id=corr_id, status=telemetry_pb2.ACK_STATUS_ERR,
        err_code=envelope_pb2.ERR_UNIMPLEMENTED)

    ack = proto.wait_for_ack(corr_id, timeout=200)

    assert ack == AckEntry(
        corr_id=corr_id, ok=False, err_code=envelope_pb2.ERR_UNIMPLEMENTED,
        status=telemetry_pb2.ACK_STATUS_ERR)


def test_config_ack_returns_none_on_timeout_with_no_matching_corr_id():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)
    corr_id = proto.config(headingKp=6.0)
    # conn.ack_result stays at its default None -- the shared matcher timed
    # out with no matching corr_id (see SerialConnection.wait_for_ack()).

    ack = proto.wait_for_ack(corr_id, timeout=50)

    assert ack is None


# ---------------------------------------------------------------------------
# 4. otos_config() (109-004) -- the OL/OA/OI direct-patch-send builder.
#    Mirrors config()'s own "one envelope, one patch, fire-and-poll" shape
#    (section 1/3 above), but for OtosConfigPatch, which config()'s flat
#    _ALL_SET_KEYS vocabulary never covered (OL/OA/OI were never SET
#    key=value text verbs).
# ---------------------------------------------------------------------------


def test_otos_config_linear_scale_builds_correct_envelope():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    corr_id = proto.otos_config(linear_scale=1.05)

    assert corr_id == 1
    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(
            otos=config_pb2.OtosConfigPatch(linear_scale=1.05)))
    assert sent.SerializeToString() == expected.SerializeToString()
    assert sent.config.WhichOneof("patch") == "otos"


def test_otos_config_angular_scale_builds_correct_envelope():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    proto.otos_config(angular_scale=-0.98)

    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(
            otos=config_pb2.OtosConfigPatch(angular_scale=-0.98)))
    assert sent.SerializeToString() == expected.SerializeToString()


def test_otos_config_init_builds_correct_envelope():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    proto.otos_config(init=True)

    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(
            otos=config_pb2.OtosConfigPatch(init=True)))
    assert sent.SerializeToString() == expected.SerializeToString()


def test_otos_config_offset_fields_build_correct_envelope():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    proto.otos_config(offset_x=-47.7, offset_y=0.0, offset_yaw=1.5708)

    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(
            otos=config_pb2.OtosConfigPatch(
                offset_x=-47.7, offset_y=0.0, offset_yaw=1.5708)))
    assert sent.SerializeToString() == expected.SerializeToString()


def test_otos_config_fields_can_combine_on_one_patch():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    proto.otos_config(linear_scale=1.05, angular_scale=0.98, init=True)

    sent = conn.sent[0]
    expected = envelope_pb2.CommandEnvelope(
        corr_id=1,
        config=envelope_pb2.ConfigDelta(
            otos=config_pb2.OtosConfigPatch(
                linear_scale=1.05, angular_scale=0.98, init=True)))
    assert sent.SerializeToString() == expected.SerializeToString()


def test_otos_config_with_no_fields_raises_value_error():
    proto = NezhaProtocol(_FakeFastConn())

    with pytest.raises(ValueError):
        proto.otos_config()


def test_otos_config_invalid_call_sends_nothing():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    with pytest.raises(ValueError):
        proto.otos_config()

    assert conn.sent == []


def test_otos_config_each_call_gets_a_fresh_corr_id():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)

    c1 = proto.otos_config(linear_scale=1.05)
    c2 = proto.otos_config(angular_scale=0.98)
    c3 = proto.otos_config(init=True)

    assert [c1, c2, c3] == [1, 2, 3]
    assert len(conn.sent) == 3


def test_otos_config_corr_id_round_trips_through_wait_for_ack():
    """End-to-end shape of an otos_config() call: send, then confirm
    receipt via the SAME ack-ring matcher config()/twist()/stop() already
    use. Unlike config()'s ERR_UNIMPLEMENTED-for-everything-but-MOTOR
    scripting, RobotLoop::handleConfig DOES apply OTOS live (see that
    method's own comment) -- scripts a real ACK_STATUS_OK."""
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)
    corr_id = proto.otos_config(linear_scale=1.05)

    conn.ack_result = telemetry_pb2.AckEntry(
        corr_id=corr_id, status=telemetry_pb2.ACK_STATUS_OK)

    ack = proto.wait_for_ack(corr_id, timeout=200)

    assert ack == AckEntry(corr_id=corr_id, ok=True, err_code=0)


def test_otos_config_ack_returns_none_on_timeout_with_no_matching_corr_id():
    conn = _FakeFastConn()
    proto = NezhaProtocol(conn)
    corr_id = proto.otos_config(init=True)

    ack = proto.wait_for_ack(corr_id, timeout=50)

    assert ack is None
