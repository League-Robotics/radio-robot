"""src/tests/unit/test_repl_v5_verbs.py — the rogo repl's protocol-v5 rebuild
(2026-08-05, out-of-process rogo revival).

The pre-rebuild repl called ``NezhaProtocol.twist()`` — deleted at the
116-001 MOVE cutover — so every motion verb raised ``AttributeError``; its
``stop`` verb was documented as a panic stop when ``stop()`` had become the
PLANNED stop; and it had no estop verb at all (``clasi/issues/later/
A-repl-motion-verbs-dead-and-mcp-calibration-push-noops.md``). These tests
pin each verb to the live v5 primitive it must build, on a recording fake
protocol (``_fake_rogo_proto.FakeRogoProto``) behind a real ``RogoSession``.
"""
from __future__ import annotations

import io
import math

import pytest

from robot_radio.io import repl as repl_mod
from robot_radio.io.repl import HALT_VERBS, RogoSession, dispatch

from _fake_rogo_proto import FakeRogoProto


def make_session() -> tuple[RogoSession, FakeRogoProto, io.StringIO]:
    proto = FakeRogoProto()
    session = RogoSession.from_protocol(proto)
    buf = io.StringIO()
    session.out = buf
    session.errout = buf
    return session, proto, buf


def call(name: str, proto: FakeRogoProto) -> dict:
    """The single recorded call of ``name`` (fails if zero or many)."""
    matches = [args for (verb, *rest) in proto.calls for args in rest if verb == name]
    assert len(matches) == 1, f"expected one {name} call, got {proto.calls}"
    return matches[0]


# -- motion verbs -----------------------------------------------------------

def test_twist_builds_time_stopped_move_twist():
    session, proto, buf = make_session()
    result = dispatch(session, "twist 150 0.5 1000")
    assert result.error is None
    args = call("move_twist", proto)
    assert args["v_x"] == 150 and args["v_y"] == 0.0 and args["omega"] == 0.5
    assert args["stop_time"] == 1000
    assert args["timeout"] >= 1000  # safety backstop present and sane
    assert "OK" in buf.getvalue()


def test_wheels_builds_teleop_wheels_arm():
    session, proto, buf = make_session()
    assert dispatch(session, "wheels 100 -100 500").error is None
    args = call("wheels", proto)
    assert (args["v_left"], args["v_right"], args["duration"]) == (100, -100, 500)


def test_drive_builds_distance_stopped_move_wheels_and_waits_completion():
    session, proto, buf = make_session()
    assert dispatch(session, "drive 200 100").error is None
    args = call("move_wheels", proto)
    assert args["v_left"] == 100 and args["v_right"] == 100
    assert args["stop_distance"] == 200
    assert args["move_id"] != 0  # completion ack rides Move.id
    out = buf.getvalue()
    assert "OK" in out and "DONE" in out


def test_drive_negative_distance_drives_backward():
    session, proto, _ = make_session()
    assert dispatch(session, "drive -200 100").error is None
    args = call("move_wheels", proto)
    assert args["v_left"] == -100 and args["v_right"] == -100
    assert args["stop_distance"] == 200  # stop condition is |arc length|


def test_drive_nowait_skips_completion_wait():
    session, proto, buf = make_session()
    assert dispatch(session, "drive 200 100 nowait").error is None
    assert "DONE" not in buf.getvalue()
    assert "OK" in buf.getvalue()  # enqueue ack still confirmed


def test_two_drives_use_distinct_never_reused_move_ids():
    session, proto, _ = make_session()
    dispatch(session, "drive 100 100")
    dispatch(session, "drive 100 100")
    ids = [args["move_id"] for (verb, args) in proto.calls if verb == "move_wheels"]
    assert len(ids) == 2 and ids[0] != ids[1]


def test_turn_ccw_commands_negative_omega_with_angle_stop():
    """+deg = world CCW; measured 2026-07-29 (playfield-testing.md): positive
    commanded omega DECREASES world yaw, so CCW requires omega < 0."""
    session, proto, _ = make_session()
    assert dispatch(session, "turn 90 45").error is None
    args = call("move_twist", proto)
    assert args["v_x"] == 0.0
    assert args["omega"] == pytest.approx(-math.radians(45))
    assert args["stop_angle"] == pytest.approx(math.pi / 2)
    assert args["move_id"] != 0


def test_turn_cw_commands_positive_omega():
    session, proto, _ = make_session()
    assert dispatch(session, "turn -90 45").error is None
    args = call("move_twist", proto)
    assert args["omega"] == pytest.approx(math.radians(45))
    assert args["stop_angle"] == pytest.approx(math.pi / 2)


# -- stop vs estop ----------------------------------------------------------

def test_stop_is_the_planned_stop_and_says_so():
    session, proto, buf = make_session()
    assert dispatch(session, "stop").error is None
    assert [v for v, *_ in proto.calls if v in ("stop", "estop")] == ["stop"]
    assert "planned" in buf.getvalue()


def test_estop_verb_exists_and_verifies_active_cleared():
    session, proto, buf = make_session()
    assert dispatch(session, "estop").error is None
    assert proto.estop_count == 1
    assert "VERIFIED" in buf.getvalue()


def test_halt_is_an_alias_for_estop():
    session, proto, _ = make_session()
    assert dispatch(session, "halt").error is None
    assert proto.estop_count == 1
    assert HALT_VERBS == {"estop", "halt"}


def test_estop_reissues_while_active_flag_stays_set(monkeypatch):
    """The single-write estop failure mode (measured 2026-08-03: 5 of 6 lone
    attempts failed): if telemetry keeps reporting active, the verb must
    re-issue rather than declare success."""
    monkeypatch.setattr(repl_mod, "_ESTOP_VERIFY_WINDOW", 40)
    session, proto, buf = make_session()
    proto.estop_leaves_active = True
    assert dispatch(session, "estop").error is None
    assert proto.estop_count == repl_mod._ESTOP_MAX_ROUNDS
    assert "NOT VERIFIED" in buf.getvalue()
    assert "MAY STILL BE MOVING" in buf.getvalue()


# -- cleartext verbs --------------------------------------------------------

def test_ping_round_trips_cleartext():
    session, proto, buf = make_session()
    assert dispatch(session, "ping").error is None
    assert ("send_fast", "PING") in proto.calls
    assert "PONG" in buf.getvalue()


def test_id_ver_hello_round_trip_cleartext():
    session, proto, buf = make_session()
    for verb in ("id", "ver", "hello"):
        assert dispatch(session, verb).error is None
    out = buf.getvalue()
    assert "ID:" in out and "VER:" in out and "DEVICE:" in out


# -- config / raw / dispatch ------------------------------------------------

def test_config_pushes_each_key_via_set_config_field():
    session, proto, buf = make_session()
    assert dispatch(session, "config pid.kp=0.4").error is None
    pushes = [c for c in proto.calls if c[0] == "set_config_field"]
    assert len(pushes) == 1 and pushes[0][3] == 0.4
    assert "OK" in buf.getvalue()


def test_config_unknown_key_errors_with_no_wire_traffic():
    session, proto, _ = make_session()
    result = dispatch(session, "config bogus.key=1")
    assert result.error is not None
    assert not [c for c in proto.calls if c[0] == "set_config_field"]


def test_raw_move_requires_timeout():
    session, proto, _ = make_session()
    result = dispatch(session, "raw move v_x=100 time=500")
    assert result.error is not None and "timeout" in result.error
    assert not [c for c in proto.calls if c[0] == "move_twist"]


def test_raw_move_wheels_variant():
    session, proto, _ = make_session()
    result = dispatch(session, "raw move v_left=80 v_right=-80 angle=1.57 timeout=3000")
    assert result.error is None
    args = call("move_wheels", proto)
    assert args["v_left"] == 80 and args["v_right"] == -80
    assert args["stop_angle"] == pytest.approx(1.57)


def test_raw_estop_sends_the_estop_arm():
    session, proto, _ = make_session()
    assert dispatch(session, "raw estop").error is None
    assert proto.estop_count == 1


def test_dispatch_unknown_verb_reports_error_and_continues():
    session, _, _ = make_session()
    result = dispatch(session, "frobnicate 1 2")
    assert result.error is not None and not result.exit


def test_dispatch_quit_requests_exit():
    session, _, _ = make_session()
    assert dispatch(session, "quit").exit is True


def test_no_verb_touches_the_deleted_v3_twist_method():
    """The defect this rebuild closes: the old repl called proto.twist(),
    which does not exist on the v5 surface. FakeRogoProto deliberately has
    no twist() attribute, so any regression raises here."""
    session, proto, _ = make_session()
    for line in ("twist 100 0 500", "drive 100 100 nowait", "turn 45",
                 "wheels 50 50 200", "stop", "estop"):
        dispatch(session, line)
    assert not hasattr(proto, "twist")
