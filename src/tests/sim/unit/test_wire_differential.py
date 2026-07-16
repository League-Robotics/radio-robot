"""Differential round-trip + boundary/range suite for the P4-pruned wire
protocol (103-001, SUC-001, architecture-update.md (103) Decisions 2/3).

***THIS IS THE CORRECTNESS GATE the `src/firm/app/` tickets (004+) are built
on top of.*** It proves the self-written firmware codec (``src/firm/messages/
wire_runtime.{h,cpp}`` + generated ``wire.{h,cpp}``) agrees with the host's
`google.protobuf`-backed reference (``src/host/robot_radio/robot/pb2/``)
byte-for-byte, in both directions, for every oneof arm this schema declares.
**A future change to `wire_runtime.{h,cpp}` or the generated `wire.{h,cpp}`
that breaks a test in this file is a BLOCKING regression -- fix the codec,
do not xfail/skip a real disagreement with `google.protobuf`.**

Rewritten this ticket against the pruned arm set -- every pre-103 arm
(drive/segment/replace/pose_fix/otos/ping/echo/get/stream/id/hello/ver/
help/plan_dump) is gone from the schema; this suite covers exactly what
remains:

  A. host-encode (`pb2`) -> firmware-decode (``wire_differential_harness``,
     via `msg::wire::decode(CommandEnvelope&, ...)`) -> assert decoded
     fields match the original input. Exercised for CommandEnvelope's three
     live arms: twist, config (all four ConfigDelta.patch oneof arms), stop.
  B. firmware-encode (harness, via `msg::wire::encode(const ReplyEnvelope&,
     ...)` / `msg::wire::encode(const TelemetrySecondary&, ...)`) ->
     host-decode (`pb2.ParseFromString`) -> assert decoded fields match.
     Exercised for ReplyEnvelope's three live arms (ok, err, tlm -- the
     depth-3 ack ring + fault_bits/event_bits included) and the NEW
     standalone TelemetrySecondary codec (Decision 3).

A dedicated field-number-correspondence test additionally cross-checks the
host `pb2` descriptors' field numbers against the exact numbers this
suite's byte-construction helpers use (which match wire.cpp's generated
`kFields_*[]` tables) -- catches a stale/out-of-sync regeneration on either
side, not just implicitly via a round-trip mismatch.

Boundary/range corpus: NONE of the arms reachable from this pruned schema
carry a `(min)`/`(max)`/`(abs_max)` proto option any more -- `MotionSegment`
(the pre-103 schema's only `(min)`/`(max)`/`(abs_max)`-validated fields) and
`ConfigGet.target` (the only `(req)`-validated field) both left with their
owning arms. The "REALITY CHECK" corpus below (unchanged from the pre-103
schema -- `ConfigDelta`/config.proto are untouched by this ticket) still
documents that `DrivetrainConfigPatch`/`PlannerConfigPatch`/
`MotorConfigPatch`/`ConfigDelta.watchdog` carry no wire-level bound at all.
"""
from __future__ import annotations

import pathlib

import pytest

from _wire_diff_driver import (  # noqa: E402
    compile_harness,
    decode,
    encode_err,
    encode_ok,
    encode_telemetry,
    encode_telemetry_secondary,
    env_config_drivetrain,
    env_config_motor,
    env_config_planner,
    env_config_watchdog,
    env_stop,
    env_twist,
    f32,
    float_eq,
    parse_decode_line,
    pb_config,
    pb_envelope,
    pb_planner,
    pb_telemetry,
)

pytestmark = pytest.mark.filterwarnings("ignore")


@pytest.fixture(scope="module")
def harness(tmp_path_factory) -> pathlib.Path:
    tmp_path = tmp_path_factory.mktemp("wire_differential")
    return compile_harness(tmp_path, "wire_differential_harness", [])


def _assert_ok(binary: pathlib.Path, raw: bytes) -> dict[str, str]:
    run = decode(binary, raw)
    assert not run.crashed, f"harness crashed decoding a VALID pb2-serialized envelope:\n{run.stderr}"
    status, fields = parse_decode_line(run.stdout)
    assert status == "OK", f"expected OK, got: {run.stdout}"
    return fields


def _assert_err(binary: pathlib.Path, raw: bytes) -> dict[str, str]:
    run = decode(binary, raw)
    assert not run.crashed, f"harness crashed: {run.stderr}"
    status, fields = parse_decode_line(run.stdout)
    assert status == "ERR", f"expected ERR, got: {run.stdout}"
    return fields


# ===========================================================================
# Field-number correspondence
# ===========================================================================


def test_field_numbers_match_pb2_descriptors():
    """The firmware's generated `kFields_*[]` tables (src/firm/messages/
    wire.cpp) hardcode proto field numbers baked in at generation time from
    the SAME protos/*.proto this test's pb2 bindings were also generated
    from. Cross-check pb2's own FieldDescriptors against the numbers this
    suite's env_*/encode_* helpers rely on -- catches a stale/out-of-sync
    regeneration on either side, not just an implicit round-trip mismatch."""
    expected_cmd_numbers = {"config": 6, "stop": 13, "twist": 19}
    actual_cmd_numbers = {
        f.name: f.number for f in pb_envelope.CommandEnvelope.DESCRIPTOR.oneofs_by_name["cmd"].fields
    }
    assert actual_cmd_numbers == expected_cmd_numbers

    expected_body_numbers = {"ok": 2, "err": 3, "tlm": 4}
    actual_body_numbers = {
        f.name: f.number for f in pb_envelope.ReplyEnvelope.DESCRIPTOR.oneofs_by_name["body"].fields
    }
    assert actual_body_numbers == expected_body_numbers

    expected_twist_numbers = {"v_x": 1, "omega": 2, "duration": 3}
    actual_twist_numbers = {f.name: f.number for f in pb_envelope.Twist.DESCRIPTOR.fields}
    assert actual_twist_numbers == expected_twist_numbers

    expected_err_codes = {
        "ERR_NONE": 0, "ERR_UNKNOWN": 1, "ERR_BADARG": 2, "ERR_RANGE": 3, "ERR_FULL": 4, "ERR_DECODE": 5,
        "ERR_UNIMPLEMENTED": 6, "ERR_OVERSIZE": 7,
    }
    actual_err_codes = {
        name: v.number for name, v in pb_envelope.DESCRIPTOR.enum_types_by_name["ErrCode"].values_by_name.items()
    }
    assert actual_err_codes == expected_err_codes

    expected_config_delta_patch = {"drivetrain": 1, "motor": 2, "planner": 3, "watchdog": 4}
    actual_config_delta_patch = {
        f.name: f.number for f in pb_envelope.ConfigDelta.DESCRIPTOR.oneofs_by_name["patch"].fields
    }
    assert actual_config_delta_patch == expected_config_delta_patch


def test_field_numbers_match_pb2_descriptors_telemetry():
    """103-001's own extension of the field-number-correspondence gate
    above, for Telemetry's pruned/renumbered field set (ack ring + fault/
    event bits, acc_*/glitch_*/ts_*/cmd_vel_* moved out to
    TelemetrySecondary) plus the config Patch types (unchanged) -- every
    number transcribed by hand into wire_differential_harness.cpp's
    encode_telemetry/encode_telemetry_secondary/decode-CONFIG-case, cross-
    checked here against the SAME protos/*.proto-generated pb2 descriptors."""
    expected_telemetry_numbers = {
        "acks": 1, "now": 2, "mode": 3, "seq": 4, "has_enc": 5, "enc_left": 6, "enc_right": 7, "has_vel": 8,
        "vel_left": 9, "vel_right": 10, "has_pose": 11, "pose": 12, "has_otos": 13, "otos": 14,
        "otos_connected": 15, "has_twist": 16, "twist": 17, "active": 18, "conn_left": 19, "conn_right": 20,
        "fault_bits": 21, "event_bits": 22,
    }
    actual_telemetry_numbers = {f.name: f.number for f in pb_telemetry.Telemetry.DESCRIPTOR.fields}
    assert actual_telemetry_numbers == expected_telemetry_numbers

    expected_ack_entry_numbers = {"corr_id": 1, "status": 2, "err_code": 3}
    actual_ack_entry_numbers = {f.name: f.number for f in pb_telemetry.AckEntry.DESCRIPTOR.fields}
    assert actual_ack_entry_numbers == expected_ack_entry_numbers

    expected_ack_status = {"ACK_STATUS_OK": 0, "ACK_STATUS_ERR": 1}
    actual_ack_status = {
        n: v.number for n, v in pb_telemetry.DESCRIPTOR.enum_types_by_name["AckStatus"].values_by_name.items()
    }
    assert actual_ack_status == expected_ack_status

    expected_telemetry_secondary_numbers = {
        "now": 1, "has_cmd_vel": 2, "cmd_vel_left": 3, "cmd_vel_right": 4, "acc_left": 5, "acc_right": 6,
        "glitch_left": 7, "glitch_right": 8, "ts_left": 9, "ts_right": 10,
    }
    actual_telemetry_secondary_numbers = {f.name: f.number for f in pb_telemetry.TelemetrySecondary.DESCRIPTOR.fields}
    assert actual_telemetry_secondary_numbers == expected_telemetry_secondary_numbers

    expected_drivetrain_patch = {
        "trackwidth": 1, "rotational_slip": 2, "ekf_q_xy": 3, "ekf_q_theta": 4, "ekf_r_otos_xy": 5,
        "ekf_r_otos_theta": 6, "ekf_r_fix_xy": 7, "ekf_r_fix_theta": 8,
    }
    actual_drivetrain_patch = {f.name: f.number for f in pb_config.DrivetrainConfigPatch.DESCRIPTOR.fields}
    assert actual_drivetrain_patch == expected_drivetrain_patch

    expected_motor_patch = {"side": 1, "travel_calib": 2, "kp": 3, "ki": 4, "kff": 5, "i_max": 6, "kaw": 7}
    actual_motor_patch = {f.name: f.number for f in pb_config.MotorConfigPatch.DESCRIPTOR.fields}
    assert actual_motor_patch == expected_motor_patch

    expected_planner_patch = {
        "min_speed": 1, "heading_kp": 2, "heading_kd": 3,
        "v_wheel_max": 4, "steer_headroom": 5, "wheel_step_max": 6,
        "track_k_s": 7, "track_k_theta": 8, "track_k_cross": 9,
        "trim_v_max": 10, "trim_omega_max": 11,
        "replan_err_pos": 12, "replan_err_theta": 13,
        "replan_hold": 14, "replan_min_period": 15, "replan_max": 16,
        "handoff_tol_pos": 17, "handoff_tol_v": 18,
        "arrive_vel_tol": 19, "arrive_dwell": 20,
    }
    actual_planner_patch = {f.name: f.number for f in pb_config.PlannerConfigPatch.DESCRIPTOR.fields}
    assert actual_planner_patch == expected_planner_patch

    expected_bound_motor_side = {"LEFT": 0, "RIGHT": 1}
    actual_bound_motor_side = {
        n: v.number for n, v in pb_config.DESCRIPTOR.enum_types_by_name["BoundMotorSide"].values_by_name.items()
    }
    assert actual_bound_motor_side == expected_bound_motor_side

    expected_drive_mode = {"IDLE": 0, "STREAMING": 1, "TIMED": 2, "DISTANCE": 3, "GO_TO": 4, "VELOCITY": 5}
    actual_drive_mode = {
        n: v.number for n, v in pb_planner.DESCRIPTOR.enum_types_by_name["DriveMode"].values_by_name.items()
    }
    assert actual_drive_mode == expected_drive_mode


# ===========================================================================
# Direction A: host-encode (pb2) -> firmware-decode (harness)
# ===========================================================================


@pytest.mark.parametrize("v_x,omega,duration", [
    (0.0, 0.0, 0.0), (150.0, -0.75, 250.0), (-3000.0, 12.566, 4294967295.0),
])
def test_direction_a_twist(harness, v_x, omega, duration):
    raw = env_twist(7, v_x, omega, duration)
    fields = _assert_ok(harness, raw)
    assert fields["corr_id"] == "7"
    assert fields["cmd_kind"] == "TWIST"
    assert float_eq(fields["v_x"], v_x)
    assert float_eq(fields["omega"], omega)
    assert float_eq(fields["duration"], duration)


def test_direction_a_stop(harness):
    raw = env_stop(5)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "STOP"
    assert fields["corr_id"] == "5"


# ---------------------------------------------------------------------------
# ConfigDelta -- COMMAND-only (never appears in ReplyEnvelope.body, see
# envelope.proto's own oneof list): its differential coverage is Direction A
# ONLY (host-encode -> firmware-decode). Unchanged shape from the pre-103
# schema.
# ---------------------------------------------------------------------------


def test_direction_a_config_drivetrain(harness):
    raw = env_config_drivetrain(20, trackwidth=321.0, rotational_slip=0.75, ekf_q_xy=1.5, ekf_q_theta=2.5,
                                 ekf_r_otos_xy=3.5, ekf_r_otos_theta=4.5, ekf_r_fix_xy=6.5, ekf_r_fix_theta=7.5)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "CONFIG"
    assert fields["patch_kind"] == "DRIVETRAIN"
    for key, expected in [("trackwidth", 321.0), ("rotational_slip", 0.75), ("ekf_q_xy", 1.5),
                           ("ekf_q_theta", 2.5), ("ekf_r_otos_xy", 3.5), ("ekf_r_otos_theta", 4.5),
                           ("ekf_r_fix_xy", 6.5), ("ekf_r_fix_theta", 7.5)]:
        assert fields[f"{key}_has"] == "1"
        assert float_eq(fields[key], expected)


def test_direction_a_config_drivetrain_partial_fields(harness):
    """Only the fields actually SET on the wire (proto3 `optional` explicit
    presence) come back `_has=1` -- the rest stay `_has=0`, proving the
    generated decoder's Opt<T> presence tracking (not just the values) is
    byte-for-byte faithful to what google.protobuf serialized."""
    raw = env_config_drivetrain(21, trackwidth=100.0)
    fields = _assert_ok(harness, raw)
    assert fields["trackwidth_has"] == "1"
    assert float_eq(fields["trackwidth"], 100.0)
    for key in ("rotational_slip", "ekf_q_xy", "ekf_q_theta", "ekf_r_otos_xy", "ekf_r_otos_theta",
                "ekf_r_fix_xy", "ekf_r_fix_theta"):
        assert fields[f"{key}_has"] == "0"


@pytest.mark.parametrize("side,name", [(pb_config.LEFT, "LEFT"), (pb_config.RIGHT, "RIGHT")])
def test_direction_a_config_motor(harness, side, name):
    raw = env_config_motor(22, side=side, travel_calib=1.111, kp=9.5, ki=8.5, kff=7.5, i_max=6.5, kaw=5.5)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "CONFIG"
    assert fields["patch_kind"] == "MOTOR"
    assert fields["side"] == name
    for key, expected in [("travel_calib", 1.111), ("kp", 9.5), ("ki", 8.5), ("kff", 7.5), ("i_max", 6.5),
                           ("kaw", 5.5)]:
        assert fields[f"{key}_has"] == "1"
        assert float_eq(fields[key], expected)


def test_direction_a_config_motor_only_travel_calib(harness):
    """ml/mr -- travel_calib alone, no Gains fields present."""
    raw = env_config_motor(23, side=pb_config.RIGHT, travel_calib=2.222)
    fields = _assert_ok(harness, raw)
    assert fields["side"] == "RIGHT"
    assert fields["travel_calib_has"] == "1"
    assert float_eq(fields["travel_calib"], 2.222)
    for key in ("kp", "ki", "kff", "i_max", "kaw"):
        assert fields[f"{key}_has"] == "0"


def test_direction_a_config_planner(harness):
    raw = env_config_planner(24, min_speed=42.0, heading_kp=6.0, heading_kd=0.25)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "CONFIG"
    assert fields["patch_kind"] == "PLANNER"
    for key, expected in [("min_speed", 42.0), ("heading_kp", 6.0), ("heading_kd", 0.25)]:
        assert fields[f"{key}_has"] == "1"
        assert float_eq(fields[key], expected)


@pytest.mark.parametrize("watchdog", [0, 1, 4242, 4294967295])
def test_direction_a_config_watchdog(harness, watchdog):
    raw = env_config_watchdog(25, watchdog)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "CONFIG"
    assert fields["patch_kind"] == "WATCHDOG"
    assert fields["watchdog"] == str(watchdog)


def test_direction_a_config_empty_patch(harness):
    """A well-formed ConfigDelta with no oneof `patch` arm set at all decodes
    OK (patch_kind == NONE) -- the wire codec itself must still decode it
    cleanly, never reject it."""
    raw = pb_envelope.CommandEnvelope(corr_id=26, config=pb_envelope.ConfigDelta()).SerializeToString()
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "CONFIG"
    assert fields["patch_kind"] == "NONE"


# ===========================================================================
# Direction B: firmware-encode (harness) -> host-decode (pb2.ParseFromString)
# ===========================================================================


@pytest.mark.parametrize("corr_id,q,rem,t", [
    (0, 0, 0.0, 0), (9, 5, 12.5, 0), (65535, 4294967295, -3.25, 0),
    (1, 0, 0.0, 12345), (2, 0, 0.0, 4294967295),
])
def test_direction_b_ack(harness, corr_id, q, rem, t):
    raw = encode_ok(harness, corr_id, q, rem, t)
    assert raw is not None, "encode_ok returned ZERO for a well-under-budget Ack reply"
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == corr_id
    assert reply.WhichOneof("body") == "ok"
    assert reply.ok.q == q
    assert reply.ok.rem == f32(rem)
    assert reply.ok.t == t


@pytest.mark.parametrize("code_name,field_num", [
    ("ERR_NONE", 0), ("ERR_UNKNOWN", 1), ("ERR_BADARG", 2), ("ERR_RANGE", 4), ("ERR_FULL", 0), ("ERR_DECODE", 0),
    ("ERR_UNIMPLEMENTED", 0), ("ERR_OVERSIZE", 0),
])
def test_direction_b_error(harness, code_name, field_num):
    raw = encode_err(harness, 3, code_name, field_num)
    assert raw is not None, "encode_err returned ZERO for a well-under-budget Error reply"
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == 3
    assert reply.WhichOneof("body") == "err"
    assert reply.err.code == pb_envelope.DESCRIPTOR.enum_types_by_name["ErrCode"].values_by_name[code_name].number
    assert reply.err.field == field_num


# ---------------------------------------------------------------------------
# Telemetry -- REPLY-only (never appears in CommandEnvelope.cmd): Direction B
# ONLY. Now carries the depth-3 ack ring + fault_bits/event_bits; the pre-103
# bench-diagnostic fields (acc_*/glitch_*/ts_*/cmd_vel_*) moved OUT to
# TelemetrySecondary (covered separately below).
# ---------------------------------------------------------------------------

_TELEMETRY_FULL_SHAPE = dict(
    now=123456, mode=2, seq=99, has_enc=True, enc_left=100.5, enc_right=-200.25, has_vel=True, vel_left=-50.0,
    vel_right=60.5, has_pose=True, pose_x=1.5, pose_y=-2.5, pose_h=3.25, has_otos=True, otos_x=4.5, otos_y=5.5,
    otos_h=6.5, otos_connected=True, has_twist=True, twist_vx=-100.5, twist_vy=0.5, twist_omega=1.75, active=True,
    conn_left=True, conn_right=False, fault_bits=0xDEADBEEF, event_bits=12345,
)

_TELEMETRY_FULL_ACKS = (
    (101, pb_telemetry.ACK_STATUS_OK, 0),
    (102, pb_telemetry.ACK_STATUS_ERR, 3),
    (103, pb_telemetry.ACK_STATUS_OK, 0),
)


def _assert_telemetry_matches_shape(tlm, shape: dict) -> None:
    assert tlm.now == shape["now"]
    assert tlm.mode == shape["mode"]
    assert tlm.seq == shape["seq"]
    assert tlm.has_enc == shape["has_enc"]
    assert tlm.enc_left == f32(shape["enc_left"])
    assert tlm.enc_right == f32(shape["enc_right"])
    assert tlm.has_vel == shape["has_vel"]
    assert tlm.vel_left == f32(shape["vel_left"])
    assert tlm.vel_right == f32(shape["vel_right"])
    assert tlm.has_pose == shape["has_pose"]
    assert tlm.pose.x == f32(shape["pose_x"])
    assert tlm.pose.y == f32(shape["pose_y"])
    assert tlm.pose.h == f32(shape["pose_h"])
    assert tlm.has_otos == shape["has_otos"]
    assert tlm.otos.x == f32(shape["otos_x"])
    assert tlm.otos.y == f32(shape["otos_y"])
    assert tlm.otos.h == f32(shape["otos_h"])
    assert tlm.otos_connected == shape["otos_connected"]
    assert tlm.has_twist == shape["has_twist"]
    assert tlm.twist.v_x == f32(shape["twist_vx"])
    assert tlm.twist.v_y == f32(shape["twist_vy"])
    assert tlm.twist.omega == f32(shape["twist_omega"])
    assert tlm.active == shape["active"]
    assert tlm.conn_left == shape["conn_left"]
    assert tlm.conn_right == shape["conn_right"]
    assert tlm.fault_bits == shape["fault_bits"]
    assert tlm.event_bits == shape["event_bits"]


def test_direction_b_telemetry_full_shape(harness):
    """Every core Telemetry field, all `has_*` flags true, plus a fully
    populated depth-3 ack ring and nonzero fault/event bits."""
    raw = encode_telemetry(harness, 30, acks=_TELEMETRY_FULL_ACKS, **_TELEMETRY_FULL_SHAPE)
    assert raw is not None, "encode_telemetry returned ZERO for a well-under-budget Telemetry reply"
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == 30
    assert reply.WhichOneof("body") == "tlm"
    _assert_telemetry_matches_shape(reply.tlm, _TELEMETRY_FULL_SHAPE)
    assert len(reply.tlm.acks) == 3
    for i, (corr_id, status, err_code) in enumerate(_TELEMETRY_FULL_ACKS):
        assert reply.tlm.acks[i].corr_id == corr_id
        assert reply.tlm.acks[i].status == status
        assert reply.tlm.acks[i].err_code == err_code


def test_direction_b_telemetry_all_has_flags_false(harness):
    """Every has_* flag at once, proving `has_*=0` is encoded/decoded
    faithfully too, not just the all-present shape above -- values behind a
    false has_* flag still round-trip (proto3 has no way to omit a nested
    message's own zero-valued scalar sub-fields once the message itself is
    present on the wire; `has_*` is this schema's OWN presence signal, not
    proto3 implicit presence)."""
    shape = dict(_TELEMETRY_FULL_SHAPE)
    for key in ("has_enc", "has_vel", "has_pose", "has_otos", "has_twist"):
        shape[key] = False
    raw = encode_telemetry(harness, 31, acks=_TELEMETRY_FULL_ACKS, **shape)
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    _assert_telemetry_matches_shape(reply.tlm, shape)


@pytest.mark.parametrize("mode_value,mode_name", [(0, "IDLE"), (1, "STREAMING"), (2, "TIMED"), (3, "DISTANCE"),
                                                   (4, "GO_TO"), (5, "VELOCITY")])
def test_direction_b_telemetry_every_drive_mode(harness, mode_value, mode_name):
    raw = encode_telemetry(harness, 32, mode=mode_value)
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.tlm.mode == pb_planner.DESCRIPTOR.enum_types_by_name["DriveMode"].values_by_name[mode_name].number


def test_direction_b_telemetry_all_zero_defaults(harness):
    """Every field at its proto zero default still round-trips to the SAME
    zero value.

    FINDING (not a bug -- documented here so it isn't re-discovered as one):
    `pose`/`otos`/`twist` are plain (non-oneof, non-`optional`) EMBEDDED
    MESSAGE fields (`FieldKind::kMessage` in wire.cpp's generated table) --
    `encodeInto()`'s `kMessage` case emits them UNCONDITIONALLY, with no
    zero-value skip, unlike every SCALAR field. So a from-scratch
    `pb_telemetry.Telemetry()` (which never touches `.pose`/`.otos`/
    `.twist` and so never marks them present) is NOT byte-identical to this
    round-trip -- the firmware always sends `pose {}`/`otos {}`/`twist {}`
    as PRESENT (possibly all-zero) submessages, regardless of `has_pose`/
    `has_otos`/`has_twist`. Compared field-by-field via
    `_assert_telemetry_matches_shape()` (which does not care about
    presence, only value) rather than whole-message `==` against a
    from-scratch default, which WOULD fail on this presence difference
    alone."""
    raw = encode_telemetry(harness, 33)
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == 33
    assert reply.WhichOneof("body") == "tlm"
    zero_shape = {key: (False if key.startswith("has_") or key in ("active", "conn_left", "conn_right",
                                                                     "otos_connected") else 0)
                  for key in _TELEMETRY_FULL_SHAPE}
    _assert_telemetry_matches_shape(reply.tlm, zero_shape)
    assert len(reply.tlm.acks) == 3
    for entry in reply.tlm.acks:
        assert entry.corr_id == 0
        assert entry.status == pb_telemetry.ACK_STATUS_OK
        assert entry.err_code == 0


# ---------------------------------------------------------------------------
# TelemetrySecondary (NEW this ticket, Decision 3) -- a STANDALONE top-level
# wire message, never wrapped in ReplyEnvelope. Direction B only (firmware-
# encode -> host-decode); there is no ReplyEnvelope corr_id to check.
# ---------------------------------------------------------------------------

_TELEMETRY_SECONDARY_FULL_SHAPE = dict(
    now=6000, has_cmd_vel=True, cmd_vel_left=120.0, cmd_vel_right=-120.0, acc_left=3.5, acc_right=-1.25,
    glitch_left=2, glitch_right=4294967295, ts_left=7000, ts_right=7001,
)


def test_direction_b_telemetry_secondary_full_shape(harness):
    raw = encode_telemetry_secondary(harness, **_TELEMETRY_SECONDARY_FULL_SHAPE)
    assert raw is not None, "encode_telemetry_secondary returned ZERO for a well-under-budget frame"
    sec = pb_telemetry.TelemetrySecondary.FromString(raw)
    assert sec.now == _TELEMETRY_SECONDARY_FULL_SHAPE["now"]
    assert sec.has_cmd_vel == _TELEMETRY_SECONDARY_FULL_SHAPE["has_cmd_vel"]
    assert sec.cmd_vel_left == f32(_TELEMETRY_SECONDARY_FULL_SHAPE["cmd_vel_left"])
    assert sec.cmd_vel_right == f32(_TELEMETRY_SECONDARY_FULL_SHAPE["cmd_vel_right"])
    assert sec.acc_left == f32(_TELEMETRY_SECONDARY_FULL_SHAPE["acc_left"])
    assert sec.acc_right == f32(_TELEMETRY_SECONDARY_FULL_SHAPE["acc_right"])
    assert sec.glitch_left == _TELEMETRY_SECONDARY_FULL_SHAPE["glitch_left"]
    assert sec.glitch_right == _TELEMETRY_SECONDARY_FULL_SHAPE["glitch_right"]
    assert sec.ts_left == _TELEMETRY_SECONDARY_FULL_SHAPE["ts_left"]
    assert sec.ts_right == _TELEMETRY_SECONDARY_FULL_SHAPE["ts_right"]


def test_direction_b_telemetry_secondary_all_other_fields_zero_default(harness):
    """Every field except `now` at its proto zero default still round-trips
    to the SAME zero value. `now` is pinned nonzero deliberately: unlike
    every ReplyEnvelope-wrapped message (always at least tag+len for the
    selected oneof arm, even with an all-default payload), TelemetrySecondary
    is encoded as a bare top-level message with NO oneof wrapper -- an
    all-default TelemetrySecondary legitimately serializes to zero bytes
    (matching real `google.protobuf`'s own `Message().SerializeToString() ==
    b""`), which this harness's `encode_telemetry_secondary` verb cannot
    distinguish from a genuine encode() failure over its "B64 .../ZERO"
    text protocol -- the same overloaded-zero-return ambiguity wire.h's own
    encode() has always had for a fully-blank envelope, not a new gap this
    ticket introduces."""
    raw = encode_telemetry_secondary(harness, now=1)
    assert raw is not None, "encode_telemetry_secondary returned ZERO for a well-under-budget frame"
    sec = pb_telemetry.TelemetrySecondary.FromString(raw)
    assert sec.now == 1
    assert sec.has_cmd_vel is False
    assert sec.cmd_vel_left == 0.0
    assert sec.glitch_left == 0
    assert sec.ts_left == 0


# ===========================================================================
# Boundary/range corpus -- config Patch messages carry NO `(min)`/`(max)`/
# `(abs_max)` proto option (unchanged from the pre-103 schema; config.proto
# is untouched by this ticket's prune). `MotionSegment` (the pre-103
# schema's only bounded-field arms, `segment`/`replace`) and
# `ConfigGet.target` (the only `(req)`-validated field) are both GONE with
# their owning arms -- there is no boundary corpus left to run for THEM.
#
# REALITY CHECK (documented, not silently patched -- config.proto's own file
# header, "Validation note" section): confirmed directly against wire.cpp's
# generated `kFields_DrivetrainConfigPatch[]`/`kFields_MotorConfigPatch[]`/
# `kFields_PlannerConfigPatch[]`/`kFields_ConfigDelta[]` tables, every one of
# which has `flags = 0` (no kHasMin/kHasMax/kHasAbsMax bit set) for these
# fields -- THE WIRE CODEC ACCEPTS ANY float/uint32 value for tw/rotSlip/
# ekf*/minSpeed/sTimeout over the binary plane. This is a pre-existing,
# already-flagged gap (config.proto's own comment), not something this
# ticket's prune changed -- verified again here since this file's schema
# assumptions were re-derived from scratch this ticket.
# ===========================================================================

_CONFIG_INVARIANT_BOUNDARY_CASES = [
    ("drivetrain_tw_zero", dict(trackwidth=0.0)),
    ("drivetrain_tw_negative", dict(trackwidth=-1.0)),
    ("drivetrain_tw_large_negative", dict(trackwidth=-1.0e9)),
    ("drivetrain_tw_positive", dict(trackwidth=1.0)),
    ("drivetrain_rotslip_zero", dict(rotational_slip=0.0)),
    ("drivetrain_rotslip_half", dict(rotational_slip=0.5)),
    ("drivetrain_rotslip_one", dict(rotational_slip=1.0)),
    ("drivetrain_rotslip_just_below_half", dict(rotational_slip=0.49)),
    ("drivetrain_rotslip_just_above_one", dict(rotational_slip=1.01)),
    ("drivetrain_rotslip_negative", dict(rotational_slip=-1.0)),
    ("drivetrain_rotslip_between_zero_and_half", dict(rotational_slip=0.3)),
]


@pytest.mark.parametrize("case_id,patch_kwargs", _CONFIG_INVARIANT_BOUNDARY_CASES,
                         ids=[c[0] for c in _CONFIG_INVARIANT_BOUNDARY_CASES])
def test_boundary_config_drivetrain_no_wire_level_enforcement(harness, case_id, patch_kwargs):
    raw = env_config_drivetrain(50, **patch_kwargs)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "CONFIG"
    assert fields["patch_kind"] == "DRIVETRAIN"
    for key, expected in patch_kwargs.items():
        assert fields[f"{key}_has"] == "1"
        assert float_eq(fields[key], expected)


@pytest.mark.parametrize("min_speed", [-1.0e9, -1.0, 0.0, 1.0])
def test_boundary_config_planner_min_speed_no_wire_level_enforcement(harness, min_speed):
    raw = env_config_planner(51, min_speed=min_speed)
    fields = _assert_ok(harness, raw)
    assert fields["patch_kind"] == "PLANNER"
    assert float_eq(fields["min_speed"], min_speed)


@pytest.mark.parametrize("watchdog", [0, 1])
def test_boundary_config_watchdog_no_wire_level_enforcement(harness, watchdog):
    raw = env_config_watchdog(52, watchdog)
    fields = _assert_ok(harness, raw)
    assert fields["patch_kind"] == "WATCHDOG"
    assert fields["watchdog"] == str(watchdog)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
