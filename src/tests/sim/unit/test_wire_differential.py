"""Differential round-trip + boundary/range suite for the wire protocol
(103-001, SUC-001, architecture-update.md (103) Decisions 2/3).

***THIS IS THE CORRECTNESS GATE the `src/firm/app/` tickets (004+) are built
on top of.*** It proves the self-written firmware codec (``src/firm/messages/
wire_runtime.{h,cpp}`` + generated ``wire.{h,cpp}``) agrees with the host's
`google.protobuf`-backed reference (``src/host/robot_radio/robot/pb2/``)
byte-for-byte, in both directions, for every oneof arm this schema declares.
**A future change to `wire_runtime.{h,cpp}` or the generated `wire.{h,cpp}`
that breaks a test in this file is a BLOCKING regression -- fix the codec,
do not xfail/skip a real disagreement with `google.protobuf`.**

Rewritten 115-009 (gut S1's own test-sweep/green-bar ticket) against the
frame-v2 Telemetry schema (115-003, `telemetry-frame-tightening-amendment-
to-gut-s1.md`) and the PLANNER-arm-deleted ConfigDelta (also 115-003);
updated 116-001 (MOVE protocol cutover) -- CommandEnvelope's three live
arms are now move/config/stop (`twist`, arm 19, deleted/reserved,
superseded by `move`, a fresh arm 21). This suite covers:

  A. host-encode (`pb2`) -> firmware-decode (``wire_differential_harness``,
     via `msg::wire::decode(CommandEnvelope&, ...)`) -> assert decoded
     fields match the original input. Exercised for CommandEnvelope's three
     live arms: move (MoveTwist|MoveWheels velocity x time|distance|angle
     stop), config (DRIVETRAIN/MOTOR/OTOS -- PLANNER deleted wholesale,
     115-003; WATCHDOG deleted, 116-001), stop.
  B. firmware-encode (harness, via `msg::wire::encode(const ReplyEnvelope&,
     ...)` / `msg::wire::encode(const TelemetrySecondary&, ...)`) ->
     host-decode (`pb2.ParseFromString`) -> assert decoded fields match.
     Exercised for ReplyEnvelope's three live arms (ok, err, tlm -- now one
     `flags` bit-string + a single ack slot + per-source timestamped
     `EncoderReading`/`OtosReading` objects) and the standalone
     TelemetrySecondary codec (Decision 3, unchanged by 115-003).

A dedicated field-number-correspondence test additionally cross-checks the
host `pb2` descriptors' field numbers against the exact numbers this
suite's byte-construction helpers use (which match wire.cpp's generated
`kFields_*[]` tables) -- catches a stale/out-of-sync regeneration on either
side, not just implicitly via a round-trip mismatch.

Boundary/range corpus: NONE of the arms reachable from this schema carry a
`(min)`/`(max)`/`(abs_max)` proto option any more -- `MotionSegment` (the
pre-103 schema's only `(min)`/`(max)`/`(abs_max)`-validated fields) and
`ConfigGet.target` (the only `(req)`-validated field) both left with their
owning arms. The "REALITY CHECK" corpus below still documents that
`DrivetrainConfigPatch`/`MotorConfigPatch`/`OtosConfigPatch` carry no
wire-level bound at all (`PlannerConfigPatch` went with 115-003's deletion,
`ConfigDelta.watchdog` with 116-001's -- there is no boundary corpus left to
run for either).
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
    env_config_otos,
    env_move_twist,
    env_move_wheels,
    env_stop,
    f32,
    float_eq,
    parse_decode_line,
    pb_config,
    pb_envelope,
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
    # "move" (109-003, the arc-command shape) is DELETED (115-003, gut S1
    # motion-stack excision) -- field 20 is `reserved`, not an active oneof
    # arm any more. "twist" (103-001) is DELETED (116-001, MOVE protocol
    # cutover) -- field 19 is `reserved`, not an active oneof arm any more.
    # `move` (116-001) is a FRESH arm at 21, a different shape from the
    # deleted 109-003 arc-command `Move` (see envelope.proto's own
    # CommandEnvelope header comment).
    expected_cmd_numbers = {"config": 6, "stop": 13, "move": 21}
    actual_cmd_numbers = {
        f.name: f.number for f in pb_envelope.CommandEnvelope.DESCRIPTOR.oneofs_by_name["cmd"].fields
    }
    assert actual_cmd_numbers == expected_cmd_numbers

    expected_body_numbers = {"ok": 2, "err": 3, "tlm": 4}
    actual_body_numbers = {
        f.name: f.number for f in pb_envelope.ReplyEnvelope.DESCRIPTOR.oneofs_by_name["body"].fields
    }
    assert actual_body_numbers == expected_body_numbers

    expected_move_twist_numbers = {"v_x": 1, "v_y": 2, "omega": 3}
    actual_move_twist_numbers = {f.name: f.number for f in pb_envelope.MoveTwist.DESCRIPTOR.fields}
    assert actual_move_twist_numbers == expected_move_twist_numbers

    expected_move_wheels_numbers = {"v_left": 1, "v_right": 2}
    actual_move_wheels_numbers = {f.name: f.number for f in pb_envelope.MoveWheels.DESCRIPTOR.fields}
    assert actual_move_wheels_numbers == expected_move_wheels_numbers

    expected_move_velocity_numbers = {"twist": 1, "wheels": 2}
    actual_move_velocity_numbers = {
        f.name: f.number for f in pb_envelope.Move.DESCRIPTOR.oneofs_by_name["velocity"].fields
    }
    assert actual_move_velocity_numbers == expected_move_velocity_numbers

    expected_move_stop_numbers = {"time": 3, "distance": 4, "angle": 5}
    actual_move_stop_numbers = {
        f.name: f.number for f in pb_envelope.Move.DESCRIPTOR.oneofs_by_name["stop"].fields
    }
    assert actual_move_stop_numbers == expected_move_stop_numbers

    expected_move_plain_numbers = {"timeout": 6, "replace": 7, "id": 8}
    actual_move_plain_numbers = {
        f.name: f.number for f in pb_envelope.Move.DESCRIPTOR.fields
        if f.name not in expected_move_velocity_numbers and f.name not in expected_move_stop_numbers
    }
    assert actual_move_plain_numbers == expected_move_plain_numbers

    # ERR_NOT_CONFIGURED (8, 114-001): composition root refused MOVE --
    # config-completeness gate not yet satisfied.
    expected_err_codes = {
        "ERR_NONE": 0, "ERR_UNKNOWN": 1, "ERR_BADARG": 2, "ERR_RANGE": 3, "ERR_FULL": 4, "ERR_DECODE": 5,
        "ERR_UNIMPLEMENTED": 6, "ERR_OVERSIZE": 7, "ERR_NOT_CONFIGURED": 8,
    }
    actual_err_codes = {
        name: v.number for name, v in pb_envelope.DESCRIPTOR.enum_types_by_name["ErrCode"].values_by_name.items()
    }
    assert actual_err_codes == expected_err_codes

    # "planner" (field 3, PlannerConfigPatch) DELETED (115-003) -- field 3 is
    # `reserved`, not an active oneof arm. "watchdog" (field 4) DELETED
    # (116-001, MOVE protocol cutover) -- field 4 is `reserved`, not an
    # active oneof arm any more (`ConfigTarget.CONFIG_WATCHDOG` stays
    # declared-unused). "otos" (109-004) is unaffected. "estimator" (117
    # ticket 003, EstimatorConfigPatch) is a fresh arm at field 6, the next
    # free number after `reserved 3, 4` and `otos = 5`.
    expected_config_delta_patch = {
        "drivetrain": 1, "motor": 2, "otos": 5, "estimator": 6,
    }
    actual_config_delta_patch = {
        f.name: f.number for f in pb_envelope.ConfigDelta.DESCRIPTOR.oneofs_by_name["patch"].fields
    }
    assert actual_config_delta_patch == expected_config_delta_patch


def test_field_numbers_match_pb2_descriptors_telemetry():
    """115-009's own extension of the field-number-correspondence gate
    above, for Telemetry's frame-v2 field set (115-003: one `flags`
    bit-string, one ack slot, per-source timestamped `EncoderReading`/
    `OtosReading` objects) plus the config Patch types -- every number
    transcribed by hand into wire_differential_harness.cpp's
    encode_telemetry/encode_telemetry_secondary/decode-CONFIG-case, cross-
    checked here against the SAME protos/*.proto-generated pb2 descriptors."""
    expected_telemetry_numbers = {
        "now": 1, "seq": 2, "mode": 3, "flags": 4, "ack_corr": 5, "ack_err": 6,
        "enc_left": 7, "enc_right": 8, "otos": 9, "pose": 10, "twist": 11, "line": 12, "color": 13,
    }
    actual_telemetry_numbers = {f.name: f.number for f in pb_telemetry.Telemetry.DESCRIPTOR.fields}
    assert actual_telemetry_numbers == expected_telemetry_numbers

    expected_encoder_reading_numbers = {"position": 1, "velocity": 2, "time": 3}
    actual_encoder_reading_numbers = {f.name: f.number for f in pb_telemetry.EncoderReading.DESCRIPTOR.fields}
    assert actual_encoder_reading_numbers == expected_encoder_reading_numbers

    expected_otos_reading_numbers = {
        "x": 1, "y": 2, "heading": 3, "v_x": 4, "v_y": 5, "omega": 6, "time": 7,
    }
    actual_otos_reading_numbers = {f.name: f.number for f in pb_telemetry.OtosReading.DESCRIPTOR.fields}
    assert actual_otos_reading_numbers == expected_otos_reading_numbers

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

    # PlannerConfigPatch DELETED wholesale (115-003, gut S1 motion-stack
    # excision) -- there is no descriptor left to pin. OtosConfigPatch
    # (109-004) is unaffected.
    expected_otos_patch = {
        "linear_scale": 1, "angular_scale": 2, "offset_x": 3, "offset_y": 4, "offset_yaw": 5, "init": 6,
    }
    actual_otos_patch = {f.name: f.number for f in pb_config.OtosConfigPatch.DESCRIPTOR.fields}
    assert actual_otos_patch == expected_otos_patch

    expected_bound_motor_side = {"LEFT": 0, "RIGHT": 1}
    actual_bound_motor_side = {
        n: v.number for n, v in pb_config.DESCRIPTOR.enum_types_by_name["BoundMotorSide"].values_by_name.items()
    }
    assert actual_bound_motor_side == expected_bound_motor_side

    # DriveMode relocated INTO telemetry.proto from the deleted planner.proto
    # (115-003 Decision 4) -- read from pb_telemetry now, not pb_planner
    # (which no longer exists).
    expected_drive_mode = {"IDLE": 0, "STREAMING": 1, "TIMED": 2, "DISTANCE": 3, "GO_TO": 4, "VELOCITY": 5}
    actual_drive_mode = {
        n: v.number for n, v in pb_telemetry.DESCRIPTOR.enum_types_by_name["DriveMode"].values_by_name.items()
    }
    assert actual_drive_mode == expected_drive_mode


# ===========================================================================
# Direction A: host-encode (pb2) -> firmware-decode (harness)
# ===========================================================================


@pytest.mark.parametrize("v_x,v_y,omega,stop_field,stop_value", [
    (0.0, 0.0, 0.0, "time", 0.0),
    (150.0, -25.0, -0.75, "time", 2000.0),
    (-3000.0, 0.0, 12.566, "distance", 300.0),
    (80.0, 0.0, 0.0, "angle", 1.5708),
])
def test_direction_a_move_twist(harness, v_x, v_y, omega, stop_field, stop_value):
    """MoveTwist velocity variant x every stop kind (116-001, MOVE protocol
    cutover) -- host-encode (pb2) -> firmware-decode round-trip."""
    raw = env_move_twist(7, v_x, v_y, omega, stop_field=stop_field, stop_value=stop_value, timeout=5000.0,
                          replace=True, move_id=42)
    fields = _assert_ok(harness, raw)
    assert fields["corr_id"] == "7"
    assert fields["cmd_kind"] == "MOVE"
    assert fields["velocity_kind"] == "TWIST"
    assert float_eq(fields["v_x"], v_x)
    assert float_eq(fields["v_y"], v_y)
    assert float_eq(fields["omega"], omega)
    assert fields["stop_kind"] == stop_field.upper()
    assert float_eq(fields[stop_field], stop_value)
    assert float_eq(fields["timeout"], 5000.0)
    assert fields["replace"] == "1"
    assert fields["id"] == "42"


@pytest.mark.parametrize("v_left,v_right,stop_field,stop_value", [
    (0.0, 0.0, "time", 0.0),
    (120.0, 100.0, "time", 1500.0),
    (-60.0, -60.0, "distance", 500.0),
    (90.0, -90.0, "angle", 3.14159),
])
def test_direction_a_move_wheels(harness, v_left, v_right, stop_field, stop_value):
    """MoveWheels velocity variant x every stop kind (116-001, MOVE protocol
    cutover) -- host-encode (pb2) -> firmware-decode round-trip."""
    raw = env_move_wheels(8, v_left, v_right, stop_field=stop_field, stop_value=stop_value, timeout=6000.0,
                           replace=False, move_id=21)
    fields = _assert_ok(harness, raw)
    assert fields["corr_id"] == "8"
    assert fields["cmd_kind"] == "MOVE"
    assert fields["velocity_kind"] == "WHEELS"
    assert float_eq(fields["v_left"], v_left)
    assert float_eq(fields["v_right"], v_right)
    assert fields["stop_kind"] == stop_field.upper()
    assert float_eq(fields[stop_field], stop_value)
    assert float_eq(fields["timeout"], 6000.0)
    assert fields["replace"] == "0"
    assert fields["id"] == "21"


def test_direction_a_move_reserved_twist_arm_ignored(harness):
    """The DELETED twist arm's old field number (19) round-trips as an
    unrecognized field -- skipped, never decoded into a live oneof arm
    (116-001; see envelope.proto's own reserved-list comment). Hand-spliced
    raw bytes, since pb2 no longer has a `Twist` message to construct."""
    from _wire_diff_driver import unknown_varint_field

    # A minimal length-delimited "field 19" wrapping a single float, spliced
    # after a normal corr_id field -- mirrors the raw-byte-splicing approach
    # the fuzz suite already uses for unknown-field coverage.
    raw = pb_envelope.CommandEnvelope(corr_id=7, stop=pb_envelope.Stop()).SerializeToString()
    raw += unknown_varint_field(19, 1)  # not a well-formed Twist, but any bytes at tag 19 must be skipped
    fields = _assert_ok(harness, raw)
    assert fields["corr_id"] == "7"
    assert fields["cmd_kind"] == "STOP"


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


def test_direction_a_config_otos(harness):
    """OtosConfigPatch's 5 Opt<float> fields (linear_scale/angular_scale/
    offset_x/offset_y/offset_yaw) round-trip host-encode -> firmware-decode,
    plus the plain (non-optional) `init` trigger bool."""
    raw = env_config_otos(24, linear_scale=1.01, angular_scale=0.99, offset_x=12.5, offset_y=-3.5,
                           offset_yaw=0.125, init=True)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "CONFIG"
    assert fields["patch_kind"] == "OTOS"
    for key, expected in [("linear_scale", 1.01), ("angular_scale", 0.99), ("offset_x", 12.5),
                           ("offset_y", -3.5), ("offset_yaw", 0.125)]:
        assert fields[f"{key}_has"] == "1"
        assert float_eq(fields[key], expected)
    assert fields["init"] == "1"


def test_direction_a_config_otos_init_false_and_no_optional_fields(harness):
    """`init` is a PLAIN bool (proto3 implicit presence, not Opt<float>) --
    it round-trips even when every optional calibration field is absent."""
    raw = env_config_otos(29, init=False)
    fields = _assert_ok(harness, raw)
    assert fields["patch_kind"] == "OTOS"
    assert fields["init"] == "0"
    for key in ("linear_scale", "angular_scale", "offset_x", "offset_y", "offset_yaw"):
        assert fields[f"{key}_has"] == "0"


def test_direction_a_config_reserved_watchdog_field_ignored(harness):
    """The DELETED ConfigDelta.watchdog arm's old field number (4) round-trips
    as an unrecognized field within the nested ConfigDelta message -- skipped,
    never decoded into a live oneof arm (116-001; see envelope.proto's own
    reserved-list comment). Hand-spliced raw bytes, since pb2 no longer has a
    `watchdog` field to construct."""
    from _wire_diff_driver import _tag, _varint

    # ConfigDelta{} with a spliced-in field 4 (varint) -- mirrors the same
    # raw-byte-splicing approach the fuzz suite uses for unknown-field
    # coverage, at the NESTED-message level this time.
    config_bytes = _tag(4, 0) + _varint(5000)
    config_field = _tag(6, 2) + _varint(len(config_bytes)) + config_bytes  # CommandEnvelope.cmd.config, field 6
    raw = pb_envelope.CommandEnvelope(corr_id=25).SerializeToString() + config_field
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "CONFIG"
    assert fields["patch_kind"] == "NONE"


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
# ONLY. Frame v2 (115-003): one `flags` bit-string (status+fault+event), a
# single `ack_corr`/`ack_err` slot (the depth-3 ack ring is gone), two
# per-source-timestamped `EncoderReading`s, one `OtosReading`, and the
# packed `line`/`color` words. Pre-103 bench-diagnostic fields (acc_*/
# glitch_*/ts_*/cmd_vel_*) stay OUT, in TelemetrySecondary (covered
# separately below).
# ---------------------------------------------------------------------------

# flags bit positions -- mirrors telemetry.proto's own Telemetry.flags
# comment table exactly (bits 16+ reserved). Grouped by the amendment
# issue's own three-group taxonomy (status / fault / event) so
# test_direction_b_telemetry_flags_* below can exercise at least one bit
# from each group individually, plus a combined value spanning all three.
_FLAG_OTOS_PRESENT = 1 << 0        # status
_FLAG_OTOS_CONNECTED = 1 << 1      # status
_FLAG_ACTIVE = 1 << 2              # status
_FLAG_CONN_LEFT = 1 << 3           # status
_FLAG_CONN_RIGHT = 1 << 4          # status
_FLAG_ACK_FRESH = 1 << 5           # status
_FLAG_FAULT_I2C_CLEARANCE = 1 << 6   # fault
_FLAG_FAULT_WEDGE_LATCH = 1 << 7     # fault
_FLAG_FAULT_I2C_NAK_TIMEOUT = 1 << 8  # fault
_FLAG_FAULT_MALFORMED_FRAME = 1 << 9  # fault
_FLAG_EVENT_DEADMAN_EXPIRED = 1 << 10  # event
_FLAG_EVENT_BOOT_READY = 1 << 11       # event
_FLAG_EVENT_CONFIG_APPLIED = 1 << 12   # event
_FLAG_LINE_PRESENT = 1 << 13       # status
_FLAG_COLOR_PRESENT = 1 << 14      # status
_FLAG_FAULT_MOVE_TIMEOUT = 1 << 15   # fault

_TELEMETRY_FULL_SHAPE = dict(
    now=123456, mode=2, seq=99,
    flags=(_FLAG_OTOS_PRESENT | _FLAG_ACTIVE | _FLAG_CONN_LEFT | _FLAG_ACK_FRESH | _FLAG_FAULT_WEDGE_LATCH
           | _FLAG_EVENT_BOOT_READY | _FLAG_LINE_PRESENT | _FLAG_COLOR_PRESENT),
    ack_corr=101, ack_err=0,
    enc_left_position=100.5, enc_left_velocity=-50.0, enc_left_time=123440,
    enc_right_position=-200.25, enc_right_velocity=60.5, enc_right_time=123440,
    otos_x=4.5, otos_y=5.5, otos_heading=6.5, otos_v_x=-100.5, otos_v_y=0.5, otos_omega=1.75, otos_time=123400,
    pose_x=1.5, pose_y=-2.5, pose_h=3.25,
    twist_v_x=-100.5, twist_v_y=0.5, twist_omega=1.75,
    line=0x04030201, color=0x0A0B0C0D,
)


def _assert_telemetry_matches_shape(tlm, shape: dict) -> None:
    assert tlm.now == shape["now"]
    assert tlm.mode == shape["mode"]
    assert tlm.seq == shape["seq"]
    assert tlm.flags == shape["flags"]
    assert tlm.ack_corr == shape["ack_corr"]
    assert tlm.ack_err == shape["ack_err"]
    assert tlm.enc_left.position == f32(shape["enc_left_position"])
    assert tlm.enc_left.velocity == f32(shape["enc_left_velocity"])
    assert tlm.enc_left.time == shape["enc_left_time"]
    assert tlm.enc_right.position == f32(shape["enc_right_position"])
    assert tlm.enc_right.velocity == f32(shape["enc_right_velocity"])
    assert tlm.enc_right.time == shape["enc_right_time"]
    assert tlm.otos.x == f32(shape["otos_x"])
    assert tlm.otos.y == f32(shape["otos_y"])
    assert tlm.otos.heading == f32(shape["otos_heading"])
    assert tlm.otos.v_x == f32(shape["otos_v_x"])
    assert tlm.otos.v_y == f32(shape["otos_v_y"])
    assert tlm.otos.omega == f32(shape["otos_omega"])
    assert tlm.otos.time == shape["otos_time"]
    assert tlm.pose.x == f32(shape["pose_x"])
    assert tlm.pose.y == f32(shape["pose_y"])
    assert tlm.pose.h == f32(shape["pose_h"])
    assert tlm.twist.v_x == f32(shape["twist_v_x"])
    assert tlm.twist.v_y == f32(shape["twist_v_y"])
    assert tlm.twist.omega == f32(shape["twist_omega"])
    assert tlm.line == shape["line"]
    assert tlm.color == shape["color"]


def test_direction_b_telemetry_full_shape(harness):
    """Every core Telemetry field, INCLUDING the two `EncoderReading`s and
    the `OtosReading` (each carrying its own sample/burst `time`, per the
    amendment issue's "per-source reading objects, timestamped" directive)
    and a `flags` value spanning all three bit groups (status/fault/event --
    see `_TELEMETRY_FULL_SHAPE`'s own construction above)."""
    raw = encode_telemetry(harness, 30, **_TELEMETRY_FULL_SHAPE)
    assert raw is not None, "encode_telemetry returned ZERO for a well-under-budget Telemetry reply"
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == 30
    assert reply.WhichOneof("body") == "tlm"
    _assert_telemetry_matches_shape(reply.tlm, _TELEMETRY_FULL_SHAPE)


@pytest.mark.parametrize("mode_value,mode_name", [(0, "IDLE"), (1, "STREAMING"), (2, "TIMED"), (3, "DISTANCE"),
                                                   (4, "GO_TO"), (5, "VELOCITY")])
def test_direction_b_telemetry_every_drive_mode(harness, mode_value, mode_name):
    raw = encode_telemetry(harness, 32, mode=mode_value)
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    # DriveMode relocated into telemetry.proto (115-003 Decision 4).
    assert reply.tlm.mode == pb_telemetry.DESCRIPTOR.enum_types_by_name["DriveMode"].values_by_name[mode_name].number


def test_direction_b_telemetry_all_zero_defaults(harness):
    """Every field at its proto zero default still round-trips to the SAME
    zero value.

    FINDING (not a bug -- documented here so it isn't re-discovered as one):
    `enc_left`/`enc_right`/`otos`/`pose`/`twist` are plain (non-oneof,
    non-`optional`) EMBEDDED MESSAGE fields (`FieldKind::kMessage` in
    wire.cpp's generated table) -- `encodeInto()`'s `kMessage` case emits
    them UNCONDITIONALLY, with no zero-value skip, unlike every SCALAR
    field. So a from-scratch `pb_telemetry.Telemetry()` (which never
    touches `.otos`/`.pose`/`.twist` and so never marks them present) is
    NOT byte-identical to this round-trip -- the firmware always sends
    `enc_left {}`/`otos {}`/`pose {}`/`twist {}` as PRESENT (possibly
    all-zero) submessages. Compared field-by-field via
    `_assert_telemetry_matches_shape()` (which does not care about
    presence, only value) rather than whole-message `==` against a
    from-scratch default, which WOULD fail on this presence difference
    alone."""
    raw = encode_telemetry(harness, 33)
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == 33
    assert reply.WhichOneof("body") == "tlm"
    zero_shape = {key: 0 for key in _TELEMETRY_FULL_SHAPE}
    _assert_telemetry_matches_shape(reply.tlm, zero_shape)


# ---------------------------------------------------------------------------
# flags semantics -- one bit from EACH of the three groups (status/fault/
# event) the amendment issue's own bit-table taxonomy declares, exercised
# individually (proves no cross-bit corruption/aliasing in the codec's
# uint32 handling) plus the full 16-bit span combined.
# ---------------------------------------------------------------------------


_FLAG_SINGLE_BIT_CASES = [
    (_FLAG_OTOS_PRESENT, "status:otos_present"),
    (_FLAG_ACTIVE, "status:active"),
    (_FLAG_CONN_RIGHT, "status:conn_right"),
    (_FLAG_ACK_FRESH, "status:ack_fresh"),
    (_FLAG_LINE_PRESENT, "status:line_present"),
    (_FLAG_FAULT_I2C_CLEARANCE, "fault:i2c_clearance"),
    (_FLAG_FAULT_WEDGE_LATCH, "fault:wedge_latch"),
    (_FLAG_FAULT_I2C_NAK_TIMEOUT, "fault:i2c_nak_timeout"),
    (_FLAG_FAULT_MOVE_TIMEOUT, "fault:move_timeout"),
    (_FLAG_EVENT_DEADMAN_EXPIRED, "event:deadman_expired"),
    (_FLAG_EVENT_BOOT_READY, "event:boot_ready"),
    (_FLAG_EVENT_CONFIG_APPLIED, "event:config_applied"),
]


@pytest.mark.parametrize("bit,name", _FLAG_SINGLE_BIT_CASES, ids=[c[1] for c in _FLAG_SINGLE_BIT_CASES])
def test_direction_b_telemetry_flags_single_bit_round_trips(harness, bit, name):
    """Each individual status/fault/event bit round-trips in isolation --
    no other bit in the 16-bit documented span flips."""
    raw = encode_telemetry(harness, 40, flags=bit)
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.tlm.flags == bit, f"{name}: expected ONLY bit {bit:#06x} set, got {reply.tlm.flags:#06x}"


def test_direction_b_telemetry_flags_all_groups_combined_round_trip(harness):
    """All 16 documented bits set at once -- status, fault, AND event groups
    share one uint32 with no aliasing between them."""
    all_bits = (
        _FLAG_OTOS_PRESENT | _FLAG_OTOS_CONNECTED | _FLAG_ACTIVE | _FLAG_CONN_LEFT | _FLAG_CONN_RIGHT
        | _FLAG_ACK_FRESH | _FLAG_FAULT_I2C_CLEARANCE | _FLAG_FAULT_WEDGE_LATCH | _FLAG_FAULT_I2C_NAK_TIMEOUT
        | _FLAG_FAULT_MALFORMED_FRAME | _FLAG_EVENT_DEADMAN_EXPIRED | _FLAG_EVENT_BOOT_READY
        | _FLAG_EVENT_CONFIG_APPLIED | _FLAG_LINE_PRESENT | _FLAG_COLOR_PRESENT | _FLAG_FAULT_MOVE_TIMEOUT
    )
    assert all_bits == 0xFFFF, "the 16 named bits above must cover exactly bits 0-15"
    raw = encode_telemetry(harness, 41, flags=all_bits)
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.tlm.flags == all_bits


# ---------------------------------------------------------------------------
# Reading `time` stamps -- monotonic across frames, and consistent with the
# frame's own `now` (readings are always sampled at-or-before the frame that
# reports them; the amendment issue's own Verification requirement).
# ---------------------------------------------------------------------------


def test_direction_b_telemetry_reading_times_consistent_with_frame_now(harness):
    """Every per-source reading's `time` is <= the frame's own `now` (a
    reading is always collected at-or-before the frame that carries it is
    assembled) and round-trips exactly -- no truncation/rounding through
    the wire for a `now`-adjacent uint32 timestamp."""
    now = 500_000
    raw = encode_telemetry(
        harness, 42, now=now,
        enc_left_time=now - 20, enc_right_time=now - 20, otos_time=now - 5,
    )
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.tlm.now == now
    assert reply.tlm.enc_left.time == now - 20 <= reply.tlm.now
    assert reply.tlm.enc_right.time == now - 20 <= reply.tlm.now
    assert reply.tlm.otos.time == now - 5 <= reply.tlm.now


def test_direction_b_telemetry_reading_times_monotonic_across_frames(harness):
    """A sequence of frames with strictly increasing `now`/reading times
    round-trips in the SAME strictly-increasing order -- the wire codec
    introduces no reordering or clamping of consecutive timestamps."""
    base = 1_000_000
    frames = []
    for i in range(5):
        now = base + i * 20
        raw = encode_telemetry(
            harness, 43 + i, now=now,
            enc_left_time=now, enc_right_time=now, otos_time=now,
        )
        frames.append(pb_envelope.ReplyEnvelope.FromString(raw).tlm)

    nows = [f.now for f in frames]
    enc_left_times = [f.enc_left.time for f in frames]
    otos_times = [f.otos.time for f in frames]
    assert nows == sorted(nows) and len(set(nows)) == len(nows)
    assert enc_left_times == nows
    assert otos_times == nows


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
# `kFields_OtosConfigPatch[]`/`kFields_ConfigDelta[]` tables, every one of
# which has `flags = 0` (no kHasMin/kHasMax/kHasAbsMax bit set) for these
# fields -- THE WIRE CODEC ACCEPTS ANY float/uint32 value for tw/rotSlip/
# ekf*/linear_scale/angular_scale over the binary plane. This is a
# pre-existing, already-flagged gap (config.proto's own comment), not
# something this ticket's prune changed -- verified again here since this
# file's schema assumptions were re-derived from scratch this ticket.
# `PlannerConfigPatch` (min_speed's own former owner) is DELETED wholesale
# (115-003); `ConfigDelta.watchdog` (the pre-116 sTimeout field) is DELETED
# (116-001) -- there is no boundary corpus left to run for either.
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


@pytest.mark.parametrize("linear_scale", [-1.0e9, -1.0, 0.0, 1.0])
def test_boundary_config_otos_linear_scale_no_wire_level_enforcement(harness, linear_scale):
    raw = env_config_otos(51, linear_scale=linear_scale)
    fields = _assert_ok(harness, raw)
    assert fields["patch_kind"] == "OTOS"
    assert float_eq(fields["linear_scale"], linear_scale)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
