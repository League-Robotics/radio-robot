"""Differential round-trip + boundary/range suite for ticket 095-006
(SUC-005, architecture-update.md M8 "Codec Test Harness").

***THIS IS THE CORRECTNESS GATE `BinaryChannel` (ticket 007) IS BUILT ON TOP
OF.*** It proves the self-written firmware codec (``source/messages/
wire_runtime.{h,cpp}`` + generated ``wire.{h,cpp}``, tickets 004/005) agrees
with the host's `google.protobuf`-backed reference (``host/robot_radio/
robot/pb2/``, ticket 002) byte-for-byte, in both directions, for every
oneof arm this sprint implements, and rejects out-of-bound field values with
the correct ``{fieldNumber, ErrCode}``. **A future change to
`wire_runtime.{h,cpp}` or the generated `wire.{h,cpp}` that breaks a test in
this file is a BLOCKING regression -- fix the codec, do not xfail/skip a
real disagreement with `google.protobuf`** (per the issue's #1 ranked risk
and this ticket's own acceptance criteria).

Two directions, per the ticket:
  A. host-encode (`pb2`) -> firmware-decode (``wire_differential_harness``,
     via `msg::wire::decode(CommandEnvelope&, ...)`) -> assert decoded
     fields match the original input. Exercised for every implemented
     `CommandEnvelope.cmd` arm: drive (twist/wheels/neutral variants),
     segment, replace (both `MotionSegment` "shapes" -- the geometry-only
     MOVE shape and the time/v/omega MOVER-teleop shape), stop, ping, echo,
     id (empty request).
  B. firmware-encode (harness, via `msg::wire::encode(const
     ReplyEnvelope&, ...)`) -> host-decode (`pb2.ParseFromString`) -> assert
     decoded fields match. `msg::wire::encode()` is `ReplyEnvelope`-only
     (the codec's own decode(Command)/encode(Reply) API is deliberately
     asymmetric -- wire.h's own header comment, Decision 4) -- exercised for
     the three `ReplyEnvelope.body` arms with actual field content this
     sprint's implemented arms can produce: `ok` (Ack), `err` (Error), `id`
     (DeviceId).

A dedicated field-number-correspondence test additionally cross-checks the
host `pb2` descriptors' field numbers against the exact numbers this suite's
byte-construction helpers use (which match wire.cpp's generated
`kFields_*[]` tables) -- the concrete mechanism ticket 006 asks for so a
future schema drift between `protos/*.proto`'s regenerated `pb2/` output and
a stale `wire.{h,cpp}` would be caught explicitly, not just implicitly via a
round-trip mismatch.

The boundary/range corpus (`test_boundary_*`) covers every `(min)`/`(max)`/
`(abs_max)`-validated field in this sprint's implemented arms -- per
ticket 001/Decision 5, that is exactly `MotionSegment`'s 11 bounded float
fields (`segment`/`replace` are the only implemented arms with validated
fields; the declared-only `ConfigGet.target`'s `(req)` field belongs to the
`get` arm, out of this ticket's implemented-arm scope and already exercised
by ticket 005's own `wire_codec_harness.cpp`). Each field gets `min-1`/`min`
(or `-abs_max-1`/`-abs_max`) and `max`/`max+1` (or `abs_max`/`abs_max+1`)
cases, asserting the exact accept/reject verdict and, on rejection, the
correct `{fieldNumber, ErrCode.ERR_RANGE}`.
"""
from __future__ import annotations

import pathlib

import pytest

from _wire_diff_driver import (  # noqa: E402
    build_motion_segment,
    compile_harness,
    decode,
    encode_echo_reply,
    encode_err,
    encode_id,
    encode_ok,
    env_drive_neutral,
    env_drive_twist,
    env_drive_wheels,
    env_echo,
    env_id_request,
    env_ping,
    env_replace,
    env_segment,
    env_stop,
    f32,
    float_eq,
    parse_decode_line,
    pb_envelope,
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
# Field-number correspondence (ticket 006's "Critical notes" instruction)
# ===========================================================================


def test_field_numbers_match_pb2_descriptors():
    """The firmware's generated `kFields_*[]` tables (source/messages/
    wire.cpp) hardcode proto field numbers baked in at generation time from
    the SAME protos/*.proto this test's pb2 bindings were also generated
    from. Cross-check pb2's own FieldDescriptors against the numbers this
    suite's env_* helpers rely on (transcribed by hand from wire.cpp when
    this ticket was implemented) -- catches a stale/out-of-sync regeneration
    on either side, not just an implicit round-trip mismatch."""
    expected_cmd_numbers = {
        "drive": 2, "segment": 3, "replace": 4, "config": 6, "pose": 7, "otos": 8,
        "ping": 9, "echo": 10, "get": 11, "stream": 12, "stop": 13, "id": 14,
    }
    actual_cmd_numbers = {
        f.name: f.number for f in pb_envelope.CommandEnvelope.DESCRIPTOR.oneofs_by_name["cmd"].fields
    }
    assert actual_cmd_numbers == expected_cmd_numbers

    expected_body_numbers = {"ok": 2, "err": 3, "tlm": 4, "cfg": 5, "evt": 6, "id": 7, "echo": 8}
    actual_body_numbers = {
        f.name: f.number for f in pb_envelope.ReplyEnvelope.DESCRIPTOR.oneofs_by_name["body"].fields
    }
    assert actual_body_numbers == expected_body_numbers

    expected_motion_segment_numbers = {
        "distance": 1, "direction": 2, "final_heading": 3, "speed_max": 4, "accel_max": 5, "jerk_max": 6,
        "yaw_rate_max": 7, "yaw_accel_max": 8, "yaw_jerk_max": 9, "time": 10, "v": 11, "omega": 12, "stream": 13,
    }
    from _wire_diff_driver import pb_motion  # local import: avoid unused-at-module-scope lint noise
    actual_motion_segment_numbers = {f.name: f.number for f in pb_motion.MotionSegment.DESCRIPTOR.fields}
    assert actual_motion_segment_numbers == expected_motion_segment_numbers

    expected_err_codes = {
        "ERR_NONE": 0, "ERR_UNKNOWN": 1, "ERR_BADARG": 2, "ERR_RANGE": 3, "ERR_FULL": 4, "ERR_DECODE": 5,
        "ERR_UNIMPLEMENTED": 6, "ERR_OVERSIZE": 7,
    }
    actual_err_codes = {
        name: v.number for name, v in pb_envelope.DESCRIPTOR.enum_types_by_name["ErrCode"].values_by_name.items()
    }
    assert actual_err_codes == expected_err_codes


# ===========================================================================
# Direction A: host-encode (pb2) -> firmware-decode (harness)
# ===========================================================================


def test_direction_a_drive_twist(harness):
    raw = env_drive_twist(7, 111.0, -22.5, 3.0, seed=True, standby=False)
    fields = _assert_ok(harness, raw)
    assert fields["corr_id"] == "7"
    assert fields["cmd_kind"] == "DRIVE"
    assert fields["control_kind"] == "TWIST"
    assert float_eq(fields["v_x"], 111.0)
    assert float_eq(fields["v_y"], -22.5)
    assert float_eq(fields["omega"], 3.0)
    assert fields["seed_has"] == "1" and fields["seed"] == "1"
    assert fields["standby_has"] == "1" and fields["standby"] == "0"


def test_direction_a_drive_wheels(harness):
    wheels = [(100.0, None), (101.5, -5.0), (None, 12.0), (0.0, 0.0)]
    raw = env_drive_wheels(1, wheels)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "DRIVE"
    assert fields["control_kind"] == "WHEELS"
    assert fields["w_count"] == "4"
    expected = [
        (True, 100.0, False, 0.0),
        (True, 101.5, True, -5.0),
        (False, 0.0, True, 12.0),
        (True, 0.0, True, 0.0),
    ]
    for i, (speed_has, speed, position_has, position) in enumerate(expected):
        assert fields[f"w{i}_speed_has"] == ("1" if speed_has else "0")
        assert fields[f"w{i}_position_has"] == ("1" if position_has else "0")
        if speed_has:
            assert float_eq(fields[f"w{i}_speed"], speed)
        if position_has:
            assert float_eq(fields[f"w{i}_position"], position)


@pytest.mark.parametrize("neutral_value,name", [(0, "BRAKE"), (1, "COAST")])
def test_direction_a_drive_neutral(harness, neutral_value, name):
    raw = env_drive_neutral(2, neutral_value)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "DRIVE"
    assert fields["control_kind"] == "NEUTRAL"
    assert fields["neutral"] == name


_MOTION_SEGMENT_GEOMETRY_SHAPE = dict(
    distance=-1500.0, direction=0.5, final_heading=1.2, speed_max=800.0, accel_max=2000.0, jerk_max=30000.0,
    yaw_rate_max=6.0, yaw_accel_max=40.0, yaw_jerk_max=100.0, time=0.0, v=0.0, omega=0.0, stream=False,
)
_MOTION_SEGMENT_TIME_VELOCITY_SHAPE = dict(
    distance=0.0, direction=0.0, final_heading=0.0, speed_max=0.0, accel_max=0.0, jerk_max=0.0, yaw_rate_max=0.0,
    yaw_accel_max=0.0, yaw_jerk_max=0.0, time=350.0, v=-450.0, omega=3.2, stream=True,
)


def _assert_motion_segment_fields(fields: dict[str, str], expected: dict):
    for key, value in expected.items():
        if key == "stream":
            assert fields["stream"] == ("1" if value else "0")
        else:
            assert float_eq(fields[key], value), f"{key}: got {fields[key]}, expected {f32(value)}"


@pytest.mark.parametrize("shape", [_MOTION_SEGMENT_GEOMETRY_SHAPE, _MOTION_SEGMENT_TIME_VELOCITY_SHAPE],
                         ids=["geometry_shape", "time_velocity_shape"])
def test_direction_a_segment(harness, shape):
    seg = build_motion_segment(**shape)
    raw = env_segment(3, seg)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "SEGMENT"
    _assert_motion_segment_fields(fields, shape)


@pytest.mark.parametrize("shape", [_MOTION_SEGMENT_GEOMETRY_SHAPE, _MOTION_SEGMENT_TIME_VELOCITY_SHAPE],
                         ids=["geometry_shape", "time_velocity_shape"])
def test_direction_a_replace(harness, shape):
    seg = build_motion_segment(**shape)
    raw = env_replace(4, seg)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "REPLACE"
    _assert_motion_segment_fields(fields, shape)


def test_direction_a_stop(harness):
    raw = env_stop(5)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "STOP"


def test_direction_a_ping(harness):
    raw = env_ping(6)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "PING"


@pytest.mark.parametrize("payload", [b"", b"\x00", b"hello", bytes(range(64))], ids=["empty", "nul", "ascii", "max64"])
def test_direction_a_echo(harness, payload):
    raw = env_echo(8, payload)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "ECHO"
    assert fields["payload_count"] == str(len(payload))
    assert fields["payload_hex"] == payload.hex()


def test_direction_a_id_request(harness):
    raw = env_id_request(9)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "ID"
    assert fields["corr_id"] == "9"


# ===========================================================================
# Direction B: firmware-encode (harness) -> host-decode (pb2.ParseFromString)
# ===========================================================================


@pytest.mark.parametrize("corr_id,q,rem,t", [
    (0, 0, 0.0, 0), (9, 5, 12.5, 0), (65535, 4294967295, -3.25, 0),
    # t (095-007, Ack schema-gap closure): PING's binary reply sets t to a
    # robot-clock timestamp (Ack{q=0,rem=0,t=<ms>}) -- these cases prove the
    # NEW field round-trips byte-for-byte against google.protobuf the same
    # way q/rem already do.
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


@pytest.mark.parametrize("model,name,serial,fw,proto_version", [
    ("NEZHA2", "bot1", 424242, "1.2.3", 3),
    ("", "", 0, "", 0),
    ("M" * 47, "N" * 47, 4294967295, "F" * 47, 255),
])
def test_direction_b_device_id(harness, model, name, serial, fw, proto_version):
    raw = encode_id(harness, 1, model, name, serial, fw, proto_version)
    assert raw is not None, "encode_id returned ZERO for a well-under-budget DeviceId reply"
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == 1
    assert reply.WhichOneof("body") == "id"
    assert reply.id.model == model
    assert reply.id.name == name
    assert reply.id.serial == serial
    assert reply.id.fw_version == fw
    assert reply.id.proto_version == proto_version


@pytest.mark.parametrize("payload", [b"", b"\x00", b"hello", bytes(range(64))],
                         ids=["empty", "nul", "ascii", "max64"])
def test_direction_b_echo_reply(harness, payload):
    """ReplyEnvelope.echo (095-007, schema-gap closure): BinaryChannel's
    ECHO reply carries `cmd.echo.payload` back verbatim -- mirrors
    handleEcho()'s text behavior. Direction B (firmware-encode ->
    host-decode) proof for the NEW oneof arm, same shape as
    test_direction_b_ack/test_direction_b_device_id above."""
    raw = encode_echo_reply(harness, 4, payload)
    assert raw is not None, "encode_echo_reply returned ZERO for a well-under-budget Echo reply"
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == 4
    assert reply.WhichOneof("body") == "echo"
    assert reply.echo.payload == payload


# ===========================================================================
# Boundary/range corpus -- every (min)/(max)/(abs_max)-validated field in
# this sprint's implemented arms (MotionSegment's 11 bounded floats, per
# Decision 5's text-constant transcription). min-1/min/max/max+1 (or
# -abs_max-1/-abs_max/abs_max/abs_max+1) per field.
# ===========================================================================

# (field_name, field_number, kind, bound-defining value(s))
_ABS_MAX_FIELDS = [
    ("distance", 1, 10000.0),
    ("direction", 2, 31.416),
    ("final_heading", 3, 31.416),
    ("v", 11, 3000.0),
    ("omega", 12, 12.566),
]
_MIN_MAX_FIELDS = [
    ("speed_max", 4, 0.0, 3000.0),
    ("accel_max", 5, 0.0, 6000.0),
    ("jerk_max", 6, 0.0, 60000.0),
    ("yaw_rate_max", 7, 0.0, 12.566),
    ("yaw_accel_max", 8, 0.0, 87.266),
    ("yaw_jerk_max", 9, 0.0, 349.066),
    ("time", 10, 0.0, 5000.0),
]


def _boundary_cases():
    cases = []
    for name, num, abs_max in _ABS_MAX_FIELDS:
        cases.append((name, num, abs_max, True, "abs_max"))
        cases.append((name, num, -abs_max, True, "-abs_max"))
        cases.append((name, num, abs_max + 1.0, False, "abs_max+1"))
        cases.append((name, num, -(abs_max + 1.0), False, "-(abs_max+1)"))
    for name, num, lo, hi in _MIN_MAX_FIELDS:
        cases.append((name, num, lo, True, "min"))
        cases.append((name, num, hi, True, "max"))
        cases.append((name, num, lo - 1.0, False, "min-1"))
        cases.append((name, num, hi + 1.0, False, "max+1"))
    return cases


@pytest.mark.parametrize("field_name,field_num,value,expect_accept,case_id", _boundary_cases(),
                         ids=[f"{c[0]}_{c[4]}" for c in _boundary_cases()])
def test_boundary_motion_segment(harness, field_name, field_num, value, expect_accept, case_id):
    seg = build_motion_segment(**{field_name: value})
    raw = env_segment(42, seg)
    if expect_accept:
        fields = _assert_ok(harness, raw)
        assert fields["cmd_kind"] == "SEGMENT"
        assert float_eq(fields[field_name], value)
    else:
        fields = _assert_err(harness, raw)
        assert fields["field"] == str(field_num), f"expected field {field_num}, got {fields}"
        assert fields["code"] == "ERR_RANGE", f"expected ERR_RANGE, got {fields}"


# Also exercise the SAME boundary corpus through the `replace` arm (same
# MotionSegment message, second oneof arm) -- cheap, and proves the bound
# validation is table-driven per FIELD, not accidentally per-arm.
@pytest.mark.parametrize("field_name,field_num,value,expect_accept,case_id", _boundary_cases(),
                         ids=[f"{c[0]}_{c[4]}" for c in _boundary_cases()])
def test_boundary_motion_segment_via_replace(harness, field_name, field_num, value, expect_accept, case_id):
    seg = build_motion_segment(**{field_name: value})
    raw = env_replace(43, seg)
    if expect_accept:
        fields = _assert_ok(harness, raw)
        assert fields["cmd_kind"] == "REPLACE"
    else:
        fields = _assert_err(harness, raw)
        assert fields["field"] == str(field_num)
        assert fields["code"] == "ERR_RANGE"


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
