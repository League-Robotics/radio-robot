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
    encode_cfg_drivetrain,
    encode_cfg_motor,
    encode_cfg_planner,
    encode_cfg_watchdog,
    encode_echo_reply,
    encode_err,
    encode_helptext,
    encode_id,
    encode_ok,
    encode_telemetry,
    env_config_drivetrain,
    env_config_motor,
    env_config_planner,
    env_config_watchdog,
    env_drive_neutral,
    env_drive_twist,
    env_drive_wheels,
    env_echo,
    env_help_request,
    env_hello_request,
    env_id_request,
    env_ping,
    env_replace,
    env_segment,
    env_stop,
    env_ver_request,
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
        # pose_fix (099-004): retypes the formerly-declared-only `pose`
        # arm (was SetPose) to PoseFix -- same field number 7.
        "drive": 2, "segment": 3, "replace": 4, "config": 6, "pose_fix": 7, "otos": 8,
        "ping": 9, "echo": 10, "get": 11, "stream": 12, "stop": 13, "id": 14,
        # hello/ver/help (stakeholder-directed 6-verb minimal command
        # surface, 2026-07-10).
        "hello": 15, "ver": 16, "help": 17,
        # plan_dump (100-001, motion-stack-v2 M9): declared-only, replies
        # ERR_UNIMPLEMENTED until ticket 009.
        "plan_dump": 18,
    }
    actual_cmd_numbers = {
        f.name: f.number for f in pb_envelope.CommandEnvelope.DESCRIPTOR.oneofs_by_name["cmd"].fields
    }
    assert actual_cmd_numbers == expected_cmd_numbers

    expected_body_numbers = {
        "ok": 2, "err": 3, "tlm": 4, "cfg": 5, "evt": 6, "id": 7, "echo": 8,
        # helptext (stakeholder-directed 6-verb minimal command surface,
        # 2026-07-10) -- HELP's reply; hello/ver reuse the existing `id` arm.
        "helptext": 9,
        # plan/trace (100-001, motion-stack-v2 M9): declared-only, replies
        # ERR_UNIMPLEMENTED (plan) until ticket 009.
        "plan": 10, "trace": 11,
    }
    actual_body_numbers = {
        f.name: f.number for f in pb_envelope.ReplyEnvelope.DESCRIPTOR.oneofs_by_name["body"].fields
    }
    assert actual_body_numbers == expected_body_numbers

    expected_motion_segment_numbers = {
        "distance": 1, "direction": 2, "final_heading": 3, "speed_max": 4, "accel_max": 5, "jerk_max": 6,
        "yaw_rate_max": 7, "yaw_accel_max": 8, "yaw_jerk_max": 9, "time": 10, "v": 11, "omega": 12, "stream": 13,
        # arc/pivot primitive fields (100-001, motion-stack-v2 M1) --
        # declared only, no C++ consumer until ticket 007.
        "arc_length": 14, "delta_heading": 15, "exit_speed": 16, "primitive": 17,
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


def test_field_numbers_match_pb2_descriptors_096_006_new_messages():
    """096-006's own extension of the field-number-correspondence gate above,
    for the three messages this ticket adds differential coverage for
    (Telemetry/ConfigDelta/ConfigSnapshot) plus the three curated Patch
    types and the two new enums they reference -- every number transcribed
    by hand into wire_differential_harness.cpp's encode_telemetry/
    encode_cfg_*/decode-CONFIG-case when this ticket was implemented,
    cross-checked here against the SAME protos/*.proto-generated pb2
    descriptors, catching a stale/out-of-sync regeneration on either side."""
    expected_telemetry_numbers = {
        "now": 1, "mode": 2, "seq": 3, "has_enc": 4, "enc_left": 5, "enc_right": 6, "has_vel": 7,
        "vel_left": 8, "vel_right": 9, "has_cmd_vel": 10, "cmd_vel_left": 11, "cmd_vel_right": 12,
        "has_pose": 13, "pose": 14, "has_otos": 15, "otos": 16, "otos_connected": 17, "has_twist": 18,
        "twist": 19, "acc_left": 20, "acc_right": 21, "active": 22, "conn_left": 23, "conn_right": 24,
        "glitch_left": 25, "glitch_right": 26, "ts_left": 27, "ts_right": 28,
    }
    actual_telemetry_numbers = {f.name: f.number for f in pb_telemetry.Telemetry.DESCRIPTOR.fields}
    assert actual_telemetry_numbers == expected_telemetry_numbers

    expected_config_delta_patch = {"drivetrain": 1, "motor": 2, "planner": 3, "watchdog": 4}
    actual_config_delta_patch = {
        f.name: f.number for f in pb_envelope.ConfigDelta.DESCRIPTOR.oneofs_by_name["patch"].fields
    }
    assert actual_config_delta_patch == expected_config_delta_patch

    expected_config_snapshot_patch = {"drivetrain": 2, "motor": 3, "planner": 4, "watchdog": 5}
    actual_config_snapshot_patch = {
        f.name: f.number for f in pb_envelope.ConfigSnapshot.DESCRIPTOR.oneofs_by_name["patch"].fields
    }
    assert actual_config_snapshot_patch == expected_config_snapshot_patch
    assert pb_envelope.ConfigSnapshot.DESCRIPTOR.fields_by_name["target"].number == 1

    expected_drivetrain_patch = {
        "trackwidth": 1, "rotational_slip": 2, "ekf_q_xy": 3, "ekf_q_theta": 4, "ekf_r_otos_xy": 5,
        "ekf_r_otos_theta": 6,
        # 099-008: ekf_r_fix_xy/ekf_r_fix_theta -- the delayed camera-fix's
        # own UNGATED noise pair.
        "ekf_r_fix_xy": 7, "ekf_r_fix_theta": 8,
    }
    actual_drivetrain_patch = {f.name: f.number for f in pb_config.DrivetrainConfigPatch.DESCRIPTOR.fields}
    assert actual_drivetrain_patch == expected_drivetrain_patch

    expected_motor_patch = {"side": 1, "travel_calib": 2, "kp": 3, "ki": 4, "kff": 5, "i_max": 6, "kaw": 7}
    actual_motor_patch = {f.name: f.number for f in pb_config.MotorConfigPatch.DESCRIPTOR.fields}
    assert actual_motor_patch == expected_motor_patch

    expected_planner_patch = {
        "min_speed": 1, "heading_kp": 2, "heading_kd": 3,
        # v_wheel_max..arrive_dwell (100-001, motion-stack-v2 M1):
        # Drive::Limits' wire-tunable subset of PlannerConfig fields 15-31.
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

    expected_config_target = {
        "CONFIG_DRIVETRAIN": 0, "CONFIG_MOTOR_LEFT": 1, "CONFIG_MOTOR_RIGHT": 2, "CONFIG_PLANNER": 3,
        "CONFIG_WATCHDOG": 4,
    }
    actual_config_target = {
        n: v.number for n, v in pb_config.DESCRIPTOR.enum_types_by_name["ConfigTarget"].values_by_name.items()
    }
    assert actual_config_target == expected_config_target

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


# ---------------------------------------------------------------------------
# hello/ver/help (stakeholder-directed 6-verb minimal command surface,
# 2026-07-10) -- zero-field request arms, same Direction A shape id/stop/
# ping already have above (no arm-specific fields beyond cmd_kind/corr_id).
# ---------------------------------------------------------------------------


def test_direction_a_hello_request(harness):
    raw = env_hello_request(12)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "HELLO"
    assert fields["corr_id"] == "12"


def test_direction_a_ver_request(harness):
    raw = env_ver_request(13)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "VER"
    assert fields["corr_id"] == "13"


def test_direction_a_help_request(harness):
    raw = env_help_request(14)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "HELP"
    assert fields["corr_id"] == "14"


# ---------------------------------------------------------------------------
# ConfigDelta (096-006) -- COMMAND-only (never appears in ReplyEnvelope.body,
# see envelope.proto's own oneof list): its differential coverage is
# Direction A ONLY (host-encode -> firmware-decode). There is no Direction B
# counterpart to write for this message -- confirmed structurally (msg::
# ReplyEnvelope's body union has no ConfigDelta arm at all, envelope.h), not
# just by the schema's doc comments.
# ---------------------------------------------------------------------------


def test_direction_a_config_drivetrain(harness):
    raw = env_config_drivetrain(20, trackwidth=321.0, rotational_slip=0.75, ekf_q_xy=1.5, ekf_q_theta=2.5,
                                 ekf_r_otos_xy=3.5, ekf_r_otos_theta=4.5)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "CONFIG"
    assert fields["patch_kind"] == "DRIVETRAIN"
    for key, expected in [("trackwidth", 321.0), ("rotational_slip", 0.75), ("ekf_q_xy", 1.5),
                           ("ekf_q_theta", 2.5), ("ekf_r_otos_xy", 3.5), ("ekf_r_otos_theta", 4.5)]:
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
    for key in ("rotational_slip", "ekf_q_xy", "ekf_q_theta", "ekf_r_otos_xy", "ekf_r_otos_theta"):
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
    """ml/mr (Decision 5) -- travel_calib alone, no Gains fields present."""
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


def test_direction_a_config_planner_only_heading_kp(harness):
    """098-005: heading_kp alone (a live `SET headingKp=...`) -- min_speed/
    heading_kd absent, mirroring test_direction_a_config_motor_only_travel_
    calib's own partial-presence proof for the now-multi-field
    PlannerConfigPatch."""
    raw = env_config_planner(24, heading_kp=6.0)
    fields = _assert_ok(harness, raw)
    assert fields["patch_kind"] == "PLANNER"
    assert fields["heading_kp_has"] == "1"
    assert float_eq(fields["heading_kp"], 6.0)
    for key in ("min_speed", "heading_kd"):
        assert fields[f"{key}_has"] == "0"


@pytest.mark.parametrize("watchdog", [0, 1, 4242, 4294967295])
def test_direction_a_config_watchdog(harness, watchdog):
    raw = env_config_watchdog(25, watchdog)
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "CONFIG"
    assert fields["patch_kind"] == "WATCHDOG"
    assert fields["watchdog"] == str(watchdog)


def test_direction_a_config_empty_patch(harness):
    """A well-formed ConfigDelta with no oneof `patch` arm set at all decodes
    OK (patch_kind == NONE) -- BinaryChannel's own behavioral handling of
    this case (ERR_UNKNOWN field=6) is tested at the sim level
    (test_binary_channel.py's test_binary_config_empty_patch_replies_err_unknown);
    the wire codec itself must still decode it cleanly, never reject it."""
    raw = pb_envelope.CommandEnvelope(corr_id=26, config=pb_envelope.ConfigDelta()).SerializeToString()
    fields = _assert_ok(harness, raw)
    assert fields["cmd_kind"] == "CONFIG"
    assert fields["patch_kind"] == "NONE"


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


@pytest.mark.parametrize("text", ["", "HELP HELLO PING ID VER STOP", "X" * 63],
                         ids=["empty", "rump_list", "max63"])
def test_direction_b_helptext_reply(harness, text):
    """ReplyEnvelope.helptext (stakeholder-directed 6-verb minimal command
    surface, 2026-07-10): HELP's binary reply carries the live registered
    verb list verbatim -- Direction B (firmware-encode -> host-decode)
    proof for the NEW oneof arm, same shape as
    test_direction_b_device_id/test_direction_b_echo_reply above. "max63"
    exercises the field's full char[64] capacity (63 content bytes + the
    decoder's reserved null terminator, wire.cpp's own kString convention)."""
    raw = encode_helptext(harness, 5, text)
    assert raw is not None, "encode_helptext returned ZERO for a well-under-budget HelpText reply"
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == 5
    assert reply.WhichOneof("body") == "helptext"
    assert reply.helptext.text == text


# ---------------------------------------------------------------------------
# Telemetry (096-006) -- REPLY-only (never appears in CommandEnvelope.cmd,
# see envelope.proto's own oneof list): its differential coverage is
# Direction B ONLY (firmware-encode -> host-decode). There is no Direction A
# counterpart -- confirmed structurally (msg::CommandEnvelope's cmd union has
# no Telemetry arm at all, envelope.h), not just by the schema's doc
# comments.
# ---------------------------------------------------------------------------

_TELEMETRY_FULL_SHAPE = dict(
    now=123456, mode=2, seq=99, has_enc=True, enc_left=100.5, enc_right=-200.25, has_vel=True, vel_left=-50.0,
    vel_right=60.5, has_cmd_vel=True, cmd_vel_left=10.0, cmd_vel_right=20.0, has_pose=True, pose_x=1.5,
    pose_y=-2.5, pose_h=3.25, has_otos=True, otos_x=4.5, otos_y=5.5, otos_h=6.5, otos_connected=True,
    has_twist=True, twist_vx=-100.5, twist_vy=0.5, twist_omega=1.75, acc_left=10.25, acc_right=-20.25,
    active=True, conn_left=True, conn_right=False, glitch_left=3, glitch_right=4294967295, ts_left=5,
    ts_right=4294967295,
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
    assert tlm.has_cmd_vel == shape["has_cmd_vel"]
    assert tlm.cmd_vel_left == f32(shape["cmd_vel_left"])
    assert tlm.cmd_vel_right == f32(shape["cmd_vel_right"])
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
    assert tlm.acc_left == f32(shape["acc_left"])
    assert tlm.acc_right == f32(shape["acc_right"])
    assert tlm.active == shape["active"]
    assert tlm.conn_left == shape["conn_left"]
    assert tlm.conn_right == shape["conn_right"]
    assert tlm.glitch_left == shape["glitch_left"]
    assert tlm.glitch_right == shape["glitch_right"]
    assert tlm.ts_left == shape["ts_left"]
    assert tlm.ts_right == shape["ts_right"]


def test_direction_b_telemetry_full_shape(harness):
    """Every one of Telemetry's 28 fields, all `has_*` flags true -- the
    STREAM/SNAP-plus-bench-diagnostics union shape (telemetry.proto's own
    file header)."""
    raw = encode_telemetry(harness, 30, **_TELEMETRY_FULL_SHAPE)
    assert raw is not None, "encode_telemetry returned ZERO for a well-under-budget Telemetry reply"
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == 30
    assert reply.WhichOneof("body") == "tlm"
    _assert_telemetry_matches_shape(reply.tlm, _TELEMETRY_FULL_SHAPE)


def test_direction_b_telemetry_all_has_flags_false(harness):
    """The CURRENT tick()'s own conditionally-absent groups (has_cmd_vel/
    has_otos) plus every other has_* flag at once, proving `has_*=0` is
    encoded/decoded faithfully too, not just the all-present shape above --
    values behind a false has_* flag still round-trip (proto3 has no way to
    omit a nested message's own zero-valued scalar sub-fields once the
    message itself is present on the wire; `has_*` is this schema's OWN
    presence signal, not proto3 implicit presence, per telemetry.proto's
    file header)."""
    shape = dict(_TELEMETRY_FULL_SHAPE)
    for key in ("has_enc", "has_vel", "has_cmd_vel", "has_pose", "has_otos", "has_twist"):
        shape[key] = False
    raw = encode_telemetry(harness, 31, **shape)
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
    `encodeInto()`'s `kMessage` case (wire.cpp) emits them UNCONDITIONALLY,
    with no zero-value skip, unlike every SCALAR field (`kScalar`'s own
    `scalarIsDefault()` guard) or `kOpt`/`kOneofMessage` field (gated by
    their own has/oneof-kind check). So a from-scratch `pb_telemetry.
    Telemetry()` (which never touches `.pose`/`.otos`/`.twist` and so never
    marks them present) is NOT byte-identical to this round-trip -- the
    firmware always sends `pose {}`/`otos {}`/`twist {}` as PRESENT
    (possibly all-zero) submessages, regardless of `has_pose`/`has_otos`/
    `has_twist`, which are separate, semantic-only bool fields. This
    matches real `google.protobuf`: an embedded message field explicitly
    constructed with `Message()` (even with every sub-field left at its
    default) IS wire-present, distinct from never touching the field at
    all -- proto3's one exception to scalar implicit presence. Compared
    field-by-field via `_assert_telemetry_matches_shape()` (which does not
    care about presence, only value) rather than whole-message `==`
    against a from-scratch default, which WOULD fail on this presence
    difference alone."""
    raw = encode_telemetry(harness, 33)
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == 33
    assert reply.WhichOneof("body") == "tlm"
    zero_shape = {key: (False if key.startswith("has_") or key in ("active", "conn_left", "conn_right",
                                                                     "otos_connected") else 0)
                  for key in _TELEMETRY_FULL_SHAPE}
    _assert_telemetry_matches_shape(reply.tlm, zero_shape)


# ---------------------------------------------------------------------------
# ConfigSnapshot (096-006) -- REPLY-only (never appears in CommandEnvelope.
# cmd, see envelope.proto's own oneof list): Direction B ONLY, mirroring
# Telemetry's own posture above. Confirmed structurally the same way.
# ---------------------------------------------------------------------------


def test_direction_b_config_snapshot_drivetrain(harness):
    # 099-008: ekf_r_fix_xy/ekf_r_fix_theta -- passed explicitly (not the
    # driver's own defaults) so this differential round-trip check covers
    # them too, mirroring the six pre-existing fields' own coverage exactly.
    raw = encode_cfg_drivetrain(harness, 40, pb_config.CONFIG_DRIVETRAIN, 321.0, 0.75, 1.5, 2.5, 3.5, 4.5,
                                 6.5, 7.5)
    assert raw is not None
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.corr_id == 40
    assert reply.WhichOneof("body") == "cfg"
    assert reply.cfg.target == pb_config.CONFIG_DRIVETRAIN
    assert reply.cfg.WhichOneof("patch") == "drivetrain"
    p = reply.cfg.drivetrain
    assert (p.trackwidth, p.rotational_slip, p.ekf_q_xy, p.ekf_q_theta, p.ekf_r_otos_xy, p.ekf_r_otos_theta,
            p.ekf_r_fix_xy, p.ekf_r_fix_theta) == (
        f32(321.0), f32(0.75), f32(1.5), f32(2.5), f32(3.5), f32(4.5), f32(6.5), f32(7.5))


@pytest.mark.parametrize("target,side,side_name", [
    (pb_config.CONFIG_MOTOR_LEFT, 0, "LEFT"), (pb_config.CONFIG_MOTOR_RIGHT, 1, "RIGHT"),
])
def test_direction_b_config_snapshot_motor(harness, target, side, side_name):
    raw = encode_cfg_motor(harness, 41, target, side, 1.111, 9.5, 8.5, 7.5, 6.5, 5.5)
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.cfg.target == target
    assert reply.cfg.WhichOneof("patch") == "motor"
    p = reply.cfg.motor
    assert p.side == pb_config.DESCRIPTOR.enum_types_by_name["BoundMotorSide"].values_by_name[side_name].number
    assert (p.travel_calib, p.kp, p.ki, p.kff, p.i_max, p.kaw) == (
        f32(1.111), f32(9.5), f32(8.5), f32(7.5), f32(6.5), f32(5.5))


def test_direction_b_config_snapshot_planner(harness):
    # 100-001: PlannerConfigPatch's 17 new fields (Drive::Limits' wire-
    # tunable subset, planner.proto 15-31) -- tovez.json's own starting
    # values.
    new_fields = (620.0, 20.0, 150.0, 2.0, 6.0, 1.5e-5, 120.0, 2.0,
                  40.0, 0.15, 0.2, 0.3, 3.0, 40.0, 0.14, 15.0, 0.15)
    raw = encode_cfg_planner(harness, 42, pb_config.CONFIG_PLANNER, 42.0, 6.0, 0.25, *new_fields)
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.cfg.target == pb_config.CONFIG_PLANNER
    assert reply.cfg.WhichOneof("patch") == "planner"
    p = reply.cfg.planner
    assert (p.min_speed, p.heading_kp, p.heading_kd) == (f32(42.0), f32(6.0), f32(0.25))
    assert (p.v_wheel_max, p.steer_headroom, p.wheel_step_max, p.track_k_s, p.track_k_theta, p.track_k_cross,
            p.trim_v_max, p.trim_omega_max, p.replan_err_pos, p.replan_err_theta, p.replan_hold,
            p.replan_min_period, p.replan_max, p.handoff_tol_pos, p.handoff_tol_v, p.arrive_vel_tol,
            p.arrive_dwell) == tuple(f32(v) for v in new_fields)


@pytest.mark.parametrize("watchdog", [0, 1, 4242, 4294967295])
def test_direction_b_config_snapshot_watchdog(harness, watchdog):
    raw = encode_cfg_watchdog(harness, 43, pb_config.CONFIG_WATCHDOG, watchdog)
    reply = pb_envelope.ReplyEnvelope.FromString(raw)
    assert reply.cfg.target == pb_config.CONFIG_WATCHDOG
    assert reply.cfg.WhichOneof("patch") == "watchdog"
    assert reply.cfg.watchdog == watchdog


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


# ===========================================================================
# 096-006's own boundary corpus for the config-plane messages this ticket
# adds (DrivetrainConfigPatch/MotorConfigPatch/PlannerConfigPatch/watchdog).
#
# REALITY CHECK (documented, not silently patched -- config.proto's own file
# header, "Validation note" section, transcribed here): unlike
# MotionSegment's 11 fields above, NONE of these new Patch messages' fields
# carry a `(min)`/`(max)`/`(abs_max)` proto option -- confirmed directly
# against wire.cpp's generated `kFields_DrivetrainConfigPatch[]`/
# `kFields_MotorConfigPatch[]`/`kFields_PlannerConfigPatch[]`/
# `kFields_ConfigDelta[]` tables, every one of which has `flags = 0` (no
# kHasMin/kHasMax/kHasAbsMax bit set) for these fields. wire.cpp's own range
# check (`validateRange()`) short-circuits to `return true` the instant
# `flags & (kHasMin|kHasMax|kHasAbsMax) == 0` -- so THE WIRE CODEC ACCEPTS
# ANY float/uint32 value for tw/rotSlip/ekf*/minSpeed/sTimeout over the
# binary plane, including values `config_commands.cpp`'s OWN
# `validateCandidate()` rejects on the text SET path (`tw <= 0`, `rotSlip`
# outside `{0} ∪ [0.5, 1.0]`, `sTimeout <= 0`) -- `binary_channel.cpp`'s
# `CONFIG` arm never calls `validateCandidate()` at all (confirmed by
# reading binary_channel.cpp's CONFIG case directly: it posts straight to
# `bb.configIn`/`bb.streamWatchdogWindowIn` once `Opt<T>.has` is true, no
# invariant check anywhere on that path).
#
# This is a pre-existing, ALREADY-FLAGGED gap (config.proto's own comment:
# "Ticket 004 (BinaryChannel config arm) inherits this gap... if closing
# this specific gap turns out to matter, ticket 004 (or a follow-up) adds
# either options here or a small hand-written check in binary_channel.cpp,
# neither of which this ticket's own acceptance criteria require") --
# ticket 006's job is to TEST reality, not silently invent wire-level
# bounds or a validateCandidate() call that no prior ticket's acceptance
# criteria asked for. The cases below therefore all `expect_accept=True`,
# including the values validateCandidate() itself would reject on the text
# plane -- this is the ACTUAL, CURRENT, documented behavior, verified
# directly rather than assumed. See this ticket's completion notes for the
# same finding, flagged for a possible follow-up issue.
# ===========================================================================

# (harness verb, target field within the Patch, the "transcribed bound"
# value(s) validateCandidate() itself uses as its own invariant boundary --
# NOT a wire-enforced bound, see the REALITY CHECK above).
_CONFIG_INVARIANT_BOUNDARY_CASES = [
    # tw > 0 (validateCandidate) -- 0 and negative are text-SET-rejected,
    # but ALWAYS wire-accepted over the binary plane (no (min) on this field).
    ("drivetrain_tw_zero", dict(trackwidth=0.0)),
    ("drivetrain_tw_negative", dict(trackwidth=-1.0)),
    ("drivetrain_tw_large_negative", dict(trackwidth=-1.0e9)),
    ("drivetrain_tw_positive", dict(trackwidth=1.0)),
    # rotSlip == 0 || [0.5, 1.0] (validateCandidate) -- 0.3/0.49/1.01/-1 are
    # ALL text-SET-rejected (outside the non-contiguous domain), but ALWAYS
    # wire-accepted (no (min)/(max) on this field either).
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
    """minSpeed has no validateCandidate() invariant at all (only tw/
    rotSlip/sTimeout do) -- included for completeness against the SAME
    "no (min)/(max)/(abs_max) on this field" reality."""
    raw = env_config_planner(51, min_speed=min_speed)
    fields = _assert_ok(harness, raw)
    assert fields["patch_kind"] == "PLANNER"
    assert float_eq(fields["min_speed"], min_speed)


@pytest.mark.parametrize("watchdog", [0, 1])
def test_boundary_config_watchdog_no_wire_level_enforcement(harness, watchdog):
    """sTimeout > 0 (validateCandidate) -- 0 is text-SET-rejected
    (parseLongStrict's signed long catches negative input server-side
    before this invariant even runs; watchdog itself is a wire `uint32`, so
    a negative value cannot be represented on the wire at all), but ALWAYS
    wire-accepted over the binary plane (no (min) on ConfigDelta.watchdog
    either)."""
    raw = env_config_watchdog(52, watchdog)
    fields = _assert_ok(harness, raw)
    assert fields["patch_kind"] == "WATCHDOG"
    assert fields["watchdog"] == str(watchdog)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
