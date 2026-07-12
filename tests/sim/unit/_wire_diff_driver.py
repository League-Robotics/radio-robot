"""Shared driver for ticket 095-006's differential/fuzz/range suite against
``google.protobuf``. NOT a test module itself (no ``test_`` prefix -- pytest
does not collect it); imported by ``test_wire_differential.py`` and
``test_wire_fuzz.py``.

Compiles ``wire_differential_harness.cpp`` (source/messages/wire.cpp +
wire_runtime.cpp linked in) with the system C++ compiler and drives it
one-shot-per-case via ``subprocess`` -- see that harness's own file-header
comment for the exact argv protocol (``decode <base64>`` /
``encode_ok|encode_err|encode_id <args...>``).

Also provides small pb2-building and wire-byte helpers (varint/tag encoding
for the fuzz suite's unknown-field-salting, float32 canonicalization for
exact-equality comparisons against the harness's ``%.9g``-printed decode
output) shared by both test files.
"""
from __future__ import annotations

import base64
import pathlib
import shutil
import struct
import subprocess
import sys
from dataclasses import dataclass, field

import pytest

# tests/sim/unit/_wire_diff_driver.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "wire_differential_harness.cpp"
_WIRE_SRC = _SOURCE_DIR / "messages" / "wire.cpp"
_WIRE_RUNTIME_SRC = _SOURCE_DIR / "messages" / "wire_runtime.cpp"

# wire.h/wire_runtime.h both document the project's ACTUAL compiled standard
# as -std=gnu++20 (095-003's finding) -- build the host harness to the same
# standard, matching wire_codec_harness.cpp's/wire_runtime_harness.cpp's own
# test drivers.
CXX_STANDARD = "c++20"

_HOST_PB2_DIR = _REPO_ROOT / "host"
if str(_HOST_PB2_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_PB2_DIR))

from robot_radio.robot.pb2 import common_pb2 as pb_common  # noqa: E402
from robot_radio.robot.pb2 import config_pb2 as pb_config  # noqa: E402
from robot_radio.robot.pb2 import drivetrain_pb2 as pb_drivetrain  # noqa: E402
from robot_radio.robot.pb2 import envelope_pb2 as pb_envelope  # noqa: E402
from robot_radio.robot.pb2 import motion_pb2 as pb_motion  # noqa: E402
from robot_radio.robot.pb2 import planner_pb2 as pb_planner  # noqa: E402
from robot_radio.robot.pb2 import telemetry_pb2 as pb_telemetry  # noqa: E402


def find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++."""
    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def compile_harness(tmp_path: pathlib.Path, binary_name: str, extra_flags: list[str]) -> pathlib.Path:
    """Compile wire_differential_harness.cpp (+ wire.cpp + wire_runtime.cpp)
    with the given extra compiler flags; returns the built binary's path."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _WIRE_SRC.is_file(), f"wire.cpp missing (run scripts/gen_messages.py?): {_WIRE_SRC}"
    assert _WIRE_RUNTIME_SRC.is_file(), f"wire_runtime.cpp missing: {_WIRE_RUNTIME_SRC}"

    cxx = find_cxx_compiler()
    binary = tmp_path / binary_name

    result = subprocess.run(
        [
            cxx,
            f"-std={CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            *extra_flags,
            "-I",
            str(_SOURCE_DIR),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
            str(_WIRE_SRC),
            str(_WIRE_RUNTIME_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, (
        f"wire_differential_harness.cpp failed to compile:\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )
    return binary


# ---------------------------------------------------------------------------
# Harness invocation + output parsing
# ---------------------------------------------------------------------------


@dataclass
class HarnessRun:
    """Raw result of a single harness subprocess invocation."""

    returncode: int
    stdout: str
    stderr: str
    crashed: bool = field(init=False)

    def __post_init__(self) -> None:
        # A clean decode/encode outcome (including an ERR result -- malformed
        # input is an ordinary, successful decode() OUTCOME, not a process
        # failure) always exits 0 with a single recognized output line. A
        # nonzero exit or ASan/UBSan sanitizer text on stderr is the actual
        # "the harness itself misbehaved" signal the fuzz suite is watching
        # for.
        sanitizer_markers = ("AddressSanitizer", "UndefinedBehaviorSanitizer", "runtime error:")
        self.crashed = self.returncode != 0 or any(m in self.stderr for m in sanitizer_markers)


def run_harness(binary: pathlib.Path, *args: str) -> HarnessRun:
    proc = subprocess.run([str(binary), *args], capture_output=True, text=True, timeout=10)
    return HarnessRun(proc.returncode, proc.stdout.strip(), proc.stderr)


def decode(binary: pathlib.Path, raw_bytes: bytes) -> HarnessRun:
    """Invoke `decode <base64>` on the harness with the given raw
    CommandEnvelope-shaped bytes (may be well-formed or deliberately
    malformed -- the fuzz suite feeds arbitrary bytes here)."""
    b64 = base64.b64encode(raw_bytes).decode("ascii")
    return run_harness(binary, "decode", b64)


def parse_decode_line(line: str) -> tuple[str, dict[str, str]]:
    """Parse a harness decode() output line ("OK ..." / "ERR ...") into
    (status, {key: value}) -- every token after the first is `key=value`."""
    tokens = line.split()
    assert tokens, f"empty harness decode output line: {line!r}"
    status = tokens[0]
    assert status in ("OK", "ERR"), f"unrecognized harness decode status: {line!r}"
    fields: dict[str, str] = {}
    for tok in tokens[1:]:
        key, _, value = tok.partition("=")
        fields[key] = value
    return status, fields


def encode_ok(binary: pathlib.Path, corr_id: int, q: int, rem: float, t: int = 0) -> bytes | None:
    # t (095-007, Ack schema-gap closure -- see envelope.proto's own Ack.t
    # doc comment): optional 5th argv, defaulting to 0 so every pre-existing
    # call site (which never passed a 4th value to the harness) keeps
    # constructing the identical Ack{q,rem,t=0} it always has.
    r = run_harness(binary, "encode_ok", str(corr_id), str(q), repr(rem), str(t))
    assert not r.crashed, f"encode_ok crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_ok output: {line!r}"
    return base64.b64decode(line[len("B64 "):])


def encode_err(binary: pathlib.Path, corr_id: int, code_name: str, field_num: int) -> bytes | None:
    r = run_harness(binary, "encode_err", str(corr_id), code_name, str(field_num))
    assert not r.crashed, f"encode_err crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_err output: {line!r}"
    return base64.b64decode(line[len("B64 "):])


def encode_id(binary: pathlib.Path, corr_id: int, model: str, name: str, serial: int, fw_version: str,
              proto_version: int) -> bytes | None:
    r = run_harness(binary, "encode_id", str(corr_id), model, name, str(serial), fw_version, str(proto_version))
    assert not r.crashed, f"encode_id crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_id output: {line!r}"
    return base64.b64decode(line[len("B64 "):])


def encode_telemetry(binary: pathlib.Path, corr_id: int, **fields) -> bytes | None:
    """096-006: builds ReplyEnvelope{tlm=Telemetry{...}} via the
    `encode_telemetry` argv verb (see wire_differential_harness.cpp's file
    header for the full positional list). `fields` keys are telemetry.proto's
    OWN field names; every field not passed defaults to its proto zero
    value (0 / 0.0 / False) -- mirrors every pb2 message constructor's own
    "omitted kwarg -> zero default" convention, so a caller only spells out
    the fields a given test case cares about."""
    order = (
        "now", "mode", "seq", "has_enc", "enc_left", "enc_right", "has_vel", "vel_left", "vel_right",
        "has_cmd_vel", "cmd_vel_left", "cmd_vel_right", "has_pose", "pose_x", "pose_y", "pose_h",
        "has_otos", "otos_x", "otos_y", "otos_h", "otos_connected", "has_twist", "twist_vx", "twist_vy",
        "twist_omega", "acc_left", "acc_right", "active", "conn_left", "conn_right", "glitch_left",
        "glitch_right", "ts_left", "ts_right",
    )
    unknown = set(fields) - set(order)
    assert not unknown, f"unknown Telemetry field(s): {unknown}"
    args = [str(corr_id)]
    for key in order:
        value = fields.get(key, 0)
        # bool -> "0"/"1", NOT Python's own str(True) == "True" -- the
        # harness parses every non-float positional arg with strtoul(), which
        # silently reads "True"/"False" as 0 (no leading digit), corrupting
        # every has_*/active/conn_*/otos_connected flag. Caught by this
        # ticket's own test_direction_b_telemetry_full_shape failing before
        # this fix -- see completion notes.
        args.append(str(int(value)) if isinstance(value, bool) else str(value))
    r = run_harness(binary, "encode_telemetry", *args)
    assert not r.crashed, f"encode_telemetry crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_telemetry output: {line!r}"
    return base64.b64decode(line[len("B64 "):])


def encode_cfg_drivetrain(binary: pathlib.Path, corr_id: int, target: int, trackwidth: float,
                           rotational_slip: float, ekf_q_xy: float, ekf_q_theta: float, ekf_r_otos_xy: float,
                           ekf_r_otos_theta: float, ekf_r_fix_xy: float = 25.0,
                           ekf_r_fix_theta: float = 0.005) -> bytes | None:
    """096-006/099-008: builds ReplyEnvelope{cfg=ConfigSnapshot{target,
    drivetrain=DrivetrainConfigPatch{...}}}. ekf_r_fix_xy/ekf_r_fix_theta
    default to ordinary in-range values so existing callers that predate
    099-008 (and only care about the other six fields) don't need updating."""
    r = run_harness(binary, "encode_cfg_drivetrain", str(corr_id), str(target), repr(trackwidth),
                     repr(rotational_slip), repr(ekf_q_xy), repr(ekf_q_theta), repr(ekf_r_otos_xy),
                     repr(ekf_r_otos_theta), repr(ekf_r_fix_xy), repr(ekf_r_fix_theta))
    assert not r.crashed, f"encode_cfg_drivetrain crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_cfg_drivetrain output: {line!r}"
    return base64.b64decode(line[len("B64 "):])


def encode_cfg_motor(binary: pathlib.Path, corr_id: int, target: int, side: int, travel_calib: float, kp: float,
                      ki: float, kff: float, i_max: float, kaw: float) -> bytes | None:
    """096-006: builds ReplyEnvelope{cfg=ConfigSnapshot{target,
    motor=MotorConfigPatch{...}}}."""
    r = run_harness(binary, "encode_cfg_motor", str(corr_id), str(target), str(side), repr(travel_calib),
                     repr(kp), repr(ki), repr(kff), repr(i_max), repr(kaw))
    assert not r.crashed, f"encode_cfg_motor crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_cfg_motor output: {line!r}"
    return base64.b64decode(line[len("B64 "):])


def encode_cfg_planner(binary: pathlib.Path, corr_id: int, target: int, min_speed: float, heading_kp: float,
                        heading_kd: float) -> bytes | None:
    """096-006 (+ 098-005: heading_kp/heading_kd): builds ReplyEnvelope{
    cfg=ConfigSnapshot{target, planner=PlannerConfigPatch{min_speed,
    heading_kp, heading_kd}}}."""
    r = run_harness(binary, "encode_cfg_planner", str(corr_id), str(target), repr(min_speed), repr(heading_kp),
                     repr(heading_kd))
    assert not r.crashed, f"encode_cfg_planner crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_cfg_planner output: {line!r}"
    return base64.b64decode(line[len("B64 "):])


def encode_cfg_watchdog(binary: pathlib.Path, corr_id: int, target: int, watchdog: int) -> bytes | None:
    """096-006: builds ReplyEnvelope{cfg=ConfigSnapshot{target,
    watchdog=<uint32>}}."""
    r = run_harness(binary, "encode_cfg_watchdog", str(corr_id), str(target), str(watchdog))
    assert not r.crashed, f"encode_cfg_watchdog crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_cfg_watchdog output: {line!r}"
    return base64.b64decode(line[len("B64 "):])


def encode_helptext(binary: pathlib.Path, corr_id: int, text: str) -> bytes | None:
    """Stakeholder-directed 6-verb minimal command surface (2026-07-10):
    builds ReplyEnvelope{helptext=HelpText{text}} via the `encode_helptext`
    argv verb."""
    r = run_harness(binary, "encode_helptext", str(corr_id), text)
    assert not r.crashed, f"encode_helptext crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_helptext output: {line!r}"
    return base64.b64decode(line[len("B64 "):])


def encode_echo_reply(binary: pathlib.Path, corr_id: int, payload: bytes) -> bytes | None:
    """Builds ReplyEnvelope{echo=Echo{payload}} (095-007, ReplyEnvelope
    schema-gap closure -- see envelope.proto's own ReplyEnvelope.echo doc
    comment). payload is hex-encoded for argv passing (mirrors decode()'s
    own payload_hex= output field) so arbitrary bytes (embedded NUL, the
    full 0-255 range) survive process argv boundaries intact."""
    r = run_harness(binary, "encode_echo_reply", str(corr_id), payload.hex())
    assert not r.crashed, f"encode_echo_reply crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_echo_reply output: {line!r}"
    return base64.b64decode(line[len("B64 "):])


# ---------------------------------------------------------------------------
# float32 canonicalization -- both sides of the differential (the harness's
# `%.9g`-printed decode output and pb2's own float getters) must be compared
# after being canonicalized through the SAME binary32 round-trip, since a
# protobuf `float` field silently truncates any Python double to binary32 on
# SerializeToString(), and 9 significant decimal digits is the proven
# sufficient precision to recover that exact binary32 value from text.
# ---------------------------------------------------------------------------


def f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def float_eq(printed: str, expected: float) -> bool:
    """Compare a harness-printed `%.9g` float string against an expected
    Python value, canonicalizing BOTH sides through the SAME binary32
    round-trip before comparing.

    9 significant decimal digits is the proven sufficient precision to
    round-trip a binary32 value through decimal text and back INTO
    binary32 -- it is NOT sufficient to round-trip into a full double and
    expect double-precision equality against another value's own
    double-promoted binary32 representation (parsing "31.4160004" as a
    double yields 31.4160004000000..., a different double than
    31.416000366210938, the double-promotion of the binary32 value both
    numbers actually came from). Canonicalizing the PARSED value back
    through f32() recovers the shared binary32 bit pattern on both sides,
    which then compare bit-identical.
    """
    return f32(float(printed)) == f32(expected)


# ---------------------------------------------------------------------------
# Raw low-level protobuf byte builders -- used by the fuzz suite to splice an
# unknown field into an otherwise-valid, pb2-serialized envelope at a
# guaranteed FIELD BOUNDARY (prepended/appended to the whole message, never
# spliced into the middle of an existing field's own bytes, which would just
# produce a different malformed-input case, not a "the unknown field must be
# correctly skipped and every OTHER field must remain intact" case).
# ---------------------------------------------------------------------------


def _varint(n: int) -> bytes:
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            out.append(b | 0x80)
        else:
            out.append(b)
            break
    return bytes(out)


def _tag(field_num: int, wire_type: int) -> bytes:
    return _varint((field_num << 3) | wire_type)


def unknown_varint_field(field_num: int, value: int) -> bytes:
    """A complete, well-formed (tag, varint-value) field -- safe to splice at
    any FIELD boundary of a protobuf message (prepend/append to the whole
    serialized message, or between two other complete fields)."""
    return _tag(field_num, 0) + _varint(value)


# ---------------------------------------------------------------------------
# pb2 CommandEnvelope builders for this sprint's implemented arms -- shared
# by both the differential and fuzz/boundary suites so every corpus is built
# from the SAME reference construction helpers.
# ---------------------------------------------------------------------------


def env_drive_twist(corr_id: int, v_x: float, v_y: float, omega: float, seed: bool | None = None,
                     standby: bool | None = None) -> bytes:
    kwargs = {}
    if seed is not None:
        kwargs["seed"] = seed
    if standby is not None:
        kwargs["standby"] = standby
    drive = pb_drivetrain.DrivetrainCommand(twist=pb_common.BodyTwist3(v_x=v_x, v_y=v_y, omega=omega), **kwargs)
    return pb_envelope.CommandEnvelope(corr_id=corr_id, drive=drive).SerializeToString()


def env_drive_wheels(corr_id: int, wheels: list[tuple[float | None, float | None]]) -> bytes:
    targets = []
    for speed, position in wheels:
        kwargs = {}
        if speed is not None:
            kwargs["speed"] = speed
        if position is not None:
            kwargs["position"] = position
        targets.append(pb_common.WheelTarget(**kwargs))
    drive = pb_drivetrain.DrivetrainCommand(wheels=pb_drivetrain.WheelTargets(w=targets))
    return pb_envelope.CommandEnvelope(corr_id=corr_id, drive=drive).SerializeToString()


def env_drive_neutral(corr_id: int, neutral: int) -> bytes:
    drive = pb_drivetrain.DrivetrainCommand(neutral=neutral)
    return pb_envelope.CommandEnvelope(corr_id=corr_id, drive=drive).SerializeToString()


_MOTION_SEGMENT_FIELDS = (
    "distance", "direction", "final_heading", "speed_max", "accel_max", "jerk_max", "yaw_rate_max",
    "yaw_accel_max", "yaw_jerk_max", "time", "v", "omega", "stream",
)


def build_motion_segment(**kwargs) -> "pb_motion.MotionSegment":
    for k in kwargs:
        assert k in _MOTION_SEGMENT_FIELDS, f"unknown MotionSegment field: {k}"
    return pb_motion.MotionSegment(**kwargs)


def env_segment(corr_id: int, seg: "pb_motion.MotionSegment") -> bytes:
    return pb_envelope.CommandEnvelope(corr_id=corr_id, segment=seg).SerializeToString()


def env_replace(corr_id: int, seg: "pb_motion.MotionSegment") -> bytes:
    return pb_envelope.CommandEnvelope(corr_id=corr_id, replace=seg).SerializeToString()


def env_stop(corr_id: int) -> bytes:
    return pb_envelope.CommandEnvelope(corr_id=corr_id, stop=pb_envelope.Stop()).SerializeToString()


def env_ping(corr_id: int) -> bytes:
    return pb_envelope.CommandEnvelope(corr_id=corr_id, ping=pb_envelope.Ping()).SerializeToString()


def env_echo(corr_id: int, payload: bytes) -> bytes:
    return pb_envelope.CommandEnvelope(corr_id=corr_id, echo=pb_envelope.Echo(payload=payload)).SerializeToString()


def env_id_request(corr_id: int) -> bytes:
    return pb_envelope.CommandEnvelope(corr_id=corr_id, id=pb_envelope.DeviceId()).SerializeToString()


# hello/ver/help (stakeholder-directed 6-verb minimal command surface,
# 2026-07-10) -- zero-field request arms, same shape env_id_request()/
# env_stop()/env_ping() already have.


def env_hello_request(corr_id: int) -> bytes:
    return pb_envelope.CommandEnvelope(corr_id=corr_id, hello=pb_envelope.Hello()).SerializeToString()


def env_ver_request(corr_id: int) -> bytes:
    return pb_envelope.CommandEnvelope(corr_id=corr_id, ver=pb_envelope.Ver()).SerializeToString()


def env_help_request(corr_id: int) -> bytes:
    return pb_envelope.CommandEnvelope(corr_id=corr_id, help=pb_envelope.Help()).SerializeToString()


# ---------------------------------------------------------------------------
# ConfigDelta builders (096-006) -- ConfigDelta is COMMAND-only (never
# appears in ReplyEnvelope.body, see envelope.proto's own oneof list), so
# unlike drive/segment/etc. above it needs only a host-encode ->
# firmware-decode direction (Direction A); there is no env_config-side
# "Direction B" counterpart to write.
# ---------------------------------------------------------------------------


def env_config_drivetrain(corr_id: int, **fields) -> bytes:
    """`fields` keys are DrivetrainConfigPatch's own proto field names
    (trackwidth/rotational_slip/ekf_q_xy/ekf_q_theta/ekf_r_otos_xy/
    ekf_r_otos_theta) -- only the ones passed are marked `has=true` on the
    wire (proto3 `optional` explicit presence), mirroring a real client
    that only sets the keys it wants to change."""
    patch = pb_config.DrivetrainConfigPatch(**fields)
    return pb_envelope.CommandEnvelope(
        corr_id=corr_id, config=pb_envelope.ConfigDelta(drivetrain=patch)).SerializeToString()


def env_config_motor(corr_id: int, side: int = pb_config.LEFT, **fields) -> bytes:
    """`fields` keys are MotorConfigPatch's own proto field names
    (travel_calib/kp/ki/kff/i_max/kaw); `side` is always present (not
    `optional` on the wire -- config.proto's own MotorConfigPatch.side is a
    plain enum field, proto3 implicit presence)."""
    patch = pb_config.MotorConfigPatch(side=side, **fields)
    return pb_envelope.CommandEnvelope(
        corr_id=corr_id, config=pb_envelope.ConfigDelta(motor=patch)).SerializeToString()


def env_config_planner(corr_id: int, **fields) -> bytes:
    """`fields` keys are PlannerConfigPatch's own proto field names
    (min_speed, plus heading_kp/heading_kd -- 098-005) -- only the ones
    passed are marked `has=true` on the wire, same partial-presence contract
    as env_config_drivetrain()/env_config_motor() above."""
    patch = pb_config.PlannerConfigPatch(**fields)
    return pb_envelope.CommandEnvelope(
        corr_id=corr_id, config=pb_envelope.ConfigDelta(planner=patch)).SerializeToString()


def env_config_watchdog(corr_id: int, watchdog: int) -> bytes:
    return pb_envelope.CommandEnvelope(
        corr_id=corr_id, config=pb_envelope.ConfigDelta(watchdog=watchdog)).SerializeToString()


__all__ = [
    "pb_common", "pb_config", "pb_drivetrain", "pb_envelope", "pb_motion", "pb_planner", "pb_telemetry",
    "compile_harness", "run_harness", "decode", "parse_decode_line",
    "encode_ok", "encode_err", "encode_id", "encode_echo_reply", "encode_helptext", "f32", "float_eq",
    "encode_telemetry", "encode_cfg_drivetrain", "encode_cfg_motor", "encode_cfg_planner",
    "encode_cfg_watchdog",
    "unknown_varint_field",
    "env_drive_twist", "env_drive_wheels", "env_drive_neutral",
    "build_motion_segment", "env_segment", "env_replace",
    "env_stop", "env_ping", "env_echo", "env_id_request",
    "env_hello_request", "env_ver_request", "env_help_request",
    "env_config_drivetrain", "env_config_motor", "env_config_planner", "env_config_watchdog",
]
