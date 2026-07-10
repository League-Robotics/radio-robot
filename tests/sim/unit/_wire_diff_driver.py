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
from robot_radio.robot.pb2 import drivetrain_pb2 as pb_drivetrain  # noqa: E402
from robot_radio.robot.pb2 import envelope_pb2 as pb_envelope  # noqa: E402
from robot_radio.robot.pb2 import motion_pb2 as pb_motion  # noqa: E402


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


def encode_ok(binary: pathlib.Path, corr_id: int, q: int, rem: float) -> bytes | None:
    r = run_harness(binary, "encode_ok", str(corr_id), str(q), repr(rem))
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


__all__ = [
    "pb_common", "pb_drivetrain", "pb_envelope", "pb_motion",
    "compile_harness", "run_harness", "decode", "parse_decode_line",
    "encode_ok", "encode_err", "encode_id", "f32", "float_eq",
    "unknown_varint_field",
    "env_drive_twist", "env_drive_wheels", "env_drive_neutral",
    "build_motion_segment", "env_segment", "env_replace",
    "env_stop", "env_ping", "env_echo", "env_id_request",
]
