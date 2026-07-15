"""Shared driver for the differential/fuzz/range suite against
``google.protobuf``. NOT a test module itself (no ``test_`` prefix -- pytest
does not collect it); imported by ``test_wire_differential.py`` and
``test_wire_fuzz.py``.

Rewritten 103-001 (SUC-001, architecture-update.md (103) Decisions 2/3)
against the P4-pruned schema -- ``CommandEnvelope.cmd`` is exactly
``{twist, config, stop}``; ``ReplyEnvelope.body`` is exactly
``{ok, err, tlm}``; ``Telemetry`` carries the depth-3 ack ring +
``fault_bits``/``event_bits``; ``TelemetrySecondary`` is a new standalone
top-level message with its own ``msg::wire::encode()`` overload.

Compiles ``wire_differential_harness.cpp`` (source/messages/wire.cpp +
wire_runtime.cpp linked in) with the system C++ compiler and drives it
one-shot-per-case via ``subprocess`` -- see that harness's own file-header
comment for the exact argv protocol.

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
from robot_radio.robot.pb2 import envelope_pb2 as pb_envelope  # noqa: E402
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


# Default ack ring (used by callers that don't care about ack-ring content
# for a given test case) -- 3 entries, all ACK_STATUS_OK/err_code=0.
_DEFAULT_ACKS = ((0, 0, 0), (0, 0, 0), (0, 0, 0))


def encode_telemetry(binary: pathlib.Path, corr_id: int, acks: tuple = _DEFAULT_ACKS, **fields) -> bytes | None:
    """Builds ReplyEnvelope{tlm=Telemetry{...}} via the `encode_telemetry`
    argv verb (see wire_differential_harness.cpp's file header for the full
    positional list). `acks` is a 3-tuple of (corr_id, status, err_code)
    triples -- the ring is ALWAYS encoded at its full depth-3 (the wire
    codec's encode() trusts the caller-supplied count with no re-clamp, so
    this driver never exercises a malformed count > 3, matching
    app/Telemetry's own designed invariant, ticket 005). `fields` keys are
    telemetry.proto's OWN field names (excluding `acks`); every field not
    passed defaults to its proto zero value (0 / 0.0 / False)."""
    order = (
        "now", "mode", "seq", "has_enc", "enc_left", "enc_right", "has_vel", "vel_left", "vel_right",
        "has_pose", "pose_x", "pose_y", "pose_h", "has_otos", "otos_x", "otos_y", "otos_h", "otos_connected",
        "has_twist", "twist_vx", "twist_vy", "twist_omega", "active", "conn_left", "conn_right",
        "fault_bits", "event_bits",
    )
    unknown = set(fields) - set(order)
    assert not unknown, f"unknown Telemetry field(s): {unknown}"
    assert len(acks) == 3, f"acks must be a 3-tuple (ring depth 3), got {len(acks)}"
    args = [str(corr_id)]
    for ack_corr_id, status, err_code in acks:
        args.extend([str(ack_corr_id), str(status), str(err_code)])
    for key in order:
        value = fields.get(key, 0)
        # bool -> "0"/"1", NOT Python's own str(True) == "True" -- the
        # harness parses every non-float positional arg with strtoul(), which
        # silently reads "True"/"False" as 0 (no leading digit), corrupting
        # every has_*/active/conn_*/otos_connected flag.
        args.append(str(int(value)) if isinstance(value, bool) else str(value))
    r = run_harness(binary, "encode_telemetry", *args)
    assert not r.crashed, f"encode_telemetry crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_telemetry output: {line!r}"
    return base64.b64decode(line[len("B64 "):])


def encode_telemetry_secondary(binary: pathlib.Path, **fields) -> bytes | None:
    """Builds a STANDALONE TelemetrySecondary (Decision 3 -- own
    independently-armored line, no ReplyEnvelope wrapper, no corr_id) via
    the `encode_telemetry_secondary` argv verb. `fields` keys are
    TelemetrySecondary's own proto field names."""
    order = (
        "now", "has_cmd_vel", "cmd_vel_left", "cmd_vel_right", "acc_left", "acc_right",
        "glitch_left", "glitch_right", "ts_left", "ts_right",
    )
    unknown = set(fields) - set(order)
    assert not unknown, f"unknown TelemetrySecondary field(s): {unknown}"
    args = []
    for key in order:
        value = fields.get(key, 0)
        args.append(str(int(value)) if isinstance(value, bool) else str(value))
    r = run_harness(binary, "encode_telemetry_secondary", *args)
    assert not r.crashed, f"encode_telemetry_secondary crashed: {r.stdout}\n{r.stderr}"
    line = r.stdout.strip()
    if line == "ZERO":
        return None
    assert line.startswith("B64 "), f"unexpected encode_telemetry_secondary output: {line!r}"
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
    double-promoted binary32 representation. Canonicalizing the PARSED
    value back through f32() recovers the shared binary32 bit pattern on
    both sides, which then compare bit-identical.
    """
    return f32(float(printed)) == f32(expected)


# ---------------------------------------------------------------------------
# Raw low-level protobuf byte builders -- used by the fuzz suite to splice an
# unknown field into an otherwise-valid, pb2-serialized envelope at a
# guaranteed FIELD BOUNDARY.
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
# pb2 CommandEnvelope builders for the pruned P4 arm set (twist/config/stop)
# -- shared by both the differential and fuzz/boundary suites so every
# corpus is built from the SAME reference construction helpers.
# ---------------------------------------------------------------------------


def env_twist(corr_id: int, v_x: float, omega: float, duration: float) -> bytes:
    twist = pb_envelope.Twist(v_x=v_x, omega=omega, duration=duration)
    return pb_envelope.CommandEnvelope(corr_id=corr_id, twist=twist).SerializeToString()


def env_stop(corr_id: int) -> bytes:
    return pb_envelope.CommandEnvelope(corr_id=corr_id, stop=pb_envelope.Stop()).SerializeToString()


# ---------------------------------------------------------------------------
# ConfigDelta builders -- unchanged shape from the pre-103 schema
# (config.proto/ConfigDelta itself is untouched by this ticket's prune).
# ---------------------------------------------------------------------------


def env_config_drivetrain(corr_id: int, **fields) -> bytes:
    """`fields` keys are DrivetrainConfigPatch's own proto field names --
    only the ones passed are marked `has=true` on the wire (proto3
    `optional` explicit presence), mirroring a real client that only sets
    the keys it wants to change."""
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
    """`fields` keys are PlannerConfigPatch's own proto field names."""
    patch = pb_config.PlannerConfigPatch(**fields)
    return pb_envelope.CommandEnvelope(
        corr_id=corr_id, config=pb_envelope.ConfigDelta(planner=patch)).SerializeToString()


def env_config_watchdog(corr_id: int, watchdog: int) -> bytes:
    return pb_envelope.CommandEnvelope(
        corr_id=corr_id, config=pb_envelope.ConfigDelta(watchdog=watchdog)).SerializeToString()


__all__ = [
    "pb_common", "pb_config", "pb_envelope", "pb_planner", "pb_telemetry",
    "compile_harness", "run_harness", "decode", "parse_decode_line",
    "encode_ok", "encode_err", "encode_telemetry", "encode_telemetry_secondary", "f32", "float_eq",
    "unknown_varint_field",
    "env_twist", "env_stop",
    "env_config_drivetrain", "env_config_motor", "env_config_planner", "env_config_watchdog",
]
