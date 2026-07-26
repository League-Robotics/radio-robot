"""src/tests/sim/system/test_sim_wire_loopback.py -- ticket 124-006
(SUC-003, REQUIRED, non-negotiable): ``Tests::SimWireLoopback``.

Both of sprint 123's wire bugs (the 0x0A-in-binary-frame corruption, the
move-enqueue-ack gap) were bench-only discoveries. They reached hardware
before anything caught them because ``sim_ctypes``'s scalar-argument
exports (``sim_inject_twist()``/``sim_inject_stop()``) hand the sim a bare
float/duration/corr_id tuple -- the Python side never builds or parses a
single wire byte for those calls. This file closes that gap: it drives a
command through the REAL host-side byte-level codec
(``robot_radio.io.wire_codec.encode_frame()`` -- the SAME function
``SimLoop.move()``/``serial_conn.py`` use for every wire-bound command,
never a stub or a hand-rolled substitute), pushes the resulting bytes
into the REAL compiled firmware (``TestSim::SimHarness`` -> the real
``App::Comms::pump()``/``decodeBinaryFrame()`` -> the real generated
``msg::wire::decode()`` -> real dispatch), and reads the reply back out
through the REAL firmware encoder (``App::Telemetry::emit()`` -> real
``msg::wire::encode()`` + real ``WireRuntime`` COBS/CRC) and the REAL host
decoder (``robot_radio.io.wire_codec.decode_frame()`` + real
``pb2.ReplyEnvelope.FromString()``) -- ``SimLoop._decode_reply_frame()``,
the same function every production ``SimLoop`` consumer (TestGUI Sim mode,
``planner/tour.py`` runs) already relies on.

This is the exact encode -> COBS -> transmit -> demux -> decode chain both
123 bugs lived in, now exercised entirely off-hardware (no ``pyocd``
probe, no ``/dev/cu.usbmodem*`` needed -- just the compiled
``src/sim/build/libfirmware_host.{dylib,so}``).

ADDITIVE, not a replacement
----------------------------
``sim_ctypes``'s existing envelope-passing exports (``sim_inject_twist()``/
``sim_inject_stop()``, and every test that calls
``SimLoop.twist()``/``.stop()`` -- e.g.
``test_sim_configure_from_robot.py``) are UNTOUCHED by this ticket and stay
exactly as they are: fast, scalar-argument, no wire bytes involved,
appropriate for the vast majority of sim-driven scenario tests that don't
care about framing. This file adds the byte-level path those tests don't
(and structurally can't) cover -- it does not modify, delete, or
supersede any of them.

Not a stub: verify by reading ``_build_move_wheels_frame()``/
``_build_stop_frame()`` below -- both call
``robot_radio.io.wire_codec.encode_frame()`` directly (COBS delimiter
0x0A + CRC-16/CCITT-FALSE, the pinned wire composition,
``wire_codec.py``'s own header) on a REAL ``pb2.CommandEnvelope.
SerializeToString()`` payload; the resulting bytes are pushed onto the
inbound ``TestSupport::FakeTransport`` via ``SimLoop.inject_command()`` ->
``sim_inject_command()`` -> ``SimHarness::injectCommand()`` -- exactly
where a real serial/radio byte stream lands. Nothing here re-implements
COBS, CRC, or protobuf framing.

The 123-006 repro shape
-------------------------
123-006 fixed a hardware-only corruption where a ``move_wheels`` envelope
whose own serialized bytes happened to embed a literal ``0x0A`` byte got
split mid-frame by the OLD text/binary line-splitting heuristic.
``_velocity_with_embedded_delimiter_byte()`` below computes a genuine,
in-range wheel-velocity value whose IEEE-754 float32 encoding contains a
literal ``0x0A`` byte -- not a synthetic/out-of-range magic constant --
reproducing that exact hazard shape. ``test_move_wheels_with_embedded_
0x0a_byte_round_trips_through_real_codec`` proves it survives the full
loopback intact: the raw pre-COBS payload genuinely contains the 0x0A byte
(sanity on the crafted input), the COBS-encoded wire frame does NOT
(the structural fix this test exists to guard), and the firmware decodes
the exact commanded velocity (read back live via
``sim_cmd_vel_left()``/``sim_cmd_vel_right()``) and drives the wheels
accordingly.

Deliberate-break sanity check (124-006 ticket, SUC-003 acceptance --
run once by hand during review, NOT part of this file or left in the
suite): reintroduce the 0x0A corruption by skipping the delimiter XOR in
``WireRuntime::cobsEncode()``/``cobsDecode()`` (``src/firm/messages/
wire_runtime.cpp``) -- i.e. treat every byte as delimiter-0x00-keyed
regardless of the ``delimiter`` argument -- rebuild
``src/sim/build/libfirmware_host.dylib`` (``just build-sim``), rerun this
file, confirm it fails, then revert and rebuild again. See the ticket's
own completion notes for the exact observed failure.

Run with::

    uv run python -m pytest src/tests/sim/system/test_sim_wire_loopback.py -v -s

Requires the compiled ``src/sim/build/libfirmware_host.{dylib,so}``
(``just build-sim``) -- skips cleanly if not present (same convention as
every other ``src/tests/sim/system/`` file in this tier).
"""
from __future__ import annotations

import pathlib
import struct
import sys

import pytest

# src/tests/sim/system/test_sim_wire_loopback.py -> system -> sim -> tests ->
# src -> repo root = FOUR hops from __file__ (same convention as this
# tier's sibling files, e.g. test_sim_configure_from_robot.py).
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_SIM_LIB_PATH = _REPO_ROOT / "src" / "sim" / "build" / _LIB_NAME

pytestmark = pytest.mark.skipif(
    not _SIM_LIB_PATH.exists(),
    reason="sim lib not built -- just build-sim (or cmake --build src/sim/build)",
)

_TRACK_WIDTH = 128.0  # [mm] matches tovez_nocal.json's own geometry.trackwidth
_MOTOR_DEADBAND = 15.0  # [mm/s] tovez_nocal.json's drive.motor_deadband -- stay comfortably above it

# Bounded polling budgets -- generous over the sim's own 40ms cycle period
# (never an unbounded/sleeping wait; every loop below is a fixed step count).
_ACK_POLL_CYCLES = 25
_DRIVE_SETTLE_CYCLES = 30


def _make_loop():
    """A bare, headless ``SimLoop`` -- deterministic manual stepping
    (``start_tick_thread=False``, ticket 009's own precedent), no
    ``SimTransport``, no Qt. Mirrors this tier's own established
    ``_make_loop()`` helper (``test_sim_configure_from_robot.py``,
    ``test_sim_boot_config_parity.py``) -- deliberately re-declared here
    rather than imported, matching this test tier's existing convention of
    each file owning its own tiny fixture rather than sharing one across
    files."""
    from robot_radio.io.sim_loop import SimLoop

    loop = SimLoop(track_width=_TRACK_WIDTH, lib_path=_SIM_LIB_PATH)
    loop.connect(start_tick_thread=False)
    return loop


def _configure(loop) -> None:
    """Push a real robot config through the FULL configure_from_robot()
    pipeline (Tier 1 wire ConfigDelta + Tier 2 direct configureMotor()
    calls) so RobotLoop's configuration-completeness gate opens and a MOVE
    is actually admitted (114-001) -- same pattern as
    ``test_sim_configure_from_robot.py``. One step() lets the Tier-1
    ConfigDelta (sitting in FakeTransport's inbound queue) actually get
    consumed by RobotLoop::handleConfig() before a caller sends a MOVE."""
    from robot_radio.config.robot_config import load_robot_config

    config = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")
    loop.configure_from_robot(config)
    loop.step(1)


def _velocity_with_embedded_delimiter_byte(
    lo: float = 90.0, hi: float = 250.0, step: float = 0.01, delimiter: int = 0x0A,
) -> float:
    """Computes a genuine, in-range ([90, 250] mm/s -- comfortably above
    tovez_nocal.json's own 15 mm/s motor_deadband, so the firmware
    actually drives, not just accepts-and-ignores) wheel-velocity value
    whose IEEE-754 float32 little-endian encoding contains a literal
    ``delimiter`` (0x0A) byte -- the exact 123-006 hazard shape (a
    MoveWheels envelope whose own serialized bytes embed a literal 0x0A),
    reproduced with a computed value rather than a hand-picked magic
    constant. Deterministic (fixed search order), not randomized."""
    v = lo
    while v <= hi:
        if delimiter in struct.pack("<f", v):
            return round(v, 2)
        v += step
    raise AssertionError(
        f"no velocity in [{lo}, {hi}] step {step} embeds a 0x{delimiter:02X} byte -- "
        "widen the search range"
    )


def _build_move_wheels_frame(
    corr_id: int, move_id: int, v_left: float, v_right: float,
    *, stop_time_ms: float = 4000.0, timeout_ms: float = 4000.0,
) -> "tuple[bytes, bytes, bytes]":
    """Builds ``CommandEnvelope{move: Move{wheels: MoveWheels{...}}}`` the
    exact way a real host does, and encodes it through the REAL production
    byte-level codec -- ``robot_radio.io.wire_codec.encode_frame()``
    (COBS, delimiter 0x0A, + CRC-16/CCITT-FALSE scoped over
    ``b"MOVE:" + payload``), the SAME function
    ``SimLoop.move()``/``serial_conn.py`` call for every wire-bound
    command. Returns ``(raw_payload, frame, wire_line)`` -- the
    pre-COBS schema bytes, the COBS+CRC frame body, and the complete
    ``<COMMAND>':'<frame>`` wire line -- so a caller can inspect each
    stage independently (this is exactly how ``test_move_wheels_with_
    embedded_0x0a_byte_round_trips_through_real_codec`` proves the byte
    codec, not a stub, ran)."""
    from robot_radio.io.wire_codec import encode_frame
    from robot_radio.robot.pb2 import envelope_pb2 as pb2

    envelope = pb2.CommandEnvelope(
        corr_id=corr_id,
        move=pb2.Move(
            wheels=pb2.MoveWheels(v_left=v_left, v_right=v_right),
            time=stop_time_ms, timeout=timeout_ms, replace=True, id=move_id,
        ),
    )
    raw_payload = envelope.SerializeToString()
    frame = encode_frame(raw_payload, command=b"MOVE")
    wire_line = b"MOVE:" + frame
    return raw_payload, frame, wire_line


def _build_stop_frame(corr_id: int) -> "tuple[bytes, bytes, bytes]":
    """Same shape as ``_build_move_wheels_frame()`` for
    ``CommandEnvelope{stop: Stop{}}`` -- the reverse-direction/no-config-
    required scenario (``RobotLoop::handleStop()`` acks unconditionally,
    robot_loop.cpp, regardless of the configuration-completeness gate)."""
    from robot_radio.io.wire_codec import encode_frame
    from robot_radio.robot.pb2 import envelope_pb2 as pb2

    envelope = pb2.CommandEnvelope(corr_id=corr_id, stop=pb2.Stop())
    raw_payload = envelope.SerializeToString()
    frame = encode_frame(raw_payload, command=b"STOP")
    wire_line = b"STOP:" + frame
    return raw_payload, frame, wire_line


def _drain_ack(loop, corr_id: int, cycles: int):
    """Steps ``loop`` up to ``cycles`` times, draining telemetry after
    each step and scanning the bounded ack ring (``TLMFrame.acks``,
    ALWAYS populated, oldest-first -- 120's own robust-match shape,
    preferred over the single "freshest ack" slot which a later frame
    could overwrite before a caller polls it) for an entry matching
    ``corr_id``. Returns the matching ``AckEntry``, or ``None`` if the
    budget is exhausted without seeing one."""
    for _ in range(cycles):
        loop.step(1)
        for frame in loop.drain_pending_tlm():
            for ack in frame.acks:
                if ack.corr_id == corr_id:
                    return ack
    return None


def test_move_wheels_with_embedded_0x0a_byte_round_trips_through_real_codec():
    """The money test: a MoveWheels command whose own serialized bytes
    embed a literal 0x0A (the 123-006 hazard shape) travels host-encode ->
    real firmware demux/decode/dispatch -> real firmware encode -> host
    demux/decode, and the exact commanded velocity comes out the other
    end -- proving the REAL byte-level codec ran in both directions, not
    a stub."""
    v = _velocity_with_embedded_delimiter_byte()
    corr_id, move_id = 40001, 50001
    raw_payload, frame, wire_line = _build_move_wheels_frame(corr_id, move_id, v, v)

    # Sanity on the crafted input: the hazard shape is genuinely present
    # pre-COBS...
    assert 0x0A in raw_payload, (
        f"velocity {v} was expected to embed a literal 0x0A byte in its serialized "
        f"MoveWheels payload but did not: {raw_payload!r}"
    )
    # ...and the REAL encode_frame()/COBS(delimiter=0x0A) composition strips/
    # XORs it out of the actual wire bytes -- the structural fix this test
    # exists to guard (123-006's own corruption class).
    assert 0x0A not in frame, (
        f"encode_frame() produced a frame still containing a literal 0x0A byte "
        f"(COBS delimiter keying is broken): {frame!r}"
    )

    loop = _make_loop()
    try:
        _configure(loop)

        loop.inject_command(wire_line)

        ack = _drain_ack(loop, corr_id, _ACK_POLL_CYCLES)
        assert ack is not None, (
            f"no ack for corr_id={corr_id} observed within {_ACK_POLL_CYCLES} cycles -- "
            "the real firmware demux/decode never dispatched our real-host-encoded MOVE"
        )
        assert ack.ok, f"MOVE was rejected: err_code={ack.err_code}"

        # Let the plant actually respond (the velocity-PID setpoint ramps
        # in via a slew-rate limiter, it is not instantaneous -- see this
        # file's own completion notes), then confirm real motion via a
        # SECOND real-firmware-encode/real-host-decode round trip (a fresh
        # Telemetry frame's enc_left/enc_right).
        frames = []
        for _ in range(_DRIVE_SETTLE_CYCLES):
            loop.step(1)
            frames.extend(loop.drain_pending_tlm())
        assert frames, "no telemetry frames decoded during the drive-settle window"

        # The firmware's live PID setpoint (NOT wire-visible telemetry --
        # read directly off the real NezhaMotor via the sim's diagnostic
        # ctypes accessor), now settled, must equal the EXACT velocity we
        # embedded a 0x0A byte inside of -- proof the firmware's real
        # msg::wire::decode() recovered the bit-exact float, not a
        # corrupted neighbor.
        cmd_vel_left = float(loop._lib.sim_cmd_vel_left(loop._handle))
        cmd_vel_right = float(loop._lib.sim_cmd_vel_right(loop._handle))
        assert cmd_vel_left == pytest.approx(v, abs=0.5), (
            f"left wheel PID setpoint {cmd_vel_left} != commanded {v} -- "
            "the embedded-0x0A velocity did not decode correctly"
        )
        assert cmd_vel_right == pytest.approx(v, abs=0.5), (
            f"right wheel PID setpoint {cmd_vel_right} != commanded {v} -- "
            "the embedded-0x0A velocity did not decode correctly"
        )

        last = frames[-1]
        assert last.enc_left is not None and last.enc_right is not None
        assert last.enc_left.position > 1.0, (
            f"left encoder position ({last.enc_left.position} mm) did not advance -- "
            "the decoded MOVE never actually reached the drivetrain"
        )
        assert last.enc_right.position > 1.0, (
            f"right encoder position ({last.enc_right.position} mm) did not advance -- "
            "the decoded MOVE never actually reached the drivetrain"
        )
    finally:
        loop.disconnect()


def test_stop_command_round_trips_through_real_codec_without_configuration():
    """Reverse-direction/no-config-required companion scenario: proves the
    loopback path is general (not MOVE-specific) and that
    RobotLoop::handleStop() acks OK unconditionally, over the SAME real
    encode -> real firmware demux/decode -> real firmware encode -> real
    host decode chain -- an UNCONFIGURED SimHarness (no
    configure_from_robot() call at all in this test)."""
    corr_id = 40002
    _, frame, wire_line = _build_stop_frame(corr_id)
    assert 0x0A not in frame, "STOP frame unexpectedly contains a literal 0x0A byte"

    loop = _make_loop()
    try:
        loop.inject_command(wire_line)

        ack = _drain_ack(loop, corr_id, _ACK_POLL_CYCLES)
        assert ack is not None, (
            f"no ack for corr_id={corr_id} observed within {_ACK_POLL_CYCLES} cycles"
        )
        assert ack.ok, f"STOP was rejected: err_code={ack.err_code}"
    finally:
        loop.disconnect()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
