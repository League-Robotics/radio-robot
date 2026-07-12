"""tests/unit/test_sim_conn_binary_plane.py -- SimConnection binary transport
(protocol-v3, stakeholder-directed: first step of migrating the Robot Test
GUI's sim path off the gutted text plane onto binary).

Tests ``SimConnection.send_envelope()``/``drain_binary_tlm()`` against the
REAL compiled sim library (``just build-sim``) -- the binary-plane
counterpart, for the ``SimConnection`` (ctypes) backend, of
``test_serial_conn_binary_plane.py``'s own coverage of
``SerialConnection.send_envelope()``/``drain_binary_tlm()`` for the
hardware backend. Also mirrors ``tests/sim/unit/test_binary_channel.py``'s
own armor/dearmor ``CommandEnvelope`` shapes, reused here through
``legacy_translate.py`` so the drive envelope this file builds is the SAME
``wheel_targets_for_drive()`` translation ``cli.py``'s ``rogo binary drive``
builder and ``_binary_envelope.py``'s ``send_drive()`` helper already use.

Requires the compiled ``libfirmware_host`` (``just build-sim``) -- the
session-scoped ``build_lib`` fixture below runs it once per test session,
duplicated (not shared) from ``tests/sim/conftest.py``'s own fixture of the
same name: ``tests/unit`` and ``tests/sim`` are independent pytest-collected
domains with no shared ``conftest.py`` (see ``tests/CLAUDE.md``'s "three
domains -- never combined" note; ``tests/unit`` is host-side unit/tooling,
not sim-domain-scoped, but this one file needs the compiled sim library as
its unit under test IS the ctypes bridge to it).

Collected under ``tests/unit/`` -- ``pyproject.toml``'s ``testpaths``
includes it, so ``uv run python -m pytest`` (and
``uv run python -m pytest tests/unit -q`` in isolation) both collect it.
"""

from __future__ import annotations

import pathlib
import subprocess
import sys

import pytest

from robot_radio.io.sim_conn import CHANNEL_RADIO, CHANNEL_SERIAL, SimConnection
from robot_radio.robot import legacy_translate
from robot_radio.robot.pb2 import drivetrain_pb2, envelope_pb2
from robot_radio.robot.protocol import TLMFrame

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session")
def build_lib() -> None:
    """Build libfirmware_host once per test session (`just build-sim`) --
    duplicated from tests/sim/conftest.py's own fixture of the same name;
    see this module's own header comment for why it is not shared."""
    subprocess.run(["just", "build-sim"], cwd=_REPO_ROOT, check=True)


@pytest.fixture
def conn(build_lib: None):
    """A connected SimConnection, disconnected in a finally so a failing
    test still frees its SimHandle (mirrors tests/sim/conftest.py's own
    `sim` fixture's try/finally shape)."""
    c = SimConnection()
    result = c.connect()
    assert "error" not in result, result
    try:
        yield c
    finally:
        c.disconnect()


# ---------------------------------------------------------------------------
# send_envelope() -- ping round trip, not-connected, corr_id passthrough.
# ---------------------------------------------------------------------------


def test_send_envelope_ping_round_trips(conn):
    """A binary ping through send_envelope() replies ReplyEnvelope{ok},
    with corr_id echoed back UNCHANGED (this module's own simplification
    vs. SerialConnection.send_envelope() -- see that method's docstring:
    the sim never overwrites envelope.corr_id, since there is no
    concurrent-request corr-id pool to manage)."""
    env = envelope_pb2.CommandEnvelope(corr_id=7)
    env.ping.SetInParent()

    reply = conn.send_envelope(env)

    assert reply is not None
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 7


def test_send_envelope_not_connected_returns_none():
    c = SimConnection()  # never connect()ed
    env = envelope_pb2.CommandEnvelope()
    env.ping.SetInParent()

    assert c.send_envelope(env) is None


def test_send_envelope_stop_posts_neutral_brake(conn):
    """Binary parity for the deleted text STOP -- a spinning direct-mode
    drive settles to zero after a binary `stop` sent via send_envelope(),
    matching test_binary_channel.py's own test_binary_stop_posts_neutral_
    brake assertion shape but through the SimConnection host wrapper."""
    wheels = legacy_translate.wheel_targets_for_drive(150.0, 150.0)
    drive_env = envelope_pb2.CommandEnvelope(
        corr_id=1, drive=drivetrain_pb2.DrivetrainCommand(wheels=wheels))
    reply = conn.send_envelope(drive_env)
    assert reply.WhichOneof("body") == "ok"

    conn.tick(1000)
    vel_l, vel_r = conn.get_true_velocity()
    assert vel_l > 50.0 and vel_r > 50.0, "binary drive never reached the plant"

    stop_env = envelope_pb2.CommandEnvelope(corr_id=2, stop=envelope_pb2.Stop())
    reply = conn.send_envelope(stop_env)
    assert reply.WhichOneof("body") == "ok"

    conn.tick(1000)
    vel_l, vel_r = conn.get_true_velocity()
    assert vel_l == pytest.approx(0.0, abs=10.0)
    assert vel_r == pytest.approx(0.0, abs=10.0)


def test_send_envelope_selects_radio_channel(conn):
    """channel=CHANNEL_RADIO routes the request AND reads the reply back
    from the RADIO ReplyStore (mirrors send_on()'s own channel selection,
    088-006) -- proven by a plain ping round-tripping identically on
    either channel."""
    env = envelope_pb2.CommandEnvelope(corr_id=9)
    env.ping.SetInParent()

    reply = conn.send_envelope(env, channel=CHANNEL_RADIO)

    assert reply is not None
    assert reply.WhichOneof("body") == "ok"
    assert reply.corr_id == 9


# ---------------------------------------------------------------------------
# drain_binary_tlm() -- empty when nothing armed, populated once streaming
# is armed, decodes via TLMFrame.from_pb2().
# ---------------------------------------------------------------------------


def test_drain_binary_tlm_empty_when_nothing_armed(conn):
    assert conn.drain_binary_tlm() == []


def test_drain_binary_tlm_not_connected_returns_empty_list():
    c = SimConnection()  # never connect()ed
    assert c.drain_binary_tlm() == []


def test_send_envelope_drive_then_drain_binary_tlm_shows_cmd_vel_and_enc(conn):
    """The scenario this ticket exists for: build a CommandEnvelope{drive},
    send_envelope() it, tick, then read binary Telemetry (via the new
    drain) showing the drive actually reached the plant -- decoded through
    TLMFrame.from_pb2(), the SAME adapter NezhaProtocol's binary telemetry
    client (protocol.py) uses for the hardware path."""
    wheels = legacy_translate.wheel_targets_for_drive(150.0, 150.0)
    drive_env = envelope_pb2.CommandEnvelope(
        corr_id=1, drive=drivetrain_pb2.DrivetrainCommand(wheels=wheels))
    reply = conn.send_envelope(drive_env)
    assert reply.WhichOneof("body") == "ok"

    stream_env = envelope_pb2.CommandEnvelope(
        corr_id=2, stream=envelope_pb2.StreamControl(binary=True, period=50))
    reply = conn.send_envelope(stream_env)
    assert reply.WhichOneof("body") == "ok"

    conn.tick(500)   # let tickTelemetry() emit several periodic frames

    frames = conn.drain_binary_tlm()
    assert frames, "expected at least one binary telemetry push frame"
    for frame in frames:
        assert frame.WhichOneof("body") == "tlm"
        assert frame.corr_id == 0, "unsolicited push frames must carry corr_id=0"

    tlm = TLMFrame.from_pb2(frames[-1].tlm)
    assert tlm.enc is not None
    assert tlm.enc[0] > 0 and tlm.enc[1] > 0, "binary TLM enc= never advanced"
    assert tlm.cmd_vel is not None
    assert tlm.cmd_vel[0] > 50 and tlm.cmd_vel[1] > 50, (
        "binary TLM cmd_vel= never reflected the drive command"
    )


def test_drain_binary_tlm_drains_only_the_requesting_channel(conn):
    """A stream armed on CHANNEL_RADIO must land its periodic frames on
    CHANNEL_RADIO's own drain, never CHANNEL_SERIAL's -- mirrors
    test_binary_channel.py's own test_binary_stream_binds_periodic_
    emission_to_the_requesting_channel, through the SimConnection wrapper."""
    reply = conn.send_envelope(
        envelope_pb2.CommandEnvelope(
            corr_id=3, stream=envelope_pb2.StreamControl(binary=True, period=50)),
        channel=CHANNEL_RADIO,
    )
    assert reply.WhichOneof("body") == "ok"

    conn.tick(240)

    radio_frames = conn.drain_binary_tlm(channel=CHANNEL_RADIO)
    serial_frames = conn.drain_binary_tlm(channel=CHANNEL_SERIAL)

    assert len(radio_frames) >= 3, f"expected >= 3 periodic frames on RADIO, got {len(radio_frames)}"
    assert serial_frames == [], "a RADIO-armed stream must never emit on SERIAL"


def test_drain_binary_tlm_frames_have_strictly_increasing_seq(conn):
    reply = conn.send_envelope(envelope_pb2.CommandEnvelope(
        corr_id=4, stream=envelope_pb2.StreamControl(binary=True, period=50)))
    assert reply.WhichOneof("body") == "ok"

    conn.tick(240)
    frames = conn.drain_binary_tlm()

    seqs = [f.tlm.seq for f in frames]
    assert len(seqs) >= 3
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)


# ---------------------------------------------------------------------------
# Text plane stays unchanged: send()/command() still work alongside the new
# binary methods on the SAME connection (dual-stack coexistence, mirrors
# test_binary_channel.py's own test_mixed_text_and_binary_session).
# ---------------------------------------------------------------------------


def test_text_plane_send_unaffected_by_binary_additions(conn):
    resp = conn.send("PING", read_timeout=200)
    assert any("pong" in line for line in resp["responses"])

    env = envelope_pb2.CommandEnvelope(corr_id=5)
    env.ping.SetInParent()
    reply = conn.send_envelope(env)
    assert reply.WhichOneof("body") == "ok"

    resp = conn.send("PING", read_timeout=200)
    assert any("pong" in line for line in resp["responses"])


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
