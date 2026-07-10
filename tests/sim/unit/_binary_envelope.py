"""_binary_envelope.py -- shared binary-armor helpers for tests/sim/unit's
migrated text-verb suites (097-006, architecture-update-r2.md Decision 9).

Mirrors test_binary_channel.py's own armor()/dearmor()/send()/
send_no_tick() pattern (095-007) plus thin CommandEnvelope-building
wrappers over host/robot_radio/robot/legacy_translate.py's verb -> envelope
translators (097-002/004) -- the SAME translation `rogo`'s proxy (ticket
004) and NezhaProtocol (ticket 002) use, so a test built against this
helper exercises the identical wire shape a real legacy-text client now
gets via the proxy, not a bespoke test-only shape.

Not a test module itself (leading underscore, same convention as
_wire_diff_driver.py -- neither is pytest-collected). Imported by
test_bare_loop_commands.py, test_bare_loop_move_and_tlm.py,
test_dtr_verbs.py, and test_binary_channel.py.
"""
from __future__ import annotations

import base64
import pathlib
import sys

# tests/sim/unit/_binary_envelope.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_HOST_DIR = _REPO_ROOT / "host"
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))

from robot_radio.robot import legacy_translate  # noqa: E402
from robot_radio.robot.pb2 import drivetrain_pb2 as pb_drivetrain  # noqa: E402
from robot_radio.robot.pb2 import envelope_pb2 as pb_envelope  # noqa: E402
from robot_radio.robot.pb2 import motion_pb2 as pb_motion  # noqa: E402

# tests/sim/conftest.py's build_lib fixture already inserts this path; guard
# against a double-insert if this module is imported before that fixture
# runs (same guard test_binary_channel.py uses).
_SIM_INFRA_DIR = _REPO_ROOT / "tests" / "_infra" / "sim"
if str(_SIM_INFRA_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_INFRA_DIR))
from firmware import CHANNEL_SERIAL  # noqa: E402

# Re-exported so callers only need to import from this one module.
ERR_RANGE = pb_envelope.ERR_RANGE
ERR_BADARG = pb_envelope.ERR_BADARG


def armor(envelope: "pb_envelope.CommandEnvelope") -> str:
    return "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")


def dearmor(line: str) -> "pb_envelope.ReplyEnvelope":
    line = line.strip()
    assert line.startswith("*B"), f"expected an armored binary reply, got: {line!r}"
    raw = base64.b64decode(line[2:])
    reply = pb_envelope.ReplyEnvelope()
    reply.ParseFromString(raw)
    return reply


def send(sim, envelope: "pb_envelope.CommandEnvelope", channel: int = CHANNEL_SERIAL) -> "pb_envelope.ReplyEnvelope":
    """Send one binary command through the sim's dt=0 synchronous channel
    (ticks once, like every text `sim.command()` call already does)."""
    return dearmor(sim.command_on(armor(envelope), channel))


def send_no_tick(sim, envelope: "pb_envelope.CommandEnvelope") -> "pb_envelope.ReplyEnvelope":
    """Route one binary command WITHOUT the trailing tick -- see
    test_binary_channel.py's own send_no_tick() for why (peeking
    bb.segmentIn/bb.replaceIn before Drivetrain::tick() drains it)."""
    return dearmor(sim.route_no_tick(armor(envelope)))


# ---------------------------------------------------------------------------
# Verb -> envelope convenience wrappers, one per deleted text verb this
# sprint gives a binary parity arm. Each is a thin CommandEnvelope wrapper
# over legacy_translate.py's own translator, corr_id defaulted to 1 (these
# migrated tests never assert on corr_id -- their text-plane predecessors
# never echoed one either).
# ---------------------------------------------------------------------------


def send_drive(sim, l: float, r: float, corr_id: int = 1,  # noqa: E741 (l/r match the wire verb's own names)
               channel: int = CHANNEL_SERIAL) -> "pb_envelope.ReplyEnvelope":
    """Binary parity for the deleted text `S <l> <r>` (097-006) --
    legacy_translate.wheel_targets_for_drive() is the SAME translation
    `rogo`'s proxy (ticket 004) uses for a legacy S line."""
    wheels = legacy_translate.wheel_targets_for_drive(l, r)
    env = pb_envelope.CommandEnvelope(corr_id=corr_id, drive=pb_drivetrain.DrivetrainCommand(wheels=wheels))
    return send(sim, env, channel)


def send_segment(sim, seg: "pb_motion.MotionSegment", corr_id: int = 1,
                  channel: int = CHANNEL_SERIAL) -> "pb_envelope.ReplyEnvelope":
    """Binary parity for the deleted text D/T/RT/MOVE (097-006) -- all four
    posted one Motion::Segment to bb.segmentIn; the binary `segment` arm is
    the SAME destination."""
    env = pb_envelope.CommandEnvelope(corr_id=corr_id, segment=seg)
    return send(sim, env, channel)


def send_replace(sim, seg: "pb_motion.MotionSegment", corr_id: int = 1,
                  channel: int = CHANNEL_SERIAL) -> "pb_envelope.ReplyEnvelope":
    """Binary parity for the deleted text MOVER (097-006) -- posted to
    bb.replaceIn; the binary `replace` arm is the SAME destination."""
    env = pb_envelope.CommandEnvelope(corr_id=corr_id, replace=seg)
    return send(sim, env, channel)
