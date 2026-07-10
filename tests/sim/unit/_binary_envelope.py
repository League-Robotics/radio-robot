"""_binary_envelope.py -- shared binary-armor helpers for tests/sim/unit's
migrated text-verb suites (097-006, architecture-update-r2.md Decision 9).

Mirrors test_binary_channel.py's own armor()/dearmor()/send()/
send_no_tick() pattern (095-007) plus thin CommandEnvelope-building
wrappers over host/robot_radio/robot/legacy_translate.py's verb -> envelope
translators (097-002/004) -- the SAME translation `rogo`'s proxy (ticket
004) and NezhaProtocol (ticket 002) use, so a test built against this
helper exercises the identical wire shape a real legacy-text client now
gets via the proxy, not a bespoke test-only shape.

097-008 adds arm_stream()/latest_tlm()/read_tlm_now(): the one-shot text
`TLM` verb (handleTlm, motion_commands.cpp) is deleted along with STREAM/
SNAP -- there is no binary one-shot TLM arm (096 Open Question 2 / 097
Decision 4's own finding, which is why NezhaProtocol.snap() -- host/
robot_radio/robot/protocol.py -- synthesizes a one-shot read from the
binary `stream` arm instead: arm, wait for a periodic frame, disarm).

read_tlm_now() is the primary entry point most migrated tests use: it
re-arms (which also RESETS the target channel's reply store, a send()/
command_on() side effect) and ticks exactly one pass so `tickTelemetry()`
-- which only ever runs from a real `sim_tick()`/`tick_for()` pass, never
from `sim.command()`/`command_on()`'s own dt=0 synchronous-command replay,
see sim_api.cpp's own `sim_command_on()` doc comment -- gets a chance to
emit exactly one fresh frame, then reads it. This costs one extra
`step`-ms tick versus the deleted text TLM's own dt=0 read, which is fine
for a point-in-time "what does it report right now" check (every migrated
test in this file except one).

An EARLIER version of this helper tried "arm once at the top of a test,
then peek many times for free" (no per-read tick cost at all) reasoning
that `sim.peek_reply_store()` is non-destructive. That is true but
insufficient: `tests/_infra/sim/sim_api.cpp`'s `ReplyStore` is a small
FIXED-size buffer with NO wraparound (`append()`'s own guard: once full,
every further append silently no-ops) -- arming once and then ticking a
multi-second test (as several of these tests do) fills it within roughly
the first 10-14 periodic frames and FREEZES it there, so a "latest" read
taken later in the test silently returns a STALE mid-motion frame, not the
current state. This is why read_tlm_now() re-arms (resets the store)
immediately before every read, so at most one frame is ever buffered.

The one exception, test_pivot_completes_promptly_single_peaked, polls "is
it idle yet" on nearly every iteration of a tight per-tick loop where an
extra tick per read would silently double the plant's effective simulated
time per iteration, corrupting the exact single-peak/prompt-idle timing
that test exists to verify -- it uses `sim.active()` (a direct,
zero-cost `bb.drivetrain.busy` peek, `tests/_infra/sim/sim_api.cpp`'s
`sim_get_active()`) instead of the telemetry wire at all. arm_stream()/
latest_tlm() are kept as the lower-level building blocks read_tlm_now()
itself composes, in case a future test needs the two steps split apart.

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


# ---------------------------------------------------------------------------
# arm_stream() / latest_tlm() / read_tlm_now() -- binary parity for the
# deleted one-shot text `TLM` verb (097-008). See this module's own header
# comment for the full rationale.
# ---------------------------------------------------------------------------


def arm_stream(sim, period: int = 20, channel: int = CHANNEL_SERIAL,
                corr_id: int = 9000) -> "pb_envelope.ReplyEnvelope":
    """Arm (or re-arm) binary periodic telemetry at `period` (20ms floor).
    A `send()` call, so it ALSO resets `channel`'s reply store as a side
    effect -- read_tlm_now() below relies on that to keep the store from
    ever accumulating more than one pass's worth of frames."""
    return send(sim, pb_envelope.CommandEnvelope(
        corr_id=corr_id, stream=pb_envelope.StreamControl(binary=True, period=period)), channel)


def latest_tlm(sim, channel: int = CHANNEL_SERIAL):
    """Non-destructively read the MOST RECENT binary periodic TLM frame
    tickTelemetry() has appended to `channel`'s reply store -- a
    msg::Telemetry (pb2), or None if none has landed yet."""
    text = sim.peek_reply_store(channel)
    frames = [dearmor(line) for line in text.splitlines() if line.strip()]
    return frames[-1].tlm if frames else None


def read_tlm_now(sim, channel: int = CHANNEL_SERIAL, step: int = 24, corr_id: int = 9000):
    """Point-in-time binary TLM read: re-arm (which resets the store),
    tick exactly one `step`-ms pass so tickTelemetry() gets a chance to
    emit, then read the (necessarily singular, so necessarily freshest)
    frame. Costs one extra `step`-ms tick versus the deleted one-shot text
    TLM's own dt=0 read -- see this module's own header comment for why
    that is fine for a point-in-time check but NOT for a per-iteration
    precision loop (use sim.active() there instead)."""
    arm_stream(sim, channel=channel, corr_id=corr_id)
    sim.tick_for(step)
    frame = latest_tlm(sim, channel)
    assert frame is not None, (
        "read_tlm_now(): no periodic binary TLM frame landed after one tick post-arm"
    )
    return frame
