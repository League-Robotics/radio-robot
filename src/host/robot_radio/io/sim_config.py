"""robot_radio.io.sim_config -- SimConfigConn: relocated from
testgui/transport.py (ticket 113-005).

Sprint 113's Design Rationale Decision 3: a headless ``SimLoop`` caller
needs the same Tier-1 wire-push capability ``SimTransport``/TestGUI already
had via ``_SimConfigConn``, but that class used to live in
``testgui/transport.py`` with no genuine Qt/GUI dependency of its own --
its home was an accident of "TestGUI needed it first," not a real layering
requirement. Relocated here (``io/`` -- lower-level than ``testgui/``,
matching ``SimLoop``'s own layer) rather than either duplicated in both
places (two copies of the same knowledge, the exact bug class this sprint
exists to close) or imported by ``io/sim_loop.py`` FROM
``testgui/transport.py`` (a layering inversion). ``testgui/transport.py``
now imports this class under its old private name (``from
robot_radio.io.sim_config import SimConfigConn as _SimConfigConn``) instead
of defining its own copy -- see that module for the import site.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from robot_radio.io.wire_codec import encode_frame

if TYPE_CHECKING:
    from robot_radio.io.sim_loop import SimLoop
    from robot_radio.robot import protocol
    from robot_radio.robot.pb2 import envelope_pb2


def _envelope_command_name(envelope: "envelope_pb2.CommandEnvelope") -> bytes:
    """The ASCII wire-verb name for a populated ``pb2.CommandEnvelope``
    (124-005, issue §1/§3) -- its own oneof arm name upper-cased, matching
    the registry (``robot_radio.io.wire_commands``). Mirrors
    ``io/serial_conn.py``'s identically-named helper (a small, deliberate
    duplication rather than a cross-module import -- this module and
    ``serial_conn.py`` are peers, neither owns the other)."""
    which = envelope.WhichOneof("cmd")
    return (which or "config").upper().encode("ascii")


class SimConfigConn:
    """Duck-typed ``SerialConnection`` substitute so ``NezhaProtocol.
    set_config()``/``set_config_field()``/``set_config_group()``/
    ``get_config()`` can be reused VERBATIM against a ``SimLoop`` --
    Architecture Revision 1's "one mechanism, not a Sim-specific fork"
    (sprint 109): the exact same envelope-building/key-vocabulary code
    hardware transports use, just injected via ``SimLoop.inject_command()``
    instead of a live serial write.

    Implements ``send_envelope_fast()`` -- the method
    ``set_config_field()``/``set_config_group()``/``set_config()`` call on
    ``self._conn`` (duck-typed, no ``isinstance`` check inside it) -- and
    ``send_envelope()`` (113-005 addition, needed by ``get_config()``'s own
    BLOCKING ``_send_envelope()`` call, 132-011 -- the one CONFIG-arm
    outcome with a genuinely synchronous reply). Deliberately does NOT
    implement ``wait_for_ack()``: ``NezhaProtocol.wait_for_ack()``
    unconditionally
    re-wraps whatever ``self._conn.wait_for_ack()`` returns via
    ``AckEntry.from_pb2()``, which expects a RAW ``telemetry_pb2.AckEntry``
    (``.status``/``.corr_id``/``.err_code``) -- but ``SimLoop.
    read_pending_binary_tlm_frames()`` already returns adapted ``TLMFrame``/
    ``AckEntry`` dataclasses, one layer past that raw shape. Correlating the
    ack ring is this class's OWN job instead (``poll_ack()`` below), called
    directly by a caller (``SimTransport``, or ``SimLoop.
    configure_from_robot()``) rather than through
    ``NezhaProtocol.wait_for_ack()``.
    """

    def __init__(self, loop: "SimLoop") -> None:
        self._loop = loop

    def send_envelope_fast(self, envelope: "envelope_pb2.CommandEnvelope") -> int:
        """Assign a corr_id, frame (COBS+CRC, command-prefixed -- 124-005),
        and inject via ``SimLoop.inject_command()`` -- the exact framing
        (123-002/003/124-005) ``SerialConnection.send_envelope_fast()``
        writes to a real serial port (see that method's own docstring),
        minus the trailing ``'\\n'`` delimiter a live serial stream needs
        and a direct ``inject_command()`` call does not (``FakeTransport::
        enqueueInboundBinary()`` takes one already-delimited line per
        call).

        113-006 correction: corr_id comes from ``self._loop._next_corr_id()``
        -- the SAME thread-safe, monotonic counter ``SimLoop.twist()``/
        ``.stop()``/``.move()`` already share -- rather than a private
        per-instance counter. The original "this adapter is the only sender
        on this path, so no cross-source collision risk" assumption (this
        docstring's own prior wording) held only as long as exactly one
        ``SimConfigConn`` ever talked to a given ``SimLoop`` at a time. It
        broke the moment a SECOND, independent ``SimConfigConn`` entered the
        picture: ``SimTransport`` keeps its own long-lived one
        (``self._config_conn``, backing ``GET``/``SET`` and its host-side
        ack-echo cache) while ``SimLoop.configure_from_robot()`` constructs a
        throwaway one on every call. Two private counters both starting at 1
        assign the SAME corr_id to DIFFERENT wire commands; since neither
        this method nor ``configure_from_robot()`` drains the ack this class
        's own ``poll_ack()`` would eventually read, a later, unrelated
        ``poll_ack(corr_id=1, ...)`` call (e.g. ``SimTransport.
        _handle_config_set()``, triggered by ``_push_robot_calibration()``
        moments after a Sim ``connect()``) can match a STALE ack left behind
        by the other sender's corr_id=1 -- observed as a spurious ``ERR nak
        <key> err_code=...`` on a command that was never actually rejected.
        Routing every ``SimConfigConn`` instance through the ``SimLoop``'s
        own counter makes corr_ids globally unique for that ``SimLoop``
        regardless of how many ``SimConfigConn``/``NezhaProtocol`` pairs are
        constructed over its lifetime, closing the collision structurally."""
        corr_id = self._loop._next_corr_id()
        envelope.corr_id = corr_id
        command = _envelope_command_name(envelope)
        frame = encode_frame(envelope.SerializeToString(), command=command)
        self._loop.inject_command(command + b":" + frame)
        return corr_id

    def send_envelope(self, envelope: "envelope_pb2.CommandEnvelope",
                      read_timeout: int = 500,  # [ms]
                      ) -> dict:
        """``SerialConnection.send_envelope()``-compatible: needed because
        ``NezhaProtocol.set_config()`` (113-005's ``SimLoop.
        configure_from_robot()`` Tier-1 caller) reaches this method (not
        ``send_envelope_fast()``) via ``set_config_binary()``'s own
        ``_send_envelope()`` call, unlike ``config()`` above, which calls
        ``send_envelope_fast()`` directly.

        Fires the envelope the same way ``send_envelope_fast()`` does, then
        returns immediately with ``reply=None`` -- a CONFIG command's
        outcome rides the ack ring inside a LATER ``Telemetry`` push, never
        a synchronous ``ReplyEnvelope`` answering this specific corr_id
        (see ``NezhaProtocol.config()``'s own docstring), so there is
        nothing to synchronously wait for here; ``reply=None`` matches
        ``SerialConnection.send_envelope()``'s own "reply is None on
        timeout" shape, not a failure to send -- the envelope has already
        reached the sim by the time this returns. A caller that needs the
        ack outcome polls it separately via ``poll_ack()`` below (mirroring
        ``SimTransport._handle_config_set()``'s own established pattern).
        """
        self.send_envelope_fast(envelope)
        return {"sent": envelope, "mode": "sim", "reply": None}

    def wait_for_ack(self, corr_id: int, timeout: int = 500,  # [ms]
                     ) -> "int | None":
        """``SerialConnection.wait_for_ack()``-compatible: returns the RAW
        PACKED ring word (``corr_id<<4 | err``) for ``corr_id``, or ``None``
        on timeout.

        Added by the command-ingestion rework so ``NezhaProtocol`` is a
        complete, working client of this class rather than a partial one.
        ``NezhaProtocol.wait_for_ack()`` calls ``self._conn.wait_for_ack()``
        and re-wraps the result via ``AckEntry.from_ring_entry()``, which
        expects the packed int -- ``poll_ack()`` below returns an
        already-adapted ``AckEntry`` instead and therefore could not be used
        for that path. Both exist: ``poll_ack()`` for the direct callers
        that predate this (``SimTransport``, ``SimLoop.
        configure_from_robot()``), ``wait_for_ack()`` for anything driving
        this connection through ``NezhaProtocol``.

        Repacks rather than plumbs the raw word through: ``SimLoop.
        read_pending_binary_tlm_frames()`` hands back adapted ``TLMFrame``/
        ``AckEntry`` dataclasses, one layer past the wire's own packed
        ints, so the word is reconstructed here with the same packing
        ``Core::Telemetry::pushAckRing()`` uses.
        """
        entry = self.poll_ack(corr_id, timeout=timeout)
        if entry is None:
            return None
        return (entry.corr_id << 4) | (entry.err_code & 0xF)

    def read_pending_binary_tlm_frames(self) -> list:
        """``SerialConnection``-compatible non-blocking telemetry drain --
        straight through to the ``SimLoop``. Same reason as
        ``wait_for_ack()`` above: a caller driving this connection through
        ``NezhaProtocol`` (or reading telemetry directly, as the square-tour
        gate does) needs the identical method name on both backends."""
        return self._loop.read_pending_binary_tlm_frames()

    def poll_ack(self, corr_id: int, timeout: int = 500,  # [ms]
                ) -> "protocol.AckEntry | None":
        """Poll ``SimLoop.read_pending_binary_tlm_frames()``'s bounded ack
        ring for ``corr_id``, mirroring ``SerialConnection.wait_for_ack()``'s
        own re-delivery-tolerant matching (returns on the FIRST frame
        carrying a match) -- a small, Sim-local reimplementation rather than
        an import of that method's private ``_match_ack_in_frames()``
        helper, since that helper matches against raw ``pb2.ReplyEnvelope``
        objects (``reply.tlm.acks``, packed ``int`` entries -- 124-008,
        issue §B4) off ``drain_binary_tlm()``, not the already-adapted
        ``TLMFrame``/``AckEntry`` dataclasses ``SimLoop.
        read_pending_binary_tlm_frames()`` returns (``TLMFrame.acks`` --
        124-008 deleted the single "freshest ack" scalar slot/``TLMFrame.
        ack`` this method used to scan; ring membership alone now means
        "really acked")."""
        deadline = time.monotonic() + (timeout / 1000.0)
        while True:
            for frame in self._loop.read_pending_binary_tlm_frames():
                for entry in frame.acks:
                    if entry.corr_id == corr_id:
                        return entry
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.01)
