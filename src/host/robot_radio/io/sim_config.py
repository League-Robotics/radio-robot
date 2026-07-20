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

import base64
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_radio.io.sim_loop import SimLoop
    from robot_radio.robot import protocol
    from robot_radio.robot.pb2 import envelope_pb2


class SimConfigConn:
    """Duck-typed ``SerialConnection`` substitute so ``NezhaProtocol.
    config()``/``NezhaProtocol.set_config()`` can be reused VERBATIM against
    a ``SimLoop`` -- Architecture Revision 1's "one mechanism, not a
    Sim-specific fork" (sprint 109): the exact same envelope-building/
    key-vocabulary code hardware transports use, just injected via
    ``SimLoop.inject_command()`` instead of a live serial write.

    Implements ``send_envelope_fast()`` -- the one method
    ``NezhaProtocol.config()`` calls on ``self._conn`` (duck-typed, no
    ``isinstance`` check inside it) -- and ``send_envelope()`` (113-005
    addition, needed by ``NezhaProtocol.set_config()``'s own
    ``set_config_binary()``/``_send_envelope()`` call chain, which
    ``config()`` does not use). Deliberately does NOT implement
    ``wait_for_ack()``: ``NezhaProtocol.wait_for_ack()`` unconditionally
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
        self._corr_counter = 0

    def send_envelope_fast(self, envelope: "envelope_pb2.CommandEnvelope") -> int:
        """Assign a corr_id (own counter -- this adapter is the only sender
        on this path, so no cross-source collision risk the way hardware's
        shared ``_corr_counter`` guards against), armor, and inject via
        ``SimLoop.inject_command()`` -- the exact ``*B<base64>`` shape
        ``SerialConnection.send_envelope_fast()`` writes to a real serial
        port (see that method's own docstring), minus the trailing
        newline framing a live serial stream needs and a direct
        ``inject_command()`` call does not (``FakeTransport::
        enqueueInbound()`` takes one already-delimited line per call)."""
        self._corr_counter += 1
        corr_id = self._corr_counter
        envelope.corr_id = corr_id
        armored = base64.b64encode(envelope.SerializeToString()).decode("ascii")
        self._loop.inject_command(f"*B{armored}")
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

    def poll_ack(self, corr_id: int, timeout: int = 500,  # [ms]
                ) -> "protocol.AckEntry | None":
        """Poll ``SimLoop.read_pending_binary_tlm_frames()``'s ack ring for
        ``corr_id``, mirroring ``SerialConnection.wait_for_ack()``'s own
        re-delivery-tolerant matching (returns on the FIRST frame carrying a
        match) -- a small, Sim-local reimplementation rather than an import
        of that method's private ``_match_ack_in_frames()`` helper, since
        that helper matches against raw ``pb2.ReplyEnvelope`` objects
        (``reply.tlm.acks``) off ``drain_binary_tlm()``, not the already-
        adapted ``TLMFrame``/``AckEntry`` dataclasses ``SimLoop.
        read_pending_binary_tlm_frames()`` returns."""
        deadline = time.monotonic() + (timeout / 1000.0)
        while True:
            for frame in self._loop.read_pending_binary_tlm_frames():
                if not frame.acks:
                    continue
                for ack in frame.acks:
                    if ack.corr_id == corr_id:
                        return ack
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.01)
