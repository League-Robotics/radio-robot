"""src/tests/unit/_wire_test_helpers.py -- shared host-side test doubles for
the sprint 123/124 COBS+CRC wire cutover (tickets 001/002/003, 124-005's
protocol v5 framing grammar cutover).

Not collected by pytest itself (no ``test_``/``_test`` filename match) --
imported by ``test_serial_conn_binary_plane.py``, ``test_serial_conn_
telemetry_secondary.py``, and ``test_protocol_binary_client.py``, which all
need a raw-byte-oriented ``pyserial.Serial`` stand-in now that
``SerialConnection`` reads via ``.read(n)``/``.in_waiting`` (never
``.readline()`` -- see ``serial_conn.py``'s own module docstring for why a
binary COBS+CRC frame's content may legitimately embed a literal ``0x0A``
under protocol v4's split terminators, which made a naive ``readline()``
call unsafe as the SOLE demux mechanism; protocol v5's uniform grammar
(124-005) actually restores that safety -- see ``ByteStreamDemuxer``'s own
docstring -- but this fake keeps the ``.read(n)``/``.in_waiting`` contract
regardless, matching ``SerialConnection``'s own real non-blocking read
strategy, which this fake exists to double).
"""

from __future__ import annotations

from robot_radio.io.wire_codec import encode_frame


class FakeSerial:
    """Byte-buffer-backed stand-in for ``pyserial.Serial``, exposing exactly
    the surface ``SerialConnection``'s reader needs: ``is_open``,
    ``in_waiting``, ``read(n)``. Construct with the complete raw byte stream
    the fake should "arrive" over the wire (build it from ``binary_frame()``/
    ``text_line()`` below); ``read()`` hands it out in caller-requested
    chunks and RAISES once exhausted, mimicking a closed port -- the same
    "reader loop exits on its own" contract the old ``readline()``-based fake
    had (``_reader_loop()``'s own ``except Exception: break``)."""

    is_open = True

    def __init__(self, data: bytes = b"") -> None:
        self._buf = bytearray(data)

    @property
    def in_waiting(self) -> int:
        return len(self._buf)

    def read(self, size: int = 1) -> bytes:
        if not self._buf:
            raise RuntimeError("fake serial exhausted (mimics a closed port)")
        n = max(1, min(size, len(self._buf)))
        chunk = bytes(self._buf[:n])
        del self._buf[:n]
        return chunk


def binary_frame(message, command: bytes) -> bytes:
    """Build one COMPLETE `<COMMAND>':'<COBS+CRC bytes>'\\n'` wire LINE
    (124-005, issue §1/§3/§7) for ``message`` (any protobuf message with
    ``SerializeToString()``) -- the on-wire replacement for the pre-123
    ``("*B" + base64 + "\\n").encode()`` shape, and for 123-002/003's own
    unprefixed, 0x00-delimited COBS+CRC frame. ``command`` is REQUIRED
    (e.g. ``b"MOVE"``/``b"TLM"``) -- protocol v5 has no unscoped binary
    frame any more."""
    return command + b":" + encode_frame(message.SerializeToString(), command=command) + b"\n"


def text_line(text: str) -> bytes:
    """A ``'\\n'``-terminated cleartext-plane line (HELLO/PING/ID/VER and
    their replies) -- 124-005, issue §7: the pre-124 ``"\\r\\n"`` is retired
    along with the rest of the two-terminator split (a real firmware never
    emits ``\\r`` for anything cleartext any more; a leading terminal's own
    ``\\r\\n`` is a SEPARATE, colon-less-line-only affordance -- see
    ``serial_conn.py``'s ``_split_wire_line()`` -- not something this
    fixture needs to simulate)."""
    return (text + "\n").encode("ascii")
