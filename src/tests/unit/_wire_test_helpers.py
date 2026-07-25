"""src/tests/unit/_wire_test_helpers.py -- shared host-side test doubles for
the sprint 123 COBS+CRC wire cutover (tickets 001/002/003).

Not collected by pytest itself (no ``test_``/``_test`` filename match) --
imported by ``test_serial_conn_binary_plane.py``, ``test_serial_conn_
telemetry_secondary.py``, and ``test_protocol_binary_client.py``, which all
need a raw-byte-oriented ``pyserial.Serial`` stand-in now that
``SerialConnection`` reads via ``.read(n)``/``.in_waiting`` (never
``.readline()`` -- see ``serial_conn.py``'s own module docstring for why a
binary COBS+CRC frame's content may embed a literal ``0x0A``, which makes a
naive ``readline()`` call unsafe as the SOLE demux mechanism the way it was
pre-123, when the wire's only binary-plane content was base64 text whose
alphabet excludes ``0x0A`` by construction).
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


def binary_frame(message) -> bytes:
    """COBS+CRC-frame ``message`` (any protobuf message with
    ``SerializeToString()``) and append the trailing 0x00 wire delimiter --
    the on-wire replacement for the pre-123 ``("*B" + base64 + "\\n").encode()``
    shape."""
    return encode_frame(message.SerializeToString()) + b"\x00"


def text_line(text: str) -> bytes:
    """A ``\\r\\n``-terminated text-plane line (HELLO/PING and their
    replies) -- unchanged framing from pre-123."""
    return (text + "\r\n").encode("ascii")
