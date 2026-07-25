"""robot_radio.io.wire_codec -- host-side mirror of
``src/firm/messages/wire_runtime.h``'s COBS + CRC-16/CCITT-FALSE primitives
(sprint 123 tickets 001/002/003, the atomic COBS+CRC wire cutover).

This is the ONE place the host encodes/decodes the wire's binary framing --
every producer/consumer of a binary frame (``io/serial_conn.py``,
``io/sim_loop.py``, ``io/sim_config.py``, ``io/cli.py``,
``testgui/transport.py``, ``robot/protocol.py``, ``src/sim/sim_ctypes.cpp``'s
Python-side counterpart) imports from here rather than re-implementing the
byte-level codec at each call site -- mirrors ``wire_runtime.h``'s own
"ONE hand-written, schema-agnostic byte-level codec toolkit" role on the
firmware side (see that file's header comment for the analogous split
between schema-agnostic bytes and schema-specific field tables).

Replaces the pre-123 ``*B<base64>\\r\\n`` line armor: a binary frame is now
CRC-then-COBS composed (append the little-endian CRC-16 to the schema
payload, THEN COBS-encode the combined bytes) and delimited on the wire by a
single ``0x00`` byte, demuxed from the HELLO/PING text-plane rump
(``\\r\\n``-terminated lines) on the SAME byte stream -- see
``comms.h``/``comms.cpp`` (firmware) for the byte-for-byte contract this
module ports to Python.

Every primitive here is a pure function operating on ``bytes`` in, ``bytes``
out (or ``None``/an exception on malformed input) -- no I/O, no threading, no
protobuf-schema knowledge. ``ByteStreamDemuxer`` is the one stateful piece:
it accumulates raw bytes from a byte-oriented transport (a real serial port,
a loopback fake, ...) and yields complete text lines or complete binary frame
bodies, mirroring ``App::Transport::readLine()``'s own demux contract
(``comms.h``) so the host reads the exact two coexisting frame shapes the
firmware writes.

CRC-16/CCITT-FALSE decision (pinned, ticket 001's completion notes, must
match byte-for-byte -- there is no negotiation, no version byte):
    poly   = 0x1021
    init   = 0xFFFF
    refin  = False (no input reflection -- processed MSB-first)
    refout = False (no output reflection)
    xorout = 0x0000 (no final XOR)
Known-answer vector (CRC RevEng catalogue): ``crc16_ccitt_false(b"123456789")
== 0x29B1`` -- exercised as an exact-value test, not merely "some CRC
changed" (see ``src/tests/unit/test_wire_codec.py``).
"""

from __future__ import annotations

__all__ = [
    "WireFrameError",
    "crc16_ccitt_false",
    "cobs_encode",
    "cobs_decode",
    "cobs_encoded_max_length",
    "encode_frame",
    "decode_frame",
    "ByteStreamDemuxer",
    "FRAME_DELIMITER",
]

# Frame delimiter -- the SAME single byte the firmware's transports append
# after every COBS+CRC frame body (Transport::send()'s own doc comment,
# comms.h) and the same byte HELLO/PING text lines never contain (0x00 is
# exclusively the binary-frame delimiter; ASCII text never carries it).
FRAME_DELIMITER = b"\x00"

# ``_looks_like_text()``'s printable-ASCII alphabet -- see that function's
# own docstring for the 123-006 rationale (this module's 0x0A-vs-0x00
# discriminator, the host-side counterpart of serial_port.cpp's
# kTextCommands EXACT-match discriminator -- see that file's own comment for
# why the two sides use a DIFFERENT recognizer despite sharing the same
# 0x00-always-ends-binary/0x0A-conditionally-ends-text demux skeleton).
# Deliberately printable-ASCII ONLY (0x20-0x7E, space through tilde) -- NOT
# also tab/other control bytes: every real text-plane line this host
# receives (DEVICE:.../OK pong.../relay '#'-comment lines) is plain
# space-separated ASCII with no control bytes, and a short, zero-free COBS
# frame's own leading code byte is exactly "block length + 1" -- for a
# short envelope that can easily land on 0x09 (tab) by pure coincidence of
# length, so admitting tab here would silently readmit the same class of
# misclassification 123-006 fixed. Keep this alphabet as narrow as the real
# traffic requires, not broader.
_TEXT_SAFE_BYTES = frozenset(range(0x20, 0x7F))  # printable ASCII only


def _looks_like_text(data: bytes) -> bool:
    """True if every byte in ``data`` is drawn from ``_TEXT_SAFE_BYTES`` --
    the alphabet every real text-plane line this host ever receives is made
    of: the ``DEVICE:...`` banner, the ``OK pong t=<ms>`` reply (both
    dynamic-content replies to HELLO/PING -- NOT a fixed literal string, so
    an exact-match check like firmware's own ``kTextCommands`` cannot work
    here), and the RadioRelay's own ``#``-prefixed command-plane comment
    lines (``_relay_handshake()``'s pre-``!GO`` traffic, which never
    contains a single ``0x00`` byte at all).

    A genuine binary COBS+CRC frame's bytes are effectively arbitrary
    across the full ``0x01``-``0xFF`` range (COBS code bytes, the CRC-16
    tail, raw protobuf field bytes) -- in practice always containing at
    least one byte outside this printable range, which is exactly the
    123-006 bench-surfaced discriminator ``ByteStreamDemuxer.feed()`` uses
    to decide whether an accumulated ``0x0A`` ends a text line or is
    ordinary binary content (wait for the frame's own ``0x00``
    delimiter)."""
    return all(b in _TEXT_SAFE_BYTES for b in data)

# COBS block cap -- WireRuntime::kCobsMaxBlockLength (wire_runtime.h): a
# block of up to this many non-zero bytes is prefixed by one code byte
# (0xFF for a full block that hit this cap before finding a zero).
_COBS_MAX_BLOCK = 254

# CRC-16/CCITT-FALSE parameters -- see this module's own header comment.
_CRC16_POLY = 0x1021
_CRC16_INIT = 0xFFFF


class WireFrameError(ValueError):
    """Raised by ``cobs_encode()``/``cobs_decode()`` on malformed input or a
    destination that cannot hold the result. ``decode_frame()`` catches this
    internally and returns ``None`` instead of raising (mirrors
    ``Comms::decodeBinaryFrame()``'s own "count and drop, never propagate a
    partial/corrupt decode" contract) -- callers that want the lower-level
    ``cobs_decode()``/``cobs_encode()`` primitives directly still see this
    exception raised."""


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE over ``data`` -- byte-for-byte port of
    ``WireRuntime::crcCompute()`` (wire_runtime.cpp): MSB-first, no input/
    output reflection, no final XOR. See this module's header for the pinned
    parameters and the known-answer test vector."""
    crc = _CRC16_INIT
    for byte in data:
        crc = (crc ^ (byte << 8)) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ _CRC16_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encoded_max_length(raw_len: int) -> int:
    """Worst-case COBS-encoded length of ``raw_len`` bytes -- port of
    ``WireRuntime::cobsEncodedMaxLength()``: one code byte per <=254-byte
    block, exact only when the input has no embedded zero bytes."""
    return raw_len + raw_len // _COBS_MAX_BLOCK + 1


def cobs_encode(data: bytes) -> bytes:
    """Consistent Overhead Byte Stuffing encode -- byte-for-byte port of
    ``WireRuntime::cobsEncode()`` (wire_runtime.cpp). Removes every ``0x00``
    byte from ``data`` so the result can be delimited on the wire by a single
    ``0x00`` -- the caller's job (this primitive never appends that trailing
    delimiter itself, matching the C++ primitive's own documented
    boundary)."""
    out = bytearray()
    out.append(0)  # placeholder for the first block's code byte
    code_pos = 0
    code = 1  # distance to the next zero (or block end), inclusive of the code byte itself
    for byte in data:
        if byte == 0:
            out[code_pos] = code
            code_pos = len(out)
            out.append(0)  # placeholder for the next block
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                # Block hit the 254-non-zero-byte cap before finding a zero:
                # flush it as a full block and start a fresh one, exactly as
                # if a zero had been seen.
                out[code_pos] = code
                code_pos = len(out)
                out.append(0)
                code = 1
    out[code_pos] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Reverse of ``cobs_encode()`` -- byte-for-byte port of
    ``WireRuntime::cobsDecode()``. Raises ``WireFrameError`` on any malformed
    or truncated input: a literal ``0x00`` code byte, a literal ``0x00``
    inside a data block (an encoder never emits one), or a code byte whose
    claimed block length runs past the end of the input. Never returns a
    partial result."""
    if len(data) == 0:
        # Even the encoding of a zero-byte payload is exactly one code byte
        # (0x01) -- a truly empty input has nothing to decode.
        raise WireFrameError("cobs_decode(): empty input")

    out = bytearray()
    read_pos = 0
    n = len(data)
    while read_pos < n:
        code = data[read_pos]
        if code == 0:
            raise WireFrameError("cobs_decode(): literal 0x00 code byte")
        read_pos += 1
        block_len = code - 1
        if block_len > n - read_pos:
            raise WireFrameError("cobs_decode(): block length exceeds remaining input")
        block = data[read_pos:read_pos + block_len]
        if 0 in block:
            raise WireFrameError("cobs_decode(): literal 0x00 inside data block")
        out.extend(block)
        read_pos += block_len
        # A block whose code is < 0xFF was terminated by an actual zero byte
        # in the original data, UNLESS this is the last block in the frame
        # (frame just ended, no zero at all). A 0xFF-coded block hit the
        # 254-byte cap with no zero, so never emit one for it.
        if code != 0xFF and read_pos < n:
            out.append(0)
    return bytes(out)


def encode_frame(payload: bytes) -> bytes:
    """Encode ``payload`` (a schema-encoded protobuf message's raw bytes)
    into a COBS+CRC frame body -- CRC-then-COBS composition, matching
    ``Comms::sendReply()``/ticket 001's completion notes EXACTLY (NOT
    COBS-then-append-CRC, which would risk emitting a literal ``0x00`` if the
    CRC bytes happen to contain one): append the little-endian CRC-16 to
    ``payload``, THEN COBS-encode the combined bytes.

    Returns the COBS-encoded frame body -- 0x00-free by construction, NOT
    including the trailing ``0x00`` wire delimiter (append ``FRAME_DELIMITER``
    yourself when writing to a byte stream, matching
    ``Transport::send()``'s own division of labor: the framer builds the
    frame body, the concrete transport appends the delimiter)."""
    crc = crc16_ccitt_false(payload)
    combined = payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))
    return cobs_encode(combined)


def decode_frame(frame: bytes) -> bytes | None:
    """Reverse of ``encode_frame()`` -- byte-for-byte port of
    ``Comms::decodeBinaryFrame()``: COBS-decode, split off the trailing
    2-byte little-endian CRC, verify it against the leading payload bytes,
    and return the payload on success.

    Returns ``None`` on ANY malformed/corrupt input (malformed COBS, a
    combined-bytes length under 2, or a CRC mismatch) -- NEVER raises. This
    mirrors ``Comms::decodeBinaryFrame()``'s own "silently count and drop,
    never propagate a partial/corrupt decode" contract: a caller that wants
    fault visibility counts its own ``None`` returns (see
    ``SerialConnection``'s ``malformed_frame_count`` for the host-side
    counterpart of firmware's ``malformedCount_``)."""
    try:
        combined = cobs_decode(frame)
    except WireFrameError:
        return None
    if len(combined) < 2:
        return None
    payload, crc_bytes = bytes(combined[:-2]), combined[-2:]
    received_crc = crc_bytes[0] | (crc_bytes[1] << 8)
    if crc16_ccitt_false(payload) != received_crc:
        return None
    return payload


class ByteStreamDemuxer:
    """Accumulates raw bytes from a byte-oriented transport and demuxes them
    into complete text lines or complete binary COBS+CRC frame bodies --
    same demux SKELETON as ``App::Transport::readLine()``'s contract
    (``comms.h``) / ``SerialPort::readLine()``'s concrete implementation
    (``serial_port.cpp``): ``0x00`` ALWAYS ends a binary frame (COBS
    guarantees a frame body is 0x00-free by construction). ``0x0A`` ends a
    TEXT line ONLY when the bytes accumulated before it (a single trailing
    ``\\r`` stripped) are recognized as text (``_looks_like_text()``);
    otherwise the ``0x0A`` is binary CONTENT, not a terminator, and
    accumulation continues to the eventual ``0x00`` delimiter. 123-006
    bench-surfaced fix: COBS only guarantees 0x00-freedom, never
    0x0A-freedom -- a prior "whichever terminator comes first" rule split
    and corrupted any binary frame (e.g. a move_wheels envelope) that
    happened to embed a literal 0x0A byte, proven 0/10 on hardware.

    NOTE the recognizer itself is NOT a byte-for-byte mirror of firmware's
    own ``kTextCommands`` exact-match check (``serial_port.cpp``): firmware
    only ever receives two fixed literal commands inbound (``HELLO``,
    ``PING`` -- protocol-v4's whole text-plane rump), so an exact-string
    match is both correct and simplest there. This class instead demuxes
    bytes arriving FROM the firmware/relay, which are NOT a fixed literal
    set -- the ``DEVICE:...`` banner and ``OK pong t=<ms>`` reply both carry
    dynamic content, and the RadioRelay's own pre-``!GO`` ``#``-comment
    lines (``_relay_handshake()``, ``serial_conn.py``) are free-form too.
    ``_looks_like_text()``'s printable-ASCII content check is the
    discriminator that generalizes correctly across all of those shapes
    while still recognizing genuine binary COBS+CRC content (which is
    essentially never all-printable) as binary. See that function's own
    docstring.

    This is the piece that makes the host's serial reads safe against a
    binary frame that happens to embed a literal ``\\n`` (0x0A) as legitimate
    COBS+CRC content -- unlike the pre-123 base64 armor (whose alphabet
    excludes 0x0A by construction), a raw ``pyserial.Serial.readline()`` call
    is NOT safe to use directly against this wire any more; every raw read
    must go through an instance of this class first.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> "list[tuple[str, bytes]]":
        """Append ``data`` to the internal buffer; return every complete
        frame now available, in wire order, as ``(kind, payload)`` tuples --
        ``kind`` is ``"text"`` (payload is the line's bytes, a single
        trailing ``\\r`` stripped, delimiting ``\\n`` consumed) or
        ``"binary"`` (payload is the still-COBS+CRC-encoded frame body, the
        delimiting ``0x00`` consumed -- decode it with ``decode_frame()``).
        Never partially delivers a frame; leftover undelimited bytes stay
        buffered for the next ``feed()`` call. See this class's own
        docstring for the ``0x00``-vs-``0x0A`` discrimination rule."""
        self._buf.extend(data)
        out: "list[tuple[str, bytes]]" = []
        while True:
            idx_bin = self._buf.find(0x00)
            idx_text = self._buf.find(0x0A)
            if idx_bin == -1 and idx_text == -1:
                break
            # A 0x00 is present: everything up to it is ONE binary frame,
            # UNLESS a text line ends (at a 0x0A) strictly before it.
            if idx_bin != -1:
                if idx_text != -1 and idx_text < idx_bin:
                    line = bytes(self._buf[:idx_text])
                    if line.endswith(b"\r"):
                        line = line[:-1]
                    if _looks_like_text(line):
                        del self._buf[:idx_text + 1]
                        out.append(("text", line))
                        continue
                    # else: the 0x0A is binary content -- fall through to
                    # the 0x00 split below.
                frame = bytes(self._buf[:idx_bin])
                del self._buf[:idx_bin + 1]
                out.append(("binary", frame))
                continue
            # No 0x00 yet: only a 0x0A is present. It's a text line if it
            # looks like text; otherwise this is an incomplete binary frame
            # -- stop and wait for its 0x00 delimiter.
            line = bytes(self._buf[:idx_text])
            if line.endswith(b"\r"):
                line = line[:-1]
            if _looks_like_text(line):
                del self._buf[:idx_text + 1]
                out.append(("text", line))
                continue
            break
        return out
