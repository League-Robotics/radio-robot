"""robot_radio.io.wire_codec -- host-side mirror of
``src/firm/messages/wire_runtime.h``'s COBS + CRC-16/CCITT-FALSE primitives
(sprint 123 tickets 001/002/003, sprint 124 ticket 005's protocol v5 framing
grammar cutover).

This is the ONE place the host encodes/decodes the wire's binary framing --
every producer/consumer of a binary frame (``io/serial_conn.py``,
``io/sim_loop.py``, ``io/sim_config.py``, ``io/cli.py``,
``testgui/transport.py``, ``robot/protocol.py``, ``src/sim/sim_ctypes.cpp``'s
Python-side counterpart) imports from here rather than re-implementing the
byte-level codec at each call site -- mirrors ``wire_runtime.h``'s own
"ONE hand-written, schema-agnostic byte-level codec toolkit" role on the
firmware side (see that file's header comment for the analogous split
between schema-agnostic bytes and schema-specific field tables).

Protocol v5 (124-005, issue protocol-v5-one-line-packets-command-prefix-and-
newline-cobs.md §1-§3): every wire packet, text or binary, in both
directions, is exactly one line -- ``<COMMAND>[':' <data>]'\\n'``. A binary
frame's ``<data>`` is CRC-then-COBS composed (append the little-endian
CRC-16 to the schema payload, THEN COBS-encode the combined bytes), the CRC
scoped over ``COMMAND ':' payload`` (not payload alone), and delimited on
the wire by a single ``0x0A`` (``'\\n'``) byte -- COBS is keyed on 0x0A now,
not 0x00 (see ``cobs_encode()``'s own docstring for why this makes the
terminator genuinely unconditional). Which verb a line names, and whether
its data is cleartext or binary, is looked up in the generated command
registry (``robot_radio.io.wire_commands``) -- see ``comms.h``/``comms.cpp``
(firmware) for the byte-for-byte contract this module ports to Python.

Every primitive here is a pure function operating on ``bytes`` in, ``bytes``
out (or ``None``/an exception on malformed input) -- no I/O, no threading, no
protobuf-schema knowledge. ``ByteStreamDemuxer`` is the one stateful piece:
it accumulates raw bytes from a byte-oriented transport (a real serial port,
a loopback fake, ...) and yields complete ``'\\n'``-terminated lines,
mirroring ``App::Transport::readLine()``'s own contract (``comms.h``).
124-005: this is now a PLAIN split-on-``'\\n'`` -- there is no more text-vs-
binary demux at this layer at all (that heuristic, ``_looks_like_text()``,
is deleted, not adapted -- see that function's own former docstring for why
it existed and why the uniform grammar makes it unnecessary). A real
transport's own ``readline()`` (e.g. ``pyserial``'s) is safe against this
wire again for the same reason: a binary line's COBS-encoded bytes never
contain a literal ``0x0A``.

CRC-16/CCITT-FALSE decision (pinned, ticket 001's completion notes, must
match byte-for-byte -- there is no negotiation, no version byte):
    poly   = 0x1021
    init   = 0xFFFF
    refin  = False (no input reflection -- processed MSB-first)
    refout = False (no output reflection)
    xorout = 0x0000 (no final XOR)
Known-answer vector (CRC RevEng catalogue): ``crc16_ccitt_false(b"123456789")
== 0x29B1`` -- exercised as an exact-value test, not merely "some CRC
changed" (see ``src/tests/unit/test_host_wire_codec.py``).
"""

from __future__ import annotations

__all__ = [
    "WireFrameError",
    "crc16_ccitt_false",
    "crc16_init",
    "crc16_update",
    "cobs_encode",
    "cobs_decode",
    "cobs_encoded_max_length",
    "encode_frame",
    "decode_frame",
    "ByteStreamDemuxer",
    "FRAME_DELIMITER",
]

# Frame delimiter -- the SAME single byte the firmware's transports append
# after every wire line (Transport::send()'s own doc comment, comms.h),
# protocol v5 (124-005, issue §2/§7): '\n' (0x0A), not the pre-124 0x00 --
# COBS is keyed on 0x0A now (see cobs_encode()'s own docstring), so a
# binary line's own bytes never contain a literal 0x0A, making this
# terminator genuinely unconditional in both directions.
FRAME_DELIMITER = b"\n"

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


def crc16_init() -> int:
    """Initial CRC-16/CCITT-FALSE register value -- the starting point for
    an incremental ``crc16_update()`` chain. Byte-for-byte port of
    ``WireRuntime::crcInit()`` (wire_runtime.cpp/.h item 9, 124-003):
    exposed so a caller composing a CRC over multiple byte ranges (see
    ``encode_frame()``/``decode_frame()`` below) never needs to
    concatenate those ranges into one ``bytes`` object first."""
    return _CRC16_INIT


def crc16_update(crc: int, data: bytes) -> int:
    """Continue a running CRC-16/CCITT-FALSE computation with more bytes --
    byte-for-byte port of ``WireRuntime::crcUpdate()``. ``crc16_ccitt_false(
    data) == crc16_update(crc16_init(), data)`` exactly (same loop body) --
    this is the incremental primitive that equivalence, and the CRC-scope
    composition in ``encode_frame()``/``decode_frame()``, are built on."""
    for byte in data:
        crc = (crc ^ (byte << 8)) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ _CRC16_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE over ``data`` -- byte-for-byte port of
    ``WireRuntime::crcCompute()`` (wire_runtime.cpp): MSB-first, no input/
    output reflection, no final XOR. See this module's header for the pinned
    parameters and the known-answer test vector."""
    return crc16_update(crc16_init(), data)


def cobs_encoded_max_length(raw_len: int) -> int:
    """Worst-case COBS-encoded length of ``raw_len`` bytes -- port of
    ``WireRuntime::cobsEncodedMaxLength()``: one code byte per <=254-byte
    block, exact only when the input has no embedded zero bytes."""
    return raw_len + raw_len // _COBS_MAX_BLOCK + 1


def cobs_encode(data: bytes, delimiter: int = 0x00) -> bytes:
    """Consistent Overhead Byte Stuffing encode -- byte-for-byte port of
    ``WireRuntime::cobsEncode()`` (wire_runtime.cpp). Removes every
    occurrence of ``delimiter`` from ``data`` so the result can be
    delimited on the wire by a single instance of that same byte -- the
    caller's job (this primitive never appends that trailing delimiter
    itself, matching the C++ primitive's own documented boundary).

    ``delimiter`` defaults to ``0x00``, matching every pre-124 call site
    byte-for-byte (XOR-ing by ``0x00`` is the identity -- see below). 124-003
    (issue §2) needs ``0x0A`` for the same reason ``wire_runtime.h``'s own
    item 8 doc comment gives: COBS guarantees ``0x00``-freedom but nothing
    about any other byte value, and a literal ``0x0A`` in the payload
    corrupts a ``\\n``-delimited wire.

    Mechanism: run the standard ``0x00``-keyed COBS algorithm exactly as
    before, but XOR every byte written to the output (data bytes AND code
    bytes) with ``delimiter`` at the moment it is finalized -- equivalent to
    computing the whole ``0x00``-keyed encoding first and XOR-ing every
    output byte afterward (XOR is position-wise, order-independent), just
    without a second pass. This never emits a byte equal to ``delimiter``:
    the pre-XOR output is ``0x00``-free by construction (code bytes are
    ``>= 1``, data bytes are non-zero), and ``b ^ delimiter == delimiter``
    iff ``b == 0``, which never occurs."""
    out = bytearray()
    out.append(0)  # placeholder for the first block's code byte
    code_pos = 0
    code = 1  # distance to the next zero (or block end), inclusive of the code byte itself
    for byte in data:
        if byte == 0:
            out[code_pos] = code ^ delimiter
            code_pos = len(out)
            out.append(0)  # placeholder for the next block
            code = 1
        else:
            out.append(byte ^ delimiter)
            code += 1
            if code == 0xFF:
                # Block hit the 254-non-zero-byte cap before finding a zero:
                # flush it as a full block and start a fresh one, exactly as
                # if a zero had been seen.
                out[code_pos] = code ^ delimiter
                code_pos = len(out)
                out.append(0)
                code = 1
    out[code_pos] = code ^ delimiter
    return bytes(out)


def cobs_decode(data: bytes, delimiter: int = 0x00) -> bytes:
    """Reverse of ``cobs_encode()`` -- byte-for-byte port of
    ``WireRuntime::cobsDecode()``. Raises ``WireFrameError`` on any malformed
    or truncated input: a literal ``0x00`` code byte, a literal ``0x00``
    inside a data block (an encoder never emits one), or a code byte whose
    claimed block length runs past the end of the input. Never returns a
    partial result.

    ``delimiter`` mirrors ``cobs_encode()``'s own parameter (default
    ``0x00``, identity XOR, byte-identical to every pre-124 call site): every
    byte read from ``data`` is XOR-ed with ``delimiter`` at the point of
    reading (rather than materializing a de-XORed copy first) -- this
    recovers exactly the ``0x00``-keyed bytes the standard algorithm below
    already knows how to walk, and it means the malformed-input checks below
    need no change for a non-``0x00`` delimiter: a literal ``delimiter`` byte
    inside a frame (impossible from a correct encoder) reads back as a
    literal ``0x00`` post-XOR, tripping the same rejections."""
    if len(data) == 0:
        # Even the encoding of a zero-byte payload is exactly one code byte
        # (0x01) -- a truly empty input has nothing to decode.
        raise WireFrameError("cobs_decode(): empty input")

    out = bytearray()
    read_pos = 0
    n = len(data)
    while read_pos < n:
        code = data[read_pos] ^ delimiter
        if code == 0:
            raise WireFrameError("cobs_decode(): literal 0x00 code byte")
        read_pos += 1
        block_len = code - 1
        if block_len > n - read_pos:
            raise WireFrameError("cobs_decode(): block length exceeds remaining input")
        block = bytes(b ^ delimiter for b in data[read_pos:read_pos + block_len])
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


def _crc_over_scope(command: bytes, payload: bytes) -> int:
    """The CRC-scope composition protocol v5 needs (124-003, issue §3):
    ``crc16(COMMAND ':' payload)`` when ``command`` is non-empty,
    ``crc16(payload)`` alone (byte-identical to protocol v4's CRC) when it
    is not -- byte-for-byte port of ``comms.cpp``'s ``crcOverScope()``.
    Built on ``crc16_init()``/``crc16_update()`` so ``command`` and
    ``payload`` are never concatenated into one ``bytes`` object just to
    hash them together."""
    crc = crc16_init()
    if command:
        crc = crc16_update(crc, command)
        crc = crc16_update(crc, b":")
    return crc16_update(crc, payload)


def encode_frame(payload: bytes, command: bytes = b"") -> bytes:
    """Encode ``payload`` (a schema-encoded protobuf message's raw bytes)
    into a COBS+CRC frame body -- CRC-then-COBS composition, matching
    ``Comms::sendReply()``/ticket 001's completion notes EXACTLY (NOT
    COBS-then-append-CRC, which would risk emitting a literal delimiter byte
    if the CRC bytes happen to contain one): append the little-endian CRC-16
    to ``payload``, THEN COBS-encode the combined bytes with delimiter
    ``0x0A`` (124-005, issue §2).

    ``command`` -- the ASCII command-name bytes (no ``':'`` separator) the
    CRC's input is scoped to extend over (124-003/124-005, issue §3); a
    SEPARATE argument, never concatenated with ``payload`` before
    COBS-encoding (the command is NOT part of the COBS input -- only its CRC
    scope). Defaults to empty, which extends nothing: ``crc16(payload)``
    alone. Every PRODUCTION caller now passes the real registry verb name
    (e.g. ``b"MOVE"``) -- the wire's own leading ``<COMMAND>':'`` prefix is a
    SEPARATE concern this function does not build (that's the caller's job,
    e.g. ``serial_conn.py``'s send paths); this function only scopes the CRC
    and COBS-encodes the data half.

    Returns the COBS-encoded frame body -- ``0x0A``-free by construction,
    NOT including the trailing ``'\\n'`` wire delimiter (append
    ``FRAME_DELIMITER`` yourself when writing to a byte stream, matching
    ``Transport::send()``'s own division of labor: the framer builds the
    frame body, the concrete transport appends the delimiter) and NOT
    including the leading ``<COMMAND>':'`` prefix either."""
    crc = _crc_over_scope(command, payload)
    combined = payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))
    return cobs_encode(combined, delimiter=0x0A)


def decode_frame(frame: bytes, command: bytes = b"") -> bytes | None:
    """Reverse of ``encode_frame()`` -- byte-for-byte port of
    ``Comms::decodeBinaryFrame()``: COBS-decode (delimiter ``0x0A``,
    124-005), split off the trailing 2-byte little-endian CRC, verify it
    (CRC-scoped over ``command`` too, per 124-003/124-005 -- see
    ``encode_frame()``'s own doc comment) against the leading payload
    bytes, and return the payload on success. ``frame`` is the COBS body
    ONLY -- the wire line's leading ``<COMMAND>':'`` prefix must already be
    stripped off by the caller (that parse is grammar, not this function's
    job -- see ``serial_conn.py``'s own dispatch).

    Returns ``None`` on ANY malformed/corrupt input (malformed COBS, a
    combined-bytes length under 2, or a CRC mismatch -- including a
    ``command`` that does not match what the frame was actually encoded
    with, e.g. a command byte mutated in transit) -- NEVER raises. This
    mirrors ``Comms::decodeBinaryFrame()``'s own "silently count and drop,
    never propagate a partial/corrupt decode" contract: a caller that wants
    fault visibility counts its own ``None`` returns (see
    ``SerialConnection``'s ``malformed_frame_count`` for the host-side
    counterpart of firmware's ``malformedCount_``)."""
    try:
        combined = cobs_decode(frame, delimiter=0x0A)
    except WireFrameError:
        return None
    if len(combined) < 2:
        return None
    payload, crc_bytes = bytes(combined[:-2]), combined[-2:]
    received_crc = crc_bytes[0] | (crc_bytes[1] << 8)
    if _crc_over_scope(command, payload) != received_crc:
        return None
    return payload


class ByteStreamDemuxer:
    """Accumulates raw bytes from a byte-oriented transport and demuxes them
    into complete ``'\\n'``-terminated wire LINES -- matches
    ``App::Transport::readLine()``'s own contract (``comms.h``) /
    ``SerialPort::readLine()``'s concrete implementation
    (``serial_port.cpp``): protocol v5's uniform grammar (124-005, issue
    §1/§7) makes ``'\\n'`` (0x0A) an UNCONDITIONAL terminator in both
    directions -- there is no more text-vs-binary demux at this layer at
    all. This is safe because COBS is now keyed on 0x0A (item 8,
    ``wire_runtime.h``): a binary line's own bytes never contain a literal
    0x0A, so a plain split-on-``'\\n'`` can never misfire the way the
    pre-124 0x00-vs-0x0A heuristic (``_looks_like_text()``, DELETED) could.

    Collapsed from the pre-124-005 two-terminator skeleton (0x00 always
    ended a binary frame, 0x0A conditionally ended a text line depending on
    printable-ASCII content) to this single rule specifically because
    123-006 proved the OLD skeleton could misclassify: a ``move_wheels``
    envelope embedding a literal 0x0A byte was split and corrupted 0/10 on
    hardware. Under the new grammar that failure mode cannot recur -- the
    terminator really is unconditional now, which is also why a real
    transport's own ``readline()`` (e.g. ``pyserial``'s) is safe to use
    directly against this wire again; this class is retained for the
    non-blocking partial-chunk buffering ``_reader_loop()``'s own
    ``ser.read(n)``-based polling still needs, not because a plain
    ``readline()`` would be unsafe any more.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> "list[bytes]":
        """Append ``data`` to the internal buffer; return every complete
        line now available, in wire order -- ``'\\n'`` consumed, NOT
        included. Never partially delivers a line; leftover undelimited
        bytes stay buffered for the next ``feed()`` call. A caller
        classifies each returned line as cleartext or binary by parsing its
        own ``<COMMAND>`` prefix and looking it up in the registry
        (``robot_radio.io.wire_commands``) -- this class has no opinion."""
        self._buf.extend(data)
        out: "list[bytes]" = []
        while True:
            idx = self._buf.find(0x0A)
            if idx == -1:
                break
            out.append(bytes(self._buf[:idx]))
            del self._buf[:idx + 1]
        return out
