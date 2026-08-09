"""src/tests/unit/test_wire_grammar.py -- sprint 124 ticket 005 (protocol v5
Part A, "framing grammar cutover"): host-side proof of the uniform packet
grammar (issue protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
§1), mirroring app_comms_harness.cpp's own grammar-edge-case scenarios
(C++ side) at the ``robot_radio.io.serial_conn`` layer (host side).

Ticket 005's own testing note deliberately does NOT add these cases as ROWS
to the shared ``src/tests/fixtures/wire_golden_vectors.txt`` fixture (ticket
004): that fixture's schema (``name|delimiter_hex|command|payload_hex|
expected_wire_hex|source``) describes the COBS+CRC byte COMPOSITION only --
a positive encode/decode round-trip proof, cross-language. "Data containing
':'" and "data containing 0x00" already had that exact composition-level
coverage BEFORE this ticket (the fixture's own ``payload_contains_colon_
and_zero`` row, ticket 004) -- proving both codecs agree that a colon or a
NUL byte inside COBS-encoded content never confuses the codec layer. The
three remaining edge cases the ticket names -- a no-data verb with a stray
trailing ':', an unknown command, and a truncated line -- are not codec-
composition vectors at all: they are PARSING/dispatch-layer behaviors (there
is no "expected_wire_hex" for an unrecognized command; it never decodes to
anything). Those belong at the grammar-PARSING layer, tested against each
language's own dispatcher (``Core::Comms::dispatchLine()`` in C++,
``_split_wire_line()``/``_handle_wire_line()`` here) -- exactly what this
file (mirroring app_comms_harness.cpp's own scenario set) and that harness
provide, kept in sync by both reading the SAME issue section rather than a
shared byte fixture (there is no byte fixture to share for a parse
decision).

Also covers acceptance item 7 (issue's own AC): a demonstration that a
PLAIN, un-demuxed ``readline()`` call is now safe against this wire --
protocol v5's uniform grammar makes '\\n' an unconditional terminator in
both directions (COBS is keyed on 0x0A, wire_runtime.h item 8), so a raw
serial read no longer needs ``ByteStreamDemuxer``'s own byte-level demux to
avoid misinterpreting an embedded '\\n' inside binary content -- the demuxer
is retained only for its non-blocking partial-chunk buffering, not because
a bare ``readline()`` would misparse.
"""

from __future__ import annotations

import io

from robot_radio.io import wire_commands
from robot_radio.io.serial_conn import SerialConnection, _split_wire_line
from robot_radio.io.wire_codec import decode_frame, encode_frame

# ---------------------------------------------------------------------------
# _split_wire_line() -- the host mirror of Core::Comms::dispatchLine()'s own
# `<COMMAND>[':' <data>]` parse (comms.cpp).
# ---------------------------------------------------------------------------


def test_split_wire_line_no_data_verb():
    assert _split_wire_line(b"HELLO") == ("HELLO", b"")


def test_split_wire_line_cleartext_data_with_further_colons():
    """DEVICE:'s own byte-frozen shape -- only the FIRST ':' ends the
    command; every later ':' is data (issue §1)."""
    assert _split_wire_line(b"DEVICE:NEZHA2:robot:my-bot:1234") == (
        "DEVICE", b"NEZHA2:robot:my-bot:1234")


def test_split_wire_line_binary_data_containing_a_colon_byte():
    """A COBS-encoded binary body may legitimately contain a literal ':'
    (0x3A) byte -- only the FIRST ':' in the whole LINE (right after the
    command name) is the grammar's own separator; everything after it,
    including further ':' bytes, is opaque data (issue §1)."""
    payload = b"foo:bar\x00baz"  # deliberately embeds ':' and 0x00
    frame = encode_frame(payload, command=b"MOVE")
    line = b"MOVE:" + frame

    command, data = _split_wire_line(line)
    assert command == "MOVE"
    assert data == frame  # the WHOLE COBS body, not truncated at an embedded ':'
    assert decode_frame(data, command=b"MOVE") == payload


def test_split_wire_line_stray_trailing_colon_on_no_data_verb():
    """A no-data verb with a stray trailing ':' (e.g. a client sending
    "PING:" instead of bare "PING") still parses as that verb with EMPTY
    data -- handled gracefully, not malformed. Mirrors app_comms_harness.
    cpp's scenarioStrayTrailingColonOnNoDataVerbHandledGracefully()."""
    assert _split_wire_line(b"PING:") == ("PING", b"")


def test_split_wire_line_unknown_command():
    """A command name not in the registry -- (None, b""), the caller's own
    signal to count it malformed rather than crash. Mirrors
    app_comms_harness.cpp's scenarioMalformedUnrecognizedTextLineRejected()."""
    assert _split_wire_line(b"NOTAREALCOMMAND") == (None, b"")
    assert _split_wire_line(b"NOTAREALCOMMAND:withdata") == (None, b"")


def test_split_wire_line_truncated_binary_line():
    """A registered BINARY verb with NO data at all ("MOVE:", colon but an
    empty COBS body) still parses to a real verb + empty data -- the
    TRUNCATION itself is caught one layer up, at cobs_decode()'s own "empty
    input" rejection (decode_frame() returns None), never here. Mirrors
    app_comms_harness.cpp's scenarioTruncatedBinaryLineCountsMalformedNotCrash()."""
    command, data = _split_wire_line(b"MOVE:")
    assert command == "MOVE"
    assert data == b""
    assert decode_frame(data, command=b"MOVE") is None  # truncated -- caught here, not raised


def test_split_wire_line_carriage_return_stripped_only_for_colonless_line():
    """A raw terminal's own "\\r\\n" line ending: the trailing '\\r' is
    stripped ONLY for a colon-less (no-data) verb candidate (issue §7) --
    never for data after a ':', which may legitimately end in a real '\\r'
    byte (binary content, or -- in principle -- cleartext data)."""
    assert _split_wire_line(b"PING\r") == ("PING", b"")
    # A colon-having line's data keeps its own trailing '\r' -- this
    # class of line is never emitted by real firmware/host today (every
    # data-carrying verb sends bare '\n'), but the grammar itself does not
    # special-case it, matching dispatchLine()'s own behavior (comms.cpp).
    command, data = _split_wire_line(b"DEVICE:x\r")
    assert command == "DEVICE"
    assert data == b"x\r"


def test_every_registered_verb_round_trips_through_split_wire_line():
    """Every verb in the generated registry (messages/commands.h's mirror)
    parses back out of its own bare, no-data wire form -- a closed-set
    sanity check that ``_split_wire_line()`` and ``wire_commands.VERB_BY_
    NAME`` (both mirrors of the SAME generated schema) agree on the exact
    verb set."""
    for entry in wire_commands.VERBS:
        command, data = _split_wire_line(entry.name.encode("ascii"))
        assert command == entry.name
        assert data == b""


# ---------------------------------------------------------------------------
# Acceptance item 7: a plain readline() call is safe against this wire again
# -- the demonstration that ByteStreamDemuxer is no longer load-bearing.
# ---------------------------------------------------------------------------


def test_plain_readline_recovers_every_line_including_embedded_0x00():
    """Builds a synthetic multi-line wire stream -- a cleartext DEVICE:
    banner, a MOVE command whose COBS body is engineered to contain a
    literal 0x00 byte (the exact shape that, under protocol v4's old
    0x00-delimited framing, would have been misread as a frame boundary),
    and a PONG: reply -- and recovers every line with io.BytesIO's own
    PLAIN ``readline()`` (never ``ByteStreamDemuxer``). Protocol v5's COBS-
    keyed-on-0x0A framing (issue §2) guarantees no line's own bytes ever
    contain a literal '\\n', so this is safe by construction, not by luck:
    if it were NOT safe, this test's own 500-byte 0x0A-content payload
    below would provoke a truncated/misparsed line and one of the
    assertions would fail."""
    banner = b"DEVICE:NEZHA2:robot:my-bot:1234\n"

    # Force at least one embedded 0x00 byte into the COBS body: an output
    # byte equals 0x00 exactly when the underlying (pre-XOR) COBS byte
    # equals the delimiter 0x0A (see cobs_encode()'s own docstring) -- a
    # payload of literal 0x0A bytes is passed through as ordinary DATA
    # (non-zero, so COBS never treats it as a block boundary) and comes out
    # XOR-ed to 0x00 in the wire bytes, deterministically.
    payload = bytes([0x0A, 1, 0x0A, 2, 0x0A, 3])
    frame = encode_frame(payload, command=b"MOVE")
    assert 0x00 in frame
    assert 0x0A not in frame
    move_line = b"MOVE:" + frame + b"\n"

    pong = b"PONG:t=123456\n"

    stream = io.BytesIO(banner + move_line + pong)

    line1 = stream.readline()
    line2 = stream.readline()
    line3 = stream.readline()
    assert stream.readline() == b""  # exhausted -- no partial/leftover bytes

    assert line1 == banner
    assert line2 == move_line
    assert line3 == pong

    # And each recovered line parses/decodes correctly end to end.
    command1, data1 = _split_wire_line(line1[:-1])
    assert command1 == "DEVICE"
    assert data1 == b"NEZHA2:robot:my-bot:1234"

    command2, data2 = _split_wire_line(line2[:-1])
    assert command2 == "MOVE"
    assert decode_frame(data2, command=b"MOVE") == payload

    command3, data3 = _split_wire_line(line3[:-1])
    assert command3 == "PONG"
    assert data3 == b"t=123456"


def test_plain_readline_via_a_serialconnection_shaped_fake():
    """Same demonstration, one layer up: a minimal fake exposing exactly
    pyserial's own ``readline()`` contract (not ``SerialConnection``'s
    ``.read(n)``/``.in_waiting`` -- this is deliberately NOT going through
    ``_reader_loop()``/``ByteStreamDemuxer`` at all) still recovers a real,
    fully-framed MOVE command line untouched, proving a plain readline()
    based reader COULD be built against this wire -- the demuxer is a
    buffering convenience now, not a correctness requirement."""

    class _ReadlineOnlySerial:
        def __init__(self, data: bytes) -> None:
            self._buf = io.BytesIO(data)

        def readline(self) -> bytes:
            return self._buf.readline()

    payload = bytes([0x0A, 9, 0x0A, 8])
    frame = encode_frame(payload, command=b"STOP")
    line = b"STOP:" + frame + b"\n"

    fake = _ReadlineOnlySerial(line)
    raw = fake.readline()
    assert raw == line

    command, data = _split_wire_line(raw[:-1])
    assert command == "STOP"
    assert decode_frame(data, command=b"STOP") == payload

    # SerialConnection itself is untouched by this test -- constructing one
    # only to prove the import/module wiring is sound (no live I/O).
    assert SerialConnection().is_open is False
