"""src/tests/unit/test_wire_codec.py -- sprint 123 tickets 001/002/003 (the
atomic COBS+CRC wire cutover): pure-Python tests for
``robot_radio.io.wire_codec``, the host-side byte-for-byte mirror of
``src/firm/messages/wire_runtime.h``'s COBS + CRC-16/CCITT-FALSE primitives.

Covers the "host-side half of SUC-002" acceptance criterion from ticket 003
(corrupted frame dropped via CRC, not mis-parsed) plus round-trip/known-answer
coverage for the codec itself. No hardware, no ``src/sim`` ctypes bridge --
these are pure functions over ``bytes``.
"""

import os

import pytest

from robot_radio.io.wire_codec import (
    ByteStreamDemuxer,
    WireFrameError,
    cobs_decode,
    cobs_encode,
    cobs_encoded_max_length,
    crc16_ccitt_false,
    crc16_init,
    crc16_update,
    decode_frame,
    encode_frame,
)


# ---------------------------------------------------------------------------
# CRC-16/CCITT-FALSE
# ---------------------------------------------------------------------------


def test_crc16_known_answer_vector():
    """CRC RevEng catalogue's own check value for CRC-16/CCITT-FALSE --
    pinned exactly the same as wire_runtime.h's own doc comment, so firmware
    and host agree byte-for-byte."""
    assert crc16_ccitt_false(b"123456789") == 0x29B1


def test_crc16_empty_input():
    assert crc16_ccitt_false(b"") == 0xFFFF  # init value, no bits processed


def test_crc16_changes_on_any_single_bit_flip():
    data = bytes(range(32))
    base_crc = crc16_ccitt_false(data)
    for i in range(len(data)):
        for bit in range(8):
            mutated = bytearray(data)
            mutated[i] ^= 1 << bit
            assert crc16_ccitt_false(bytes(mutated)) != base_crc


# ---------------------------------------------------------------------------
# CRC-16/CCITT-FALSE incremental primitives (124-003, issue §3): crc16_init()/
# crc16_update() are the base crc16_ccitt_false() is built on, and the
# primitive encode_frame()/decode_frame()'s CRC-scope composition uses so a
# command-name prefix and a payload -- two byte ranges that are not one
# ``bytes`` object -- can be hashed together without concatenating them first.
# ---------------------------------------------------------------------------


def test_crc16_incremental_matches_crc16_ccitt_false():
    data = b"123456789"
    assert crc16_update(crc16_init(), data) == crc16_ccitt_false(data)
    assert crc16_ccitt_false(data) == 0x29B1  # sanity: still the pinned value


def test_crc16_incremental_composes_across_two_ranges():
    """Composing over TWO separate ranges (prefix, then suffix) matches
    crc16_ccitt_false() over their concatenation -- the property
    _crc_over_scope() (and comms.cpp's crcOverScope()) are built on."""
    prefix = b"MOVE:"
    suffix = bytes([0x08, 0x07, 0x1A, 0x00, 0x22, 0x03])
    via_concatenation = crc16_ccitt_false(prefix + suffix)

    crc = crc16_init()
    crc = crc16_update(crc, prefix)
    crc = crc16_update(crc, suffix)
    assert crc == via_concatenation


# ---------------------------------------------------------------------------
# COBS encode/decode
# ---------------------------------------------------------------------------


def test_cobs_roundtrip_random_payloads():
    rng = os.urandom
    for _ in range(500):
        n = rng(1)[0] % 400
        data = rng(n)
        encoded = cobs_encode(data)
        assert 0 not in encoded, "COBS output must never contain a 0x00 byte"
        assert len(encoded) <= cobs_encoded_max_length(len(data))
        assert cobs_decode(encoded) == data


def test_cobs_roundtrip_empty_payload():
    encoded = cobs_encode(b"")
    assert encoded == b"\x01"
    assert cobs_decode(encoded) == b""


def test_cobs_roundtrip_payload_full_of_zeros():
    data = b"\x00" * 10
    encoded = cobs_encode(data)
    assert 0 not in encoded
    assert cobs_decode(encoded) == data


def test_cobs_roundtrip_block_boundary_254_nonzero_bytes():
    """Exercises the 254-byte block-cap boundary (0xFF code byte, no zero
    terminator) explicitly, not just via random fuzzing."""
    data = bytes((i % 255) + 1 for i in range(254))  # 254 non-zero bytes
    assert 0 not in data
    encoded = cobs_encode(data)
    assert cobs_decode(encoded) == data


def test_cobs_decode_rejects_empty_input():
    with pytest.raises(WireFrameError):
        cobs_decode(b"")


def test_cobs_decode_rejects_literal_zero_code_byte():
    with pytest.raises(WireFrameError):
        cobs_decode(b"\x00\x01\x02")


def test_cobs_decode_rejects_truncated_block():
    # Code byte claims 5 data bytes follow; only 2 are present.
    with pytest.raises(WireFrameError):
        cobs_decode(bytes([6, 1, 2]))


def test_cobs_decode_rejects_literal_zero_inside_data_block():
    # A well-formed encoder never emits a 0x00 inside a data block.
    with pytest.raises(WireFrameError):
        cobs_decode(bytes([3, 1, 0]))


# ---------------------------------------------------------------------------
# COBS delimiter parameterization (124-003, issue §2): cobs_encode()/
# cobs_decode() gained a ``delimiter`` parameter, default 0x00.
# ---------------------------------------------------------------------------


def test_cobs_default_delimiter_matches_explicit_0x00():
    """Omitting ``delimiter`` matches passing 0x00 explicitly, byte-for-byte
    -- every pre-124 call site keeps computing the exact same output."""
    data = bytes([0x41, 0x00, 0x0A, 0x00, 0x42, 0x00])
    assert cobs_encode(data) == cobs_encode(data, delimiter=0x00)
    # A literal 0x0A survives untouched in the output when delimiter=0x00 --
    # only 0x00 is special under the pre-124 default.
    assert 0x0A in cobs_encode(data)


def test_cobs_delimiter_0x0a_adversarial_vectors_from_issue():
    """The exact worked/adversarial vectors ``protocol-v5-...md`` §2 verified
    by hand: all-0x0A, all-0x00, and a 0x00..0x0F sweep, each run through the
    SAME "payload + CRC, then COBS(delimiter=0x0A)" composition the issue's
    own table describes -- asserted against the EXACT wire bytes the issue
    lists, not just "some encoding happened."

    124-004: these vectors used to be HARDCODED here, independently, as a
    second copy of the exact same table hardcoded into
    ``wire_runtime_harness.cpp``'s ``scenarioCobsKeyedOn0x0AAdversarialVectors``
    -- exactly the "two hand-maintained copies with no shared vector forcing
    them to agree" defect class the issue itself is about. Both now read
    ``src/tests/fixtures/wire_golden_vectors.txt``, the ONE shared fixture
    (see ``test_wire_golden_vectors.py`` for the full byte-for-byte
    cross-language suite this table is now a thin wrapper over); this test
    stays as a readable, issue-section-scoped subset for anyone landing on
    this file first."""
    from test_wire_golden_vectors import load_golden_vectors

    issue_vectors = [v for v in load_golden_vectors() if v.source == "issue_section2_table"]
    assert len(issue_vectors) == 3, "expected exactly the issue's own 3-row §2 table"

    for vector in issue_vectors:
        crc = crc16_ccitt_false(vector.payload)
        combined = vector.payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))
        wire = cobs_encode(combined, delimiter=0x0A)

        assert wire == vector.expected_wire, f"{vector.name}: wire mismatch"
        assert 0x0A not in wire, f"{vector.name}: wire contains a literal 0x0A"

        decoded_combined = cobs_decode(wire, delimiter=0x0A)
        assert decoded_combined[:-2] == vector.payload, f"{vector.name}: round-trip mismatch"


def test_cobs_delimiter_0x0a_no_literal_and_roundtrips_up_to_251_bytes():
    """Property test (issue AC #3): for payloads up to 251 bytes, the
    encoded frame contains no 0x0A byte and decode round-trips -- a
    deterministic 0x00-0xFF sweep plus random sampling, not just one shape."""
    # Every byte value at least once.
    sweep = bytes(range(256))
    encoded = cobs_encode(sweep, delimiter=0x0A)
    assert 0x0A not in encoded
    assert cobs_decode(encoded, delimiter=0x0A) == sweep

    # Random payloads up to 251 bytes (the issue's own affordable ceiling,
    # §6), including the empty payload.
    for _ in range(500):
        n = os.urandom(1)[0] % 252
        data = os.urandom(n)
        encoded = cobs_encode(data, delimiter=0x0A)
        assert 0x0A not in encoded
        assert cobs_decode(encoded, delimiter=0x0A) == data


# ---------------------------------------------------------------------------
# Frame encode/decode (CRC-then-COBS composition)
# ---------------------------------------------------------------------------


def test_frame_roundtrip_random_payloads():
    for _ in range(500):
        n = os.urandom(1)[0] % 250
        payload = os.urandom(n)
        frame = encode_frame(payload)
        assert 0x0A not in frame  # 124-005: COBS is keyed on 0x0A now, not 0x00
        assert decode_frame(frame) == payload


def test_frame_roundtrip_empty_payload():
    frame = encode_frame(b"")
    assert decode_frame(frame) == b""


def test_decode_frame_rejects_malformed_cobs_never_raises():
    """A malformed COBS frame (not just a bit-flip) is dropped -- returns
    None, does not raise -- mirroring Comms::decodeBinaryFrame()'s own
    'never propagate a partial/corrupt decode' contract."""
    assert decode_frame(b"\x00\x01\x02") is None
    assert decode_frame(b"") is None


def test_decode_frame_rejects_crc_corrupted_frame():
    """SUC-002 host-side acceptance: a bit-flipped, well-COBS-framed frame
    is detected via CRC and dropped, not mis-parsed as valid content."""
    payload = b"the quick brown fox jumps over the lazy dog"
    frame = bytearray(encode_frame(payload))

    # Flip one bit in a byte that is guaranteed not to equal the wire
    # delimiter (0x0A, 124-005 -- COBS is keyed on 0x0A now, not 0x00), so
    # the mutated frame is still well-formed COBS, isolating the
    # CRC-mismatch path from the malformed-COBS path already covered above.
    for i in range(len(frame)):
        candidate = frame[i] ^ 0x01
        if candidate != 0x0A:
            frame[i] = candidate
            break

    assert decode_frame(bytes(frame)) is None


def test_decode_frame_rejects_combined_length_under_two():
    """A COBS-valid frame whose decoded payload is under 2 bytes (too short
    to even hold a CRC) is rejected, not misread as a zero-length payload
    with a bogus CRC."""
    # delimiter=0x0A -- decode_frame() COBS-decodes at that delimiter
    # (124-005); a frame encoded at the primitive's own 0x00 default would
    # fail to decode for the WRONG reason (delimiter mismatch, not the
    # "combined length under 2" case this test targets).
    frame = cobs_encode(b"\x01", delimiter=0x0A)  # decodes to exactly 1 byte -- no room for a CRC
    assert decode_frame(frame) is None


# ---------------------------------------------------------------------------
# CRC-scope extension (124-003/124-005, issue §3): encode_frame()/
# decode_frame() take a ``command: bytes = b""`` argument -- the CRC's input
# extends to cover ``command + b":" + payload`` instead of ``payload``
# alone. Empty (the default) means no scope extension, byte-identical to
# protocol v4's CRC.
# ---------------------------------------------------------------------------


def test_encode_frame_default_command_matches_empty_command():
    """Omitting ``command`` is identical to passing ``b""`` explicitly --
    both mean no CRC-scope extension."""
    payload = b"the quick brown fox"
    assert encode_frame(payload) == encode_frame(payload, command=b"")


def test_frame_roundtrip_with_matching_command_scope():
    # encode_frame()/decode_frame() COBS(delimiter=0x0A) at this layer
    # (124-005, issue §2) -- a literal 0x0A in the payload is exactly what
    # this delimiter choice must survive round-tripping (COBS removes every
    # occurrence from the wire bytes and restores it on decode); a literal
    # 0x00 is now ordinary content, unlike pre-124.
    payload = b"MoveTwist payload bytes \x00\x0a\xff"
    frame = encode_frame(payload, command=b"MOVE")
    assert 0x0A not in frame
    assert decode_frame(frame, command=b"MOVE") == payload


def test_decode_frame_rejects_mismatched_command_scope():
    """A frame CRC-scoped under one command name fails verification when
    decoded under a DIFFERENT one -- the 'command byte mutated in transit'
    acceptance criterion (issue §3 / ticket 003 AC)."""
    payload = b"identical payload bytes"
    frame = encode_frame(payload, command=b"MOVE")

    assert decode_frame(frame, command=b"MOVE") == payload  # control: matching scope succeeds
    assert decode_frame(frame, command=b"STOP") is None  # different command name -> CRC mismatch
    assert decode_frame(frame) is None  # no command at all (empty scope) -> also a CRC mismatch


def test_encode_frame_two_different_commands_produce_two_different_crcs():
    """Identical payload under two different command names produces two
    different CRCs (issue §3 / ticket 003 AC) -- proven by showing the
    schema payload bytes are identical between the two frames while the
    trailing 2-byte CRC differs, isolating the difference to the CRC
    exactly."""
    payload = b"identical payload bytes"
    frame_move = encode_frame(payload, command=b"MOVE")
    frame_stop = encode_frame(payload, command=b"STOP")

    assert frame_move != frame_stop

    combined_move = cobs_decode(frame_move, delimiter=0x0A)
    combined_stop = cobs_decode(frame_stop, delimiter=0x0A)
    assert combined_move[:-2] == combined_stop[:-2] == payload  # same payload, both directions
    assert combined_move[-2:] != combined_stop[-2:]  # different CRC


# ---------------------------------------------------------------------------
# ByteStreamDemuxer (124-005: collapsed to a plain split-on-'\n' -- protocol
# v5's uniform grammar makes '\n' an UNCONDITIONAL terminator in both
# directions, since COBS is keyed on 0x0A now (wire_runtime.h item 8) and a
# binary line's own bytes can never contain a literal 0x0A. No more
# per-entry ``(kind, payload)`` tuples -- ``feed()`` returns a plain
# ``list[bytes]`` of raw lines; a caller classifies each one by parsing its
# own ``<COMMAND>`` prefix against the registry (this class has no opinion,
# see its own docstring). ``_looks_like_text()`` -- the printable-ASCII
# content-sniffing heuristic this collapse deletes -- is gone; its own
# former tests (relay-comment/DEVICE:/PONG: "dynamic content" recognition)
# are deleted too, not ported: there is nothing left to recognize once every
# line is delivered the same way.
# ---------------------------------------------------------------------------


def test_demuxer_splits_multiple_lines_in_one_feed():
    frame1 = encode_frame(b"first payload")
    frame2 = encode_frame(b"second payload")
    stream = frame1 + b"\n" + b"HELLO\n" + frame2 + b"\n" + b"PONG:t=5\n"

    demux = ByteStreamDemuxer()
    results = demux.feed(stream)

    assert results == [frame1, b"HELLO", frame2, b"PONG:t=5"]


def test_demuxer_does_not_strip_trailing_cr():
    """124-005, issue §7: '\\r' is legal binary (or cleartext-with-data)
    content under the uniform grammar -- ONLY a caller that has already
    classified a line as a no-data cleartext verb strips it (see
    ``serial_conn.py``'s ``_split_wire_line()``). The demuxer itself never
    inspects content at all any more, so a trailing '\\r' survives into the
    returned line unchanged (contrast the pre-124 behavior, which stripped
    it unconditionally for every text line)."""
    demux = ByteStreamDemuxer()
    assert demux.feed(b"PING\r\n") == [b"PING\r"]


def test_demuxer_handles_partial_feeds_across_calls():
    """A line split across two feed() calls (simulating a partial serial
    read) is not delivered until the terminator actually arrives."""
    frame = encode_frame(b"\x01payload")
    demux = ByteStreamDemuxer()

    assert demux.feed(frame[:3]) == []
    assert demux.feed(frame[3:]) == []  # still no delimiter yet
    assert demux.feed(b"\n") == [frame]


def test_demuxer_handles_multiple_frames_in_one_feed():
    frames = [encode_frame(bytes([i] * 5)) for i in range(1, 6)]
    stream = b"".join(f + b"\n" for f in frames)

    demux = ByteStreamDemuxer()
    results = demux.feed(stream)

    assert results == frames


def _find_0x0a_free_frame_with_a_zero_byte(label: bytes = b"payload") -> bytes:
    """Return an encoded frame containing at least one literal 0x00 byte
    (ordinary content now, 124-005) -- proves the demuxer correctly treats
    it as ordinary content, never mistaking it for the wire delimiter
    (which is '\\n'/0x0A now, not 0x00). Every ``encode_frame()`` output is
    ALREADY guaranteed 0x0A-free by construction (COBS's own delimiter-
    exclusion property), so no seed search is needed for that half; this
    just needs a payload virtually certain to also contain a literal 0x00
    somewhere (varint/protobuf-shaped small integers commonly do)."""
    for seed in range(200):
        candidate = encode_frame(bytes([0, seed]) + label)
        if 0x00 in candidate:
            return candidate
    pytest.fail("could not find a 0x00-containing encoded frame")  # pragma: no cover


# ---------------------------------------------------------------------------
# 123-006 fixed a 0x0A-in-binary-frame corruption bug under protocol v4's
# split-terminator demux; 124-005's uniform grammar removes the BUG CLASS
# entirely (COBS is keyed on 0x0A now, so a binary line can never contain
# one) rather than papering over it with a better heuristic. These tests
# confirm the new invariant: a literal 0x00 embedded in a binary line's
# COBS bytes is ordinary content and never terminates it early -- only '\n'
# does, unconditionally.
# ---------------------------------------------------------------------------


def test_demuxer_binary_frame_with_embedded_0x00_single_feed():
    frame = _find_0x0a_free_frame_with_a_zero_byte()
    demux = ByteStreamDemuxer()
    assert demux.feed(frame + b"\n") == [frame]


def test_demuxer_binary_frame_with_embedded_0x00_split_across_feeds():
    frame = _find_0x0a_free_frame_with_a_zero_byte()
    idx = frame.index(0x00)
    demux = ByteStreamDemuxer()

    # Split the feed so the 0x00 byte itself lands in the FIRST chunk --
    # under protocol v4 this WAS the delimiter and would have ended the
    # frame early; under v5 it is ordinary content and changes nothing.
    assert demux.feed(frame[: idx + 1]) == []
    assert demux.feed(frame[idx + 1 :]) == []  # still no '\n' delimiter yet
    assert demux.feed(b"\n") == [frame]


def test_demuxer_binary_frame_with_embedded_0x00_interleaved_with_hello_ping():
    frame = _find_0x0a_free_frame_with_a_zero_byte()
    stream = b"HELLO\n" + frame + b"\n" + b"PING\n"

    demux = ByteStreamDemuxer()
    results = demux.feed(stream)

    assert results == [b"HELLO", frame, b"PING"]


def test_demuxer_multiple_0x00_bytes_inside_one_binary_frame():
    """Not just one embedded 0x00 -- a frame with several must still be
    delivered whole.

    An output byte equals 0x00 exactly when the underlying (pre-XOR) COBS
    byte equals the delimiter (0x0A) -- see cobs_encode()'s own docstring's
    "Mechanism" paragraph. A payload of literal 0x0A bytes (non-zero, so
    COBS passes each through unchanged as ordinary data, never treating it
    as a block boundary the way a literal 0x00 payload byte would) is
    therefore GUARANTEED to produce several literal 0x00 bytes in the
    encoded output -- deterministic, no seed search needed."""
    frame = encode_frame(bytes([0x0A, 1, 0x0A, 2, 0x0A, 3]))
    assert 0x00 in frame
    assert 0x0A not in frame
    demux = ByteStreamDemuxer()
    assert demux.feed(frame + b"\n") == [frame]


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
