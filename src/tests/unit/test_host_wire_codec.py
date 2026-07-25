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
    lists, not just "some encoding happened."""

    def wire_frame(payload: bytes) -> bytes:
        crc = crc16_ccitt_false(payload)
        combined = payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))
        return cobs_encode(combined, delimiter=0x0A)

    cases = {
        bytes([0x0A] * 8): bytes.fromhex("01 00 00 00 00 00 00 00 00 41 78"),
        bytes(8): bytes.fromhex("0b 0b 0b 0b 0b 0b 0b 0b 09 34 3b"),
        bytes(range(0x10)): bytes.fromhex(
            "0b 18 0b 08 09 0e 0f 0c 0d 02 03 00 01 06 07 04 05 3d 31"
        ),
    }

    for payload, expected_wire in cases.items():
        wire = wire_frame(payload)
        assert wire == expected_wire, f"payload={payload.hex()}: wire mismatch"
        assert 0x0A not in wire, f"payload={payload.hex()}: wire contains a literal 0x0A"

        combined = cobs_decode(wire, delimiter=0x0A)
        assert combined[:-2] == payload, f"payload={payload.hex()}: round-trip mismatch"


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
        assert 0 not in frame
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

    # Flip one bit in a byte that is guaranteed non-zero (so the mutated
    # frame is still well-formed COBS, isolating the CRC-mismatch path from
    # the malformed-COBS path already covered above).
    for i in range(len(frame)):
        candidate = frame[i] ^ 0x01
        if candidate != 0:
            frame[i] = candidate
            break

    assert decode_frame(bytes(frame)) is None


def test_decode_frame_rejects_combined_length_under_two():
    """A COBS-valid frame whose decoded payload is under 2 bytes (too short
    to even hold a CRC) is rejected, not misread as a zero-length payload
    with a bogus CRC."""
    frame = cobs_encode(b"\x01")  # decodes to exactly 1 byte -- no room for a CRC
    assert decode_frame(frame) is None


# ---------------------------------------------------------------------------
# CRC-scope extension (124-003, issue §3): encode_frame()/decode_frame()
# gained a ``command: bytes = b""`` argument -- the CRC's input extends to
# cover ``command + b":" + payload`` instead of ``payload`` alone. Empty
# (the default) means no scope extension, byte-identical to protocol v4's
# CRC -- every CURRENT caller still gets that, since no wire ASCII command
# prefix exists yet (ticket 124-005's own grammar cutover).
# ---------------------------------------------------------------------------


def test_encode_frame_default_command_matches_pre_124_behavior():
    """Omitting ``command`` reproduces the exact pre-124 frame bytes --
    every existing call site (serial_conn.py, sim_loop.py, ...) keeps
    computing byte-identical frames."""
    payload = b"the quick brown fox"
    assert encode_frame(payload) == encode_frame(payload, command=b"")


def test_frame_roundtrip_with_matching_command_scope():
    # encode_frame()/decode_frame() still COBS(delimiter=0x00) at this layer
    # (124-003 does not thread the delimiter through the higher-level frame
    # API, only cobs_encode()/cobs_decode() themselves -- see this module's
    # header) -- only 0x00 bytes are special here, so a literal 0x0A in the
    # payload is ordinary content and survives untouched, same as pre-124.
    payload = b"MoveTwist payload bytes \x00\x0a\xff"
    frame = encode_frame(payload, command=b"MOVE")
    assert 0 not in frame
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

    combined_move = cobs_decode(frame_move)
    combined_stop = cobs_decode(frame_stop)
    assert combined_move[:-2] == combined_stop[:-2] == payload  # same payload, both directions
    assert combined_move[-2:] != combined_stop[-2:]  # different CRC


# ---------------------------------------------------------------------------
# ByteStreamDemuxer
# ---------------------------------------------------------------------------


def test_demuxer_splits_interleaved_text_and_binary():
    frame1 = encode_frame(b"first payload")
    frame2 = encode_frame(b"second payload")
    stream = frame1 + b"\x00" + b"HELLO\r\n" + frame2 + b"\x00" + b"OK pong t=5\n"

    demux = ByteStreamDemuxer()
    results = demux.feed(stream)

    assert results == [
        ("binary", frame1),
        ("text", b"HELLO"),
        ("binary", frame2),
        ("text", b"OK pong t=5"),
    ]


def test_demuxer_strips_trailing_cr_from_text_lines():
    demux = ByteStreamDemuxer()
    results = demux.feed(b"PING\r\n")
    assert results == [("text", b"PING")]


def test_demuxer_handles_partial_feeds_across_calls():
    """A frame/line split across two feed() calls (simulating a partial
    serial read) is not delivered until the terminator actually arrives."""
    # Pick a payload whose encoded frame happens to contain no 0x0A byte --
    # 123-006 fixed the demux so an embedded 0x0A no longer splits/corrupts a
    # binary frame (see test_demuxer_binary_frame_with_embedded_0x0a_* below),
    # but this test targets partial-feed buffering specifically, so it still
    # picks a 0x0A-free frame to keep that behavior isolated from the
    # 0x0A-discrimination behavior covered separately.
    for seed in range(20):
        candidate = encode_frame(bytes([seed]) + b"payload")
        if 0x0A not in candidate:
            frame = candidate
            break
    else:  # pragma: no cover -- astronomically unlikely
        pytest.fail("could not find a 0x0A-free encoded frame for this test")
    demux = ByteStreamDemuxer()

    assert demux.feed(frame[:3]) == []
    assert demux.feed(frame[3:]) == []  # still no delimiter yet
    assert demux.feed(b"\x00") == [("binary", frame)]


def test_demuxer_handles_multiple_frames_in_one_feed():
    frames = [encode_frame(bytes([i] * 5)) for i in range(1, 6)]
    stream = b"".join(f + b"\x00" for f in frames)

    demux = ByteStreamDemuxer()
    results = demux.feed(stream)

    assert results == [("binary", f) for f in frames]


def _find_0x0a_frame(label: bytes = b"payload") -> bytes:
    """Return an encoded frame that embeds at least one literal 0x0A byte
    -- COBS only guarantees 0x00-freedom, never 0x0A-freedom, so any long
    enough envelope has a real chance of containing one (this is the exact
    class of frame that corrupted the bench's move_wheels command, proven
    0/10 on hardware before 123-006's fix).

    Also includes a literal ``0x01`` byte in the payload -- COBS never
    rewrites a non-zero data byte, so it survives into the encoded frame
    unchanged, GUARANTEEING (not just "very likely", regardless of which
    ``seed`` also happens to produce the 0x0A) that the frame contains a
    byte outside ``_looks_like_text()``'s printable-ASCII alphabet. This
    makes every test using this helper deterministic: the demuxer must
    always classify the frame as binary, never text."""
    for seed in range(200):
        candidate = encode_frame(b"\x01" + bytes([seed]) + label)
        if 0x0A in candidate:
            return candidate
    pytest.fail("could not find a 0x0A-containing encoded frame")  # pragma: no cover


# ---------------------------------------------------------------------------
# 123-006 regression: 0x0A embedded inside a binary frame must NOT terminate
# it early -- only 0x00 ends a binary frame; 0x0A ends a TEXT line only when
# what precedes it "looks like text" (_looks_like_text() -- printable ASCII).
# ---------------------------------------------------------------------------


def test_demuxer_binary_frame_with_embedded_0x0a_single_feed():
    frame = _find_0x0a_frame()
    demux = ByteStreamDemuxer()
    assert demux.feed(frame + b"\x00") == [("binary", frame)]


def test_demuxer_binary_frame_with_embedded_0x0a_split_across_feeds():
    frame = _find_0x0a_frame()
    idx = frame.index(0x0A)
    demux = ByteStreamDemuxer()

    # Split the feed so the 0x0A byte itself lands in the FIRST chunk --
    # the old "whichever terminator comes first" logic would have emitted a
    # (bogus) text line right there, before the frame's own 0x00 ever
    # arrives.
    assert demux.feed(frame[: idx + 1]) == []
    assert demux.feed(frame[idx + 1 :]) == []  # still no 0x00 delimiter yet
    assert demux.feed(b"\x00") == [("binary", frame)]


def test_demuxer_binary_frame_with_embedded_0x0a_interleaved_with_hello_ping():
    frame = _find_0x0a_frame()
    stream = b"HELLO\r\n" + frame + b"\x00" + b"PING\n"

    demux = ByteStreamDemuxer()
    results = demux.feed(stream)

    assert results == [
        ("text", b"HELLO"),
        ("binary", frame),
        ("text", b"PING"),
    ]


def test_demuxer_multiple_0x0a_bytes_inside_one_binary_frame():
    """Not just one embedded 0x0A -- a frame with several must still be
    delivered whole."""
    frame = encode_frame(b"\x0a\x01\x0a\x02\x0a\x03")
    assert 0x0A in frame
    demux = ByteStreamDemuxer()
    assert demux.feed(frame + b"\x00") == [("binary", frame)]


def test_demuxer_recognizes_relay_comment_lines_with_no_0x00_ever():
    """RadioRelay's own pre-``!GO`` command-plane responses
    (``serial_conn.py``'s ``_relay_handshake()``) are ``#``-prefixed text
    lines on a wire segment that carries NO ``0x00`` byte at all -- these
    must still be delivered as text immediately, not held forever waiting
    for a ``0x00`` delimiter that will never arrive. This is exactly why
    123-006's fix uses a content-shape recognizer (``_looks_like_text()``)
    on the host side rather than mirroring firmware's closed HELLO/PING
    literal-match list, which would never recognize this free-form text."""
    demux = ByteStreamDemuxer()
    assert demux.feed(b"# channel: 0\r\n") == [("text", b"# channel: 0")]
    assert demux.feed(b"# entering data plane\n") == [
        ("text", b"# entering data plane")
    ]


def test_demuxer_recognizes_device_banner_and_pong_reply_dynamic_content():
    """The firmware's two real text-plane replies carry DYNAMIC content
    (device name/serial, a live timestamp) -- not a fixed literal string --
    so an exact-match recognizer (mirroring firmware's own inbound
    HELLO/PING check) could never recognize them. Confirms
    ``_looks_like_text()`` handles both real shapes."""
    demux = ByteStreamDemuxer()
    assert demux.feed(b"DEVICE:NEZHA2:robot:my-bot:1234\r\n") == [
        ("text", b"DEVICE:NEZHA2:robot:my-bot:1234")
    ]
    assert demux.feed(b"OK pong t=987654\n") == [("text", b"OK pong t=987654")]


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
