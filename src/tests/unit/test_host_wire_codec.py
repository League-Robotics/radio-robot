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
    # COBS only guarantees 0x00-freedom, not 0x0A-freedom (see
    # ByteStreamDemuxer's own docstring: an embedded 0x0A INSIDE a frame is a
    # real, firmware-shared ambiguity this test must avoid to isolate the
    # "partial feed" behavior it actually targets).
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


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
