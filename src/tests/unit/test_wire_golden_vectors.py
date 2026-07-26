"""src/tests/unit/test_wire_golden_vectors.py -- sprint 124 ticket 004
(SUC-002, REQUIRED, non-negotiable): the host-side reader of THE shared
cross-language golden-vector fixture,
``src/tests/fixtures/wire_golden_vectors.txt``.

This is one of the stakeholder's two structural fixes for the defect class
that shipped in 123: a ``move_wheels`` envelope containing a literal 0x0A
byte was split and corrupted at the terminator because firmware's and
host's demuxers were two independent heuristics with no shared vector
forcing them to agree. The fixture this file reads is consumed
byte-for-byte-identically by the C++ side too
(``src/tests/sim/unit/wire_golden_vector_harness.cpp``, run via
``src/tests/sim/unit/test_wire_golden_vectors.py``'s compile+run wrapper --
note that is a DIFFERENT file at a different path, same basename by
convention with every other harness-wrapper pair in ``sim/unit/``). If a
future engineer edits the fixture to change one language's expected bytes,
BOTH suites read the new row and one of them will fail unless both codecs
agree on the new value -- that is the whole point: one fixture, two
readers, never two hand-maintained copies.

Composition each row's ``expected_wire_hex`` encodes (see the fixture
file's own header comment for the full rationale and provenance of each
row):

    crc      = crc16_ccitt_false(command + ":" + payload) if command else
               crc16_ccitt_false(payload)
    combined = payload + crc16_le(crc)
    expected_wire = cobs_encode(combined, delimiter)

Built on ``robot_radio.io.wire_codec``'s own primitives
(``crc16_init()``/``crc16_update()``/``cobs_encode()``/``cobs_decode()``) --
the exact same primitives ``_crc_over_scope()``/``encode_frame()`` compose,
at the delimiter=0x0A + command-scope composition ticket 124-005's framing
cutover will wire into ``encode_frame()`` itself (see the fixture header
for why this ticket exercises the primitives directly rather than
``encode_frame()``, which still defaults to delimiter=0x00/no scope as of
ticket 003).
"""

from __future__ import annotations

import pathlib

import pytest

from robot_radio.io.wire_codec import (
    cobs_decode,
    cobs_encode,
    crc16_init,
    crc16_update,
)

# src/tests/unit/test_wire_golden_vectors.py -> unit -> tests -> src -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_FIXTURE_PATH = _REPO_ROOT / "src" / "tests" / "fixtures" / "wire_golden_vectors.txt"

_COMMAND_SEPARATOR = b":"


class GoldenVector:
    """One parsed row of the shared fixture."""

    def __init__(self, name: str, delimiter: int, command: bytes, payload: bytes, expected_wire: bytes,
                 source: str) -> None:
        self.name = name
        self.delimiter = delimiter
        self.command = command
        self.payload = payload
        self.expected_wire = expected_wire
        self.source = source

    def __repr__(self) -> str:  # pragma: no cover -- pytest id/debug display only
        return f"GoldenVector({self.name!r})"


def load_golden_vectors(path: pathlib.Path = _FIXTURE_PATH) -> list[GoldenVector]:
    """Parse the shared pipe-delimited fixture -- see
    ``wire_golden_vectors.txt``'s own header comment for the exact column
    layout. '#'-prefixed and blank lines are ignored. The first
    non-comment, non-blank line is the column-header row and is skipped by
    name match, not just position, so an accidental reorder/duplicate
    header is caught rather than silently parsed as a data row."""
    assert path.is_file(), f"shared golden-vector fixture missing: {path}"
    vectors: list[GoldenVector] = []
    for lineno, raw_line in enumerate(path.read_text().splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split("|")
        assert len(fields) == 6, f"{path}:{lineno}: expected 6 pipe-delimited fields, got {len(fields)}: {line!r}"
        name, delimiter_hex, command, payload_hex, expected_wire_hex, source = fields
        if name == "name" and delimiter_hex == "delimiter_hex":
            continue  # the header row itself
        vectors.append(GoldenVector(
            name=name,
            delimiter=int(delimiter_hex, 16),
            command=command.encode("ascii"),
            payload=bytes.fromhex(payload_hex),
            expected_wire=bytes.fromhex(expected_wire_hex),
            source=source,
        ))
    assert vectors, f"{path}: no vectors parsed -- fixture empty or malformed"
    return vectors


_VECTORS = load_golden_vectors()


def _crc_over_scope(command: bytes, payload: bytes) -> int:
    """Same composition as ``wire_codec.py``'s own (private)
    ``_crc_over_scope()`` -- reimplemented at the call site here (rather
    than importing the underscore-prefixed internal) using ONLY the public
    ``crc16_init()``/``crc16_update()`` primitives, so this test exercises
    the same public surface the C++ side's harness is limited to (which has
    no access to ``wire_codec.py``'s Python-private helper at all)."""
    crc = crc16_init()
    if command:
        crc = crc16_update(crc, command)
        crc = crc16_update(crc, _COMMAND_SEPARATOR)
    return crc16_update(crc, payload)


@pytest.mark.parametrize("vector", _VECTORS, ids=[v.name for v in _VECTORS])
def test_golden_vector_encode_matches_shared_fixture(vector: GoldenVector):
    """Host codec's COBS(delimiter=0x0A) + CRC-scope composition reproduces
    the shared fixture's ``expected_wire_hex`` EXACTLY -- the fixture is
    ground truth (hand-verified from the issue, or an independently
    computed clean-room reference; see the fixture's own header), not
    something derived from this codec itself."""
    crc = _crc_over_scope(vector.command, vector.payload)
    combined = vector.payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))
    wire = cobs_encode(combined, delimiter=vector.delimiter)

    assert wire == vector.expected_wire, (
        f"{vector.name} ({vector.source}): host-computed wire bytes disagree with the shared fixture -- "
        f"got {wire.hex()}, fixture says {vector.expected_wire.hex()}"
    )
    assert vector.delimiter not in wire, f"{vector.name}: wire bytes contain a literal delimiter byte"


@pytest.mark.parametrize("vector", _VECTORS, ids=[v.name for v in _VECTORS])
def test_golden_vector_decode_round_trips(vector: GoldenVector):
    """The fixture's own ``expected_wire_hex`` decodes back to the original
    payload, and its CRC (scoped the same way encode composed it) verifies."""
    combined = cobs_decode(vector.expected_wire, delimiter=vector.delimiter)
    assert len(combined) >= 2, f"{vector.name}: decoded combined bytes too short to hold a CRC"
    payload, crc_bytes = combined[:-2], combined[-2:]
    assert payload == vector.payload, f"{vector.name}: decoded payload disagrees with the fixture"

    received_crc = crc_bytes[0] | (crc_bytes[1] << 8)
    expected_crc = _crc_over_scope(vector.command, vector.payload)
    assert received_crc == expected_crc, f"{vector.name}: CRC does not verify"


def test_crc_scope_vector_pair_proves_different_commands_differ():
    """SUC-002 / issue §3 acceptance: the identical payload under two
    different command names produces two different CRCs, and the SAME
    payload bytes are recovered under both -- isolating the difference to
    the CRC exactly. Both rows already live in the shared fixture
    (``crc_scope_move``/``crc_scope_stop_same_payload``); this test cross-
    checks them against EACH OTHER, not just against their own fixture row,
    which is what the raw per-row tests above already do."""
    by_name = {v.name: v for v in _VECTORS}
    move = by_name["crc_scope_move"]
    stop = by_name["crc_scope_stop_same_payload"]

    assert move.payload == stop.payload, "the pair must share the identical payload to isolate the CRC difference"
    assert move.command != stop.command
    assert move.expected_wire != stop.expected_wire

    combined_move = cobs_decode(move.expected_wire, delimiter=move.delimiter)
    combined_stop = cobs_decode(stop.expected_wire, delimiter=stop.delimiter)
    assert combined_move[:-2] == combined_stop[:-2] == move.payload, "same payload bytes recovered under both"
    assert combined_move[-2:] != combined_stop[-2:], "different CRC bytes"


def test_fixture_covers_every_required_vector_kind():
    """Guards the fixture itself against silent shrinkage -- every AC2/AC5
    vector kind this ticket requires must have at least one row: all-0x0A,
    all-0x00, a 0x00..0xFF-range sweep, an empty payload, and a CRC-scope
    pair. If a future edit deletes a row without replacing its kind, this
    fails loudly instead of the suite quietly losing coverage."""
    names = {v.name for v in _VECTORS}
    assert "all_0x0a" in names
    assert "all_0x00" in names
    assert any(v.name.startswith("sweep_0x00_0x0f") for v in _VECTORS)
    assert any(v.name.startswith("sweep_0x00_0xff") for v in _VECTORS)
    assert "empty_payload" in names
    assert "crc_scope_move" in names and "crc_scope_stop_same_payload" in names


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
