"""src/tests/unit/test_serial_conn_ack_ring.py — 104-003 (serial_conn ack
matcher hardening + TelemetrySecondary consumption), rewritten for the
bounded ack RING (120, bench-single-ack-slot-observability-collapses-at-
40ms.md).

Sprint 103's ack matcher (the poll/match/timeout loop behind
``wait_for_ack()``) lived inline in ``NezhaProtocol`` -- a minimal slice
serving exactly the two-then-three callers that send a P4 ``CommandEnvelope``
oneof arm with no synchronous reply (``move``/``stop``/``config``). Ticket
104-003 promoted that algorithm to ``SerialConnection.wait_for_ack()`` so
every future caller — not just ``NezhaProtocol`` — gets the identical
matching guarantee without duplicating it (``NezhaProtocol.wait_for_ack()``
is a thin adapter; see ``src/tests/unit/test_twist_stop_ack_matcher.py``'s
own section 3 for that delegation's own coverage).

115-003 (gut-to-minimal-firmware S1) replaced the pre-115 depth-3
``AckEntry`` ring with a single ``ack_corr``/``ack_err`` scalar slot, valid
iff ``flags`` bit 5 (``ack_fresh``). 120 brings a wire ``AckEntry`` message
back — a bounded, depth-4 ring (``Telemetry.acks``) — because bench
measurement at the real 40ms cycle / ~15Hz host read rate showed the single
slot lost 12/43 ``move_protocol_bench.py`` checks, every miss a transient
enqueue/STOP/CONFIG ack overwritten before the host's next read. This file
covers the RING-based matching algorithm, directly against
``SerialConnection`` (no ``NezhaProtocol`` involved), with synthetic
``pb2.ReplyEnvelope{tlm: Telemetry{acks: [AckEntry{...}, ...]}}`` frames --
no live hardware, no real serial port (mirrors
``test_serial_conn_binary_plane.py``'s own ``_new_conn()`` no-I/O
construction pattern):

1. ``_match_ack_in_frames()`` — the pure-function matching core, exercised
   directly against hand-built frame batches (exact match anywhere in a
   ring, first-match-wins ordering across frames AND across ring entries
   within one frame, non-``tlm`` frames ignored, an EMPTY ring never
   matches, no ``ack_fresh``-style freshness gate needed).
2. ``SerialConnection.wait_for_ack()`` — the full poll/match/timeout loop,
   covering this design's required scenarios: exact ``corr_id`` match
   (including a corr_id NOT the ring's freshest entry), ring saturation
   (more than depth-4 OTHER acks pushed before this one is ever read,
   evicting it -- the ring's own residual, narrower version of the old
   single-slot "ack-depth-1 tradeoff"), and a bounded timeout (never an
   infinite wait).

Collected under ``src/tests/unit/`` — ``pyproject.toml``'s ``testpaths`` includes
``tests/unit``, so ``uv run python -m pytest`` collects it by default.
"""

from __future__ import annotations

import time

from robot_radio.io.serial_conn import SerialConnection, _match_ack_in_frames
from robot_radio.robot.pb2 import envelope_pb2, telemetry_pb2

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _new_conn() -> SerialConnection:
    """A SerialConnection with no real I/O performed (mirrors
    test_serial_conn_binary_plane.py's own ``_new_conn()``) -- ``_ser`` stays
    ``None``; ``wait_for_ack()`` only ever touches ``_binary_tlm_queue`` via
    ``drain_binary_tlm()``, never ``_ser``."""
    return SerialConnection()


def _frame_with_ring(entries: list[tuple[int, int]]) -> "envelope_pb2.ReplyEnvelope":
    """Build a synthetic ``tlm``-body ``ReplyEnvelope`` carrying the bounded
    ack ring -- the same wire shape ``_handle_binary_reply()`` queues into
    ``_binary_tlm_queue``. ``entries`` is a list of ``(corr_id, err)`` pairs,
    in wire order (oldest-pushed first, matching ``Telemetry::ack()``'s own
    push/evict order)."""
    reply = envelope_pb2.ReplyEnvelope(corr_id=0)
    for corr_id, err in entries:
        reply.tlm.acks.add(corr_id=corr_id, err=err)
    return reply


def _frame_with_ack(corr_id: int, ok: bool, err_code: int = 0) -> "envelope_pb2.ReplyEnvelope":
    """Single-entry-ring convenience wrapper around ``_frame_with_ring()``."""
    return _frame_with_ring([(corr_id, 0 if ok else err_code)])


# ---------------------------------------------------------------------------
# 1. _match_ack_in_frames() -- the pure matching core
# ---------------------------------------------------------------------------


def test_match_ack_in_frames_exact_match():
    frames = [_frame_with_ack(5, True)]

    entry = _match_ack_in_frames(frames, 5)

    assert entry is not None
    assert entry.corr_id == 5
    assert entry.err == 0


def test_match_ack_in_frames_no_match_returns_none():
    frames = [_frame_with_ack(1, True), _frame_with_ack(2, True)]

    assert _match_ack_in_frames(frames, 5) is None


def test_match_ack_in_frames_matches_a_non_freshest_ring_entry():
    """A corr_id that is NOT the ring's last (newest) entry still matches --
    unlike the pre-120 single-slot design (which only ever exposed the
    freshest ack), the ring has no "freshest wins" concept; every entry is
    independently matchable."""
    frames = [_frame_with_ring([(1, 0), (2, 0), (3, 0)])]

    entry = _match_ack_in_frames(frames, 1)

    assert entry is not None
    assert entry.corr_id == 1
    assert entry.err == 0


def test_match_ack_in_frames_returns_first_matching_frame_in_list_order():
    frames = [
        _frame_with_ack(1, True),
        _frame_with_ring([(5, envelope_pb2.ERR_BADARG)]),
        _frame_with_ack(5, True),  # a later, differing entry for the SAME
                                    # corr_id -- must be ignored once the
                                    # first match is found.
    ]

    entry = _match_ack_in_frames(frames, 5)

    assert entry.corr_id == 5
    assert entry.err == envelope_pb2.ERR_BADARG


def test_match_ack_in_frames_scans_every_entry_within_one_frame():
    """A single frame's ring can carry up to kAckRingDepth (4) entries --
    every one of them is checked, not just the first or the last."""
    frames = [_frame_with_ring([(10, 0), (11, 0), (12, envelope_pb2.ERR_FULL), (13, 0)])]

    entry = _match_ack_in_frames(frames, 12)

    assert entry is not None
    assert entry.corr_id == 12
    assert entry.err == envelope_pb2.ERR_FULL


def test_match_ack_in_frames_ignores_non_tlm_frames():
    ok_reply = envelope_pb2.ReplyEnvelope(corr_id=9)
    ok_reply.ok.q = 1  # an "ok" body, not "tlm" -- must be skipped, not raise

    frames = [ok_reply, _frame_with_ack(5, True)]

    entry = _match_ack_in_frames(frames, 5)

    assert entry is not None
    assert entry.corr_id == 5


def test_match_ack_in_frames_empty_ring_never_matches():
    """A frame with NO ack-ring entries at all (the common case -- most
    telemetry frames carry no ack) never matches anything, including
    corr_id=0 -- an empty repeated field decodes to an empty list, not a
    phantom zero-valued entry."""
    reply = envelope_pb2.ReplyEnvelope(corr_id=0)
    reply.tlm.now = 5  # an ordinary frame, no acks pushed

    assert _match_ack_in_frames([reply], 0) is None


def test_match_ack_in_frames_empty_batch_returns_none():
    assert _match_ack_in_frames([], 5) is None


# ---------------------------------------------------------------------------
# 2. SerialConnection.wait_for_ack() -- exact corr_id match
# ---------------------------------------------------------------------------


def test_wait_for_ack_matches_exact_corr_id_queued_directly():
    """Realistic path: a frame lands in _binary_tlm_queue the same way
    _handle_binary_reply() puts it there (not a monkeypatched drain)."""
    conn = _new_conn()
    conn._binary_tlm_queue.put_nowait(_frame_with_ack(5, True))

    entry = conn.wait_for_ack(5, timeout=200)

    assert entry is not None
    assert entry.corr_id == 5
    assert entry.err == 0


def test_wait_for_ack_matches_exact_corr_id_err():
    conn = _new_conn()
    conn._binary_tlm_queue.put_nowait(
        _frame_with_ack(9, False, envelope_pb2.ERR_RANGE))

    entry = conn.wait_for_ack(9, timeout=200)

    assert entry.err == envelope_pb2.ERR_RANGE


def test_wait_for_ack_finds_a_rapid_fire_burst_all_in_one_frame():
    """The ring's own headline property: N (<= depth 4) rapid-fire acks
    landing in the SAME frame are all independently matchable -- the exact
    scenario the single-slot design lost (only the last of a same-period
    burst ever survived). A fresh connection/queue per corr_id, each
    seeded with the identical 4-entry ring frame, mirrors 4 separate
    real-world wait_for_ack() calls each polling the same telemetry
    stream for a different one of the 4 rapid-fire acks."""
    for corr_id in (1, 2, 3, 4):
        conn = _new_conn()
        conn._binary_tlm_queue.put_nowait(_frame_with_ring([(1, 0), (2, 0), (3, 0), (4, 0)]))
        entry = conn.wait_for_ack(corr_id, timeout=200)
        assert entry is not None and entry.corr_id == corr_id


def test_wait_for_ack_skips_non_matching_frames_then_matches_on_a_later_poll():
    conn = _new_conn()
    conn.drain_binary_tlm = _scripted_drain([
        [_frame_with_ack(1, True), _frame_with_ack(2, True)],
        [_frame_with_ring([])],  # an ordinary frame, empty ring
        [_frame_with_ack(5, False, envelope_pb2.ERR_BADARG)],
    ])

    entry = conn.wait_for_ack(5, timeout=500)

    assert entry.corr_id == 5
    assert entry.err == envelope_pb2.ERR_BADARG


def _scripted_drain(batches: list[list["envelope_pb2.ReplyEnvelope"]]):
    """Return a callable that yields each of ``batches`` in turn on
    successive calls, then ``[]`` forever after -- stands in for
    ``drain_binary_tlm()``'s own non-blocking drain, monkeypatched onto a
    ``SerialConnection`` instance so a test can script exactly what each
    poll iteration inside ``wait_for_ack()``'s loop observes."""
    it = iter(batches)

    def _drain() -> list["envelope_pb2.ReplyEnvelope"]:
        return next(it, [])

    return _drain


# ---------------------------------------------------------------------------
# 3. SerialConnection.wait_for_ack() -- defensive: the same corr_id appearing
#    in more than one drained frame (e.g. still in the ring on a LATER frame
#    too, since the ring persists until evicted) does not crash or behave
#    differently than a single match.
# ---------------------------------------------------------------------------


def test_wait_for_ack_tolerates_the_same_corr_id_present_in_multiple_frames():
    conn = _new_conn()
    for _ in range(3):
        conn._binary_tlm_queue.put_nowait(_frame_with_ack(5, True))

    entry = conn.wait_for_ack(5, timeout=200)

    assert entry is not None
    assert entry.corr_id == 5
    assert entry.err == 0


# ---------------------------------------------------------------------------
# 4. SerialConnection.wait_for_ack() -- ring saturation (the ring's own,
#    narrower residual version of the pre-120 single-slot "ack-depth-1
#    tradeoff" -- more than kAckRingDepth=4 OTHER acks pushed before this
#    one is ever read evicts it).
# ---------------------------------------------------------------------------


def test_wait_for_ack_evicted_past_ring_depth_times_out():
    """A corr_id whose ack was genuinely pushed, but every polled frame's
    ring is already full of 4 OTHER, later corr_ids -- indistinguishable,
    from wait_for_ack()'s perspective, from this corr_id never having been
    acked at all (the ring holds no memory of an evicted entry). A real,
    bounded failure (narrower than the single slot's own failure mode --
    this needs FOUR unrelated acks to collide, not just one), not a bug.
    Simulated here by scripting every polled batch to carry a full ring of
    OTHER (unrelated) corr_ids, never the one awaited."""
    conn = _new_conn()
    conn.drain_binary_tlm = _scripted_drain([
        [_frame_with_ring([(1, 0), (2, 0), (3, 0), (4, 0)])],
        [_frame_with_ring([(5, 0), (6, 0), (7, 0), (8, 0)])],
    ])

    entry = conn.wait_for_ack(99, timeout=50)

    assert entry is None


# ---------------------------------------------------------------------------
# 5. SerialConnection.wait_for_ack() -- bounded timeout, never infinite
# ---------------------------------------------------------------------------


def test_wait_for_ack_returns_none_on_timeout_with_empty_queue():
    conn = _new_conn()  # _binary_tlm_queue stays empty -- nothing ever arrives

    start = time.monotonic()
    entry = conn.wait_for_ack(5, timeout=50)
    elapsed = time.monotonic() - start

    assert entry is None
    # Bounded wait -- never blocks past (roughly) the requested timeout, and
    # never hangs indefinitely.
    assert elapsed < 1.0


def test_wait_for_ack_bounded_timeout_respects_requested_duration():
    conn = _new_conn()

    start = time.monotonic()
    conn.wait_for_ack(5, timeout=120)
    elapsed_ms = (time.monotonic() - start) * 1000.0

    # Generous upper bound (poll granularity + scheduling slack) -- proves
    # this is a BOUNDED wait scaled to the requested timeout, not a fixed
    # constant or an unbounded loop.
    assert 100 <= elapsed_ms < 1000


if __name__ == "__main__":
    import sys

    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
