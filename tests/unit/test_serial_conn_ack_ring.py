"""tests/unit/test_serial_conn_ack_ring.py — 104-003 (serial_conn ack-ring
matcher hardening + TelemetrySecondary consumption).

Sprint 103's ack-ring matcher (the poll/match/timeout loop behind
``wait_for_ack()``) lived inline in ``NezhaProtocol`` -- a minimal slice
serving exactly the two-then-three callers that send a P4 ``CommandEnvelope``
oneof arm with no synchronous reply (``twist``/``stop``/``config``). This
ticket promotes that algorithm to ``SerialConnection.wait_for_ack()`` so
every future caller — not just ``NezhaProtocol`` — gets the identical
matching guarantee without duplicating it (``NezhaProtocol.wait_for_ack()``
is now a thin adapter; see ``tests/unit/test_twist_stop_ack_matcher.py``'s
own updated section 3 for that delegation's own coverage).

This file covers the PROMOTED algorithm itself, directly against
``SerialConnection`` (no ``NezhaProtocol`` involved), with synthetic
``pb2.ReplyEnvelope{tlm: Telemetry{acks: [...]}}`` frames -- no live
hardware, no real serial port (mirrors ``test_serial_conn_binary_plane.py``'s
own ``_new_conn()`` no-I/O construction pattern):

1. ``_match_ack_in_frames()`` — the pure-function matching core, exercised
   directly against hand-built frame batches (exact match, first-match-wins
   ordering, non-``tlm`` frames ignored, empty rings ignored).
2. ``SerialConnection.wait_for_ack()`` — the full poll/match/timeout loop,
   covering this ticket's four required scenarios: exact ``corr_id`` match,
   tolerated ring re-delivery (the SAME ``corr_id`` riding more than one
   frame is not an error), ring-wrap (an older, un-observed ``corr_id``
   evicted from the depth-3 ring before this method ever sees a frame
   carrying it — a real, bounded failure, not a bug), and a bounded timeout
   (never an infinite wait).

Collected under ``tests/unit/`` — ``pyproject.toml``'s ``testpaths`` includes
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


def _frame_with_acks(acks: list[tuple[int, bool, int]], corr_id: int = 0
                     ) -> "envelope_pb2.ReplyEnvelope":
    """Build a synthetic ``tlm``-body ``ReplyEnvelope`` carrying a scripted
    ack-ring -- (corr_id, ok, err_code) triples -- the same wire shape
    ``_handle_binary_reply()`` queues into ``_binary_tlm_queue``."""
    reply = envelope_pb2.ReplyEnvelope(corr_id=corr_id)
    for ack_corr_id, ok, err_code in acks:
        entry = reply.tlm.acks.add()
        entry.corr_id = ack_corr_id
        entry.status = (telemetry_pb2.ACK_STATUS_OK if ok
                         else telemetry_pb2.ACK_STATUS_ERR)
        entry.err_code = err_code
    return reply


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
# 1. _match_ack_in_frames() -- the pure matching core
# ---------------------------------------------------------------------------


def test_match_ack_in_frames_exact_match():
    frames = [_frame_with_acks([(5, True, 0)])]

    ack = _match_ack_in_frames(frames, 5)

    assert ack is not None
    assert ack.corr_id == 5
    assert ack.status == telemetry_pb2.ACK_STATUS_OK
    assert ack.err_code == 0


def test_match_ack_in_frames_no_match_returns_none():
    frames = [_frame_with_acks([(1, True, 0), (2, True, 0)])]

    assert _match_ack_in_frames(frames, 5) is None


def test_match_ack_in_frames_returns_first_matching_frame_in_list_order():
    frames = [
        _frame_with_acks([(1, True, 0)]),
        _frame_with_acks([(5, False, envelope_pb2.ERR_BADARG)]),
        _frame_with_acks([(5, True, 0)]),  # a later, differing entry for the
                                            # SAME corr_id -- must be ignored
                                            # once the first match is found.
    ]

    ack = _match_ack_in_frames(frames, 5)

    assert ack.corr_id == 5
    assert ack.status == telemetry_pb2.ACK_STATUS_ERR
    assert ack.err_code == envelope_pb2.ERR_BADARG


def test_match_ack_in_frames_ignores_non_tlm_frames():
    ok_reply = envelope_pb2.ReplyEnvelope(corr_id=9)
    ok_reply.ok.q = 1  # an "ok" body, not "tlm" -- must be skipped, not raise

    frames = [ok_reply, _frame_with_acks([(5, True, 0)])]

    ack = _match_ack_in_frames(frames, 5)

    assert ack is not None
    assert ack.corr_id == 5


def test_match_ack_in_frames_ignores_empty_rings():
    frames = [_frame_with_acks([]), _frame_with_acks([(5, True, 0)])]

    ack = _match_ack_in_frames(frames, 5)

    assert ack is not None


def test_match_ack_in_frames_empty_batch_returns_none():
    assert _match_ack_in_frames([], 5) is None


# ---------------------------------------------------------------------------
# 2. SerialConnection.wait_for_ack() -- exact corr_id match
# ---------------------------------------------------------------------------


def test_wait_for_ack_matches_exact_corr_id_queued_directly():
    """Realistic path: a frame lands in _binary_tlm_queue the same way
    _handle_binary_reply() puts it there (not a monkeypatched drain)."""
    conn = _new_conn()
    conn._binary_tlm_queue.put_nowait(_frame_with_acks([(5, True, 0)]))

    ack = conn.wait_for_ack(5, timeout=200)

    assert ack is not None
    assert ack.corr_id == 5
    assert ack.status == telemetry_pb2.ACK_STATUS_OK
    assert ack.err_code == 0


def test_wait_for_ack_matches_exact_corr_id_err():
    conn = _new_conn()
    conn._binary_tlm_queue.put_nowait(
        _frame_with_acks([(9, False, envelope_pb2.ERR_RANGE)]))

    ack = conn.wait_for_ack(9, timeout=200)

    assert ack.status == telemetry_pb2.ACK_STATUS_ERR
    assert ack.err_code == envelope_pb2.ERR_RANGE


def test_wait_for_ack_skips_non_matching_frames_then_matches_on_a_later_poll():
    conn = _new_conn()
    conn.drain_binary_tlm = _scripted_drain([
        [_frame_with_acks([(1, True, 0)]), _frame_with_acks([(2, True, 0)])],
        [_frame_with_acks([])],
        [_frame_with_acks([(5, False, envelope_pb2.ERR_BADARG)])],
    ])

    ack = conn.wait_for_ack(5, timeout=500)

    assert ack.corr_id == 5
    assert ack.status == telemetry_pb2.ACK_STATUS_ERR
    assert ack.err_code == envelope_pb2.ERR_BADARG


# ---------------------------------------------------------------------------
# 3. SerialConnection.wait_for_ack() -- tolerated ring re-delivery
# ---------------------------------------------------------------------------


def test_wait_for_ack_tolerates_the_same_corr_id_riding_multiple_frames_in_one_drain():
    """Ring re-delivery: depth-3 acks legitimately repeat the same corr_id
    across consecutive Telemetry pushes. A single non-blocking drain can
    return several such frames at once -- this must not raise or behave any
    differently than a single match (103-009's own documented contract)."""
    conn = _new_conn()
    for _ in range(3):
        conn._binary_tlm_queue.put_nowait(_frame_with_acks([(5, True, 0)]))

    ack = conn.wait_for_ack(5, timeout=200)

    assert ack is not None
    assert ack.corr_id == 5
    assert ack.status == telemetry_pb2.ACK_STATUS_OK


def test_wait_for_ack_tolerates_redelivery_across_separate_polls():
    conn = _new_conn()
    conn.drain_binary_tlm = _scripted_drain([
        [_frame_with_acks([(5, True, 0)])],
        [_frame_with_acks([(5, True, 0)])],  # re-delivered copy on the next poll
    ])

    ack = conn.wait_for_ack(5, timeout=500)

    assert ack is not None
    assert ack.corr_id == 5


# ---------------------------------------------------------------------------
# 4. SerialConnection.wait_for_ack() -- ring-wrap (evicted corr_id)
# ---------------------------------------------------------------------------


def test_wait_for_ack_ring_wrap_evicted_corr_id_times_out():
    """Ring-wrap: an OLDER corr_id evicted from the depth-3 ring before this
    method ever observes a frame carrying it is a real, bounded failure per
    103 Decision 2, not a bug -- the host never sees it and this method
    times out exactly like an unacked/never-sent corr_id.  Simulated here by
    scripting every polled batch to carry OTHER (unrelated) corr_ids, never
    the one awaited -- indistinguishable, from wait_for_ack()'s perspective,
    from the target corr_id having been evicted before the host polled."""
    conn = _new_conn()
    conn.drain_binary_tlm = _scripted_drain([
        [_frame_with_acks([(1, True, 0)])],
        [_frame_with_acks([(2, True, 0)])],
        [_frame_with_acks([(3, True, 0)])],
    ])

    ack = conn.wait_for_ack(99, timeout=50)

    assert ack is None


# ---------------------------------------------------------------------------
# 5. SerialConnection.wait_for_ack() -- bounded timeout, never infinite
# ---------------------------------------------------------------------------


def test_wait_for_ack_returns_none_on_timeout_with_empty_queue():
    conn = _new_conn()  # _binary_tlm_queue stays empty -- nothing ever arrives

    start = time.monotonic()
    ack = conn.wait_for_ack(5, timeout=50)
    elapsed = time.monotonic() - start

    assert ack is None
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
