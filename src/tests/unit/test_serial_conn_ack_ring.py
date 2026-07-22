"""src/tests/unit/test_serial_conn_ack_ring.py — 104-003 (serial_conn ack
matcher hardening + TelemetrySecondary consumption), rewritten for the
single-ack-slot design (115-003 frame v2).

Sprint 103's ack matcher (the poll/match/timeout loop behind
``wait_for_ack()``) lived inline in ``NezhaProtocol`` -- a minimal slice
serving exactly the two-then-three callers that send a P4 ``CommandEnvelope``
oneof arm with no synchronous reply (``twist``/``stop``/``config``). Ticket
104-003 promoted that algorithm to ``SerialConnection.wait_for_ack()`` so
every future caller — not just ``NezhaProtocol`` — gets the identical
matching guarantee without duplicating it (``NezhaProtocol.wait_for_ack()``
is now a thin adapter; see ``src/tests/unit/test_twist_stop_ack_matcher.py``'s
own updated section 3 for that delegation's own coverage).

115-003 (gut-to-minimal-firmware S1) replaced the depth-3 ``AckEntry`` ring
this file originally scanned with a single ``ack_corr``/``ack_err`` slot,
valid iff ``flags`` bit 5 (``ack_fresh``) -- there is no wire ``AckEntry``
message any more. This file covers the PROMOTED algorithm itself, directly
against ``SerialConnection`` (no ``NezhaProtocol`` involved), with synthetic
``pb2.ReplyEnvelope{tlm: Telemetry{flags: ..., ack_corr: ..., ack_err: ...}}``
frames -- no live hardware, no real serial port (mirrors
``test_serial_conn_binary_plane.py``'s own ``_new_conn()`` no-I/O
construction pattern):

1. ``_match_ack_in_frames()`` — the pure-function matching core, exercised
   directly against hand-built frame batches (exact match, first-match-wins
   ordering, non-``tlm`` frames ignored, non-fresh acks ignored).
2. ``SerialConnection.wait_for_ack()`` — the full poll/match/timeout loop,
   covering this design's required scenarios: exact ``corr_id`` match,
   slot-overwrite (an older, un-observed ``corr_id`` overwritten by a later
   command's ack before this method ever sees a frame carrying it — a real,
   bounded failure, not a bug — the "ack-depth-1 tradeoff",
   stakeholder-accepted), and a bounded timeout (never an infinite wait).

Collected under ``src/tests/unit/`` — ``pyproject.toml``'s ``testpaths`` includes
``tests/unit``, so ``uv run python -m pytest`` collects it by default.
"""

from __future__ import annotations

import time

from robot_radio.io.serial_conn import SerialConnection, _match_ack_in_frames
from robot_radio.robot.pb2 import envelope_pb2, telemetry_pb2

# flags bit 5 -- telemetry.proto Telemetry.flags (ack_fresh). Mirrors
# serial_conn.py's own private _ACK_FRESH_BIT constant.
_ACK_FRESH_BIT = 1 << 5


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _new_conn() -> SerialConnection:
    """A SerialConnection with no real I/O performed (mirrors
    test_serial_conn_binary_plane.py's own ``_new_conn()``) -- ``_ser`` stays
    ``None``; ``wait_for_ack()`` only ever touches ``_binary_tlm_queue`` via
    ``drain_binary_tlm()``, never ``_ser``."""
    return SerialConnection()


def _frame_with_ack(corr_id: int, ok: bool, err_code: int = 0, *, fresh: bool = True,
                    ) -> "envelope_pb2.ReplyEnvelope":
    """Build a synthetic ``tlm``-body ``ReplyEnvelope`` carrying the single
    ack slot -- the same wire shape ``_handle_binary_reply()`` queues into
    ``_binary_tlm_queue``. ``fresh=False`` builds a frame whose ``flags``
    does NOT set bit 5 (``ack_fresh``) -- a normal telemetry push with no
    new ack this cycle, even though ``ack_corr``/``ack_err`` still carry
    their last-written (stale) values, matching real firmware behavior."""
    reply = envelope_pb2.ReplyEnvelope(corr_id=0)
    reply.tlm.flags = _ACK_FRESH_BIT if fresh else 0
    reply.tlm.ack_corr = corr_id
    reply.tlm.ack_err = 0 if ok else err_code
    return reply


# ---------------------------------------------------------------------------
# 1. _match_ack_in_frames() -- the pure matching core
# ---------------------------------------------------------------------------


def test_match_ack_in_frames_exact_match():
    frames = [_frame_with_ack(5, True)]

    telemetry = _match_ack_in_frames(frames, 5)

    assert telemetry is not None
    assert telemetry.ack_corr == 5
    assert telemetry.ack_err == 0


def test_match_ack_in_frames_no_match_returns_none():
    frames = [_frame_with_ack(1, True), _frame_with_ack(2, True)]

    assert _match_ack_in_frames(frames, 5) is None


def test_match_ack_in_frames_returns_first_matching_frame_in_list_order():
    frames = [
        _frame_with_ack(1, True),
        _frame_with_ack(5, False, envelope_pb2.ERR_BADARG),
        _frame_with_ack(5, True),  # a later, differing entry for the SAME
                                    # corr_id -- must be ignored once the
                                    # first match is found.
    ]

    telemetry = _match_ack_in_frames(frames, 5)

    assert telemetry.ack_corr == 5
    assert telemetry.ack_err == envelope_pb2.ERR_BADARG


def test_match_ack_in_frames_ignores_non_tlm_frames():
    ok_reply = envelope_pb2.ReplyEnvelope(corr_id=9)
    ok_reply.ok.q = 1  # an "ok" body, not "tlm" -- must be skipped, not raise

    frames = [ok_reply, _frame_with_ack(5, True)]

    telemetry = _match_ack_in_frames(frames, 5)

    assert telemetry is not None
    assert telemetry.ack_corr == 5


def test_match_ack_in_frames_ignores_non_fresh_acks():
    """A frame whose ``flags`` does not set ``ack_fresh`` (bit 5) must be
    skipped even when ``ack_corr`` happens to match -- ``ack_corr``/
    ``ack_err`` hold their last-written value on every ordinary telemetry
    push, not just the push where the ack was produced; only ``ack_fresh``
    says "this is a NEW ack this frame"."""
    frames = [_frame_with_ack(5, True, fresh=False), _frame_with_ack(5, True, fresh=True)]

    telemetry = _match_ack_in_frames(frames, 5)

    assert telemetry is not None
    # The FIRST fresh match wins -- the stale-flagged frame ahead of it in
    # list order is skipped entirely, not just deprioritized.
    assert telemetry is frames[1].tlm


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

    telemetry = conn.wait_for_ack(5, timeout=200)

    assert telemetry is not None
    assert telemetry.ack_corr == 5
    assert telemetry.ack_err == 0


def test_wait_for_ack_matches_exact_corr_id_err():
    conn = _new_conn()
    conn._binary_tlm_queue.put_nowait(
        _frame_with_ack(9, False, envelope_pb2.ERR_RANGE))

    telemetry = conn.wait_for_ack(9, timeout=200)

    assert telemetry.ack_err == envelope_pb2.ERR_RANGE


def test_wait_for_ack_skips_non_matching_frames_then_matches_on_a_later_poll():
    conn = _new_conn()
    conn.drain_binary_tlm = _scripted_drain([
        [_frame_with_ack(1, True), _frame_with_ack(2, True)],
        [_frame_with_ack(3, True, fresh=False)],
        [_frame_with_ack(5, False, envelope_pb2.ERR_BADARG)],
    ])

    telemetry = conn.wait_for_ack(5, timeout=500)

    assert telemetry.ack_corr == 5
    assert telemetry.ack_err == envelope_pb2.ERR_BADARG


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
# 3. SerialConnection.wait_for_ack() -- defensive: a fresh ack for the SAME
#    corr_id observed more than once in one drain does not crash or behave
#    differently than a single match (not normal firmware behavior under
#    the one-shot ack_fresh design, but the matcher must not choke on it).
# ---------------------------------------------------------------------------


def test_wait_for_ack_tolerates_the_same_corr_id_fresh_in_multiple_frames_in_one_drain():
    conn = _new_conn()
    for _ in range(3):
        conn._binary_tlm_queue.put_nowait(_frame_with_ack(5, True))

    telemetry = conn.wait_for_ack(5, timeout=200)

    assert telemetry is not None
    assert telemetry.ack_corr == 5
    assert telemetry.ack_err == 0


# ---------------------------------------------------------------------------
# 4. SerialConnection.wait_for_ack() -- slot-overwrite (the "ack-depth-1
#    tradeoff", stakeholder-accepted)
# ---------------------------------------------------------------------------


def test_wait_for_ack_slot_overwritten_by_other_commands_times_out():
    """Slot-overwrite: this corr_id's ack was produced but every polled
    frame's fresh ack belongs to a DIFFERENT, later command -- indistin-
    guishable, from wait_for_ack()'s perspective, from this corr_id never
    having been acked at all (115-003's single-slot design has no ring to
    fall back to). A real, bounded failure per the amendment issue's own
    "ack-depth-1 tradeoff" note (rare at bench rates; timeout+retry
    covers it), not a bug. Simulated here by scripting every polled batch
    to carry OTHER (unrelated) fresh corr_ids, never the one awaited."""
    conn = _new_conn()
    conn.drain_binary_tlm = _scripted_drain([
        [_frame_with_ack(1, True)],
        [_frame_with_ack(2, True)],
        [_frame_with_ack(3, True)],
    ])

    telemetry = conn.wait_for_ack(99, timeout=50)

    assert telemetry is None


# ---------------------------------------------------------------------------
# 5. SerialConnection.wait_for_ack() -- bounded timeout, never infinite
# ---------------------------------------------------------------------------


def test_wait_for_ack_returns_none_on_timeout_with_empty_queue():
    conn = _new_conn()  # _binary_tlm_queue stays empty -- nothing ever arrives

    start = time.monotonic()
    telemetry = conn.wait_for_ack(5, timeout=50)
    elapsed = time.monotonic() - start

    assert telemetry is None
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
