"""src/tests/unit/test_repl_confirm_ack_ring.py — ticket 128-001.

``RogoSession.confirm()`` used to read the deleted ``TLMFrame.ack`` scalar
slot (124-008 deleted it — ring membership in ``TLMFrame.acks`` is the only
"really acked" signal now): the first telemetry frame pumped during any
motion-issuing verb (``twist``/``stop``/``config``/``raw``) raised
``AttributeError``, which ``dispatch()`` never caught, killing the whole
``rogo repl`` process
(``repl-confirm-reads-deleted-ack-slot-scan-the-ack-ring.md``). This covers
the ring-scan rewrite directly against ``RogoSession.confirm()``, bypassing
``RogoSession.__init__`` (which opens a real serial connection) the same
way ``test_geofence_capture_fix_yaw_wrap.py`` bypasses ``Geofence.__init__``.

Covers:
  1. A frame with a populated ``acks`` ring containing the awaited
     ``corr_id`` — ``confirm()`` returns the matching ``AckEntry``.
  2. Non-matching ring entries (wrong ``corr_id``, or earlier frames) are
     skipped until the right one arrives.
  3. An empty ring times out to ``None`` instead of raising — the bug this
     ticket fixes would have raised ``AttributeError`` on the very first
     frame instead.
"""
from __future__ import annotations

from robot_radio.io.repl import RogoSession
from robot_radio.robot.protocol import AckEntry, TLMFrame


def _make_session(frame_batches: "list[list[TLMFrame]]") -> RogoSession:
    """Build a ``RogoSession`` without running ``__init__`` (which opens a
    real connection) — ``confirm()`` only calls ``self.pump()``, which is
    replaced here with a canned sequence of pre-drained frame batches, one
    per call (an empty list once exhausted, mirroring "nothing pending
    right now" rather than raising)."""
    session = object.__new__(RogoSession)
    session.recorder = None
    session._latest = None
    # New session fields grown by the 2026-08-05 rogo revival: the ack
    # backlog confirm() now scans first (acks drained by an EARLIER pump
    # stay findable), and the daemon's abort event.
    import collections
    import threading
    session._ack_backlog = collections.deque(maxlen=64)
    session.abort_event = threading.Event()
    batches = iter(frame_batches)

    def _pump() -> "list[TLMFrame]":
        frames = next(batches, [])
        for f in frames:
            session._latest = f
        return frames

    session.pump = _pump
    return session


def test_confirm_matches_corr_id_in_populated_ack_ring():
    frame = TLMFrame(acks=[AckEntry(corr_id=5, ok=True, err_code=0)])
    session = _make_session([[frame]])

    ack = session.confirm(5, timeout_ms=200)

    assert ack == AckEntry(corr_id=5, ok=True, err_code=0)


def test_confirm_skips_non_matching_entries_and_frames_then_matches():
    frame1 = TLMFrame(acks=[AckEntry(corr_id=1, ok=True, err_code=0)])
    frame2 = TLMFrame(acks=[
        AckEntry(corr_id=9, ok=True, err_code=0),
        AckEntry(corr_id=2, ok=False, err_code=3),
    ])
    session = _make_session([[frame1], [frame2]])

    ack = session.confirm(2, timeout_ms=200)

    assert ack == AckEntry(corr_id=2, ok=False, err_code=3)


def test_confirm_times_out_to_none_on_empty_ring_never_raises():
    """The bug this fixes: the old confirm() read f.ack (deleted by
    124-008) and raised AttributeError on the very first frame. An empty
    ring (or one with no matching corr_id) must time out to None instead."""
    frame = TLMFrame(acks=[])
    session = _make_session([[frame], [frame], [frame]])

    ack = session.confirm(99, timeout_ms=50)

    assert ack is None
