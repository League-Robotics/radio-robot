"""src/tests/unit/test_planner_tour.py -- 121-002 (tour-1-final-leg-completes-
only-on-stop.md): ``planner.tour``'s own completion-ack matching, re-created
after the pre-121-002 file of the same name (deleted somewhere along the
sprint-107/109/protocol-v4 history -- see ``git log`` if the prior contents
are ever needed).

Root cause (planning-time static analysis, confirmed by reproduction --
see ``tour.py``'s own file header and ``_drain_and_poll()``'s docstring):
``_drain_and_poll()``/``_outcome_for_terminal_frame()`` used to read ONLY
``TLMFrame.ack`` (the single "freshest ack" scalar slot, valid on exactly
ONE drained frame) even after sprint 120 added the bounded, depth-4 ack
RING (``TLMFrame.acks``) that ``wait_for_ack()`` (``io/serial_conn.py``)
already scans. On a lossy link, a `Move`'s own completion ack riding
exactly one scalar-slot frame is invisible forever if that ONE frame is
dropped -- even though the SAME ack kept riding the ring for several more
frames. This file covers the ring-aware fix directly:

1. ``_drain_and_poll()`` -- the pure matching core, exercised against
   hand-built ``TLMFrame`` batches (ring match, scalar-slot fallback for a
   frame whose ring carries no match, and the disambiguating case where a
   frame's scalar slot belongs to a DIFFERENT, unrelated corr_id while its
   own ring still carries the awaited one).
2. ``_outcome_for_terminal_frame()`` -- reads ``ok``/``err_code`` off the
   MATCHED entry (ring or scalar-slot fallback), never off the frame's own
   (possibly unrelated) scalar slot; ``fault_move_timeout`` stays
   frame-level.
3. ``run_tour()`` end to end, against a fake ``MoveTransport``
   (``_RingOnlyFakeTransport``) that models the EXACT lossy-link failure
   mode: every completion rides the ring ONLY -- ``TLMFrame.ack`` is
   `None` on every frame this fake ever returns, so a caller reading only
   the scalar slot (the pre-121-002 behavior) would time out and FAULT
   every leg, including (per the filed issue) the final one. This is the
   ticket's own required regression coverage: "the fresh-slot frame is
   dropped but the ring carries the ack."

Collected under ``src/tests/unit/`` -- ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by
default. No hardware, no sim ctypes lib, no serial port -- pure Python
against hand-built ``TLMFrame``/``AckEntry`` dataclasses and a fake
transport, mirroring ``test_serial_conn_ack_ring.py``'s own no-I/O
convention for the wire-level counterpart of this same ring.

Run with::

    uv run python -m pytest src/tests/unit/test_planner_tour.py -v
"""
from __future__ import annotations

import math
from types import SimpleNamespace

from robot_radio.planner.executor import RunOutcome
from robot_radio.planner.heading import HeadingCorrector
from robot_radio.planner.model import PlannerParams
from robot_radio.planner.tour import (
    TourLeg,
    _drain_and_poll,
    _outcome_for_terminal_frame,
    parse_tour,
    run_tour,
)
from robot_radio.robot.protocol import AckEntry, TLMFrame

_FLAG_FAULT_MOVE_TIMEOUT = 1 << 15


def _frame(*, ack: "AckEntry | None" = None, acks: "list[AckEntry] | None" = None,
           flags: int = 0) -> TLMFrame:
    """A minimal hand-built ``TLMFrame`` -- only the fields
    ``_drain_and_poll()``/``_outcome_for_terminal_frame()`` actually read."""
    return TLMFrame(flags=flags, ack=ack, acks=acks or [])


class _StubTransport:
    """The minimal slice of ``MoveTransport`` ``_drain_and_poll()`` itself
    needs -- just ``read_pending_binary_tlm_frames()`` -- for testing that
    one function directly, without a full fake ``move()``/``stop()``."""

    def __init__(self, frames: list[TLMFrame]) -> None:
        self._frames = frames

    def read_pending_binary_tlm_frames(self) -> list[TLMFrame]:
        frames, self._frames = self._frames, []
        return frames


# ---------------------------------------------------------------------------
# 1. _drain_and_poll() -- the pure matching core
# ---------------------------------------------------------------------------


def test_drain_and_poll_matches_ring_when_scalar_slot_is_absent():
    """The headline fix: a frame whose scalar slot was never fresh for
    move_id (``frame.ack is None``) still matches via its ``acks`` ring --
    the pre-121-002 code would have returned `None` here."""
    frame = _frame(ack=None, acks=[AckEntry(corr_id=7, ok=True, err_code=0)])
    transport = _StubTransport([frame])
    latest_frame = [None]

    terminal = _drain_and_poll(transport, 7, latest_frame)

    assert terminal is not None
    matched_frame, matched_ack = terminal
    assert matched_frame is frame
    assert matched_ack == AckEntry(corr_id=7, ok=True, err_code=0)
    assert latest_frame[0] is frame


def test_drain_and_poll_falls_back_to_scalar_slot_when_ring_has_no_match():
    """Backward compatibility: a frame with an EMPTY ring (e.g. a test
    double, or firmware/build that never populated it) still matches via
    the scalar "freshest ack" slot -- the ring is additive, not a
    replacement (docs/protocol-v4.md section 7.1)."""
    ack = AckEntry(corr_id=7, ok=True, err_code=0)
    frame = _frame(ack=ack, acks=[])
    transport = _StubTransport([frame])
    latest_frame = [None]

    terminal = _drain_and_poll(transport, 7, latest_frame)

    assert terminal is not None
    matched_frame, matched_ack = terminal
    assert matched_frame is frame
    assert matched_ack is ack


def test_drain_and_poll_ring_match_wins_over_a_differing_scalar_slot_on_the_same_frame():
    """Disambiguation: the SAME frame's own scalar slot can be fresh for a
    DIFFERENT, later corr_id (some other command's ack) while its ring
    still carries the awaited move_id -- the ring match must be found, not
    shadowed by the unrelated scalar slot."""
    unrelated = AckEntry(corr_id=99, ok=False, err_code=5)
    frame = _frame(ack=unrelated, acks=[AckEntry(corr_id=7, ok=True, err_code=0)])
    transport = _StubTransport([frame])
    latest_frame = [None]

    terminal = _drain_and_poll(transport, 7, latest_frame)

    assert terminal is not None
    matched_frame, matched_ack = terminal
    assert matched_ack.corr_id == 7
    assert matched_ack.ok is True


def test_drain_and_poll_returns_none_when_no_frame_matches():
    frame = _frame(ack=None, acks=[AckEntry(corr_id=1, ok=True, err_code=0)])
    transport = _StubTransport([frame])
    latest_frame = [None]

    terminal = _drain_and_poll(transport, 7, latest_frame)

    assert terminal is None
    assert latest_frame[0] is frame  # still tracks the latest drained frame


def test_drain_and_poll_returns_first_matching_frame_in_batch_order():
    frame_a = _frame(acks=[AckEntry(corr_id=1, ok=True, err_code=0)])
    frame_b = _frame(acks=[AckEntry(corr_id=7, ok=True, err_code=0)])
    frame_c = _frame(acks=[AckEntry(corr_id=7, ok=False, err_code=9)])  # a later, differing
    # entry for the SAME corr_id -- must be ignored once the first match is found (mirrors
    # _match_ack_in_frames()'s own "first match wins" policy, io/serial_conn.py).
    transport = _StubTransport([frame_a, frame_b, frame_c])
    latest_frame = [None]

    terminal = _drain_and_poll(transport, 7, latest_frame)

    matched_frame, matched_ack = terminal
    assert matched_frame is frame_b
    assert matched_ack.err_code == 0
    # latest_frame[0] still ends up as the LAST frame drained this batch,
    # independent of which one matched -- pose/heading bookkeeping must
    # keep advancing even past the matched frame.
    assert latest_frame[0] is frame_c


def test_drain_and_poll_scans_every_ring_entry_within_one_frame():
    frame = _frame(acks=[
        AckEntry(corr_id=10, ok=True, err_code=0),
        AckEntry(corr_id=11, ok=True, err_code=0),
        AckEntry(corr_id=12, ok=False, err_code=3),
        AckEntry(corr_id=13, ok=True, err_code=0),
    ])
    transport = _StubTransport([frame])
    latest_frame = [None]

    terminal = _drain_and_poll(transport, 12, latest_frame)

    assert terminal is not None
    _, matched_ack = terminal
    assert matched_ack.corr_id == 12
    assert matched_ack.err_code == 3


# ---------------------------------------------------------------------------
# 2. _outcome_for_terminal_frame() -- reads the MATCHED entry, not the
#    frame's own (possibly unrelated) scalar slot.
# ---------------------------------------------------------------------------


def test_outcome_for_terminal_frame_completed_on_ok_ack_no_timeout_flag():
    frame = _frame(flags=0)
    ack = AckEntry(corr_id=7, ok=True, err_code=0)

    assert _outcome_for_terminal_frame(frame, ack) == RunOutcome.COMPLETED


def test_outcome_for_terminal_frame_fault_on_nonzero_err_on_the_matched_entry():
    frame = _frame(flags=0)
    ack = AckEntry(corr_id=7, ok=False, err_code=4)

    assert _outcome_for_terminal_frame(frame, ack) == RunOutcome.FAULT


def test_outcome_for_terminal_frame_fault_on_move_timeout_flag_even_with_ok_ack():
    frame = _frame(flags=_FLAG_FAULT_MOVE_TIMEOUT)
    ack = AckEntry(corr_id=7, ok=True, err_code=0)

    assert _outcome_for_terminal_frame(frame, ack) == RunOutcome.FAULT


def test_outcome_for_terminal_frame_ignores_the_frames_own_unrelated_scalar_slot():
    """The matched entry is OK, but the enclosing frame's own scalar
    "freshest ack" slot belongs to a DIFFERENT, failed command -- the
    outcome must follow the matched entry, not `frame.ack`."""
    unrelated_failed_ack = AckEntry(corr_id=99, ok=False, err_code=5)
    frame = _frame(flags=0, ack=unrelated_failed_ack)
    matched = AckEntry(corr_id=7, ok=True, err_code=0)

    assert _outcome_for_terminal_frame(frame, matched) == RunOutcome.COMPLETED


# ---------------------------------------------------------------------------
# 3. run_tour() end to end -- a fake MoveTransport whose completions ride
#    the ack RING ONLY (TLMFrame.ack is None on every frame it ever
#    returns), modeling the exact lossy-bench-link mechanism this ticket
#    root-caused. AC: "the tour retires the final leg even when the single
#    fresh-slot frame is dropped but the ring carries the ack" / "a full
#    tour completes ... WITHOUT a STOP press."
# ---------------------------------------------------------------------------


_RING_SURVIVAL_PUSHES = 4  # mirrors kAckRingDepth (telemetry.proto) -- see the class docstring
# below: a real ring entry survives across roughly this many SUBSEQUENT telemetry pushes
# (evicted only once genuinely full), not just the one push it was first created on.


class _RingOnlyFakeTransport:
    """``MoveTransport`` double: integrates each leg open-loop (mirrors
    ``test_tour1_geometry.py``'s own ``_FakeTwistTransport.move()``), but
    reports every completion ack ONLY via the bounded ``acks`` ring --
    ``TLMFrame.ack``/``ack_corr``/``ack_err`` stay `None` on every frame
    this fake ever returns. A caller that reads only the scalar slot (the
    pre-121-002 `_drain_and_poll()`) would never see a completion here and
    every leg would time out (`RunOutcome.FAULT`) -- this fake exists to
    prove the fixed, ring-scanning `_drain_and_poll()` does.

    Each ``read_pending_binary_tlm_frames()`` call models ONE incoming
    telemetry push: it returns a single frame carrying a SNAPSHOT of every
    still-alive ring entry, then ages each entry by one push, evicting it
    once it has survived ``_RING_SURVIVAL_PUSHES`` reads. This -- not a
    one-shot "return it once, forget it forever" queue -- is what makes
    this fake an honest model of the real wire ring `run_tour()`'s
    one-leg lookahead depends on: leg N+1's own completion ack is queued
    (`move()`) before leg N's own completion has even been polled for, so
    it must still be observable on leg N+1's own LATER poll even after an
    EARLIER poll (leg N's) already drained a batch that happened to carry
    the SAME underlying push.
    """

    def __init__(self) -> None:
        self.move_calls: list[dict] = []
        self.stop_calls = 0
        self._corr_id = 0
        self._ring: list[list[int]] = []  # [move_id, pushes_remaining] pairs, oldest-first
        self._x = 0.0  # [mm]
        self._y = 0.0  # [mm]
        self._heading = 0.0  # [rad]
        self._enc = 0.0  # [mm]

    def move(self, *, v_x: float = 0.0, v_y: float = 0.0, omega: float = 0.0,
             v_left: "float | None" = None, v_right: "float | None" = None,
             stop_time: "float | None" = None, stop_distance: "float | None" = None,
             stop_angle: "float | None" = None, timeout: float,
             replace: bool = True, id: "int | None" = None) -> int:
        self._corr_id += 1
        move_id = id if id is not None else self._corr_id
        self.move_calls.append(dict(v_x=v_x, omega=omega, stop_distance=stop_distance,
                                    stop_angle=stop_angle, timeout=timeout, id=move_id))
        if stop_distance is not None:
            signed = math.copysign(stop_distance, v_x) if v_x else stop_distance
            self._x += signed * math.cos(self._heading)
            self._y += signed * math.sin(self._heading)
            self._enc += signed
        elif stop_angle is not None:
            signed = math.copysign(stop_angle, omega) if omega else stop_angle
            self._heading += signed
        self._ring.append([move_id, _RING_SURVIVAL_PUSHES])
        return move_id

    def stop(self) -> int:
        self._corr_id += 1
        self.stop_calls += 1
        return self._corr_id

    def read_pending_binary_tlm_frames(self) -> list[TLMFrame]:
        acks = [AckEntry(corr_id=move_id, ok=True, err_code=0) for move_id, _ in self._ring]
        frame = self._make_frame(acks)
        # Age every entry by this one push; evict anything that has now
        # exhausted its survival window (mirrors the real ring's own
        # bounded, oldest-evicted-first depth).
        for entry in self._ring:
            entry[1] -= 1
        self._ring = [entry for entry in self._ring if entry[1] > 0]
        return [frame]

    def _make_frame(self, acks: "list[AckEntry]") -> TLMFrame:
        enc_i = int(self._enc)
        pose = (int(self._x), int(self._y), int(round(math.degrees(self._heading) * 100.0)))
        # ack/ack_corr/ack_err stay None -- see this class's own docstring:
        # every completion this fake reports rides the ring ONLY.
        return TLMFrame(enc=(enc_i, enc_i), pose=pose, otos=pose, flags=0,
                        ack=None, ack_corr=None, ack_err=None, acks=acks)


def _heading_for_test() -> HeadingCorrector:
    return HeadingCorrector(
        PlannerParams(), robot_config=SimpleNamespace(geometry=SimpleNamespace(otos_untrusted=True)))


def test_run_tour_single_leg_retires_completed_when_only_the_ring_carries_the_ack():
    transport = _RingOnlyFakeTransport()
    legs = [TourLeg(kind="distance", value=300.0, speed=150.0)]

    result = run_tour(transport, PlannerParams(), _heading_for_test(), legs)

    assert result.stopped_at is None
    assert len(result.legs) == 1
    assert result.legs[0].outcome == RunOutcome.COMPLETED
    assert transport.stop_calls == 0, "a completed tour must never call stop()"


def test_run_tour_full_tour_completes_without_stop_when_every_leg_only_rides_the_ring():
    """AC: 'a full tour completes and reports closure WITHOUT a STOP
    press' -- exercised against TOUR_1's own shape (distance/turn legs
    chained via the one-leg lookahead), every completion carried ONLY by
    the ack ring. This is the direct regression for
    tour-1-final-leg-completes-only-on-stop.md: with the pre-121-002
    single-slot-only `_drain_and_poll()`, this fake never satisfies ANY
    leg's completion (`TLMFrame.ack` is always `None`) and every leg,
    including the final one, would FAULT on `move_timeout` -- never
    retire COMPLETED on its own."""
    from robot_radio.planner.tour import TOUR_1

    transport = _RingOnlyFakeTransport()
    legs = parse_tour(TOUR_1)
    assert len(legs) == 13

    result = run_tour(transport, PlannerParams(), _heading_for_test(), legs)

    assert result.stopped_at is None, (
        f"tour did not complete on its own -- stopped at leg {result.stopped_at} "
        f"({result.stopped_outcome}); every completion ack in this fake rides the "
        f"ring only, so a non-ring-aware _drain_and_poll() would never see it"
    )
    assert len(result.legs) == 13
    for i, leg_result in enumerate(result.legs):
        assert leg_result.outcome == RunOutcome.COMPLETED, (
            f"leg {i + 1}/13 ({leg_result.leg.kind} {leg_result.leg.value:g}) "
            f"did not retire COMPLETED on its own: {leg_result.outcome}"
        )
    # The whole point of the fix: no leg -- especially not the FINAL one --
    # required a STOP to retire.
    assert transport.stop_calls == 0, "no leg should have required stop() to retire"
    assert len(transport.move_calls) == 13, "one move() call per leg, no re-sends"


if __name__ == "__main__":
    import sys

    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
