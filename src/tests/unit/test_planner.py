"""src/tests/unit/test_planner.py -- ticket 127-006 originally; reshaped
135-007.

Pure, synthetic-input unit tests for ``robot_radio.pathplan.planner`` --
the give-up/termination logic, the id allocator, the ack-verified
``GO_TO`` send/retry machinery, and ``gotoWorld()``/``gotoRobot()``/
``followPath()`` as thin ``GO_TO`` senders. No hardware, no sim binary, no
camera daemon, no serial port -- the full end-to-end loop against a REAL
firmware ``Motion::Navigator`` is exercised separately at the sim tier
(``src/tests/sim/test_pathplan_goto_convergence.py``, real ``SimLoop``).

History (135-007): before this ticket, ``gotoWorld()``/``gotoRobot()`` ran
their own arc-solving/replace-throttling loop and this file tested that
loop directly -- ``_shouldReplace()`` (the material-change throttle),
``ProgressCheck`` (the liveness backstop), ``_moveTimeoutFor()``
(per-move timeout derivation), ``_targetBehindReason()`` (the host-side
target-behind guard's own give-up message), and ``_sendVerifiedTwist()``
(the ``move_twist()`` ack-retry sender). All of those are DELETED
alongside the functions/classes they served -- ``Motion::Navigator`` now
owns material-change throttling, liveness, per-move timeouts, and
bearing/pivot handling entirely firmware-side (ctest-covered by
`src/motion/navigator/tests/navigator_test.cpp`, tickets 002-003) -- and
this file's own coverage of them is deleted, not left red, per the
ticket's own instruction. What survives, tests the SAME thing it always
did (give-up is explicit and reachable; the id allocator is strictly
monotonic; a frame drain dispatches correctly; the geofence is checked
inside the time-advance window; an ack-verified send retries correctly on
loss/ERR_FULL/a non-retryable NACK); what is NEW covers
``gotoWorld()``/``gotoRobot()`` sending a ``GO_TO`` directly and waiting
for its one completion ack, and ``followPath()`` streaming
``pursuitTarget()``'s picks as ``GO_TO`` commands on a throttled cadence.

Covers (ticket acceptance criteria):
  1. Give-up is explicit and reachable: ``_giveUpReason()`` (iteration and
     timeout caps) always reports WHY.
  2. ``MoveIdAllocator`` is strictly monotonic, never emits 0 -- now the
     shared id source for BOTH Move.id and GoTo.id.
  3. ``_advance()`` checks the geofence INSIDE its own time-advance
     window, not between segments.
  4. ``_readFrames()`` dispatches to whichever backend proto._conn
     exposes.
  5. ``_sendVerifiedGoTo()`` -- ack verification and retry for the
     ``GO_TO`` send, including the non-retryable-NACK and
     exhausted-retries cases.
  6. ``gotoWorld()``/``gotoRobot()`` send ONE ``GO_TO`` (frame WORLD/ROBOT
     respectively) with ack-verified retry, then wait for the single
     completion ack -- covering success, an aborted goto, a never-acked
     enqueue, and a give-up while waiting; ``gotoRobot()`` needs no
     pre-existing ``WorldPose`` fix (the firmware resolves the robot-frame
     offset itself).
  7. ``followPath()``: a straight multi-waypoint run reaches every
     waypoint and succeeds at the terminal one; ``GiveUpLimits`` fires
     explicitly for a genuinely stalled robot; every cycle sends a fresh
     ``GO_TO`` unconditionally (no throttle -- see ``planner.py``'s own
     module-level comment above ``_lookaheadFor()`` for the measured
     reason a wall-clock send throttle was tried and reverted).
"""

from __future__ import annotations

import dataclasses
import time

import pytest

from robot_radio.nav.pose import Pose
from robot_radio.pathplan.world_pose import WorldPose
from robot_radio.robot.protocol import (
    GOTO_FRAME_ROBOT,
    GOTO_FRAME_WORLD,
    AckEntry,
    EncoderReading,
    TLMFrame,
)

from robot_radio.pathplan.planner import (
    AckRetry,
    FollowPathResult,
    GiveUpLimits,
    GotoResult,
    MoveIdAllocator,
    MoveRejected,
    _advance,
    _ERR_FULL,
    _giveUpReason,
    _lookaheadFor,
    _readFrames,
    _recordAcks,
    _sendVerifiedGoTo,
    followPath,
    gotoRobot,
    gotoWorld,
)


# ---------------------------------------------------------------------------
# 1. Termination/give-up logic -- _giveUpReason().
# ---------------------------------------------------------------------------


def test_give_up_reason_none_within_budget():
    limits = GiveUpLimits(maxIterations=200, giveUpTimeout=30.0)
    assert _giveUpReason(iterations=5, elapsed=1.0, limits=limits) is None


def test_give_up_reason_fires_at_max_iterations():
    limits = GiveUpLimits(maxIterations=10, giveUpTimeout=30.0)
    reason = _giveUpReason(iterations=10, elapsed=1.0, limits=limits)
    assert reason is not None
    assert "10" in reason and "iterations" in reason


def test_give_up_reason_fires_at_timeout():
    limits = GiveUpLimits(maxIterations=1000, giveUpTimeout=5.0)
    reason = _giveUpReason(iterations=1, elapsed=5.0, limits=limits)
    assert reason is not None
    assert "5.0s" in reason


def test_give_up_reason_is_never_silent():
    # "not a silent infinite retry" -- once a budget is exhausted the
    # reason string is always non-empty and explains WHY.
    limits = GiveUpLimits(maxIterations=3, giveUpTimeout=1.0)
    reason = _giveUpReason(iterations=3, elapsed=0.1, limits=limits)
    assert reason
    assert isinstance(reason, str)


# ---------------------------------------------------------------------------
# 2. MoveIdAllocator -- strictly monotonic, never 0. Shared by Move.id and
#    GoTo.id alike since 135-007; behavior/API unchanged.
# ---------------------------------------------------------------------------


def test_move_id_allocator_starts_at_one_by_default():
    allocator = MoveIdAllocator()
    assert allocator.next() == 1


def test_move_id_allocator_strictly_monotonic_no_repeats_no_regression():
    allocator = MoveIdAllocator()
    ids = [allocator.next() for _ in range(20)]
    assert ids == sorted(set(ids))  # strictly increasing -> sorted == itself, no dupes
    assert len(set(ids)) == 20
    for a, b in zip(ids, ids[1:]):
        assert b > a


def test_move_id_allocator_rejects_zero_or_negative_start():
    with pytest.raises(ValueError):
        MoveIdAllocator(start=0)
    with pytest.raises(ValueError):
        MoveIdAllocator(start=-5)


def test_move_id_allocator_never_emits_zero():
    allocator = MoveIdAllocator()
    ids = [allocator.next() for _ in range(50)]
    assert 0 not in ids


# ---------------------------------------------------------------------------
# 3. _readFrames() dispatches to whichever backend proto._conn exposes.
# ---------------------------------------------------------------------------


class _ConnWithOwnReader:
    """Mimics SimConfigConn: the CONNECTION itself exposes
    read_pending_binary_tlm_frames() (already-adapted TLMFrames)."""

    def __init__(self, frames):
        self._frames = frames

    def read_pending_binary_tlm_frames(self):
        return self._frames


class _ProtoWithSimLikeConn:
    def __init__(self, frames):
        self._conn = _ConnWithOwnReader(frames)

    def read_pending_binary_tlm_frames(self):
        raise AssertionError("should never be called -- _conn has its own reader")


class _ConnWithNoReader:
    """Mimics SerialConnection: no read_pending_binary_tlm_frames() of its
    own -- only drain_binary_tlm()/read_binary_tlm(), which
    NezhaProtocol.read_pending_binary_tlm_frames() itself adapts."""


class _ProtoWithHardwareLikeConn:
    def __init__(self, frames):
        self._conn = _ConnWithNoReader()
        self._frames = frames

    def read_pending_binary_tlm_frames(self):
        return self._frames


def test_read_frames_prefers_the_connections_own_reader_when_present():
    sentinel = ["sim-frame"]
    proto = _ProtoWithSimLikeConn(sentinel)
    assert _readFrames(proto) is sentinel


def test_read_frames_falls_back_to_the_protocol_level_reader():
    sentinel = ["hardware-frame"]
    proto = _ProtoWithHardwareLikeConn(sentinel)
    assert _readFrames(proto) is sentinel


# ---------------------------------------------------------------------------
# 4. _advance() checks the geofence INSIDE its own time-advance window.
# ---------------------------------------------------------------------------


class _FakeProtoNoFrames:
    """Mimics the HARDWARE path shape: no `_conn.read_pending_binary_tlm_
    frames()` of its own (matching plain `SerialConnection`), so
    `_readFrames()` falls back to this object's own
    `read_pending_binary_tlm_frames()` -- exactly like a real
    `NezhaProtocol` wrapping a `SerialConnection`."""

    _conn = None

    def read_pending_binary_tlm_frames(self):
        return []


class _RecordingGeofence:
    def __init__(self):
        self.checkCount = 0

    def check(self):
        self.checkCount += 1


def test_advance_checks_geofence_inside_the_window_not_between_segments():
    proto = _FakeProtoNoFrames()
    geofence = _RecordingGeofence()

    frames = _advance(proto, seconds=0.05, geofence=geofence)

    assert frames == []
    # The geofence must have been checked DURING this call, not left for
    # the caller to check afterward -- "never between segments".
    assert geofence.checkCount >= 1


def test_advance_never_checks_a_none_geofence():
    proto = _FakeProtoNoFrames()
    # No exception, no-op -- a caller with no camera (e.g. a bare sim run).
    frames = _advance(proto, seconds=0.02, geofence=None)
    assert frames == []


# ---------------------------------------------------------------------------
# 5. _sendVerifiedGoTo() -- ack verification and retry, the GO_TO analogue
#    of the deleted move_twist()-era _sendVerifiedTwist().
# ---------------------------------------------------------------------------


class _FakeAckFrame:
    """The minimal frame shape `_sendVerifiedGoTo()`/`_recordAcks()` need
    -- just `.acks`, no pose/encoder fields."""

    def __init__(self, acks):
        self.acks = acks


class _NullWorldPose:
    """A no-op stand-in for `WorldPose` -- `_sendVerifiedGoTo()` calls
    `.ingest()` on every frame it drains; these tests don't care about
    pose tracking, only about the ack state machine."""

    def ingest(self, frame):
        pass


class _AckScheduleProto:
    """Fakes `NezhaProtocol` for `_sendVerifiedGoTo()` tests: each
    `go_to()` call is one "attempt" (1-based); `ackByAttempt` maps attempt
    number -> `(ok, err_code)` to hand back (as an `AckEntry` matching
    that attempt's own assigned corr_id) on every subsequent
    `read_pending_binary_tlm_frames()` poll, until superseded by the next
    attempt's own send. An attempt number absent from `ackByAttempt` never
    acks at all within its window -- exercises the retry-on-no-ack path
    distinctly from the retry-on-ERR_FULL path."""

    _conn = None

    def __init__(self, ackByAttempt: dict):
        self._ackByAttempt = ackByAttempt
        self._attempt = 0
        self._nextCorr = 100
        self._currentCorr = None
        self.gotoIds = []

    def go_to(self, x, y, *, frame, speed, arrive, timeout, goto_id):
        self._attempt += 1
        self.gotoIds.append(goto_id)
        self._currentCorr = self._nextCorr
        self._nextCorr += 1
        return self._currentCorr

    def read_pending_binary_tlm_frames(self):
        spec = self._ackByAttempt.get(self._attempt)
        if spec is None:
            return []
        ok, errCode = spec
        return [_FakeAckFrame([AckEntry(corr_id=self._currentCorr, ok=ok, err_code=errCode)])]


def _verify(proto, *, gotoId=1, maxAttempts=4, ackTimeout=0.03, cyclePeriod=0.01):
    ackSeen: "dict[int, object]" = {}
    return _sendVerifiedGoTo(
        proto, _NullWorldPose(), None, ackSeen, gotoId=gotoId,
        x=300.0, y=0.0, frame=GOTO_FRAME_WORLD, speed=150.0, arrive=0.0, timeout=3000.0,
        cyclePeriod=cyclePeriod, ackRetry=AckRetry(maxAttempts=maxAttempts, ackTimeout=ackTimeout))


def test_send_verified_go_to_acked_on_first_attempt():
    proto = _AckScheduleProto({1: (True, 0)})
    acked, attempts = _verify(proto)
    assert acked is True
    assert attempts == 1
    assert proto.gotoIds == [1]  # exactly one wire attempt -- no retry needed


def test_send_verified_go_to_retries_on_lost_ack_reusing_the_same_goto_id():
    # Attempt 1 never acks at all (simulates a dropped inbound packet);
    # attempt 2 acks ok.
    proto = _AckScheduleProto({2: (True, 0)})
    acked, attempts = _verify(proto, gotoId=7)
    assert acked is True
    assert attempts == 2
    # Both wire attempts of ONE logical send reuse the SAME GoTo.id -- see
    # _sendVerifiedGoTo()'s own docstring for why that is still correct
    # even though (unlike Move.id) there is no firmware dedup ring behind
    # it: a retry just restarts navigation toward the identical target.
    assert proto.gotoIds == [7, 7]


def test_send_verified_go_to_retries_on_err_full():
    proto = _AckScheduleProto({1: (False, _ERR_FULL), 2: (True, 0)})
    acked, attempts = _verify(proto)
    assert acked is True
    assert attempts == 2


def test_send_verified_go_to_raises_on_non_retryable_err():
    _ERR_BADARG = 2  # envelope.proto ErrCode.ERR_BADARG
    proto = _AckScheduleProto({1: (False, _ERR_BADARG)})
    with pytest.raises(MoveRejected):
        _verify(proto)
    # A non-retryable NACK fails LOUDLY on the first bad ack -- it must
    # not burn the rest of the retry budget on a command that can never
    # succeed.
    assert proto.gotoIds == [1]


def test_send_verified_go_to_exhausts_retries_and_reports_unacked():
    proto = _AckScheduleProto({})  # never acks at all
    acked, attempts = _verify(proto, maxAttempts=3)
    assert acked is False
    assert attempts == 3
    assert proto.gotoIds == [1, 1, 1]  # every retry of ONE send reuses the id


def test_record_acks_folds_every_frame_ring_into_the_shared_dict():
    ackSeen: "dict[int, object]" = {}
    frames = [
        _FakeAckFrame([AckEntry(corr_id=5, ok=True, err_code=0)]),
        _FakeAckFrame([AckEntry(corr_id=6, ok=False, err_code=_ERR_FULL)]),
    ]
    _recordAcks(frames, ackSeen)
    assert ackSeen[5].ok is True
    assert ackSeen[6].ok is False


# ---------------------------------------------------------------------------
# 6. gotoWorld()/gotoRobot() -- thin GO_TO senders (135-007).
# ---------------------------------------------------------------------------


def _tlmFrame(x_mm: float, y_mm: float, heading_cdeg: float, acks=()) -> TLMFrame:
    """A synthetic, hand-decoded TLMFrame at REST (zero twist, so
    WorldPose's frame-age extrapolation is a no-op regardless of real
    wall-clock elapsed time -- deterministic)."""
    return TLMFrame(
        t=1000, pose=(x_mm, y_mm, heading_cdeg), twist=(0, 0), recvTime=time.monotonic(),
        enc_left=EncoderReading(position=0.0, velocity=0.0, age=0, position_epoch=0),
        enc_right=EncoderReading(position=0.0, velocity=0.0, age=0, position_epoch=0),
        acks=list(acks))


class _GotoFakeProto:
    """`NezhaProtocol`-shaped fake for `gotoWorld()`/`gotoRobot()`
    integration tests. Records every `go_to()` call; hands back an
    ENQUEUE ack on the read immediately following each call (unless
    `ackEnqueue` is False, simulating a lost enqueue ack entirely that
    never recovers), and -- keyed on whatever `goto_id` the call under
    test actually used, captured from the call itself rather than
    hardcoded test-side -- a single COMPLETION ack after
    `completeAfterReads` more reads (`None` means never, for a
    give-up-while-waiting test)."""

    _conn = None

    def __init__(self, *, ackEnqueue: bool = True, completeAfterReads: "int | None" = 2,
                 completeOk: bool = True, completeErrCode: int = 0,
                 x_mm: float = 0.0, y_mm: float = 0.0, heading_cdeg: float = 0.0):
        self._ackEnqueue = ackEnqueue
        self._completeAfterReads = completeAfterReads
        self._completeOk = completeOk
        self._completeErrCode = completeErrCode
        self._x_mm = x_mm
        self._y_mm = y_mm
        self._heading_cdeg = heading_cdeg
        # Starts far above any goto_id these tests allocate (small,
        # MoveIdAllocator-default-start values) -- corr_id and goto_id are
        # DIFFERENT key spaces on the real wire (envelope corr_id vs.
        # GoTo.id), and this fake must not let them collide numerically or
        # it will conflate an enqueue ack with a completion ack.
        self._nextCorr = 1000
        self._readsSinceLastCall = 0
        self._pendingEnqueueAck = None
        self._gotoId: "int | None" = None
        self.calls: "list[dict]" = []
        self.estopCount = 0

    def go_to(self, x, y, *, frame, speed, arrive, timeout, goto_id):
        self.calls.append(dict(x=x, y=y, frame=frame, speed=speed, arrive=arrive,
                               timeout=timeout, goto_id=goto_id))
        corr = self._nextCorr
        self._nextCorr += 1
        self._gotoId = goto_id
        self._readsSinceLastCall = 0
        if self._ackEnqueue:
            self._pendingEnqueueAck = AckEntry(corr_id=corr, ok=True, err_code=0)
        return corr

    def read_pending_binary_tlm_frames(self):
        self._readsSinceLastCall += 1
        acks = []
        if self._pendingEnqueueAck is not None:
            acks.append(self._pendingEnqueueAck)
            self._pendingEnqueueAck = None
        if (self._completeAfterReads is not None and self._gotoId is not None
                and self._readsSinceLastCall >= self._completeAfterReads):
            acks.append(AckEntry(corr_id=self._gotoId, ok=self._completeOk,
                                 err_code=self._completeErrCode))
            self._gotoId = None  # fire exactly once
        return [_tlmFrame(self._x_mm, self._y_mm, self._heading_cdeg, acks=acks)]

    def estop(self):
        self.estopCount += 1


def test_goto_world_sends_go_to_with_world_frame_and_scaled_mm():
    worldPose = WorldPose()
    proto = _GotoFakeProto(completeAfterReads=1)
    result = gotoWorld(proto, worldPose, x=30.0, y=5.0, cyclePeriod=0.01)
    assert result.success is True
    assert len(proto.calls) == 1
    call = proto.calls[0]
    assert call["frame"] == GOTO_FRAME_WORLD
    assert call["x"] == pytest.approx(300.0)  # 30 cm -> 300 mm
    assert call["y"] == pytest.approx(50.0)   # 5 cm -> 50 mm


def test_goto_robot_sends_go_to_with_robot_frame_unrotated():
    """gotoRobot() no longer transforms x/y itself -- unlike its
    pre-135-007 form, the (x, y) offset reaches the wire EXACTLY as given
    (only scaled cm -> mm), with frame=ROBOT telling the firmware to
    resolve it against the robot's own live pose at acceptance."""
    worldPose = WorldPose()  # deliberately NEVER ingest()-ed -- see the test below
    proto = _GotoFakeProto(completeAfterReads=1)
    result = gotoRobot(proto, worldPose, x=40.0, y=-10.0, cyclePeriod=0.01)
    assert result.success is True
    call = proto.calls[0]
    assert call["frame"] == GOTO_FRAME_ROBOT
    assert call["x"] == pytest.approx(400.0)
    assert call["y"] == pytest.approx(-100.0)


def test_goto_robot_works_without_any_prior_world_pose():
    """The pre-135-007 gotoRobot() raised RuntimeError without a seeded
    WorldPose (it needed a current pose to compose the robot-frame offset
    itself). That requirement is gone: the FIRMWARE resolves the offset
    now, using its own live OTOS pose, so gotoRobot() needs nothing from
    WorldPose to send its GO_TO."""
    worldPose = WorldPose()
    assert worldPose.worldPose() is None  # confirm: genuinely unseeded
    proto = _GotoFakeProto(completeAfterReads=1)
    result = gotoRobot(proto, worldPose, x=10.0, y=0.0, cyclePeriod=0.01)
    assert result.success is True


def test_goto_world_reports_failure_on_an_aborted_completion_ack():
    worldPose = WorldPose()
    proto = _GotoFakeProto(completeAfterReads=1, completeOk=False, completeErrCode=9)
    result = gotoWorld(proto, worldPose, x=30.0, y=0.0, cyclePeriod=0.01)
    assert result.success is False
    assert "aborted" in result.reason
    assert "9" in result.reason
    assert result.unacked == 0  # the ENQUEUE was fine -- the GOTO itself aborted


def test_goto_world_reports_unacked_when_enqueue_never_lands():
    worldPose = WorldPose()
    proto = _GotoFakeProto(ackEnqueue=False)
    ackRetry = AckRetry(maxAttempts=2, ackTimeout=0.02)
    result = gotoWorld(proto, worldPose, x=30.0, y=0.0, cyclePeriod=0.01, ackRetry=ackRetry)
    assert result.success is False
    assert result.unacked == 1
    assert result.sent == 1
    assert "link/command loss" in result.reason
    # Never even reached the completion-ack wait -- no send was possible.
    assert result.iterations == 0


def test_goto_world_gives_up_waiting_for_a_completion_ack_that_never_arrives():
    worldPose = WorldPose()
    proto = _GotoFakeProto(completeAfterReads=None)  # enqueue acks fine; completion never comes
    giveUp = GiveUpLimits(maxIterations=100_000, giveUpTimeout=0.1)
    result = gotoWorld(proto, worldPose, x=30.0, y=0.0, cyclePeriod=0.01, giveUp=giveUp)
    assert result.success is False
    assert "gave up after" in result.reason
    assert "no completion ack" in result.reason
    assert result.unacked == 0  # the enqueue itself was fine


def test_goto_world_estops_on_every_exit():
    worldPose = WorldPose()
    # Success case.
    proto = _GotoFakeProto(completeAfterReads=1)
    gotoWorld(proto, worldPose, x=30.0, y=0.0, cyclePeriod=0.01)
    assert proto.estopCount == 1

    # Give-up case.
    proto2 = _GotoFakeProto(completeAfterReads=None)
    giveUp = GiveUpLimits(maxIterations=100_000, giveUpTimeout=0.05)
    gotoWorld(proto2, WorldPose(), x=30.0, y=0.0, cyclePeriod=0.01, giveUp=giveUp)
    assert proto2.estopCount == 1


class _NackingGoToProto:
    """`go_to()` always NACKs with a non-retryable ErrCode -- exercises
    MoveRejected propagation out of gotoWorld() (through _gotoAndWait()'s
    own finally block)."""

    _conn = None

    def __init__(self):
        self._nextCorr = 1
        self.estopCount = 0

    def go_to(self, x, y, *, frame, speed, arrive, timeout, goto_id):
        corr = self._nextCorr
        self._nextCorr += 1
        self._pendingCorr = corr
        return corr

    def read_pending_binary_tlm_frames(self):
        acks = [AckEntry(corr_id=self._pendingCorr, ok=False, err_code=2)]  # ERR_BADARG
        self._pendingCorr = None
        return [_tlmFrame(0.0, 0.0, 0.0, acks=acks)]

    def estop(self):
        self.estopCount += 1


def test_goto_world_raises_move_rejected_and_still_estops():
    proto = _NackingGoToProto()
    with pytest.raises(MoveRejected):
        gotoWorld(proto, WorldPose(), x=30.0, y=0.0, cyclePeriod=0.01,
                 ackRetry=AckRetry(maxAttempts=2, ackTimeout=0.02))
    assert proto.estopCount == 1  # the halt still ran despite the exception


def test_goto_world_derives_the_wire_timeout_from_give_up_timeout():
    proto = _GotoFakeProto(completeAfterReads=1)
    giveUp = GiveUpLimits(maxIterations=200, giveUpTimeout=12.5)
    gotoWorld(proto, WorldPose(), x=1.0, y=0.0, cyclePeriod=0.01, giveUp=giveUp)
    assert proto.calls[0]["timeout"] == pytest.approx(12500.0)


def test_goto_world_shares_one_allocator_across_sequential_calls():
    allocator = MoveIdAllocator(start=5)
    proto1 = _GotoFakeProto(completeAfterReads=1)
    gotoWorld(proto1, WorldPose(), x=1.0, y=0.0, cyclePeriod=0.01, moveIds=allocator)
    proto2 = _GotoFakeProto(completeAfterReads=1)
    gotoWorld(proto2, WorldPose(), x=2.0, y=0.0, cyclePeriod=0.01, moveIds=allocator)
    assert proto1.calls[0]["goto_id"] == 5
    assert proto2.calls[0]["goto_id"] == 6  # strictly greater -- one shared id space


# ---------------------------------------------------------------------------
# 7. followPath() -- streams pursuitTarget()'s picks as GO_TO commands
#    (135-007). The target geometry itself (solver.pursuitTarget()) is
#    unit-tested standalone in test_solver.py; these tests exercise
#    followPath() ITSELF -- the ingest/target/send ordering, the
#    end-of-path arrival test, GiveUpLimits, and the unconditional
#    every-cycle send (no throttle -- see planner.py's own module-level
#    comment above _lookaheadFor() for the measured reason a wall-clock
#    send throttle was tried here first and reverted).
# ---------------------------------------------------------------------------


def _cm(mm: float) -> float:
    return mm / 10.0


def test_lookahead_derivation():
    # 2.0 * 150 mm/s * (0.15 s dead time + 0.23 s plant tau) == 114 mm --
    # see the module-level comment above _ACTUATION_DELAY for both bounds
    # (steering-lag floor, corner-cutting ceiling) and why they nearly meet.
    assert _lookaheadFor(150.0) == pytest.approx(114.0)
    # The floor still binds for a very slow commanded speed.
    assert _lookaheadFor(10.0) == pytest.approx(60.0)


def test_follow_path_result_has_waypoints_reached_field():
    fieldNames = {f.name for f in dataclasses.fields(FollowPathResult)}
    assert fieldNames == {"success", "reason", "finalPose", "iterations", "sent",
                          "waypointsReached", "retries", "unacked"}


class _MovingFakeGoToProto:
    """`NezhaProtocol`-shaped fake for `followPath()` integration tests:
    the world pose it reports advances in a STRAIGHT LINE (+x) by
    `stepMm` every `read_pending_binary_tlm_frames()` call, simulating a
    robot that tracks a commanded on-heading target perfectly."""

    _conn = None

    def __init__(self, stepMm: float = 5.0) -> None:
        self._x_mm = 0.0
        self._stepMm = stepMm
        self._nextCorr = 1
        self._pendingAck = None
        self.calls: "list[dict]" = []
        self.estopped = False

    def go_to(self, x, y, *, frame, speed, arrive, timeout, goto_id):
        self.calls.append(dict(x=x, y=y, frame=frame, speed=speed, arrive=arrive,
                               timeout=timeout, goto_id=goto_id))
        corr = self._nextCorr
        self._nextCorr += 1
        self._pendingAck = AckEntry(corr_id=corr, ok=True, err_code=0)
        return corr

    def read_pending_binary_tlm_frames(self):
        self._x_mm += self._stepMm
        acks = []
        if self._pendingAck is not None:
            acks = [self._pendingAck]
            self._pendingAck = None
        return [_tlmFrame(self._x_mm, 0.0, 0, acks=acks)]

    def estop(self):
        self.estopped = True


def test_follow_path_straight_line_reaches_every_waypoint():
    worldPose = WorldPose()
    proto = _MovingFakeGoToProto(stepMm=5.0)
    waypoints = [Pose(x=_cm(mm), y=0.0, heading=0.0) for mm in (30.0, 60.0, 90.0, 120.0)]
    crossed: "list[int]" = []

    result = followPath(
        proto, worldPose, waypoints, speed=150.0, geofence=None, tolerance=12.0,
        giveUp=GiveUpLimits(maxIterations=500, giveUpTimeout=5.0), cyclePeriod=0.01,
        onWaypoint=lambda index, pose: crossed.append(index))

    assert result.success is True
    assert result.waypointsReached == len(waypoints)
    # Every waypoint crossed EXACTLY once, in strictly increasing order --
    # the follower's own ordering guarantee (ingest pose -> pick a target
    # from the monotone path projection -> send, never solve-then-project).
    assert crossed == list(range(len(waypoints)))
    # Every GO_TO sent this run carried the WORLD frame -- followPath()
    # always streams world-frame targets, regardless of the path's own
    # provenance.
    assert all(call["frame"] == GOTO_FRAME_WORLD for call in proto.calls)
    # estop(), never the planned stop() -- this loop's own finally block,
    # run on every return.
    assert proto.estopped is True


def test_follow_path_sends_every_cycle_with_no_throttle():
    """135-007's own measured correction: followPath() sends a fresh
    GO_TO on EVERY cycle, unconditionally -- an earlier design throttled
    sends to a 0.5s wall-clock interval (matching ticket 005's own
    sim-tested EXTERNAL-mode streaming cadence) and was MEASURED to cause
    repeated spurious stop-then-pivot-then-arc cycles on square_tour.py's
    own tight fillets (Motion::Navigator's bearing-to-target check runs
    every internal tick against a target that had gone stale). This test
    locks the current (correct) behavior: one logical send per iteration,
    every iteration, each with a fresh id -- not throttled to fewer sends
    than cycles run."""
    worldPose = WorldPose()
    proto = _MovingFakeGoToProto(stepMm=5.0)
    waypoints = [Pose(x=_cm(mm), y=0.0, heading=0.0) for mm in (30.0, 60.0, 90.0, 120.0)]

    result = followPath(
        proto, worldPose, waypoints, speed=150.0, geofence=None, tolerance=12.0,
        giveUp=GiveUpLimits(maxIterations=500, giveUpTimeout=5.0), cyclePeriod=0.01)

    assert result.success is True
    assert result.sent == result.iterations  # one send per iteration, no throttle
    assert result.sent > 1
    assert len(proto.calls) == result.sent
    # Every logical send drew a FRESH id -- followPath() never reuses a
    # goto_id across DIFFERENT streamed targets (only WITHIN one send's
    # own ack-loss retries, covered separately in section 5 above).
    ids = [call["goto_id"] for call in proto.calls]
    assert len(set(ids)) == len(ids)


class _StationaryFakeGoToProto:
    """`NezhaProtocol`-shaped fake: reports a FIXED world pose forever (a
    stalled/deadband robot, or a target the fake never actually reaches)
    while every `go_to()` send is acked OK immediately -- for exercising
    `GiveUpLimits` when arrival is never reached despite every send
    landing cleanly."""

    _conn = None

    def __init__(self, x_mm: float = 0.0, y_mm: float = 0.0, heading_cdeg: float = 0.0):
        self._x_mm = x_mm
        self._y_mm = y_mm
        self._heading_cdeg = heading_cdeg
        self._nextCorr = 1
        self._pendingAck = None
        self.calls: "list[dict]" = []

    def go_to(self, x, y, *, frame, speed, arrive, timeout, goto_id):
        self.calls.append(dict(x=x, y=y, frame=frame, speed=speed, arrive=arrive,
                               timeout=timeout, goto_id=goto_id))
        corr = self._nextCorr
        self._nextCorr += 1
        self._pendingAck = AckEntry(corr_id=corr, ok=True, err_code=0)
        return corr

    def read_pending_binary_tlm_frames(self):
        acks = []
        if self._pendingAck is not None:
            acks = [self._pendingAck]
            self._pendingAck = None
        return [_tlmFrame(self._x_mm, self._y_mm, self._heading_cdeg, acks=acks)]

    def estop(self):
        pass


def test_follow_path_gives_up_via_giveup_limits_for_a_stalled_robot():
    """A robot whose position never advances (every send acked ok, but
    nothing ever moves) must still terminate via GiveUpLimits, with an
    explicit reason -- never an infinite loop. UNLIKE the pre-135-007
    version of this test, sends DO happen here (there is no more
    target-behind guard to suppress them, and every cycle sends
    unconditionally) -- what never happens is arrival."""
    worldPose = WorldPose()
    proto = _StationaryFakeGoToProto(x_mm=0.0, y_mm=0.0)
    waypoints = [Pose(x=_cm(500.0), y=0.0, heading=0.0), Pose(x=_cm(1000.0), y=0.0, heading=0.0)]
    giveUp = GiveUpLimits(maxIterations=50, giveUpTimeout=5.0)

    result = followPath(proto, worldPose, waypoints, speed=150.0, geofence=None,
                        tolerance=12.0, giveUp=giveUp, cyclePeriod=0.01)

    assert result.success is False
    assert "gave up after" in result.reason
    assert result.sent > 0  # sends happened -- the robot simply never arrived
    assert result.waypointsReached == 0  # the projection never advances either
