"""src/tests/unit/test_planner.py -- ticket 127-006.

Pure, synthetic-input unit tests for ``robot_radio.pathplan.planner`` --
the throttled-replacement decision, the give-up/termination logic, the
``Move.id`` allocator, and ``gotoRobot()``'s composition through
``gotoWorld()`` (design issue T5). No hardware, no sim binary, no camera
daemon, no serial port -- the full end-to-end control loop (telemetry in,
``move_twist()`` out) is exercised separately at the sim tier
(``src/tests/sim/test_pathplan_goto_convergence.py``, real ``SimLoop``),
per this ticket's own Testing section.

Covers (ticket acceptance criteria):
  1. ``gotoRobot()`` composes through ``gotoWorld()`` -- exactly one
     control-loop implementation, confirmed by mocking ``gotoWorld()``
     and checking it is called exactly once with the correctly
     world-transformed target.
  2. Throttled replacement: ``_shouldReplace()`` sends materially
     different solutions and suppresses unchanged ones; a synthetic
     multi-cycle sequence shows fewer sends than "send every cycle
     unconditionally".
  3. The termination-tolerance/give-up constant
     (``TERMINATION_TOLERANCE``) is isolated at one PROVISIONAL,
     commented site.
  4. Give-up is explicit and reachable: ``_giveUpReason()`` (iteration
     and timeout caps) and ``_targetBehindReason()`` (the solver's
     target-behind guard) both always report WHY.
  5. ``MoveIdAllocator`` is strictly monotonic, never emits 0.
  6. ``_advance()`` checks the geofence INSIDE its own time-advance
     window, not between segments.
  7. ``followPath()`` (out-of-process, 2026-07-30, this IS ticket 008's own
     algorithm): a straight multi-waypoint run reaches every waypoint via
     pass-through and succeeds at the terminal one; a target that is
     permanently behind and NEVER sent anything still terminates cleanly
     via ``GiveUpLimits`` (not a bearing-snapshot verdict -- two earlier
     approaches that used one were tried and rejected, see the module-level
     comment above ``followPath()``'s own solve call); a target that goes
     behind AFTER a real send exists gets that same last-known-good arc
     RESENT while genuinely stalled, rather than leaving the vehicle idle
     for the rest of the give-up budget (the measured regression that fix
     addresses). ``_lookaheadFloorFor()``'s own derivation is locked
     against regression.
"""

from __future__ import annotations

import dataclasses
import math
import time

import pytest

from robot_radio.nav.pose import Pose
from robot_radio.pathplan.solver import ArcSolution, SolverLimits
from robot_radio.pathplan.world_pose import WorldPose
from robot_radio.robot.protocol import AckEntry, EncoderReading, TLMFrame

import robot_radio.pathplan.planner as planner_mod
from robot_radio.pathplan.planner import (
    TERMINATION_TOLERANCE,
    AckRetry,
    FollowPathResult,
    GiveUpLimits,
    GotoResult,
    MoveIdAllocator,
    MoveRejected,
    ProgressCheck,
    ReplaceThreshold,
    _advance,
    _ERR_FULL,
    _giveUpReason,
    _lookaheadFloorFor,
    _moveTimeoutFor,
    _readFrames,
    _recordAcks,
    _sendVerifiedTwist,
    _shouldReplace,
    _targetBehindReason,
    followPath,
    gotoRobot,
    gotoWorld,
)


def _solution(omega: float, arcLength: float, stop: bool = False) -> ArcSolution:
    return ArcSolution(v_x=150.0, omega=omega, arcLength=arcLength, stop=stop)


# ---------------------------------------------------------------------------
# 1. TERMINATION_TOLERANCE is the one, isolated, provisional constant.
# ---------------------------------------------------------------------------


def test_termination_tolerance_is_provisional_and_positive():
    # Locks the constant's existence/positivity as a regression check --
    # the "isolated at one named, commented site" property is structural
    # (there is exactly one definition, in this module) and is checked by
    # code inspection/grep per the ticket's own acceptance criterion, not
    # re-derived as a runtime assertion here.
    assert TERMINATION_TOLERANCE > 0.0
    # Chosen from the design issue's actuation-delay analysis (>=100 mm
    # carrot distance at ~150 ms actuation delay) -- not an arbitrary guess.
    assert TERMINATION_TOLERANCE == 100.0


# ---------------------------------------------------------------------------
# 2. Throttled replacement -- _shouldReplace().
# ---------------------------------------------------------------------------


def test_should_replace_true_on_first_solve():
    assert _shouldReplace(None, _solution(0.1, 200.0), 0.05, 15.0) is True


def test_should_replace_false_when_unchanged():
    lastSent = _solution(0.1, 200.0)
    candidate = _solution(0.1, 200.0)
    assert _shouldReplace(lastSent, candidate, 0.05, 15.0) is False


def test_should_replace_false_when_change_below_both_thresholds():
    lastSent = _solution(0.10, 200.0)
    candidate = _solution(0.12, 205.0)  # +0.02 rad/s, +5 mm -- both under threshold
    assert _shouldReplace(lastSent, candidate, 0.05, 15.0) is False


def test_should_replace_true_when_omega_changes_beyond_threshold():
    lastSent = _solution(0.10, 200.0)
    candidate = _solution(0.20, 200.0)  # +0.10 rad/s > 0.05 threshold
    assert _shouldReplace(lastSent, candidate, 0.05, 15.0) is True


def test_should_replace_true_when_arc_length_changes_beyond_threshold():
    lastSent = _solution(0.10, 200.0)
    candidate = _solution(0.10, 220.0)  # +20 mm > 15 mm threshold
    assert _shouldReplace(lastSent, candidate, 0.05, 15.0) is True


def test_replacement_rate_drops_when_solution_is_not_moving_materially():
    """The ticket's own acceptance criterion, directly: a sequence with
    several materially-unchanged cycles sends far fewer replacements than
    sending unconditionally on every cycle."""
    sequence = [
        _solution(0.30, 500.0),   # 1: first solve -- always sent
        _solution(0.30, 500.0),   # 2: identical -- suppressed
        _solution(0.30, 498.0),   # 3: tiny drift -- suppressed
        _solution(0.30, 495.0),   # 4: tiny drift -- suppressed
        _solution(0.05, 300.0),   # 5: real change -- sent
        _solution(0.05, 299.0),   # 6: tiny drift -- suppressed
        _solution(0.05, 298.0),   # 7: tiny drift -- suppressed
        _solution(0.00, 100.0),   # 8: real change -- sent
    ]
    omegaThreshold, arcLengthThreshold = 0.05, 15.0

    throttledSends = 0
    lastSent = None
    for candidate in sequence:
        if _shouldReplace(lastSent, candidate, omegaThreshold, arcLengthThreshold):
            throttledSends += 1
            lastSent = candidate

    unconditionalSends = len(sequence)  # "send every cycle" baseline

    assert throttledSends == 3
    assert throttledSends < unconditionalSends


# ---------------------------------------------------------------------------
# 3. Termination/give-up logic -- _giveUpReason() / _targetBehindReason().
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


def test_target_behind_reason_reports_the_bearing_explicitly():
    reason = _targetBehindReason(math.radians(150.0))
    assert "behind" in reason
    assert "150.0" in reason
    assert "out of scope" in reason


# ---------------------------------------------------------------------------
# 4. MoveIdAllocator -- strictly monotonic, never 0.
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
# 5. gotoRobot() composes through gotoWorld() -- one implementation.
# ---------------------------------------------------------------------------


def _tlmFrame(x_mm: float, y_mm: float, heading_cdeg: float) -> TLMFrame:
    """A synthetic, hand-decoded TLMFrame at REST (zero twist, so
    WorldPose's frame-age extrapolation is a no-op regardless of real
    wall-clock elapsed time -- deterministic, matches
    test_world_pose.py's own convention)."""
    return TLMFrame(
        t=1000, pose=(x_mm, y_mm, heading_cdeg), twist=(0, 0), recvTime=time.monotonic(),
        enc_left=EncoderReading(position=0.0, velocity=0.0, age=0, position_epoch=0),
        enc_right=EncoderReading(position=0.0, velocity=0.0, age=0, position_epoch=0),
    )


def test_goto_robot_composes_through_goto_world(monkeypatch):
    worldPose = WorldPose()
    # Current world pose: (100 cm, 50 cm), heading +90 deg (east=0, CCW+).
    worldPose.ingest(_tlmFrame(x_mm=1000.0, y_mm=500.0, heading_cdeg=9000.0))

    captured = {}

    def _fakeGotoWorld(proto, wp, x, y, theta=None, **kwargs):
        captured["proto"] = proto
        captured["worldPose"] = wp
        captured["x"] = x
        captured["y"] = y
        captured["theta"] = theta
        captured["kwargs"] = kwargs
        return GotoResult(success=True, reason="stub", finalPose=None, iterations=0, sent=0)

    monkeypatch.setattr(planner_mod, "gotoWorld", _fakeGotoWorld)

    sentinelProto = object()
    sentinelLimits = object()
    result = gotoRobot(sentinelProto, worldPose, x=10.0, y=0.0, limits=sentinelLimits)

    assert result.success is True  # the stub's own result, proving gotoRobot returned it verbatim
    assert captured["proto"] is sentinelProto
    assert captured["worldPose"] is worldPose
    # Robot-frame (10, 0) rotated by +90 deg heading and translated by
    # (100, 50) -> (100, 60) -- hand-verified tangent-circle-free rotation.
    assert math.isclose(captured["x"], 100.0, abs_tol=1e-9)
    assert math.isclose(captured["y"], 60.0, abs_tol=1e-9)
    assert captured["theta"] is None
    assert captured["kwargs"]["limits"] is sentinelLimits


def test_goto_robot_raises_without_a_world_pose_yet():
    worldPose = WorldPose()  # never ingest()-ed -- no current pose
    with pytest.raises(RuntimeError):
        gotoRobot(object(), worldPose, x=10.0, y=0.0, limits=object())


# ---------------------------------------------------------------------------
# 6. _readFrames() dispatches to whichever backend proto._conn exposes.
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
# 7. _advance() checks the geofence INSIDE its own time-advance window.
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
# 8. OOP fix (2026-07-30): _moveTimeoutFor() -- per-move timeout derivation.
# ---------------------------------------------------------------------------


def test_move_timeout_scales_with_arc_length():
    short = _moveTimeoutFor(arcLength=100.0, speed=150.0)
    long = _moveTimeoutFor(arcLength=2000.0, speed=150.0)
    assert long > short


def test_move_timeout_has_a_floor_for_a_tiny_move():
    # A near-zero arcLength still gets a floor big enough to cover an
    # actual accel/decel ramp and one ack round trip, never ~0ms.
    timeout = _moveTimeoutFor(arcLength=1.0, speed=150.0)
    assert timeout >= 3000.0


def test_move_timeout_is_capped_for_a_huge_arc_length():
    # A pathological/huge arcLength (e.g. an unreachable, far-off-field
    # solve) must not be able to request an enormous timeout that would
    # itself blow past this loop's own give-up budget.
    timeout = _moveTimeoutFor(arcLength=1_000_000.0, speed=150.0)
    assert timeout == planner_mod._MOVE_TIMEOUT_CAP


def test_move_timeout_guards_against_zero_speed():
    # A misconfigured/zero speed must not raise ZeroDivisionError.
    timeout = _moveTimeoutFor(arcLength=250.0, speed=0.0)
    assert timeout > 0.0


# ---------------------------------------------------------------------------
# 9. OOP fix: _sendVerifiedTwist() -- ack verification and retry.
# ---------------------------------------------------------------------------


class _FakeAckFrame:
    """The minimal frame shape `_sendVerifiedTwist()`/`_recordAcks()` need
    -- just `.acks`, no pose/encoder fields (this test drives the ack
    state machine in isolation from `WorldPose`/pose tracking, which
    `test_progress_check_forces_resend_when_robot_is_stalled()` below
    covers separately via full `TLMFrame`s)."""

    def __init__(self, acks):
        self.acks = acks


class _NullWorldPose:
    """A no-op stand-in for `WorldPose` -- `_sendVerifiedTwist()` calls
    `.ingest()` on every frame it drains; these tests don't care about
    pose tracking, only about the ack state machine."""

    def ingest(self, frame):
        pass


class _AckScheduleProto:
    """Fakes `NezhaProtocol` for `_sendVerifiedTwist()` tests: each
    `move_twist()` call is one "attempt" (1-based); `ackByAttempt` maps
    attempt number -> `(ok, err_code)` to hand back (as an `AckEntry`
    matching that attempt's own assigned corr_id) on every subsequent
    `read_pending_binary_tlm_frames()` poll, until superseded by the next
    attempt's own send. An attempt number absent from `ackByAttempt`
    never acks at all within its window -- exercises the retry-on-no-ack
    path distinctly from the retry-on-ERR_FULL path."""

    _conn = None

    def __init__(self, ackByAttempt: dict):
        self._ackByAttempt = ackByAttempt
        self._attempt = 0
        self._nextCorr = 100
        self._currentCorr = None
        self.moveIds = []

    def move_twist(self, *, move_id, **kwargs):
        self._attempt += 1
        self.moveIds.append(move_id)
        self._currentCorr = self._nextCorr
        self._nextCorr += 1
        return self._currentCorr

    def read_pending_binary_tlm_frames(self):
        spec = self._ackByAttempt.get(self._attempt)
        if spec is None:
            return []
        ok, errCode = spec
        return [_FakeAckFrame([AckEntry(corr_id=self._currentCorr, ok=ok, err_code=errCode)])]


def _verify(proto, *, moveId=1, maxAttempts=4, ackTimeout=0.03, cyclePeriod=0.01):
    ackSeen: "dict[int, object]" = {}
    return _sendVerifiedTwist(
        proto, _NullWorldPose(), None, ackSeen, moveId=moveId,
        kwargs=dict(v_x=150.0, v_y=0.0, omega=0.0, stop_distance=250.0, timeout=3000.0),
        cyclePeriod=cyclePeriod, ackRetry=AckRetry(maxAttempts=maxAttempts, ackTimeout=ackTimeout))


def test_send_verified_twist_acked_on_first_attempt():
    proto = _AckScheduleProto({1: (True, 0)})
    acked, attempts = _verify(proto)
    assert acked is True
    assert attempts == 1
    assert proto.moveIds == [1]  # exactly one wire attempt -- no retry needed


def test_send_verified_twist_retries_on_lost_ack_reusing_the_same_move_id():
    # Attempt 1 never acks at all (simulates a dropped inbound packet);
    # attempt 2 acks ok.
    proto = _AckScheduleProto({2: (True, 0)})
    acked, attempts = _verify(proto, moveId=7)
    assert acked is True
    assert attempts == 2
    # Both wire attempts of ONE logical send reuse the SAME Move.id -- the
    # firmware's dedup ring is what makes that retry idempotent
    # (RobotLoop::handleMove()'s own documented contract).
    assert proto.moveIds == [7, 7]


def test_send_verified_twist_retries_on_err_full():
    proto = _AckScheduleProto({1: (False, _ERR_FULL), 2: (True, 0)})
    acked, attempts = _verify(proto)
    assert acked is True
    assert attempts == 2


def test_send_verified_twist_raises_on_non_retryable_err():
    _ERR_BADARG = 2  # envelope.proto ErrCode.ERR_BADARG
    proto = _AckScheduleProto({1: (False, _ERR_BADARG)})
    with pytest.raises(MoveRejected):
        _verify(proto)
    # A non-retryable NACK fails LOUDLY on the first bad ack -- it must
    # not burn the rest of the retry budget on a command that can never
    # succeed.
    assert proto.moveIds == [1]


def test_send_verified_twist_exhausts_retries_and_reports_unacked():
    proto = _AckScheduleProto({})  # never acks at all
    acked, attempts = _verify(proto, maxAttempts=3)
    assert acked is False
    assert attempts == 3
    assert proto.moveIds == [1, 1, 1]  # every retry of ONE send reuses the id


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
# 10. OOP fix: gotoWorld() integration -- ProgressCheck forces a resend
#     when the robot is stalled even though every send is acked ok.
# ---------------------------------------------------------------------------


class _StallFakeProto:
    """A `NezhaProtocol`-shaped fake for `gotoWorld()` integration tests:
    the world pose it reports NEVER MOVES (simulating a stalled/deadband
    move, or a genuinely stuck robot) while every `move_twist()` send is
    acked OK immediately -- isolating `ProgressCheck`'s own forced-resend
    behavior from `AckRetry`'s (covered directly above). This is the
    exact shape of the OOP ticket's own measured failure: commands land
    and ack fine, the robot just never moves, and the old throttle-only
    loop sent exactly once over 264 iterations because the (unmoving)
    solution never looked "materially different" from the last one sent.
    """

    _conn = None  # no connection-level reader -- _readFrames() falls back to this object's own

    def __init__(self, x_mm: float, y_mm: float, heading_cdeg: float):
        self._x_mm = x_mm
        self._y_mm = y_mm
        self._heading_cdeg = heading_cdeg
        self._nextCorr = 1
        self._pendingAck = None
        self.moveIds: "list[int]" = []

    def move_twist(self, *, move_id, **kwargs):
        corr = self._nextCorr
        self._nextCorr += 1
        self.moveIds.append(move_id)
        self._pendingAck = AckEntry(corr_id=corr, ok=True, err_code=0)
        return corr

    def read_pending_binary_tlm_frames(self):
        acks = []
        if self._pendingAck is not None:
            acks = [self._pendingAck]
            self._pendingAck = None
        return [TLMFrame(
            t=1000, pose=(self._x_mm, self._y_mm, self._heading_cdeg), twist=(0, 0),
            recvTime=time.monotonic(),
            enc_left=EncoderReading(position=0.0, velocity=0.0, age=0, position_epoch=0),
            enc_right=EncoderReading(position=0.0, velocity=0.0, age=0, position_epoch=0),
            acks=acks)]

    def estop(self):
        pass


def test_progress_check_forces_resend_when_robot_is_stalled():
    worldPose = WorldPose()
    proto = _StallFakeProto(x_mm=0.0, y_mm=0.0, heading_cdeg=0.0)
    limits = SolverLimits(trackWidth=120.0, speed=150.0)
    # A target far enough away that TERMINATION_TOLERANCE is never met
    # (the robot never moves in this fake), so the loop runs until
    # giveUp.giveUpTimeout -- short here so the test stays fast.
    giveUp = GiveUpLimits(maxIterations=100_000, giveUpTimeout=0.25)
    progress = ProgressCheck(window=0.05, threshold=20.0)
    ackRetry = AckRetry(maxAttempts=1, ackTimeout=0.02)

    result = gotoWorld(
        proto, worldPose, x=100.0, y=0.0,  # [cm] -- 1000 mm straight ahead
        limits=limits, geofence=None, tolerance=50.0,
        giveUp=giveUp, progress=progress, ackRetry=ackRetry, cyclePeriod=0.01)

    assert result.success is False
    # Every send WAS acked -- this is purely a liveness stall, not a link
    # problem, and the result must say so distinctly.
    assert result.unacked == 0
    assert "did not converge" in result.reason
    # The throttle alone (unchanged solution every cycle, since the pose
    # never moves) would have sent exactly once -- ProgressCheck must have
    # forced at least one more.
    assert result.forcedResends >= 1
    assert result.sent >= 2
    # Every logical send -- throttle-triggered OR forced -- drew a FRESH
    # Move.id (never reused across DIFFERENT logical sends; only WITHIN
    # one send's own ack-loss retries, covered separately above).
    assert len(set(proto.moveIds)) == len(proto.moveIds)


# ---------------------------------------------------------------------------
# 11. followPath() -- pure-pursuit multi-waypoint loop (out-of-process,
#     2026-07-30, this IS ticket 008's own algorithm). The advance rule's
#     own geometry (hasPassedWaypoint()/advanceWaypointIndex()) is unit-
#     tested standalone in test_solver.py; these tests exercise followPath()
#     ITSELF -- the ingest/advance/solve ordering, the terminal-only arrival
#     test, and what happens when the target-behind guard fires: nothing is
#     sent while a robot that never sent anything stays stuck (give up via
#     GiveUpLimits), and the last-known-good arc gets resent once genuinely
#     stalled if one exists. See the module-level comment above
#     followPath()'s own solve call (planner.py) for the two REJECTED
#     approaches (an unbounded then a bounded "treat behind as passed"
#     skip, and a widened guard angle) this design replaced, and why.
# ---------------------------------------------------------------------------


def _cm(mm: float) -> float:
    return mm / 10.0


def test_lookahead_floor_derivation():
    # 150 mm/s * 0.15 s actuation delay == 22.5 mm -- see the module-level
    # comment above _ACTUATION_DELAY for the derivation (matches
    # hil_drive.py's own actuationDelay = 150.0 ms).
    assert _lookaheadFloorFor(150.0) == pytest.approx(22.5)


def test_follow_path_result_has_waypoints_reached_field():
    fieldNames = {f.name for f in dataclasses.fields(FollowPathResult)}
    assert fieldNames == {"success", "reason", "finalPose", "iterations", "sent",
                          "waypointsReached", "retries", "forcedResends", "unacked"}


class _MovingFakeProto:
    """`NezhaProtocol`-shaped fake for `followPath()` integration tests: the
    world pose it reports advances in a STRAIGHT LINE (+x) by `stepMm`
    every `read_pending_binary_tlm_frames()` call, simulating a robot that
    tracks a commanded on-heading (omega=0) twist perfectly. Enough to
    exercise `followPath()`'s own ingest/advance/solve loop, ack
    bookkeeping, and terminal arrival test end to end without real
    differential-drive kinematics -- the geometry of the advance rule
    itself is already covered standalone by test_solver.py's own
    `hasPassedWaypoint()`/`advanceWaypointIndex()` tests."""

    _conn = None

    def __init__(self, stepMm: float = 5.0) -> None:
        self._x_mm = 0.0
        self._stepMm = stepMm
        self._nextCorr = 1
        self._pendingAck = None
        self.moveIds: "list[int]" = []
        self.estopped = False

    def move_twist(self, *, move_id, **kwargs):
        corr = self._nextCorr
        self._nextCorr += 1
        self.moveIds.append(move_id)
        self._pendingAck = AckEntry(corr_id=corr, ok=True, err_code=0)
        return corr

    def read_pending_binary_tlm_frames(self):
        self._x_mm += self._stepMm
        acks = []
        if self._pendingAck is not None:
            acks = [self._pendingAck]
            self._pendingAck = None
        return [TLMFrame(
            t=1000, pose=(self._x_mm, 0.0, 0), twist=(0, 0),
            recvTime=time.monotonic(),
            enc_left=EncoderReading(position=0.0, velocity=0.0, age=0, position_epoch=0),
            enc_right=EncoderReading(position=0.0, velocity=0.0, age=0, position_epoch=0),
            acks=acks)]

    def estop(self):
        self.estopped = True


def test_follow_path_straight_line_reaches_every_waypoint_via_pass_through():
    worldPose = WorldPose()
    proto = _MovingFakeProto(stepMm=5.0)
    limits = SolverLimits(trackWidth=120.0, speed=150.0)
    waypoints = [Pose(x=_cm(mm), y=0.0, heading=0.0) for mm in (30.0, 60.0, 90.0, 120.0)]
    crossed: "list[int]" = []

    result = followPath(
        proto, worldPose, waypoints, limits, geofence=None, tolerance=12.0,
        giveUp=GiveUpLimits(maxIterations=500, giveUpTimeout=5.0), cyclePeriod=0.01,
        onWaypoint=lambda index, pose: crossed.append(index))

    assert result.success is True
    assert result.waypointsReached == len(waypoints)
    # Every waypoint crossed EXACTLY once, in strictly increasing order --
    # the pass-through advance rule's own ordering guarantee (ingest pose
    # -> advance target if passed -> solve, never solve-then-advance).
    assert crossed == list(range(len(waypoints)))
    # estop(), never the planned stop() -- this loop's own finally block,
    # run on every return.
    assert proto.estopped is True


def test_follow_path_gives_up_via_giveup_limits_when_never_sent_anything():
    """When the target-behind guard fires and NO previous solution exists
    to fall back on (a robot that never managed a single real send), this
    loop sends nothing at all, every cycle, forever -- so it must still
    terminate via `GiveUpLimits`, never via a bearing-snapshot verdict (two
    earlier approaches used one and were rejected; see the module-level
    comment above `followPath()`'s own solve call in planner.py). A
    stationary robot (`_StallFakeProto`, reused from the ProgressCheck
    tests above) facing WEST (heading 180 deg) with both waypoints due
    EAST never moves, so `advanceWaypointIndex()` cannot pass-through past
    waypoint 0 either -- this isolates the "nothing to fall back on" give-up
    path from the resend path (covered separately below)."""
    worldPose = WorldPose()
    proto = _StallFakeProto(x_mm=0.0, y_mm=0.0, heading_cdeg=18000.0)  # facing west
    limits = SolverLimits(trackWidth=120.0, speed=150.0)
    waypoints = [Pose(x=_cm(50.0), y=0.0, heading=0.0), Pose(x=_cm(100.0), y=0.0, heading=0.0)]
    giveUp = GiveUpLimits(maxIterations=20, giveUpTimeout=5.0)

    result = followPath(proto, worldPose, waypoints, limits, geofence=None,
                        tolerance=12.0, giveUp=giveUp, cyclePeriod=0.01)

    assert result.success is False
    assert "gave up after" in result.reason
    assert "no move was ever sent" in result.reason
    assert result.sent == 0
    # index never advances (the robot never moves, and hasPassedWaypoint()'s
    # own projection test never fires) -- reachedIndex stays at its initial
    # sentinel the whole call.
    assert result.waypointsReached == 0


class _BehindAfterFakeProto:
    """`NezhaProtocol`-shaped fake for the "resend the last-known-good arc"
    recovery test: reports heading EAST (0 deg) for the first
    `switchAfter` telemetry reads -- long enough for `followPath()` to
    solve and send a real arc toward an east-facing target -- then reports
    heading WEST (180 deg) forever after, so the SAME target becomes
    permanently behind. Position is held fixed throughout (this test only
    cares whether a resend happens, not whether the path is completed)."""

    _conn = None

    def __init__(self, switchAfter: int = 2) -> None:
        self._reads = 0
        self._switchAfter = switchAfter
        self._nextCorr = 1
        self._pendingAck = None
        self.moveIds: "list[int]" = []

    def move_twist(self, *, move_id, **kwargs):
        corr = self._nextCorr
        self._nextCorr += 1
        self.moveIds.append(move_id)
        self._pendingAck = AckEntry(corr_id=corr, ok=True, err_code=0)
        return corr

    def read_pending_binary_tlm_frames(self):
        self._reads += 1
        headingCdeg = 0.0 if self._reads <= self._switchAfter else 18000.0
        acks = []
        if self._pendingAck is not None:
            acks = [self._pendingAck]
            self._pendingAck = None
        return [TLMFrame(
            t=1000, pose=(0.0, 0.0, headingCdeg), twist=(0, 0),
            recvTime=time.monotonic(),
            enc_left=EncoderReading(position=0.0, velocity=0.0, age=0, position_epoch=0),
            enc_right=EncoderReading(position=0.0, velocity=0.0, age=0, position_epoch=0),
            acks=acks)]

    def estop(self):
        pass


def test_follow_path_resends_last_known_good_arc_when_stalled_and_behind():
    """The recovery path this fix adds (module comment above followPath()'s
    own solve call): a Distance-stopped Move is bounded and completes on
    its own, so sending NOTHING while the guard fires would eventually
    leave the vehicle idle, heading frozen, bearing never improving, for
    the rest of the give-up budget -- measured 2026-07-30. Once
    `ProgressCheck` says the robot has genuinely stalled, and a previous
    solution exists, that EXACT arc must be resent (fresh Move.id, same
    v_x/omega/arcLength) rather than nothing."""
    worldPose = WorldPose()
    proto = _BehindAfterFakeProto(switchAfter=2)
    limits = SolverLimits(trackWidth=120.0, speed=150.0)
    waypoints = [Pose(x=_cm(500.0), y=0.0, heading=0.0), Pose(x=_cm(1000.0), y=0.0, heading=0.0)]
    giveUp = GiveUpLimits(maxIterations=60, giveUpTimeout=3.0)
    progress = ProgressCheck(window=0.05, threshold=1000.0)  # position never changes -> stalls fast

    result = followPath(proto, worldPose, waypoints, limits, geofence=None,
                        tolerance=12.0, giveUp=giveUp, progress=progress, cyclePeriod=0.01)

    assert result.success is False
    # At least one real send (while heading was still east) PLUS at least
    # one forced resend of that SAME arc once heading flipped and the
    # guard started firing -- proving the recovery path actually sends
    # something instead of leaving the vehicle idle.
    assert result.sent >= 2
    assert result.forcedResends >= 1
    assert len(set(proto.moveIds)) == len(proto.moveIds)  # every send still gets a fresh id
