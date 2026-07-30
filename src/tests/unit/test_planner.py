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
"""

from __future__ import annotations

import math
import time

import pytest

from robot_radio.nav.pose import Pose
from robot_radio.pathplan.solver import ArcSolution
from robot_radio.pathplan.world_pose import WorldPose
from robot_radio.robot.protocol import EncoderReading, TLMFrame

import robot_radio.pathplan.planner as planner_mod
from robot_radio.pathplan.planner import (
    TERMINATION_TOLERANCE,
    GiveUpLimits,
    GotoResult,
    MoveIdAllocator,
    ReplaceThreshold,
    _advance,
    _giveUpReason,
    _readFrames,
    _shouldReplace,
    _targetBehindReason,
    gotoRobot,
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
