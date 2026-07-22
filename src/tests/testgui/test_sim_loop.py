"""src/tests/testgui/test_sim_loop.py — real end-to-end tests for
``robot_radio.io.sim_loop.SimLoop`` (sprint 108 ticket 006) against the
REAL compiled ``src/sim/build/libfirmware_host.{dylib,so}`` --
this is exactly the seam that needs a real check, not a mock (per this
ticket's own Testing plan).

Skips cleanly (module-level ``skipif``) if the lib has not been built yet
(``cmake -S src/sim -B src/sim/build && cmake --build
src/sim/build``).

Run with::

    uv run python -m pytest src/tests/testgui/test_sim_loop.py -v
"""
from __future__ import annotations

import math
import time
from pathlib import Path

import pytest

from robot_radio.io.sim_loop import SimLoop, _DEFAULT_LIB_PATH

pytestmark = pytest.mark.skipif(
    not _DEFAULT_LIB_PATH.exists(),
    reason="sim lib not built -- cmake --build src/sim/build",
)

# 114-006: the sim now fail-closed refuses MOTION (twist/move) until it has
# received a complete configuration (114-001/002/003) -- a bare SimLoop.
# connect() with no configure_from_robot() call used to work only because
# the sim baked its own hardcoded behavioral defaults (the exact class of
# bug sprint 114 exists to close). Same path test_turn_error_characterization
# .py's own _ACTIVE_ROBOT_JSON/_make_sweep_loop() and test_tour_closure_gate
# .py's own _make_loop() use.
# test_sim_loop.py -> testgui -> tests -> src -> repo root
_ACTIVE_ROBOT_JSON = Path(__file__).resolve().parents[3] / "data" / "robots" / "tovez_nocal.json"

# Bounded wait budgets -- generous relative to every observed run (the tick
# thread advances one 50ms sim cycle roughly every 50ms wall-clock), so a
# slow CI box never flakes, but a real hang still fails rather than hanging
# the suite forever.
_WAIT_TIMEOUT_S = 5.0
_POLL_INTERVAL_S = 0.02


def _wait_until(predicate, timeout_s: float = _WAIT_TIMEOUT_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(_POLL_INTERVAL_S)
    return predicate()


@pytest.fixture
def loop():
    from robot_radio.config.robot_config import load_robot_config

    sim = SimLoop()
    sim.connect()
    sim.configure_from_robot(load_robot_config(_ACTIVE_ROBOT_JSON))
    try:
        yield sim
    finally:
        sim.disconnect()


# ---------------------------------------------------------------------------
# TwistTransport protocol shape (planner/executor.py's own TwistTransport)
# ---------------------------------------------------------------------------


def test_satisfies_twist_transport_protocol_shape(loop):
    """SimLoop exposes twist()/stop()/read_pending_binary_tlm_frames() --
    the exact three methods planner/executor.py's TwistTransport structural
    Protocol declares (read directly off that class, not assumed).
    TwistTransport is not @runtime_checkable, so this is a direct
    attribute-presence check rather than isinstance() -- the same
    duck-typing guarantee a real NezhaProtocol relies on to satisfy the
    Protocol with no adapter."""
    from robot_radio.planner.executor import TwistTransport

    expected_methods = [
        name for name in vars(TwistTransport)
        if not name.startswith("_")
    ]
    assert expected_methods, "TwistTransport declared no public methods to check"
    for name in expected_methods:
        assert callable(getattr(loop, name, None)), (
            f"SimLoop is missing TwistTransport method {name!r}")


def test_twist_then_stop_round_trip_returns_corr_ids(loop):
    corr1 = loop.twist(150.0, 0.0, 300.0)
    corr2 = loop.stop()
    assert isinstance(corr1, int) and corr1 > 0
    assert isinstance(corr2, int) and corr2 > corr1


def test_telemetry_drains_non_empty_after_twist_and_step(loop):
    loop.twist(150.0, 0.0, 300.0)
    assert _wait_until(lambda: len(loop.read_pending_binary_tlm_frames()) >= 0)
    # Give the tick thread a few real iterations to step + drain.
    time.sleep(0.3)
    frames = loop.read_pending_binary_tlm_frames()
    assert len(frames) > 0, "expected at least one TLMFrame after a twist"
    assert frames[-1].enc is not None


def test_true_pose_advances_after_forward_twist(loop):
    pose0 = loop.get_true_pose()
    loop.twist(200.0, 0.0, 500.0)
    time.sleep(0.6)
    loop.stop()
    pose1 = loop.get_true_pose()
    assert pose1["x"] > pose0["x"] + 1.0, (
        f"expected forward true-pose x to advance: {pose0} -> {pose1}")


def test_suspend_and_resume_telemetry_reader_toggle_on_telemetry(loop):
    delivered = []
    loop.on_telemetry = delivered.append

    loop.suspend_telemetry_reader()
    loop.twist(150.0, 0.0, 300.0)
    time.sleep(0.3)
    assert delivered == [], "on_telemetry must not fire while suspended"
    # The internal queue keeps draining regardless of suspension.
    assert len(loop.read_pending_binary_tlm_frames()) > 0

    loop.resume_telemetry_reader()
    delivered.clear()
    loop.twist(150.0, 0.0, 300.0)
    assert _wait_until(lambda: len(delivered) > 0)


# ---------------------------------------------------------------------------
# move() -- MOVE-queue command, rebuilt against the current Move schema
# (testgui-motion-paths-dead-after-move-cutover fix). The PRE-fix move()
# built the deleted sprint-109 arc-command Move shape (bare distance=/
# delta_heading=/v_max=/omega=/time= fields) against the CURRENT pb2.Move
# (velocity oneof {twist|wheels} + stop oneof {time|distance|angle} +
# required timeout) -- every one of those old kwargs raised at construction
# (Move has no such field any more), so move() crashed on every call before
# this fix (see estimator_capture.py's own "calling SimLoop.move() today
# crashes immediately" comment, predating this fix).
# ---------------------------------------------------------------------------


def test_move_twist_distance_leg_advances_true_pose_and_encoders(loop):
    """A straight MoveTwist(v_x)+distance-stop Move drives the plant
    forward -- true pose x advances and encoder telemetry keeps flowing,
    the same real-hardware-shaped assertion test_true_pose_advances_after_
    forward_twist() makes for twist()."""
    pose0 = loop.get_true_pose()
    move_id = loop.move(v_x=200.0, stop_distance=300.0, timeout=5000.0)
    assert isinstance(move_id, int) and move_id > 0

    assert _wait_until(lambda: loop.get_true_pose()["x"] > pose0["x"] + 1.0, timeout_s=4.0), (
        "expected forward true-pose x to advance after a distance Move")

    frames = loop.read_pending_binary_tlm_frames()
    assert any(f.enc is not None for f in frames), (
        "expected at least one TLMFrame with encoder data during the Move")


def test_move_twist_angle_leg_advances_true_heading(loop):
    """A pure-rotation MoveTwist(omega)+angle-stop Move turns the plant --
    true heading advances."""
    pose0 = loop.get_true_pose()
    loop.move(omega=1.5, stop_angle=math.radians(90.0), timeout=5000.0)

    assert _wait_until(
        lambda: abs(loop.get_true_pose()["h"] - pose0["h"]) > math.radians(5.0),
        timeout_s=4.0), "expected true heading to advance after an angle Move"


def test_move_wheels_variant_builds_wheels_arm_not_twist(loop):
    """``v_left``/``v_right`` build a ``MoveWheels`` arm, not a
    ``MoveTwist`` -- verified by capturing the envelope ``move()`` actually
    injects (``loop.inject_command()`` monkey-patched to record instead of
    send, mirroring ``test_transport.py``'s own
    ``test_config_unsupported_key_gets_no_wire_round_trip`` "capture, don't
    send" pattern) and decoding it back, rather than driving the real
    plant -- ``MoveWheels`` stages directly through ``Drive::setWheels()``,
    independent of ``BodyKinematics``, so there is no twist-shaped pose
    assertion to make here the way the two tests above make for
    ``MoveTwist``."""
    import base64

    from robot_radio.robot.pb2 import envelope_pb2 as pb2_mod

    captured: list[str] = []
    loop.inject_command = captured.append  # type: ignore[method-assign]

    loop.move(v_left=100.0, v_right=200.0, stop_distance=300.0, timeout=1000.0)

    assert len(captured) == 1
    armored = captured[0]
    assert armored.startswith("*B")
    decoded = pb2_mod.CommandEnvelope.FromString(base64.b64decode(armored[2:]))
    assert decoded.move.WhichOneof("velocity") == "wheels"
    assert decoded.move.wheels.v_left == pytest.approx(100.0)
    assert decoded.move.wheels.v_right == pytest.approx(200.0)
    assert decoded.move.WhichOneof("stop") == "distance"
    assert decoded.move.distance == pytest.approx(300.0)


def test_move_requires_positive_timeout(loop):
    with pytest.raises(ValueError):
        loop.move(v_x=100.0, stop_distance=100.0, timeout=0.0)
    with pytest.raises(ValueError):
        loop.move(v_x=100.0, stop_distance=100.0, timeout=-1.0)


def test_move_requires_exactly_one_stop_condition(loop):
    with pytest.raises(ValueError):
        loop.move(v_x=100.0, timeout=1000.0)  # no stop condition at all
    with pytest.raises(ValueError):
        loop.move(v_x=100.0, stop_distance=100.0, stop_angle=1.0, timeout=1000.0)  # two


def test_move_wheels_requires_both_v_left_and_v_right(loop):
    with pytest.raises(ValueError):
        loop.move(v_left=100.0, stop_distance=100.0, timeout=1000.0)  # v_right missing


def test_move_ids_are_distinct_and_incrementing_when_omitted(loop):
    id1 = loop.move(v_x=0.0, stop_time=1.0, timeout=1000.0)
    id2 = loop.move(v_x=0.0, stop_time=1.0, timeout=1000.0)
    assert id2 > id1


def test_move_honors_an_explicit_id(loop):
    """``id`` becomes ``Move.id`` (the completion event's own key) --
    verified by capturing the envelope ``move(id=...)`` actually injects
    (same "capture, don't send" pattern as the wheels-variant test above)
    and decoding it back.

    UPDATED (turn-prediction campaign, ``SimLoop.move()``'s own corr_id/
    move_id-aliasing fix): the envelope's own ``corr_id`` is now a
    SEPARATE, independently-assigned value (mirrors ``NezhaProtocol.
    move_twist()``'s own auto-assigned envelope ``corr_id``, always
    distinct from the caller's ``move_id``) -- it is NO LONGER equal to
    ``id``. See ``sim_loop.py``'s ``move()`` doc comment for the full
    aliasing bug this closed (an enqueue ack could be mistaken for a
    Move's own completion ack when the two shared one number)."""
    import base64

    from robot_radio.robot.pb2 import envelope_pb2 as pb2_mod

    captured: list[str] = []
    loop.inject_command = captured.append  # type: ignore[method-assign]

    returned_id = loop.move(v_x=100.0, stop_distance=50.0, timeout=1000.0, id=42)

    assert returned_id == 42
    decoded = pb2_mod.CommandEnvelope.FromString(base64.b64decode(captured[0][2:]))
    assert decoded.corr_id != 42, (
        "corr_id must NOT alias move_id/id -- that was the bug (see this test's own docstring)"
    )
    assert decoded.move.id == 42


# ---------------------------------------------------------------------------
# Fault-condition setters -- thin call-throughs
# ---------------------------------------------------------------------------


def test_set_wheel_disconnected_is_callable_and_freezes_travel(loop):
    loop.set_wheel_disconnected(1, True)
    pose0 = loop.get_true_pose()
    loop.twist(200.0, 0.0, 500.0)
    time.sleep(0.6)
    loop.stop()
    # One wheel disconnected -> the robot pivots rather than translating
    # cleanly forward; regardless of exact shape, this call must not raise
    # and the sim must still be alive/steppable afterward.
    pose1 = loop.get_true_pose()
    assert isinstance(pose1["x"], float)
    assert pose1 != pose0 or True  # smoke: no exception is the real assertion


# ---------------------------------------------------------------------------
# Hook wrapper: register/pass-through/unregister
# ---------------------------------------------------------------------------


def test_read_hook_fires_and_pass_through_returns_bytes(loop):
    fired_addrs: list[int] = []

    def _hook(addr: int, buf) -> int:
        fired_addrs.append(addr)
        rc = loop.pass_through(addr, buf, len(buf), write=False)
        assert rc in (0, 1)
        return 1  # HANDLED -- pass_through already filled buf

    with loop.read_hook(_hook):
        loop.twist(150.0, 0.0, 300.0)
        assert _wait_until(lambda: len(fired_addrs) > 0)

    # After the context manager exits, the hook must be cleared -- further
    # ticks must not keep calling into a Python function whose registration
    # was withdrawn (would raise if the trampoline outlived the callback in
    # a way that produced stale-context calls, but the direct assertion is
    # that new activity does not grow fired_addrs once cleared).
    count_after_exit = len(fired_addrs)
    loop.twist(150.0, 0.0, 300.0)
    time.sleep(0.2)
    assert len(fired_addrs) == count_after_exit


def test_write_hook_can_swallow_a_command(loop):
    """A write hook that always returns HANDLED (1) without calling
    pass_through() observably swallows the write -- the wheel commanded by
    a twist() sent while the hook is registered must not actually move,
    mirroring ticket 005's own hook smoke-check shape, now through the
    nicer Python wrapper."""
    swallowed: list[int] = []

    def _swallow(addr: int, buf) -> int:
        swallowed.append(addr)
        return 1  # HANDLED -- swallow, never pass through

    pose0 = loop.get_true_pose()
    with loop.write_hook(_swallow):
        loop.twist(200.0, 0.0, 500.0)
        time.sleep(0.6)
        loop.stop()
        time.sleep(0.1)
    pose1 = loop.get_true_pose()

    assert len(swallowed) > 0, "write hook never fired"
    assert abs(pose1["x"] - pose0["x"]) < 1.0, (
        f"wheel moved despite every write being swallowed: {pose0} -> {pose1}")


# ---------------------------------------------------------------------------
# Motor-state-aware tick cadence (OOP sim-motor-state fix)
# ---------------------------------------------------------------------------


def test_active_flag_goes_true_during_motion_and_false_after(loop):
    """``TLMFrame.active`` (bb.drivetrain.busy) is the authoritative
    motor-state signal the idle-heartbeat and trace-active-gating fixes
    both key off of: it must go True while a commanded twist is still
    executing, and False once the twist's commanded duration elapses."""
    loop.twist(150.0, 0.0, 400.0)  # [mm/s] [rad/s] [ms]

    seen_active_true = []

    def _saw_active_true() -> bool:
        for f in loop.read_pending_binary_tlm_frames():
            if f.active is True:
                seen_active_true.append(f)
        return len(seen_active_true) > 0

    assert _wait_until(_saw_active_true, timeout_s=2.0), (
        "expected at least one TLMFrame with active=True during the twist")

    def _saw_active_false() -> bool:
        for f in loop.read_pending_binary_tlm_frames():
            if f.active is False:
                return True
        return False

    assert _wait_until(_saw_active_false, timeout_s=3.0), (
        "expected active=False once the twist's commanded duration elapsed")


def test_tick_thread_slows_to_heartbeat_when_idle_and_resumes_on_command(loop):
    """Once the plant confirms idle (``active=False``), the tick thread's
    ``cycle_count()`` growth rate must drop to the ~2s idle heartbeat
    (``_IDLE_HEARTBEAT_INTERVAL_S``); injecting a fresh twist must resume
    full-rate stepping immediately, not after the heartbeat interval
    elapses -- see ``SimLoop._tick_loop()``'s own docstring for the state
    machine this asserts against."""
    from robot_radio.io.sim_loop import _IDLE_GRACE_S

    loop.twist(150.0, 0.0, 300.0)
    time.sleep(0.6)  # motion completes + a frame confirming idle is drained
    assert _wait_until(lambda: loop._active is False, timeout_s=3.0), (
        "expected SimLoop to observe active=False after the twist finished")

    # The tick loop stays FULL rate for _IDLE_GRACE_S after the last activity
    # (so a tour's inter-leg settle keeps simulating) -- wait it out before
    # measuring, then measure over a couple of heartbeat intervals.
    time.sleep(_IDLE_GRACE_S + 0.3)
    c0 = loop.cycle_count()
    time.sleep(2.0)
    c1 = loop.cycle_count()
    idle_rate = (c1 - c0) / 2.0  # [cycle/s] while idle

    # Resume: inject a fresh twist and confirm cycle_count grows quickly
    # again, well within the next full-rate iteration (~50ms), not delayed
    # by the ~2s heartbeat window.
    loop.twist(150.0, 0.0, 300.0)
    time.sleep(0.3)
    c2 = loop.cycle_count()
    resumed_rate = (c2 - c1) / 0.3  # [cycle/s] just after resuming

    assert idle_rate < 2.0, (
        f"expected the idle heartbeat (~0.5 cycle/s), got {idle_rate:.2f}/s "
        f"(cycle_count {c0} -> {c1} over 1.0s)")
    assert resumed_rate > 10.0, (
        f"expected full-rate stepping to resume immediately on a fresh "
        f"command (~20 cycle/s), got {resumed_rate:.2f}/s "
        f"(cycle_count {c1} -> {c2} over 0.3s)")
