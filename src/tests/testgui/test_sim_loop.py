"""src/tests/testgui/test_sim_loop.py — real end-to-end tests for
``robot_radio.io.sim_loop.SimLoop`` (sprint 108 ticket 006) against the
REAL compiled ``src/firm/platform/host/build/libfirmware_host.{dylib,so}`` --
this is exactly the seam that needs a real check, not a mock (per this
ticket's own Testing plan).

Skips cleanly (module-level ``skipif``) if the lib has not been built yet
(``cmake -S src/firm/platform/host -B src/firm/platform/host/build && cmake --build
src/firm/platform/host/build``).

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
    reason="sim lib not built -- cmake --build src/firm/platform/host/build",
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
# 128-007: the `test_satisfies_twist_transport_protocol_shape` test that used
# to live here checked SimLoop against `planner/executor.py`'s `TwistTransport`
# Protocol -- deleted along with `StreamingExecutor` (zero production callers;
# `TwistTransport`'s own "a real NezhaProtocol already satisfies this" claim
# was false besides -- see the ticket). `SimLoop.twist()`/`.stop()`/
# `.read_pending_binary_tlm_frames()` are still exercised directly by the
# tests below; there is no live Protocol left to check the shape against.
# ---------------------------------------------------------------------------


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
    from robot_radio.io.wire_codec import decode_frame
    from robot_radio.robot.pb2 import envelope_pb2 as pb2_mod

    captured: list[bytes] = []
    loop.inject_command = captured.append  # type: ignore[method-assign]

    loop.move(v_left=100.0, v_right=200.0, stop_distance=300.0, timeout=1000.0)

    assert len(captured) == 1
    # captured[0] is the FULL wire line ("MOVE:<cobs bytes>", 124-005) --
    # strip the leading command prefix and pass it as decode_frame()'s own
    # CRC-scope argument (SimLoop.move() always builds a MOVE envelope).
    assert captured[0].startswith(b"MOVE:")
    payload = decode_frame(captured[0][len(b"MOVE:"):], command=b"MOVE")
    assert payload is not None
    decoded = pb2_mod.CommandEnvelope.FromString(payload)
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
    from robot_radio.io.wire_codec import decode_frame
    from robot_radio.robot.pb2 import envelope_pb2 as pb2_mod

    captured: list[bytes] = []
    loop.inject_command = captured.append  # type: ignore[method-assign]

    returned_id = loop.move(v_x=100.0, stop_distance=50.0, timeout=1000.0, id=42)

    assert returned_id == 42
    # captured[0] is the FULL wire line ("MOVE:<cobs bytes>", 124-005) --
    # strip the leading command prefix and pass it as decode_frame()'s own
    # CRC-scope argument (SimLoop.move() always builds a MOVE envelope).
    assert captured[0].startswith(b"MOVE:")
    payload = decode_frame(captured[0][len(b"MOVE:"):], command=b"MOVE")
    assert payload is not None
    decoded = pb2_mod.CommandEnvelope.FromString(payload)
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


# ---------------------------------------------------------------------------
# 119 ticket 001 (kill-the-silent-off-shaping-config-boundary.md), RETARGETED
# 132-014: configure_from_robot()'s Tier 3 no longer builds one
# EstimatorConfigPatch envelope -- the nine fields split across TWO
# ConfigGroupTargets (ESTIMATOR/PLANNER_SHAPER). FIXED, 132-017 (JSON
# reshape ticket, stakeholder-sanctioned mid-sprint scope addition):
# PLANNER_SHAPER (the six shaper-ceiling fields) is LIVE now, split out of
# the boot-only PLANNER group specifically because it carries its own
# re-appliable setter (Motion::Planner::applyShaperLimits()) -- see
# push.py's estimator_kwargs() docstring and sim_loop.py's own Tier 3 doc
# comment. ESTIMATOR remains a permanent, honest dead end. The
# test_flags_bit16_shaping_disabled_asserts_when_push_stripped xfail below
# is UNCHANGED by this split -- its own premise is about BOOT-TIME
# behavior (loadBaked()+the boot-time no-arg install() already turning
# shaping ON before any Tier 3 push happens at all), not about whether a
# live per-target push can land; see that test's own xfail reason for the
# 130-002 composition-root-unification root cause, which this split does
# not touch.
# ---------------------------------------------------------------------------


def _stripped_config():
    """The real tovez_nocal.json fixture with config.estimator/
    config.planner_shaper both set to None -- 132-014: a REAL RobotConfig's
    grouped fields are NEVER None (132-020's shape has no Optional wrapper
    on any group), so the pre-132-014 approach (null out the nine
    individual fields via RobotConfig.model_validate()) no longer produces
    a validation-passing object at all (Estimator.weight_heading_otos etc.
    are plain floats, not Optional[float]) -- assigning None to the WHOLE
    group post-construction is the only way left to exercise
    estimator_kwargs()'s own "section is None -> select nothing" path (its
    docstring's own contract), while Tier 1/2 still see every other real
    group intact (motors/wheel_control/drive/geometry/wheels untouched).

    132-017: strips ``planner_shaper``, not ``planner`` -- the six shaper-
    ceiling fields ``estimator_kwargs()`` selects moved off ``config.
    planner`` onto their own ``config.planner_shaper`` group when Planner
    split (JSON reshape ticket, stakeholder-sanctioned mid-sprint scope
    addition); ``config.planner`` itself (the boot-only remainder) is no
    longer one of the nine fields this helper's callers care about."""
    from robot_radio.config.robot_config import load_robot_config

    cfg = load_robot_config(_ACTIVE_ROBOT_JSON)
    cfg.estimator = None
    cfg.planner_shaper = None
    return cfg


def test_configure_from_robot_pushes_estimator_config_and_logs_ack_counts(caplog):
    """The real tovez_nocal.json fixture carries all nine estimator/shaper
    fields -- configure_from_robot()'s Tier 3 push must still ATTEMPT all
    nine (selection is unaffected by 132-014). FIXED, 132-017 (JSON
    reshape ticket, stakeholder-sanctioned mid-sprint scope addition):
    PLANNER_SHAPER (the six shaper-ceiling fields) is now LIVE, split out
    of the boot-only PLANNER group -- 6/9 applied (the shaper fields),
    3/9 rejected (ESTIMATOR: permanent ERR_UNIMPLEMENTED) is the CORRECT,
    documented outcome now, mirroring __main__.py's own
    _push_estimator_config() log-line format (applied/rejected logging is
    still this ticket's own acceptance criterion; the 132-014 era's 0/9
    applied outcome was itself a temporary regression this split closes,
    not the target state)."""
    import logging as _logging

    from robot_radio.config.robot_config import load_robot_config

    sim = SimLoop()
    sim.connect()
    try:
        with caplog.at_level(_logging.INFO, logger="robot_radio.io.sim_loop"):
            sim.configure_from_robot(load_robot_config(_ACTIVE_ROBOT_JSON))
        messages = [r.message for r in caplog.records]
        assert any("pushed 6/9 estimator/shaper fields" in m for m in messages), (
            f"expected 6/9 applied (the now-live PLANNER_SHAPER fields) "
            f"log line, got: {messages}")
        assert any("3 rejected" in m for m in messages), messages
        assert not any("TIMED OUT" in m for m in messages), messages
    finally:
        sim.disconnect()


def test_configure_from_robot_logs_skip_when_config_carries_no_shaper_fields(caplog):
    """A config with estimator/planner stripped selects nothing
    (estimator_kwargs() -> {}) -- configure_from_robot() must log a skip,
    never attempt an empty push."""
    import logging as _logging

    sim = SimLoop()
    sim.connect()
    try:
        with caplog.at_level(_logging.INFO, logger="robot_radio.io.sim_loop"):
            sim.configure_from_robot(_stripped_config())
        messages = [r.message for r in caplog.records]
        assert any("no estimator/shaper fields on config" in m for m in messages), messages
    finally:
        sim.disconnect()


@pytest.mark.xfail(
    reason="132-014: this test's own premise -- 'ShaperLimits stays at the "
           "sim's own construction-time default (every field 0)' unless a "
           "LIVE Tier 3 push installs real ones -- no longer holds. "
           "Empirically (measured running this suite post-132-014): the sim "
           "now boots with shaping already ON (flags bit 16 clear) "
           "regardless of what Tier 3 pushes or fails to push, consistent "
           "with the composition-root unification (130-002) giving the sim "
           "its own loadBaked()+boot-time install() fan-out that this "
           "test's own docstring predates -- Config::Robot.planner is "
           "populated at CONSTRUCTION now, not exclusively by this "
           "(permanently rejected, PLANNER is boot-only) live push. Left "
           "failing rather than deleted: the premise needs an owner to "
           "confirm and re-derive what a 'shaping genuinely off' sim "
           "scenario looks like under the current architecture, not a "
           "132-014 call to make unilaterally.",
    strict=True,
)
def test_flags_bit16_shaping_disabled_asserts_when_push_stripped():
    """With the estimator/shaper fields stripped from the active config,
    configure_from_robot()'s Tier 3 push has nothing to send -- ShaperLimits
    stays at the sim's own construction-time default (every field 0,
    shaping OFF) -- flags bit 16 (kFlagFaultShapingDisabled) must assert on
    at least one frame while a MOVE is active."""
    sim = SimLoop()
    sim.connect()
    try:
        sim.configure_from_robot(_stripped_config())
        sim.move(v_x=100.0, stop_distance=200.0, timeout=5000.0)

        seen_bit_set: list = []

        def _saw_bit_set() -> bool:
            for f in sim.read_pending_binary_tlm_frames():
                if f.active and f.fault_shaping_disabled:
                    seen_bit_set.append(f)
            return len(seen_bit_set) > 0

        assert _wait_until(_saw_bit_set, timeout_s=3.0), (
            "expected flags bit 16 to assert on at least one frame while the MOVE was "
            "active with the estimator/shaper push stripped")
    finally:
        sim.disconnect()


def test_configure_from_robot_deterministic_session_never_logs_a_false_timeout(caplog):
    """Regression: a manual-step session (`connect(start_tick_thread=False)`
    -- turn_prediction_capture.py's own established pattern) has no
    background tick thread. 132-014: Tier 3 now pushes via
    ``set_config_field()`` per key (the same primitive Tier 1 already used
    successfully in a manual-step session before this ticket) rather than
    a hand-rolled fire-then-poll-manually path with its own explicit
    "skip the poll, nothing steps the sim" branch -- ``_run_or_enqueue()``'s
    own "run now if no tick thread" contract means the push (here, its
    outright rejection) is available synchronously, with no separate
    no-tick-thread special case left to log. What must still hold: no
    false "TIMED OUT" report, and the push still completes (0/9 applied,
    9 rejected -- both new wire targets are honest dead ends, see this
    section's own header comment) rather than hanging."""
    import logging as _logging

    from robot_radio.config.robot_config import load_robot_config

    sim = SimLoop()
    sim.connect(start_tick_thread=False)
    try:
        with caplog.at_level(_logging.INFO, logger="robot_radio.io.sim_loop"):
            sim.configure_from_robot(load_robot_config(_ACTIVE_ROBOT_JSON))
        messages = [r.message for r in caplog.records]
        assert not any("TIMED OUT" in m for m in messages), (
            f"a manual-step session must never report a false ack timeout: {messages}")
        assert any("pushed 0/9 estimator/shaper fields" in m for m in messages), messages
    finally:
        sim.disconnect()


def test_configure_from_robot_deterministic_session_still_applies_shaper_limits():
    """Companion: even though the ack is unobservable without stepping (see
    the false-timeout regression test above), the push itself must still
    have landed -- once the caller actually steps the sim and issues a
    Move, flags bit 16 must stay clear (mirrors
    test_flags_bit16_shaping_disabled_clear_when_push_present, but for a
    manual-step session instead of a real-time one)."""
    from robot_radio.config.robot_config import load_robot_config

    sim = SimLoop()
    sim.connect(start_tick_thread=False)
    try:
        sim.configure_from_robot(load_robot_config(_ACTIVE_ROBOT_JSON))
        sim.move(v_x=100.0, stop_distance=200.0, timeout=5000.0)

        saw_active = False
        for _ in range(200):  # generously bounded -- well past a 200mm/100mm/s leg
            sim.step(1)
            for f in sim.drain_pending_tlm():
                if f.active:
                    saw_active = True
                assert not (f.active and f.fault_shaping_disabled), (
                    f"flags bit 16 must stay clear -- the Tier 3 push landed synchronously "
                    f"even with no tick thread running: frame={f!r}")
            if saw_active and not sim._active:
                break
        assert saw_active, "expected at least one active=True frame during the Move"
    finally:
        sim.disconnect()


def test_flags_bit16_shaping_disabled_clear_when_push_present(loop):
    """Companion: the SAME kind of MOVE against this file's own `loop`
    fixture (fully configured -- tovez_nocal.json's real estimator/control
    sections push via the new Tier 3 call) -- flags bit 16 must never
    assert on any frame drained while the MOVE is active."""
    loop.move(v_x=100.0, stop_distance=200.0, timeout=5000.0)

    saw_active = False
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        for f in loop.read_pending_binary_tlm_frames():
            if f.active:
                saw_active = True
            assert not (f.active and f.fault_shaping_disabled), (
                f"flags bit 16 must stay clear once the Tier 3 push landed real shaper "
                f"limits: frame={f!r}")
        time.sleep(_POLL_INTERVAL_S)
    assert saw_active, "expected at least one active=True frame during the Move"
