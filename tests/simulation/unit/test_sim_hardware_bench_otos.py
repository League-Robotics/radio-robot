"""
test_sim_hardware_bench_otos.py — 074-001 regression tests for SimHardware's
bench-OTOS parity with firmware (NezhaHAL/MecanumHAL).

Prior to this ticket, `SimHardware::setOtosBench()` only recorded a flag and
`otos()` always returned the ground-truth `SimOdometer`; `DebugCommands::
handleDbgOtos()`'s `HOST_BUILD` branch hardcoded `ideal=0,0,0 otos=0,0,0`.
`DBG OTOS BENCH 1` was therefore a structural no-op in every sim/TestGUI-sim
session -- no moving object existed behind it.

These tests exercise the real fix: `SimHardware` now owns a `BenchOtosSensor`
member and performs a real active-pointer swap identical to `NezhaHAL`'s,
driven every actuator tick via `advance()`'s dt-baseline-gated call.  Every
assertion here that checks a non-zero / changing `ideal=`/`otos=` triple would
FAIL against the pre-fix code (which always replies `ideal=0,0,0 otos=0,0,0`
in HOST_BUILD).
"""
import re

from firmware import Sim


def send(s: Sim, cmd: str) -> str:
    return s.send_command(cmd)


def parse_triple(reply: str, key: str):
    """Extract the (x, y, h) integer triple following `key=` in a DBG OTOS reply."""
    m = re.search(re.escape(key) + r"=(-?\d+),(-?\d+),(-?\d+)", reply)
    assert m is not None, f"{key}= triple not found in: {reply!r}"
    return tuple(int(v) for v in m.groups())


# ---------------------------------------------------------------------------
# Bench sensor tracks commanded motion once enabled (the ticket's core fix).
# ---------------------------------------------------------------------------

def test_bench_otos_tracks_commanded_arc_when_enabled():
    """DBG OTOS BENCH 1 + a driving command must produce a MOVING bench OTOS.

    Regression for 074-001: pre-fix, SimHardware had no BenchOtosSensor to
    swap to and handleDbgOtos()'s HOST_BUILD branch hardcoded ideal=0,0,0
    otos=0,0,0 -- these assertions fail against that code (ideal/otos would
    stay exactly (0, 0, 0) no matter how long the robot drives).
    """
    with Sim() as s:
        assert "otos bench=1" in send(s, "DBG OTOS BENCH 1")

        # Drive an arc: forward + turning, so both position and heading move.
        send(s, "VW 100 300")
        s.tick_for(500)

        reply = send(s, "DBG OTOS")
        ideal = parse_triple(reply, "ideal")
        otos = parse_triple(reply, "otos")

        assert ideal != (0, 0, 0), (
            f"bench OTOS ideal pose must track commanded motion, got {ideal}; "
            f"full reply: {reply!r}"
        )
        # otos (errored accumulator) tracks ideal closely with zero configured
        # noise -- both must have moved together.
        assert otos != (0, 0, 0), f"bench OTOS otos pose must track motion, got {otos}"
        # Forward speed was commanded positive -> x must have advanced forward.
        assert ideal[0] > 0, f"expected forward x growth, got ideal={ideal}"
        # Positive omega was commanded -> heading must have advanced too.
        assert ideal[2] != 0, f"expected heading change from turning, got ideal={ideal}"


def test_bench_otos_position_grows_with_more_ticks():
    """The bench accumulator keeps advancing tick-over-tick while enabled (not
    a one-shot jump on enable) -- confirms it is driven every advance() call
    via the dt-baseline discipline, not just once."""
    with Sim() as s:
        send(s, "DBG OTOS BENCH 1")
        send(s, "VW 100 0")

        s.tick_for(200)
        ideal_1 = parse_triple(send(s, "DBG OTOS"), "ideal")

        s.tick_for(200)
        ideal_2 = parse_triple(send(s, "DBG OTOS"), "ideal")

        assert ideal_2[0] > ideal_1[0], (
            f"expected continued forward growth: ideal_1={ideal_1} ideal_2={ideal_2}"
        )


# ---------------------------------------------------------------------------
# Disabling bench mode freezes the bench accumulator and restores the real
# ground-truth SimOdometer as the active odometer.
# ---------------------------------------------------------------------------

def test_bench_otos_freezes_after_disable():
    """DBG OTOS BENCH 0 stops driving the bench accumulator -- further driving
    does not change ideal=/otos= (the accumulator freezes, it is not reset)."""
    with Sim() as s:
        send(s, "DBG OTOS BENCH 1")
        send(s, "VW 100 0")
        s.tick_for(300)
        ideal_enabled = parse_triple(send(s, "DBG OTOS"), "ideal")
        assert ideal_enabled != (0, 0, 0)

        assert "otos bench=0" in send(s, "DBG OTOS BENCH 0")

        # Keep driving after disabling -- the bench accumulator must not move.
        s.tick_for(300)
        ideal_after_disable = parse_triple(send(s, "DBG OTOS"), "ideal")

        assert ideal_after_disable == ideal_enabled, (
            f"bench OTOS must freeze once disabled: before={ideal_enabled} "
            f"after={ideal_after_disable}"
        )


def test_bench_otos_round_trip_swaps_active_odometer():
    """DBG OTOS BENCH 1 / 0 really swaps Hardware::otos() -- not just a flag.

    Injects a deterministic read failure into the real SimOdometer
    (sim_set_otos_read_failure).  While bench mode is OFF, hal.otos() is the
    (failing) SimOdometer, so DBG OTOS's status/statusOk fields show the
    failure.  While bench mode is ON, hal.otos() is the (always-healthy)
    BenchOtosSensor, so the SAME injected SimOdometer failure must be masked.
    Disabling again must re-expose it.  This is the direct proof that
    setOtosBench()/otos() perform a real pointer swap (074-001) -- the same
    proof NezhaHAL's isBenchMode()/_otosActive already provides in firmware.
    """
    with Sim() as s:
        s.set_otos_read_failure(True)

        reply_before = send(s, "DBG OTOS")
        assert "status=0xFF" in reply_before and "statusOk=0" in reply_before, (
            f"real SimOdometer's injected failure must be visible when bench "
            f"mode is off: {reply_before!r}"
        )

        send(s, "DBG OTOS BENCH 1")
        reply_bench = send(s, "DBG OTOS")
        assert "status=0x00" in reply_bench and "statusOk=1" in reply_bench, (
            f"bench mode must swap otos() away from the failing SimOdometer "
            f"to the always-healthy BenchOtosSensor: {reply_bench!r}"
        )

        send(s, "DBG OTOS BENCH 0")
        reply_after = send(s, "DBG OTOS")
        assert "status=0xFF" in reply_after and "statusOk=0" in reply_after, (
            f"disabling bench mode must restore otos() to the real (still "
            f"failing) SimOdometer: {reply_after!r}"
        )


def test_bench_otos_isbenchmode_round_trip():
    """DBG OTOS BENCH 1 then 0 round-trips the bench=<n> flag reported via
    Hardware::isBenchMode()."""
    with Sim() as s:
        assert "otos bench=1" in send(s, "DBG OTOS BENCH 1")
        assert "otos bench=0" in send(s, "DBG OTOS BENCH 0")


# ---------------------------------------------------------------------------
# 074-002: Drive's LIVE fusion/telemetry path observes a runtime bench-OTOS
# swap.
#
# The tests above (074-001) prove the SUBSTRATE really swaps the active
# odometer (Hardware::setOtosBench()/otos(), exercised through the DBG OTOS
# command, which reads the bench sensor's accumulators directly via
# benchOtosPtr() -- a code path that never touches Drive at all).
#
# These tests prove the SEPARATE, previously-broken half: Drive::tickUpdate()
# STEP 5 -- the sole live OTOS-read-and-fuse path that feeds the EKF and the
# `otos=` TLM clause (RobotTelemetry.cpp reads ds.optical.pose, exposed here
# via get_optical_pose()) -- actually RE-RESOLVES the active odometer on every
# read instead of forever calling methods on the C++ reference it captured
# once at Robot construction time (before any DBG OTOS BENCH command could
# ever run). Pre-fix, that reference is permanently bound to the real
# SimOdometer regardless of any later bench-mode toggle.
# ---------------------------------------------------------------------------

def test_drive_live_path_observes_bench_swap_mid_session():
    """074-002 regression: a runtime DBG OTOS BENCH toggle must reach
    Drive::tickUpdate() STEP 5 -- the live fusion/telemetry path -- on the
    VERY NEXT tick.

    Method: inject a deterministic read FAILURE into the real, ground-truth
    SimOdometer (sim_set_otos_read_failure) and force Drive to attempt an
    OTOS read every tick (set_otos_fusion bypasses the internal lag gate).
    While bench mode is off, every read must fail, so Drive's raw optical
    pose (state().optical, the same field RobotTelemetry's `otos=` clause
    reads) must stay frozen at its never-successfully-written initial value
    (0, 0, 0) no matter how long the robot drives.

    Enabling bench mode swaps the ACTIVE odometer to the always-healthy
    BenchOtosSensor. THE FIX UNDER TEST: the live path must observe a
    successful read -- and optical.pose must start moving -- starting the
    very next tick. Pre-fix, `Drive::_otos` is a reference bound once, at
    construction, to the real (still-failing) SimOdometer; toggling bench
    mode later is a no-op for it, so optical.pose would stay stuck at
    (0, 0, 0) forever and this test's central assertion would fail against
    that code -- confirmed by temporarily reverting Drive's constructor
    signature to `IOdometer& otos` / `_otos(otos)` locally and re-running
    this test during implementation.
    """
    with Sim() as s:
        s.set_otos_fusion(True)        # bypass the lag gate: fuse every tick
        s.set_otos_read_failure(True)  # real SimOdometer.readTransformed() always fails

        send(s, "VW 100 300")
        s.tick_for(200)

        frozen = s.get_optical_pose()
        assert frozen == (0.0, 0.0, 0.0), (
            f"real SimOdometer is failing every read -- Drive's optical pose "
            f"must stay at its never-successfully-written initial value, "
            f"got {frozen}"
        )

        # THE FIX UNDER TEST: swap the live path's active odometer.
        assert "otos bench=1" in send(s, "DBG OTOS BENCH 1")
        s.tick_for(48)  # a couple of control periods is enough

        after_toggle = s.get_optical_pose()
        assert after_toggle != (0.0, 0.0, 0.0), (
            f"Drive's live fusion/telemetry path must observe the bench "
            f"sensor's successful read starting the very next tick after "
            f"DBG OTOS BENCH 1 -- pre-fix Drive keeps reading the stale, "
            f"still-failing real SimOdometer forever and this would stay "
            f"(0, 0, 0): {after_toggle}"
        )

        # Keep ticking: the bench sensor keeps advancing (074-001's
        # dt-baseline discipline) and the live path must keep observing it.
        s.tick_for(200)
        later = s.get_optical_pose()
        assert later != after_toggle, (
            f"bench sensor must keep advancing tick-over-tick on the live "
            f"path: after_toggle={after_toggle} later={later}"
        )

        # Toggle back off: the live path must observe the real (still
        # failing) SimOdometer resume being the active sensor -- optical
        # pose freezes again at whatever the bench sensor last wrote.
        assert "otos bench=0" in send(s, "DBG OTOS BENCH 0")
        s.tick_for(200)
        frozen_again = s.get_optical_pose()
        assert frozen_again == later, (
            f"disabling bench mode must restore the real (failing) "
            f"SimOdometer as Drive's active sensor on the live path -- "
            f"optical pose must freeze again: before={later} "
            f"after={frozen_again}"
        )
