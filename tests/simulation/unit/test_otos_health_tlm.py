"""
test_otos_health_tlm.py — Ticket 074-004 (OTOS health on the wire).

The issue's acceptance sketch asks that "a persistent OTOS read failure or
fusion block is surfaced on the wire ... not silent." Before this ticket the
only wire-visible symptom of a fusion block was an indirectly-inferred one: a
climbing `ekf_rej=` counter combined with a suspiciously static `otos=`
value -- exactly the pattern that made the issue's field session hard to
diagnose. `Drive::tickUpdate()` STEP 5/6 now copies the raw OTOS STATUS byte
and `_otosFusionBlocked` into `msg::DrivetrainState`, and
`Robot::buildTlmFrame` emits them unconditionally (no freshness gate,
mirroring `wedge=`'s precedent) as `otos_health=<status>,<blocked>` whenever
`TLM_FIELD_OTOS_HEALTH` is set (on by default, part of `TLM_FIELD_ALL`).

This also closes the issue's fourth investigation pointer: `otos=` itself is
documented (RobotTelemetry.cpp, Drive.cpp STEP 5) as the raw, last-
successfully-read pose, independent of fusion-gate state -- it does not go
stale or change meaning when `_otosFusionBlocked` is true. A read FAILURE
(as opposed to a fusion block) is a different condition entirely: it clears
`ds.otos.valid` the same tick, so the existing N8 freshness gate makes
`otos=` disappear from TLM rather than repeating a stale value forever.

Tests
-----
test_otos_health_reflects_blocked_state_on_wire
    A persistent WARNING-bit injection (the existing CR-06 STATUS-bit
    trigger, `sim.set_otos_warn(True)`) engages `_otosFusionBlocked`; the
    very next SNAP's `otos_health=` clause reflects `blocked=True` and the
    raw STATUS byte the sim's WARNING injection reports. Clearing the warn
    bit and ticking past `kOtosCleanReadmitN` re-admits fusion, and a
    subsequent SNAP shows `otos_health=0,False`.

test_otos_read_failure_clears_otos_field_not_stale
    SUC-005 regression: once `sim.set_otos_read_failure(True)` is injected
    and the sim ticks well past `2 * lagOtos` (the existing N8 freshness
    window `otos=` already uses), `otos=` is ABSENT from the next TLM frame
    -- not present with the last-good value repeated -- proving no
    stale-cache-masks-a-read-failure defect exists in the raw OTOS path,
    independent of `otos_health=`'s own (unconditional) presence in the same
    frame.
"""
import sys
from pathlib import Path

# host/ is on sys.path via tests/conftest.py, but be defensive in case this
# file is ever run standalone (mirrors test_064_004_wedge_blindspots.py).
_HOST_DIR = Path(__file__).resolve().parents[3] / "host"
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))

from robot_radio.robot.protocol import parse_tlm  # noqa: E402


# ---------------------------------------------------------------------------
# otos_health= wire visibility: blocked <-> re-admitted transition.
# ---------------------------------------------------------------------------

def test_otos_health_reflects_blocked_state_on_wire(sim):
    """A persistent WARNING-bit block shows blocked=True on the wire; clearing
    the warn bit and re-admitting fusion flips it back to blocked=False."""
    sim.send_command("SET sTimeout=60000")

    sim.set_otos_fusion(True)
    sim.set_otos_warn(True)

    r = sim.send_command("VW 200 0")
    assert "OK" in r.upper(), f"VW command failed: {r!r}"

    # Well past kOtosWarnPersistK (3 ticks) -- fusion should now be blocked
    # (mirrors test_otos_warn_persistence.py's own tick budget for the same
    # trigger).
    sim.tick_for(40 * 24, step_ms=24)

    reply = sim.send_command("SNAP")
    frame = parse_tlm(reply)
    assert frame is not None, f"SNAP did not parse as TLM: {reply!r}"
    assert frame.otos_health is not None, (
        f"otos_health= missing from SNAP frame (should be unconditional, "
        f"on by default): {reply!r}"
    )
    status, blocked = frame.otos_health
    # SimOdometer::readStatus() reports 0x02 (warnOpticalTracking) while
    # sim.set_otos_warn(True) is active (source/hal/sim/SimOdometer.h).
    assert status == 2, (
        f"expected raw STATUS byte 2 (warnOpticalTracking) while warned, "
        f"got {status} (raw: {reply!r})"
    )
    assert blocked is True, (
        f"expected otos_health= blocked=True once the WARNING-bit streak "
        f"persists past kOtosWarnPersistK, got blocked={blocked} "
        f"(raw: {reply!r})"
    )

    # ---- Recovery: clear the warn bit AND stop driving. The sim's OTOS pose
    # stays frozen at (0,0,0) throughout this test (no enable_otos_model()/
    # set_otos_pose() call) -- if the robot kept driving, the independent
    # 074-003 pose-VALUE staleness check (encoder-evidenced motion +
    # unchanged pose) would re-arm and keep the gate blocked for a second,
    # unrelated reason. Stopping disarms that check regardless of the frozen
    # pose (encMotion gates it), isolating this test to the STATUS-bit path
    # alone -- the same "stop before checking re-admission" shape
    # test_otos_warn_persistence.py's and test_otos_stuck_value_gate.py's own
    # recovery phases already use. ----
    sim.set_otos_warn(False)
    sim.send_command("X")
    sim.tick_for(20 * 24, step_ms=24)

    reply2 = sim.send_command("SNAP")
    frame2 = parse_tlm(reply2)
    assert frame2 is not None, f"SNAP did not parse as TLM: {reply2!r}"
    assert frame2.otos_health is not None, (
        f"otos_health= missing from SNAP frame: {reply2!r}"
    )
    status2, blocked2 = frame2.otos_health
    assert status2 == 0, (
        f"expected a clean STATUS byte (0) once the warn bit clears, "
        f"got {status2} (raw: {reply2!r})"
    )
    assert blocked2 is False, (
        f"expected otos_health= blocked=False once fusion is re-admitted "
        f"after the clean streak, got blocked={blocked2} (raw: {reply2!r})"
    )


# ---------------------------------------------------------------------------
# SUC-005 regression: a read failure clears otos='s freshness envelope.
# ---------------------------------------------------------------------------

def test_otos_read_failure_clears_otos_field_not_stale(sim):
    """A persistent OTOS read failure makes otos= disappear (N8 freshness
    gate), not repeat the last-good value forever."""
    sim.send_command("SET sTimeout=60000")
    sim.set_otos_fusion(True)

    r = sim.send_command("VW 200 0")
    assert "OK" in r.upper(), f"VW command failed: {r!r}"

    # A few clean ticks first so otos= is actually populated -- proves the
    # absence asserted below is caused by the injected failure, not by
    # otos= never having been present in the first place.
    sim.tick_for(5 * 24, step_ms=24)
    reply0 = sim.send_command("SNAP")
    frame0 = parse_tlm(reply0)
    assert frame0 is not None and frame0.otos is not None, (
        f"setup: otos= should be present before injecting a read failure: "
        f"{reply0!r}"
    )
    last_good_otos = frame0.otos

    sim.set_otos_read_failure(True)

    # Tick well past 2 * lagOtos (default 10 ms -- see RobotConfig.lagOtos,
    # the same N8 freshness window otos= already uses, RobotTelemetry.cpp)
    # so at least one STEP-5 OTOS read attempt (and the freshness re-check)
    # has run with the failure active. sim.set_otos_fusion(True) also forces
    # OTOS correction every tick (bypasses the lag gate entirely), so this
    # margin is generous, not marginal.
    sim.tick_for(20 * 24, step_ms=24)

    reply1 = sim.send_command("SNAP")
    frame1 = parse_tlm(reply1)
    assert frame1 is not None, f"SNAP did not parse as TLM: {reply1!r}"
    assert frame1.otos is None, (
        f"otos= should be ABSENT after a persistent read failure clears the "
        f"freshness envelope, not repeating the last-good value "
        f"{last_good_otos}: got {frame1.otos} (raw: {reply1!r})"
    )

    # Confirm the absence holds on a later frame too (not a one-tick blip).
    sim.tick_for(20 * 24, step_ms=24)
    reply2 = sim.send_command("SNAP")
    frame2 = parse_tlm(reply2)
    assert frame2 is not None and frame2.otos is None, (
        f"otos= reappeared while the read failure is still active: {reply2!r}"
    )

    # otos_health= itself, by contrast, is unconditional -- it must still be
    # present (and reporting fusion as effectively non-progressing) even
    # while the raw pose field above has gone silent. This is the whole
    # point of the two fields having different gating (Design Rationale 4).
    assert frame2.otos_health is not None, (
        f"otos_health= should remain present even while otos= is absent "
        f"(it has no freshness gate): {reply2!r}"
    )
