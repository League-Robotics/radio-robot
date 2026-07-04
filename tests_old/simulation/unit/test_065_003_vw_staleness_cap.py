"""
test_065_003_vw_staleness_cap.py — Regression tests for ticket 065-003 /
CR-05b: a second, `+`-independent staleness cap on open-ended VW-class
(S/VW/R/_VW) velocity targets.

Background (see clasi/sprints/065-.../issues/
stop-delivery-and-keepalive-watchdog-architecture.md CR-05b and
architecture-update.md Step 4-5 item 3 / Design Rationale Decision 3):

  Ticket 065-002 narrowed the motion watchdog reset to '+'/motion verbs
  only (CR-05a).  But '+' itself still resets the watchdog by design --
  that is its whole purpose.  If the host's SerialConnection keepalive
  daemon thread is still alive and emitting '+' independent of whatever
  code is actually supposed to be refreshing the VW target (e.g. a frozen
  Qt GUI event loop), the firmware cannot tell the difference: '+' keeps
  arriving, the watchdog keeps resetting, and the robot keeps driving at
  the last VW target forever.

  Fix: Planner::beginVelocity()/beginRawVelocity() stamp a new
  _lastVelocityRefreshMs timestamp on every genuine open-ended
  velocity-target refresh.  Superstructure::evaluateSafety()'s watchdog
  block additionally trips if that timestamp goes stale (same sTimeoutMs
  threshold used for the '+'/motion-verb watchdog), regardless of whether
  '+' is still arriving.

  Implementation note: the VW "D6 origin guard" keepalive path
  (MotionCommands.cpp handleVW) updates an already-active RETARGETABLE
  command's target via activeCmd().setTarget() directly, bypassing
  beginVelocity() by design (to avoid cancel/reconfigure churn on every
  resend).  Planner::markVelocityRefreshed() stamps the same timestamp
  from that call site so genuine VW resends (the KeyboardDriver
  resend-timer pattern, and S-mode streaming) are correctly recognized as
  fresh and do not go stale.

Tests exercise the actual compiled C++ via the firmware.Sim ctypes
wrapper (the `sim` fixture from tests/conftest.py), not a Python mirror of
the classification logic.
"""
import ctypes

TICK_STEP_MS = 24


def _tick(sim, ms: int = TICK_STEP_MS) -> None:
    """Advance the sim clock by one step, keeping sim._t in sync."""
    sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
    sim._t += ms


# ---------------------------------------------------------------------------
# 1. The sprint's exact CR-05b regression scenario: an open-ended VW kept
#    "alive" by an ambient '+' keepalive ONLY (no fresh VW resend at all)
#    must still safety-stop once sTimeoutMs has elapsed since the last
#    genuine VW.  Pre-065-003 this scenario was impossible to construct --
#    '+' alone reset the (only) watchdog signal forever, so the robot would
#    drive forever as long as a background keepalive thread stayed alive.
# ---------------------------------------------------------------------------

def test_vw_goes_stale_despite_continuous_plus_keepalive(sim):
    """'+'-only keepalive (no VW resend) must NOT prevent the VW-staleness
    cap from safety-stopping an open-ended VW once sTimeoutMs has elapsed
    since the last genuine velocity-target refresh (CR-05b)."""
    sim.send_command("SET sTimeout=500")
    sim.get_async_evts()  # drain

    resp = sim.send_command("VW 200 0")
    assert "OK" in resp.upper(), f"VW not accepted; resp={resp!r}"

    # Send '+' every ~192ms (well under the 500ms window) for 1.44s -- no
    # VW resend at all.  Under 065-002-only behaviour this held VW alive
    # indefinitely; 065-003 must trip the vwDelta signal regardless of '+'.
    #
    # Drain async EVTs before each send_command call: sim_command() resets
    # the reply store at the start of every command so it captures only
    # that command's own synchronous reply -- an un-drained safety_stop EVT
    # would otherwise be silently discarded by the next '+' call.
    evts = ""
    for i in range(60):
        _tick(sim)
        evts += sim.get_async_evts()
        if i % 8 == 0:
            sim.send_command("+")

    evts += sim.get_async_evts()
    assert "EVT safety_stop" in evts, (
        f"Expected the VW-staleness cap to fire despite continuous '+' "
        f"keepalive with no fresh VW; evts={evts!r}"
    )

    vel_l = float(sim._lib.sim_get_vel_l(sim._h))
    assert abs(vel_l) < 1.0, (
        f"Expected robot stopped after the VW-staleness safety_stop, "
        f"got vel_l={vel_l}"
    )


# ---------------------------------------------------------------------------
# 2. Regression: VW resent by the caller itself (no '+' at all) must
#    continue to satisfy the watchdog -- _lastVelocityRefreshMs alone is
#    sufficient, confirming the staleness signal does not require both
#    signals simultaneously.  This is the KeyboardDriver resend-timer
#    pattern and exercises the D6 origin-guard's markVelocityRefreshed()
#    call (the resend does NOT go through beginVelocity() again once a
#    RETARGETABLE command is already active).
# ---------------------------------------------------------------------------

def test_vw_resend_without_plus_keeps_it_alive(sim):
    """Fresh VW resends alone (no '+' at all) must hold an open-ended VW
    alive past sTimeoutMs."""
    sim.send_command("SET sTimeout=500")
    sim.get_async_evts()

    resp = sim.send_command("VW 200 0")
    assert "OK" in resp.upper(), f"VW not accepted; resp={resp!r}"

    # Resend VW every ~192ms (well under 500ms) for 1.44s -- no '+' ever.
    evts = ""
    for i in range(60):
        _tick(sim)
        evts += sim.get_async_evts()
        if i % 8 == 0:
            r = sim.send_command("VW 200 0")
            assert "cancelled" not in r.lower(), (
                f"VW resend while already active must be treated as a "
                f"keepalive (D6 origin guard), not a preempting new "
                f"command; resp={r!r}"
            )

    evts += sim.get_async_evts()
    assert "EVT safety_stop" not in evts, (
        f"VW resends alone (no '+') must hold VW alive; evts={evts!r}"
    )

    vel_l = float(sim._lib.sim_get_vel_l(sim._h))
    assert abs(vel_l) > 1.0, (
        f"Expected VW to still be driving after VW-resend-only keepalive, "
        f"got vel_l={vel_l}"
    )


# ---------------------------------------------------------------------------
# 3. Regression: _VW (raw velocity, beginRawVelocity) resent by the caller
#    (no '+' at all) must also continue to satisfy the watchdog --
#    exercises beginRawVelocity()'s now_ms stamp directly (its single call
#    site, handle_VW, is not gated by the D6 origin guard -- it always
#    calls beginRawVelocity() fresh).
# ---------------------------------------------------------------------------

def test_raw_vw_resend_without_plus_keeps_it_alive(sim):
    """_VW resends alone (no '+') must hold an open-ended raw-velocity
    stream alive past sTimeoutMs."""
    sim.send_command("SET sTimeout=500")
    sim.get_async_evts()

    resp = sim.send_command("_VW 200 0")
    assert "OK" in resp.upper(), f"_VW not accepted; resp={resp!r}"

    evts = ""
    for i in range(60):
        _tick(sim)
        evts += sim.get_async_evts()
        if i % 8 == 0:
            sim.send_command("_VW 200 0")

    evts += sim.get_async_evts()
    assert "EVT safety_stop" not in evts, (
        f"_VW resends alone (no '+') must hold the raw-velocity stream "
        f"alive; evts={evts!r}"
    )

    vel_l = float(sim._lib.sim_get_vel_l(sim._h))
    assert abs(vel_l) > 1.0, (
        f"Expected _VW to still be driving after resend-only keepalive, "
        f"got vel_l={vel_l}"
    )


# ---------------------------------------------------------------------------
# 4. Regression: bounded/self-terminating verbs (T/D/G/TURN/RT) are
#    unaffected by the new VW-staleness cap.  T calls beginVelocity() (it
#    is routed through Goal::VELOCITY) but immediately installs a TIME
#    stop, which exempts it via needsWatchdog == false; D/G/TURN/RT never
#    call beginVelocity()/beginRawVelocity() for their own primary command
#    at all.  Either way the vwDelta check in evaluateSafety() is gated on
#    needsWatchdog, so it never engages for these verbs regardless of
#    keepalive/VW-resend traffic (zero of either is sent here).
# ---------------------------------------------------------------------------

def test_bounded_verbs_unaffected_by_vw_staleness_cap(sim):
    """T/D/G/TURN/RT complete normally with ZERO keepalives and zero VW
    resends -- the VW-staleness cap must not engage for TIME-stop-exempt,
    self-terminating commands."""
    sim.send_command("SET sTimeout=500")

    cases = [
        ("T 150 150 300", "EVT done T", 1_500),
        ("D 150 150 200", "EVT done D", 5_000),
        ("G 300 0 200", "EVT done G", 15_000),
        ("TURN 4500", "EVT done TURN", 5_000),
        ("RT 4500", "EVT done RT", 5_000),
    ]
    for cmd, done_evt, budget_ms in cases:
        sim.get_async_evts()  # drain leftovers from the previous case
        resp = sim.send_command(cmd)
        assert "err" not in resp.lower(), f"{cmd!r} not accepted; resp={resp!r}"

        for _ in range(budget_ms // TICK_STEP_MS):
            _tick(sim)

        evts = sim.get_async_evts()
        assert done_evt in evts, (
            f"Expected {done_evt!r} for {cmd!r} with zero keepalives and "
            f"zero VW resends (TIME-stop exemption must be unaffected by "
            f"the VW-staleness cap); evts={evts!r}"
        )
        assert "safety_stop" not in evts, (
            f"Unexpected safety_stop for {cmd!r}: evts={evts!r}"
        )
