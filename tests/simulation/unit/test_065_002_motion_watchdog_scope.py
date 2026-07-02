"""
test_065_002_motion_watchdog_scope.py — Regression tests for ticket 065-002 /
CR-05a: scope the firmware motion-watchdog reset to keepalive ('+') and
motion verbs only, not every inbound line.

Background (see clasi/sprints/065-.../issues/
stop-delivery-and-keepalive-watchdog-architecture.md CR-05a and
architecture-update.md Step 4-5 item 2):

  Pre-fix, LoopScheduler::runCommsIn (both the serial and radio branches)
  and LoopScheduler::run_test() called sched.resetWatchdog(now)
  unconditionally after EVERY inbound line -- GET, SNAP, and any other
  non-motion query reset the same timestamp '+'/VW do.  tests/_infra/sim/
  sim_api.cpp's sim_command() had an identical, explicitly-commented
  parallel copy.  Net effect: a host that only polls telemetry (no '+', no
  motion resend) could silently mask a stalled motion watchdog for
  open-ended S/VW/R motion -- the firmware could not distinguish "host is
  alive and driving" from "host is alive and merely polling."

  Fix: CommandTypes.h gains a CMD_MOTION_WATCHDOG bitmask flag, OR'd into
  the flags of the '+' descriptor (SystemCommands.cpp) and the 11
  motion-verb descriptors (S, T, D, G, R, TURN, RT, VW, _VW, X, STOP in
  MotionCommands.cpp).  CommandProcessor records the matched descriptor's
  flags in dispatchTable() (_lastDispatchFlags) right after a successful
  parse; a new public accessor, lastCommandResetsWatchdog(), exposes the
  classification.  LoopScheduler::runCommsIn/run_test and sim_command() gate
  their resetWatchdog()/_ts.watchdogMs write on it instead of calling
  unconditionally.

Tests below exercise the actual compiled C++ via the firmware.Sim ctypes
wrapper, not a Python mirror of the classification logic.
"""
import ctypes

from firmware import Sim

TICK_STEP_MS = 24


def _tick(s: Sim, ms: int = TICK_STEP_MS) -> None:
    """Advance the sim clock by one step, keeping s._t in sync."""
    s._lib.sim_tick(s._h, ctypes.c_uint32(s._t))
    s._t += ms


# ---------------------------------------------------------------------------
# 1. The sprint's exact CR-05a regression scenario: an open-ended VW session
#    that receives ONLY GET/SNAP traffic (no '+', no fresh VW) for longer
#    than sTimeoutMs must still safety-stop.  Pre-fix this scenario was
#    impossible to construct because any inbound line -- including GET and
#    SNAP -- reset the watchdog, so the robot would drive forever as long as
#    a host kept polling telemetry.
# ---------------------------------------------------------------------------

def test_get_snap_only_traffic_does_not_mask_watchdog_during_vw(build_lib):
    """Open-ended VW + GET/SNAP-only polling (no '+', no VW resend) must
    still safety-stop at sTimeoutMs -- ambient telemetry polling must no
    longer substitute for '+'/motion-verb keepalive (CR-05a)."""
    with Sim() as s:
        s.send_command("SET sTimeout=500")
        s.get_async_evts()  # drain

        resp = s.send_command("VW 200 0")
        assert "OK" in resp.upper(), f"VW not accepted; resp={resp!r}"

        # Poll with GET/SNAP only for > sTimeoutMs (500 ms) -- no '+', no
        # fresh VW.  60 * 24ms = 1440ms, well past the 500ms window.
        #
        # Drain async EVTs (from the preceding tick) BEFORE each send_command
        # call: sim_command() resets the reply store at the start of every
        # command so it captures only that command's own synchronous reply
        # (see sim_api.cpp's sim_command doc comment) -- an un-drained
        # safety_stop EVT would otherwise be silently discarded by the next
        # SNAP/GET call.
        evts = ""
        for i in range(60):
            _tick(s)
            evts += s.get_async_evts()
            s.send_command("SNAP" if i % 2 == 0 else "GET ml")

        evts += s.get_async_evts()
        assert "EVT safety_stop" in evts, (
            f"Expected the motion watchdog to fire despite continuous "
            f"GET/SNAP polling -- ambient traffic (GET/SNAP) must not reset "
            f"the motion watchdog; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# 2. Regression guard: narrowing the classification must not break the
#    legitimate keepalive path.  An active VW session kept alive by '+'
#    plus periodic VW resends must NOT trip the watchdog.
#
#    065-003 / CR-05b note: prior to 065-003, '+' ALONE (no VW resend) was
#    sufficient to hold VW alive indefinitely, and this test originally
#    encoded exactly that claim (test_plus_alone_keeps_vw_alive_without_resend,
#    sending only '+' and never resending VW).  065-003 closes that gap:
#    Planner now stamps _lastVelocityRefreshMs on every genuine
#    velocity-target refresh, and Superstructure::evaluateSafety() trips
#    the watchdog if that timestamp alone goes stale -- an ambient '+'
#    keepalive thread no longer substitutes for a genuine VW refresh. This
#    test now resends VW periodically (the realistic KeyboardDriver
#    pattern) alongside '+', which still demonstrates that '+' correctly
#    keeps resetting the classification-based watchdog signal from this
#    ticket. The now-superseded "'+' alone is sufficient forever" claim,
#    and its replacement ("'+' alone is NOT sufficient past sTimeoutMs"),
#    are covered by tests/simulation/unit/test_065_003_vw_staleness_cap.py.
# ---------------------------------------------------------------------------

def test_plus_and_vw_resend_keep_vw_alive(build_lib):
    """'+' keepalive plus periodic VW resends must hold an open-ended VW
    alive -- narrowing the watchdog-reset classification to '+'/motion
    verbs must not break the legitimate keepalive path."""
    with Sim() as s:
        s.send_command("SET sTimeout=500")
        s.get_async_evts()  # drain

        resp = s.send_command("VW 200 0")
        assert "OK" in resp.upper(), f"VW not accepted; resp={resp!r}"

        # Send '+' AND resend VW every ~192ms (well under the 500ms window)
        # for 1.44s.  Drain async EVTs before each send_command call -- see
        # the comment in
        # test_get_snap_only_traffic_does_not_mask_watchdog_during_vw.
        evts = ""
        for i in range(60):
            _tick(s)
            evts += s.get_async_evts()
            if i % 8 == 0:
                s.send_command("+")
                s.send_command("VW 200 0")

        evts += s.get_async_evts()
        assert "EVT safety_stop" not in evts, (
            f"'+' plus periodic VW resend must hold VW alive; evts={evts!r}"
        )

        # The robot should still be actively driving (VW never cancelled).
        vel_l = float(s._lib.sim_get_vel_l(s._h))
        assert abs(vel_l) > 1.0, (
            f"Expected VW to still be driving after '+' + VW-resend "
            f"keepalive, got vel_l={vel_l}"
        )


# ---------------------------------------------------------------------------
# 3. Sanity check on the other side of the exemption boundary: GET/SNAP-only
#    traffic during a TIME-stopped command (D, which is watchdog-exempt via
#    its own TIME net -- see test_watchdog_exemption.py) must still complete
#    normally.  Not strictly required by the classification change, but
#    documents that narrowing the reset condition does not regress the
#    pre-existing TIME-stop exemption for commands that poll heavily.
# ---------------------------------------------------------------------------

def test_get_snap_polling_does_not_disturb_time_exempt_d(build_lib):
    """GET/SNAP polling during a TIME-stop-exempt D command must not prevent
    it from completing (the TIME-stop exemption does not depend on ambient
    traffic resetting the watchdog)."""
    with Sim() as s:
        s.send_command("SET sTimeout=500")
        s.get_async_evts()  # drain

        resp = s.send_command("D 150 150 300")
        assert "OK" in resp.upper(), f"D not accepted; resp={resp!r}"

        # Drain async EVTs before each send_command call -- see the comment
        # in test_get_snap_only_traffic_does_not_mask_watchdog_during_vw.
        evts = ""
        for i in range(120):  # ~2.9s, generous for 300mm @ 150mm/s (~2s)
            _tick(s)
            evts += s.get_async_evts()
            s.send_command("SNAP" if i % 2 == 0 else "GET ml")

        evts += s.get_async_evts()
        assert "EVT done D" in evts, (
            f"D (TIME-stop exempt) should complete despite GET/SNAP-only "
            f"polling; evts={evts!r}"
        )
        assert "safety_stop" not in evts, (
            f"Unexpected safety_stop on a TIME-stop-exempt command; "
            f"evts={evts!r}"
        )
