"""
test_065_001_stop_clause_overflow.py — Regression tests for ticket 065-001 /
CR-01: D-command stop-clause double-booking and non-fatal addStop overflow.

Background (see clasi/sprints/065-.../issues/stop-clause-overflow-aborts-process.md
and architecture-update.md Step 4-5 item 1):

  Pre-fix, `handleD` (MotionCommands.cpp) pre-populated gr.stops[0] with a
  makeDistanceStop(mm) *in addition to* the DISTANCE+TIME pair that
  distanceDrive() -> beginDistance() already installs internally.
  Superstructure::requestGoal's Goal::DISTANCE case then unconditionally
  re-added every entry of gr.stops[] on top of those 2 internal stops.  Net
  effect: plain D installed 3 stops (1 wasted duplicate); D with 2 wire
  stop=/sensor= clauses installed 5 against MotionCommand::kMaxStopConds==4,
  and MotionCommand::addStop()'s live assert(false) fired.  The sim build sets
  no NDEBUG, so that assert ABORTS THE WHOLE PYTHON PROCESS hosting the sim
  (this pytest run, or the TestGUI's SimTransport tick-thread) -- not a normal
  test failure.

  The fix has two independent layers:
    1. handleD no longer pre-populates gr.stops[0] -- gr.stops[] carries only
       wire-supplied stop=/sensor= clauses, so plain D installs exactly 2
       stops (DISTANCE+TIME, no duplicate) and D with 2 wire clauses installs
       exactly 4 (at the ceiling, not over it).
    2. MotionCommand::addStop() returns false instead of asserting on
       overflow; Superstructure::requestGoal's DISTANCE and VELOCITY cases
       check that return value and, on the first false, HARD-cancel the
       just-started command and reply a wire-visible "ERR stopoverflow"
       instead of continuing with silently-incomplete stop coverage.

  Tests below exercise the actual compiled C++ (via the `firmware.Sim` ctypes
  wrapper), not a Python mirror -- if the live assert regressed, the test
  process would abort (SIGABRT) rather than fail cleanly, which is itself the
  strongest possible signal something is wrong here.

  Update (sprint 072, ticket 002): `beginDistance()` now installs a THIRD
  built-in stop condition (`SAFETY_MARGIN`, the runaway safety net --
  architecture-update.md Decision 2), so a plain D now installs 3 stops
  (DISTANCE+TIME+SAFETY_MARGIN), not 2. This shrinks the wire-clause budget
  from 2 down to 1 (3 internal + 1 wire == 4, still exactly at
  `kMaxStopConds`) -- an explicitly-anticipated consequence of 072-002's own
  Design Rationale ("headroom for one more... out of this sprint's scope").
  The two "two wire clauses fit" tests below are updated to one wire clause
  each to match the new (still-exactly-at-the-ceiling) budget; their actual
  intent (prove no crash / prove the wire clause governs when tripped early)
  is unchanged. `test_d_three_wire_clauses_overflow_is_recoverable_err_not_crash`
  needed no change -- it already overflowed before 072-002 and overflows more
  now.
"""
import ctypes

from firmware import Sim

TICK_STEP_MS = 24


def _tick_collect(s: Sim, n: int) -> str:
    """Tick n times, accumulating async events. Returns all EVT strings."""
    evts = ""
    for _ in range(n):
        s._lib.sim_tick(s._h, ctypes.c_uint32(s._t))
        s._t += TICK_STEP_MS
        evts += s.get_async_evts()
    return evts


# ---------------------------------------------------------------------------
# Sprint's exact regression scenario: D with 1 wire clause (sensor=).
#
# Pre-065-001-fix total stop count: 2 internal (DISTANCE+TIME) + 1 duplicate
# DISTANCE + 2 wire (TIME+SENSOR) == 5 > kMaxStopConds(4) -> live assert ->
# process abort.  Post-065-001-fix, pre-072-002: 2 internal + 2 wire == 4,
# exactly at the ceiling, no overflow.  Post-072-002 (this sprint):
# beginDistance() installs a THIRD internal stop (SAFETY_MARGIN, the runaway
# safety net), so the budget is now 3 internal + 1 wire == 4 -- still exactly
# at the ceiling, just with one fewer wire-clause slot (an explicitly
# anticipated consequence of 072-002's Design Rationale Decision 2). The wire
# clause never fires (the sensor is never tripped), so the DISTANCE stop --
# the earliest-firing condition -- governs.
# ---------------------------------------------------------------------------

def test_d_two_wire_clauses_completes_without_crash(build_lib):
    """D 150 150 300 sensor=line0:ge:500 must not crash the sim and must
    honor the earliest-firing clause (DISTANCE).

    Only 1 wire clause now (072-002 shrank the budget to 3 internal + 1 wire
    == 4; see module docstring) -- the test's original 2-wire-clause name is
    kept for continuity with the ticket 065-001 history it regression-guards."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.init_line_sensor()
        s.set_line_values(0, 0, 0, 0)
        _tick_collect(s, 3)
        s.get_async_evts()  # drain

        resp = s.send_command("D 150 150 300 sensor=line0:ge:500")
        assert "OK drive" in resp, f"D command not accepted; resp={resp!r}"
        assert "ERR stopoverflow" not in resp, (
            f"Unexpected overflow with only 1 wire clause; resp={resp!r}"
        )

        evts = _tick_collect(s, 120)  # ~2.9s, generous for 300mm @ 150mm/s (~2s)

        assert "EVT done D" in evts, (
            f"D did not complete; evts={evts!r}"
        )
        assert "reason=dist" in evts, (
            f"Expected the DISTANCE clause (earliest-firing) to govern, not "
            f"the un-tripped sensor; evts={evts!r}"
        )
        assert "ERR stopoverflow" not in evts, (
            f"Unexpected overflow with only 1 wire clause; evts={evts!r}"
        )


def test_d_sensor_clause_wins_when_tripped_early(build_lib):
    """The wire sensor= clause must actually be installed and able to govern
    (proving it was not silently dropped or crowded out by the old duplicate
    DISTANCE stop) -- "whichever clause fires first" per the issue's own
    acceptance criterion.

    Uses a large DISTANCE target (3000mm) so the DISTANCE stop cannot race
    the early sensor trip. Only 1 wire clause (see module docstring, 072-002).
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.init_line_sensor()
        s.set_line_values(0, 0, 0, 0)
        _tick_collect(s, 3)
        s.get_async_evts()  # drain

        resp = s.send_command("D 150 150 3000 sensor=line0:ge:500")
        assert "OK drive" in resp, f"D command not accepted; resp={resp!r}"

        _tick_collect(s, 3)  # let the command start
        s.set_line_values(800, 0, 0, 0)  # trip line0 above 500
        evts = _tick_collect(s, 40)  # ~1s -- well before the ~20s DISTANCE target

        assert "EVT done D" in evts, f"Expected sensor stop to fire; evts={evts!r}"
        assert "reason=line0" in evts, (
            f"Expected the wire sensor= clause to govern; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# Defense-in-depth: a D with enough wire clauses to still overflow after the
# duplicate-stop fix (2 internal + 3 wire == 5 > kMaxStopConds(4)).  Must be
# cancelled cleanly with a wire-visible "ERR stopoverflow" -- never an assert,
# never a crash, never a command left silently running with truncated stop
# coverage.
# ---------------------------------------------------------------------------

def test_d_three_wire_clauses_overflow_is_recoverable_err_not_crash(build_lib):
    """D 150 150 300 with 3 stop= clauses overflows kMaxStopConds even after
    the duplicate fix; must reply ERR stopoverflow and cancel, never crash."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.get_async_evts()  # drain

        resp = s.send_command(
            "D 150 150 300 stop=t:1000 stop=t:2000 stop=t:3000"
        )

        assert "ERR stopoverflow" in resp, (
            f"Expected 'ERR stopoverflow' on addStop overflow; resp={resp!r}"
        )

        # The command must have been cancelled outright -- it must NEVER
        # later emit "EVT done D" (that would mean it kept running with
        # silently-truncated stop coverage instead of being cancelled).
        more_evts = _tick_collect(s, 150)  # ~3.6s -- longer than 300mm @ 150mm/s
        assert "EVT done D" not in more_evts, (
            f"Cancelled command should never emit EVT done; evts={more_evts!r}"
        )

        # Robot must be left stopped -- encoders should not advance as if the
        # DISTANCE travel had continued after cancellation.
        enc_l = float(s._lib.sim_get_enc_l(s._h))
        enc_r = float(s._lib.sim_get_enc_r(s._h))
        assert abs(enc_l) < 20.0 and abs(enc_r) < 20.0, (
            f"Robot kept driving after overflow cancel; "
            f"enc_l={enc_l:.1f} enc_r={enc_r:.1f}"
        )
