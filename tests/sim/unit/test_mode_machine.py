"""Off-hardware acceptance proof for ticket 084-005 (SUC-004): extends TLM's
``mode=`` field (``source/commands/telemetry_commands.cpp``) from sprint
082's minimal ``I``/``S`` (a direct ``drivetrain.active()`` read) to the full
``I``/``S``/``T``/``D``/``G`` vocabulary, now sourced exclusively from
``Subsystems::Planner::state().mode`` (architecture-update.md (084)
Decision 6).

Drives ``libfirmware_host`` through the full wire dispatch (``Sim.command()``)
-- ``CommandProcessor`` -> ``source/commands/motion_commands.cpp`` ->
``MotionLoopState``'s outbox -> ``dev_loop.cpp``'s drain step ->
``Subsystems::Planner`` -> the simulated plant -- the SAME path
``test_motion_commands.py``/``test_motion_commands_arc_turn.py``/
``test_motion_commands_goto.py`` already exercise; this file adds no new
motion behavior, only ``mode=`` assertions layered on top of command
sequences already proven correct (by true-pose/EVT assertions) in those
files. Durations/parameters below are reused from those files' own tuned
values rather than re-derived here.

Every completion check below is POLLED via ``SNAP`` alone -- this file
deliberately never calls ``sim.get_async_evts()`` before asserting the `I`
transition, proving the ticket's acceptance criterion that `mode=` returns
to `I` independent of whether the host ever drains the corresponding
``EVT done``/``safety_stop`` line. (Periodic ``STREAM``-driven frames emitted
during a bare ``tick_for()`` window are not separately observable through
this ABI -- see ``test_tlm_stream_snap.py``'s own note on this -- so
"polling" here means repeated ``SNAP`` calls, exactly like every other
``mode=`` test in this directory.)

Approved mapping under test (this ticket's own acceptance table / Decision
6; see ``docs/protocol-v2.md`` section 8's ``mode=`` verb-sharing note):
  I -- no active Planner command (boot, after STOP, after any completion)
  S -- ``S``, a bare ``R`` (no ``stop=``)
  T -- ``T``, an ``R`` WITH a ``stop=`` clause, ``TURN``, ``RT``
  D -- ``D``
  G -- ``G``
"""


def _parse_tlm(line: str) -> dict[str, str]:
    """Parse one "TLM t=... mode=... ..." wire line into a key->value dict.

    Local, small, deliberately duplicated per test file -- mirrors this
    directory's existing precedent (e.g. ``test_tlm_stream_snap.py``'s own
    copy)."""
    parts = line.strip().split()
    assert parts[0] == "TLM", f"not a TLM line: {line!r}"
    return dict(p.split("=", 1) for p in parts[1:])


def _mode(sim) -> str:
    """Issue SNAP and return just its mode= value -- the one field every
    test below cares about."""
    reply = sim.command("SNAP").strip()
    lines = reply.splitlines()
    assert len(lines) == 1, f"expected exactly one TLM line from SNAP, got: {reply!r}"
    return _parse_tlm(lines[0])["mode"]


def test_mode_is_idle_before_any_motion_command(sim):
    assert _mode(sim) == "I"


def test_mode_s_streams_and_returns_to_idle_after_stop(sim):
    reply = sim.command("S 120 120")
    assert reply.strip() == "OK drive l=120 r=120"
    assert _mode(sim) == "S"

    sim.tick_for(200)
    assert _mode(sim) == "S", "S is open-ended -- mode stays S until STOP"

    reply = sim.command("STOP")
    assert reply.strip() == "OK stop"
    # Polled via SNAP alone -- no get_async_evts() call anywhere in this
    # test (STOP itself emits no EVT to begin with; see
    # test_motion_commands.py's test_stop_halts_immediately_with_no_evt).
    assert _mode(sim) == "I"


def test_mode_t_reports_timed_and_returns_to_idle_on_completion(sim):
    reply = sim.command("T 150 150 1000")
    assert reply.strip() == "OK drive l=150 r=150 ms=1000"
    assert _mode(sim) == "T"

    sim.tick_for(200)   # well short of the 1 s duration
    assert _mode(sim) == "T"

    sim.tick_for(2000)  # matches test_motion_commands.py's own margin
    assert _mode(sim) == "I", "polled via SNAP alone -- no EVT drained"


def test_mode_d_reports_distance_and_returns_to_idle_on_completion(sim):
    reply = sim.command("D 200 200 500")
    assert reply.strip() == "OK drive l=200 r=200 mm=500"
    assert _mode(sim) == "D"

    sim.tick_for(3000)  # matches test_motion_commands.py's own margin
    assert _mode(sim) == "I", "polled via SNAP alone -- no EVT drained"


def test_mode_bare_r_reports_streaming_and_returns_to_idle_after_stop(sim):
    """A bare R (no stop= clause) is open-ended, exactly like S -- Decision
    6 folds it into the SAME DriveMode::STREAMING/'S' bucket."""
    reply = sim.command("R 150 500")
    assert reply.strip() == "OK arc speed=150 radius=500"
    assert _mode(sim) == "S"

    sim.tick_for(2000)  # matches test_motion_commands_arc_turn.py's own margin
    assert _mode(sim) == "S", "bare R never self-terminates"

    reply = sim.command("STOP")
    assert reply.strip() == "OK stop"
    assert _mode(sim) == "I"


def test_mode_bounded_r_reports_timed_and_returns_to_idle_on_completion(sim):
    """An R WITH a stop= clause self-terminates -- Decision 6 folds it into
    the SAME DriveMode::TIMED/'T' bucket a plain T uses."""
    reply = sim.command("R 200 500 stop=d:400")
    assert reply.strip() == "OK arc speed=200 radius=500"
    assert _mode(sim) == "T"

    sim.tick_for(3000)  # matches test_motion_commands_arc_turn.py's own margin
    assert _mode(sim) == "I", "polled via SNAP alone -- no EVT drained"


def test_mode_turn_reports_timed_and_returns_to_idle_on_completion(sim):
    """TURN always carries its own built-in HEADING stop -- always
    self-terminating, always the TIMED/'T' bucket (never a dedicated
    character -- Decision 6)."""
    reply = sim.command("TURN 9000")
    assert reply.strip() == "OK turn heading=9000 eps=300"
    assert _mode(sim) == "T"

    sim.tick_for(3000)  # matches test_motion_commands_arc_turn.py's own margin
    assert _mode(sim) == "I", "polled via SNAP alone -- no EVT drained"


def test_mode_rt_reports_timed_and_returns_to_idle_on_completion(sim):
    """RT always carries its own built-in ROTATION stop -- same reasoning
    as TURN above."""
    reply = sim.command("RT 9000")
    assert reply.strip() == "OK rt rot=9000"
    assert _mode(sim) == "T"

    sim.tick_for(3000)  # matches test_motion_commands_arc_turn.py's own margin
    assert _mode(sim) == "I", "polled via SNAP alone -- no EVT drained"


def test_mode_g_reports_go_to_and_returns_to_idle_on_completion(sim):
    reply = sim.command("G 300 0 200")
    assert reply.strip() == "OK goto x=300 y=0 speed=200"
    assert _mode(sim) == "G"

    sim.tick_for(6000)  # matches test_motion_commands_goto.py's own margin
    assert _mode(sim) == "I", "polled via SNAP alone -- no EVT drained"


def test_mode_via_stream_immediate_reply_reflects_same_source(sim):
    """STREAM's own reply carries an immediate TLM frame (telemetry_commands.h's
    documented "first STREAM after boot emits right away" behavior) -- proves
    mode= is consistent whether observed via SNAP or STREAM's own reply, not
    just a SNAP-specific code path."""
    reply = sim.command("T 150 150 1000")
    assert reply.strip() == "OK drive l=150 r=150 ms=1000"

    stream_reply = sim.command("STREAM 50").strip()
    lines = stream_reply.splitlines()
    assert lines[0] == "OK stream period=50"
    assert len(lines) == 2, f"expected STREAM's own reply to carry one immediate TLM frame: {stream_reply!r}"
    assert _parse_tlm(lines[1])["mode"] == "T"
