"""
test_motion_handlers_coverage.py — error/edge-path coverage for
MotionCommandHandlers.cpp (sprint 045 ticket 003).

Targets the verb parser ERR branches (missing required args, out-of-range
values) and the queue-path sensor= validation that the existing motion tests
do not exercise.  All commands go through the sim's wired-queue path
(sim.send_command), which is the live firmware path.

OQ-3 — `ctx->queue == nullptr` direct-call fallback paths:
  Each converter handler (handleS/T/D/G/R/TURN/RT) has an
  `if (ctx->queue != nullptr) { ... queue path ... } else { ... direct begin*() ...}`
  shape.  The sim fixture ALWAYS wires the queue (SimHandle ctor calls
  cmd.setQueue(&_queue) + robot.setMotionQueue(&_queue)), so the `else`
  (queue==nullptr) branches are DEAD-IN-PRACTICE for the sim and cannot be
  reached via sim.send_command.  Reaching them would require constructing a
  MotionCtx with queue=nullptr and calling the handler directly — there is no
  C API exposing that, and adding one purely to cover a host-only fallback (the
  firmware always wires the queue via LoopScheduler) is not warranted.  They are
  therefore documented as dead-in-practice rather than covered.  The QUEUE-path
  branches (the live path) and all parser ERR branches ARE exercised below.
"""


def _is_err(reply: str) -> bool:
    return "ERR" in reply.upper()


# ---------------------------------------------------------------------------
# parseVW ERR branches
# ---------------------------------------------------------------------------

def test_vw_no_args_errors(sim):
    """VW with no arguments → ERR (parseVW ntokens < 2 badarg branch)."""
    r = sim.send_command("VW")
    assert _is_err(r), f"Expected ERR for bare VW, got {r!r}"


def test_vw_one_arg_errors(sim):
    """VW with only one argument → ERR (parseVW ntokens < 2)."""
    r = sim.send_command("VW 100")
    assert _is_err(r), f"Expected ERR for 'VW 100' (missing omega), got {r!r}"


def test_vw_v_out_of_range_errors(sim):
    """VW with v out of [-1000,1000] → ERR range v (parseVW range branch)."""
    r = sim.send_command("VW 5000 0")
    assert _is_err(r), f"Expected ERR range for VW v=5000, got {r!r}"


def test_vw_omega_out_of_range_errors(sim):
    """VW with omega out of [-3142,3142] → ERR range omega."""
    r = sim.send_command("VW 0 9999")
    assert _is_err(r), f"Expected ERR range for VW omega=9999, got {r!r}"


# ---------------------------------------------------------------------------
# parseS / parseT / parseD ERR branches
# ---------------------------------------------------------------------------

def test_s_missing_arg_errors(sim):
    """S with one arg → ERR (parseS ntokens < 2)."""
    r = sim.send_command("S 100")
    assert _is_err(r), f"Expected ERR for 'S 100', got {r!r}"


def test_s_left_out_of_range_errors(sim):
    """S with l out of [-1000,1000] → ERR range l."""
    r = sim.send_command("S 5000 200")
    assert _is_err(r), f"Expected ERR range for S l=5000, got {r!r}"


def test_s_right_out_of_range_errors(sim):
    """S with r out of range → ERR range r (the second range check in parseS)."""
    r = sim.send_command("S 200 5000")
    assert _is_err(r), f"Expected ERR range for S r=5000, got {r!r}"


def test_t_missing_ms_errors(sim):
    """T with only two args (no ms) → ERR (parseT ntokens < 3)."""
    r = sim.send_command("T 200 200")
    assert _is_err(r), f"Expected ERR for 'T 200 200' (missing ms), got {r!r}"


def test_t_ms_out_of_range_errors(sim):
    """T with ms out of [1,30000] → ERR range ms."""
    r = sim.send_command("T 200 200 99999")
    assert _is_err(r), f"Expected ERR range for T ms=99999, got {r!r}"


def test_d_missing_mm_errors(sim):
    """D with only two args (no mm) → ERR (parseD ntokens < 3)."""
    r = sim.send_command("D 200 200")
    assert _is_err(r), f"Expected ERR for 'D 200 200' (missing mm), got {r!r}"


def test_d_mm_out_of_range_errors(sim):
    """D with mm out of [1,10000] → ERR range mm."""
    r = sim.send_command("D 200 200 99999")
    assert _is_err(r), f"Expected ERR range for D mm=99999, got {r!r}"


# ---------------------------------------------------------------------------
# parseG / parseR / parseTURN / parseRT ERR branches
# ---------------------------------------------------------------------------

def test_g_missing_args_errors(sim):
    """G with two args (no speed) → ERR (parseG ntokens < 3)."""
    r = sim.send_command("G 100 100")
    assert _is_err(r), f"Expected ERR for 'G 100 100' (missing speed), got {r!r}"


def test_g_speed_out_of_range_errors(sim):
    """G with speed out of [1,1000] → ERR range speed."""
    r = sim.send_command("G 100 100 5000")
    assert _is_err(r), f"Expected ERR range for G speed=5000, got {r!r}"


def test_r_missing_radius_errors(sim):
    """R with one arg (no radius) → ERR (parseR ntokens < 2)."""
    r = sim.send_command("R 200")
    assert _is_err(r), f"Expected ERR for 'R 200' (missing radius), got {r!r}"


def test_r_speed_out_of_range_errors(sim):
    """R with speed out of [-1000,1000] → ERR range speed."""
    r = sim.send_command("R 5000 100")
    assert _is_err(r), f"Expected ERR range for R speed=5000, got {r!r}"


def test_turn_no_args_errors(sim):
    """TURN with no args → ERR (parseTURN ntokens < 1)."""
    r = sim.send_command("TURN")
    assert _is_err(r), f"Expected ERR for bare TURN, got {r!r}"


def test_turn_heading_out_of_range_errors(sim):
    """TURN with heading out of [-18000,18000] → ERR range heading."""
    r = sim.send_command("TURN 99999")
    assert _is_err(r), f"Expected ERR range for TURN heading=99999, got {r!r}"


def test_turn_eps_out_of_range_errors(sim):
    """TURN with eps out of [10,1800] → ERR range eps (parseTURN eps branch)."""
    r = sim.send_command("TURN 9000 eps=9999")
    assert _is_err(r), f"Expected ERR range for TURN eps=9999, got {r!r}"


def test_rt_no_args_errors(sim):
    """RT with no args → ERR (parseRT ntokens < 1)."""
    r = sim.send_command("RT")
    assert _is_err(r), f"Expected ERR for bare RT, got {r!r}"


def test_rt_out_of_range_errors(sim):
    """RT with deg out of [-180000,180000] → ERR range deg."""
    r = sim.send_command("RT 999999")
    assert _is_err(r), f"Expected ERR range for RT deg=999999, got {r!r}"


# ---------------------------------------------------------------------------
# mc_parseSensorToken failure paths — bad channel name / bad op string.
# Validated on the queue path (handleT/D/TURN call mc_parseSensorToken before
# replying OK; a parse failure → ERR badarg sensor).
# ---------------------------------------------------------------------------

def test_t_bad_sensor_channel_errors(sim):
    """T ... sensor=<badchan>:ge:100 → ERR (mc_parseSensorToken unknown channel)."""
    r = sim.send_command("T 200 200 2000 sensor=notachan:ge:100")
    assert _is_err(r), f"Expected ERR for bad sensor channel, got {r!r}"


def test_t_bad_sensor_op_errors(sim):
    """T ... sensor=line0:<badop>:100 → ERR (mc_parseSensorToken unknown op)."""
    r = sim.send_command("T 200 200 2000 sensor=line0:zz:100")
    assert _is_err(r), f"Expected ERR for bad sensor op, got {r!r}"


def test_t_malformed_sensor_no_colon_errors(sim):
    """T ... sensor=line0 (no colons) → ERR (mc_parseSensorToken no colon1)."""
    r = sim.send_command("T 200 200 2000 sensor=line0")
    assert _is_err(r), f"Expected ERR for sensor token without colons, got {r!r}"


def test_d_bad_sensor_channel_errors(sim):
    """D ... sensor=<badchan>:le:50 → ERR (D path mc_parseSensorToken)."""
    r = sim.send_command("D 200 200 500 sensor=nope:le:50")
    assert _is_err(r), f"Expected ERR for bad sensor channel on D, got {r!r}"


def test_turn_bad_sensor_errors(sim):
    """TURN ... sensor=<badchan>:ge:100 → ERR (TURN path mc_parseSensorToken)."""
    r = sim.send_command("TURN 9000 sensor=bogus:ge:100")
    assert _is_err(r), f"Expected ERR for bad sensor channel on TURN, got {r!r}"


# ---------------------------------------------------------------------------
# Valid sensor= token is accepted (the success branch of mc_parseSensorToken on
# the queue path: validate → OK, then push VW).
# ---------------------------------------------------------------------------

def test_t_valid_sensor_token_accepted(sim):
    """T ... sensor=colorR:ge:200 with a VALID token replies OK (not ERR).

    Exercises the success path of mc_parseSensorToken on the queue path
    (channel colorR=4 found, op ge valid) — the validate-then-OK branch in
    handleT before pushVW.
    """
    r = sim.send_command("T 200 200 2000 sensor=colorR:ge:200")
    assert not _is_err(r), f"Valid sensor= token wrongly rejected: {r!r}"
    assert "OK" in r.upper(), f"Expected OK for valid T sensor= token, got {r!r}"
