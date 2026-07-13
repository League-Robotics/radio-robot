"""
test_system_commands_coverage.py — gap-closer coverage for SystemCommands.cpp
(sprint 045 ticket 004 follow-on: close the 85% simulatable-coverage gap).

Targets the simulatable error/edge branches of SystemCommands that the 045-004
happy-path tests did not reach: the full ID response, RF (noradio in sim, since
sched is null), the SAFE on/off/numeric branches, and the ZERO / STREAM / HALT
sub-verb badarg branches.  These are all reachable through the sim's wired-queue
command path; none are inside `#ifndef HOST_BUILD`.
"""


def _is_err(reply: str) -> bool:
    return "ERR" in reply.upper()


# ---------------------------------------------------------------------------
# ID — full identification response (handleId, including the caps builder).
# ---------------------------------------------------------------------------

def test_id_reports_full_identity(sim):
    """ID returns model/name/serial/fw/proto/caps (handleId + addCap loop)."""
    r = sim.send_command("ID")
    assert "model=" in r and "fw=" in r and "proto=" in r and "caps=" in r, (
        f"ID response missing fields: {r!r}"
    )


def test_id_with_corr_id_echoes_it(sim):
    """ID #42 routes through the corrId branch of handleId."""
    r = sim.send_command("ID #42")
    assert "#42" in r, f"ID did not echo the corrId: {r!r}"


# ---------------------------------------------------------------------------
# RF — radio channel set; in the sim sched is null → noradio ERR (handleRf).
# ---------------------------------------------------------------------------

def test_rf_noradio_in_sim(sim):
    """RF <ch> returns ERR noradio in the sim (LoopScheduler is null)."""
    r = sim.send_command("RF 5")
    assert _is_err(r) and "noradio" in r, f"RF should ERR noradio in sim: {r!r}"


def test_rf_no_arg_noradio(sim):
    """RF with no arg also reaches handleRf (sched null → noradio)."""
    r = sim.send_command("RF")
    assert _is_err(r), f"RF with no arg should ERR in sim: {r!r}"


# ---------------------------------------------------------------------------
# SAFE — on / off / numeric branches (handleSafe).
# ---------------------------------------------------------------------------

def test_safe_off_branch(sim):
    """SAFE off disables the watchdog (handleSafe 'off' branch)."""
    r = sim.send_command("SAFE off")
    assert "OK" in r.upper() and "off" in r.lower(), f"SAFE off: {r!r}"


def test_safe_on_with_timeout_branch(sim):
    """SAFE on 3000 enables the watchdog with a timeout (handleSafe 'on' branch)."""
    r = sim.send_command("SAFE on 3000")
    assert "OK" in r.upper() and "3000" in r, f"SAFE on 3000: {r!r}"


def test_safe_numeric_off_branch(sim):
    """SAFE 0 takes the numeric one-shot-disable path (handleSafe numeric branch)."""
    r = sim.send_command("SAFE 0")
    assert "OK" in r.upper(), f"SAFE 0: {r!r}"


def test_safe_numeric_on_branch(sim):
    """SAFE 2500 takes the numeric enable-with-timeout path."""
    r = sim.send_command("SAFE 2500")
    assert "OK" in r.upper(), f"SAFE 2500: {r!r}"


# ---------------------------------------------------------------------------
# ZERO — badarg branch (parseZero: no recognized token).
# ---------------------------------------------------------------------------

def test_zero_no_token_errors(sim):
    """ZERO with no recognized token → ERR badarg (parseZero)."""
    r = sim.send_command("ZERO")
    assert _is_err(r), f"bare ZERO should ERR badarg: {r!r}"


def test_zero_bad_token_errors(sim):
    """ZERO bogus → ERR badarg (parseZero unrecognized token)."""
    r = sim.send_command("ZERO bogus")
    assert _is_err(r), f"ZERO bogus should ERR badarg: {r!r}"


def test_zero_t_and_d_baselines(sim):
    """ZERO T and ZERO D reset the HALT TIME/DIST baselines (parseZero hasT/hasD)."""
    rt = sim.send_command("ZERO T")
    assert "OK" in rt.upper() and "T" in rt, f"ZERO T: {rt!r}"
    rd = sim.send_command("ZERO D")
    assert "OK" in rd.upper() and "D" in rd, f"ZERO D: {rd!r}"


# ---------------------------------------------------------------------------
# STREAM — badarg branch (no positional period and no fields=).
# ---------------------------------------------------------------------------

def test_stream_no_arg_errors(sim):
    """STREAM with no args → ERR badarg (handleStream 'usage: STREAM <ms>')."""
    r = sim.send_command("STREAM")
    assert _is_err(r), f"bare STREAM should ERR badarg: {r!r}"


# ---------------------------------------------------------------------------
# HALT — sub-verb badarg branches (handleHalt usage errors).
# ---------------------------------------------------------------------------

def test_halt_time_missing_ms_errors(sim):
    """HALT TIME with no ms → ERR badarg (HALT TIME usage branch)."""
    r = sim.send_command("HALT TIME")
    assert _is_err(r), f"HALT TIME (no ms) should ERR: {r!r}"


def test_halt_color_missing_args_errors(sim):
    """HALT COLOR with too few args → ERR badarg (HALT COLOR usage branch)."""
    r = sim.send_command("HALT COLOR 120")
    assert _is_err(r), f"HALT COLOR (too few args) should ERR: {r!r}"


def test_halt_line_missing_args_errors(sim):
    """HALT LINE with too few args → ERR badarg (HALT LINE usage branch)."""
    r = sim.send_command("HALT LINE")
    assert _is_err(r), f"HALT LINE (too few args) should ERR: {r!r}"


def test_halt_unknown_subverb_errors(sim):
    """HALT BOGUS → ERR badarg (HALT unknown sub-verb usage branch)."""
    r = sim.send_command("HALT BOGUS")
    assert _is_err(r), f"HALT BOGUS should ERR: {r!r}"


def test_halt_dist_soft_registers(sim):
    """HALT DIST 500 SOFT registers a soft-style distance condition (SOFT branch)."""
    sim.send_command("ZERO D")
    r = sim.send_command("HALT DIST 500 SOFT")
    assert "OK" in r.upper() and "id=" in r, f"HALT DIST SOFT: {r!r}"
