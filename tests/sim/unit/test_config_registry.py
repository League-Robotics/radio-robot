"""Off-hardware acceptance proof for ticket 084-006 (SUC-005): registers the
top-level ``SET``/``GET`` config-plane verbs (``source/commands/
config_commands.cpp``), mapping the deliberately-scoped key table
(architecture-update.md (084) Decision 2) onto ``msg::DrivetrainConfig``/
``msg::MotorConfig``/``msg::PlannerConfig``/ticket 002's streaming-drive
watchdog window, re-propagated via ``Drivetrain::configure()``/
``PoseEstimator::configure()``/``Planner::configure()``/the bound pair's
``Hal::Motor::configure()``.

Like ``test_motion_commands.py``, this drives ``libfirmware_host`` through
the full wire dispatch (``Sim.command()``) -- ``CommandProcessor`` ->
``source/commands/config_commands.cpp`` -> the real subsystems/motors.
"""

import pytest

# Boot defaults (tests/_infra/sim/sim_api.cpp's defaultMotorConfigSet()/
# defaultSimDrivetrainConfig()/defaultSimPlannerConfig() -- NOT the real
# firmware's boot_config.cpp values, which only main.cpp uses):
#   tw=150 (Hal::PhysicsWorld::kDefaultTrackwidth), ml=mr=0.000 (travel_calib
#   left unset in the sim's own boot config), pid.kp=0.002/ki=0.002/
#   kff=0.004 (0.0022/0.0018/0.0038 rounded to 3 decimals), pid.iMax=0.300,
#   pid.kaw=0.000, rotSlip=0.000 (unset sentinel), ekf*=0.000 (unset),
#   minSpeed=0, sTimeout=500 (StreamingDriveWatchdog::kDefaultWindow).
_BOOT_DUMP = (
    "CFG tw=150 ml=0.000 mr=0.000 pid.kp=0.002 pid.ki=0.002 pid.kff=0.004 "
    "pid.iMax=0.300 pid.kaw=0.000 rotSlip=0.000 ekfQxy=0.000 ekfQtheta=0.000 "
    "ekfROtosXy=0.000 ekfROtosTheta=0.000 minSpeed=0 sTimeout=500"
)


def test_get_with_no_args_dumps_every_registered_key_with_boot_defaults(sim):
    assert sim.command("GET").strip() == _BOOT_DUMP


# ---------------------------------------------------------------------------
# Round-trip every implemented key: SET <key>=<value>, then GET <key> reads
# the same value back, formatted per docs/protocol-v2.md §7's wire spec
# (float keys %.3f, tw/minSpeed/sTimeout plain integer text).
# ---------------------------------------------------------------------------
_ROUND_TRIP_CASES = [
    ("tw", "130", "tw=130"),
    ("ml", "0.750", "ml=0.750"),
    ("mr", "0.640", "mr=0.640"),
    ("pid.kp", "12.5", "pid.kp=12.500"),
    ("pid.ki", "0.250", "pid.ki=0.250"),
    ("pid.kff", "0.100", "pid.kff=0.100"),
    ("pid.iMax", "5.000", "pid.iMax=5.000"),
    ("pid.kaw", "2.000", "pid.kaw=2.000"),
    ("rotSlip", "0.75", "rotSlip=0.750"),
    ("ekfQxy", "100.000", "ekfQxy=100.000"),
    ("ekfQtheta", "0.250", "ekfQtheta=0.250"),
    ("ekfROtosXy", "40.000", "ekfROtosXy=40.000"),
    ("ekfROtosTheta", "0.015", "ekfROtosTheta=0.015"),
    ("minSpeed", "80", "minSpeed=80"),
    ("sTimeout", "750", "sTimeout=750"),
]


@pytest.mark.parametrize("key,value,expected_kv", _ROUND_TRIP_CASES)
def test_set_then_get_round_trips_every_implemented_key(sim, key, value, expected_kv):
    reply = sim.command(f"SET {key}={value}")
    assert reply.strip() == f"OK set {expected_kv}"

    reply = sim.command(f"GET {key}")
    assert reply.strip() == f"CFG {expected_kv}"


def test_set_tw_130_then_get_tw_round_trips_and_visibly_changes_arc_geometry(sim):
    """The sprint's own headline acceptance example (ticket 006's acceptance
    criteria): SET tw=130, GET tw round-trips to 130, and the trackwidth
    change is visible in the Drivetrain's own commanded wheel-target split
    for an UNCHANGED body twist -- BodyKinematics::inverse(v, omega, tw,
    ...): wL = v - omega*tw/2, wR = v + omega*tw/2. With v=100, omega=1.0:
    tw=150 (boot) -> (25, 175); tw=130 -> (35, 165). Drivetrain::state()'s
    vel= reports these PRE-governor commanded targets directly (drivetrain.
    cpp's commandedWheelTargets()), recomputed fresh from whatever twist is
    currently staged -- no need to re-issue DEV DT VW after the SET."""
    reply = sim.command("DEV DT VW 100 0 1.0")
    assert reply.strip() == "OK DEV DT vx=100.0 vy=0.0 omega=1.000"

    before = sim.command("DEV DT STATE")
    assert "vel=25.0,175.0" in before

    reply = sim.command("SET tw=130")
    assert reply.strip() == "OK set tw=130"

    reply = sim.command("GET tw")
    assert reply.strip() == "CFG tw=130"

    after = sim.command("DEV DT STATE")
    assert "vel=35.0,165.0" in after


def test_dev_dt_cfg_trackwidth_visibly_changes_arc_geometry(sim):
    """088-008: the DEV-plane config surface (`DEV DT CFG`,
    dev_commands.cpp's applyDrivetrainCfgKey/handleDevDtCfg) is a SEPARATE
    code path from SET's config plane (config_commands.cpp) -- this file's
    own test_set_tw_130_then_get_tw_round_trips_and_visibly_changes_arc_
    geometry above only proves the SET path reaches Drivetrain's kinematics.
    This closes the gap for the bench-diagnostic DEV plane: `DEV DT CFG
    trackwidth=<n>` must reach the SAME DrivetrainConfig.trackwidth field
    (confirmed via `GET tw` -- the SET-plane's own read-back -- reflecting
    the new value) AND visibly split the commanded wheel targets by it,
    identical to the SET-driven test's own assertions."""
    reply = sim.command("DEV DT VW 100 0 1.0")
    assert reply.strip() == "OK DEV DT vx=100.0 vy=0.0 omega=1.000"

    before = sim.command("DEV DT STATE")
    assert "vel=25.0,175.0" in before

    reply = sim.command("DEV DT CFG trackwidth=130")
    assert reply.strip() == "OK DEV DT trackwidth=130.0"

    # DEV DT CFG's ConfigDelta lands on the SAME DrivetrainConfig field
    # SET tw= writes -- GET tw must reflect it too, proving the two config
    # surfaces converge on one subsystem config, not two independently
    # shadowed copies.
    reply = sim.command("GET tw")
    assert reply.strip() == "CFG tw=130"

    after = sim.command("DEV DT STATE")
    assert "vel=35.0,165.0" in after


def test_set_atomic_failure_applies_neither_key(sim):
    """SET pid.kp=1.5 tw=0 (tw=0 is invalid -- docs/protocol-v2.md §7's own
    documented invariant, division by zero in odometry arc/heading math):
    applies NEITHER key and returns ERR badval tw=0 -- pid.kp must NOT have
    been committed either, confirmed by a follow-up GET still showing the
    boot default."""
    reply = sim.command("SET pid.kp=1.5 tw=0")
    assert reply.strip() == "ERR badval tw=0"

    reply = sim.command("GET pid.kp tw")
    assert reply.strip() == "CFG pid.kp=0.002 tw=150"


def _snap_encpose_heading(sim) -> int:  # [centidegrees]
    """Issue SNAP and pull encpose='s heading component (docs/protocol-v2.md
    §8: `encpose=x,y,h`, h in centidegrees) -- local, small, deliberately
    duplicated per test-file precedent (test_tlm_stream_snap.py's own
    ``_parse_tlm``/``_snap``)."""
    reply = sim.command("SNAP").strip()
    line = reply.splitlines()[0]
    parts = line.split()
    fields = dict(p.split("=", 1) for p in parts[1:])
    return int(fields["encpose"].split(",")[2])


def test_set_rotslip_visibly_changes_pose_estimator_dead_reckoning(sim):
    """rotSlip is not just accepted and stored -- it visibly changes
    PoseEstimator's own encoder-only dead-reckoning heading computation
    (pose_estimator.cpp: dTheta = ((dR-dL)/trackwidth) * effectiveSlip(
    rotationalSlip)). Two identical in-place-spin segments (same DEV DT VW
    command, same duration -- so the SAME encoder differential accrues each
    time) should accumulate roughly HALF the heading delta once rotSlip is
    set to 0.5 partway through, since effectiveSlip(0.5) == 0.5 exactly
    (docs/protocol-v2.md §7's documented invariant range)."""
    sim.command("DEV DT PORTS 1 2")

    sim.command("DEV DT VW 0 0 1.0")
    sim.tick_for(500)
    h_default = _snap_encpose_heading(sim)
    assert h_default > 0, "expected a nonzero heading delta from the first spin segment"

    reply = sim.command("SET rotSlip=0.5")
    assert reply.strip() == "OK set rotSlip=0.500"

    sim.command("DEV DT VW 0 0 1.0")
    sim.tick_for(500)
    h_after_second_segment = _snap_encpose_heading(sim)
    delta_second_segment = h_after_second_segment - h_default

    # Generous bounds (0.3x-0.7x of the first, unscaled segment) -- this is a
    # visibility proof, not a precision calibration check; the encoder-arc
    # midpoint integration and heading accumulation introduce some drift
    # away from an exact 0.5x scaling.
    assert 0.3 * h_default < delta_second_segment < 0.7 * h_default, (
        f"expected the rotSlip=0.5 segment's heading delta ({delta_second_segment}) "
        f"to be roughly half the unscaled segment's ({h_default})"
    )


def test_set_rejects_out_of_range_rotslip_atomically(sim):
    """rotSlip's own documented invariant (docs/protocol-v2.md §7): 0.0 is
    the unset sentinel, [0.5, 1.0] is the calibrated range -- 0.3 is neither
    and is rejected, leaving rotSlip at its boot default."""
    reply = sim.command("SET rotSlip=0.3")
    assert reply.strip() == "ERR badval rotSlip=0.300"

    reply = sim.command("GET rotSlip")
    assert reply.strip() == "CFG rotSlip=0.000"


def test_set_non_numeric_value_reports_badval_without_a_value(sim):
    """A non-numeric value is a PARSE failure (docs/protocol-v2.md §7):
    ERR badval <key> with no value shown, distinct from an out-of-range
    invariant failure (ERR badval <key>=<value>)."""
    assert sim.command("SET tw=abc").strip() == "ERR badval tw"


def test_set_with_no_key_value_pairs_reports_badarg(sim):
    """docs/protocol-v2.md §7's own documented example: `SET` (bare, no
    args) -> `ERR badarg no key=value pairs`, NOT `ERR badkey`."""
    assert sim.command("SET").strip() == "ERR badarg no key=value pairs"


# ---------------------------------------------------------------------------
# Every explicitly-dropped key (architecture-update.md (084) Decision 2)
# correctly surfaces as ERR badkey -- identical wire behavior to any
# never-existed key, not a special "removed key" error class.
# ---------------------------------------------------------------------------
_DROPPED_KEYS = [
    "kff", "klf", "klb", "krf", "krb",
    "adjThr", "adjGain",
    "distScale", "turnScale",
    "tick", "tlmPeriod",
]


@pytest.mark.parametrize("key", _DROPPED_KEYS)
def test_dropped_keys_return_err_badkey_on_set(sim, key):
    assert sim.command(f"SET {key}=1").strip() == f"ERR badkey {key}"


@pytest.mark.parametrize("key", _DROPPED_KEYS)
def test_dropped_keys_return_err_badkey_on_get(sim, key):
    # GET's grammar always flushes a (possibly empty) trailing CFG line
    # alongside any ERR badkey lines (docs/protocol-v2.md §7's
    # `GET [<key>…] -> CFG <key>=<value>…` arrow, matching source_old/
    # robot/ConfigRegistry.cpp's handleGet precedent) -- assert the ERR
    # line is present rather than requiring it to be the ONLY line.
    assert f"ERR badkey {key}" in sim.command(f"GET {key}")


def test_get_unknown_key_returns_err_badkey(sim):
    reply = sim.command("GET badkey")
    assert "ERR badkey badkey" in reply


def test_get_mixes_valid_and_unknown_keys(sim):
    reply = sim.command("GET ml badkey")
    assert "ERR badkey badkey" in reply
    assert "CFG ml=0.000" in reply


# ---------------------------------------------------------------------------
# ml/mr/pid.* follow a DEV DT PORTS rebind: SET always resolves the
# CURRENTLY bound pair, read via Drivetrain::ports() at SET-time -- never a
# hardcoded port.
# ---------------------------------------------------------------------------
def test_ml_follows_a_dev_dt_ports_rebind_to_the_newly_bound_motor(sim):
    # Boot-bound pair is ports 1,2. Write ml -- lands on port 1's shadow.
    reply = sim.command("SET ml=0.500")
    assert reply.strip() == "OK set ml=0.500"
    assert sim.command("GET ml").strip() == "CFG ml=0.500"

    # Rebind to ports 3,4 -- ml/mr now mean port 3's/4's shadow.
    reply = sim.command("DEV DT PORTS 3 4")
    assert reply.strip() == "OK DEV DT ports=3,4"

    # Port 3 was never touched -- still at its boot default (0.000), NOT
    # port 1's 0.500.
    assert sim.command("GET ml").strip() == "CFG ml=0.000"

    # SET now while bound to 3,4 -- lands on port 3's shadow, not port 1's.
    reply = sim.command("SET ml=0.900")
    assert reply.strip() == "OK set ml=0.900"
    assert sim.command("GET ml").strip() == "CFG ml=0.900"

    # Rebinding back to the original pair proves port 1's shadow was never
    # touched by the SET issued while bound to port 3.
    sim.command("DEV DT PORTS 1 2")
    assert sim.command("GET ml").strip() == "CFG ml=0.500"


def test_pid_kp_follows_a_dev_dt_ports_rebind_to_the_newly_bound_motor(sim):
    reply = sim.command("SET pid.kp=1.000")
    assert reply.strip() == "OK set pid.kp=1.000"
    assert sim.command("GET pid.kp").strip() == "CFG pid.kp=1.000"

    sim.command("DEV DT PORTS 3 4")
    # Port 3's own boot pid.kp (0.0022 -> 0.002), untouched by the write
    # above (which landed on ports 1/2's shadows).
    assert sim.command("GET pid.kp").strip() == "CFG pid.kp=0.002"

    reply = sim.command("SET pid.kp=3.000")
    assert reply.strip() == "OK set pid.kp=3.000"
    assert sim.command("GET pid.kp").strip() == "CFG pid.kp=3.000"

    sim.command("DEV DT PORTS 1 2")
    assert sim.command("GET pid.kp").strip() == "CFG pid.kp=1.000"
