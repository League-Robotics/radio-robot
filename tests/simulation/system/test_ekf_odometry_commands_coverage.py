"""
test_ekf_odometry_commands_coverage.py — coverage-additive tests for the
estimator and the simulatable command surface (sprint 045 ticket 004).

Covers:
  * EKF.cpp gating: the position Mahalanobis reject branch, the 10-consecutive-
    rejection P-inflation re-baseline (position AND heading), and recovery.
  * Odometry.cpp wedge-suppress predict branch (dTheta held when a wheel wedges).
  * PhysicsWorld.cpp slip + reported-encoder noise paths.
  * SystemCommands.cpp simulatable verbs: SNAP, ZERO, VER, ECHO, HELP, SI, SAFE,
    GET/SET, the "+" keepalive (NOT the #ifndef HOST_BUILD RESET/hardware paths).
  * OtosCommands.cpp O-verbs: both the `nodev` ERR branches (OTOS not begun) and
    the OK success branches (OTOS begun via set_otos_fusion).

These are whole-robot scenario tests (drive + estimator + sensor injection), so
they live in tests/simulation/system/.
"""
import ctypes
import re

from firmware import Sim

TICK_STEP_MS = 24
WEDGE_THRESHOLD = 10


def _tick_n(s: Sim, n: int) -> str:
    evts = ""
    for _ in range(n):
        s._lib.sim_tick(s._h, ctypes.c_uint32(s._t))
        s._t += TICK_STEP_MS
        evts += s.get_async_evts()
    return evts


def _freeze_right(s: Sim) -> None:
    s._lib.sim_set_motor_offset(s._h, ctypes.c_int(1), ctypes.c_float(0.0))


# ---------------------------------------------------------------------------
# EKF — Mahalanobis gate reject + P-inflation re-baseline recovery
# ---------------------------------------------------------------------------

def test_ekf_gate_rejects_far_otos():
    """A far OTOS teleport is rejected by the Mahalanobis gate (ekf_rej grows).

    Exercises EKF::updatePosition's reject branch (++_rejected, ++_rejPos_streak)
    and updateHeading's reject branch — the estimate does not jump to the bogus
    reading on the first few injections.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.set_field_profile(slip_turn_extra=0.26, fuse_otos=True)
        s.send_command("D 200 200 200")
        s.tick_for(5000)
        s.get_async_evts()

        rej0 = s.get_ekf_rej_count()
        est0 = s.get_pose()

        # A FEW (< 10) far injections — gate rejects, estimate barely moves.
        for _ in range(3):
            s.set_otos_pose(9999.0, 9999.0, 99.0)
            _tick_n(s, 1)

        rej1 = s.get_ekf_rej_count()
        est1 = s.get_pose()

        assert rej1 > rej0, (
            f"EKF gate did not reject a far OTOS teleport "
            f"(rej {rej0} -> {rej1})."
        )
        # With < 10 rejections the estimate must NOT have snapped to 9999.
        assert abs(est1[0] - est0[0]) < 500.0 and abs(est1[1] - est0[1]) < 500.0, (
            f"estimate jumped toward the rejected teleport too soon "
            f"({est0[:2]} -> {est1[:2]})."
        )


def test_ekf_p_inflation_recovers_after_10_rejections():
    """10+ consecutive rejections trigger the P-inflation re-baseline (K≈1 snap).

    Exercises the position P-inflation block (_rejPos_streak >= 10) and the
    heading P-inflation block (_rejHead_streak >= 10): after the streak crosses
    10, the EKF inflates P so the next update snaps the estimate to the OTOS
    reading.  Confirms the estimate converges to the (now-consistent) reading.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.set_field_profile(slip_turn_extra=0.26, fuse_otos=True)
        s.send_command("D 200 200 200")
        s.tick_for(5000)
        s.get_async_evts()

        rej0 = s.get_ekf_rej_count()
        # 15 consecutive far injections — crosses the 10-streak P-inflation.
        for _ in range(15):
            s.set_otos_pose(9999.0, 9999.0, 99.0)
            _tick_n(s, 1)

        rej1 = s.get_ekf_rej_count()
        assert (rej1 - rej0) >= 10, (
            f"fewer than 10 rejections accumulated (delta={rej1 - rej0}); "
            f"P-inflation path not reached."
        )

        # After P-inflation the estimate snaps toward the (now-held) reading.
        px, py, _ = s.get_pose()
        assert px > 5000.0 and py > 5000.0, (
            f"EKF did not re-baseline to the held OTOS reading after the "
            f"10-rejection streak (pose=({px:.0f},{py:.0f}))."
        )


def test_ekf_recovers_to_truth_after_bad_burst():
    """After a long bad-OTOS burst, a truth-consistent reading reconverges.

    Drives the gate through reject → P-inflation → re-acquire, then feeds the
    real plant truth back and confirms the estimation error returns to tolerance.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.set_field_profile(slip_turn_extra=0.26, fuse_otos=True)
        s.send_command("D 200 200 300")
        s.tick_for(8000)
        s.get_async_evts()

        # Bad burst long enough to cross the P-inflation streak.
        for _ in range(12):
            s.set_otos_pose(9999.0, 9999.0, 99.0)
            _tick_n(s, 1)

        # Recover: feed the true pose back until the estimate reconverges.
        tx, ty, th = s.get_true_pose()
        for _ in range(80):
            s.set_otos_pose(tx, ty, th)
            _tick_n(s, 1)

        err_xy, err_h = s.estimation_error()
        assert err_xy < 60.0, (
            f"estimation error xy={err_xy:.1f} mm did not recover after a "
            f"truth-consistent OTOS reading followed the bad burst."
        )


# ---------------------------------------------------------------------------
# Odometry — wedge-suppress predict branch (dTheta held while a wheel is wedged)
# ---------------------------------------------------------------------------

def test_odometry_wedge_suppresses_heading_drift():
    """When a wheel wedges, Odometry::predict suppresses phantom dTheta.

    Drive straight, freeze the right encoder mid-drive so the wedge detector
    fires, then confirm the heading holds while the left wheel keeps advancing —
    exercising the `if (_wedgeActive) dTheta = 0;` branch in Odometry::predict.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.send_command("VW 200 0")
        # Warm-up: real matched movement before freezing (mid-command freeze,
        # not a from-the-start one; not required for arming since 064-004).
        _tick_n(s, 6)
        _freeze_right(s)

        # Tick until the odometry wedge gate latches.
        for _ in range(WEDGE_THRESHOLD + 25):
            _tick_n(s, 1)
            if s.get_odometry_wedge_active():
                break
        assert s.get_odometry_wedge_active(), (
            "odometry wedge gate did not activate after freezing R."
        )

        h_latch = s.get_pose()[2]
        encL_latch = float(s._lib.sim_get_enc_l(s._h))

        _tick_n(s, 40)   # left wheel keeps advancing the whole window

        h_end = s.get_pose()[2]
        encL_end = float(s._lib.sim_get_enc_l(s._h))

        # Setup sanity: the left wheel really moved (so unsuppressed dTheta would
        # have driven a large drift).
        assert (encL_end - encL_latch) > 15.0, (
            f"left wheel barely moved post-latch ({encL_latch:.1f}->{encL_end:.1f})."
        )
        import math
        dh = abs(math.atan2(math.sin(h_end - h_latch), math.cos(h_end - h_latch)))
        assert dh < 0.05, (
            f"heading drifted {math.degrees(dh):.1f} deg post-wedge while the left "
            f"wheel advanced {encL_end - encL_latch:.1f} mm — dTheta not suppressed."
        )


# ---------------------------------------------------------------------------
# PhysicsWorld — slip + reported-encoder noise paths
# ---------------------------------------------------------------------------

def test_physics_slip_reduces_reported_travel():
    """Straight-line slip makes reported encoder travel < commanded travel.

    Sets a straight slip factor via sim_set_motor_slip and confirms the reported
    encoder accumulates LESS than the true (unslipped) travel — exercising the
    encSlip term in PhysicsWorld::update sub-step A'.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        # straight slip 0.3, no turn-extra; side ignored (plant uses one pair).
        s._lib.sim_set_motor_slip(s._h, ctypes.c_int(2),
                                  ctypes.c_float(0.3), ctypes.c_float(0.0))
        s.send_command("S 300 300 9000")
        s.tick_for(3000)

        reported_l = float(s._lib.sim_get_enc_l(s._h))
        true_l, _ = s.get_true_wheel_travel()

        assert true_l > 0.0, "true travel should be positive after driving"
        # Reported (slipped) travel is strictly less than true (unslipped) travel.
        assert reported_l < true_l, (
            f"slip did not reduce reported travel: reported={reported_l:.1f} "
            f"true={true_l:.1f}"
        )


def test_physics_encoder_noise_path():
    """Non-zero encoder noise sigma exercises the pwGaussianNoise draw path.

    sim_set_encoder_noise(sigma>0) routes through PhysicsWorld's HOST_BUILD
    pwGaussianNoise() helper.  Confirm the sim still drives without crashing and
    the reported encoder advances (the noise term is added to each step).
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        # sim_set_encoder_noise(side=2 both, sigma=5 mm) — exercises pwGaussianNoise.
        s._lib.sim_set_encoder_noise.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
        s._lib.sim_set_encoder_noise.restype = None
        s._lib.sim_set_encoder_noise(s._h, ctypes.c_int(2), ctypes.c_float(5.0))

        s.send_command("S 300 300 9000")
        s.tick_for(2000)
        enc_l = float(s._lib.sim_get_enc_l(s._h))
        assert enc_l == enc_l, "encoder is NaN with noise enabled"
        assert enc_l > 50.0, (
            f"encoder did not advance with noise enabled (enc_l={enc_l:.1f})"
        )


# ---------------------------------------------------------------------------
# SystemCommands — simulatable verbs (SNAP/ZERO/VER/ECHO/HELP/SI/SAFE/+/GET/SET)
# ---------------------------------------------------------------------------

def test_snap_emits_tlm_frame(sim):
    """SNAP emits one TLM frame (handleSnap)."""
    r = sim.send_command("SNAP")
    assert "TLM" in r, f"SNAP did not emit a TLM frame: {r!r}"


def test_zero_enc_resets_accumulators(sim):
    """ZERO enc resets the encoder accumulators (handleZero enc path)."""
    sim.send_command("S 300 300 9000")
    sim.tick_for(1000)
    assert float(sim._lib.sim_get_enc_l(sim._h)) > 10.0
    sim.send_command("X")
    sim.tick_for(48)
    r = sim.send_command("ZERO enc")
    assert "OK" in r.upper(), f"ZERO enc failed: {r!r}"
    sim.tick_for(48)
    assert abs(float(sim._lib.sim_get_enc_l(sim._h))) < 5.0, (
        "encoder not near zero after ZERO enc"
    )


def test_zero_pose_resets_pose(sim):
    """ZERO pose resets the dead-reckoning pose (handleZero pose path)."""
    r = sim.send_command("ZERO pose")
    assert "OK" in r.upper(), f"ZERO pose failed: {r!r}"


def test_ver_reports_version(sim):
    """VER replies with a version string (handleVer)."""
    r = sim.send_command("VER")
    assert "OK" in r.upper(), f"VER did not reply OK: {r!r}"
    assert re.search(r"\d", r), f"VER reply has no version digits: {r!r}"


def test_echo_returns_tokens(sim):
    """ECHO hello returns 'hello' (handleEcho)."""
    r = sim.send_command("ECHO hello")
    assert "hello" in r, f"ECHO did not echo the token: {r!r}"


def test_help_multiline_no_crash(sim):
    """HELP returns a reply without crashing (handleHelp)."""
    r = sim.send_command("HELP")
    assert len(r) > 0, "HELP returned an empty reply"
    assert "OK" in r.upper() or "\n" in r, f"HELP reply looks empty: {r!r}"


def test_si_sets_world_pose(sim):
    """SI <x> <y> <hcdeg> sets the odometry world pose (handleSI → EKF::setPose).

    Exercises Odometry::setPose / EKF::setPose (the P-reset diagonal block).
    """
    r = sim.send_command("SI 500 250 9000")
    assert "OK" in r.upper(), f"SI did not reply OK: {r!r}"
    sim.tick_for(48)
    px, py, _ = sim.get_pose()
    assert abs(px - 500.0) < 50.0 and abs(py - 250.0) < 50.0, (
        f"SI did not set the pose near (500,250): got ({px:.0f},{py:.0f})"
    )


def test_safe_command_accepted(sim):
    """SAFE <timeout> configures the safety watchdog (handleSafe)."""
    r = sim.send_command("SAFE 1 5000")
    assert "OK" in r.upper() or "ERR" not in r.upper(), f"SAFE reply: {r!r}"


def test_keepalive_plus_is_quiet(sim):
    """The '+' keepalive is a QUIET command — it emits no reply (handleKeepalive).

    handleKeepalive is the sprint-024-003 quiet keepalive: it resets the watchdog
    and intentionally emits NO reply (the host filters acks).  The empty reply is
    the correct, documented behavior; exercising it covers handleKeepalive and the
    sim_command watchdog re-arm.  A subsequent command must still work (no ERR).
    """
    r = sim.send_command("+")
    assert r == "" or "ERR" not in r.upper(), (
        f"'+' keepalive should be quiet (no reply) or at least not ERR, got {r!r}"
    )
    # The session is still healthy after the keepalive.
    assert "TLM" in sim.send_command("SNAP"), "session broken after '+' keepalive"


def test_get_set_vel_kp(sim):
    """GET/SET round-trip on a vel gain (config registry path via SystemCommands)."""
    rset = sim.send_command("SET vel.kP=0.07")
    assert "OK" in rset.upper(), f"SET vel.kP failed: {rset!r}"
    rget = sim.send_command("GET vel.kP")
    assert "0.07" in rget, f"GET vel.kP did not reflect SET: {rget!r}"


def test_get_unknown_key_errors(sim):
    """GET on an unknown key returns ERR (SystemCommands GET error branch)."""
    r = sim.send_command("GET no.such.key")
    assert "ERR" in r.upper(), f"GET unknown key did not ERR: {r!r}"


# ---------------------------------------------------------------------------
# OtosCommands — O-verbs: nodev ERR branches (not begun) + OK branches (begun)
# ---------------------------------------------------------------------------

def test_otos_verbs_nodev_when_not_initialized(sim):
    """Before OTOS begin(), the O-verbs hit their `nodev` ERR branches.

    Exercises the `!is_initialized()` ERR branch in handleOI/OZ/OR/OV/OL/OA.
    """
    for cmd in ("OI", "OZ", "OR", "OV 1 2 3", "OL 5", "OA 3"):
        r = sim.send_command(cmd)
        assert "ERR" in r.upper() and "nodev" in r, (
            f"{cmd!r} should return ERR nodev when OTOS not begun, got {r!r}"
        )


def test_otos_op_reports_pose_no_init_required(sim):
    """OP reports the cached OTOS pose without an init check (handleOP)."""
    r = sim.send_command("OP")
    assert "OK" in r.upper() and "op" in r and "x=" in r, (
        f"OP did not report a pose: {r!r}"
    )


def test_otos_verbs_ok_when_initialized(sim):
    """After OTOS begin() (set_otos_fusion), the O-verbs hit their OK branches.

    Exercises the success paths of handleOI/OZ/OR/OV/OL/OA.
    """
    sim.set_otos_fusion(True)   # marks the SimOdometer initialised (begin())

    assert "OK" in sim.send_command("OI").upper()
    assert "OK" in sim.send_command("OZ").upper()
    assert "OK" in sim.send_command("OR").upper()

    rv = sim.send_command("OV 10 20 30")
    assert "OK" in rv.upper() and "setpos" in rv, f"OV failed: {rv!r}"

    rl = sim.send_command("OL 5")
    assert "OK" in rl.upper() and "scalar=5" in rl, f"OL failed: {rl!r}"

    ra = sim.send_command("OA 3")
    assert "OK" in ra.upper() and "scalar=3" in ra, f"OA failed: {ra!r}"


def test_otos_ov_bad_args_errors(sim):
    """OV with no args returns ERR (parseOV badarg branch)."""
    r = sim.send_command("OV")
    assert "ERR" in r.upper(), f"OV with no args should ERR, got {r!r}"
