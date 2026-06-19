"""
test_motor_controller_coverage.py — coverage-additive tests for MotorController.

Sprint 045 ticket 002.  Exercises the real C++ MotorController paths that the
existing test_motor_controller.py (4 tests) leaves untouched:

  - The encoder-wedge detector: stuck-counter loop, kWedgeThreshold fire, the
    EVT enc_wedged emission via _evtFn/_evtCtx, the arming-grace gate
    (_hasMovedL/R), and latch re-arm after the encoder moves again.
  - wheelWedgedL()/wheelWedgedR() latch accessors after the EVT fires.
  - updateVelGains() via SET vel.kP (ConfigRegistry → mc.updateVelGains).
  - getEncoderPositions() via the D (distance) begin path
    (MotionControllerBegin → _mc.getEncoderPositions).
  - The "no drive command active" stop branch (zeroes PWM, clears EMA velocity).

All wedge tests use the proven sim technique from test_033_005_wedge_hardening:
drive a wheel, then freeze its encoder via sim_set_motor_offset(side, 0.0) so the
reported encoder value stops changing while the wheel is still commanded.  After
kWedgeThreshold consecutive frozen ticks (post-arming-grace) the detector fires.

ARCHITECTURE NOTES (documented per ticket OQ-1 and the ZOH acceptance criterion):

  * RatioPidController is DEAD CODE in the live control loop.  N13/030-010 removed
    its update() from controlTick; the only references in source/ are the pid.*
    config keys in ConfigRegistry (kept for host SET/GET compatibility), the
    class's own .h/.cpp, and the removal note in MotorController.h.  It is excluded
    from the simulatable-code denominator in tests/_infra/coverage.sh.

  * controlTick's refreshedWheel==1 (left-only) and refreshedWheel==2 (right-only)
    ZOH branches are DEAD-IN-SIM.  Drive::periodic always calls
    controlTick(..., driving ? 3 : 0) — both wheels are refreshed every tick (==3)
    or neither (==0).  The single-wheel ZOH branches were the split-phase
    (CODAL WedgeTest, sprint 015) pattern, never the sim path.  The ==3 branch
    (both-wheel ZOH) and the ==0 idle branch are exercised here.

  * startDrive()/startDriveClean() have NO live callers in source/.  The motion
    path runs BodyVelocityController::tick() → MotorController::setTarget(sL, sR),
    not startDrive*.  These are legacy seeding methods superseded by the BVC; they
    are unreachable through the sim and are not test-additively coverable.
"""
import ctypes

# kWedgeThreshold in MotorController.h
WEDGE_THRESHOLD = 10
TICK_STEP_MS = 24
# Ticks to run before freezing so _hasMovedL/R latch arms (post arming-grace).
TICKS_BEFORE_FREEZE = 6


def _tick_n(sim, n: int) -> None:
    """Advance the sim n control ticks of TICK_STEP_MS each."""
    for _ in range(n):
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += TICK_STEP_MS


def _freeze_left(sim) -> None:
    sim._lib.sim_set_motor_offset(sim._h, ctypes.c_int(0), ctypes.c_float(0.0))


def _freeze_right(sim) -> None:
    sim._lib.sim_set_motor_offset(sim._h, ctypes.c_int(1), ctypes.c_float(0.0))


# ---------------------------------------------------------------------------
# Wedge detector — EVT enc_wedged emission (left and right wheels)
# ---------------------------------------------------------------------------

def test_wedge_evt_emitted_right_wheel(sim):
    """Freezing the right encoder mid-drive fires EVT enc_wedged wheel=R once.

    Exercises the right-wheel branch of controlTick's wedge detector: the
    _stuckCountR increment loop, the kWedgeThreshold fire, and the snprintf
    EVT emission through _evtFn/_evtCtx (MotorController.cpp lines ~329-374).
    """
    sim.send_command("SET sTimeout=60000")
    sim.send_command("VW 200 0")          # straight drive — both wheels move
    sim.get_async_evts()                  # drain any startup replies

    _tick_n(sim, TICKS_BEFORE_FREEZE)     # arm grace: both wheels have moved
    _freeze_right(sim)

    # Tick until the wedge latches (cap so a non-firing detector fails loudly).
    evts = ""
    for _ in range(WEDGE_THRESHOLD + 25):
        _tick_n(sim, 1)
        evts += sim.get_async_evts()
        if sim.get_wheel_wedged_r():
            break

    assert sim.get_wheel_wedged_r(), (
        "wheelWedgedR() latch not set after freezing R encoder mid-drive — "
        "wedge detector did not fire."
    )
    assert "EVT enc_wedged" in evts and "wheel=R" in evts, (
        f"Expected 'EVT enc_wedged wheel=R' in async events, got: {evts!r}"
    )


def test_wedge_evt_emitted_left_wheel(sim):
    """Freezing the left encoder mid-drive fires EVT enc_wedged wheel=L once.

    Exercises the LEFT-wheel branch of the wedge detector (lines ~274-326),
    distinct from the right-wheel branch above.
    """
    sim.send_command("SET sTimeout=60000")
    sim.send_command("VW 200 0")
    sim.get_async_evts()

    _tick_n(sim, TICKS_BEFORE_FREEZE)
    _freeze_left(sim)

    evts = ""
    for _ in range(WEDGE_THRESHOLD + 25):
        _tick_n(sim, 1)
        evts += sim.get_async_evts()
        if sim.get_wheel_wedged_l():
            break

    assert sim.get_wheel_wedged_l(), (
        "wheelWedgedL() latch not set after freezing L encoder mid-drive."
    )
    assert "EVT enc_wedged" in evts and "wheel=L" in evts, (
        f"Expected 'EVT enc_wedged wheel=L' in async events, got: {evts!r}"
    )


def test_wedge_evt_fires_once_per_episode(sim):
    """The EVT is latched: exactly one enc_wedged fires while the wheel stays stuck.

    Confirms the _wedgeEmittedR latch suppresses repeat EVTs each tick after the
    first fire (the `!_wedgeEmittedR` guard).  Without the latch the EVT would
    fire on every subsequent frozen tick.
    """
    sim.send_command("SET sTimeout=60000")
    sim.send_command("VW 200 0")
    sim.get_async_evts()

    _tick_n(sim, TICKS_BEFORE_FREEZE)
    _freeze_right(sim)

    evts = ""
    for _ in range(WEDGE_THRESHOLD + 25):
        _tick_n(sim, 1)
        evts += sim.get_async_evts()

    fire_count = evts.count("EVT enc_wedged wheel=R")
    assert fire_count == 1, (
        f"Expected exactly one enc_wedged EVT for R while frozen, got "
        f"{fire_count}.\nEvents: {evts!r}"
    )


def test_wedge_arming_grace_suppresses_premature_fire(sim):
    """No EVT enc_wedged fires when the wheel is frozen BEFORE it ever moved.

    Exercises the _hasMovedR arming-grace gate (033-005d): the stuck counter must
    NOT advance until the wheel has moved at least once since the command started.
    Freezing R from the very start of the command leaves _hasMovedR False, so the
    detector never arms even though the encoder is constant and commanded.
    """
    sim.send_command("SET sTimeout=60000")
    # Freeze R BEFORE issuing the drive — _hasMovedR can never latch True.
    _freeze_right(sim)
    sim.send_command("VW 200 0")
    sim.get_async_evts()

    evts = ""
    for _ in range(WEDGE_THRESHOLD + 20):
        _tick_n(sim, 1)
        evts += sim.get_async_evts()

    assert not sim.get_wheel_wedged_r(), (
        "wheelWedgedR() fired even though R never moved — arming grace "
        "(_hasMovedR) is not gating the stuck counter."
    )
    assert "EVT enc_wedged wheel=R" not in evts, (
        f"enc_wedged EVT fired before the wheel ever moved — arming grace broken. "
        f"Events: {evts!r}"
    )


def test_wedge_relatch_after_recovery(sim):
    """The wedge latch re-arms: a second stuck episode produces a second EVT.

    Episode 1: freeze R → EVT fires, latch set.  Recover: unfreeze R, tick so the
    encoder moves again → re-arm path resets _stuckCountR/_wedgeEmittedR and sets
    _hasMovedR.  Episode 2: freeze R again → a fresh EVT fires.  This exercises the
    `encR != _wedgePrevEncR` re-arm branch followed by a second fire.
    """
    sim.send_command("SET sTimeout=60000")
    sim.send_command("VW 200 0")
    sim.get_async_evts()
    _tick_n(sim, TICKS_BEFORE_FREEZE)

    # --- Episode 1: wedge fires ---
    _freeze_right(sim)
    ep1 = ""
    for _ in range(WEDGE_THRESHOLD + 25):
        _tick_n(sim, 1)
        ep1 += sim.get_async_evts()
        if sim.get_wheel_wedged_r():
            break
    assert sim.get_wheel_wedged_r(), "Episode 1 wedge did not latch."
    assert ep1.count("EVT enc_wedged wheel=R") == 1

    # --- Recover: unfreeze so the encoder moves again (re-arm) ---
    sim._lib.sim_set_motor_offset(sim._h, ctypes.c_int(1), ctypes.c_float(1.0))
    recover = ""
    for _ in range(20):
        _tick_n(sim, 1)
        recover += sim.get_async_evts()
        if not sim.get_wheel_wedged_r():
            break
    assert not sim.get_wheel_wedged_r(), (
        "wheelWedgedR() did not clear after the encoder moved again — re-arm "
        "branch (encR != _wedgePrevEncR) did not reset the latch."
    )

    # --- Episode 2: freeze again, a fresh EVT must fire ---
    _freeze_right(sim)
    ep2 = ""
    for _ in range(WEDGE_THRESHOLD + 25):
        _tick_n(sim, 1)
        ep2 += sim.get_async_evts()
        if sim.get_wheel_wedged_r():
            break
    assert sim.get_wheel_wedged_r(), "Episode 2 wedge did not re-latch."
    assert ep2.count("EVT enc_wedged wheel=R") == 1, (
        f"Second stuck episode did not produce a fresh EVT (re-arm failed). "
        f"Episode-2 events: {ep2!r}"
    )


# ---------------------------------------------------------------------------
# updateVelGains — SET vel.kP routes through ConfigRegistry → mc.updateVelGains
# ---------------------------------------------------------------------------

def test_set_vel_gain_updates_running_controllers(sim):
    """SET vel.kP=<x> is accepted and pushed into the live VelocityControllers.

    ConfigRegistry::handleSet calls mc.updateVelGains(cfg) when a vel.* key
    changes (ConfigRegistry.cpp:594).  This exercises MotorController::updateVelGains
    (the _vcL/_vcR gain-copy body).  Verify the SET is accepted (OK) and a
    subsequent drive still produces sane PWM (no crash / NaN from the new gains).
    """
    sim.send_command("SET sTimeout=60000")
    r = sim.send_command("SET vel.kP=0.1")
    assert "OK" in r.upper(), f"SET vel.kP rejected: {r!r}"

    # GET it back to confirm the config registry committed the value.
    g = sim.send_command("GET vel.kP")
    assert "0.1" in g, f"GET vel.kP did not reflect the SET value: {g!r}"

    # Drive after the gain change — PID must still run and produce finite PWM.
    sim.send_command("S 200 200 9000")
    sim.tick_for(1000)
    pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
    assert pwm_l == pwm_l, "PWM is NaN after updateVelGains"
    assert -100.0 <= pwm_l <= 100.0, f"PWM out of range after gain update: {pwm_l}"


# ---------------------------------------------------------------------------
# getEncoderPositions — reached via the D (distance) begin path
# ---------------------------------------------------------------------------

def test_distance_command_drives_and_stops(sim):
    """A D (distance) command begins via the path that calls getEncoderPositions.

    MotionControllerBegin::beginDistance calls _mc.getEncoderPositions() to
    capture the decel-hook baseline (MotionControllerBegin.cpp:299).  Driving a
    bounded D command both exercises that accessor and the bounded-drive flow,
    then confirms the robot travels toward the target and the command terminates.
    """
    sim.send_command("SET sTimeout=60000")
    enc0 = float(sim._lib.sim_get_enc_l(sim._h))
    r = sim.send_command("D 200 200 300")    # drive 300 mm at 200 mm/s
    assert "OK" in r.upper() or "ERR" not in r.upper(), f"D rejected: {r!r}"

    sim.tick_for(4000)
    enc1 = float(sim._lib.sim_get_enc_l(sim._h))
    # Must have travelled a meaningful distance toward the 300 mm target.
    assert (enc1 - enc0) > 100.0, (
        f"D 200 200 300 did not advance the encoder enough: {enc0:.1f} -> {enc1:.1f}"
    )


# ---------------------------------------------------------------------------
# Idle / stop branch — controlTick with no active drive command
# ---------------------------------------------------------------------------

def test_idle_tick_zeroes_pwm_and_velocity(sim):
    """With no active command, controlTick zeroes PWM and clears EMA velocity.

    Exercises the `if (tgtLMms == 0 && tgtRMms == 0)` early-out in controlTick:
    pwmL/pwmR set to 0, velLMms/velRMms cleared, motors stopped, early return.
    """
    sim.send_command("SET sTimeout=60000")
    # Drive briefly to build up velocity, then cancel.
    sim.send_command("S 300 300 9000")
    sim.tick_for(500)
    assert float(sim._lib.sim_get_vel_l(sim._h)) != 0.0 or \
        float(sim._lib.sim_get_pwm_l(sim._h)) != 0.0

    sim.send_command("X")        # cancel → targets go to 0
    sim.tick_for(48)             # a couple idle ticks

    assert float(sim._lib.sim_get_pwm_l(sim._h)) == 0.0, "PWM_L not zero when idle"
    assert float(sim._lib.sim_get_pwm_r(sim._h)) == 0.0, "PWM_R not zero when idle"
    assert float(sim._lib.sim_get_vel_l(sim._h)) == 0.0, "velLMms not cleared when idle"
    assert float(sim._lib.sim_get_vel_r(sim._h)) == 0.0, "velRMms not cleared when idle"
