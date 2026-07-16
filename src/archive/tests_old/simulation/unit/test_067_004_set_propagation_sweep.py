"""test_067_004_set_propagation_sweep.py — SET-to-consumer propagation
regression sweep (sprint 067, ticket 004).

Background (see clasi/sprints/067-set-to-planner-config-propagation-fix/
architecture-update.md, the full audit table in Step 1-2, and Design
Rationale Decisions 1-2):

  Before this sprint, `Planner` held its `RobotConfig` by VALUE
  (`RobotConfig _cfg;`), snapshotted once at boot, instead of by live
  reference like every sibling subsystem (`Drive`, `Superstructure`,
  `MotorController`, `Ports`, `BodyVelocityController`, `Motor`,
  `OtosSensor`).  `SET rotSlip=<x>` (and five sibling plain keys:  `tw`,
  `vWheelMax`, `rotGainPos`, `rotGainNeg`, `ctrlPeriod` -- plus a further
  eight keys that only propagated when bundled with an annotated key in the
  same `SET` line, e.g. `turnGate`) replied `OK`, `GET` read the new value
  back, and NOTHING in Planner's actual control math ever changed, because
  `Planner::_cfg` was a private copy the live `SET` path never touched.
  Ticket 001 fixed this by converting `_cfg` to `const RobotConfig&`.

  `Drive` had a second, narrower instance of the same disease: its
  `_drvCfg` shadow-cache (a `msg::DrivetrainConfig` snapshot, refreshed only
  on a `"drive"`-annotated `SET`) shadowed the live `_robCfg` fallback for
  `tw`/`lag.otos` in `tickUpdate()`'s EKF-predict step.  Ticket 002 fixed
  that pair of read sites.  While implementing 002, a THIRD, structurally
  identical shadow read was found in `Drive::tickAction()`'s TWIST
  inverse-kinematics case (`Drive.cpp`, the `trackwidth` ternary feeding
  `BodyKinematics::inverse()`) -- out of 002's scope (which was limited to
  `tickUpdate()`), but the exact same bug shape: `_drvCfg.get_trackwidth()`
  is populated once at boot (`Robot::Robot()` calls
  `drive.configure(toDriveConfig(config))`) and never again, because `tw`
  is not `"drive"`-annotated, so `SET tw=<x>` alone never reached the TWIST
  path's wheel-speed conversion either.  This ticket (067-004) fixes that
  site too (read `_robCfg.trackwidthMm` directly, matching Ticket 002's
  fix and the `rotationalSlip` neighbor one line below it in
  `tickUpdate()`), since `Planner`'s own TWIST output is the vehicle for
  essentially every motion command (RT, D, T, VW, G -- `Planner::tick()`
  always packs a `DrivetrainCommand{TWIST}`, verb_id=1).

  Ticket 003 added a noise-only `EKFTiny::setNoise()` path so
  `SET ekfRHead=<x>` reaches the live EKF measurement-noise state without
  resetting the fused pose/covariance (which a naive `initEKF()` re-call
  would have done).

This sweep is the regression guard the issue's acceptance criteria demand:
for each motion-critical key the audit marked STALE, SET it to two values
distinct from its compiled boot default and assert the OWNING CONSUMER's
observable behavior differs between the two -- not just that `GET` reads
the value back (which passed even with the bug present; `GET` reads the
committed `RobotConfig` field directly, independent of whether any
consumer's private copy was refreshed).

Each measurement uses a FRESH `Sim()` instance -- never a shared `sim`
fixture reused across measurements in a single test -- per the exact
isolation discipline Ticket 001 introduced when it fixed
`test_rt_slip.py`'s false-positive bug: that file's `_arc_after_rt()`
helper called `sim.send_command("ZERO")` (no token) between measurements
without checking the reply.  `parseZero()` (`source/commands/
SystemCommands.cpp`) rejects a bare `ZERO` with `ERR badarg` -- it does NOT
reset the encoders -- so encoder state silently accumulated across the two
sequential `RT 9000` calls each test function made, faking a slip effect
that was not real.  Every command reply below that matters for isolation
correctness is checked (`assert "OK" in reply.upper()`).

Keys covered (the audit's STALE rows, `architecture-update.md` Step 1-2):
  rotSlip, tw, vWheelMax, rotGainPos, rotGainNeg, turnGate, ctrlPeriod,
  lag.otos, ekfRHead.
"""
import pytest

from firmware import Sim


# ---------------------------------------------------------------------------
# rotSlip / tw — RT encoder-arc probe (Planner::beginRotation, PlannerBegin.cpp)
#
#   arc = |deg| * (pi/180) * (trackwidth/2) / effectiveSlip(rotationalSlip)
#
# This measures the STOP THRESHOLD Planner computes for RT's ROTATION stop
# condition (`makeRotationStop(stopArc)`), which fires purely on encoder
# POSITION crossing that threshold -- it is deliberately NOT a probe of
# Drive's TWIST inverse-kinematics trackwidth read (Drive.cpp,
# tickAction()'s TWIST case): the wheel-speed conversion used to GET to the
# threshold does not affect the final measured arc, only how long it takes
# to get there. See test_tw_changes_twist_inverse_kinematics below (using a
# fire-and-forget `_VW` command instead of RT) for the dedicated probe of
# that separate, 067-004-fixed shadow-cache read.
# ---------------------------------------------------------------------------

def _arc_after_rt(tw_value=None, rotslip_value=None, cdeg: int = 9000) -> float:
    """Fresh Sim(): optionally SET tw/rotSlip, issue RT <cdeg>, return the
    per-wheel encoder arc |encR - encL| / 2 in mm after completion.

    Mirrors test_rt_slip.py's _arc_after_rt(), but takes a fresh Sim() per
    call instead of reusing one instance with a ZERO-enc reset between
    measurements -- the simplest form of the isolation Ticket 001 requires.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        if tw_value is not None:
            r = s.send_command(f"SET tw={tw_value}")
            assert "OK" in r.upper(), f"SET tw={tw_value} -> {r!r}"
        if rotslip_value is not None:
            r = s.send_command(f"SET rotSlip={rotslip_value}")
            assert "OK" in r.upper(), f"SET rotSlip={rotslip_value} -> {r!r}"

        r = s.send_command(f"RT {cdeg}")
        assert "OK" in r.upper(), f"RT {cdeg} -> {r!r}"
        s.tick_for(8000)

        enc_l, enc_r = s.get_true_wheel_travel()
        return abs(enc_r - enc_l) / 2.0


def test_rotslip_changes_rt_arc():
    """SET rotSlip=<x> alone must change the RT encoder-arc target.

    Pre-067-001: Planner::_cfg.rotationalSlip was frozen at the compiled
    default (0.92) regardless of any SET -- this is the sprint's headline
    bug (architecture-update.md item 4, empirically reproduced there).
    Two distinct SET values (neither equal to the 0.92 compiled default)
    must produce clearly different arcs.
    """
    arc_slip_1_0 = _arc_after_rt(rotslip_value=1.0)
    arc_slip_0_5 = _arc_after_rt(rotslip_value=0.5)

    assert arc_slip_0_5 > arc_slip_1_0 * 1.5, (
        f"RT arc did not scale with rotSlip: rotSlip=1.0 -> {arc_slip_1_0:.2f} mm, "
        f"rotSlip=0.5 -> {arc_slip_0_5:.2f} mm (expected ~2x larger at 0.5)"
    )


def test_tw_changes_rt_arc():
    """SET tw=<x> alone must change the RT encoder-arc TARGET Planner
    computes (beginRotation()'s `arc = ... * (tw * 0.5f) / slip`).

    rotSlip is pinned to 1.0 in both measurements to isolate tw's effect
    from rotSlip's (covered separately above). Two distinct SET values
    (neither equal to the 128 mm compiled default) must produce clearly
    different arcs.

    Note: this measures the STOP THRESHOLD Planner computes, not the wheel
    speed used to get there -- it is insensitive to (and does not exercise)
    Drive::tickAction()'s separate TWIST inverse-kinematics trackwidth read
    (see test_tw_changes_twist_inverse_kinematics below), because RT's
    ROTATION stop condition fires purely on encoder POSITION crossing this
    threshold, regardless of how fast the wheels turned to get there
    (empirically confirmed: this test alone still passed when the
    TWIST-path fix below was reverted).
    """
    arc_tw_64 = _arc_after_rt(tw_value=64, rotslip_value=1.0)
    arc_tw_256 = _arc_after_rt(tw_value=256, rotslip_value=1.0)

    assert arc_tw_256 > arc_tw_64 * 3.0, (
        f"RT arc did not scale with tw: tw=64 -> {arc_tw_64:.2f} mm, "
        f"tw=256 -> {arc_tw_256:.2f} mm (expected ~4x larger at tw=256)"
    )


def _twist_wheel_speed(tw_value, omega_mrad: int = 500,
                        window_ms: int = 500) -> float:
    """Fresh Sim(): SET tw, issue a raw `_VW 0 <omega_mrad>` (fire-and-forget
    pure-rotation TWIST -- no ramp, no encoder-threshold stop condition),
    and return the steady-state |right wheel speed| (mm/s, plant truth)
    after a short settle window.

    `_VW` bypasses beginRotation()/beginGoTo() entirely (Planner::
    beginRawVelocity() seeds the BVC directly), so the ONLY trackwidth read
    this exercises is Drive::tickAction()'s TWIST inverse-kinematics
    conversion (BodyKinematics::inverse(vx=0, omega, trackwidth, vL, vR)) --
    the shadow site this ticket (067-004) fixed.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        r = s.send_command(f"SET tw={tw_value}")
        assert "OK" in r.upper(), f"SET tw={tw_value} -> {r!r}"
        r = s.send_command(f"_VW 0 {omega_mrad}")
        assert "OK" in r.upper(), f"_VW 0 {omega_mrad} -> {r!r}"
        s.tick_for(window_ms, step_ms=24)
        _vl, vr = s.get_true_velocity()
        return abs(vr)


def test_tw_changes_twist_inverse_kinematics():
    """SET tw=<x> alone must change the wheel speed Drive::tickAction()'s
    TWIST case commands for a FIXED requested body omega.

    This is the regression guard for the shadow-cache read Ticket 002's
    programmer found (`Drive.cpp` TWIST case, `_drvCfg.get_trackwidth() >
    0.0f ? ... : _robCfg.trackwidthMm`) but left out of 002's scope
    (`tickUpdate()`'s EKF-predict site only) -- fixed as part of this
    ticket (067-004) by reading `_robCfg.trackwidthMm` directly, the same
    pattern 002 used one function up.

    Two distinct SET values (neither equal to the 128 mm compiled default).
    Empirically confirmed to FAIL (both wheel speeds frozen at the same
    ~28 mm/s regardless of tw, matching the boot-time _drvCfg snapshot)
    when the fix in Drive.cpp was temporarily reverted.
    """
    speed_tw_64 = _twist_wheel_speed(64)
    speed_tw_500 = _twist_wheel_speed(500)

    assert speed_tw_500 > speed_tw_64 * 3.0, (
        f"TWIST wheel speed did not scale with tw for a fixed omega: "
        f"tw=64 -> {speed_tw_64:.2f} mm/s, tw=500 -> {speed_tw_500:.2f} mm/s "
        f"(expected a clear increase; a frozen shadow-cache read would show "
        f"nearly identical speeds regardless of tw)"
    )


# ---------------------------------------------------------------------------
# vWheelMax — PRE_ROTATE omega cap (Planner::_startPreRotate, PlannerBegin.cpp)
#
#   omegaMax = 2 * vWheelMax / trackwidthMm; omega is clamped to it.
#
# Investigation note (this ticket): this specific clamp turns out to be
# UNREACHABLE as a behaviorally-observable binding constraint in the
# current control stack, for any vWheelMax value, because two OTHER
# already-live clamps downstream always bind at least as tightly:
#   1. BodyVelocityController::advance() clamps the ramp TARGET to
#      yawRateMax (70 deg/s ~= 1.22 rad/s ~= 78 mm/s-equivalent wheel
#      speed) before ramping -- and yawRateMax's own `_cfg` is already a
#      live reference, so this bind is unconditional and independent of
#      whatever Planner requests.
#   2. BodyVelocityController::advance() ALSO saturates the post-kinematics
#      wheel speeds via BodyKinematics::saturate(vL, vR, _cfg.vWheelMax,
#      _cfg.steerHeadroom, sL, sR) -- and BVC's `_cfg` is ALSO a live
#      reference (audit: LIVE, no fix needed), using the exact same
#      `2 * vWheelMax / trackwidthMm` formula _startPreRotate computes.
# Since compiled-default vWheelMax (400 mm/s => 6.25 rad/s) already exceeds
# yawRateMax's ceiling, yawRateMax is the binding constraint at and above
# the default for ANY vWheelMax value (frozen or live -- both exceed it,
# so both get equally overridden). Below yawRateMax's ~78 mm/s-equivalent
# threshold, BVC's OWN wheel saturation (formula-identical to Planner's
# clamp, and always live) binds instead and produces the numerically
# correct result regardless of what Planner's separate, possibly-stale
# clamp requested. Empirically confirmed by reverting Planner.h/.cpp to
# the pre-067-001 value-copy `_cfg` and re-running this exact scenario:
# the result was numerically IDENTICAL to the fixed build at every
# vWheelMax value tried (40 through 1200 mm/s) -- this consumer's own
# fix has no isolable behavioral signature today, independent of the
# `_cfg`-reference fix ticket 001 made for every OTHER field.
#
# This test therefore demonstrates SET vWheelMax reaching a live,
# observable consumer end-to-end (satisfying "exercises the one
# sim-observable behavior that depends on it") via BodyVelocityController's
# wheel saturation -- BVC's `_cfg` was never stale, but the overall
# SET->motion pipeline genuinely does respond to vWheelMax, which is the
# regression a stakeholder cares about ("does SET vWheelMax do anything at
# all"). It is NOT a regression guard specific to `_startPreRotate`'s own
# (also-fixed, but currently non-binding) clamp -- flagging that
# specifically so a future maintainer does not mistake this for coverage
# of that exact code path.
# ---------------------------------------------------------------------------

def _pre_rotate_cruise_speed(vwheel_max, window_ms: int = 2000) -> float:
    """Fresh Sim(): SET vWheelMax, command a G whose bearing (~180 deg) is
    far outside the default turnInPlaceGate (35 deg), so the robot spends
    the whole window spinning in place (PRE_ROTATE). Returns the
    steady-state |right wheel speed| (mm/s, plant truth) near the end of
    the window.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        r = s.send_command(f"SET vWheelMax={vwheel_max}")
        assert "OK" in r.upper(), f"SET vWheelMax={vwheel_max} -> {r!r}"
        # tx=-9999, ty=1: bearing = atan2(1, -9999) ~= 180 deg, comfortably
        # outside the 35 deg gate -- PRE_ROTATE needs ~145 deg of rotation,
        # giving ample runway before the HEADING stop transitions to PURSUE.
        r = s.send_command("G -9999 1 1000")
        assert "OK" in r.upper(), f"G -9999 1 1000 -> {r!r}"
        s.tick_for(window_ms, step_ms=24)
        _vl, vr = s.get_true_velocity()
        return abs(vr)


def test_vwheelmax_changes_prerotate_cruise_speed():
    """SET vWheelMax=<x> alone must change PRE_ROTATE's cruise wheel speed.

    Two distinct SET values (neither equal to the 400 mm/s compiled
    default), both below the yawRateMax-implied ceiling (~78 mm/s) so
    vWheelMax's saturation is the binding constraint in both measurements
    (see module-level note above for why this exercises
    BodyVelocityController's saturation, not Planner::_startPreRotate's own
    clamp specifically).
    """
    speed_low = _pre_rotate_cruise_speed(40)
    speed_high = _pre_rotate_cruise_speed(100)

    assert speed_high > speed_low * 2.0, (
        f"PRE_ROTATE cruise speed did not scale with vWheelMax: "
        f"vWheelMax=40 -> {speed_low:.2f} mm/s, "
        f"vWheelMax=100 -> {speed_high:.2f} mm/s (expected a clear increase)"
    )
    assert speed_low < 30.0, (
        f"vWheelMax=40 should bind well under the yawRateMax-implied ceiling "
        f"(~78 mm/s), got cruise speed {speed_low:.2f} mm/s"
    )


# ---------------------------------------------------------------------------
# rotGainPos / rotGainNeg — PRE_ROTATE feedforward gain
# (Planner::_startPreRotate, PlannerBegin.cpp)
#
#   dirGain = (turnSign > 0) ? rotationGainPos : rotationGainNeg
#   wheelSpd = speed / dirGain;  omega = turnSign * 2 * wheelSpd / trackwidthMm
#
# vWheelMax is pinned to a very large value in both measurements so its
# clamp never binds (isolating the gain's effect); the requested G `speed`
# is kept small (20 mm/s) so the resulting omega stays well under the
# yawRateMax-implied ceiling for every gain value tested (same masking
# concern as the vWheelMax test above).
# ---------------------------------------------------------------------------

def _pre_rotate_gain_speed(gain_key: str, gain_value: float, ty_sign: int,
                            window_ms: int = 900) -> float:
    """Fresh Sim(): SET the given rotation-gain key, command a G with a
    bearing whose sign selects rotGainPos (ty_sign > 0) or rotGainNeg
    (ty_sign < 0), and return the steady-state |right wheel speed| (mm/s)
    partway through the PRE_ROTATE phase (before the HEADING gate can
    plausibly fire -- empirically confirmed comfortably ahead of the
    fastest gain-value's ~1.1-1.2 s transition to PURSUE).
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        r = s.send_command("SET vWheelMax=100000")
        assert "OK" in r.upper(), r
        r = s.send_command(f"SET {gain_key}={gain_value}")
        assert "OK" in r.upper(), f"SET {gain_key}={gain_value} -> {r!r}"
        ty = 9999 if ty_sign > 0 else -9999
        r = s.send_command(f"G 1 {ty} 20")
        assert "OK" in r.upper(), f"G 1 {ty} 20 -> {r!r}"
        s.tick_for(window_ms, step_ms=24)
        _vl, vr = s.get_true_velocity()
        return abs(vr)


def test_rotgainpos_changes_prerotate_speed():
    """SET rotGainPos=<x> alone must change PRE_ROTATE's cruise speed for a
    CCW (bearing > 0) pre-rotate. Two distinct values, neither equal to the
    0.956 compiled default; a SMALLER gain must produce a LARGER wheel
    speed (wheelSpd = speed / dirGain).
    """
    speed_low_gain = _pre_rotate_gain_speed("rotGainPos", 0.3, ty_sign=+1)
    speed_high_gain = _pre_rotate_gain_speed("rotGainPos", 1.5, ty_sign=+1)

    assert speed_low_gain > speed_high_gain * 2.0, (
        f"PRE_ROTATE speed did not scale (inversely) with rotGainPos: "
        f"rotGainPos=0.3 -> {speed_low_gain:.2f} mm/s, "
        f"rotGainPos=1.5 -> {speed_high_gain:.2f} mm/s "
        f"(expected the smaller gain to drive a clearly larger speed)"
    )


def test_rotgainneg_changes_prerotate_speed():
    """SET rotGainNeg=<x> alone must change PRE_ROTATE's cruise speed for a
    CW (bearing < 0) pre-rotate. Mirrors test_rotgainpos_changes_prerotate_speed
    with the opposite bearing sign so rotationGainNeg (not Pos) is read.
    """
    speed_low_gain = _pre_rotate_gain_speed("rotGainNeg", 0.3, ty_sign=-1)
    speed_high_gain = _pre_rotate_gain_speed("rotGainNeg", 1.5, ty_sign=-1)

    assert speed_low_gain > speed_high_gain * 2.0, (
        f"PRE_ROTATE speed did not scale (inversely) with rotGainNeg: "
        f"rotGainNeg=0.3 -> {speed_low_gain:.2f} mm/s, "
        f"rotGainNeg=1.5 -> {speed_high_gain:.2f} mm/s "
        f"(expected the smaller gain to drive a clearly larger speed)"
    )


# ---------------------------------------------------------------------------
# turnGate — PRE_ROTATE vs PURSUE branch decision (Planner::beginGoTo,
# PlannerBegin.cpp)
#
#   if (bearing > turnInPlaceGate) { pre-rotate first } else { pursue directly }
#
# A fixed 50 deg bearing with turnGate=80 (> bearing) takes the PURSUE
# branch immediately (forward body velocity ramps up right away); the same
# bearing with turnGate=10 (< bearing) takes the PRE_ROTATE branch (v stays
# at 0 while the robot spins in place first). This is a clean binary
# discriminator, not a continuous-scaling one.
# ---------------------------------------------------------------------------

def _fused_v_after_goto(turn_gate, window_ms: int = 500) -> float:
    """Fresh Sim(): SET turnGate, command a G with a fixed 50 deg bearing,
    and return the fused forward speed after a short fixed window.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        r = s.send_command(f"SET turnGate={turn_gate}")
        assert "OK" in r.upper(), f"SET turnGate={turn_gate} -> {r!r}"
        # tx=643, ty=766 => bearing = atan2(766, 643) ~= 50.0 deg.
        r = s.send_command("G 643 766 200")
        assert "OK" in r.upper(), f"G 643 766 200 -> {r!r}"
        s.tick_for(window_ms, step_ms=24)
        return s.get_fused_v()


def test_turngate_changes_prerotate_vs_pursue_branch():
    """SET turnGate=<x> alone must change whether a fixed 50 deg bearing
    triggers PRE_ROTATE or goes directly to PURSUE.

    Two distinct values, neither equal to the 35 deg compiled default:
    turnGate=80 (> 50 deg bearing) must pursue immediately (v ramps up);
    turnGate=10 (< 50 deg bearing) must pre-rotate first (v stays at 0).
    """
    v_pursue_direct = _fused_v_after_goto(80)
    v_pre_rotate_first = _fused_v_after_goto(10)

    assert v_pursue_direct > 50.0, (
        f"turnGate=80 (> 50 deg bearing) should pursue immediately "
        f"(nonzero forward speed within 500 ms), got fused_v={v_pursue_direct:.2f} mm/s"
    )
    assert v_pre_rotate_first < 5.0, (
        f"turnGate=10 (< 50 deg bearing) should pre-rotate first (no forward "
        f"speed yet within 500 ms), got fused_v={v_pre_rotate_first:.2f} mm/s"
    )


# ---------------------------------------------------------------------------
# ctrlPeriod — Planner's own tick-throttle (Planner::driveAdvance,
# Planner.cpp)
#
#   if ((now_ms - _lastTickMs) < controlPeriodMs) return;
#
# Planner::tick() (called every simulated loop iteration, unthrottled)
# always re-packs whatever body twist driveAdvance() last computed into the
# outgoing DrivetrainCommand{TWIST} -- so when driveAdvance() is starved by
# a large controlPeriodMs, the BVC's ramp never advances and the robot
# never starts moving, regardless of how much wall-clock/sim time elapses.
# ---------------------------------------------------------------------------

def _distance_traveled(ctrl_period_ms, window_ms: int = 500) -> float:
    """Fresh Sim(): SET ctrlPeriod, issue a D command, and return the
    per-wheel travel (mm, plant truth) after a short fixed window.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        r = s.send_command(f"SET ctrlPeriod={ctrl_period_ms}")
        assert "OK" in r.upper(), f"SET ctrlPeriod={ctrl_period_ms} -> {r!r}"
        r = s.send_command("D 200 200 2000")
        assert "OK" in r.upper(), f"D 200 200 2000 -> {r!r}"
        s.tick_for(window_ms, step_ms=24)
        enc_l, _enc_r = s.get_true_wheel_travel()
        return abs(enc_l)


def test_ctrlperiod_throttles_planner_tick():
    """SET ctrlPeriod=<x> alone must change how much the robot moves within
    a short fixed window.

    Two distinct values, neither equal to the 10 ms compiled default:
    ctrlPeriod=5 (faster than the 24 ms sim step -- effectively unthrottled)
    must move substantially within 500 ms; ctrlPeriod=2000 (far larger than
    the window) must move essentially nothing, since driveAdvance() never
    fires within the window at all.
    """
    dist_fast = _distance_traveled(5)
    dist_throttled = _distance_traveled(2000)

    assert dist_fast > 20.0, (
        f"ctrlPeriod=5 should move substantially within a 500 ms window, "
        f"got {dist_fast:.2f} mm"
    )
    assert dist_throttled < 2.0, (
        f"ctrlPeriod=2000 should move essentially nothing within a 500 ms "
        f"window (driveAdvance never fires), got {dist_throttled:.2f} mm"
    )


# ---------------------------------------------------------------------------
# lag.otos — Drive::tickUpdate() OTOS-lag-gated fusion (067-002)
#
# Already the subject of a dedicated regression test
# (test_067_002_drive_drvcfg_shadow_ekf_predict.py::
#  test_set_lag_otos_alone_changes_otos_fusion_gate); included here too so
# the full STALE-key list has one, single, canonical sweep covering every
# row the audit table found live-but-broken.
# ---------------------------------------------------------------------------

def _otos_fusion_pulled_x(lag_value, injected_x: float = 500.0,
                           window_ms: int = 1000) -> float:
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s._lib.sim_begin_otos(s._h)
        r = s.send_command(f"SET lag.otos={lag_value}")
        assert "OK" in r.upper(), f"SET lag.otos={lag_value} -> {r!r}"
        s._lib.sim_set_otos_pose(s._h, injected_x, 0.0, 0.0)
        s.tick_for(window_ms, step_ms=24)
        return float(s._lib.sim_get_pose_x(s._h))


def test_lag_otos_changes_otos_fusion_gate():
    """SET lag.otos=<x> alone must change whether an injected OTOS pose is
    fused within a fixed window. Two distinct values, neither equal to the
    10 ms compiled default.
    """
    fused_x_short_lag = _otos_fusion_pulled_x(5)
    fused_x_long_lag = _otos_fusion_pulled_x(5000)

    assert fused_x_short_lag == pytest.approx(500.0, abs=5.0), (
        f"lag.otos=5 should let OTOS fusion pull fused x to ~500 mm within "
        f"a 1 s window, got {fused_x_short_lag:.2f}"
    )
    assert fused_x_long_lag == pytest.approx(0.0, abs=5.0), (
        f"lag.otos=5000 should suppress OTOS fusion for the full 1 s window, "
        f"got {fused_x_long_lag:.2f}"
    )


# ---------------------------------------------------------------------------
# ekfRHead — noise-only EKF update (067-003)
#
# Already the subject of a dedicated regression test
# (test_067_003_ekf_setnoise.py); included here too, in a lighter form, so
# the sweep is a complete, single-file record of every STALE key.
# ---------------------------------------------------------------------------

def _heading_correction_after_one_tick(ekf_r_head: float) -> float:
    with Sim() as sim:
        sim.send_command("SET sTimeout=60000")
        sim.send_command(f"SET ekfRHead={ekf_r_head}")
        sim.tick_for(500, step_ms=24)

        enc_x, enc_y, enc_h = sim.get_enc_pose()
        h_before = sim.get_fused_pose()[2]

        otos_h = enc_h + 0.2
        sim.set_otos_pose(enc_x, enc_y, otos_h)
        sim.set_otos_fusion(True)

        sim.tick_for(24, step_ms=24)

        h_after = sim.get_fused_pose()[2]
        rejects = sim.get_ekf_rej_count()

    assert rejects == 0, (
        f"heading update was gate-rejected at ekfRHead={ekf_r_head} "
        f"(rej_count={rejects})"
    )
    return abs(h_after - h_before)


def test_ekfrhead_changes_heading_correction_strength():
    """SET ekfRHead=<x> alone must change how strongly a single-tick OTOS
    heading disagreement is corrected. Two distinct values, neither equal
    to the 0.01 compiled default; a smaller (more-trusting) R must produce
    a markedly larger single-tick correction than a larger R.
    """
    correction_low_r = _heading_correction_after_one_tick(0.001)
    correction_high_r = _heading_correction_after_one_tick(5.0)

    assert correction_low_r > 0.01, (
        f"low-R correction too small to be meaningful: {correction_low_r:.5f} rad"
    )
    assert correction_low_r > correction_high_r * 1.5, (
        f"SET ekfRHead did not change heading-correction strength: "
        f"low-R (0.001) correction={correction_low_r:.5f} rad, "
        f"high-R (5.0) correction={correction_high_r:.5f} rad"
    )
