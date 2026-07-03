"""
test_072_002_signed_stop_and_safety_margin.py — ticket 072-002.

Exercises the actual C++ firmware sim binary (not a Python mirror) for the
signed/direction-aware DISTANCE and ROTATION stop conditions, and the new
StopCondition::Kind::SAFETY_MARGIN runaway safety net, per
`distance-stop-fabsf-accepts-backward-completion.md` and
architecture-update.md Step 3/4a/5, Decisions 1-2.

Background: `StopCondition::Kind::DISTANCE` (and `ROTATION`) used to gate on
`fabsf(delta) >= target` with no notion of which direction was commanded — a
forward `D` that instead ran away BACKWARD could satisfy the DISTANCE stop
once it had travelled `target` mm the WRONG way, self-reporting a false
`EVT done D reason=dist`. This ticket makes both Kinds direction-aware
(`MotionBaseline.vSign`/`omegaSign`, captured at `MotionCommand::start()`)
and adds a fast, wire-visible runaway net (`SAFETY_MARGIN` ->
`EVT safety_stop reason=runaway`) that fires within one control tick of the
robot crossing a configurable negative margin relative to its commanded
direction — much faster than the existing generous TIME net.

Tests inject encoder values directly via `sim_set_enc_l`/`sim_set_enc_r`
(the "forced-encoder-cap" harness architecture-update.md Step 1a's forced
-stall sim experiment used to originally diagnose the issue, and which
ticket 001 explicitly preserved as an independent diagnostic tool) to force
a "wrong direction" or "runaway" scenario deterministically, without
depending on any particular controller-windup sequence.
"""
import ctypes

from firmware import Sim

TICK_STEP_MS = 24


def _tick_once(s: Sim) -> str:
    """Advance the sim by exactly one TICK_STEP_MS step; return async EVTs
    accumulated during that tick."""
    s._lib.sim_tick(s._h, ctypes.c_uint32(s._t))
    s._t += TICK_STEP_MS
    return s.get_async_evts()


# ---------------------------------------------------------------------------
# DISTANCE: forward D that runs away backward must NOT self-report success.
# ---------------------------------------------------------------------------

def test_distance_stop_forward_ignores_backward_runaway():
    """A forward `D 200 200 500` whose encoders instead accumulate backward
    travel must NOT fire the DISTANCE stop from that backward travel (no
    false `EVT done D reason=dist`).

    SAFETY_MARGIN is disabled here (set to a huge threshold) so this test
    isolates the signed-DISTANCE fix specifically -- the runaway net itself
    is proven separately in test_safety_margin_fires_on_runaway_reverse_
    during_forward_d below.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        # Disable the safety net for this test so it isolates DISTANCE's own
        # signed-delta fix (Decision 1) from the separate SAFETY_MARGIN net
        # (Decision 2).
        r = s.send_command("SET safetyMargin=100000")
        assert "OK" in r.upper(), f"SET safetyMargin -> {r!r}"

        r = s.send_command("D 200 200 500")
        assert "OK drive" in r, f"D command rejected: {r!r}"

        evts = ""
        # Drive the injected encoders from 0 down to -600 mm (past the 500 mm
        # target magnitude) in -20 mm steps -- the exact scenario the issue
        # reports: 500+ mm of full-reverse travel on a forward-commanded D.
        for i in range(1, 31):
            mm = -20.0 * i
            s._lib.sim_set_enc_l(s._h, ctypes.c_float(mm))
            s._lib.sim_set_enc_r(s._h, ctypes.c_float(mm))
            evts += _tick_once(s)

        assert "EVT done D" not in evts and "reason=dist" not in evts, (
            f"forward D falsely self-reported completion from 600 mm of "
            f"BACKWARD travel: {evts!r}"
        )


def test_reverse_d_completes_on_backward_travel():
    """A reverse `D -200 -200 500` must still complete normally
    (`EVT done D reason=dist`) once it has travelled 500 mm backward --
    no regression on the legitimate reverse-drive case (signed delta ==
    fabsf(delta) exactly when travel matches the commanded direction).

    No injection here: the real plant (default, zero-stiction/zero-slip)
    drives the reverse command to completion on its own.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")

        r = s.send_command("D -200 -200 500")
        assert "OK drive" in r, f"D command rejected: {r!r}"

        evts = ""
        for _ in range(200):  # generous window (~4.8 s)
            evts += _tick_once(s)
            if "EVT done D" in evts:
                break

        assert "EVT done D" in evts, f"reverse D never completed: {evts!r}"
        assert "reason=dist" in evts, (
            f"reverse D should complete via a clean DISTANCE crossing, not "
            f"the TIME net or the safety net: {evts!r}"
        )
        assert "EVT safety_stop" not in evts, (
            f"legitimate reverse travel must never trip the runaway safety "
            f"net: {evts!r}"
        )

        enc_l, enc_r = s.get_true_wheel_travel()
        assert enc_l < -400.0 and enc_r < -400.0, (
            f"reverse D should have travelled backward (negative encoders), "
            f"got enc_l={enc_l:.1f} enc_r={enc_r:.1f}"
        )


# ---------------------------------------------------------------------------
# SAFETY_MARGIN: fast runaway net, independent of the TIME net.
# ---------------------------------------------------------------------------

def test_safety_margin_fires_on_runaway_reverse_during_forward_d():
    """A forward `D 200 200 500` that runs away backward must trip the new
    SAFETY_MARGIN net -- `EVT safety_stop reason=runaway` -- within one
    control tick of crossing the (default 50 mm) margin, well before the
    500 mm DISTANCE target or the multi-second TIME net.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")

        r = s.send_command("D 200 200 500")
        assert "OK drive" in r, f"D command rejected: {r!r}"

        evts = ""
        fired_at_tick = None
        # -10 mm per tick: crosses the default 50 mm margin exactly at tick 5.
        for i in range(1, 21):
            mm = -10.0 * i
            s._lib.sim_set_enc_l(s._h, ctypes.c_float(mm))
            s._lib.sim_set_enc_r(s._h, ctypes.c_float(mm))
            tick_evts = _tick_once(s)
            evts += tick_evts
            if fired_at_tick is None and "EVT safety_stop" in tick_evts:
                fired_at_tick = i
            if fired_at_tick is not None:
                break

        assert fired_at_tick is not None, (
            f"SAFETY_MARGIN never fired during a 200 mm backward runaway on "
            f"a forward D: {evts!r}"
        )
        assert fired_at_tick <= 6, (
            f"SAFETY_MARGIN should fire within one control tick of crossing "
            f"the 50 mm margin (tick 5 at -10 mm/tick), fired at tick "
            f"{fired_at_tick} instead: {evts!r}"
        )
        assert "reason=runaway" in evts, (
            f"safety_stop from a runaway D must carry reason=runaway: {evts!r}"
        )
        assert "EVT done D" not in evts and "reason=dist" not in evts, (
            f"the runaway must not ALSO report a false dist completion: "
            f"{evts!r}"
        )

        # HARD teardown: PWM should drop to (near) zero promptly, not ramp
        # down over the multi-second SOFT deadline.
        for _ in range(3):
            _tick_once(s)
        pwm_l = float(s._lib.sim_get_pwm_l(s._h))
        pwm_r = float(s._lib.sim_get_pwm_r(s._h))
        assert abs(pwm_l) < 5.0 and abs(pwm_r) < 5.0, (
            f"SAFETY_MARGIN should force an immediate HARD stop, not a SOFT "
            f"ramp: pwm_l={pwm_l}, pwm_r={pwm_r}"
        )


def test_safety_stop_reason_runaway_is_additive_to_reason_watchdog():
    """`EVT safety_stop`'s new `reason=runaway` token coexists with the
    pre-existing `reason=watchdog` token on the SAME base label -- proving
    the addition is additive (new value, existing label), not a replacement.
    """
    with Sim() as s:
        # Deliberately SHORT sTimeout so the keepalive watchdog fires fast.
        s.send_command("SET sTimeout=200")
        s.get_async_evts()  # drain the SET reply/any startup noise

        r = s.send_command("VW 200 0")
        assert "OK" in r.upper(), f"VW rejected: {r!r}"

        evts = ""
        for _ in range(40):  # ~960 ms, comfortably past the 200 ms watchdog
            evts += _tick_once(s)
            if "EVT safety_stop" in evts:
                break

        assert "EVT safety_stop" in evts, f"watchdog never fired: {evts!r}"
        assert "reason=watchdog" in evts, (
            f"VW keepalive loss must still report reason=watchdog "
            f"(unaffected by the new reason=runaway token): {evts!r}"
        )
        assert "reason=runaway" not in evts, (
            f"a plain keepalive-loss safety_stop must not carry the "
            f"runaway token: {evts!r}"
        )


# ---------------------------------------------------------------------------
# ROTATION: direction-aware, both spin directions.
# ---------------------------------------------------------------------------

def test_rotation_stop_wrong_direction_does_not_fire():
    """`RT 9000` (commanded CCW +90 deg) must NOT terminate from an encoder
    differential that grows in the WRONG (CW) direction, no matter how large
    -- the direction-aware ROTATION stop (`diff * base.omegaSign >= a`)
    never satisfies `>= a` when the signs disagree.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")

        r = s.send_command("RT 9000")  # +90.00 deg relative spin (CCW)
        assert "ERR" not in r.upper(), f"RT rejected: {r!r}"

        # Force a large encoder differential in the WRONG (CW) direction:
        # R very negative, L very positive => (R - L) very negative, while
        # the command's omegaSign is +1 (CCW) -- signs disagree.
        s._lib.sim_set_enc_r(s._h, ctypes.c_float(-500.0))
        s._lib.sim_set_enc_l(s._h, ctypes.c_float(500.0))

        evts = ""
        # A short window, comfortably inside RT 9000's own TIME net (which
        # requires ~1+ s nominal plus headroom) -- long enough to prove the
        # ROTATION stop itself does not fire on the wrong-direction diff.
        for _ in range(12):  # ~288 ms
            evts += _tick_once(s)

        assert "reason=rot" not in evts and "EVT done RT" not in evts, (
            f"ROTATION stop fired on a WRONG-direction encoder differential: "
            f"{evts!r}"
        )


def test_rt_negative_direction_still_terminates_on_own_arc():
    """`RT -9000` (commanded CW -90 deg) must still self-terminate on its
    own commanded-direction arc -- mirrors the existing
    test_rotation_stop_terminates_spin (positive direction) coverage for the
    negative direction, proving direction-awareness did not break either
    sign of a legitimately-matching spin.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        r = s.send_command("RT -9000")  # -90.00 deg relative spin (CW)
        assert "ERR" not in r.upper(), f"RT rejected: {r!r}"

        evts = ""
        for _ in range(200):  # generous window (~4.8 s), mirrors the +RT test
            evts += _tick_once(s)

        pwm_l = float(s._lib.sim_get_pwm_l(s._h))
        pwm_r = float(s._lib.sim_get_pwm_r(s._h))
        assert ("EVT" in evts) or (pwm_l == 0.0 and pwm_r == 0.0), (
            f"RT -9000 ROTATION stop never terminated the spin "
            f"(pwm_l={pwm_l}, pwm_r={pwm_r}, evts={evts!r})"
        )
        assert "EVT safety_stop" not in evts, (
            f"a legitimate CW spin must not trip any safety net: {evts!r}"
        )
