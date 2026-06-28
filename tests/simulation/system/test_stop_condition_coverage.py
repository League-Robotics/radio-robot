"""
test_stop_condition_coverage.py — fire each StopCondition::Kind through the C++
binary so its evaluate() branch is actually exercised (sprint 045 ticket 003).

The existing test_stop_condition.py is a pure-Python mirror (zero gcovr coverage),
and test_halt_controller.py only *registers* COLOR/LINE conditions without firing
them.  This module drives real motion + injected sensor values so the SENSOR,
COLOR, LINE_ANY, and ROTATION evaluate() branches of StopCondition.cpp run in the
instrumented binary.

Sensor injection uses the 045-003 sim-infra wrappers
sim_set_line_values / sim_set_color_rgbc (single-row schedule on the Sim
line/color sensor), exposed as Sim.set_line_values / Sim.set_color_rgbc.
init_line_sensor()/init_color_sensor() must run first so the LineSensor/
ColorSensor subsystem periodics read the injected values into HardwareState
(where StopCondition::evaluate and HaltController::evaluate see them).

These are whole-robot scenario tests (drive + sensors + halt), so they live in
tests/simulation/system/.
"""
import ctypes
import re

from firmware import Sim

TICK_STEP_MS = 24


def _find_evt_halt(evts: str):
    m = re.search(r"EVT halt id=(\d+)", evts)
    return int(m.group(1)) if m else None


def _tick_collect(s: Sim, n: int) -> str:
    """Tick n times, accumulating async events."""
    evts = ""
    for _ in range(n):
        s._lib.sim_tick(s._h, ctypes.c_uint32(s._t))
        s._t += TICK_STEP_MS
        evts += s.get_async_evts()
    return evts


# ---------------------------------------------------------------------------
# Kind::LINE_ANY — HALT LINE ANY GE <thr>; fired by an injected line channel.
# ---------------------------------------------------------------------------

def test_line_any_stop_fires_ge():
    """HALT LINE ANY GE <thr> fires when an injected line channel crosses it.

    Exercises StopCondition::evaluate Kind::LINE_ANY (the OR-across-4-channels
    short-circuit, GE branch).
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.init_line_sensor()
        # Start below threshold so it does not fire on registration.
        s.set_line_values(0, 0, 0, 0)

        s.send_command("VW 200 0")
        r = s.send_command("HALT LINE ANY GE 500")
        assert "id=" in r, f"HALT LINE register failed: {r!r}"

        # Tick a bit below threshold — must NOT fire.
        early = _tick_collect(s, 5)
        assert _find_evt_halt(early) is None, (
            f"LINE_ANY fired before the channel crossed threshold: {early!r}"
        )

        # Raise channel 2 above threshold (tests the OR short-circuit on a
        # non-zero index) and tick — must fire now.
        s.set_line_values(0, 0, 700, 0)
        late = _tick_collect(s, 10)
        assert _find_evt_halt(late) is not None, (
            f"LINE_ANY GE 500 did not fire after injecting line2=700: {late!r}"
        )


def test_line_any_stop_fires_le():
    """HALT LINE ANY LE <thr> fires when a line channel drops to/below it.

    Exercises the LE branch of Kind::LINE_ANY.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.init_line_sensor()
        # Start ALL channels above the LE threshold.  Start driving, THEN settle
        # several ticks so the LineSensor periodic populates HardwareState.line[]
        # with the HIGH values and they are fresh, BEFORE the HALT is registered.
        # HaltController::evaluate runs before lineSensor.periodic each tick, so
        # the HALT sees the value from the prior tick — settling first guarantees
        # it sees 900, not the default-schedule low values (which would trip LE).
        s.set_line_values(900, 900, 900, 900)
        s.send_command("VW 200 0")
        _tick_collect(s, 4)

        r = s.send_command("HALT LINE ANY LE 100")
        assert "id=" in r, f"HALT LINE LE register failed: {r!r}"

        early = _tick_collect(s, 5)
        assert _find_evt_halt(early) is None, (
            f"LINE_ANY LE fired while all channels were high: {early!r}"
        )

        # Drop channel 0 to/below threshold.
        s.set_line_values(50, 900, 900, 900)
        late = _tick_collect(s, 10)
        assert _find_evt_halt(late) is not None, (
            f"LINE_ANY LE 100 did not fire after line0 dropped to 50: {late!r}"
        )


# ---------------------------------------------------------------------------
# Kind::COLOR — HALT COLOR <h> <s> <v> <dist>; fired by an injected RGBC value.
# ---------------------------------------------------------------------------

def test_color_stop_fires_on_match():
    """HALT COLOR fires when the injected RGBC maps to an HSV near the target.

    Exercises Kind::COLOR: rgbToHSV(), hueDistance(), sqrtf HSV distance, and the
    `dist <= ax` fire test.

    Inject a pure-red RGBC (r high, g=b low, c=normaliser).  rgbToHSV gives
    hue≈0, high saturation, value≈1.  Register a COLOR target near that HSV with
    a generous distance threshold so the match fires.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.init_color_sensor()
        # Pure red: r=c (value≈1), g=b=0 (saturation≈1, hue≈0).
        s.set_color_rgbc(255, 0, 0, 255)

        s.send_command("VW 200 0")
        # target hue 0, sat 1.0, val 1.0, distance threshold 0.5 (generous).
        r = s.send_command("HALT COLOR 0 1.0 1.0 0.5")
        assert "id=" in r, f"HALT COLOR register failed: {r!r}"

        late = _tick_collect(s, 10)
        assert _find_evt_halt(late) is not None, (
            f"COLOR stop did not fire on a matching red RGBC injection: {late!r}"
        )


def test_color_stop_does_not_fire_on_mismatch():
    """HALT COLOR does NOT fire when the injected colour is far from the target.

    Exercises the Kind::COLOR `dist > ax` non-fire path (and the C==0 / wrong-hue
    distance computation).  Inject pure red but target green with a tight
    threshold — the HSV distance exceeds the threshold so it must not fire.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.init_color_sensor()
        s.set_color_rgbc(255, 0, 0, 255)        # pure red

        s.send_command("VW 200 0")
        # Target green (hue 120) with a tight distance — red is far away.
        r = s.send_command("HALT COLOR 120 1.0 1.0 0.1")
        assert "id=" in r, f"HALT COLOR register failed: {r!r}"

        evts = _tick_collect(s, 10)
        assert _find_evt_halt(evts) is None, (
            f"COLOR stop fired on a non-matching colour (red vs green target): {evts!r}"
        )


# ---------------------------------------------------------------------------
# Kind::SENSOR — DOCUMENTED UNREACHABLE-IN-SIM (queue path drops the stop).
#
# Sprint 052-001: the queue-path prefix bug was fixed.  parseT/parseD/parseTURN
# now pack sensor= tokens with the full "sensor=<value>" prefix (via
# mc_packStopKVs), and handleVW calls mc_applyStopClauses which correctly
# matches both "stop=" and "sensor=" prefixes.  Kind::SENSOR now fires on the
# queue path exactly as on the direct path.
# ---------------------------------------------------------------------------

def test_sensor_stop_fires_on_queue_path():
    """Queue-path SENSOR stop fires when line0 crosses threshold (052-001 fix).

    `T 200 200 9000 sensor=line0:ge:500` attaches a SENSOR stop; when line0 is
    injected above 500 the drive terminates early (EVT done), leaving PWM at 0.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.init_line_sensor()
        s.set_line_values(0, 0, 0, 0)
        _tick_collect(s, 3)

        r = s.send_command("T 200 200 9000 sensor=line0:ge:500")
        assert "ERR" not in r.upper(), f"valid sensor= token wrongly rejected: {r!r}"

        _tick_collect(s, 5)
        # Cross the threshold well above 500.
        s.set_line_values(900, 0, 0, 0)
        evts = _tick_collect(s, 80)  # generous window; SENSOR stop should fire quickly

        pwm_l = float(s._lib.sim_get_pwm_l(s._h))
        assert "EVT done" in evts or pwm_l == 0.0, (
            f"SENSOR stop did not fire on queue path after line0 crossed threshold "
            f"(pwm_l={pwm_l}, evts={evts!r})"
        )


# ---------------------------------------------------------------------------
# Kind::ROTATION — RT <cdeg> spin-in-place stops on encoder arc differential.
# ---------------------------------------------------------------------------

def test_rotation_stop_terminates_spin():
    """RT <cdeg> spins in place and stops when the encoder arc reaches target.

    RT registers a ROTATION StopCondition (makeRotationStop) whose evaluate()
    uses (encR - encL) - encDiff0.  Drive a relative turn and confirm the spin
    terminates within a bounded window (EVT done / pwm zero), exercising
    Kind::ROTATION.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        r = s.send_command("RT 9000")            # +90.00 deg relative spin
        assert "ERR" not in r.upper(), f"RT rejected: {r!r}"

        # Spin should run then self-terminate as the encoder differential grows.
        evts = _tick_collect(s, 200)             # generous window (~4.8 s)

        pwm_l = float(s._lib.sim_get_pwm_l(s._h))
        pwm_r = float(s._lib.sim_get_pwm_r(s._h))
        assert ("EVT" in evts) or (pwm_l == 0.0 and pwm_r == 0.0), (
            f"RT 9000 ROTATION stop never terminated the spin "
            f"(pwm_l={pwm_l}, pwm_r={pwm_r}, evts={evts!r})"
        )
