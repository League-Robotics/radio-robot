#!/usr/bin/env python3
"""test_stop_condition.py — Unit tests for StopCondition logic (017-003).

Pure Python mirrors of each Kind's evaluate() logic from:
  source/control/StopCondition.h / .cpp

Tests verify:
  - Each Kind fires at its exact threshold and not before.
  - OR-across-array: first condition to fire wins.
  - NONE: always returns False.
  - SENSOR GE and LE fire correctly.
  - Zero stop conditions: no self-termination.
  - Baseline delta (distance / heading) computed from snapshot.
  - DISTANCE is direction-aware (072-004): gates on the SIGNED delta
    (raw * base.vSign), not fabsf(raw) — see TestDistance.

Implementation note: these tests mirror the C++ evaluate() logic in Python.
They do NOT test the C++ binary — they validate the algorithm independently.
For the real-binary/end-to-end direction-aware + SAFETY_MARGIN coverage,
see tests/simulation/unit/test_072_002_signed_stop_and_safety_margin.py.
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Python mirrors of HardwareState and MotionBaseline
# ---------------------------------------------------------------------------

class HardwareState:
    """Minimal subset of HardwareState fields used by StopCondition."""

    def __init__(self, **kwargs):
        # Encoder odometry (mm, cumulative)
        self.encLMm   = kwargs.get('encLMm',   0.0)
        self.encRMm   = kwargs.get('encRMm',   0.0)
        # Pose
        self.poseX    = kwargs.get('poseX',    0.0)
        self.poseY    = kwargs.get('poseY',    0.0)
        self.poseHrad = kwargs.get('poseHrad', 0.0)
        # Line sensor (4-channel)
        line = kwargs.get('line', [0, 0, 0, 0])
        self.line = list(line)
        # Color sensor (RGBC)
        self.colorR = kwargs.get('colorR', 0)
        self.colorG = kwargs.get('colorG', 0)
        self.colorB = kwargs.get('colorB', 0)
        self.colorC = kwargs.get('colorC', 0)
        # Analog inputs (4-channel)
        analog = kwargs.get('analogIn', [0, 0, 0, 0])
        self.analogIn = list(analog)


class MotionBaseline:
    """Baseline snapshot captured at command start."""

    def __init__(self, **kwargs):
        self.t0Ms       = kwargs.get('t0Ms',       0)
        self.enc0Mm     = kwargs.get('enc0Mm',     0.0)
        self.heading0Rad = kwargs.get('heading0Rad', 0.0)
        self.pose0X     = kwargs.get('pose0X',     0.0)
        self.pose0Y     = kwargs.get('pose0Y',     0.0)
        # vSign (072-004): commanded-direction sign captured at
        # MotionCommand::start(), mirrors source/control/StopCondition.h's
        # MotionBaseline.vSign. Default +1.0 (forward) so every pre-existing
        # DISTANCE test above, which only ever drives positive (forward)
        # travel, is unaffected by the signed-delta change below.
        self.vSign      = kwargs.get('vSign',      1.0)


# ---------------------------------------------------------------------------
# Python mirror of StopCondition.evaluate()
# ---------------------------------------------------------------------------

def wrap_angle(x: float) -> float:
    """Wrap x into (-π, π] using atan2(sin, cos) — matches C++ implementation."""
    return math.atan2(math.sin(x), math.cos(x))


def get_sensor_value(s: HardwareState, channel: int) -> float:
    """Map channel selector to sensor value — matches getSensorValue() in .cpp."""
    if channel < 4:
        return float(s.line[channel])
    if channel == 4:  return float(s.colorR)
    if channel == 5:  return float(s.colorG)
    if channel == 6:  return float(s.colorB)
    if channel == 7:  return float(s.colorC)
    if channel == 8:  return float(s.analogIn[0])
    if channel == 9:  return float(s.analogIn[1])
    if channel == 10: return float(s.analogIn[2])
    if channel == 11: return float(s.analogIn[3])
    return 0.0


def evaluate(kind: str, a: float, b: float, ax: float,
             sensor: int, cmp_dir: str,
             s: HardwareState, now_ms: int,
             base: MotionBaseline) -> bool:
    """Python mirror of StopCondition::evaluate() for all Kinds."""
    if kind == 'NONE':
        return False

    elif kind == 'TIME':
        elapsed = now_ms - base.t0Ms      # signed Python int; no overflow
        return elapsed >= int(a)

    elif kind == 'DISTANCE':
        # 072-004: signed-delta gate (was fabsf(raw) >= a), mirroring
        # source/control/StopCondition.cpp's Kind::DISTANCE fix
        # (distance-stop-fabsf-accepts-backward-completion.md). A drive that
        # travels in the commanded direction is unaffected (signedDelta ==
        # |raw| when raw already agrees with base.vSign); a drive that runs
        # the WRONG way no longer satisfies the stop from that wrong-way
        # travel alone.
        enc_avg = (s.encLMm + s.encRMm) * 0.5
        raw = enc_avg - base.enc0Mm
        signed_delta = raw * base.vSign
        return signed_delta >= a

    elif kind == 'HEADING':
        current_delta = wrap_angle(s.poseHrad - base.heading0Rad)
        error = wrap_angle(current_delta - a)
        return abs(error) < b

    elif kind == 'POSITION':
        # ax = target X, a = target Y, b = radius.
        dx = s.poseX - ax
        dy = s.poseY - a
        dist2 = dx * dx + dy * dy
        return dist2 < (b * b)

    elif kind == 'SENSOR':
        val = get_sensor_value(s, sensor)
        if cmp_dir == 'GE':
            return val >= a
        else:
            return val <= a

    return False


# ---------------------------------------------------------------------------
# Helper: evaluate an OR-array of conditions
# ---------------------------------------------------------------------------

def evaluate_array(conditions: list[dict],
                   s: HardwareState, now_ms: int,
                   base: MotionBaseline) -> int:
    """
    Evaluate an array of conditions OR-combined.
    Returns the index of the first firing condition, or -1 if none fire.
    Mirrors MotionCommand::tick() stop-evaluation loop.
    """
    for i, c in enumerate(conditions):
        if evaluate(
            c.get('kind',   'NONE'),
            c.get('a',      0.0),
            c.get('b',      0.0),
            c.get('ax',     0.0),
            c.get('sensor', 0),
            c.get('cmp',    'GE'),
            s, now_ms, base
        ):
            return i
    return -1


# ---------------------------------------------------------------------------
# Tests — NONE
# ---------------------------------------------------------------------------

class TestNone:
    """NONE always returns False."""

    def test_none_always_false(self):
        s    = HardwareState()
        base = MotionBaseline()
        result = evaluate('NONE', 0.0, 0.0, 0.0, 0, 'GE', s, 0, base)
        assert result is False

    def test_none_false_with_nonzero_state(self):
        s    = HardwareState(encLMm=500.0, encRMm=500.0, poseHrad=3.14)
        base = MotionBaseline(t0Ms=0, enc0Mm=0.0)
        result = evaluate('NONE', 1000.0, 0.0, 0.0, 0, 'GE', s, 99999, base)
        assert result is False


# ---------------------------------------------------------------------------
# Tests — TIME
# ---------------------------------------------------------------------------

class TestTime:
    """TIME fires when elapsed >= threshold; not before."""

    def test_time_fires_at_threshold(self):
        """At exactly 1000 ms, TIME(a=1000) fires."""
        base = MotionBaseline(t0Ms=5000)
        s    = HardwareState()
        # now_ms = 6000, elapsed = 1000 ms — exactly at threshold
        assert evaluate('TIME', 1000.0, 0.0, 0.0, 0, 'GE', s, 6000, base) is True

    def test_time_does_not_fire_one_tick_before(self):
        """At 999 ms, TIME(a=1000) does not fire."""
        base = MotionBaseline(t0Ms=5000)
        s    = HardwareState()
        # now_ms = 5999, elapsed = 999 ms — one ms short
        assert evaluate('TIME', 1000.0, 0.0, 0.0, 0, 'GE', s, 5999, base) is False

    def test_time_fires_after_threshold(self):
        """At 1500 ms, TIME(a=1000) fires."""
        base = MotionBaseline(t0Ms=0)
        s    = HardwareState()
        assert evaluate('TIME', 1000.0, 0.0, 0.0, 0, 'GE', s, 1500, base) is True

    def test_time_zero_threshold_fires_immediately(self):
        """TIME(a=0) fires at t0 itself."""
        base = MotionBaseline(t0Ms=100)
        s    = HardwareState()
        assert evaluate('TIME', 0.0, 0.0, 0.0, 0, 'GE', s, 100, base) is True

    def test_time_does_not_fire_before_t0(self):
        """Elapsed is negative before t0 — must not fire."""
        base = MotionBaseline(t0Ms=5000)
        s    = HardwareState()
        # now_ms < t0Ms: Python signed subtraction gives negative
        assert evaluate('TIME', 500.0, 0.0, 0.0, 0, 'GE', s, 4999, base) is False


# ---------------------------------------------------------------------------
# Tests — DISTANCE
# ---------------------------------------------------------------------------

class TestDistance:
    """DISTANCE fires when signed(enc_avg - enc0) >= threshold; not one mm
    short, and not from wrong-direction travel (072-004:
    distance-stop-fabsf-accepts-backward-completion.md)."""

    def test_distance_fires_at_threshold(self):
        """Exactly 200 mm of travel fires DISTANCE(a=200)."""
        base = MotionBaseline(enc0Mm=100.0)       # enc0 = 100 mm
        # encLMm=200, encRMm=200 → enc_avg=200 → traveled=100 mm  -- not enough
        # Need traveled=200 mm: enc_avg = 100 + 200 = 300
        s = HardwareState(encLMm=300.0, encRMm=300.0)
        assert evaluate('DISTANCE', 200.0, 0.0, 0.0, 0, 'GE', s, 0, base) is True

    def test_distance_does_not_fire_one_mm_short(self):
        """199 mm of travel does NOT fire DISTANCE(a=200)."""
        base = MotionBaseline(enc0Mm=100.0)
        # enc_avg = 100 + 199 = 299 → traveled = 199 mm
        s = HardwareState(encLMm=299.0, encRMm=299.0)
        assert evaluate('DISTANCE', 200.0, 0.0, 0.0, 0, 'GE', s, 0, base) is False

    def test_distance_uses_raw_encoder_average(self):
        """DISTANCE uses (encLMm + encRMm)/2, not encLMm alone."""
        base = MotionBaseline(enc0Mm=0.0)
        # encL=300, encR=100 → enc_avg = 200 → traveled = 200 mm
        s = HardwareState(encLMm=300.0, encRMm=100.0)
        assert evaluate('DISTANCE', 200.0, 0.0, 0.0, 0, 'GE', s, 0, base) is True

    def test_distance_fires_for_commanded_reverse(self):
        """072-004 (split from test_distance_fires_for_reverse): a
        commanded-reverse D (vSign=-1) still completes on backward travel —
        no regression on the legitimate reverse-drive case.
        signedDelta = raw * vSign flips the negative raw delta positive,
        exactly matching the fabsf outcome for this direction-matching case.
        """
        base = MotionBaseline(enc0Mm=200.0, vSign=-1.0)
        # enc_avg = 0 → raw = -200 (backward travel, matches commanded reverse)
        s = HardwareState(encLMm=0.0, encRMm=0.0)
        assert evaluate('DISTANCE', 200.0, 0.0, 0.0, 0, 'GE', s, 0, base) is True

    def test_distance_does_not_fire_for_wrong_direction_travel(self):
        """072-004 (split from test_distance_fires_for_reverse, NEW case):
        a forward-commanded D (vSign=+1) that instead travels BACKWARD must
        NOT fire — this is the exact scenario
        distance-stop-fabsf-accepts-backward-completion.md reports (a
        forward D running away backward used to self-report
        `EVT done D reason=dist` once it had gone the target magnitude the
        WRONG way). This test encodes the fix, not just documents it: under
        the OLD fabsf(raw) >= a semantics this would have fired (traveled).
        """
        base = MotionBaseline(enc0Mm=200.0, vSign=1.0)
        # enc_avg = 0 → raw = -200 (backward travel), commanded forward
        s = HardwareState(encLMm=0.0, encRMm=0.0)
        assert evaluate('DISTANCE', 200.0, 0.0, 0.0, 0, 'GE', s, 0, base) is False

    def test_distance_zero_threshold_fires_immediately(self):
        """DISTANCE(a=0) fires when enc_avg == enc0 (zero travel)."""
        base = MotionBaseline(enc0Mm=50.0)
        s = HardwareState(encLMm=50.0, encRMm=50.0)
        assert evaluate('DISTANCE', 0.0, 0.0, 0.0, 0, 'GE', s, 0, base) is True


# ---------------------------------------------------------------------------
# Tests — HEADING
# ---------------------------------------------------------------------------

class TestHeading:
    """HEADING fires within eps of target heading delta."""

    def test_heading_fires_within_eps(self):
        """Heading at exactly the target delta fires HEADING(a=1.0, b=0.05)."""
        # baseline heading = 0; current heading = 1.0 rad → delta = 1.0
        base = MotionBaseline(heading0Rad=0.0)
        s    = HardwareState(poseHrad=1.0)
        # current_delta = wrap(1.0 - 0.0) = 1.0
        # error = wrap(1.0 - 1.0) = 0.0 < 0.05 → fires
        assert evaluate('HEADING', 1.0, 0.05, 0.0, 0, 'GE', s, 0, base) is True

    def test_heading_does_not_fire_just_outside_eps(self):
        """Heading 0.1 rad from target does not fire HEADING(b=0.05)."""
        base = MotionBaseline(heading0Rad=0.0)
        # current heading = 0.9, target delta = 1.0 → error = |wrap(-0.1)| = 0.1 > 0.05
        s = HardwareState(poseHrad=0.9)
        assert evaluate('HEADING', 1.0, 0.05, 0.0, 0, 'GE', s, 0, base) is False

    def test_heading_fires_at_pi_wrap(self):
        """Heading wrapping around ±π fires correctly."""
        # Target delta = π (half turn). Current heading = π → delta = π.
        base = MotionBaseline(heading0Rad=0.0)
        s    = HardwareState(poseHrad=math.pi)
        # wrap(pi - 0) = ±pi; wrap(±pi - pi) = 0.0 < 0.1 → should fire
        result = evaluate('HEADING', math.pi, 0.1, 0.0, 0, 'GE', s, 0, base)
        assert result is True

    def test_heading_negative_delta(self):
        """Negative heading delta (CW) fires correctly."""
        # Turn -π/2 CW. Baseline = 0, current = -π/2.
        base = MotionBaseline(heading0Rad=0.0)
        s    = HardwareState(poseHrad=-math.pi / 2)
        # current_delta = wrap(-pi/2) = -pi/2
        # error = wrap(-pi/2 - (-pi/2)) = 0.0 < 0.1 → fires
        assert evaluate('HEADING', -math.pi / 2, 0.1, 0.0, 0, 'GE', s, 0, base) is True


# ---------------------------------------------------------------------------
# Tests — POSITION
# ---------------------------------------------------------------------------

class TestPosition:
    """POSITION fires within radius; not just outside."""

    def test_position_fires_inside_radius(self):
        """Robot at (100, 100), target (100, 100), radius 10 → fires."""
        base = MotionBaseline()
        s    = HardwareState(poseX=100.0, poseY=100.0)
        # ax=100 (target X), a=100 (target Y), b=10 (radius)
        # dist = 0 < 10 → fires
        assert evaluate('POSITION', 100.0, 10.0, 100.0, 0, 'GE', s, 0, base) is True

    def test_position_does_not_fire_just_outside_radius(self):
        """Robot at (110.1, 100), target (100, 100), radius 10 → does not fire."""
        base = MotionBaseline()
        s    = HardwareState(poseX=110.1, poseY=100.0)
        # dx=10.1, dy=0 → dist2=102.01 >= 100 → does not fire
        assert evaluate('POSITION', 100.0, 10.0, 100.0, 0, 'GE', s, 0, base) is False

    def test_position_fires_at_radius_boundary(self):
        """Robot exactly at radius boundary (dist < radius) — fires."""
        base = MotionBaseline()
        # dist = 9.99 < 10 → fires
        s = HardwareState(poseX=109.99, poseY=100.0)
        assert evaluate('POSITION', 100.0, 10.0, 100.0, 0, 'GE', s, 0, base) is True

    def test_position_fires_for_any_direction(self):
        """Robot arrives from diagonal direction — fires when inside radius."""
        base = MotionBaseline()
        # Target (0,0), radius 15. Robot at (10, 10) → dist = ~14.14 < 15 → fires.
        s = HardwareState(poseX=10.0, poseY=10.0)
        assert evaluate('POSITION', 0.0, 15.0, 0.0, 0, 'GE', s, 0, base) is True

    def test_position_does_not_fire_far_away(self):
        """Robot far from target does not fire."""
        base = MotionBaseline()
        s    = HardwareState(poseX=0.0, poseY=0.0)
        # Target (500, 500), radius 10 → does not fire at (0,0)
        assert evaluate('POSITION', 500.0, 10.0, 500.0, 0, 'GE', s, 0, base) is False


# ---------------------------------------------------------------------------
# Tests — SENSOR
# ---------------------------------------------------------------------------

class TestSensor:
    """SENSOR GE fires when value >= threshold; LE fires when value <= threshold."""

    def test_sensor_ge_fires_at_threshold(self):
        """Line sensor channel 0 = 800, threshold 800, GE → fires."""
        s    = HardwareState(line=[800, 0, 0, 0])
        base = MotionBaseline()
        assert evaluate('SENSOR', 800.0, 0.0, 0.0, 0, 'GE', s, 0, base) is True

    def test_sensor_ge_does_not_fire_below_threshold(self):
        """Line sensor channel 0 = 799, threshold 800, GE → does not fire."""
        s    = HardwareState(line=[799, 0, 0, 0])
        base = MotionBaseline()
        assert evaluate('SENSOR', 800.0, 0.0, 0.0, 0, 'GE', s, 0, base) is False

    def test_sensor_le_fires_at_threshold(self):
        """Line sensor channel 2 = 100, threshold 100, LE → fires."""
        s    = HardwareState(line=[0, 0, 100, 0])
        base = MotionBaseline()
        assert evaluate('SENSOR', 100.0, 0.0, 0.0, 2, 'LE', s, 0, base) is True

    def test_sensor_le_does_not_fire_above_threshold(self):
        """Line sensor channel 2 = 101, threshold 100, LE → does not fire."""
        s    = HardwareState(line=[0, 0, 101, 0])
        base = MotionBaseline()
        assert evaluate('SENSOR', 100.0, 0.0, 0.0, 2, 'LE', s, 0, base) is False

    def test_sensor_color_channel(self):
        """Color channel 4 (colorR) = 512, threshold 500, GE → fires."""
        s    = HardwareState(colorR=512)
        base = MotionBaseline()
        assert evaluate('SENSOR', 500.0, 0.0, 0.0, 4, 'GE', s, 0, base) is True

    def test_sensor_analog_channel(self):
        """analogIn channel 8 = 300, threshold 200, GE → fires."""
        s    = HardwareState(analogIn=[300, 0, 0, 0])
        base = MotionBaseline()
        assert evaluate('SENSOR', 200.0, 0.0, 0.0, 8, 'GE', s, 0, base) is True

    def test_sensor_ge_fires_above_threshold(self):
        """GE fires when value > threshold too (not just ==)."""
        s    = HardwareState(line=[900, 0, 0, 0])
        base = MotionBaseline()
        assert evaluate('SENSOR', 800.0, 0.0, 0.0, 0, 'GE', s, 0, base) is True

    def test_sensor_le_fires_below_threshold(self):
        """LE fires when value < threshold too (not just ==)."""
        s    = HardwareState(line=[0, 0, 50, 0])
        base = MotionBaseline()
        assert evaluate('SENSOR', 100.0, 0.0, 0.0, 2, 'LE', s, 0, base) is True


# ---------------------------------------------------------------------------
# Tests — OR-across-array
# ---------------------------------------------------------------------------

class TestOrArray:
    """OR-across-array: two conditions; first fires; second not yet satisfied."""

    def test_first_fires_second_not(self):
        """Two conditions; TIME fires; DISTANCE not yet reached."""
        # TIME(500ms) fires at t=500; DISTANCE(1000mm) not yet reached.
        conds = [
            {'kind': 'TIME',     'a': 500.0},
            {'kind': 'DISTANCE', 'a': 1000.0},
        ]
        base = MotionBaseline(t0Ms=0, enc0Mm=0.0)
        # Only 100mm traveled, but time expired.
        s = HardwareState(encLMm=100.0, encRMm=100.0)
        idx = evaluate_array(conds, s, 500, base)
        assert idx == 0, f"Expected index 0 to fire, got {idx}"

    def test_second_fires_first_not(self):
        """Two conditions; DISTANCE fires; TIME not yet expired."""
        conds = [
            {'kind': 'TIME',     'a': 5000.0},    # 5 second timeout (not reached)
            {'kind': 'DISTANCE', 'a': 200.0},      # 200 mm
        ]
        base = MotionBaseline(t0Ms=0, enc0Mm=0.0)
        s = HardwareState(encLMm=200.0, encRMm=200.0)   # 200 mm traveled
        idx = evaluate_array(conds, s, 100, base)        # only 100 ms elapsed
        assert idx == 1, f"Expected index 1 to fire, got {idx}"

    def test_neither_fires(self):
        """Both conditions not satisfied: returns -1."""
        conds = [
            {'kind': 'TIME',     'a': 1000.0},
            {'kind': 'DISTANCE', 'a': 500.0},
        ]
        base = MotionBaseline(t0Ms=0, enc0Mm=0.0)
        s = HardwareState(encLMm=100.0, encRMm=100.0)
        idx = evaluate_array(conds, s, 500, base)
        assert idx == -1

    def test_both_fire_first_wins(self):
        """Both conditions satisfied: index 0 (first) wins."""
        conds = [
            {'kind': 'TIME',     'a': 100.0},
            {'kind': 'DISTANCE', 'a': 100.0},
        ]
        base = MotionBaseline(t0Ms=0, enc0Mm=0.0)
        s = HardwareState(encLMm=100.0, encRMm=100.0)
        idx = evaluate_array(conds, s, 1000, base)
        assert idx == 0, f"Expected index 0, got {idx}"

    def test_none_in_array_does_not_terminate(self):
        """Array with NONE conditions never fires."""
        conds = [
            {'kind': 'NONE'},
            {'kind': 'NONE'},
        ]
        base = MotionBaseline()
        s    = HardwareState()
        idx = evaluate_array(conds, s, 99999, base)
        assert idx == -1


# ---------------------------------------------------------------------------
# Tests — Zero conditions (no self-termination)
# ---------------------------------------------------------------------------

class TestZeroConditions:
    """A command with no stop conditions never self-terminates."""

    def test_empty_array_never_fires(self):
        """Empty condition array returns -1 on every tick."""
        base = MotionBaseline()
        s    = HardwareState(encLMm=10000.0, encRMm=10000.0)
        for now_ms in range(0, 100000, 1000):
            idx = evaluate_array([], s, now_ms, base)
            assert idx == -1, f"Empty array fired at now_ms={now_ms}"


# ---------------------------------------------------------------------------
# Tests — Baseline delta correctness
# ---------------------------------------------------------------------------

class TestBaselineDelta:
    """Stop conditions compute deltas correctly from captured baseline."""

    def test_distance_baseline_subtracted(self):
        """DISTANCE threshold relative to enc0Mm, not absolute enc_avg."""
        # baseline at enc0=500mm; need 200mm travel → enc_avg=700 fires
        base = MotionBaseline(enc0Mm=500.0)
        s_short = HardwareState(encLMm=699.0, encRMm=699.0)  # 199 mm, not enough
        s_exact = HardwareState(encLMm=700.0, encRMm=700.0)  # 200 mm, fires

        assert evaluate('DISTANCE', 200.0, 0.0, 0.0, 0, 'GE', s_short, 0, base) is False
        assert evaluate('DISTANCE', 200.0, 0.0, 0.0, 0, 'GE', s_exact, 0, base) is True

    def test_heading_baseline_subtracted(self):
        """HEADING is relative to heading0Rad baseline."""
        # baseline heading = π/4; target delta = π/4; current heading must be π/2
        base = MotionBaseline(heading0Rad=math.pi / 4)
        s_close  = HardwareState(poseHrad=math.pi / 2)       # delta = π/4, matches
        s_far    = HardwareState(poseHrad=math.pi / 4 + 0.5)  # delta = 0.5, far from π/4

        assert evaluate('HEADING', math.pi / 4, 0.05, 0.0, 0, 'GE', s_close, 0, base) is True
        assert evaluate('HEADING', math.pi / 4, 0.05, 0.0, 0, 'GE', s_far,   0, base) is False
