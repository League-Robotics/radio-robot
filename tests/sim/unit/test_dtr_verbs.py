"""Post-094 (out-of-process): D / T / RT re-parsed into a single
`Motion::Segment` each and posted to `bb.segmentIn`, exactly like `MOVE`
(`source/commands/motion_commands.cpp` `handleD`/`handleT`/`handleRT`).

  - D <l> <r> <mm>  -> straight segment of `mm`, signed by the drive dir.
  - T <l> <r> <ms>  -> straight segment of distance = v * (ms/1000).
  - RT <relAngle>   -> pure in-place turn segment (finalHeading = relAngle).

Motion correctness only (per stakeholder): the segment executes with a
Ruckig profile, genuinely drives the plant, and settles to zero with no
reverse-creep. Distance/turn *accuracy* is explicitly NOT asserted here.
"""
from __future__ import annotations

import pytest


def _drive_to_settle(sim, seconds=6.0, step=24):
    """Tick `seconds` in `step`-ms increments; track per-wheel peak |v| and
    enforce no-reverse-creep (a wheel, once substantially moving, never flips
    past a small settle-noise floor). Returns (max|vL|, max|vR|)."""
    sign_l = sign_r = 0
    max_l = max_r = 0.0
    for _ in range(int(seconds * 1000 / step)):
        sim.tick_for(step)
        vl, vr = sim.vel()
        max_l, max_r = max(max_l, abs(vl)), max(max_r, abs(vr))
        if sign_l == 0 and abs(vl) > 20.0:
            sign_l = 1 if vl > 0 else -1
        if sign_r == 0 and abs(vr) > 20.0:
            sign_r = 1 if vr > 0 else -1
        if sign_l == 1:
            assert vl > -15.0, f"left reverse-crept: {vl}"
        elif sign_l == -1:
            assert vl < 15.0, f"left reverse-crept: {vl}"
        if sign_r == 1:
            assert vr > -15.0, f"right reverse-crept: {vr}"
        elif sign_r == -1:
            assert vr < 15.0, f"right reverse-crept: {vr}"
    return max_l, max_r


def test_d_straight_forward_drives_and_settles(sim):
    assert sim.command("D 200 200 300").strip() == "OK drive l=200 r=200 mm=300"
    max_l, max_r = _drive_to_settle(sim)
    assert max_l > 50.0 and max_r > 50.0, "D segment never genuinely drove"
    vl, vr = sim.vel()
    assert vl == pytest.approx(0.0, abs=10.0) and vr == pytest.approx(0.0, abs=10.0)


def test_d_reverse_drives_backward(sim):
    assert sim.command("D -200 -200 300").strip() == "OK drive l=-200 r=-200 mm=300"
    # Both wheels should move NEGATIVE (backward) at their peak.
    min_l = min_r = 0.0
    for _ in range(150):
        sim.tick_for(24)
        vl, vr = sim.vel()
        min_l, min_r = min(min_l, vl), min(min_r, vr)
    assert min_l < -50.0 and min_r < -50.0, "D with negative wheels did not drive backward"


def test_t_timed_drives_straight_and_settles(sim):
    assert sim.command("T 200 200 1000").strip() == "OK drive l=200 r=200 ms=1000"
    max_l, max_r = _drive_to_settle(sim)
    assert max_l > 50.0 and max_r > 50.0, "T segment never genuinely drove"
    vl, vr = sim.vel()
    assert vl == pytest.approx(0.0, abs=10.0) and vr == pytest.approx(0.0, abs=10.0)


def test_rt_turns_in_place_wheels_counter_rotate_and_settle(sim):
    assert sim.command("RT 9000").strip() == "OK rt rot=9000"
    # An in-place turn drives both wheels in OPPOSITE directions at peak.
    peak_l = peak_r = 0.0
    for _ in range(200):
        sim.tick_for(24)
        vl, vr = sim.vel()
        if abs(vl) > abs(peak_l):
            peak_l = vl
        if abs(vr) > abs(peak_r):
            peak_r = vr
    assert abs(peak_l) > 20.0 and abs(peak_r) > 20.0, "RT never spun the wheels"
    assert (peak_l > 0) != (peak_r > 0), "RT wheels did not counter-rotate (not an in-place turn)"
    vl, vr = sim.vel()
    assert vl == pytest.approx(0.0, abs=10.0) and vr == pytest.approx(0.0, abs=10.0)


def test_d_then_rt_string_together(sim):
    """A D straight immediately followed by an RT turn: both execute in order
    off bb.segmentIn's queue (the stringing the stakeholder wants to see)."""
    assert sim.command("D 200 200 250").strip() == "OK drive l=200 r=200 mm=250"
    assert sim.command("RT 9000").strip() == "OK rt rot=9000"
    saw_straight = saw_turn = False
    for _ in range(400):   # up to ~9.6s for both segments
        sim.tick_for(24)
        vl, vr = sim.vel()
        if vl > 40.0 and vr > 40.0:
            saw_straight = True            # both same-sign forward = TRANSLATE
        if abs(vl) > 20.0 and abs(vr) > 20.0 and (vl > 0) != (vr > 0):
            saw_turn = True                # opposite-sign = the RT pivot
    assert saw_straight, "never saw the D straight phase"
    assert saw_turn, "never saw the RT turn phase (segments did not string)"
    vl, vr = sim.vel()
    assert vl == pytest.approx(0.0, abs=10.0) and vr == pytest.approx(0.0, abs=10.0)
