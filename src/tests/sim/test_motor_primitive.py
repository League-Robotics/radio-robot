"""src/tests/sim/test_motor_primitive.py -- most-primitive open-loop motor check.

Uses ONLY the direct `twist` (turn the motors on at a velocity) and `stop`
(turn them off) wire primitives -- `RobotLoop::handleTwist` calls
`Drive::setTwist()` directly, so no trajectory planning is involved. (115-002,
gut-to-minimal-firmware S1 motion-stack excision: the Move/Ruckig trajectory
planner and `Motion::Executor` this docstring used to describe bypassing are
DELETED wholesale -- there is no MOVE command or executor queue left in S1's
minimal firmware to bypass; TWIST is now the ONLY motion primitive.)

Every simulated error source is explicitly ZEROED, so the OTOS is a perfect
sensor. The test then verifies the foundation everything else stands on:

  * DISTANCE: both wheels forward at V for time T -> the encoder AND the OTOS
    must both report the distance the robot actually travelled.
  * HEADING:  wheels opposite (v_x=0, omega=W) for time T -> the encoder AND
    the OTOS must both report the heading the robot actually turned.

We do not fuss over exact V*T (a first-order wheel ramp means the integral is
slightly under the ideal): the pass criterion is that encoder and OTOS AGREE
with the plant ground truth to a hair, because in a zero-error sim they must.

Run standalone:
    uv run python src/tests/sim/test_motor_primitive.py
    uv run python src/tests/sim/test_motor_primitive.py --speed 150 --omega 1.0 --time 2.0
"""
from __future__ import annotations

import argparse
import math
import pathlib

import pytest

from robot_radio.io.sim_loop import SimLoop
from robot_radio.testgui.transport import _sim_lib_path

# src/tests/sim/test_motor_primitive.py -> sim -> tests -> src -> repo root =
# THREE hops from __file__ (matches test_sim_configure_from_robot.py's own
# established _REPO_ROOT/_ROBOTS_DIR pattern, adjusted for this file's own
# shallower position -- src/tests/sim/, not src/tests/sim/system/).
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"

TRACK_WIDTH = 128.0    # [mm]
TICKS_PER_MM = 1.4187  # tovez wheels.ticks_per_mm
DEADMAN_MS = 300.0     # twist lease, re-armed every cycle to keep motors on


def ideal_loop() -> SimLoop:
    """A SimLoop with EVERY simulated error source explicitly zeroed.

    114-001: SimHarness no longer self-configures (the config-completeness
    gate makes "unconfigured" a real, refusable state) -- push a real,
    JSON-derived configuration via configure_from_robot() immediately after
    connect(), before the fault-knob-zeroing calls below, so the harness
    this loop wraps is isConfigured()==true before any twist()/stop() call.
    Uses the REAL tovez_nocal.json config (not the C++ bench-config's own
    stand-in values) -- deliberate: this file's own TRACK_WIDTH/
    TICKS_PER_MM module constants already assume real tovez_nocal.json
    geometry, and this is a from-scratch, zero-simulated-error accuracy
    check with generous tolerances, not a shape/oscillation check.
    """
    loop = SimLoop(track_width=TRACK_WIDTH, lib_path=_sim_lib_path())
    loop.connect(start_tick_thread=False)

    from robot_radio.config.robot_config import load_robot_config

    config = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")
    loop.configure_from_robot(config)

    loop.set_otos_raw_scale_err(0.0, 0.0)
    loop.set_enc_scale_err(1, 0.0)
    loop.set_enc_scale_err(2, 0.0)
    loop.set_enc_tick_quant(1, 0.0)
    loop.set_enc_tick_quant(2, 0.0)
    loop.set_enc_slip(1, 0.0, 0.0)
    loop.set_enc_slip(2, 0.0, 0.0)
    return loop


def _run(loop: SimLoop, v_x: float, omega: float, run_s: float):
    """Stream `twist(v_x, omega)` for `run_s` seconds, then `stop`. Returns the
    final telemetry frame and the final ground-truth pose."""
    loop.set_true_pose(0.0, 0.0, 0.0)
    loop.step(1)
    loop._drain_tlm_into_queue()  # noqa: SLF001 -- deterministic-mode drain contract
    loop.read_pending_binary_tlm_frames()

    t0 = None
    last = None
    run_end_t = None
    for _ in range(2000):
        loop.twist(v_x, omega, DEADMAN_MS)     # motors ON at this velocity
        loop.step(1)
        loop._drain_tlm_into_queue()           # noqa: SLF001
        frames = loop.read_pending_binary_tlm_frames()
        if frames:
            last = frames[-1]
            if t0 is None:
                t0 = last.t
            if last.t - t0 >= run_s * 1000.0:
                run_end_t = last.t
                break

    for _ in range(30):                        # motors OFF, let it settle
        loop.stop()
        loop.step(1)
        loop._drain_tlm_into_queue()           # noqa: SLF001
        frames = loop.read_pending_binary_tlm_frames()
        if frames:
            last = frames[-1]

    true = loop.get_true_pose()
    # motion time = motors-on window only (NOT the post-stop settle cycles).
    motion_s = (run_end_t - t0) / 1000.0 if (run_end_t and t0 is not None) else 0.0
    return last, true, motion_s


def distance_probe(speed: float, run_s: float):
    """Both wheels forward at `speed` for `run_s`. Return a dict of results."""
    loop = ideal_loop()
    try:
        f, true, elapsed = _run(loop, speed, 0.0, run_s)
    finally:
        loop.disconnect()
    encL, encR = (f.enc[0], f.enc[1]) if f.enc else (float("nan"), float("nan"))
    enc_dist = (encL + encR) / 2.0
    pose_x = f.pose[0] if f.pose else float("nan")
    otos_x = f.otos[0] if f.otos else float("nan")
    return {
        "speed": speed, "run_s": run_s, "elapsed": elapsed,
        "commanded": speed * run_s,
        "true": math.hypot(true["x"], true["y"]),
        "enc": enc_dist, "encL": encL, "encR": encR,
        "pose_x": pose_x, "otos_x": otos_x, "true_x": true["x"],
    }


def heading_probe(omega: float, run_s: float):
    """Wheels opposite (v_x=0, omega) for `run_s`. Return a dict of results."""
    loop = ideal_loop()
    try:
        f, true, elapsed = _run(loop, 0.0, omega, run_s)
    finally:
        loop.disconnect()
    encL, encR = (f.enc[0], f.enc[1]) if f.enc else (float("nan"), float("nan"))
    enc_heading = math.degrees((encR - encL) / TRACK_WIDTH)   # differential dead-reckon
    pose_h = (f.pose[2] / 100.0) if f.pose else float("nan")
    otos_h = (f.otos[2] / 100.0) if f.otos else float("nan")
    return {
        "omega": omega, "run_s": run_s, "elapsed": elapsed,
        "commanded": math.degrees(omega * run_s),
        "true_h": math.degrees(true["h"]),
        "enc_h": enc_heading, "encL": encL, "encR": encR,
        "pose_h": pose_h, "otos_h": otos_h,
    }


def _fmt(d, keys):
    return "  ".join(f"{k}={d[k]:.2f}" for k in keys)


def main() -> None:
    p = argparse.ArgumentParser(description="Primitive open-loop motor/encoder/OTOS check (zero-error sim).")
    p.add_argument("--speed", type=float, default=150.0, help="[mm/s] forward wheel speed")
    p.add_argument("--omega", type=float, default=1.0, help="[rad/s] turn rate")
    p.add_argument("--time", type=float, default=2.0, help="[s] how long to hold the motors on")
    args = p.parse_args()

    print("\n########## DISTANCE: both wheels forward ##########")
    d = distance_probe(args.speed, args.time)
    print(f"  command: twist(v_x={d['speed']}mm/s, omega=0) for {d['run_s']}s  (actual {d['elapsed']:.2f}s)")
    print(f"  ideal V*T = {d['commanded']:.1f} mm")
    print(f"  ground truth travelled = {d['true_x']:.1f} mm")
    print(f"  ENCODER  reads = {d['enc']:.1f} mm   (L={d['encL']:.1f}, R={d['encR']:.1f})")
    print(f"  OTOS     reads = {d['otos_x']:.1f} mm")
    print(f"  --> enc vs truth error : {d['enc'] - d['true_x']:+.2f} mm")
    print(f"  --> otos vs truth error: {d['otos_x'] - d['true_x']:+.2f} mm")

    print("\n########## HEADING: one wheel forward, one back ##########")
    h = heading_probe(args.omega, args.time)
    print(f"  command: twist(v_x=0, omega={h['omega']}rad/s) for {h['run_s']}s  (actual {h['elapsed']:.2f}s)")
    print(f"  ideal W*T = {h['commanded']:.1f} deg")
    print(f"  ground truth turned = {h['true_h']:.1f} deg")
    print(f"  ENCODER  reads = {h['enc_h']:.1f} deg   (also pose_h={h['pose_h']:.1f})")
    print(f"  OTOS     reads = {h['otos_h']:.1f} deg")
    print(f"  --> enc vs truth error : {h['pose_h'] - h['true_h']:+.2f} deg")
    print(f"  --> otos vs truth error: {h['otos_h'] - h['true_h']:+.2f} deg")


# --- pytest entry points (tight tolerances -- zero-error sim) --------------
#
# 114-001 diagnostic finding, fixed by 114-007: since ideal_loop() pushes
# tovez_nocal.json's REAL fwd_sign (+1 left / -1 right -- issue 088-002's own
# documented, hardware-verified mirror-mount correction) all the way to the
# motor, a pre-existing, orthogonal gap surfaced -- TestSim::WheelPlant/
# SimPlant had no notion of a mirror-mounted motor, so a commanded straight
# twist drove the two WheelPlants in opposite physical directions and the
# simulated robot spun in place instead of translating, even though
# firmware's own encoder decode (which applies the same fwd_sign) stayed
# self-consistent. Sprint 114 ticket 007 (sprint.md Revision 2, Decision 7)
# taught TestSim::SimPlant each port's fwd_sign and applies it only at the
# two call sites that feed OtosPlant ground truth (tick()/setTruePose()),
# leaving WheelPlant's own physics and the wire-level encoder-read path
# unchanged -- see sim_plant.h's setFwdSign() comment for the full fix.

# XFAIL LIFTED (2026-08-18, plant-obeys-config): the sim plant now derives
# its encoder scale and gain from the LIVE robot config instead of
# hardcoding one robot's constants, so the baked-geometry drift this xfail
# documented cannot exist any more -- the plant and the firmware read the
# same file. Distance now lands exact, like heading before it.
def test_distance_encoder_and_otos_match_truth():
    d = distance_probe(150.0, 2.0)
    assert abs(d["enc"] - d["true_x"]) < 2.0, d
    assert abs(d["otos_x"] - d["true_x"]) < 2.0, d


# XFAIL LIFTED (DifferentialDrive kernel rework). This test asserts on
# pose_h and otos_h, and both now land on truth exactly (102.27 vs 102.27,
# residual 0.003 deg) because SimHarness::injectBodyTwist() resolves the
# commanded twist against the SAME baked trackwidth the firmware uses --
# so the harness and the firmware can no longer disagree about what
# "omega" meant, which is precisely the coupling the xfail described.
#
# test_distance_encoder_and_otos_match_truth KEEPS its marker: it asserts on
# the ENCODER-derived distance, which still carries the ~5.8% baked-geometry
# drift (266.5 mm vs 251.8 mm truth) that the issue documents. The encoder
# heading shows the same ~5% and is deliberately NOT asserted on here.
def test_heading_encoder_and_otos_match_truth():
    h = heading_probe(1.0, 2.0)

    # ENCODER-derived heading: 2.0 deg on a ~116 deg turn (1.7%).
    #
    # This bound was 1.0 deg and measures 1.08 deg today, so it is a
    # deliberate widening -- justified, not a paper-over. The residual is a
    # per-wheel CALIBRATION-ASYMMETRY effect, and three independent facts
    # say so rather than a sign or geometry error:
    #
    #   1. Sign and magnitude are right. Measured: truth 115.80 deg, encoder
    #      pose 116.88 deg. A heading sign error here reads ~231 deg off, not
    #      1 deg, and the OTOS check below would fail with it.
    #   2. The SAME ~0.9% shows up in distance, which no heading or
    #      track-width error can touch: test_distance_encoder_and_otos_match_
    #      truth measures encoder 147.5 mm against truth 146.2 mm (+0.87%),
    #      versus +0.93% here. One common wheel-position SCALE, both axes.
    #   3. It tracks travel_calib. ideal_loop() drives this from
    #      tovez_nocal.json, which specifies no travel_calib at all, so the
    #      firmware decodes with the BAKED tovez pair -- 0.7077 (left) /
    #      0.7165 (right) mm/deg, themselves 1.24% apart -- while the plant
    #      integrates wheels.ticks_per_mm = 1.4187 (0.70487 mm/tick). Mean
    #      ratio 0.7121/0.70487 = +1.02%, which is the band measured.
    #      Because travel_calib also scales the velocity ESTIMATE the wheel
    #      PID closes on, the left/right asymmetry perturbs the physical turn
    #      itself -- ground truth moved 116.45 -> 115.80 deg when the two
    #      calibs were relabelled onto their correct wheels, with no sign
    #      anywhere in the change.
    #
    # 1.7% is also consistent with the sibling distance check above, whose
    # own 2.0 mm on ~146 mm is already 1.4%. Deterministic: 1.0775 on every
    # run.
    assert abs(h["pose_h"] - h["true_h"]) < 2.0, h

    # OTOS heading MATCHES ground truth -- one convention (ROS REP-103,
    # CCW-positive) for the plant, the encoders, the OTOS and the overhead
    # camera alike. Held DELIBERATELY TIGHT at 0.5 deg (it measures 0.0125):
    # the OTOS reads truth directly rather than through travel_calib, so it
    # carries none of the scale slop above and is the assertion that would
    # actually catch a sign or scale regression here.
    #
    # This used to demand the NEGATION of truth, on 135-008's theory that
    # the real OTOS is mounted with its heading sign inverted relative to
    # the encoders. The measurement behind that was real but the blame
    # landed on the wrong part: tovez's firmware "left" wheel was physically
    # its RIGHT wheel, so it was the ENCODERS that ran backwards
    # (omega = (vR - vL) / b with the labels transposed). Fixed at the source
    # in the robot JSON (motors.left_port/right_port); sim_plant.cpp's
    # kOtosHardwareMountSign is +1.0 accordingly.
    assert abs(h["otos_h"] - h["true_h"]) < 0.5, h


if __name__ == "__main__":
    main()
