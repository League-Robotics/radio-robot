"""src/tests/sim/test_motor_primitive.py -- most-primitive open-loop motor check.

Bypasses the Move/Ruckig trajectory planner entirely: uses ONLY the direct
`twist` (turn the motors on at a velocity) and `stop` (turn them off) wire
primitives -- `RobotLoop::handleTwist` calls `Drive::setTwist()` directly and
flushes the Motion::Executor queue, so no trajectory planning is involved.

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

from robot_radio.io.sim_loop import SimLoop
from robot_radio.testgui.transport import _sim_lib_path

TRACK_WIDTH = 128.0    # [mm]
TICKS_PER_MM = 1.4187  # tovez wheels.ticks_per_mm
DEADMAN_MS = 300.0     # twist lease, re-armed every cycle to keep motors on


def ideal_loop() -> SimLoop:
    """A SimLoop with EVERY simulated error source explicitly zeroed."""
    loop = SimLoop(track_width=TRACK_WIDTH, lib_path=_sim_lib_path())
    loop.connect(start_tick_thread=False)
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

def test_distance_encoder_and_otos_match_truth():
    d = distance_probe(150.0, 2.0)
    assert abs(d["enc"] - d["true_x"]) < 2.0, d
    assert abs(d["otos_x"] - d["true_x"]) < 2.0, d


def test_heading_encoder_and_otos_match_truth():
    h = heading_probe(1.0, 2.0)
    assert abs(h["pose_h"] - h["true_h"]) < 1.0, h
    assert abs(h["otos_h"] - h["true_h"]) < 1.0, h


if __name__ == "__main__":
    main()
