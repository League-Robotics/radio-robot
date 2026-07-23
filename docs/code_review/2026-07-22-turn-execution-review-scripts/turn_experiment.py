"""Headless A/B: 90-degree turn + TOUR_1 in the deterministic sim.

A: estimator/shaper push ON  (stop_lead_ms=45, alpha taper, jerk)
B: push OFF -- MoveQueue defaults (stopLead=0, shaping off).
Measured against SimPlant ground truth, mirroring test_tour_closure_gate.py.
"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import os
REPO = Path(os.environ.get("RRE_REPO", Path(__file__).resolve().parent.parent))  # run from repo, or set RRE_REPO
sys.path.insert(0, str(REPO / "src" / "host"))

from robot_radio.config.robot_config import load_robot_config
from robot_radio.io.sim_config import SimConfigConn
from robot_radio.io.sim_loop import SimLoop
from robot_radio.robot.protocol import NezhaProtocol

LIB = next((p for p in (REPO / "src/sim/build/libfirmware_host.dylib",
                        REPO / "src/sim/build/libfirmware_host.so",
                        Path("/tmp/simbuild/libfirmware_host.so")) if p.exists()), None)
CFG = REPO / "data" / "robots" / "tovez_nocal.json"

PUSH_KWARGS = dict(stop_lead_ms=45.0, a_max=800.0, a_decel=800.0,
                   alpha_max=7.0, alpha_decel=7.0, j_max=5000.0,
                   yaw_jerk_max=100.0)


def wait_ack(loop, corr_id, max_cycles=400):
    for _ in range(max_cycles):
        loop.step(1)
        for f in loop.drain_pending_tlm():
            if f.ack is not None and f.ack.corr_id == corr_id:
                return f.ack
    return None


def make_loop(push: bool) -> SimLoop:
    loop = SimLoop(track_width=128.0, lib_path=LIB)
    loop.connect(start_tick_thread=False)
    loop.configure_from_robot(load_robot_config(CFG))
    if push:
        proto = NezhaProtocol(SimConfigConn(loop))
        ack = wait_ack(loop, proto.estimator_config(**PUSH_KWARGS))
        assert ack is not None and ack.ok, f"estimator push failed: {ack}"
    loop.set_otos_raw_scale_err(0.0, 0.0)
    for p in (1, 2):
        loop.set_enc_scale_err(p, 0.0)
        loop.set_enc_tick_quant(p, 0.0)
        loop.set_enc_slip(p, 0.0, 0.0)
    return loop


def single_turn(push: bool, angle_deg: float = 90.0, omega: float = 2.0,
                trace: bool = False):
    loop = make_loop(push)
    try:
        loop.step(5)
        loop.drain_pending_tlm()
        h0 = loop.get_true_pose()["h"]
        loop.move(omega=omega, stop_angle=math.radians(angle_deg),
                  timeout=8000.0, replace=True, id=4242)
        done_cycle = None
        rows = []
        for i in range(400):
            loop.step(1)
            frames = loop.drain_pending_tlm()
            tp = loop.get_true_pose()
            last = frames[-1] if frames else None
            if done_cycle is None:
                for f in frames:
                    if f.ack is not None and f.ack.corr_id == 4242:
                        done_cycle = i
            if trace:
                pose_h = last.pose[2] / 100.0 if (last and last.pose) else None
                tw = last.twist if last else None
                rows.append((i * 0.05, math.degrees(tp["h"] - h0), pose_h,
                             tw, done_cycle == i))
            if done_cycle is not None and i >= done_cycle + 30:
                break
        hf = loop.get_true_pose()["h"]
        achieved = math.degrees(hf - h0)
        if trace:
            print("    t[s]  truth[deg]  tlm_pose_h[deg]  twist")
            for r in rows:
                mark = " <== completion ack" if r[4] else ""
                ph = f"{r[2]:8.2f}" if r[2] is not None else "       -"
                print(f"    {r[0]:5.2f}  {r[1]:8.2f}  {ph}  {r[3]}{mark}")
        done_t = None if done_cycle is None else round(done_cycle * 0.05, 3)
        print(f"  single {angle_deg:.0f}deg turn, push={'ON ' if push else 'OFF'}: "
              f"achieved={achieved:+8.2f}deg  error={achieved - angle_deg:+7.2f}deg  "
              f"completion_ack_at={done_t}s")
        return achieved - angle_deg
    finally:
        loop.disconnect()


def tour(push: bool):
    from robot_radio.planner.heading import HeadingCorrector
    from robot_radio.planner.model import PlannerParams
    from robot_radio.planner.tour import TOUR_1, parse_tour, run_tour

    loop = make_loop(push)
    try:
        legs = parse_tour(TOUR_1)
        params = PlannerParams()
        heading = HeadingCorrector(params, robot_config=SimpleNamespace(
            geometry=SimpleNamespace(otos_untrusted=True)))
        clock = SimpleNamespace(now_s=0.0)

        def clock_fn():
            return clock.now_s

        def sleep_fn(_dt):
            loop.step(1)
            loop._drain_tlm_into_queue()
            clock.now_s += 0.05

        true_poses = [loop.get_true_pose()]
        turns = []

        def on_leg(index, total, leg, leg_result):
            pose = loop.get_true_pose()
            if leg.kind == "turn":
                before = math.degrees(true_poses[-1]["h"])
                after = math.degrees(pose["h"])
                d = after - before
                while d > 180.0:
                    d -= 360.0
                while d <= -180.0:
                    d += 360.0
                turns.append((index, leg.value, d, d - leg.value))
            true_poses.append(pose)

        result = run_tour(loop, params, heading, legs, v_max=150.0,
                          on_leg=on_leg, clock_fn=clock_fn, sleep_fn=sleep_fn,
                          poll_interval=0.05)
        ok = result.stopped_at is None
        print(f"  TOUR_1 push={'ON ' if push else 'OFF'}: completed={ok}"
              + ("" if ok else f" (stopped at leg {result.stopped_at}:"
                               f" {result.stopped_outcome})"))
        worst = 0.0
        total_err = 0.0
        for idx, cmd, ach, err in turns:
            print(f"    turn leg {idx + 1:2d}: commanded={cmd:+7.1f}  "
                  f"achieved={ach:+8.2f}  error={err:+7.2f} deg")
            worst = max(worst, abs(err))
            total_err += err
        n = max(1, len(turns))
        print(f"    worst |err|={worst:.2f}deg  mean err={total_err / n:+.2f}deg")
    finally:
        loop.disconnect()


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("all", "single"):
        print("== SINGLE 90deg TURN ==")
        single_turn(push=False)
        single_turn(push=True)
    if what == "trace":
        print("== SINGLE 90deg TURN, per-cycle trace, push=OFF ==")
        single_turn(push=False, trace=True)
        print("== SINGLE 90deg TURN, per-cycle trace, push=ON ==")
        single_turn(push=True, trace=True)
    if what in ("all", "tour"):
        print("== TOUR_1 ==")
        tour(push=False)
        tour(push=True)
