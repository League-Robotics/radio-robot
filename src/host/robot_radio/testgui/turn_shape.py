"""turn_shape.py -- diagnose and validate the wheel-speed SHAPE of a single turn.

The stakeholder's spec for a good turn (2026-07-17): a pivot's wheel-speed
trace must look like a single trapezoid per wheel -- ramp up to a cruise
speed, hold, ramp back down to zero -- and NOTHING else. Two hard properties
follow, and this tool checks them mathematically against the sim (zero-error
by default, so any violation is a systematic control defect, not noise):

  1. NO REVERSALS. A pivot wheel spins one direction the whole turn; its
     speed must never cross zero. Every zero-crossing is a churn event.
  2. INTEGRAL == ANGLE. Integrating body angular rate (vR - vL)/trackwidth
     over time must equal the commanded turn angle. Reversals inflate the
     |path| integral above the net -- the gap measures wasted back-and-forth.

It captures BOTH the commanded body twist (what the firmware asked for) and
the actual wheel speeds (what the plant did), so a diagnosis can say whether
the churn is commanded by the control law or produced by the plant.

Usage:
    uv run python -m robot_radio.testgui.turn_shape --angle 360
    uv run python -m robot_radio.testgui.turn_shape --angle 360 --rate 3.0 --csv /tmp/turn.csv
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

from robot_radio.io.sim_loop import SimLoop
from robot_radio.testgui.transport import _sim_lib_path

TRACK_WIDTH = 128.0  # [mm] tovez trackwidth


@dataclass
class Sample:
    t: float          # [ms] robot clock
    vL: float         # [mm/s] actual left wheel speed
    vR: float         # [mm/s] actual right wheel speed
    cmd_omega: float  # [rad/s] COMMANDED body yaw rate (firmware twist)
    pose_h: float     # [deg] encoder dead-reckoned heading
    otos_h: float     # [deg] raw OTOS heading
    true_h: float     # [deg] ground-truth heading


@dataclass
class TurnCapture:
    angle_deg: float
    samples: list[Sample] = field(default_factory=list)
    started: bool = False
    settled: bool = False
    notes: str = ""


def _frame_to_sample(f, true_h: float) -> Sample:
    vL = f.vel[0] if f.vel else 0.0
    vR = f.vel[1] if f.vel else 0.0
    omega = (f.twist[1] / 1000.0) if f.twist else 0.0  # mrad/s -> rad/s
    pose_h = (f.pose[2] / 100.0) if f.pose else float("nan")
    otos_h = (f.otos[2] / 100.0) if f.otos else float("nan")
    return Sample(f.t, vL, vR, omega, pose_h, otos_h, true_h)


def capture_turn_live(angle_deg: float, *,
                      realistic: bool = False, speed_factor: int = 4,
                      timeout_s: float = 30.0) -> TurnCapture:
    """GUI-FAITHFUL capture: real tick thread (like TestGUI Sim mode), frames
    collected via the on_telemetry callback exactly as the GUI receives them.
    This exercises the real-time scheduling the deterministic harness removes."""
    import threading
    import time

    loop = SimLoop(track_width=TRACK_WIDTH, lib_path=_sim_lib_path())
    cap = TurnCapture(angle_deg=angle_deg)
    lock = threading.Lock()

    def on_tlm(f):
        # tick-thread callback -- we are already ON the tick thread, so read
        # ground truth DIRECTLY (loop._read_true_pose, no re-scheduling round
        # trip -- get_true_pose() would deadlock by re-scheduling onto us).
        try:
            th = math.degrees(loop._read_true_pose()["h"])  # noqa: SLF001
        except Exception:
            th = float("nan")
        with lock:
            cap.samples.append(_frame_to_sample(f, th))

    loop.on_telemetry = on_tlm
    loop.connect(start_tick_thread=True)
    try:
        if not realistic:
            loop.set_otos_raw_scale_err(0.0, 0.0)
            loop.set_enc_scale_err(1, 0.0)
            loop.set_enc_scale_err(2, 0.0)
            loop.set_enc_tick_quant(1, 0.0)
            loop.set_enc_tick_quant(2, 0.0)
            loop.set_enc_slip(1, 0.0, 0.0)
            loop.set_enc_slip(2, 0.0, 0.0)
        loop.set_speed_factor(speed_factor)
        loop.set_true_pose(0.0, 0.0, 0.0)
        time.sleep(0.2)
        with lock:
            cap.samples.clear()
        loop.move(delta_heading=math.radians(angle_deg))

        deadline = time.monotonic() + timeout_s
        idle = 0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            with lock:
                if not cap.samples:
                    continue
                last = cap.samples[-1]
                moving = abs(last.vL) > 15.0 or abs(last.vR) > 15.0
                if moving:
                    cap.started = True
                    idle = 0
                elif cap.started:
                    idle += 1
                    if idle > 8:
                        cap.settled = True
                        break
        return cap
    finally:
        loop.disconnect()


def capture_turn_gui(angle_deg: float, *,
                     timeout_s: float = 30.0) -> TurnCapture:
    """MOST GUI-FAITHFUL capture: drives through the real ``SimTransport`` and
    pushes the active robot's calibration on connect exactly as TestGUI's
    ``_push_robot_calibration()`` does -- crucially the ``OI`` OTOS-init that
    ACTIVATES the OTOS, flipping the firmware's AUTO heading policy onto it."""
    import threading
    import time

    from robot_radio.calibration.push import calibration_commands
    from robot_radio.config.robot_config import get_robot_config
    from robot_radio.testgui.transport import SimTransport

    cap = TurnCapture(angle_deg=angle_deg)
    lock = threading.Lock()
    t = SimTransport()

    def on_tlm(f):
        loop = t.protocol
        try:
            th = math.degrees(loop._read_true_pose()["h"]) if loop else float("nan")  # noqa: SLF001
        except Exception:
            th = float("nan")
        with lock:
            cap.samples.append(_frame_to_sample(f, th))

    t.on_telemetry = on_tlm
    t.connect()
    try:
        cfg = get_robot_config()
        robot = getattr(cfg, "robot_name", "?")
        pushed = []
        for cmd, rt in calibration_commands(cfg):
            reply = t.command(cmd, read_timeout=rt)
            pushed.append((cmd, (reply or "").strip()))
        cap.notes = f"robot={robot}; pushed: " + "; ".join(f"{c}->{r}" for c, r in pushed)

        loop = t.protocol
        loop.set_true_pose(0.0, 0.0, 0.0)
        time.sleep(0.3)
        with lock:
            cap.samples.clear()
        loop.move(delta_heading=math.radians(angle_deg))

        deadline = time.monotonic() + timeout_s
        idle = 0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            with lock:
                if not cap.samples:
                    continue
                last = cap.samples[-1]
                moving = abs(last.vL) > 15.0 or abs(last.vR) > 15.0
                if moving:
                    cap.started = True
                    idle = 0
                elif cap.started:
                    idle += 1
                    if idle > 8:
                        cap.settled = True
                        break
        return cap
    finally:
        t.disconnect()


def capture_turn(angle_deg: float, *,
                 realistic: bool = False, max_cycles: int = 1000) -> TurnCapture:
    """Command ONE in-place turn of `angle_deg` and record the full per-cycle
    trace, deterministically stepped (no tick thread -> no scheduling jitter).
    Ideal chip by default (every error knob explicitly zeroed)."""
    loop = SimLoop(track_width=TRACK_WIDTH, lib_path=_sim_lib_path())
    loop.connect(start_tick_thread=False)
    try:
        if not realistic:
            loop.set_otos_raw_scale_err(0.0, 0.0)
            loop.set_enc_scale_err(1, 0.0)
            loop.set_enc_scale_err(2, 0.0)
            loop.set_enc_tick_quant(1, 0.0)
            loop.set_enc_tick_quant(2, 0.0)
            loop.set_enc_slip(1, 0.0, 0.0)
            loop.set_enc_slip(2, 0.0, 0.0)

        loop.set_true_pose(0.0, 0.0, 0.0)
        # flush boot frames
        loop.step(1)
        loop._drain_tlm_into_queue()  # noqa: SLF001 -- deterministic-mode drain contract
        loop.read_pending_binary_tlm_frames()

        loop.move(delta_heading=math.radians(angle_deg))

        cap = TurnCapture(angle_deg=angle_deg)
        idle_after_motion = 0
        for _ in range(max_cycles):
            loop.step(1)
            loop._drain_tlm_into_queue()  # noqa: SLF001
            frames = loop.read_pending_binary_tlm_frames()
            if not frames:
                continue
            f = frames[-1]
            vL = f.vel[0] if f.vel else 0.0
            vR = f.vel[1] if f.vel else 0.0
            omega = (f.twist[1] / 1000.0) if f.twist else 0.0  # mrad/s -> rad/s
            pose_h = (f.pose[2] / 100.0) if f.pose else float("nan")
            otos_h = (f.otos[2] / 100.0) if f.otos else float("nan")
            true_h = math.degrees(loop.get_true_pose()["h"])
            cap.samples.append(Sample(f.t, vL, vR, omega, pose_h, otos_h, true_h))

            moving = abs(vL) > 15.0 or abs(vR) > 15.0
            if moving:
                cap.started = True
                idle_after_motion = 0
            elif cap.started:
                idle_after_motion += 1
                if idle_after_motion > 25:
                    cap.settled = True
                    break
        return cap
    finally:
        loop.disconnect()


def zero_crossings(series: list[float], *, deadband: float = 5.0) -> list[int]:
    """Indices where `series` crosses zero (sign flip), ignoring values inside
    +/-deadband so sensor quantization near zero isn't counted as a reversal."""
    idx = []
    last_sign = 0
    for i, v in enumerate(series):
        if abs(v) < deadband:
            continue
        sign = 1 if v > 0 else -1
        if last_sign != 0 and sign != last_sign:
            idx.append(i)
        last_sign = sign
    return idx


def integrate(series: list[float], ts: list[float]) -> float:
    """Trapezoidal integral of `series` (per-second units) over ts [ms]."""
    return sum(
        0.5 * (series[i] + series[i - 1]) * (ts[i] - ts[i - 1]) / 1000.0
        for i in range(1, len(series))
    )


@dataclass
class Analysis:
    cycles: int
    duration_s: float
    reversals_vL: list[int]
    reversals_vR: list[int]
    reversals_cmd: list[int]
    net_deg: float
    path_deg: float
    cmd_net_deg: float
    true_final_deg: float
    churn_onset: int | None


def analyze(cap: TurnCapture) -> Analysis:
    s = cap.samples
    ts = [x.t for x in s]
    vL = [x.vL for x in s]
    vR = [x.vR for x in s]
    cmd = [x.cmd_omega for x in s]
    body_omega = [(x.vR - x.vL) / TRACK_WIDTH for x in s]  # [rad/s]
    rL = zero_crossings(vL)
    rR = zero_crossings(vR)
    rC = zero_crossings([w * TRACK_WIDTH / 2 for w in cmd], deadband=5.0)
    onset = min([i for i in (rL + rR) if i > 3], default=None)
    return Analysis(
        cycles=len(s),
        duration_s=(ts[-1] - ts[0]) / 1000.0 if len(ts) > 1 else 0.0,
        reversals_vL=rL, reversals_vR=rR, reversals_cmd=rC,
        net_deg=math.degrees(integrate(body_omega, ts)),
        path_deg=math.degrees(integrate([abs(w) for w in body_omega], ts)),
        cmd_net_deg=math.degrees(integrate(cmd, ts)),
        true_final_deg=s[-1].true_h if s else float("nan"),
        churn_onset=onset,
    )


def print_report(cap: TurnCapture, a: Analysis) -> None:
    print(f"\n=== TURN {cap.angle_deg:.0f} deg  ({a.cycles} cycles, {a.duration_s:.2f}s, "
          f"started={cap.started} settled={cap.settled}) ===\n")
    print("REVERSALS (zero-crossings -- a clean pivot has ZERO):")
    print(f"  actual vL : {len(a.reversals_vL):3d}  {a.reversals_vL[:25]}")
    print(f"  actual vR : {len(a.reversals_vR):3d}  {a.reversals_vR[:25]}")
    print(f"  cmd omega : {len(a.reversals_cmd):3d}  {a.reversals_cmd[:25]}")
    print(f"  churn onset cycle: {a.churn_onset}")
    print("\nINTEGRAL (should equal the commanded angle):")
    print(f"  net actual body-omega : {a.net_deg:8.1f} deg")
    print(f"  |path| actual         : {a.path_deg:8.1f} deg   (wasted back-and-forth = {a.path_deg - abs(a.net_deg):.1f})")
    print(f"  net commanded omega   : {a.cmd_net_deg:8.1f} deg")
    print(f"  ground-truth final    : {a.true_final_deg:8.1f} deg")
    print("\nTRACE (downsampled):")
    print("  cyc  t_ms    vL     vR   cmdw(r/s)  pose_h  otos_h  true_h")
    s = cap.samples
    step = max(1, len(s) // 60)
    for i in range(0, len(s), step):
        x = s[i]
        print(f"  {i:3d} {x.t:6.0f} {x.vL:6.0f} {x.vR:6.0f}   {x.cmd_omega:7.3f}  "
              f"{x.pose_h:7.1f} {x.otos_h:7.1f} {x.true_h:7.1f}")


def main() -> None:
    p = argparse.ArgumentParser(description="Diagnose/validate a single turn's wheel-speed shape.")
    p.add_argument("--angle", type=float, default=360.0, help="turn angle [deg]")
    p.add_argument("--realistic", action="store_true", help="use realistic error profile")
    p.add_argument("--csv", type=str, default=None, help="write full per-cycle trace to CSV")
    p.add_argument("--live", action="store_true",
                   help="real tick thread (vs deterministic stepping)")
    p.add_argument("--gui", action="store_true",
                   help="MOST faithful: SimTransport + robot calibration push (OI activates OTOS)")
    args = p.parse_args()

    if args.gui:
        cap = capture_turn_gui(args.angle)
    elif args.live:
        cap = capture_turn_live(args.angle, realistic=args.realistic)
    else:
        cap = capture_turn(args.angle, realistic=args.realistic)
    if cap.notes:
        print(f"\n[setup] {cap.notes}")
    a = analyze(cap)
    print_report(cap, a)

    if args.csv:
        with open(args.csv, "w") as fh:
            fh.write("t_ms,vL,vR,cmd_omega_radps,pose_h,otos_h,true_h\n")
            for x in cap.samples:
                fh.write(f"{x.t},{x.vL},{x.vR},{x.cmd_omega},{x.pose_h},{x.otos_h},{x.true_h}\n")
        print(f"\nwrote {len(cap.samples)} rows -> {args.csv}")


if __name__ == "__main__":
    main()
