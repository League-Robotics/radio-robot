#!/usr/bin/env python3
"""velocity_step_response.py — P4 binary-wire velocity-step response
characterization (106-002, resonance taming).

Bench setup: robot on the stand, wheels free (`.claude/rules/hardware-bench-
testing.md`). This is the "drive-arm step method" the resonance issue
(`heading-loop-output-clamp-and-velocity-resonance.md` Part 2) prescribes:
command a clean forward velocity STEP (planner out — no host-side profiling,
just `twist(v_x, omega=0)` straight from zero) at each of the historical
resonance-grid speeds (70/140/250 mm/s), capture the measured `vel_left`/
`vel_right` trace via binary telemetry, and report peak/overshoot/rise-time.

Unlike the pre-P4 `pid_hold_speed.py` (text `DEV M <port> VEL`, no longer
reachable — see that file's own module docstring for the wire this repo
actually speaks now), this script drives BOTH wheels together via the P4
wire's only motion verb (`NezhaProtocol.twist()`), and applies gains LIVE via
`NezhaProtocol.config()` (106-002's own ConfigDelta live-apply) — no reflash
between gain trials, matching binding requirement #9 ("everything tunable
live").

Usage:
    # One gain set, the full 70/140/250 grid, live-applied (no reflash):
    uv run python tests/bench/velocity_step_response.py \\
        --kp 0.0014 --ki 0.005 --kff 0.00135 --imax 0.3 --kaw 20.0

    # Boot-default gains (skip config(), just characterize what's shipped):
    uv run python tests/bench/velocity_step_response.py --no-config

    # A single speed:
    uv run python tests/bench/velocity_step_response.py --speeds 140
"""
from __future__ import annotations

import argparse
import csv
import pathlib
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
ACK_TIMEOUT = 500  # [ms]
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--relay", action="store_true", help="port is a radio relay dongle (default: auto-detect)")
    p.add_argument("--speeds", default="70,140,250", help="comma-separated step targets, mm/s")
    p.add_argument("--step-duration", type=float, default=1.6,  # [s]
                   help="twist() duration -- must cover --capture (default 1.6)")
    p.add_argument("--capture", type=float, default=1.3,  # [s]
                   help="seconds of telemetry captured per step, from the twist() call (default 1.3)")
    p.add_argument("--settle", type=float, default=0.6,  # [s]
                   help="seconds stopped between steps before the next one starts (default 0.6)")
    p.add_argument("--kp", type=float, default=None)
    p.add_argument("--ki", type=float, default=None)
    p.add_argument("--kff", type=float, default=None)
    p.add_argument("--imax", type=float, default=None)
    p.add_argument("--kaw", type=float, default=None)
    p.add_argument("--no-config", action="store_true",
                   help="skip config() entirely -- characterize whatever gains are already live")
    p.add_argument("--csv", default=str(_REPO_ROOT / "tests" / "bench" / "out" / "velocity_step_response.csv"))
    return p.parse_args()


# ---------------------------------------------------------------------------
# Pure analysis helpers -- no I/O, testable without hardware.
# ---------------------------------------------------------------------------

def overshoot_pct(target: float, peak: float) -> float:
    """Percent the peak exceeds target; 0.0 if it never overshoots."""
    if target <= 0.0 or peak <= target:
        return 0.0
    return (peak - target) / target * 100.0


def rise_time_s(samples: list[tuple[float, float]], target: float, frac: float = 0.9) -> float | None:
    """Seconds from the first sample to the first crossing of `frac`*target.

    `samples` is a list of (t_s, value) pairs, t_s relative to the step's own
    t=0. Returns None if the threshold is never crossed (e.g. target is 0 or
    the step never gets there within the capture window).
    """
    if not samples or target <= 0.0:
        return None
    threshold = frac * target
    t0 = samples[0][0]
    for t, v in samples:
        if v >= threshold:
            return t - t0
    return None


def summarize_step(times_ms: list[int], vel: list[float], target: float) -> dict:
    """Reduce one step's raw (firmware-ms, measured-mm/s) samples to peak/
    overshoot/rise-time/settled-mean. `times_ms` are the firmware's own `now`
    field (monotonic within one boot, wraps only after ~49 days -- fine for a
    single bench session)."""
    if not times_ms:
        return {"peak": None, "overshoot_pct": None, "rise_time_s": None, "settled_mean": None}
    t0 = times_ms[0]
    rel = [(t - t0) / 1000.0 for t in times_ms]
    peak = max(vel)
    rt = rise_time_s(list(zip(rel, vel)), target)
    # "settled" = last 30% of the capture window.
    cutoff = rel[0] + 0.7 * (rel[-1] - rel[0]) if len(rel) > 1 else rel[0]
    settled = [v for t, v in zip(rel, vel) if t >= cutoff]
    settled_mean = sum(settled) / len(settled) if settled else None
    return {
        "peak": peak,
        "overshoot_pct": overshoot_pct(target, peak),
        "rise_time_s": rt,
        "settled_mean": settled_mean,
    }


# ---------------------------------------------------------------------------
# Bench I/O
# ---------------------------------------------------------------------------

def dev_wait_ack(proto: NezhaProtocol, corr_id: int, attempts: int = 3):
    """Bounded retry over wait_for_ack() -- mirrors rig_dev.py's own
    wait_for_ack_retrying(): the live wire occasionally delays an individual
    ack (see clasi/issues/ack-ring-intermittent-delivery-gap.md), not a bug
    in this script."""
    ack = None
    for _ in range(attempts):
        ack = proto.wait_for_ack(corr_id, timeout=ACK_TIMEOUT)
        if ack is not None:
            return ack
    return ack


def _one_twist_capture(proto: NezhaProtocol, target: float, step_duration: float,
                        capture: float) -> tuple[bool, bool, list[int], list[float], list[float]]:
    """One twist()+capture attempt. Returns (acked, moved, times_ms, vel_l, vel_r).

    Deliberately does NOT call `NezhaProtocol.wait_for_ack()` -- that call's
    own docstring warns it "destructively drains the same telemetry queue
    while searching for its own corr_id match", which would silently
    discard exactly the early-ramp samples this capture most needs.
    Instead, `read_binary_tlm_frames()` (the non-destructive blocking drain
    every frame goes through) is called ONCE for the whole capture window,
    and the ack is found by scanning its own result -- one shared,
    non-lossy read stream for both purposes.
    """
    corr_id = proto.twist(v_x=target, omega=0.0, duration=step_duration * 1000.0)
    frames: list[TLMFrame] = proto.read_binary_tlm_frames(int(capture * 1000))

    acked = False
    moved = False
    times_ms: list[int] = []
    vel_l: list[float] = []
    vel_r: list[float] = []
    for f in frames:
        if f.acks:
            for entry in f.acks:
                if entry.corr_id == corr_id and entry.ok:
                    acked = True
        if f.active:
            moved = True
        if f.vel is None or f.t is None:
            continue
        times_ms.append(f.t)
        vel_l.append(float(f.vel[0]))
        vel_r.append(float(f.vel[1]))
        if abs(f.vel[0]) > 5.0 or abs(f.vel[1]) > 5.0:
            moved = True
    return acked, moved, times_ms, vel_l, vel_r


def run_step(proto: NezhaProtocol, target: float, step_duration: float, capture: float,
             attempts: int = 3) -> dict:
    """Command one clean forward velocity step (from a stopped start) and
    capture measured vel_left/vel_right for `capture` seconds. Returns a
    dict with per-wheel summaries plus the raw trace (for CSV logging).

    Retries the WHOLE twist()+capture attempt (not just the ack wait) up to
    `attempts` times if neither an ack nor any real wheel motion was
    observed -- the direct-USB CDC link is a characterized, pre-existing
    bench gotcha (`clasi/issues/ack-ring-intermittent-delivery-gap.md`,
    `bench-verification-gotchas-088.md`): an occasional OUTBOUND command can
    be dropped outright, which no amount of ack-wait retrying alone fixes
    (`twist()` is fire-and-poll -- if the bytes never land, nothing comes
    back to wait for). Confirmed independent of this ticket's own
    ConfigDelta changes: reproduced with a bare `twist()`/`stop()` sequence,
    no `config()` involved.
    """
    acked = moved = False
    times_ms: list[int] = []
    vel_l: list[float] = []
    vel_r: list[float] = []
    for attempt in range(attempts):
        acked, moved, times_ms, vel_l, vel_r = _one_twist_capture(proto, target, step_duration, capture)
        if acked or moved:
            break
        print(f"    (retry {attempt + 1}/{attempts}: no ack and no motion observed -- re-sending twist())")

    proto.stop()

    left = summarize_step(times_ms, vel_l, target)
    right = summarize_step(times_ms, vel_r, target)
    return {
        "target": target,
        "twist_acked": acked,
        "left": left,
        "right": right,
        "trace": list(zip(times_ms, vel_l, vel_r)),
    }


def main() -> int:
    args = _args()
    speeds = [float(s) for s in args.speeds.split(",") if s.strip()]
    mode = "relay" if args.relay else None

    gains = {}
    if not args.no_config:
        if args.kp is not None:
            gains["pid.kp"] = args.kp
        if args.ki is not None:
            gains["pid.ki"] = args.ki
        if args.kff is not None:
            gains["pid.kff"] = args.kff
        if args.imax is not None:
            gains["pid.iMax"] = args.imax
        if args.kaw is not None:
            gains["pid.kaw"] = args.kaw

    csv_path = pathlib.Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    conn = SerialConnection(port=args.port, mode=mode)
    proto: NezhaProtocol | None = None
    csv_file = None

    try:
        info = conn.connect()
        if info.get("status") not in ("connected", "already_connected"):
            print(f"ERROR: connect failed: {info}")
            return 2
        print(f"  connected: mode={info.get('mode')}")
        proto = NezhaProtocol(conn)
        time.sleep(1.0)  # let boot-time telemetry queue drain before the first step
        proto.read_pending_binary_tlm_frames()

        if gains:
            print(f"  applying gains live (no reflash): {gains}")
            corr_id = proto.config(**gains)
            ack = dev_wait_ack(proto, corr_id)
            print(f"    config() ack: {ack}")
            if ack is None or not ack.ok:
                print("  WARNING: config() did not ack OK -- gains may not have applied")
        else:
            print("  --no-config: characterizing whatever gains are already live")

        csv_file = open(csv_path, "a", newline="")
        writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            writer.writerow(["gains", "target", "wheel", "t_ms", "vel"])

        results = []
        for target in speeds:
            print(f"\n  --- step target={target:g} mm/s ---")
            time.sleep(args.settle)
            result = run_step(proto, target, args.step_duration, args.capture)
            results.append(result)
            for t_ms, vl, vr in result["trace"]:
                writer.writerow([str(gains), target, "L", t_ms, vl])
                writer.writerow([str(gains), target, "R", t_ms, vr])
            L, R = result["left"], result["right"]
            print(f"    twist acked: {result['twist_acked']}")
            print(f"    L: peak={L['peak']}  overshoot={L['overshoot_pct']:.1f}%"
                  f"  rise={L['rise_time_s']}  settled={L['settled_mean']}"
                  if L["peak"] is not None else "    L: no data")
            print(f"    R: peak={R['peak']}  overshoot={R['overshoot_pct']:.1f}%"
                  f"  rise={R['rise_time_s']}  settled={R['settled_mean']}"
                  if R["peak"] is not None else "    R: no data")

        print(f"\n  === summary ({gains if gains else 'boot-default gains'}) ===")
        print(f"  {'target':>8}  {'ovL%':>6}  {'ovR%':>6}  {'riseL':>7}  {'riseR':>7}")
        worst_overshoot = 0.0
        for result in results:
            L, R = result["left"], result["right"]
            ov_l = L["overshoot_pct"] or 0.0
            ov_r = R["overshoot_pct"] or 0.0
            worst_overshoot = max(worst_overshoot, ov_l, ov_r)
            print(f"  {result['target']:>8.0f}  {ov_l:>6.1f}  {ov_r:>6.1f}  "
                  f"{str(L['rise_time_s']):>7}  {str(R['rise_time_s']):>7}")
        print(f"\n  worst overshoot across the grid: {worst_overshoot:.1f}%  "
              f"({'PASS' if worst_overshoot < 10.0 else 'FAIL'} vs <~10% bar)")
        print(f"  CSV: {csv_path}")

    finally:
        if csv_file is not None:
            csv_file.close()
        if proto is not None:
            try:
                proto.stop()
            except Exception as exc:
                print(f"  WARN: stop() failed during cleanup: {exc}")
        if conn.is_open:
            conn.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
