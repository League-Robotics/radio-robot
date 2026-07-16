"""turn_sweep.py -- turn-overshoot dataset collection (HITL CLI, not pytest).

Runs a grid of in-place pivots -- commanded angle x yaw-rate ceiling, with
alternating direction so the robot stays wound-neutral on the playfield --
and records one CSV row per turn:

    cmd_deg        signed commanded turn
    ceiling_wheel  commanded per-wheel speed ceiling  [mm/s]
    peak_wheel     measured peak |wheel speed|        [mm/s]
    final_deg      measured stop heading (encoders)   [deg]
    overshoot_deg  final - cmd (signed toward the turn direction)

The dataset feeds the terminal-coast fit: overshoot ~ c0 + c1 * peak_rate
(see clasi/issues/real-robot-motion-calibration-undershoot.md). In-place
pivots do not translate, so this is playfield-safe; every turn ends with a
verified stop and the baseline is taken only after confirmed stillness.

Usage:
    uv run python src/tests/bench/turn_sweep.py [--out tests/notebooks/out/turn_sweep.csv]
"""
from __future__ import annotations

import argparse
import base64
import csv
import math
import pathlib
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "host"))

from robot_radio.io.serial_conn import SerialConnection  # noqa: E402
from robot_radio.robot import legacy_translate  # noqa: E402
from robot_radio.robot.pb2 import envelope_pb2  # noqa: E402
from robot_radio.testgui.transport import find_robot_serial_port  # noqa: E402

TRACKWIDTH = 128.0  # [mm]
ANGLES = [30.0, 90.0, 180.0, 360.0]           # [deg] magnitude; sign alternates
CEILINGS = [70.0, 140.0, 210.0, 280.0, 384.0]  # [mm/s] per-wheel; 384 = firmware default (6 rad/s)
TURN_TIMEOUT = 25.0   # [s]
SETTLE = 0.8          # [s] idle dwell that ends a capture


def stop_env():
    env = envelope_pb2.CommandEnvelope()
    env.stop.SetInParent()
    return env


def turn_env(angle, ceiling_wheel):  # [deg], [mm/s]
    omega = ceiling_wheel / (TRACKWIDTH / 2.0)          # [rad/s]
    seg = legacy_translate.segment_for_move(
        0.0, 0.0, angle * 100.0,
        yaw_rate_max=math.degrees(omega) * 100.0)       # [cdeg/s]
    return envelope_pb2.CommandEnvelope(segment=seg)


def send_verified(conn, env, tries=4):
    for _ in range(tries):
        r = conn.send_envelope(env, read_timeout=800)["reply"]
        if r is not None and r.WhichOneof("body") == "ok":
            return True
        time.sleep(0.25)
    return False


def wait_still_and_baseline(conn, budget=8.0):
    """STOP until both wheels read < 5 mm/s for a 0.4s dwell; return the last
    encoder pair as the baseline. Encoder movement is ground truth -- flags
    in lossy frames are not (project bench-harness rule)."""
    still_since = None
    base = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < budget:
        conn.send_envelope(stop_env(), read_timeout=300)
        probe_t = time.monotonic()
        calm = True
        while time.monotonic() - probe_t < 0.4:
            for f in conn.drain_binary_tlm():
                if f.tlm.has_enc:
                    base = (f.tlm.enc_left, f.tlm.enc_right)
                if f.tlm.has_vel and (abs(f.tlm.vel_left) > 5 or abs(f.tlm.vel_right) > 5):
                    calm = False
            time.sleep(0.02)
        if calm and base is not None:
            still_since = still_since or time.monotonic()
            if time.monotonic() - still_since >= 0.4:
                return base
        else:
            still_since = None
    raise RuntimeError("could not confirm stillness -- aborting sweep")


TRACE_FIELDS = ["now", "ts_left", "ts_right", "enc_left", "enc_right",
                "vel_left", "vel_right", "cmd_vel_left", "cmd_vel_right", "active"]


def one_turn(conn, angle, ceiling, trace=None):
    """Execute one pivot; return (final_deg, peak_wheel). Resend only after
    proving nothing started AND re-verifying stillness (ring-cleared).
    If ``trace`` is a list, every enc-bearing frame is appended to it as a
    dict of TRACE_FIELDS (frame loss over the radio leaves holes; encoder
    values are cumulative so heading-vs-time survives them)."""
    base = wait_still_and_baseline(conn)
    for attempt in range(4):
        if attempt:
            base = wait_still_and_baseline(conn)
        conn.send_envelope(turn_env(angle, ceiling), read_timeout=800)
        # start probe: busy flag OR encoder movement
        started = False
        probe_end = time.monotonic() + 3.0
        while time.monotonic() < probe_end and not started:
            for f in conn.drain_binary_tlm():
                if not f.tlm.has_enc:
                    continue
                if f.tlm.active or abs(f.tlm.enc_left - base[0]) > 2.0 \
                        or abs(f.tlm.enc_right - base[1]) > 2.0:
                    started = True
            time.sleep(0.02)
        if started:
            break
        print(f"    send lost (proven idle) -- safe resend {attempt + 1}/3")
    else:
        raise RuntimeError("segment never started after 4 verified-idle sends")

    last = None
    peak = 0.0
    seen, idle_since = False, None
    t0 = time.monotonic()
    while time.monotonic() - t0 < TURN_TIMEOUT:
        for f in conn.drain_binary_tlm():
            if not f.tlm.has_enc:
                continue
            last = f.tlm
            if trace is not None:
                trace.append({k: getattr(f.tlm, k) for k in TRACE_FIELDS})
            if f.tlm.has_vel:
                peak = max(peak, abs(f.tlm.vel_left), abs(f.tlm.vel_right))
            if f.tlm.active:
                seen = True
        if seen and last is not None and not last.active:
            idle_since = idle_since or time.monotonic()
            if time.monotonic() - idle_since >= SETTLE:
                break
        else:
            idle_since = None
        time.sleep(0.02)
    conn.send_envelope(stop_env(), read_timeout=400)
    final = math.degrees(((last.enc_right - base[1]) - (last.enc_left - base[0])) / TRACKWIDTH)
    return final, peak


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(REPO / "tests/notebooks/out/turn_sweep.csv"))
    ap.add_argument("--port", default=None)
    ap.add_argument("--relay", action="store_true",
                    help="connect through the radio relay dongle (playfield)")
    ap.add_argument("--both", action="store_true",
                    help="run BOTH directions in every (angle, ceiling) cell "
                         "(un-confounds direction from angle; round-1 grid "
                         "alternated sign across cells)")
    ap.add_argument("--ceilings", default=None,
                    help="comma-separated wheel-speed ceilings [mm/s]")
    ap.add_argument("--angles", default=None,
                    help="comma-separated turn magnitudes [deg]")
    ap.add_argument("--trace", action="store_true",
                    help="dump per-frame telemetry to one CSV per turn "
                         "(turn_trace_<cmd>_<ceiling>.csv beside --out)")
    args = ap.parse_args()

    ceilings = [float(c) for c in args.ceilings.split(",")] if args.ceilings else CEILINGS
    angles = [float(a) for a in args.angles.split(",")] if args.angles else ANGLES

    if args.relay:
        # Playfield: the robot is reached over the radio through the relay
        # dongle; SerialConnection(mode='relay') runs the !GO handshake.
        from robot_radio.io.serial_conn import list_serial_ports
        port = args.port or (list_serial_ports() or [None])[0]
        assert port, "no relay serial port found"
        print(f"relay port: {port}")
        conn = SerialConnection(port=port, mode="relay")
    else:
        port = args.port or find_robot_serial_port()
        assert port, "no robot serial port found (use --relay on the playfield)"
        print(f"robot port: {port}")
        conn = SerialConnection(port=port, mode="direct")
    conn.connect(skip_ping=False)
    assert conn.is_open, "connect failed"

    rows = []
    sign = 1.0
    try:
        assert send_verified(conn, envelope_pb2.CommandEnvelope(
            stream=envelope_pb2.StreamControl(period=20, binary=True))), "stream arm failed"
        time.sleep(0.3)
        dirs_per_cell = 2 if args.both else 1
        total = len(ceilings) * len(angles) * dirs_per_cell
        n = 0
        for ceiling in ceilings:
            for mag in angles:
                for _ in range(dirs_per_cell):
                    n += 1
                    cmd = sign * mag
                    sign = -sign                  # alternate -- stay wound-neutral
                    trace = [] if args.trace else None
                    final, peak = one_turn(conn, cmd, ceiling, trace=trace)
                    if trace:
                        tp = pathlib.Path(args.out).parent / \
                            f"turn_trace_{cmd:+.0f}_{ceiling:.0f}.csv"
                        with tp.open("w", newline="") as tf:
                            tw = csv.DictWriter(tf, fieldnames=TRACE_FIELDS)
                            tw.writeheader()
                            tw.writerows(trace)
                    over = (final - cmd) * (1.0 if cmd >= 0 else -1.0)
                    rows.append({"cmd_deg": cmd, "ceiling_wheel": ceiling,
                                 "peak_wheel": round(peak, 1),
                                 "final_deg": round(final, 2),
                                 "overshoot_deg": round(over, 2)})
                    print(f"[{n:2d}/{total}] cmd={cmd:+7.1f}  ceil={ceiling:5.0f}  "
                          f"peak={peak:5.0f}  final={final:+8.2f}  over={over:+6.2f}")
    finally:
        try:
            wait_still_and_baseline(conn, budget=6.0)
            print("[safety] wheels confirmed stopped")
        finally:
            conn.disconnect()

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["cmd_deg", "ceiling_wheel", "peak_wheel",
                                          "final_deg", "overshoot_deg"])
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {len(rows)} rows -> {out}")


if __name__ == "__main__":
    main()
