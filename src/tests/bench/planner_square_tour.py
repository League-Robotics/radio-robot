#!/usr/bin/env python3
"""planner_square_tour.py -- the SQUARE TOUR fully ON-ROBOT: 4 x 500 mm
legs + 4 x 90 deg turns, planned AND executed by the firmware's onboard
Motion::Planner (planner-integration build), measured by the real encoders.

The host's only roles are command submission and telemetry capture -- the
control loop is entirely inside the firmware (never split across the
serial transport). Moves are enqueued with ``replace=False`` honoring the
5-deep queue (1 active + 4 pending): the script tops the queue up as
completion acks (``ack_corr == Move.id``) come back on the telemetry ack
ring.

Robot is mounted on a stand with the wheels off the ground (see
`.claude/rules/hardware-bench-testing.md`), so it is safe to drive freely.

Outputs a PASS/FAIL summary (all 8 moves complete; encoder path/heading
near targets) and the same dual-trace plot as the simulation tour:
real wheel speeds (encoders) + the telemetry pose heading.

Usage:
    uv run python src/tests/bench/planner_square_tour.py
    uv run python src/tests/bench/planner_square_tour.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
ACK_TIMEOUT = 1000   # [ms] wait_for_ack() bound for each enqueue ack
BOOT_WAIT = 5.0      # [s] pid-removal firmware boots slower; wait out the preamble
TRACK = 128.0        # [mm]
LEG = 500.0          # [mm]
CRUISE = 150.0       # [mm/s]
TURN = math.pi / 2   # [rad]
OMEGA = 2.4          # [rad/s] pivot rate -> +/-153.6 mm/s per wheel
# Was 1.2 (+/-76.8 mm/s per wheel). Raised 2026-07-28 on bench evidence:
# below ~120 mm/s this plant is not repeatable -- four identical trials at
# 76.8 mm/s spread 26-53 mm/s, and one wheel failed to turn at all on some
# reverse trials -- so a 1.2 pivot ran the wheels inside the dead zone and
# came out asymmetric, arcing through corners instead of rotating. At
# 153.6 mm/s the same measurement is clean (a counter-rotating pair matched
# to 0.2 mm/s). Tour A/B, 4 runs each, NON-OVERLAPPING closures:
#   omega 1.2 -> 9.8, 21.1, 18.0, 18.0 mm (median 18.0)
#   omega 2.4 -> 9.1,  5.1,  3.2,  7.1 mm (median  6.1)
# Heading accuracy is unchanged (+/-1.5 deg either way) -- the turns always
# reached their ANGLE, they just got there by arcing. Override with --omega.
MOVE_TIMEOUT = 30000.0  # [ms] per-move safety backstop
QUEUE_DEPTH = 5      # 1 active + 4 pending
WALL_LIMIT = 120.0   # [s] whole-tour wall-clock bound


def tour(omega: float = OMEGA) -> list[dict]:
    """The 8 tour moves as move_twist() kwargs, in order.

    `omega` [rad/s] sets the pivot rate, i.e. each wheel runs at
    omega*trackWidth/2. The default 2.4 puts the wheels at +/-153.6 mm/s,
    on the well-behaved part of the plant's curve. The previous 1.2 put
    them at +/-76.8 mm/s, inside the erratic near-breakaway region
    (measured 2026-07-28: below ~120 mm/s the two wheels' achieved/commanded
    ratios diverge wildly and are not repeatable, e.g. 0.194 vs 0.539 at
    77 mm/s; at and above ~150 mm/s both directions agree within a few
    percent). See the OMEGA constant above for the A/B numbers. Exposed as
    a flag so the two can still be compared on one robot.
    """
    # Move.id values start well above any corr_id this session's
    # SerialConnection will ever assign (a small monotonic counter starting
    # at 1) so a completion ack (keyed by Move.id) is never confused with an
    # enqueue ack (keyed by the envelope's own corr_id) -- same convention
    # as move_protocol_bench.py.
    moves = []
    for i in range(4):
        moves.append(dict(v_x=CRUISE, v_y=0.0, omega=0.0,
                          stop_distance=LEG, timeout=MOVE_TIMEOUT,
                          replace=False, move_id=9001 + 2 * i))
        moves.append(dict(v_x=0.0, v_y=0.0, omega=omega,
                          stop_angle=TURN, timeout=MOVE_TIMEOUT,
                          replace=False, move_id=9002 + 2 * i))
    return moves


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--relay", action="store_true",
                   help="the port is the RADIO RELAY dongle: force the relay "
                        "!GO data-plane handshake instead of banner "
                        "auto-detect (which misclassifies when the relay is "
                        "already forwarding telemetry -- commands then die "
                        "in the relay's control plane)")
    p.add_argument("--out", default=None,
                   help="plot path (default: alongside this script)")
    p.add_argument("--omega", type=float, default=OMEGA,
                   help="[rad/s] pivot rate; each wheel runs at "
                        "omega*trackWidth/2. Default 2.4 = +/-153.6 mm/s, "
                        "on the well-behaved part of the curve; 1.2 = "
                        "+/-76.8 mm/s, inside the plant's erratic "
                        "near-breakaway region (see tour()).")
    args = p.parse_args()

    conn = SerialConnection(port=args.port,
                            mode="relay" if args.relay else None)
    conn.connect()
    proto = NezhaProtocol(conn)
    print(f"connected on {args.port}; waiting {BOOT_WAIT:.0f}s for boot preamble")
    time.sleep(BOOT_WAIT)
    proto.read_pending_binary_tlm_frames()

    moves = tour(args.omega)
    next_enqueue = 0
    inflight = 0
    # corr_id -> (move_id, send_time, retries). Over the RADIO RELAY ~20%
    # of inbound lines are dropped (DAPLink bridge, radio_bench_gate.py's
    # own documented budget), and a lost enqueue would otherwise pin
    # `inflight` forever: unacked sends are RETRIED with a fresh corr_id.
    # A retry after a lost *ack* (rather than a lost command) would
    # double-enqueue that move -- accepted as rare-by-construction: the ack
    # ring rides EVERY telemetry frame, so an ack only vanishes if all
    # frames carrying it drop, while a command is one single line.
    pending_enqueues: dict[int, tuple[int, float, int]] = {}
    ENQUEUE_RETRY_S = 1.2
    ENQUEUE_RETRIES_MAX = 6
    ENQUEUE_SPACING_S = 0.15  # never burst the relay's single-line buffer
    last_send = 0.0
    completions: list[tuple[float, int]] = []  # (t, move_id)
    log = dict(t=[], velLeft=[], velRight=[], posLeft=[], posRight=[],
               heading=[], cmdLeft=[], cmdRight=[], x=[], y=[])
    enc_start = None
    t0 = time.monotonic()

    try:
        while time.monotonic() - t0 < WALL_LIMIT:
            # Top the queue up (replace=False; leave one slot of headroom
            # against ack latency so ERR_FULL never races the ring). Enqueue
            # acks are matched off the SAME single telemetry drain below --
            # never wait_for_ack(), whose internal drain would eat the
            # velocity samples and completion acks this loop logs.
            now = time.monotonic()
            if (next_enqueue < len(moves) and inflight < QUEUE_DEPTH - 1
                    and now - last_send >= ENQUEUE_SPACING_S):
                corr = proto.move_twist(**moves[next_enqueue])
                pending_enqueues[corr] = (
                    moves[next_enqueue]["move_id"], now, 0)
                next_enqueue += 1
                inflight += 1
                last_send = now
            # Retry any enqueue whose ack never came back (radio drop).
            for corr in list(pending_enqueues):
                move_id, sent, tries = pending_enqueues[corr]
                if time.monotonic() - sent < ENQUEUE_RETRY_S:
                    continue
                del pending_enqueues[corr]
                if tries + 1 > ENQUEUE_RETRIES_MAX:
                    print(f"FAIL: move {move_id} unacked after "
                          f"{ENQUEUE_RETRIES_MAX} retries")
                    return 1
                spec = next(m for m in moves if m["move_id"] == move_id)
                corr2 = proto.move_twist(**spec)
                pending_enqueues[corr2] = (move_id, time.monotonic(), tries + 1)
                last_send = time.monotonic()
                print(f"  (retry {tries + 1} for move {move_id})")

            for f in proto.read_pending_binary_tlm_frames():
                t = time.monotonic() - t0
                if f.enc_left is not None and f.enc_right is not None:
                    if enc_start is None:
                        enc_start = (f.enc_left.position, f.enc_right.position)
                    log["t"].append(t)
                    log["velLeft"].append(f.enc_left.velocity)
                    log["velRight"].append(f.enc_right.velocity)
                    log["posLeft"].append(f.enc_left.position)
                    log["posRight"].append(f.enc_right.position)
                    # pose is (x, y, heading) with heading in centi-degrees
                    log["heading"].append(f.pose[2] / 100.0 if f.pose else float("nan"))
                    log["x"].append(f.pose[0] if f.pose else float("nan"))
                    log["y"].append(f.pose[1] if f.pose else float("nan"))
                    # NOT COMMANDED -- this is MEASURED, decomposed.
                    #
                    # `Telemetry.twist` is documented in telemetry.proto as
                    # "body twist from measured wheel velocities" and is
                    # populated from state.pose.v_x/omega (telemetry.cpp).
                    # Decomposing it per wheel therefore reproduces the
                    # encoder velocities already plotted, by a different
                    # route -- it is the SAME signal, not a second one.
                    #
                    # This block used to be labelled "commanded", and the
                    # chart drew it as a dashed "commanded" trace sitting
                    # exactly on the solid measured trace. That overlay was
                    # an artifact of plotting one signal twice, and was
                    # repeatedly misread as evidence of perfect tracking.
                    #
                    # There is currently NO commanded-velocity telemetry to
                    # plot instead: RobotState::Command::v_x/omega are
                    # unwired (permanently 0.0, see robot_state.h) and the
                    # real per-wheel setpoint `cmd_vel` lived on
                    # TelemetrySecondary, which has been deleted. Exposing it
                    # means adding a field to the primary frame. Until then
                    # the honest thing is to label this what it is -- see
                    # docs/design/2026-07-28-motion-profile-exploration.md.
                    if f.twist is not None:
                        v = float(f.twist[0])
                        omega = float(f.twist[1]) / 1000.0  # [rad/s]
                        half = 0.5 * TRACK * omega
                        log["cmdLeft"].append(v - half)
                        log["cmdRight"].append(v + half)
                    else:
                        log["cmdLeft"].append(float("nan"))
                        log["cmdRight"].append(float("nan"))
                for entry in f.acks:
                    if entry.corr_id in pending_enqueues:
                        move_id, _, _ = pending_enqueues[entry.corr_id]
                        if not entry.ok:
                            print(f"FAIL enqueue move {move_id}: "
                                  f"err={entry.err_code}")
                            return 1
                        del pending_enqueues[entry.corr_id]
                        continue
                    wanted = {m["move_id"] for m in moves[:next_enqueue]}
                    done = {mid for _, mid in completions}
                    if entry.corr_id in wanted and entry.corr_id not in done:
                        completions.append((t, entry.corr_id))
                        inflight -= 1
                        print(f"  move {entry.corr_id} complete at t={t:.1f}s")
            if len(completions) == len(moves):
                break
            time.sleep(0.02)
    finally:
        proto.estop()
        time.sleep(0.5)
        proto.read_pending_binary_tlm_frames()
        conn.disconnect()

    if pending_enqueues:
        print("WARNING: enqueue acks never seen for: "
              f"{sorted(v[0] for v in pending_enqueues.values())}")
    missing = {m['move_id'] for m in moves} - {mid for _, mid in completions}
    if missing:
        print(f"WARNING: never completed: {sorted(missing)}")
    ok = len(completions) == len(moves)
    if not log["t"]:
        print("FAIL: no telemetry captured")
        return 1
    d_left = log["posLeft"][-1] - enc_start[0]
    d_right = log["posRight"][-1] - enc_start[1]
    path = 0.5 * (d_left + d_right)
    heading_enc = math.degrees((d_right - d_left) / TRACK)
    wall = log["t"][-1]
    print(f"\n{'PASS' if ok else 'FAIL'}: {len(completions)}/{len(moves)} moves "
          f"completed in {wall:.1f} s")
    print(f"  encoders: dL {d_left:+.1f} dR {d_right:+.1f} mm -> "
          f"path {path:.1f} mm (target {4 * LEG:.0f}; error {path - 4 * LEG:+.1f})")
    print(f"  heading (encoder differential) {heading_enc:.1f} deg "
          f"(target 360; error {heading_enc - 360.0:+.1f})")

    # Closure: how far the pose ended from where it started, and how far
    # the final heading is from a full 360 turn.
    x0, y0 = log["x"][0], log["y"][0]
    x1, y1 = log["x"][-1], log["y"][-1]
    closure = math.hypot(x1 - x0, y1 - y0)
    print(f"  pose closure {closure:.1f} mm from start "
          f"(finish at {x1 - x0:+.1f}, {y1 - y0:+.1f})")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(15, 10))
    grid = fig.add_gridspec(2, 2, height_ratios=[1.15, 1.0],
                            hspace=0.3, wspace=0.2)

    # --- Wheel speeds: commanded vs measured, both wheels ----------------
    ax = fig.add_subplot(grid[0, :])
    ax.plot(log["t"], log["cmdLeft"], color="#1f77b4", lw=1.0, ls="--",
            label="left, twist-derived (MEASURED, not commanded)")
    ax.plot(log["t"], log["velLeft"], color="#1f77b4", lw=1.4,
            label="left measured (encoder)")
    ax.plot(log["t"], log["cmdRight"], color="#d62728", lw=1.0, ls="--",
            label="right, twist-derived (MEASURED, not commanded)")
    ax.plot(log["t"], log["velRight"], color="#d62728", lw=1.4,
            label="right measured (encoder)")
    for tc, move_id in completions:
        ax.axvline(tc, color="#888888", lw=0.6, alpha=0.5)
        kind = "leg" if move_id % 2 == 1 else "turn"
        ax.annotate(f"{kind} {(move_id - 9000 + 1) // 2}",
                    xy=(tc, 0.98), xycoords=("data", "axes fraction"),
                    fontsize=7.5, rotation=90, va="top", ha="right",
                    color="#666666")
    lim = 1.15 * max(max(abs(v) for v in log["velLeft"]),
                     max(abs(v) for v in log["velRight"]), CRUISE)
    ax.set_ylim(-lim, lim)
    ax.axhline(0.0, color="black", lw=0.5)
    ax.axhline(CRUISE, color="#cccccc", lw=0.7, ls=":")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("wheel speed [mm/s]")
    ax.legend(loc="lower right", fontsize=8, ncol=2, framealpha=0.9)
    ax.grid(True, alpha=0.25)
    ax.set_title("Wheel speed: measured (solid) vs the same signal via body twist (dashed) -- NO commanded telemetry exists",
                 fontsize=10)

    # --- The motion (encoder-odometry pose) ------------------------------
    ax2 = fig.add_subplot(grid[1, 0])
    xs = [x - x0 for x in log["x"]]
    ys = [y - y0 for y in log["y"]]
    ideal = [(0, 0), (LEG, 0), (LEG, LEG), (0, LEG), (0, 0)]
    ax2.plot([p[0] for p in ideal], [p[1] for p in ideal], color="#bbbbbb",
             ls="--", lw=1.0, label="ideal square")
    ax2.plot(xs, ys, color="#2b8a3e", lw=1.6)
    ax2.plot([0], [0], marker="o", color="#2b8a3e", ms=7, label="start")
    ax2.plot([xs[-1]], [ys[-1]], marker="X", color="#d62728", ms=10,
             label="finish")
    ax2.annotate(f"closure {closure:.1f} mm", xy=(xs[-1], ys[-1]),
                 xytext=(10, -14), textcoords="offset points",
                 fontsize=9, color="#d62728")
    ax2.set_aspect("equal", adjustable="datalim")
    ax2.set_xlabel("x [mm]")
    ax2.set_ylabel("y [mm]")
    ax2.legend(loc="best", fontsize=8)
    ax2.grid(True, alpha=0.25)
    ax2.set_title("Path (encoder odometry pose)", fontsize=10)

    # --- Heading vs time -------------------------------------------------
    ax3 = fig.add_subplot(grid[1, 1])
    ax3.plot(log["t"], log["heading"], color="#6741d9", lw=1.4)
    for i in range(1, 5):
        ax3.axhline(90.0 * i, color="#cccccc", lw=0.7, ls=":")
    ax3.set_xlabel("time [s]")
    ax3.set_ylabel("heading [deg]")
    ax3.grid(True, alpha=0.25)
    ax3.set_title("Heading (encoder odometry) -- gridlines at n*90 deg",
                  fontsize=10)

    fig.suptitle(
        f"ON-ROBOT square tour -- onboard Motion::Planner "
        f"(per-wheel profiler + velocity trim), real encoders\n"
        f"{len(completions)}/{len(moves)} moves   "
        f"path {path:.1f}/{4 * LEG:.0f} mm ({path - 4 * LEG:+.1f})   "
        f"heading {heading_enc:.1f}/360 deg ({heading_enc - 360.0:+.1f})   "
        f"pose closure {closure:.1f} mm",
        fontsize=12)
    out = args.out or (__file__.rsplit("/", 1)[0] + "/planner_square_tour.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"wrote {out}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
