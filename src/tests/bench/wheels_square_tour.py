#!/usr/bin/env python3
"""wheels_square_tour.py -- the square tour with NO planner: every leg and
turn is a host-shaped trapezoid streamed as plain ``wheels()`` commands.

The firmware has no profile feature on this path and needs none: the host
samples each segment's trapezoid every --tick seconds and re-arms a bounded
``WHEELS`` command (the same staircase idiom velocity_profile_gate.py
proved). ``Core::DifferentialDrive`` actuates each step; ``Motion::Planner`` never runs.
This is the control arm against planner_square_tour.py -- same square, same
closure metric, opposite command path.

Tour: 4 x 500 mm legs + 4 x 90 deg CCW pivots, a settle dwell at every
segment boundary (rest poses bracket every segment, per
.claude/rules/playfield-testing.md's boundary-pose doctrine -- here from
telemetry pose rather than the camera, since the robot is on a stand).

Closure = distance between the first and last REST poses, plus heading
error vs the +360 deg the four turns should sweep. Pose is the firmware's
own encoder odometry off the telemetry stream; on a stand that is the only
truth available, and it is exactly what the planner tour reports too.

Usage:
    uv run python src/tests/bench/wheels_square_tour.py --port <tovez> \
        [--leg 500] [--speed 200] [--turn-speed 100] [--outdir DIR]
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT / "src" / "host"))

TRACK = 128.0          # [mm] tovez geometry.trackwidth (data/robots/tovez.json)
TICK = 0.10            # [s] host re-arm period
LEASE = 0.4            # [s] each command's own bounded window
RAMP_LEG = 0.75        # [s] leg trapezoid ramp
RAMP_TURN = 0.30       # [s] turn trapezoid ramp
SETTLE = 1.2           # [s] rest dwell at every segment boundary


def trapezoid(peak, ramp, duration, t):  # [mm/s] [s] [s] [s]
    if t < 0.0 or t >= duration:
        return 0.0
    if t < ramp:
        return peak * (t / ramp)
    if t > duration - ramp:
        return peak * ((duration - t) / ramp)
    return peak


def stream_segment(proto, log, v_left_peak, v_right_peak, ramp, duration,
                   drain_fn=None):
    """One segment: both wheels follow the same trapezoid shape, scaled per
    wheel (signs included), streamed as re-armed WHEELS commands.

    Returns the list of (t, v_left, v_right) actually commanded, so a caller
    measuring the trim mechanism's own authority can see what was asked for
    (the sub-deadband floor lives here, not in the plant).
    """
    sink = drain_fn or drain
    commanded = []
    start = time.monotonic()
    step = 0
    while True:
        now = time.monotonic() - start
        if now >= duration:
            break
        shape = trapezoid(1.0, ramp, duration, now + TICK / 2.0)
        proto.wheels(v_left_peak * shape, v_right_peak * shape, LEASE * 1000.0)
        commanded.append((now, v_left_peak * shape, v_right_peak * shape))
        step += 1
        sink(proto, log)
        remaining = (start + step * TICK) - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
    proto.wheels(0.0, 0.0, LEASE * 1000.0)
    return commanded


def drain(proto, log):
    for f in proto.read_pending_binary_tlm_frames():
        if f.pose is None:
            continue
        log.append((f.t, f.pose[0], f.pose[1], f.pose[2] / 100.0,
                    float(f.enc_left.velocity) if f.enc_left else 0.0,
                    float(f.enc_right.velocity) if f.enc_right else 0.0,
                    float(f.enc_left.position) if f.enc_left else 0.0,
                    float(f.enc_right.position) if f.enc_right else 0.0))


def rest_stats(samples):
    """Spread of the heading samples taken during the back half of a rest
    dwell -- the measurement noise floor any trim tolerance is fighting."""
    tail = samples[len(samples) // 2:] or samples
    if not tail:
        return dict(n=0, sd=float("nan"), ptp=float("nan"))
    mean = sum(tail) / len(tail)
    var = sum((s - mean) ** 2 for s in tail) / len(tail)
    return dict(n=len(tail), sd=math.sqrt(var), ptp=max(tail) - min(tail),
                mean=mean, last=tail[-1])


def settle_measure(proto, log):
    """Rest dwell; returns ((x, y, heading), rest-heading spread stats).

    The pose returned is the LAST sample, exactly as before -- the stats are
    pure instrumentation and feed nothing back into the motion."""
    mark = len(log)
    end = time.monotonic() + SETTLE
    while time.monotonic() < end:
        proto.wheels(0.0, 0.0, LEASE * 1000.0)
        time.sleep(TICK)
        drain(proto, log)
    pose = log[-1][1:4] if log else (0.0, 0.0, 0.0)
    return pose, rest_stats([r[3] for r in log[mark:]])


def settle(proto, log):
    return settle_measure(proto, log)[0]


def trim_to_heading(proto, log, pose, target, tol, max_nudges, turn_scale,
                    settle_fn=None, drain_fn=None):
    """Close the cumulative-heading residual to `target` with short low-speed
    pivot nudges off telemetry heading. Pure WHEELS -- the same mechanism the
    tour's turns use, only slower and shorter.

    This is THE trim mechanism under test; planner_square_tour.py imports it
    rather than growing a second copy. Returns
    (pose, nudge_records, elapsed) -- every nudge records what residual it was
    asked to close, what it commanded, and what heading delta it actually
    delivered, so a bounce (nudge authority > tolerance) is visible in the
    data instead of being inferred from a closure number.
    """
    settle_fn = settle_fn or (lambda: settle_measure(proto, log))
    nudges = []
    t_trim = time.monotonic()
    for i in range(max_nudges):
        residual = target - pose[2]                          # [deg]
        if abs(residual) <= tol:
            break
        nudge_arc = math.radians(abs(residual)) * (TRACK / 2.0) * turn_scale
        v = 60.0 if nudge_arc > 8.0 else 30.0                # [mm/s] gentle
        nudge_T = nudge_arc / v + 0.2                        # [s]
        sign = 1.0 if residual > 0 else -1.0
        h_before = pose[2]
        commanded = stream_segment(proto, log, -sign * v, sign * v, 0.2,
                                   nudge_T, drain_fn)
        pose, stats = settle_fn()
        nudges.append(dict(
            n=i + 1, residual_before=residual, arc=nudge_arc, peak=v,
            duration=nudge_T, steps=len(commanded),
            peak_commanded=max((abs(c[2]) for c in commanded), default=0.0),
            heading_before=h_before, heading_after=pose[2],
            delivered=pose[2] - h_before, residual_after=target - pose[2],
            rest_sd=stats["sd"], rest_ptp=stats["ptp"]))
    return pose, nudges, time.monotonic() - t_trim


def latest_measure(proto, log):
    drain(proto, log)
    return log[-1] if log else None


def online_segment(proto, log, target, direction, cruise, args, kind):
    """Drive until MEASURED progress reaches `target` [mm of mean wheel
    travel for legs; deg of heading for turns]. No schedule: each tick
    commands v = min(cruise, sqrt(2*decel*remaining)), slewed by accel,
    and the segment ends when the measurement -- not a clock -- says so.

    kind='leg': direction is +1 (forward); progress = mean wheel travel.
    kind='turn': direction is +1 CCW; progress = heading delta [deg].
    A wall-clock timeout (3x the ideal duration + 5s) stays as the safety
    backstop, never the terminator."""
    m0 = latest_measure(proto, log)
    if m0 is None:
        return
    h0 = m0[3]
    e0 = 0.5 * (m0[6] + m0[7])
    v = 0.0
    ideal = target / cruise if kind == "leg" else \
        (math.radians(target) * (TRACK / 2.0)) / cruise
    deadline = time.monotonic() + 3.0 * ideal + 5.0
    step = 0
    start = time.monotonic()
    while time.monotonic() < deadline:
        m = latest_measure(proto, log)
        # PREDICTED progress: the raw measurement is one sample-age plus one
        # actuation delay old by the time a decision made on it takes
        # effect (~0.2s total) -- at pivot rates near 90 deg/s that is the
        # whole +4 deg/turn overshoot the unpredicted version measured.
        # Extrapolate forward by args.lead using the MEASURED rate, and
        # decide on the prediction.
        if kind == "leg":
            progress = 0.5 * (m[6] + m[7]) - e0                 # [mm]
            rate = 0.5 * (m[4] + m[5])                           # [mm/s]
            predicted = progress + rate * args.lead              # [mm]
            remaining = target - predicted                       # [mm]
            rem_lin = remaining
        else:
            progress = m[3] - h0                                 # [deg]
            rate = math.degrees((m[5] - m[4]) / TRACK)           # [deg/s]
            predicted = progress + rate * args.lead              # [deg]
            remaining = target - predicted                       # [deg]
            rem_lin = math.radians(max(0.0, remaining)) * (TRACK / 2.0)
        if remaining <= (1.0 if kind == "leg" else 0.3):
            break
        envelope = math.sqrt(2.0 * args.decel * max(0.0, rem_lin) / 1000.0) \
            * math.sqrt(1000.0)                                  # [mm/s]
        want = min(cruise, envelope)
        v = min(want, v + args.accel * TICK)                     # slew up only
        if kind == "leg":
            proto.wheels(v, v, LEASE * 1000.0)
        else:
            proto.wheels(-v, v, LEASE * 1000.0)
        step += 1
        pause = (start + step * TICK) - time.monotonic()
        if pause > 0:
            time.sleep(pause)
    proto.wheels(0.0, 0.0, LEASE * 1000.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--leg", type=float, default=500.0)         # [mm]
    ap.add_argument("--speed", type=float, default=200.0)       # [mm/s] leg peak
    ap.add_argument("--turn-speed", type=float, default=100.0)  # [mm/s] wheel peak in a pivot
    ap.add_argument("--leg-scale", type=float, default=1.0,
                    help="multiply the commanded leg distance -- the wheels-"
                         "path distance calibration (legs deliver ~101.5% raw)")
    ap.add_argument("--turn-scale", type=float, default=1.0,
                    help="multiply the commanded pivot arc -- the wheels-path "
                         "turn calibration (measured 2026-08-04: raw geometric "
                         "arc delivers ~86.3 of 90 deg on tovez)")
    ap.add_argument("--trim", action="store_true",
                    help="after each turn settles, close the residual to the "
                         "cumulative 90*n target with short low-speed pivot "
                         "nudges off telemetry heading -- still pure WHEELS")
    ap.add_argument("--trim-tol", type=float, default=1.0)      # [deg]
    ap.add_argument("--trim-max-nudges", type=int, default=3,
                    help="cap on trim nudges per corner. A tolerance below "
                         "the nudge mechanism's own authority cannot "
                         "converge -- it bounces; the cap bounds the bounce "
                         "and the per-nudge record shows it.")
    ap.add_argument("--label", default="tour")
    ap.add_argument("--online", action="store_true",
                    help="ONLINE mode: no time schedule at all. Each tick the "
                         "command is computed from MEASURED remaining distance "
                         "(mean encoder travel for legs, telemetry heading for "
                         "turns): v = min(cruise, sqrt(2*a*remaining)), accel "
                         "slew-limited, terminated when the measurement says "
                         "arrived. Needs no leg/turn scale and no trim -- the "
                         "goal is closed on, not scheduled.")
    ap.add_argument("--decel", type=float, default=300.0)       # [mm/s^2] braking envelope
    ap.add_argument("--accel", type=float, default=400.0)       # [mm/s^2] ramp-in slew
    ap.add_argument("--lead", type=float, default=0.20,
                    help="[s] measurement age + actuation delay -- the horizon "
                         "the stop decision extrapolates measured progress "
                         "over, at the measured rate")
    ap.add_argument("--outdir", default=str(_REPO_ROOT / "src" / "tests" / "bench" / "output"))
    args = ap.parse_args()

    # Trapezoid durations from area: D = peak * (T - ramp)  ->  T = D/peak + ramp
    leg_T = (args.leg * args.leg_scale) / args.speed + RAMP_LEG  # [s]
    arc = (math.pi / 2.0) * (TRACK / 2.0) * args.turn_scale     # [mm] per wheel, 90 deg pivot
    turn_T = arc / args.turn_speed + RAMP_TURN                  # [s]

    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol
    conn = SerialConnection(port=args.port)
    info = conn.connect()
    print(f"  connected: {args.port}  device="
          f"{(info.get('announcement') or {}).get('device_name')}")
    proto = NezhaProtocol(conn)
    log: "list[tuple]" = []
    rest_poses = []
    corners: "list[dict]" = []
    started_wall = time.time()
    started = time.monotonic()
    try:
        proto.tlmOn()
        time.sleep(1.5)
        proto.read_pending_binary_tlm_frames()
        pose, stats = settle_measure(proto, log)                # start pose
        rest_poses.append(pose)
        start_stats = stats
        h_start = rest_poses[0][2]
        for corner in range(4):
            if args.online:
                online_segment(proto, log, args.leg, +1, args.speed, args, "leg")
            else:
                stream_segment(proto, log, args.speed, args.speed, RAMP_LEG, leg_T)
            pose, leg_stats = settle_measure(proto, log)        # after leg
            rest_poses.append(pose)
            # CCW pivot: left backward, right forward
            if args.online:
                online_segment(proto, log, 90.0, +1, args.turn_speed, args, "turn")
            else:
                stream_segment(proto, log, -args.turn_speed, args.turn_speed,
                               RAMP_TURN, turn_T)
            pose, turn_stats = settle_measure(proto, log)       # after turn
            rest_poses.append(pose)
            # Close the CUMULATIVE heading target (not the per-turn one), so
            # leg drift and earlier turn residue are absorbed too.
            target = h_start + 90.0 * (corner + 1)              # [deg]
            rec = dict(corner=corner + 1, target=target,
                       heading_pre_trim=pose[2],
                       residual_pre_trim=target - pose[2],
                       raw_turn=pose[2] - rest_poses[-2][2],
                       rest_sd_after_turn=turn_stats["sd"],
                       rest_ptp_after_turn=turn_stats["ptp"],
                       rest_sd_after_leg=leg_stats["sd"],
                       nudges=[], trim_seconds=0.0)
            if args.trim:
                pose, nudges, elapsed = trim_to_heading(
                    proto, log, pose, target, args.trim_tol,
                    args.trim_max_nudges, args.turn_scale)
                rest_poses[-1] = pose
                rec["nudges"] = nudges
                rec["trim_seconds"] = elapsed
            rec["heading_post_trim"] = rest_poses[-1][2]
            rec["residual_post_trim"] = target - rest_poses[-1][2]
            rec["nudge_count"] = len(rec["nudges"])
            rec["converged"] = abs(rec["residual_post_trim"]) <= args.trim_tol
            corners.append(rec)
            turn_delta = rest_poses[-1][2] - rest_poses[-2][2]  # [deg]
            print(f"  corner {corner + 1}/4: heading {rest_poses[-1][2]:7.1f} "
                  f"(turn {turn_delta:+6.1f})  x={rest_poses[-1][0]:7.1f} "
                  f"y={rest_poses[-1][1]:7.1f}  resid "
                  f"{rec['residual_pre_trim']:+5.2f} -> "
                  f"{rec['residual_post_trim']:+5.2f} in "
                  f"{rec['nudge_count']} nudge(s) / {rec['trim_seconds']:.1f}s"
                  + ("" if rec["converged"] else "  [NOT CONVERGED]"))
    finally:
        try:
            proto.estop()
            proto.tlmOff()
        except Exception:
            pass
        conn.disconnect()

    x0, y0, h0 = rest_poses[0]
    x1, y1, h1 = rest_poses[-1]
    closure = math.hypot(x1 - x0, y1 - y0)                      # [mm]
    heading_sweep = h1 - h0                                     # [deg] target +360
    turn_deltas = [rest_poses[2 * i + 2][2] - rest_poses[2 * i + 1][2]
                   for i in range(4)]
    print(f"\n  CLOSURE {closure:.1f} mm   heading sweep {heading_sweep:+.1f} deg "
          f"(target +360)   turns: "
          f"{' '.join(f'{d:+.1f}' for d in turn_deltas)}")

    # Per-segment truth from the same nine REST poses the closure uses:
    # leg n spans rest_poses[2n] -> rest_poses[2n+1], turn n spans
    # rest_poses[2n+1] -> rest_poses[2n+2]. Reporting only -- nothing below
    # feeds back into the motion.
    seg_lengths = [math.hypot(rest_poses[2 * i + 1][0] - rest_poses[2 * i][0],
                              rest_poses[2 * i + 1][1] - rest_poses[2 * i][1])
                   for i in range(4)]                           # [mm]
    leg_heading_drift = [rest_poses[2 * i + 1][2] - rest_poses[2 * i][2]
                         for i in range(4)]                     # [deg]
    print("  per-leg length [mm]: "
          + "  ".join(f"{d:6.1f}" for d in seg_lengths)
          + f"   (target {args.leg:.0f})")
    print("  per-turn angle [deg]: "
          + "  ".join(f"{d:+6.1f}" for d in turn_deltas) + "   (target +90)")
    print("  in-leg heading drift [deg]: "
          + "  ".join(f"{d:+6.1f}" for d in leg_heading_drift))

    wall = time.monotonic() - started
    nudge_total = sum(c["nudge_count"] for c in corners)
    trim_total = sum(c["trim_seconds"] for c in corners)
    resid_post = [c["residual_post_trim"] for c in corners]
    print(f"  cumulative residual after trim [deg]: "
          + "  ".join(f"{r:+5.2f}" for r in resid_post)
          + f"   mean|r| {sum(abs(r) for r in resid_post) / 4.0:.2f}")
    print(f"  trim: {nudge_total} nudges / {trim_total:.1f}s over 4 corners; "
          f"tour wall {wall:.1f}s; rest-heading sd at start "
          f"{start_stats['sd']:.3f} deg")

    import csv as _csv
    import json as _json
    pathlib.Path(args.outdir).mkdir(parents=True, exist_ok=True)
    results = pathlib.Path(args.outdir) / "tour_results.csv"
    new = not results.exists()
    with open(results, "a", newline="") as fh:
        w = _csv.writer(fh)
        if new:
            w.writerow(["label", "turn_scale", "leg_scale", "trim", "closure",
                        "heading_sweep", "l1", "l2", "l3", "l4",
                        "t1", "t2", "t3", "t4"])
        w.writerow([args.label, args.turn_scale, args.leg_scale, int(args.trim),
                    f"{closure:.1f}", f"{heading_sweep:+.1f}",
                    *[f"{d:.1f}" for d in seg_lengths],
                    *[f"{d:+.1f}" for d in turn_deltas]])

    # Per-run record: everything the trim-tolerance question needs, so an
    # arm can be re-analyzed without re-running the robot.
    run = dict(label=args.label, arm="wheels", started_wall=started_wall,
               started_iso=time.strftime("%Y-%m-%dT%H:%M:%S",
                                         time.localtime(started_wall)),
               trim=bool(args.trim), trim_tol=args.trim_tol,
               trim_max_nudges=args.trim_max_nudges,
               turn_scale=args.turn_scale, leg_scale=args.leg_scale,
               speed=args.speed, turn_speed=args.turn_speed,
               closure=closure, heading_sweep=heading_sweep,
               wall=wall, nudge_total=nudge_total, trim_seconds=trim_total,
               seg_lengths=seg_lengths, turn_deltas=turn_deltas,
               leg_heading_drift=leg_heading_drift,
               rest_poses=[list(p) for p in rest_poses],
               start_rest_stats=start_stats, corners=corners)
    jpath = pathlib.Path(args.outdir) / f"trimtol_{args.label}.json"
    jpath.write_text(_json.dumps(run, indent=1))
    print(f"  run record: {jpath}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax_path, ax_v) = plt.subplots(1, 2, figsize=(15.0, 7.0))
    xs = [r[1] for r in log]
    ys = [r[2] for r in log]
    ax_path.plot(xs, ys, lw=1.5, color="#2a78d6")
    ax_path.plot([r[0] for r in rest_poses], [r[1] for r in rest_poses], "o",
                 color="#eb6834", ms=6, label="rest poses")
    ax_path.plot(x0, y0, "s", color="0.2", ms=9, label="start")
    ax_path.annotate(f"closure {closure:.1f} mm", xy=(x1, y1), fontsize=11,
                     xytext=(10, 10), textcoords="offset points")
    ax_path.set_aspect("equal")
    ax_path.set_xlabel("x  [mm]")
    ax_path.set_ylabel("y  [mm]")
    ax_path.set_title(f"encoder-odometry path -- closure {closure:.1f} mm, "
                      f"heading {heading_sweep:+.1f} deg")
    ax_path.grid(alpha=0.3)
    ax_path.legend(fontsize=9)

    t0 = log[0][0] if log else 0
    ts = [(r[0] - t0) / 1000.0 for r in log]
    ax_v.plot(ts, [r[4] for r in log], lw=1.2, label="left wheel", color="#2a78d6")
    ax_v.plot(ts, [r[5] for r in log], lw=1.2, label="right wheel", color="#eb6834")
    ax_v.set_xlabel("time  [s]")
    ax_v.set_ylabel("wheel speed  [mm/s]")
    ax_v.set_title("streamed WHEELS trapezoids -- 4 legs + 4 CCW pivots")
    ax_v.grid(alpha=0.3)
    ax_v.legend(fontsize=9)

    fig.suptitle("SQUARE TOUR, wheels-only (no planner) -- host-shaped "
                 "trapezoids as re-armed WHEELS commands", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    outdir = pathlib.Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    out = outdir / f"wheels_square_tour_{args.label}.png"
    fig.savefig(out, dpi=130)
    print(f"  chart: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
