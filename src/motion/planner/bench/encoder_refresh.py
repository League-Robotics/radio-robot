#!/usr/bin/env python3
"""encoder_refresh.py -- characterize the REAL encoder refresh interval
(motion-planner issue §7 item 3).

ANSWERED 2026-07-26 by a dedicated bench firmware
(src/tests/firmware/encoder_rate/): the register is LIVE at <=16 ms --
the "~80 ms" folklore was false (historical staleness was
interposed-traffic sample invalidation under the pre-118 loop schedule).
Authoritative write-up: docs/design/encoder-refresh-characterization.md.
This script remains useful as a THROUGH-THE-LOOP confirmation: on the
current interleaved schedule, expect a fresh sample every 50 ms cycle.
`Motion::WheelChannel` still gates its EMA on `sampleTime` changing
(re-feeding the same sample would silently re-weight old data), and the
noise tier's every-N-th-cycle staleness is now a degraded-mode test.

What it measures, per wheel: the distribution of intervals between
CHANGES in the reported encoder sample -- both the robot-clock stamp
(`EncoderReading.time`, which is the firmware's own "when I collected
this") and the reported position, since a register that re-stamps without
moving is a different fact from one that does not re-stamp at all.

The robot is on a stand with its wheels off the ground
(`.claude/rules/hardware-bench-testing.md`), so driving freely at several
speeds is safe. Speed matters: an encoder that reports on a fixed timer
has a speed-independent interval, while one that reports per N counts has
an interval inversely proportional to speed. Which of those it is decides
whether "staleness" is a constant or a function of velocity.

Usage:

    # capture from the robot and analyze in one shot
    uv run python src/motion/planner/bench/encoder_refresh.py \\
        --port /dev/cu.usbmodem2121102

    # analyze a capture taken earlier (this script's CSV, or tlm_log.py's)
    uv run python src/motion/planner/bench/encoder_refresh.py \\
        --analyze src/motion/planner/bench/out/encoder_refresh.csv

Outputs, both under `src/motion/planner/bench/out/`:
  * `encoder_refresh.csv`  -- one row per telemetry frame, raw
  * `encoder_refresh.md`   -- the histogram + the numbers to quote

stdlib + robot_radio only; no numpy.
"""
from __future__ import annotations

import argparse
import csv
import pathlib
import statistics
import sys
import time

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
DEFAULT_PORT = "/dev/cu.usbmodem2121102"
DEFAULT_OUT = pathlib.Path(__file__).resolve().parent / "out"

# Drive each of these for `--dwell` seconds. Slow/medium/fast: enough
# spread to separate a fixed-timer refresh from a per-count one.
DEFAULT_SPEEDS = (60.0, 150.0, 300.0)  # [mm/s]

CSV_FIELDNAMES = ("host_time", "commanded", "robot_time",
                  "left_position", "left_velocity", "left_time",
                  "right_position", "right_velocity", "right_time")

ACK_TIMEOUT = 500  # [ms]


# ---------------------------------------------------------------------------
# analysis -- pure, CSV rows in / report text out (no I/O, no hardware)
# ---------------------------------------------------------------------------

def refreshIntervals(rows: "list[dict]", wheel: str) -> "dict[str, list[float]]":
    """Intervals [ms] between successive CHANGES, per commanded speed.

    Two independent series per wheel, because they answer different
    questions: `stamp` is the gap between distinct `EncoderReading.time`
    values (how often the firmware collects a new sample) and `position`
    is the gap between distinct reported positions (how often the register
    underneath it actually moves). They coincide only if every collect
    sees a changed count.
    """
    byStamp: "dict[str, list[float]]" = {}
    byPosition: "dict[str, list[float]]" = {}
    lastStamp: "dict[str, tuple[float, float]]" = {}   # speed -> (value, robot_time)
    lastPosition: "dict[str, tuple[float, float]]" = {}

    for row in rows:
        speed = row["commanded"]
        robotTime = _asFloat(row["robot_time"])
        stamp = _asFloat(row[f"{wheel}_time"])
        position = _asFloat(row[f"{wheel}_position"])
        if robotTime is None or stamp is None or position is None:
            continue
        previous = lastStamp.get(speed)
        if previous is not None and stamp != previous[0]:
            byStamp.setdefault(speed, []).append(stamp - previous[0])
        if previous is None or stamp != previous[0]:
            lastStamp[speed] = (stamp, robotTime)
        previous = lastPosition.get(speed)
        if previous is not None and position != previous[0]:
            byPosition.setdefault(speed, []).append(stamp - previous[1])
        if previous is None or position != previous[0]:
            lastPosition[speed] = (position, stamp)

    return {"stamp": byStamp, "position": byPosition}


def _asFloat(value) -> "float | None":
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def histogram(intervals: "list[float]", binWidth: float = 10.0) -> "list[str]":  # [ms]
    """A fixed-width text histogram -- the point of the exercise is the
    SHAPE (one spike? two? a tail?), which a mean alone hides."""
    if not intervals:
        return ["    (no samples)"]
    lines = []
    counts: "dict[int, int]" = {}
    for value in intervals:
        counts[int(value // binWidth)] = counts.get(int(value // binWidth), 0) + 1
    peak = max(counts.values())
    for index in range(min(counts), max(counts) + 1):
        count = counts.get(index, 0)
        bar = "#" * int(round(40.0 * count / peak))
        lines.append(f"    {index * binWidth:6.0f}-{(index + 1) * binWidth:<6.0f} "
                     f"{count:5d} {bar}")
    return lines


def summarize(intervals: "list[float]") -> str:
    if not intervals:
        return "no samples"
    ordered = sorted(intervals)
    def percentile(fraction: float) -> float:
        return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]
    return (f"n={len(ordered)} min={ordered[0]:.1f} p50={percentile(0.50):.1f} "
            f"p90={percentile(0.90):.1f} p99={percentile(0.99):.1f} "
            f"max={ordered[-1]:.1f} mean={statistics.fmean(ordered):.1f} "
            f"stdev={statistics.pstdev(ordered):.1f}")


def report(rows: "list[dict]") -> str:
    """The markdown note. Everything a reader needs to set the EMA weight
    and the noise model without re-running the capture."""
    out = ["# Encoder refresh interval -- bench measurement",
           "",
           "Generated by `src/motion/planner/bench/encoder_refresh.py`",
           "(motion-planner issue §7 item 3). All intervals in [ms], robot",
           "clock. `stamp` = gap between distinct `EncoderReading.time`",
           "values; `position` = gap between distinct reported positions.",
           "",
           f"Frames analyzed: {len(rows)}",
           ""]
    for wheel in ("left", "right"):
        series = refreshIntervals(rows, wheel)
        out.append(f"## {wheel} wheel")
        out.append("")
        for kind in ("stamp", "position"):
            out.append(f"### {kind} changes")
            out.append("")
            bySpeed = series[kind]
            if not bySpeed:
                out.append("    (no changes observed)")
                out.append("")
                continue
            for speed in sorted(bySpeed, key=lambda s: _asFloat(s) or 0.0):
                out.append(f"* commanded {speed} mm/s: {summarize(bySpeed[speed])}")
                out.append("")
                out.append("```")
                out.extend(histogram(bySpeed[speed]))
                out.append("```")
                out.append("")
    out += ["## What to do with this",
            "",
            "* If the p50 is flat across speeds, the refresh is a fixed",
            "  timer: `sampleDivisor = round(p50 / controlPeriod)` in the",
            "  planner's noise tier, and the EMA weight can be set purely",
            "  from the velocity noise.",
            "* If it scales as 1/speed, the refresh is per-count: staleness",
            "  is worst at LOW speed, which is exactly where a Move lands,",
            "  so the settle-confirm window must cover the slow-speed p99.",
            "* The p99, not the p50, is what `PlannerLimits.settleWindow`",
            "  and any staleness guard have to tolerate.",
            ""]
    return "\n".join(out)


# ---------------------------------------------------------------------------
# capture -- the hardware half
# ---------------------------------------------------------------------------

def capture(port: str, speeds: "tuple[float, ...]", dwell: float,
            csvPath: pathlib.Path) -> "list[dict]":  # [s]
    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol

    conn = SerialConnection(port=port)  # mode=None -> auto-detect direct vs relay
    info = conn.connect()
    if info.get("status") != "connected":
        raise SystemExit(f"connect failed: {info}")
    proto = NezhaProtocol(conn)
    print(f"connected: mode={info.get('mode')}")

    rows: "list[dict]" = []
    try:
        for speed in speeds:
            proto.read_pending_binary_tlm_frames()  # drop anything stale
            window = dwell * 1000.0  # [ms]
            corr = proto.move_twist(speed, 0.0, 0.0, stop_time=window,
                                    timeout=window + 1000.0, replace=True)
            if proto.wait_for_ack(corr, timeout=ACK_TIMEOUT) is None:
                raise SystemExit(f"no enqueue ack for {speed} mm/s")
            print(f"  driving {speed} mm/s for {dwell:.0f} s ...")
            deadline = time.monotonic() + dwell
            while time.monotonic() < deadline:
                for frame in proto.read_pending_binary_tlm_frames():
                    rows.append(_frameRow(frame, speed))
                time.sleep(0.005)
            proto.wait_for_ack(proto.stop(), timeout=ACK_TIMEOUT)
            time.sleep(0.3)  # let the wheels come to rest before the next leg
    finally:
        try:
            proto.stop()
        finally:
            conn.close()

    csvPath.parent.mkdir(parents=True, exist_ok=True)
    with csvPath.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {len(rows)} rows to {csvPath}")
    return rows


def _frameRow(frame, speed: float) -> dict:
    left, right = frame.enc_left, frame.enc_right
    return {
        "host_time": f"{time.monotonic():.6f}",
        "commanded": speed,
        "robot_time": frame.t,
        "left_position": left.position if left is not None else None,
        "left_velocity": left.velocity if left is not None else None,
        "left_time": left.time if left is not None else None,
        "right_position": right.position if right is not None else None,
        "right_velocity": right.velocity if right is not None else None,
        "right_time": right.time if right is not None else None,
    }


def loadRows(path: pathlib.Path) -> "list[dict]":
    """Read this script's CSV, or a `tlm_log.py` CSV (whose columns are
    named `enc_left_*`/`now` instead) -- so an existing capture can be
    analyzed without re-running the robot."""
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f"{path} is empty")
    if "left_time" in rows[0]:
        return rows
    if "enc_left_time" not in rows[0]:
        raise SystemExit(f"{path}: not an encoder_refresh.py or tlm_log.py CSV")
    return [{"host_time": row.get("now"),
             "commanded": row.get("twist_v_x") or "unknown",
             "robot_time": row.get("now"),
             "left_position": row.get("enc_left_position"),
             "left_velocity": row.get("enc_left_velocity"),
             "left_time": row.get("enc_left_time"),
             "right_position": row.get("enc_right_position"),
             "right_velocity": row.get("enc_right_velocity"),
             "right_time": row.get("enc_right_time")} for row in rows]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--analyze", type=pathlib.Path, default=None,
                        help="skip the hardware capture; analyze this CSV")
    parser.add_argument("--dwell", type=float, default=60.0,  # [s]
                        help="seconds to hold each speed (issue asks >= 60)")
    parser.add_argument("--speeds", type=float, nargs="+",
                        default=list(DEFAULT_SPEEDS))  # [mm/s]
    parser.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    csvPath = args.out / "encoder_refresh.csv"
    if args.analyze is not None:
        rows = loadRows(args.analyze)
    else:
        rows = capture(args.port, tuple(args.speeds), args.dwell, csvPath)

    text = report(rows)
    args.out.mkdir(parents=True, exist_ok=True)
    notePath = args.out / "encoder_refresh.md"
    notePath.write_text(text)
    print(text)
    print(f"\nwrote {notePath}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
