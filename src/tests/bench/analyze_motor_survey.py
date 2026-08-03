#!/usr/bin/env python3
"""analyze_motor_survey.py -- compare motors across a `motor_survey.py` run.

Turns the survey's `summary.csv` into the comparison the survey exists for:
which motors are strong, which are weak, how wide the spread is, and whether
the spread is big enough that building robots from *matched* pairs would buy
anything.

    uv run python src/tests/bench/analyze_motor_survey.py \\
        --survey src/tests/bench/output/motor_survey_YYYYMMDD

Two metrics, deliberately separated:

  **capability** -- plateau tracking, the fraction of commanded speed a wheel
  actually reaches mid-profile, at its BEST observed run. This is the honest
  per-motor number: it is measured away from the profile's ends, so it is not
  contaminated by start/stop behaviour, and taking the best run removes the
  cold-start convergence transient that otherwise reads as a weak motor.

  **match** -- the left/right gap WITHIN one robot. A robot whose wheels are
  both weak drives straight and short (correctable with one scale factor); a
  robot whose wheels disagree pulls to one side, which no scalar fixes. For
  building robots out of a bin of motors, the gap matters more than the mean.

Absolute millimetres are NOT compared across boards: every board runs one
baked `mm_per_wheel_deg`, so a board with different wheels reports different
absolute travel for identical behaviour. Ratios are the comparable quantity.
"""
from __future__ import annotations

import argparse
import csv
import pathlib
import sys
from collections import defaultdict
from statistics import mean, pstdev

COLOR_LEFT = "#2a78d6"
COLOR_RIGHT = "#eb6834"
COLOR_INK = "#52514e"
COLOR_GRID = "#d9d8d3"


def load(path: pathlib.Path) -> "list[dict]":
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def capability(rows: "list[dict]") -> "dict[tuple[str, str], float]":
    """(board, wheel) -> best observed plateau tracking across all runs and
    both profiles. Best, not mean: a cold run measures the controller's
    convergence state, not the motor."""
    best: "dict[tuple[str, str], float]" = {}
    for row in rows:
        try:
            value = float(row["plateau_tracking"])
        except (ValueError, KeyError):
            continue
        if value != value:
            continue
        key = (row["board"], row["wheel"])
        if key not in best or value > best[key]:
            best[key] = value
    return best


def cold_vs_warm(rows: "list[dict]") -> "dict[str, tuple[float, float]]":
    """board -> (cold, best-warm) mean plateau tracking over both wheels."""
    buckets: "dict[tuple[str, str], list[float]]" = defaultdict(list)
    for row in rows:
        try:
            value = float(row["plateau_tracking"])
        except (ValueError, KeyError):
            continue
        if value != value:
            continue
        phase = "cold" if row["run"] == "cold" else "warm"
        buckets[(row["board"], phase)].append(value)
    out: "dict[str, tuple[float, float]]" = {}
    for board in sorted({row["board"] for row in rows}):
        cold = buckets.get((board, "cold"), [])
        warm = buckets.get((board, "warm"), [])
        if cold and warm:
            out[board] = (mean(cold), mean(warm))
    return out


def render(rows: "list[dict]", path: pathlib.Path) -> "list[str]":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    best = capability(rows)
    boards = sorted({board for board, _ in best})
    if not boards:
        return ["no usable rows"]

    # Order boards by their WEAKER wheel: the wheel that limits the robot.
    def weakest(board: str) -> float:
        return min(best.get((board, "left"), float("nan")),
                   best.get((board, "right"), float("nan")))

    boards = sorted(boards, key=weakest, reverse=True)
    left = [best.get((board, "left"), float("nan")) * 100 for board in boards]
    right = [best.get((board, "right"), float("nan")) * 100 for board in boards]
    gaps = [abs(l - r) for l, r in zip(left, right)]

    every = left + right
    spread = max(every) - min(every)
    # Dot plots, not bars: every motor sits in a narrow 85-100% band, and a
    # bar (whose LENGTH encodes the value, so it must start at zero) renders
    # a 14-point spread as four visually identical bars. A dot encodes
    # POSITION, so a non-zero baseline is legitimate and the spread is
    # readable -- which is the entire question this chart exists to answer.
    figure, axes = plt.subplots(1, 2, figsize=(15.5, 5.9),
                                gridspec_kw={"width_ratios": [1.32, 1]})
    figure.suptitle(
        f"Motor capability across {len(boards)} robots — best plateau speed reached, per wheel\n"
        f"8 motors span {min(every):.0f}–{max(every):.0f}% of commanded ({spread:.0f} points); "
        f"worst within-robot L/R mismatch {max(gaps):.1f} points",
        fontsize=14)

    low = min(every) - 3.0
    high = max(102.0, max(every) + 2.5)

    ax = axes[0]
    positions = list(range(len(boards)))
    for p, (l, r) in enumerate(zip(left, right)):
        # The connecting segment IS the L/R gap -- no separate panel needed.
        ax.plot([l, r], [p, p], color=COLOR_INK, linewidth=2.4, alpha=0.35,
                solid_capstyle="round", zorder=2)
        ax.annotate(f"gap {abs(l - r):.1f}", xy=(max(l, r), p), xytext=(10, 0),
                    textcoords="offset points", va="center", fontsize=8.5,
                    color=COLOR_INK)
    ax.scatter(left, positions, s=105, color=COLOR_LEFT, label="left wheel",
               zorder=4, edgecolors="white", linewidths=1.4)
    ax.scatter(right, positions, s=105, color=COLOR_RIGHT, label="right wheel",
               zorder=4, edgecolors="white", linewidths=1.4)
    ax.axvline(100, color=COLOR_INK, linestyle=(0, (5, 3)), linewidth=1.6, zorder=1)
    ax.annotate("commanded", xy=(100, len(boards) - 0.35), xytext=(5, 0),
                textcoords="offset points", fontsize=9, color=COLOR_INK)
    ax.set_yticks(positions)
    ax.set_yticklabels(boards)
    ax.set_ylim(-0.7, len(boards) - 0.15)
    ax.set_xlabel("plateau speed reached  [% of commanded]")
    ax.set_xlim(low, high)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=2,
              frameon=False, fontsize=9)
    ax.set_title("per robot — ranked by its weaker wheel", fontsize=11)

    ax = axes[1]
    motors = sorted(((f"{board} {side[0].upper()}",
                      best[(board, side)] * 100,
                      COLOR_LEFT if side == "left" else COLOR_RIGHT)
                     for (board, side) in best),
                    key=lambda item: item[1])
    motor_positions = list(range(len(motors)))
    for p, (_, value, color) in zip(motor_positions, motors):
        ax.plot([low, value], [p, p], color=COLOR_GRID, linewidth=1.2, zorder=1)
        ax.scatter([value], [p], s=95, color=color, zorder=3,
                   edgecolors="white", linewidths=1.3)
        ax.annotate(f"{value:.1f}%", xy=(value, p), xytext=(13, 0),
                    textcoords="offset points", va="center", fontsize=9,
                    color=COLOR_INK)
    ax.axvline(100, color=COLOR_INK, linestyle=(0, (5, 3)), linewidth=1.6, zorder=1)
    ax.set_yticks(motor_positions)
    ax.set_yticklabels([name for name, _, _ in motors], fontsize=9)
    ax.set_ylim(-0.7, len(motors) - 0.3)
    ax.set_xlabel("plateau speed reached  [% of commanded]")
    ax.set_xlim(low, high)
    ax.set_title("every motor, ranked — the range you are building from", fontsize=11)

    for ax in axes:
        ax.grid(True, axis="x", color=COLOR_GRID, linewidth=0.7, alpha=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)

    figure.tight_layout(rect=(0, 0, 1, 0.87))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=140, facecolor="white")
    plt.close(figure)

    report = [
        f"{'board':>18}  {'left':>7}  {'right':>7}  {'gap':>6}  {'weaker':>7}",
    ]
    for board, l, r, gap in zip(boards, left, right, gaps):
        report.append(f"{board:>18}  {l:>6.1f}%  {r:>6.1f}%  {gap:>5.1f}  {min(l, r):>6.1f}%")
    every_motor = [(board, side, value * 100) for (board, side), value in best.items()]
    values = [v for _, _, v in every_motor]
    report.append("")
    report.append(f"  {len(values)} motors: mean {mean(values):.1f}%  sd {pstdev(values):.1f}  "
                  f"range {min(values):.1f}–{max(values):.1f}%  spread {max(values) - min(values):.1f} points")
    strongest = max(every_motor, key=lambda item: item[2])
    weakest_motor = min(every_motor, key=lambda item: item[2])
    report.append(f"  strongest: {strongest[0]} {strongest[1]} {strongest[2]:.1f}%")
    report.append(f"  weakest:   {weakest_motor[0]} {weakest_motor[1]} {weakest_motor[2]:.1f}%")
    return report


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--survey", required=True, help="survey directory (holds summary.csv)")
    p.add_argument("--chart", default=None)
    args = p.parse_args()

    survey = pathlib.Path(args.survey)
    summary = survey / "summary.csv"
    if not summary.exists():
        print(f"ERROR: no summary.csv in {survey}")
        return 2
    rows = load(summary)
    if not rows:
        print("ERROR: summary.csv is empty")
        return 2

    chart = pathlib.Path(args.chart) if args.chart else survey / "motor_comparison.png"
    report = render(rows, chart)

    print("\n=== capability: best plateau speed reached, per wheel ===")
    for line in report:
        print(line)

    transients = cold_vs_warm(rows)
    if transients:
        print("\n=== cold-start deficit (mean plateau tracking, both wheels) ===")
        print(f"  {'board':>18}  {'cold':>7}  {'warm':>7}  {'gain':>7}")
        for board, (cold, warm) in sorted(transients.items(),
                                          key=lambda kv: kv[1][1] - kv[1][0], reverse=True):
            print(f"  {board:>18}  {cold * 100:>6.1f}%  {warm * 100:>6.1f}%  "
                  f"{(warm - cold) * 100:>+6.1f}")

    print(f"\n  chart: {chart}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
