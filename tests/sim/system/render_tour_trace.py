#!/usr/bin/env python3
"""render_tour_trace.py — render a Tour 1 / Tour 2 ground-truth trace PNG for
a human to eyeball (ticket 086-004's "rendered-tour check").

Follows the established charting precedent this ticket was told to reuse
(``tests/bench/velocity_chart.py``'s dark-background dashboard for style,
``tests/playfield/plot_square.py``'s Agg-backend / ``fig.savefig()`` CLI-tool
shape for structure) rather than inventing a new pattern: matplotlib, an
``Agg`` backend (headless-safe -- no display server needed), argparse, one
PNG per run under ``tests/sim/system/out/`` (git-ignored build artifact,
same rationale as ``tests/bench/out/``).

Drives the SAME canonical leg lists the GUI's tour buttons drive
(``robot_radio.testgui.commands.TOUR_1``/``TOUR_2``) through
``libfirmware_host`` directly (``tests/_infra/sim/firmware.py``'s ``Sim``),
exactly like ``tests/sim/system/test_tour_geometry.py``'s per-leg test --
this script is the informational/visual counterpart to that test's
assertions, not a second source of truth for the geometry.

Usage::

    uv run python tests/sim/system/render_tour_trace.py
    uv run python tests/sim/system/render_tour_trace.py --tour "Tour 2" \\
        --out tests/sim/system/out/tour2_trace.png
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve().parent
_REPO_ROOT = _HERE.parent.parent.parent
_SIM_INFRA_DIR = _REPO_ROOT / "tests" / "_infra" / "sim"
_DEFAULT_OUT_DIR = _HERE / "out"

if str(_SIM_INFRA_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_INFRA_DIR))

_TICK_STEP = 24    # [ms] matches the project's own control-period convention
_LEG_BUDGET = 6000   # [ms] ample for the longest single leg + settle
_SETTLE_WINDOW = 800  # [ms] after "EVT done <verb>" -- matches test_tour_geometry.py


def _drive_tour_collecting_trace(steps: list[str]) -> tuple[list[float], list[float], dict]:
    """Drive every leg of ``steps`` to completion, sampling ground-truth
    (x, y) every tick. Returns (xs_mm, ys_mm, summary) where ``summary`` has
    per-leg completion info useful for the plot title/legend.
    """
    from firmware import Sim  # noqa: PLC0415 -- sys.path set up above

    xs: list[float] = []
    ys: list[float] = []
    leg_boundaries: list[int] = []   # sample index where each leg completed

    with Sim() as sim:
        sim.command("DEV WD 60000")
        x0, y0, _h0 = sim.true_pose()
        xs.append(x0)
        ys.append(y0)

        for cmd in steps:
            verb = cmd.split()[0]
            reply = sim.command(cmd)
            assert reply.startswith("OK"), f"{cmd!r} rejected: {reply!r}"

            elapsed = 0
            done_at: int | None = None
            while elapsed < _LEG_BUDGET:
                sim.tick_for(_TICK_STEP, step=_TICK_STEP)
                elapsed += _TICK_STEP
                x, y, _h = sim.true_pose()
                xs.append(x)
                ys.append(y)
                evts = sim.get_async_evts()
                if done_at is None and f"EVT done {verb}" in evts:
                    done_at = elapsed
                if done_at is not None and elapsed >= done_at + _SETTLE_WINDOW:
                    break
            assert done_at is not None, (
                f"{cmd!r} never completed within a {_LEG_BUDGET}ms budget"
            )
            leg_boundaries.append(len(xs) - 1)

        xf, yf, hf = sim.true_pose()

    summary = {
        "final_xy": (xf, yf),
        "final_h_deg": math.degrees(hf),
        "dist_from_origin": math.hypot(xf, yf),
        "leg_boundaries": leg_boundaries,
        "n_legs": len(steps),
    }
    return xs, ys, summary


def render(tour_name: str, steps: list[str], out_path: pathlib.Path) -> dict:
    xs, ys, summary = _drive_tour_collecting_trace(steps)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(xs, ys, color="tab:red", lw=1.6, label="Sim ground truth (true_pose)")
    ax.plot(xs[0], ys[0], "ko", ms=10, label="start (origin)", zorder=5)
    ax.plot(xs[-1], ys[-1], "s", color="tab:blue", ms=10, label="end", zorder=5)

    # Mark each leg's completion point so a human can see the tour step by
    # step, not just the overall shape.
    lb = summary["leg_boundaries"]
    ax.plot([xs[i] for i in lb], [ys[i] for i in lb], "o", color="grey",
             ms=4, alpha=0.7, zorder=4, label="leg completion")

    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    ax.set_title(
        f"{tour_name} — sim ground-truth trace ({summary['n_legs']} legs)\n"
        f"end ({summary['final_xy'][0]:.0f}, {summary['final_xy'][1]:.0f}) mm, "
        f"h={summary['final_h_deg']:.1f} deg, "
        f"{summary['dist_from_origin']:.0f} mm from origin"
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f"saved {out_path}  (final dist-from-origin "
          f"{summary['dist_from_origin']:.1f} mm, final heading "
          f"{summary['final_h_deg']:.1f} deg)")
    return summary


def main() -> int:
    from robot_radio.testgui.commands import TOURS

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tour", choices=sorted(TOURS), default=None,
                     help="Render only this tour (default: render all tours).")
    ap.add_argument("--out", default=None,
                     help="Output PNG path (only valid with --tour). Default: "
                          f"{_DEFAULT_OUT_DIR}/<tour>_trace.png")
    args = ap.parse_args()

    tours = {args.tour: TOURS[args.tour]} if args.tour else TOURS
    if args.out and len(tours) != 1:
        ap.error("--out requires --tour (ambiguous output path for multiple tours)")

    for name, steps in tours.items():
        slug = name.lower().replace(" ", "")
        out_path = pathlib.Path(args.out) if args.out else _DEFAULT_OUT_DIR / f"{slug}_trace.png"
        render(name, steps, out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
