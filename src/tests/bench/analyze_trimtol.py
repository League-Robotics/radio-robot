#!/usr/bin/env python3
"""analyze_trimtol.py -- read the per-run records wheels_square_tour.py and
planner_square_tour.py write (``trimtol_<label>.json``) and produce the
trim-tolerance comparison: one table across all arms, a per-run CSV, and the
four charts that carry the finding.

The question the data answers: is square-tour closure governed by per-corner
cumulative-heading residual, and is the TRIM TOLERANCE the knob that sets it?

Usage:
    uv run python src/tests/bench/analyze_trimtol.py [--outdir DIR]
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import pathlib
import statistics as st

# dataviz categorical slots 1-2 (validated all-pairs, light + dark)
C_WHEELS = "#2a78d6"
C_PLANNER = "#eb6834"
INK = "#0b0b0b"
MUTED = "#52514e"

# Arm definition: label prefix -> (display name, path family, tol or None)
ARMS = [
    ("wnt_", "wheels\nno trim", "wheels", None),
    ("t03late_", "wheels\ntrim 0.3 (late)", "wheels", 0.3),
    ("t10_", "wheels\ntrim 1.0", "wheels", 1.0),
    ("t05_", "wheels\ntrim 0.5", "wheels", 0.5),
    ("t03_", "wheels\ntrim 0.3", "wheels", 0.3),
    ("pship_", "planner\nshipped", "planner", None),
    ("pnt_", "planner\nseq no trim", "planner", None),
    ("p10_", "planner\nseq 1.0", "planner", 1.0),
    ("p03_", "planner\nseq 0.3", "planner", 0.3),
    ("wpost_", "wheels 1.0\npost-swap", "wheels", 1.0),
    ("ppost_", "planner 1.0\npost-swap", "planner", 1.0),
]

# Runs quarantined from every aggregate. The session ended in a drivetrain
# fault: from 22:22 onward the SAME commanded leg and pivot delivered
# progressively less travel (mean leg 496.2 -> 494.5 -> 425.3 -> 55.3 -> 0.0 mm;
# mean raw pivot 86.9 -> 84.8 -> 71.6 -> 2.3 -> 0.0 deg) against 498-501 mm and
# 89.8-90.5 deg for every run before it, ending with the I2C safety net and the
# wedge latch both set (flags 0x00d8) and tovez dropping off USB altogether.
# These runs measure the fault, not the arm. The wheels-path no-trim control is
# the casualty: it is the arm that happened to be scheduled when the drivetrain
# went. It did NOT differ in flags -- turn_scale 1.0363 / leg_scale 0.985 are
# recorded in each of its own run records, identical to every other wheels arm.
QUARANTINE = {
    "wnt_": "drivetrain fault onset -- pivots 86.9->84.8 deg and legs "
            "496->494 mm against 90.0 deg / 499 mm all session",
    "t03late_": "drivetrain failed -- legs 425/55/0 mm, pivots 72/2/0 deg, "
                "I2C safety net + wedge latch set, then tovez left the USB bus",
}


def load(outdir: pathlib.Path):
    runs = []
    for path in sorted(glob.glob(str(outdir / "trimtol_*.json"))):
        d = json.load(open(path))
        label = d["label"]
        for prefix, name, family, tol in ARMS:
            if label.startswith(prefix):
                d["arm_name"] = name.replace("\n", " ")
                d["arm_key"] = prefix
                d["family"] = family
                d["arm_tol"] = tol
                d["quarantine"] = QUARANTINE.get(prefix, "")
                runs.append(d)
                break
    return runs


def residual_stats(d):
    r = [c["residual_post_trim"] for c in d["corners"]] if d["corners"] else []
    if not r:
        return float("nan"), float("nan")
    return (sum(abs(x) for x in r) / len(r),
            500.0 * math.sqrt(sum(math.radians(x) ** 2 for x in r)))


def corr(a, b):
    pairs = [(x, y) for x, y in zip(a, b)
             if not (math.isnan(x) or math.isnan(y))]
    a = [x for x, _ in pairs]
    b = [y for _, y in pairs]
    ma, mb = st.mean(a), st.mean(b)
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    den = math.sqrt(sum((x - ma) ** 2 for x in a) * sum((y - mb) ** 2 for y in b))
    return num / den if den else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", default=str(pathlib.Path(__file__).resolve()
                                            .parent / "output"))
    args = ap.parse_args()
    outdir = pathlib.Path(args.outdir)
    runs = load(outdir)
    if not runs:
        print("no trimtol_*.json records found")
        return 1

    # ---- per-run CSV -------------------------------------------------
    rows = []
    for d in runs:
        mean_r, pred = residual_stats(d)
        rows.append(dict(
            label=d["label"], arm=d["arm_name"], family=d["family"],
            started=d["started_iso"], trim=int(d["trim"]),
            trim_tol=d["trim_tol"] if d["trim"] else "",
            closure=round(d["closure"], 1),
            heading_sweep=round(d["heading_sweep"], 1),
            turn_sd=round(st.pstdev(d["turn_deltas"]), 2) if d["turn_deltas"] else "",
            leg_sd=round(st.pstdev(d["seg_lengths"]), 2) if d["seg_lengths"] else "",
            mean_abs_resid=round(mean_r, 2) if not math.isnan(mean_r) else "",
            pred_closure=round(pred, 1) if not math.isnan(pred) else "",
            nudges=d["nudge_total"], trim_s=round(d["trim_seconds"], 1),
            wall=round(d["wall"], 1),
            raw_turn_mean=round(st.mean([c["raw_turn"] for c in d["corners"]]), 2)
            if d["corners"] else "",
            mean_leg=round(st.mean(d["seg_lengths"]), 1) if d["seg_lengths"] else "",
            not_converged=sum(1 for c in d["corners"] if not c["converged"])
            if d["corners"] else "",
            valid=0 if d["quarantine"] else 1,
            quarantine_reason=d["quarantine"]))
    rows.sort(key=lambda r: r["started"])
    per_run = outdir / "trimtol_runs.csv"
    with open(per_run, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # ---- arm summary -------------------------------------------------
    valid = [d for d in runs if not d["quarantine"]]
    summary = []
    for prefix, name, family, tol in ARMS:
        g = [d for d in valid if d["arm_key"] == prefix]
        if not g:
            continue
        clo = [d["closure"] for d in g]
        mr = [residual_stats(d)[0] for d in g if d["corners"]]
        tsd = [st.pstdev(d["turn_deltas"]) for d in g if d["turn_deltas"]]
        summary.append(dict(
            arm=name.replace("\n", " "), n=len(g),
            closure_mean=round(st.mean(clo), 1),
            closure_sd=round(st.stdev(clo), 1) if len(clo) > 1 else 0.0,
            closure_min=round(min(clo), 1), closure_max=round(max(clo), 1),
            turn_sd=round(st.mean(tsd), 2) if tsd else "",
            mean_abs_resid=round(st.mean(mr), 2) if mr else "",
            nudges_per_corner=round(sum(d["nudge_total"] for d in g) / (4 * len(g)), 2),
            trim_s_per_corner=round(sum(d["trim_seconds"] for d in g) / (4 * len(g)), 2),
            tour_s=round(st.mean([d["wall"] for d in g]), 1)))
    summ = outdir / "trimtol_summary.csv"
    with open(summ, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)

    hdr = (f"{'arm':24s} {'n':>2s} {'closure mean':>12s} {'sd':>5s} {'min':>5s} "
           f"{'max':>5s} {'turn sd':>7s} {'mean|r|':>7s} {'nudge/cnr':>9s} "
           f"{'trim s/cnr':>10s} {'tour s':>6s}")
    print(hdr)
    print("-" * len(hdr))
    for s in summary:
        print(f"{s['arm']:24s} {s['n']:2d} {s['closure_mean']:12.1f} "
              f"{s['closure_sd']:5.1f} {s['closure_min']:5.1f} {s['closure_max']:5.1f} "
              f"{str(s['turn_sd']):>7s} {str(s['mean_abs_resid']):>7s} "
              f"{s['nudges_per_corner']:9.2f} {s['trim_s_per_corner']:10.2f} "
              f"{s['tour_s']:6.1f}")

    for prefix, reason in QUARANTINE.items():
        g = [d for d in runs if d["arm_key"] == prefix]
        if g:
            print(f"\nQUARANTINED  {g[0]['arm_name']}  (n={len(g)}, closures "
                  f"{', '.join(f'{d['closure']:.1f}' for d in g)} mm)"
                  f"\n             {reason}")

    trimmed = [d for d in valid if d["corners"] and d["trim"]]
    all_c = [d for d in valid if d["corners"]]
    clo = [d["closure"] for d in all_c]
    pred = [residual_stats(d)[1] for d in all_c]
    print(f"\ncorr(closure, L*sqrt(sum residual^2)) = {corr(clo, pred):+.3f} "
          f"over n={len(all_c)} runs with per-corner records")
    tclo = [d["closure"] for d in trimmed]
    tpred = [residual_stats(d)[1] for d in trimmed]
    print(f"  ... restricted to the {len(trimmed)} TRIMMED runs (no-trim arms "
          f"excluded, so the fit is not carried by two extreme clusters): "
          f"{corr(tclo, tpred):+.3f}")

    deliv = [abs(n["delivered"]) for d in valid for c in d["corners"]
             for n in c["nudges"]]
    moved = [x for x in deliv if x >= 0.25]
    print(f"nudges n={len(deliv)}: {len(deliv) - len(moved)} "
          f"({100 * (len(deliv) - len(moved)) / len(deliv):.0f}%) delivered "
          f"<0.25 deg (no breakaway); the rest median {st.median(moved):.2f} deg, "
          f"10th pct {sorted(moved)[len(moved) // 10]:.2f}")

    # ---- charts ------------------------------------------------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(15.5, 10.5))
    for ax in axes.flat:
        ax.grid(alpha=0.22, lw=0.6)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    # 1. closure by arm -- dot strip, identity carried by x position
    ax = axes[0][0]
    present = [(p, n, f, t) for p, n, f, t in ARMS
               if any(d["arm_key"] == p for d in valid)]
    for i, (prefix, name, family, tol) in enumerate(present):
        g = [d["closure"] for d in valid if d["arm_key"] == prefix]
        color = C_WHEELS if family == "wheels" else C_PLANNER
        jit = [i + 0.16 * ((k % 5) - 2) / 2.0 for k in range(len(g))]
        ax.plot(jit, g, "o", ms=8, color=color, alpha=0.75, mew=0)
        m = st.mean(g)
        ax.plot([i - 0.3, i + 0.3], [m, m], color=INK, lw=2.0, solid_capstyle="round")
        ax.annotate(f"{m:.0f}", xy=(i + 0.33, m), fontsize=10, color=INK,
                    va="center")
    ax.set_xticks(range(len(present)))
    ax.set_xticklabels([n for _, n, _, _ in present], fontsize=8, color=MUTED)
    ax.set_ylabel("closure  [mm]", color=MUTED)
    ax.set_title("Closure by arm -- every valid run, black bar = arm mean\n"
                 "blue = wheels path, orange = planner path", fontsize=11,
                 color=INK, loc="left")

    # 2. nudge authority
    ax = axes[0][1]
    ax.hist(deliv, bins=[i * 0.25 for i in range(21)], color=C_WHEELS,
            edgecolor="#fcfcfb", lw=0.8)
    for tol, style in ((0.3, "-"), (0.5, "--"), (1.0, ":")):
        ax.axvline(tol, color=INK, lw=1.4, ls=style)
        ax.annotate(f"tol {tol}", xy=(tol, ax.get_ylim()[1] * 0.95),
                    fontsize=9, color=INK, rotation=90, ha="right", va="top")
    ax.set_xlabel("heading actually delivered by one trim nudge  [deg]",
                  color=MUTED)
    ax.set_ylabel("nudges", color=MUTED)
    ax.set_title("Trim-nudge authority is bimodal and COARSER than every\n"
                 "tolerance tested: no breakaway, or ~1.8 deg", fontsize=11,
                 color=INK, loc="left")

    # 3. closure vs the residual model
    ax = axes[1][0]
    for family, color in (("wheels", C_WHEELS), ("planner", C_PLANNER)):
        g = [d for d in all_c if d["family"] == family]
        ax.plot([residual_stats(d)[1] for d in g], [d["closure"] for d in g],
                "o", ms=8, color=color, alpha=0.75, mew=0, label=family)
    lim = max(max(clo), max(pred)) * 1.05
    ax.plot([0, lim], [0, lim], color=MUTED, lw=1.2, ls="--", label="y = x")
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    ax.set_aspect("equal")
    ax.set_xlabel("L * sqrt(sum of squared cumulative residuals)  [mm]",
                  color=MUTED)
    ax.set_ylabel("measured closure  [mm]", color=MUTED)
    ax.legend(fontsize=9, frameon=False)
    ax.set_title(f"Closure IS the cumulative-heading residual\n"
                 f"r = {corr(clo, pred):+.2f} over {len(all_c)} runs",
                 fontsize=11, color=INK, loc="left")

    # 4. the fault: delivered travel per identical command, over the session
    ax = axes[1][1]
    t0 = min(d["started_wall"] for d in runs)
    with_legs = sorted([d for d in runs if d["seg_lengths"]],
                       key=lambda d: d["started_wall"])
    ok = [d for d in with_legs if not d["quarantine"]]
    bad = [d for d in with_legs if d["quarantine"]]
    ax.plot([(d["started_wall"] - t0) / 60.0 for d in ok],
            [st.mean(d["seg_lengths"]) for d in ok],
            "o", ms=8, color=C_WHEELS, alpha=0.8, mew=0, label="valid runs")
    ax.plot([(d["started_wall"] - t0) / 60.0 for d in bad],
            [st.mean(d["seg_lengths"]) for d in bad],
            "X", ms=11, color=C_PLANNER, mew=0, label="quarantined (fault)")
    ax.axhline(500.0, color=MUTED, lw=1.2, ls="--")
    ax.annotate("commanded 500 mm", xy=(0.5, 505), fontsize=9, color=MUTED)
    if bad:
        onset = min((d["started_wall"] - t0) / 60.0 for d in bad)
        ax.axvline(onset - 0.4, color=INK, lw=1.4, ls=":")
        ax.annotate("drivetrain fault onset", xy=(onset - 0.8, 250),
                    fontsize=9.5, color=INK, rotation=90, ha="right")
    ax.set_xlabel("minutes into the session", color=MUTED)
    ax.set_ylabel("mean leg delivered per commanded 500 mm  [mm]", color=MUTED)
    ax.legend(fontsize=9, frameon=False, loc="lower left")
    ax.set_title("Session state: identical command, delivered travel.\n"
                 "Flat at 498-501 mm all session, then the drivetrain went",
                 fontsize=11, color=INK, loc="left")

    fig.suptitle("Square-tour closure vs. trim tolerance -- tovez, "
                 f"{runs[0]['started_iso'][:10]}, {len(runs)} runs",
                 fontsize=13, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    chart = outdir / "trimtol_comparison.png"
    fig.savefig(chart, dpi=130)
    print(f"\nwrote {per_run}\nwrote {summ}\nwrote {chart}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
