"""goldens -- plot / compare / bless over golden_trace.py.

Wires the (previously zero-caller) golden library to the systest datasets:
per-signal golden dirs at goldens/<tier>/<tour>/<signal>.{csv,json,png}.

- plot:    render every extractable signal from a dataset to PNGs.
- bless:   promote a dataset's signals to goldens (manual, stakeholder-
           triggered -- nothing self-blesses). --runs feeds
           suggest_tolerance() from no-change repeat runs; without them the
           per-signal DEFAULT_TOLERANCE seeds the band.
- compare: score a dataset against the blessed goldens; nonzero exit on
           any signal outside tolerance. The verdict is analytic
           (max/rms deviation), never pixels (golden_trace's own design).
"""

from __future__ import annotations

import argparse
import gzip
import json
import shutil
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parents[2]
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_REPO / "src" / "tests" / "tools"))

import golden_trace as gt  # noqa: E402
from signals import (  # noqa: E402
    DEFAULT_TOLERANCE, dataset_meta, extract_signals, load_dataset,
)


def _track_width(meta: dict) -> float:  # [mm]
    cfg_path = Path(meta.get("robot_config", ""))
    try:
        cfg = json.loads(cfg_path.read_text())
        return float(cfg["geometry"]["trackwidth"])
    except Exception:
        return 128.0  # ship default; recorded in the golden meta either way


def _spec_for(x: np.ndarray, y: np.ndarray) -> gt.PlotSpec:
    """Pinned axes from the blessing run's own extent + 15% margin,
    rounded outward -- stored with the golden, reused verbatim by every
    later compare (never autoscaled again)."""

    def bounds(v: np.ndarray) -> tuple[float, float]:
        lo, hi = float(np.min(v)), float(np.max(v))
        span = max(hi - lo, 1e-6)
        return lo - 0.15 * span, hi + 0.15 * span

    x_lo, x_hi = bounds(x)
    y_lo, y_hi = bounds(y)
    return gt.PlotSpec(x_low=x_lo, x_high=x_hi, y_low=y_lo, y_high=y_hi)


def _signals_from(dataset: str) -> tuple[dict, dict]:
    recs = load_dataset(dataset)
    meta = dataset_meta(recs)
    sigs = extract_signals(recs, track_width=_track_width(meta))
    return meta, sigs


def _golden_dir(base: str, meta: dict) -> Path:
    tour_name = Path(meta["tour_file"]).stem
    return Path(base) / meta["tier"] / tour_name


def cmd_plot(args: argparse.Namespace) -> int:
    meta, sigs = _signals_from(args.dataset)
    out = Path(args.out) / (Path(args.dataset).stem + "_plots")
    out.mkdir(parents=True, exist_ok=True)
    for name, (x, y) in sigs.items():
        tol = DEFAULT_TOLERANCE.get(name, 10.0)
        gt.render_png(out / f"{name}.png", x, y, _spec_for(x, y), tol, name)
    print(f"plot: {len(sigs)} signals -> {out}")
    return 0


def cmd_bless(args: argparse.Namespace) -> int:
    meta, sigs = _signals_from(args.dataset)
    gdir = _golden_dir(args.goldens, meta)
    spread_runs = [_signals_from(r)[1] for r in (args.runs or [])]
    for name, (x, y) in sigs.items():
        spec = _spec_for(x, y)
        runs = [(rx, ry) for run in spread_runs
                for rname, (rx, ry) in run.items() if rname == name]
        if runs:
            tol = gt.suggest_tolerance(runs, x, y)
        else:
            tol = DEFAULT_TOLERANCE.get(name, 10.0)
        gt.save_golden(gdir, name, x, y, spec, tol)
        gt.render_png(gdir / f"{name}.png", x, y, spec, tol, name)
    with open(args.dataset, "rb") as f_in, \
            gzip.open(gdir / "dataset.jsonl.gz", "wb") as f_out:
        shutil.copyfileobj(f_in, f_out)
    print(f"bless: {len(sigs)} signals -> {gdir}")
    print("goldens are stakeholder-approved artifacts: commit them with a "
          "message saying WHY they changed.")
    return 0


def cmd_compare(args: argparse.Namespace) -> int:
    meta, sigs = _signals_from(args.dataset)
    gdir = _golden_dir(args.goldens, meta)
    if not gdir.is_dir():
        print(f"compare: no goldens at {gdir} (bless first)")
        return 2
    failures = 0
    names = sorted(p.stem for p in gdir.glob("*.csv"))
    for name in names:
        gx, gy, spec, tol = gt.load_golden(gdir, name)
        if name not in sigs:
            print(f"FAIL {name}: signal missing from this run")
            failures += 1
            continue
        x, y = sigs[name]
        cmp_result = gt.compare(x, y, gx, gy, spec, tol)
        verdict = cmp_result.passed()
        line = (f"{'PASS' if verdict else 'FAIL'} {name}: "
                f"max_dev {cmp_result.max_deviation:.2f} "
                f"rms {cmp_result.rms_deviation:.2f} tol {tol:.2f} "
                f"outside {cmp_result.outside_count}")
        print(line)
        if not verdict:
            failures += 1
    extra = sorted(set(sigs) - set(names))
    if extra:
        print(f"note: {len(extra)} unblessed signals present: "
              f"{', '.join(extra)}")
    print(f"compare: {len(names) - failures}/{len(names)} signals within "
          f"tolerance")
    return 1 if failures else 0


def dispatch(args: argparse.Namespace) -> int:
    return {"plot": cmd_plot, "bless": cmd_bless,
            "compare": cmd_compare}[args.which](args)
