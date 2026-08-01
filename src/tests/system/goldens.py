"""goldens -- plot / compare / bless over golden_trace.py.

Wires the (previously zero-caller) golden library to the systest datasets:
per-signal golden dirs at goldens/<tier>/<tour>/<signal>.{csv,json,png}.

- plot:    render every extractable signal from a dataset to PNGs.
- bless:   promote a dataset's signals to goldens (manual, stakeholder-
           triggered -- nothing self-blesses). --runs feeds
           suggest_tolerance() from no-change repeat runs; without them the
           per-signal AXIS_TOLERANCE seeds the band.
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
    AXIS_TOLERANCE, CANONICAL_DOMAIN, DEFAULT_AXIS_TOLERANCE,
    assert_within_domain,
    dataset_meta, extract_signals, load_dataset,
)


def _track_width(meta: dict) -> float:  # [mm]
    cfg_path = Path(meta.get("robot_config", ""))
    try:
        cfg = json.loads(cfg_path.read_text())
        return float(cfg["geometry"]["trackwidth"])
    except Exception:
        return 128.0  # ship default; recorded in the golden meta either way


def _spec_for(name: str) -> gt.PlotSpec:
    """The signal's CANONICAL, pinned axes -- never the run's own extent.

    Deriving limits from the blessing run (the previous behaviour) meant every
    golden carried a different coordinate system, so two runs of different
    length rendered to similar-looking images at different scales. A pinned
    domain makes the same data land on the same pixels, every run, forever.
    """
    dom = CANONICAL_DOMAIN.get(name)
    if dom is None:
        raise KeyError(
            f"no canonical plot domain for signal {name!r} -- add it to "
            f"CANONICAL_DOMAIN and AXIS_TOLERANCE in signals.py. Refusing to "
            f"autoscale: an unpinned axis silently makes runs incomparable.")
    x_lo, x_hi, y_lo, y_hi = dom
    return gt.PlotSpec(x_low=x_lo, x_high=x_hi, y_low=y_lo, y_high=y_hi)


def _tolerance_for(name: str) -> gt.AxisTolerance:
    """Per-axis acceptance tolerance for one signal."""
    x_tol, y_tol = AXIS_TOLERANCE.get(name, DEFAULT_AXIS_TOLERANCE)
    return gt.AxisTolerance(x_tol, y_tol)


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
        for complaint in assert_within_domain(name, x, y):
            print(f"WARNING {complaint}", file=sys.stderr)
        gt.render_png(out / f"{name}.png", x, y, _spec_for(name),
                      _tolerance_for(name), name)
    print(f"plot: {len(sigs)} signals -> {out}")
    return 0


def cmd_bless(args: argparse.Namespace) -> int:
    meta, sigs = _signals_from(args.dataset)
    gdir = _golden_dir(args.goldens, meta)
    spread_runs = [_signals_from(r)[1] for r in (args.runs or [])]
    for name, (x, y) in sigs.items():
        spec = _spec_for(name)
        runs = [(rx, ry) for run in spread_runs
                for rname, (rx, ry) in run.items() if rname == name]
        if runs:
            # Repeat runs set the SIZE; the table sets the ellipse's SHAPE.
            tol = gt.suggest_axis_tolerance(runs, x, y, _tolerance_for(name))
        else:
            tol = _tolerance_for(name)
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
        if name in sigs:
            # A clipped run cannot be scored: the band stops where the trace
            # leaves the domain, so the tail would pass by never being drawn.
            clipped = assert_within_domain(name, *sigs[name])
            if clipped:
                for complaint in clipped:
                    print(f"FAIL {complaint}")
                failures += 1
                continue
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
