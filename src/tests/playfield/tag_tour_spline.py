"""Smooth closed SPLINE visiting the field tags, with a hard minimum turn radius.

The robot drives this without stopping or pivoting, so the curve needs a floor
on its turning radius (8 cm by default). Two facts drive the design, both
measured rather than assumed:

  * The tag ORDER matters enormously. The originally requested
    1 -> 10 -> 12 -> 13 -> 14 -> 16 makes a star with ~20 degree vertices; no
    smooth curve gets near those tags at 8 cm. All 60 distinct closed orders
    were searched.
  * Even the best order cannot be hit EXACTLY: an interpolating periodic cubic
    through all six tags tops out at 4.7 cm minimum radius. Tag 1 sits at the
    field centre, so every tour has to dip inward for it.

So the tags are targets, not hard constraints (stakeholder: "it doesn't have to
be exactly what I laid out"), and this fits a penalized smoothing spline

    minimise  lambda * sum |p[i-1] - 2p[i] + p[i+1]|^2  +  sum |p[k_tag] - tag|^2

bisecting lambda to the SMALLEST smoothing that reaches the radius floor --
i.e. the closest the curve can stay to the tags while still being drivable.
Curvature is always measured on a uniformly arc-length resampled curve.

    uv run python src/tests/playfield/tag_tour_spline.py
    uv run python src/tests/playfield/tag_tour_spline.py --min-radius 10
    uv run python src/tests/playfield/tag_tour_spline.py --order 1,10,12,13,14,16
"""
import argparse
import itertools
import json
import math
import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

# Re-surveyed 2026-08-14 after the tags were moved inboard; median of 60 frames
TAGS = {1: (0.13, -0.12), 10: (-44.97, 22.54), 12: (42.97, 25.09),
        13: (-23.26, -25.51), 14: (-5.77, 26.59), 16: (37.78, -27.06)}
IDS = [1, 10, 12, 13, 14, 16]

FIELD_X, FIELD_Y = 67.15, 44.65
HALF_W = 6.0
BOX_X, BOX_Y = FIELD_X - HALF_W, FIELD_Y - HALF_W
N = 300


def radii(P):
    a, b, c = np.roll(P, 1, 0), P, np.roll(P, -1, 0)
    ab = np.linalg.norm(b-a, axis=1); bc = np.linalg.norm(c-b, axis=1)
    ca = np.linalg.norm(a-c, axis=1)
    cr = np.abs((b[:, 0]-a[:, 0])*(c[:, 1]-a[:, 1]) -
                (b[:, 1]-a[:, 1])*(c[:, 0]-a[:, 0]))
    R = np.full(len(P), np.inf)
    m = cr > 1e-12
    R[m] = ab[m]*bc[m]*ca[m]/(2*cr[m])
    return R


def resample(P, n=N):
    C = np.vstack([P, P[:1]])
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))]
    if d[-1] < 1e-9:
        return P
    s = np.linspace(0.0, d[-1], n, endpoint=False)
    return np.c_[np.interp(s, d, C[:, 0]), np.interp(s, d, C[:, 1])]


def seed(order):
    pts = np.array([TAGS[t] for t in order], float)
    C = np.vstack([pts, pts[:1]])
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))]
    s = np.linspace(0.0, d[-1], N, endpoint=False)
    P = np.c_[np.interp(s, d, C[:, 0]), np.interp(s, d, C[:, 1])]
    idx = [int(np.argmin(np.linalg.norm(P - pts[i], axis=1))) for i in range(len(pts))]
    return P, idx


def solve(lmb, idx, T):
    D = (-2.0*np.eye(N) + np.roll(np.eye(N), 1, 1) + np.roll(np.eye(N), -1, 1))
    A = lmb*(D.T @ D) + np.diag(np.isin(np.arange(N), idx).astype(float))
    out = np.zeros((N, 2))
    for k in (0, 1):
        rhs = np.zeros(N); rhs[idx] = T[:, k]
        out[:, k] = np.linalg.solve(A, rhs)
    return out


def fit(order, lmb, rounds=5):
    T = np.array([TAGS[t] for t in order], float)
    P, idx = seed(order)
    for _ in range(rounds):
        P = solve(lmb, idx, T)
        if not np.all(np.isfinite(P)):
            return None, None
        P = resample(P)
        nidx = [int(np.argmin(np.linalg.norm(P - T[i], axis=1))) for i in range(len(T))]
        if len(set(nidx)) < len(nidx):
            break
        idx = nidx
    return P, idx


def evaluate(order, min_radius):
    """Smallest smoothing that reaches the radius floor; returns its miss."""
    T = np.array([TAGS[t] for t in order], float)
    lo, hi, best = 1e-3, 1e6, None
    for _ in range(30):
        mid = math.sqrt(lo*hi)
        P, idx = fit(order, mid)
        if P is None:
            lo = mid; continue
        r = radii(P); fin = r[np.isfinite(r)]
        if fin.size == 0:
            lo = mid; continue
        over = max(np.max(np.abs(P[:, 0]))-BOX_X, np.max(np.abs(P[:, 1]))-BOX_Y)
        if fin.min() >= min_radius and over <= 0:
            best = (mid, P, float(fin.min()), idx); hi = mid
        else:
            lo = mid
    if best is None:
        return None
    lmb, P, rmin, idx = best
    dev = [float(np.min(np.linalg.norm(P - T[i], axis=1))) for i in range(len(T))]
    L = float(np.sum(np.linalg.norm(np.diff(np.vstack([P, P[:1]]), axis=0), axis=1)))
    return dict(order=order, lmb=lmb, P=P, rmin=rmin, dev=dev, length=L, idx=idx)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min-radius", type=float, default=8.0)
    ap.add_argument("--order", default=None, help="e.g. 1,10,12,13,14,16")
    args = ap.parse_args()

    if args.order:
        cands = [[int(x) for x in args.order.split(",")]]
    else:
        cands = []
        for perm in itertools.permutations(IDS[1:]):
            o = [IDS[0]] + list(perm)
            if o[1] > o[-1]:
                continue
            cands.append(o)
    print(f"  evaluating {len(cands)} tag order(s) at min radius "
          f"{args.min_radius:.0f} cm ...")
    results = [r for r in (evaluate(o, args.min_radius) for o in cands) if r]
    if not results:
        print("  no order satisfies the constraint"); return 1
    results.sort(key=lambda r: max(r["dev"]))
    best = results[0]

    P, dev, order = best["P"], best["dev"], best["order"]
    print(f"  feasible orders: {len(results)} of {len(cands)}\n")
    print(f"  BEST ORDER: {' -> '.join(map(str, order))} -> {order[0]}")
    print(f"  length {best['length']:.1f} cm   lambda {best['lmb']:.3g}")
    print(f"  min turn radius {best['rmin']:.2f} cm   (limit {args.min_radius:.0f})")
    print(f"  distance from each tag to the curve:")
    for t, d in zip(order, dev):
        print(f"      tag {t:>2}: {d:5.1f} cm")
    print(f"  worst tag miss {max(dev):.1f} cm")
    print(f"  extent x [{P[:,0].min():+.1f},{P[:,0].max():+.1f}] "
          f"y [{P[:,1].min():+.1f},{P[:,1].max():+.1f}]  box +-{BOX_X:.1f}/{BOX_Y:.1f}  ON TABLE")
    if len(results) > 1:
        print(f"\n  runners-up (worst miss):")
        for r in results[1:4]:
            print(f"      {' -> '.join(map(str, r['order']))}: {max(r['dev']):.1f} cm")

    outj = os.path.join(_REPO, "src/tests/bench/output/tag_tour_spline.json")
    os.makedirs(os.path.dirname(outj), exist_ok=True)
    with open(outj, "w") as fh:
        json.dump({"order": order, "tags": TAGS, "lambda": best["lmb"],
                   "min_radius_cm": best["rmin"], "length_cm": best["length"],
                   "tag_miss_cm": dev, "path_cm": P.tolist()}, fh)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(12, 11),
                                  gridspec_kw={"height_ratios": [2.5, 1]})
    C = np.vstack([P, P[:1]])
    R = radii(P)
    lc = LineCollection(np.stack([C[:-1], C[1:]], axis=1), cmap="viridis_r",
                        array=np.clip(R, 0, 60), linewidths=3.4)
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, fraction=0.03, pad=0.02)
    cb.set_label("local turn radius [cm] (darker = tighter)")
    for i, t in enumerate(order):
        x, y = TAGS[t]
        ax.plot([x], [y], "o", ms=13, color="#d62728", zorder=5)
        j = int(np.argmin(np.linalg.norm(P - np.array([x, y]), axis=1)))
        ax.plot([x, P[j, 0]], [y, P[j, 1]], "-", color="#d62728", lw=1.0, alpha=0.6)
        ax.annotate(f" {t} ({dev[i]:.0f}cm)", (x, y), fontsize=11,
                    fontweight="bold", color="#d62728", zorder=6)
    ax.plot([TAGS[order[0]][0]], [TAGS[order[0]][1]], "*", ms=22, color="#2ca02c",
            zorder=7, label=f"start / finish (tag {order[0]})")
    ax.add_patch(plt.Rectangle((-FIELD_X, -FIELD_Y), 2*FIELD_X, 2*FIELD_Y,
                               fill=False, ec="#888", lw=1.4, label="table"))
    ax.add_patch(plt.Rectangle((-BOX_X, -BOX_Y), 2*BOX_X, 2*BOX_Y, fill=False,
                               ec="#ccc", ls="--", lw=1.0, label="drivable (robot centre)"))
    ax.set_xlim(-FIELD_X-4, FIELD_X+4); ax.set_ylim(-FIELD_Y-4, FIELD_Y+4)
    ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlabel("x [cm]"); ax.set_ylabel("y [cm]")
    ax.legend(loc="upper left", fontsize=9)
    ax.set_title("Tag tour SPLINE: " + " -> ".join(map(str, order)) + f" -> {order[0]}\n"
                 f"{best['length']:.0f} cm, min turn radius {best['rmin']:.1f} cm "
                 f"(limit {args.min_radius:.0f}), worst tag miss {max(dev):.1f} cm")
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))][:-1]
    ax2.plot(d, np.clip(R, 0, 100), "-", color="#1f77b4", lw=1.6)
    ax2.axhline(args.min_radius, color="#d62728", ls="--", lw=1.5,
                label=f"minimum {args.min_radius:.0f} cm")
    for i, t in enumerate(order):
        j = int(np.argmin(np.linalg.norm(P - np.array(TAGS[t]), axis=1)))
        ax2.axvline(d[j], color="#999", ls=":", lw=1.0)
        ax2.annotate(f"{t}", (d[j], 92), fontsize=9, color="#555")
    ax2.set_xlabel("distance along path [cm]"); ax2.set_ylabel("turn radius [cm]")
    ax2.set_ylim(0, 100); ax2.grid(alpha=0.25); ax2.legend(fontsize=9)
    outp = os.path.join(_REPO, "src/tests/bench/output/tag_tour_spline.png")
    fig.tight_layout(); fig.savefig(outp, dpi=110)
    print(f"\n  chart: {outp}")
    print(f"  path:  {outj}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
