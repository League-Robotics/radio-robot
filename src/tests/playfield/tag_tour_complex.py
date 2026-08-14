"""A DELIBERATELY complicated closed path through the field tags.

The point of this path is to exercise the drivetrain: it crosses itself, it
alternates left and right turns, and it uses the tightest radius the robot is
allowed. A smoothing spline is the wrong tool for that -- minimising bending
energy always collapses toward a convex blob, and every feasible order came out
with ZERO self-crossings and the shape of a circle.

So this keeps the interesting shape and fixes only what is illegal:

  1. interpolate a periodic cubic through the tags in the requested order --
     this crosses itself and reverses curvature repeatedly, but has corners far
     tighter than the robot can drive,
  2. repair curvature LOCALLY: only points whose radius is under the floor get
     relaxed, in proportion to how badly they violate it. Everything else is
     left alone, so the crossings and the long diagonal runs survive,
  3. hold the curve near the tags with a weak spring, so rounding a corner
     does not walk the whole path away from the tag it was rounding,
  4. keep it inside the drivable box.

    uv run python src/tests/playfield/tag_tour_complex.py
    uv run python src/tests/playfield/tag_tour_complex.py --min-radius 10
    uv run python src/tests/playfield/tag_tour_complex.py --order 1,12,13,10,16,14
"""
import argparse
import itertools
import json
import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

TAGS = {1: (0.13, -0.12), 10: (-44.97, 22.54), 12: (42.97, 25.09),
        13: (-23.26, -25.51), 14: (-5.77, 26.59), 16: (37.78, -27.06)}
IDS = [1, 10, 12, 13, 14, 16]
DEFAULT_ORDER = [1, 10, 12, 13, 14, 16]     # the stakeholder's original

FIELD_X, FIELD_Y = 67.15, 44.65
HALF_W = 6.0
BOX_X, BOX_Y = FIELD_X - HALF_W, FIELD_Y - HALF_W
N = 400


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


def periodic_cubic(pts, n=N):
    """Closed natural cubic spline through pts, chord-length parameterised."""
    P = np.asarray(pts, float); m = len(P)
    C = np.vstack([P, P[:1]])
    h = np.linalg.norm(np.diff(C, axis=0), axis=1)
    A = np.zeros((m, m)); rhs = np.zeros((m, 2))
    for i in range(m):
        im, ip = (i-1) % m, (i+1) % m
        A[i, im] = h[im]; A[i, i] = 2*(h[im]+h[i]); A[i, ip] = h[i]
        rhs[i] = 6*((C[i+1]-C[i])/h[i] - (C[i]-P[im])/h[im])
    M = np.linalg.solve(A, rhs)
    Mc = np.vstack([M, M[:1]])
    d = np.r_[0.0, np.cumsum(h)]
    s = np.linspace(0, d[-1], n, endpoint=False)
    out = np.zeros((n, 2))
    for k, sv in enumerate(s):
        i = min(np.searchsorted(d, sv, side="right")-1, m-1)
        t = sv-d[i]; hi = h[i]
        A0 = (hi-t)/hi; B0 = t/hi
        out[k] = (A0*C[i] + B0*C[i+1]
                  + ((A0**3-A0)*Mc[i] + (B0**3-B0)*Mc[i+1])*(hi*hi)/6.0)
    return out


def repair(order, min_radius, iters=6000, spring=0.06):
    """Local curvature repair; keeps crossings, fixes only illegal corners."""
    T = np.array([TAGS[t] for t in order], float)
    P = periodic_cubic(T)
    for _ in range(iters):
        R = radii(P)
        bad = R < min_radius
        if bad.any():
            w = np.zeros(len(P))
            w[bad] = np.clip((min_radius - R[bad]) / min_radius, 0, 1)
            lap = 0.5*(np.roll(P, 1, 0) + np.roll(P, -1, 0)) - P
            P = P + 0.45 * (w[:, None] * lap)
        # weak spring: each tag pulls its nearest curve point back
        for i in range(len(T)):
            j = int(np.argmin(np.linalg.norm(P - T[i], axis=1)))
            P[j] += spring * (T[i] - P[j])
        P = resample(P)
        np.clip(P[:, 0], -BOX_X, BOX_X, out=P[:, 0])
        np.clip(P[:, 1], -BOX_Y, BOX_Y, out=P[:, 1])
        if not bad.any():
            break
    return P


def repair_tuned(order, min_radius):
    """Bisect the tag spring: strongest attraction that still clears the floor.

    The spring and the curvature repair pull against each other -- holding the
    curve ON a sharp tag REQUIRES a tight corner there. Too strong and the
    repair never converges (measured: min radius stuck at 0.86 cm); too weak
    and the path drifts away from the tags and loses its shape. So find the
    strongest spring that still reaches the radius floor."""
    lo, hi, best = 0.0, 0.30, None
    for _ in range(14):
        mid = 0.5*(lo+hi)
        P = repair(order, min_radius, spring=mid)
        R = radii(P); fin = R[np.isfinite(R)]
        ok = fin.size > 0 and fin.min() >= min_radius - 0.15
        if ok:
            best = (mid, P); lo = mid
        else:
            hi = mid
    if best is None:
        return repair(order, min_radius, spring=0.0)
    return best[1]


def crossings(P):
    n = len(P); C = np.vstack([P, P[:1]])
    A, B = C[:-1], C[1:]
    cnt = 0
    for i in range(n):
        p, r = A[i], B[i]-A[i]
        for j in range(i+2, n if i > 0 else n-1):
            q, s = A[j], B[j]-A[j]
            d = r[0]*s[1]-r[1]*s[0]
            if abs(d) < 1e-12:
                continue
            t = ((q[0]-p[0])*s[1]-(q[1]-p[1])*s[0])/d
            u = ((q[0]-p[0])*r[1]-(q[1]-p[1])*r[0])/d
            if 0 < t < 1 and 0 < u < 1:
                cnt += 1
    return cnt


def turn_flips(P):
    C = np.vstack([P, P[:1], P[1:2]])
    v = np.diff(C, axis=0)
    cr = v[:-1, 0]*v[1:, 1] - v[:-1, 1]*v[1:, 0]
    L = np.linalg.norm(v[:-1], axis=1)*np.linalg.norm(v[1:], axis=1)
    k = np.where(L > 1e-9, cr/np.maximum(L, 1e-9), 0.0)
    med = np.median(np.abs(k)) + 1e-12
    sig = np.sign(np.where(np.abs(k) < 0.05*med, 0, k))
    sig = sig[sig != 0]
    return int(np.sum(sig[1:] != sig[:-1]))


def score(order, min_radius):
    P = repair_tuned(order, min_radius)
    R = radii(P); fin = R[np.isfinite(R)]
    if fin.size == 0:
        return None
    T = np.array([TAGS[t] for t in order], float)
    dev = [float(np.min(np.linalg.norm(P - T[i], axis=1))) for i in range(len(T))]
    L = float(np.sum(np.linalg.norm(np.diff(np.vstack([P, P[:1]]), axis=0), axis=1)))
    over = max(np.max(np.abs(P[:, 0]))-BOX_X, np.max(np.abs(P[:, 1]))-BOX_Y)
    return dict(order=order, P=P, rmin=float(fin.min()), dev=dev, length=L,
                cross=crossings(P), flips=turn_flips(P), over=max(0.0, float(over)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min-radius", type=float, default=8.0)
    ap.add_argument("--order", default=None)
    ap.add_argument("--search", action="store_true",
                    help="search all orders for the most complex feasible path")
    args = ap.parse_args()

    if args.order:
        orders = [[int(x) for x in args.order.split(",")]]
    elif args.search:
        orders = []
        for perm in itertools.permutations(IDS[1:]):
            o = [IDS[0]]+list(perm)
            if o[1] > o[-1]:
                continue
            orders.append(o)
    else:
        orders = [DEFAULT_ORDER]

    results = [r for r in (score(o, args.min_radius) for o in orders) if r]
    ok = [r for r in results if r["rmin"] >= args.min_radius-0.15 and r["over"] <= 0.01]
    pool = ok or results
    # Rank: crossings first, then genuine left/right reversals, then tag fit.
    # A path that misses a tag by more than 12 cm is not "visiting" it, so it
    # is demoted regardless of how interesting it looks.
    pool.sort(key=lambda r: (max(r["dev"]) > 12.0, -r["cross"], -r["flips"],
                             max(r["dev"])))
    best = pool[0]
    P, order, dev = best["P"], best["order"], best["dev"]

    if len(orders) > 1:
        print(f"  searched {len(orders)} orders, {len(ok)} feasible\n"
              f"  {'cross':>6} {'LRflips':>8} {'miss':>6} {'len':>6}  order")
        for r in pool[:6]:
            print(f"  {r['cross']:6d} {r['flips']:8d} {max(r['dev']):6.1f} "
                  f"{r['length']:6.0f}  {' -> '.join(map(str, r['order']))}")
        print()
    print(f"  ORDER: {' -> '.join(map(str, order))} -> {order[0]}")
    print(f"  length {best['length']:.0f} cm   min turn radius {best['rmin']:.2f} cm"
          f"   (limit {args.min_radius:.0f})")
    print(f"  SELF-CROSSINGS {best['cross']}   left/right reversals {best['flips']}")
    print(f"  tag misses: " + "  ".join(f"{t}:{d:.1f}" for t, d in zip(order, dev)))
    print(f"  extent x [{P[:,0].min():+.1f},{P[:,0].max():+.1f}] "
          f"y [{P[:,1].min():+.1f},{P[:,1].max():+.1f}]  "
          f"box +-{BOX_X:.1f}/{BOX_Y:.1f}  overshoot {best['over']:.2f} cm")

    outj = os.path.join(_REPO, "src/tests/bench/output/tag_tour_complex.json")
    os.makedirs(os.path.dirname(outj), exist_ok=True)
    with open(outj, "w") as fh:
        json.dump({"order": order, "tags": TAGS, "min_radius_cm": best["rmin"],
                   "length_cm": best["length"], "crossings": best["cross"],
                   "turn_reversals": best["flips"], "tag_miss_cm": dev,
                   "path_cm": P.tolist()}, fh)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(12, 11),
                                  gridspec_kw={"height_ratios": [2.5, 1]})
    C = np.vstack([P, P[:1]])
    R = radii(P)
    lc = LineCollection(np.stack([C[:-1], C[1:]], axis=1), cmap="viridis_r",
                        array=np.clip(R, 0, 60), linewidths=3.0)
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
                               ec="#ccc", ls="--", lw=1.0, label="drivable"))
    ax.set_xlim(-FIELD_X-4, FIELD_X+4); ax.set_ylim(-FIELD_Y-4, FIELD_Y+4)
    ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlabel("x [cm]"); ax.set_ylabel("y [cm]")
    ax.legend(loc="upper left", fontsize=9)
    ax.set_title("Complex tag tour: " + " -> ".join(map(str, order)) + f" -> {order[0]}\n"
                 f"{best['length']:.0f} cm, min radius {best['rmin']:.1f} cm, "
                 f"{best['cross']} self-crossings, {best['flips']} left/right reversals")
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))][:-1]
    C2 = np.vstack([P, P[:1], P[1:2]])
    v = np.diff(C2, axis=0)
    cr = v[:-1, 0]*v[1:, 1]-v[:-1, 1]*v[1:, 0]
    signed = np.where(np.abs(R) > 1e-9, np.sign(cr)*np.clip(R, 0, 100), 0)
    ax2.plot(d, signed, "-", color="#1f77b4", lw=1.4)
    ax2.axhline(args.min_radius, color="#d62728", ls="--", lw=1.4,
                label=f"+-{args.min_radius:.0f} cm limit")
    ax2.axhline(-args.min_radius, color="#d62728", ls="--", lw=1.4)
    ax2.axhline(0, color="#333", lw=0.8)
    ax2.set_xlabel("distance along path [cm]")
    ax2.set_ylabel("signed turn radius [cm]\n(+ left, - right)")
    ax2.set_ylim(-100, 100); ax2.grid(alpha=0.25); ax2.legend(fontsize=9)
    outp = os.path.join(_REPO, "src/tests/bench/output/tag_tour_complex.png")
    fig.tight_layout(); fig.savefig(outp, dpi=110)
    print(f"\n  chart: {outp}")
    print(f"  path:  {outj}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
