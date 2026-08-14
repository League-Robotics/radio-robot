"""Closed SPLINE through every tag exactly, with extra knots to open the turns.

The tags are interpolation points -- the curve passes through all six EXACTLY.
Between each pair of tags sit extra free knots, and those are moved until the
whole curve respects a minimum turning radius. That is the "extra knots in the
spline" idea: you cannot change where the curve must go, but you can change how
it gets there, and a spline with more control points can round a corner that a
6-point spline has to take sharply.

Everything is a natural periodic cubic spline, so curvature varies smoothly --
there are no constant-radius circular arcs anywhere in the output.

The tag ORDER decides whether this is possible at all. Sweeping orders that run
14 -> 12 -> 16 (down the right-hand side) instead of jumping corner to corner
turns ~20 degree vertices into gentle ones.

    uv run python src/tests/playfield/tag_tour_knots.py
    uv run python src/tests/playfield/tag_tour_knots.py --order 1,13,14,12,16,10
    uv run python src/tests/playfield/tag_tour_knots.py --min-radius 10
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

FIELD_X, FIELD_Y = 67.15, 44.65
HALF_W = 6.0
BOX_X, BOX_Y = FIELD_X - HALF_W, FIELD_Y - HALF_W
DENSE = 600


def periodic_cubic(K, n=DENSE):
    """Closed natural cubic spline through knots K, chord-length parameterised."""
    K = np.asarray(K, float); m = len(K)
    C = np.vstack([K, K[:1]])
    h = np.linalg.norm(np.diff(C, axis=0), axis=1)
    h = np.maximum(h, 1e-6)
    A = np.zeros((m, m)); rhs = np.zeros((m, 2))
    for i in range(m):
        im, ip = (i-1) % m, (i+1) % m
        A[i, im] = h[im]; A[i, i] = 2*(h[im]+h[i]); A[i, ip] = h[i]
        rhs[i] = 6*((C[i+1]-C[i])/h[i] - (C[i]-K[im])/h[im])
    M = np.linalg.solve(A, rhs)
    Mc = np.vstack([M, M[:1]])
    d = np.r_[0.0, np.cumsum(h)]
    sv = np.linspace(0, d[-1], n, endpoint=False)
    i = np.clip(np.searchsorted(d, sv, side="right")-1, 0, m-1)
    hi = h[i]
    t = sv - d[i]
    a0 = ((hi-t)/hi)[:, None]; b0 = (t/hi)[:, None]
    h2 = (hi*hi)[:, None]
    return (a0*C[i] + b0*C[i+1]
            + ((a0**3-a0)*Mc[i] + (b0**3-b0)*Mc[i+1])*h2/6.0)


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


def init_knots(order, per_seg):
    """Tags (fixed) plus per_seg free knots between each consecutive pair."""
    T = np.array([TAGS[t] for t in order], float)
    K, fixed = [], []
    for i in range(len(T)):
        K.append(T[i]); fixed.append(True)
        a, b = T[i], T[(i+1) % len(T)]
        for j in range(1, per_seg+1):
            K.append(a + (b-a)*j/(per_seg+1)); fixed.append(False)
    return np.array(K), np.array(fixed)


def objective(K, min_radius):
    """(min radius, on-table) for the spline through K. Higher is better."""
    P = periodic_cubic(K)
    R = radii(P)
    fin = R[np.isfinite(R)]
    if fin.size == 0:
        return -1e9, P
    over = max(np.max(np.abs(P[:, 0]))-BOX_X, np.max(np.abs(P[:, 1]))-BOX_Y)
    pen = max(0.0, over) * 50.0
    return float(fin.min()) - pen, P


def optimize(order, min_radius, per_seg=3, iters=4000, seed=0):
    """Hill-climb the FREE knots to maximise the tightest radius on the curve.

    Directly optimising the quantity we care about, rather than nudging knots
    by a hand-rolled rule: an earlier version pushed each violating sample's
    nearest knot outward and diverged badly (radius 0.07 cm, path length blown
    from 270 to 550 cm) because the pushes fought each other and cusped the
    spline. Tag knots never move, so the curve stays exact at every tag.
    """
    rng = np.random.default_rng(seed)
    K, fixed = init_knots(order, per_seg)
    free = np.where(~fixed)[0]
    cur, _ = objective(K, min_radius)
    scale = 6.0
    for it in range(iters):
        j = free[rng.integers(len(free))]
        step = rng.normal(0.0, scale, size=2)
        trial = K.copy()
        trial[j] = trial[j] + step
        trial[:, 0] = np.clip(trial[:, 0], -BOX_X, BOX_X)
        trial[:, 1] = np.clip(trial[:, 1], -BOX_Y, BOX_Y)
        val, _ = objective(trial, min_radius)
        if val > cur:
            K, cur = trial, val
            if cur >= min_radius:
                break
        if it % 500 == 499:
            scale = max(1.0, scale*0.7)
    return K


def report(order, min_radius, per_seg, iters=20000):
    K = optimize(order, min_radius, per_seg, iters=iters)
    P = periodic_cubic(K)
    R = radii(P); fin = R[np.isfinite(R)]
    T = np.array([TAGS[t] for t in order], float)
    dev = [float(np.min(np.linalg.norm(P - T[i], axis=1))) for i in range(len(T))]
    L = float(np.sum(np.linalg.norm(np.diff(np.vstack([P, P[:1]]), axis=0), axis=1)))
    over = max(np.max(np.abs(P[:, 0]))-BOX_X, np.max(np.abs(P[:, 1]))-BOX_Y)
    return dict(order=order, K=K, P=P, R=R, rmin=float(fin.min()) if fin.size else 0.0,
                dev=dev, length=L, over=max(0.0, float(over)))


def crossings(P):
    n = len(P); C = np.vstack([P, P[:1]]); A, B = C[:-1], C[1:]
    cnt = 0
    for i in range(0, n, 2):
        p, r = A[i], B[i]-A[i]
        for j in range(i+2, n if i > 0 else n-1, 2):
            q, s = A[j], B[j]-A[j]
            d = r[0]*s[1]-r[1]*s[0]
            if abs(d) < 1e-12:
                continue
            t = ((q[0]-p[0])*s[1]-(q[1]-p[1])*s[0])/d
            u = ((q[0]-p[0])*r[1]-(q[1]-p[1])*r[0])/d
            if 0 < t < 1 and 0 < u < 1:
                cnt += 1
    return cnt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min-radius", type=float, default=8.0)
    ap.add_argument("--per-seg", type=int, default=5)
    ap.add_argument("--iters", type=int, default=20000)
    ap.add_argument("--order", default=None)
    args = ap.parse_args()

    if args.order:
        cands = [[int(x) for x in args.order.split(",")]]
    else:
        # orders that sweep 14 -> 12 -> 16 down the right-hand side
        cands = []
        for perm in itertools.permutations(IDS[1:]):
            o = [IDS[0]]+list(perm)
            s = o+o
            if not any(s[i:i+3] == [14, 12, 16] or s[i:i+3] == [16, 12, 14]
                       for i in range(len(o))):
                continue
            if o[1] > o[-1]:
                continue
            cands.append(o)
    print(f"  evaluating {len(cands)} order(s), {args.per_seg} extra knots per segment")
    res = [report(o, args.min_radius, args.per_seg, args.iters) for o in cands]
    ok = [r for r in res if r["rmin"] >= args.min_radius-0.05 and r["over"] <= 0.01]
    pool = ok or res
    pool.sort(key=lambda r: (-(r["rmin"] >= args.min_radius-0.05), max(r["dev"]),
                             -crossings(r["P"])))
    best = pool[0]
    P, order, dev = best["P"], best["order"], best["dev"]
    xs = crossings(P)

    if len(cands) > 1:
        print(f"  feasible: {len(ok)} of {len(cands)}")
        for r in pool[:5]:
            print(f"      minR {r['rmin']:5.2f}  miss {max(r['dev']):4.1f}  "
                  f"len {r['length']:5.0f}  {' -> '.join(map(str, r['order']))}")
        print()
    print(f"  ORDER: {' -> '.join(map(str, order))} -> {order[0]}")
    print(f"  length {best['length']:.0f} cm   min turn radius {best['rmin']:.2f} cm"
          f"  (limit {args.min_radius:.0f})  {'OK' if best['rmin']>=args.min_radius-0.05 else 'VIOLATION'}")
    print(f"  worst tag miss {max(dev):.2f} cm  (all tags are spline knots)")
    print(f"  self-crossings {xs}   overshoot {best['over']:.2f} cm")

    outj = os.path.join(_REPO, "src/tests/bench/output/tag_tour_knots.json")
    os.makedirs(os.path.dirname(outj), exist_ok=True)
    with open(outj, "w") as fh:
        json.dump({"order": order, "tags": TAGS, "min_radius_cm": best["rmin"],
                   "length_cm": best["length"], "crossings": xs,
                   "knots_cm": best["K"].tolist(), "path_cm": P.tolist()}, fh)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(12, 11),
                                  gridspec_kw={"height_ratios": [2.5, 1]})
    C = np.vstack([P, P[:1]])
    R = best["R"]
    lc = LineCollection(np.stack([C[:-1], C[1:]], axis=1), cmap="viridis_r",
                        array=np.clip(R, 0, 60), linewidths=3.0)
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, fraction=0.03, pad=0.02)
    cb.set_label("local turn radius [cm] (darker = tighter)")
    K = best["K"]
    fx = np.array([any(abs(K[i]-np.array(TAGS[t])).max() < 1e-6 for t in order)
                   for i in range(len(K))])
    ax.plot(K[~fx, 0], K[~fx, 1], "x", color="#1f77b4", ms=7, mew=1.6,
            label="extra spline knots", zorder=4)
    for i, t in enumerate(order):
        x, y = TAGS[t]
        ax.plot([x], [y], "o", ms=13, color="#d62728", zorder=5)
        ax.annotate(f" {t}", (x, y), fontsize=12, fontweight="bold",
                    color="#d62728", zorder=6)
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
    ax.set_title("Spline through ALL tags: " + " -> ".join(map(str, order))
                 + f" -> {order[0]}\n{best['length']:.0f} cm, min radius "
                 f"{best['rmin']:.1f} cm, worst tag miss {max(dev):.2f} cm, "
                 f"{xs} self-crossings")
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))][:-1]
    C2 = np.vstack([P, P[:1], P[1:2]])
    v = np.diff(C2, axis=0)
    cr = v[:-1, 0]*v[1:, 1]-v[:-1, 1]*v[1:, 0]
    ax2.plot(d, np.sign(cr)*np.clip(R, 0, 100), "-", color="#1f77b4", lw=1.4)
    ax2.axhline(args.min_radius, color="#d62728", ls="--", lw=1.4,
                label=f"+-{args.min_radius:.0f} cm limit")
    ax2.axhline(-args.min_radius, color="#d62728", ls="--", lw=1.4)
    ax2.axhline(0, color="#333", lw=0.8)
    for t in order:
        j = int(np.argmin(np.linalg.norm(P-np.array(TAGS[t]), axis=1)))
        ax2.axvline(d[j], color="#999", ls=":", lw=1.0)
        ax2.annotate(f"{t}", (d[j], 88), fontsize=9, color="#555")
    ax2.set_xlabel("distance along path [cm]")
    ax2.set_ylabel("signed turn radius [cm]\n(+ left, - right)")
    ax2.set_ylim(-100, 100); ax2.grid(alpha=0.25); ax2.legend(fontsize=9, loc="lower right")
    outp = os.path.join(_REPO, "src/tests/bench/output/tag_tour_knots.png")
    fig.tight_layout(); fig.savefig(outp, dpi=110)
    print(f"\n  chart: {outp}")
    print(f"  path:  {outj}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
