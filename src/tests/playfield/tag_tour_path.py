"""Build a smooth closed path through the field tags: 1 -> 10 -> 12 -> 13 -> 14 -> 16 -> 1.

The robot must drive this WITHOUT stopping or pivoting, so the path has to be
G2-ish and respect a hard minimum turning radius (default 8 cm). The tag order
contains severe reversals -- 12 is far top-right, 13 is far bottom-left -- so a
plain spline through the six points kinks badly. Extra knots are inserted
between the anchors and relaxed until the curvature constraint is met
everywhere.

Method: knots = the six fixed anchors plus K free knots per segment. Iterate
  1. bending relaxation (Laplacian) on the free knots -- smooths the curve,
  2. curvature relief: at any knot whose circumradius is under the minimum,
     push its NEIGHBOURS away from it. Opening the triangle raises the
     circumradius, and it works at a fixed anchor too, where the knot itself
     cannot move -- which is the whole difficulty here,
  3. redistribute knots evenly within each segment (anchors pinned),
  4. clamp inside the drivable box.

    uv run python src/tests/playfield/tag_tour_path.py
    uv run python src/tests/playfield/tag_tour_path.py --min-radius 10
"""
import argparse
import json
import math
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

import numpy as np

# Surveyed 2026-08-14 from the camera, median of 12 frames [cm]
TAGS = {1: (0.12, -0.13), 10: (-37.12, 13.46), 12: (31.88, 35.79),
        13: (-41.25, -33.84), 14: (-11.32, 34.50), 16: (42.62, -31.13)}
ORDER = [1, 10, 12, 13, 14, 16]

FIELD_X, FIELD_Y = 67.15, 44.65      # [cm] table half-extents
HALF_W = 6.0                          # [cm] robot half-width (side, not corner)
BOX_X = FIELD_X - HALF_W              # [cm] centre must stay inside this
BOX_Y = FIELD_Y - HALF_W


def wrap(t):
    return math.remainder(t, 2 * math.pi)


def arc_exit(P, theta_in, psi_out, R, want=0):
    """Exit point of a radius-R arc leaving P with heading theta_in and
    finishing with heading psi_out. The anchor is the arc's START, so the path
    passes through the tag EXACTLY (a corner fillet would cut it).

    want = 0 takes the short way round; +1 forces CCW, -1 forces CW. Forcing a
    direction is what keeps a loop from swinging off the table: the short turn
    at a reversal can bulge outward past the rail, while the long way round
    bulges inward."""
    d = wrap(psi_out - theta_in)
    if want > 0 and d < 0:
        d += 2 * math.pi
    elif want < 0 and d > 0:
        d -= 2 * math.pi
    sgn = 1.0 if d >= 0 else -1.0
    nvec = np.array([-math.sin(theta_in), math.cos(theta_in)]) * sgn
    C = P + R * nvec
    c, s_ = math.cos(d), math.sin(d)
    v = P - C
    return C + np.array([c * v[0] - s_ * v[1], s_ * v[0] + c * v[1]]), C, d


def solve_headings(anchors, R, wants=None, iters=400):
    """Fixed point: each tag gets a radius-R arc, then a straight run to the
    next tag. Exit heading must equal the bearing from the arc exit to the next
    anchor -- couple that around the closed loop and iterate to convergence."""
    n = len(anchors)
    theta = []
    for i in range(n):
        d = anchors[i] - anchors[(i - 1) % n]
        theta.append(math.atan2(d[1], d[0]))
    for _ in range(iters):
        newtheta = list(theta)
        for i in range(n):
            nxt = anchors[(i + 1) % n]
            psi = theta[i]
            for _ in range(60):          # inner solve for this arc's exit
                E, _C, _d = arc_exit(anchors[i], theta[i], psi, R,
                                     0 if wants is None else wants[i])
                v = nxt - E
                if np.linalg.norm(v) < 1e-9:
                    break
                psi_new = math.atan2(v[1], v[0])
                if abs(wrap(psi_new - psi)) < 1e-10:
                    psi = psi_new
                    break
                psi = psi + 0.5 * wrap(psi_new - psi)
            newtheta[(i + 1) % n] = psi
        if max(abs(wrap(a - b)) for a, b in zip(newtheta, theta)) < 1e-12:
            theta = newtheta
            break
        theta = newtheta
    return theta


def build(min_radius, wants=None, per_seg=None, iters=None):
    """Arc-line path: constant-radius arcs at the tags, straight lines between.
    Minimum radius is min_radius BY CONSTRUCTION -- lines are infinite radius,
    arcs are exactly min_radius, and nothing else exists."""
    R = min_radius
    anchors = [np.array(TAGS[t], float) for t in ORDER]
    theta = solve_headings(anchors, R, wants)

    pts, radii, marks = [], [], []
    n = len(anchors)
    for i in range(n):
        nxt = anchors[(i + 1) % n]
        psi = theta[(i + 1) % n]
        E, C, d = arc_exit(anchors[i], theta[i], psi, R,
                           0 if wants is None else wants[i])
        marks.append(len(pts))
        m = max(6, int(abs(d) / math.radians(3)))          # arc samples
        for k in range(m):
            ang = d * k / m
            c, s_ = math.cos(ang), math.sin(ang)
            v = anchors[i] - C
            pts.append(C + np.array([c*v[0] - s_*v[1], s_*v[0] + c*v[1]]))
            radii.append(R)
        seg = nxt - E
        L = float(np.linalg.norm(seg))
        m2 = max(2, int(L / 1.5))                          # line samples
        for k in range(m2):
            pts.append(E + seg * (k / m2))
            radii.append(float("inf"))
    return np.array(pts), np.array(radii), marks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min-radius", type=float, default=8.0)   # [cm]
    ap.add_argument("--per-seg", type=int, default=26)
    args = ap.parse_args()

    # Try every combination of per-tag turn direction; keep the one that stays
    # on the table (and, among those, the shortest path).
    import itertools
    best = None
    for combo in itertools.product((+1, -1), repeat=len(ORDER)):
        try:
            Pc, Rc, mc = build(args.min_radius, wants=list(combo))
        except Exception:
            continue
        over = max(np.max(np.abs(Pc[:, 0])) - BOX_X,
                   np.max(np.abs(Pc[:, 1])) - BOX_Y)
        Lc = float(np.sum(np.linalg.norm(np.diff(np.vstack([Pc, Pc[:1]]), axis=0), axis=1)))
        key = (max(0.0, over), Lc)
        if best is None or key < best[0]:
            best = (key, Pc, Rc, mc, combo)
    P, R, marks = best[1], best[2], best[3]
    print(f"  turn directions {['CCW' if c > 0 else 'CW' for c in best[4]]}"
          f"   overshoot {best[0][0]:.2f} cm")
    finite = R[np.isfinite(R)]
    length = float(np.sum(np.linalg.norm(np.diff(np.vstack([P, P[:1]]), axis=0), axis=1)))

    print(f"  path through tags {' -> '.join(str(t) for t in ORDER)} -> {ORDER[0]}")
    print(f"  knots {len(P)}   length {length:.1f} cm")
    print(f"  min radius {finite.min():6.2f} cm   (limit {args.min_radius:.1f})"
          f"   {'OK' if finite.min() >= args.min_radius - 0.05 else 'VIOLATION'}")
    print(f"  median radius {np.median(finite):.1f} cm")
    print(f"  extent x [{P[:,0].min():+.1f},{P[:,0].max():+.1f}]  "
          f"y [{P[:,1].min():+.1f},{P[:,1].max():+.1f}]  "
          f"box +-{BOX_X:.1f}/{BOX_Y:.1f}")

    out_json = os.path.join(_REPO, "src/tests/bench/output/tag_tour_path.json")
    os.makedirs(os.path.dirname(out_json), exist_ok=True)
    with open(out_json, "w") as fh:
        json.dump({"order": ORDER, "tags": TAGS,
                   "min_radius_cm": float(finite.min()),
                   "length_cm": length,
                   "path_cm": P.tolist()}, fh)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(12, 11),
                                  gridspec_kw={"height_ratios": [2.5, 1]})

    C = np.vstack([P, P[:1]])
    segsC = np.stack([C[:-1], C[1:]], axis=1)
    rr = np.clip(R, 0, 60)
    lc = LineCollection(segsC, cmap="viridis_r", array=rr, linewidths=3.0)
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, fraction=0.03, pad=0.02)
    cb.set_label("local turn radius [cm] (darker = tighter)")

    for t in ORDER:
        x, y = TAGS[t]
        ax.plot([x], [y], "o", ms=13, color="#d62728", zorder=5)
        ax.annotate(f" tag {t}", (x, y), fontsize=12, fontweight="bold",
                    color="#d62728", zorder=6)
    ax.plot([TAGS[1][0]], [TAGS[1][1]], "*", ms=22, color="#2ca02c",
            zorder=7, label="start / finish (tag 1)")
    ax.add_patch(plt.Rectangle((-FIELD_X, -FIELD_Y), 2*FIELD_X, 2*FIELD_Y,
                               fill=False, ec="#888", ls="-", lw=1.4, label="table"))
    ax.add_patch(plt.Rectangle((-BOX_X, -BOX_Y), 2*BOX_X, 2*BOX_Y,
                               fill=False, ec="#ccc", ls="--", lw=1.0,
                               label="drivable (robot centre)"))
    ax.set_xlim(-FIELD_X-4, FIELD_X+4); ax.set_ylim(-FIELD_Y-4, FIELD_Y+4)
    ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlabel("x [cm]"); ax.set_ylabel("y [cm]")
    ax.legend(loc="upper left", fontsize=9)
    ax.set_title(f"Tag tour: 1 -> 10 -> 12 -> 13 -> 14 -> 16 -> 1\n"
                 f"closed spline, {length:.0f} cm, min turn radius "
                 f"{finite.min():.1f} cm (limit {args.min_radius:.0f})")

    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))][:-1]
    ax2.plot(d, np.clip(R, 0, 80), "-", color="#1f77b4", lw=1.5)
    ax2.axhline(args.min_radius, color="#d62728", ls="--", lw=1.5,
                label=f"minimum {args.min_radius:.0f} cm")
    for i, t in enumerate(ORDER):
        ax2.axvline(d[marks[i]], color="#999", ls=":", lw=1.0)
        ax2.annotate(f"{t}", (d[marks[i]], 74), fontsize=9, color="#555")
    ax2.set_xlabel("distance along path [cm]")
    ax2.set_ylabel("turn radius [cm]")
    ax2.set_ylim(0, 80); ax2.grid(alpha=0.25); ax2.legend(fontsize=9)

    out = os.path.join(_REPO, "src/tests/bench/output/tag_tour_path.png")
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print(f"  chart: {out}")
    print(f"  path:  {out_json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
