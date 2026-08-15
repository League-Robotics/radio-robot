"""splinefile -- the system-test SPLINE path format, and an SVG importer.

A spline path is a closed or open curve in FIELD coordinates that the robot
follows with pure pursuit. One JSON file per path, stored next to the tours:

    {
      "name":   "tag_tour",
      "units":  "mm",              # always mm, field frame, +x east +y north
      "closed": true,
      "source": "how it was produced",
      "points": [[x, y], ...]      # densely sampled, >= 2 points
    }

JSON rather than SVG as the stored form: SVG carries a viewBox, a y-down
frame, transforms and styling that have nothing to do with a robot path, and
every consumer would have to re-implement Bezier flattening. Splines authored
in a drawing program are IMPORTED from SVG once, here, and land in the same
JSON every other path uses -- so there is exactly one format to follow.

    uv run python src/tests/system/splinefile.py import-svg IN.svg OUT.json \
        --name complex --fit
    uv run python src/tests/system/splinefile.py info PATH.json
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

FIELD_X, FIELD_Y = 671.5, 446.5     # [mm] table half-extents
HALF_W = 60.0                        # [mm] robot half-width
BOX_X, BOX_Y = FIELD_X - HALF_W, FIELD_Y - HALF_W
# The robot is NOT a disc: the nose extends well ahead of the centre of
# rotation and the caster behind it. A centre-only bound passed a path whose
# apex put the centre at a "legal" y=370 while the NOSE ground the north rail.
NOSE = 110.0                         # [mm] centre of rotation -> front extent
TAIL = 120.0                         # [mm] centre of rotation -> rear extent


def sweep_violation(points, closed=True, cross_track=60.0):
    """Worst intrusion [mm] of the swept BODY footprint outside the table.

    For each path point the heading is the local tangent; the footprint is
    nose/tail along it and half-width across it, inflated by the follower's
    allowed cross-track. Returns (worst_mm, index) -- worst <= 0 fits.
    """
    import math as _m
    n = len(points)
    worst, wi = -1e9, -1
    for i in range(n):
        a = points[i - 1]
        b = points[(i + 1) % n] if closed else points[min(i + 1, n - 1)]
        hx, hy = b[0] - a[0], b[1] - a[1]
        L = _m.hypot(hx, hy) or 1.0
        hx, hy = hx / L, hy / L
        px, py = points[i]
        for fx, fy in ((px + hx * NOSE, py + hy * NOSE),
                       (px - hx * TAIL, py - hy * TAIL),
                       (px - hy * HALF_W, py + hx * HALF_W),
                       (px + hy * HALF_W, py - hx * HALF_W)):
            v = max(abs(fx) - (FIELD_X - cross_track),
                    abs(fy) - (FIELD_Y - cross_track))
            if v > worst:
                worst, wi = v, i
    return worst, wi


@dataclass(frozen=True)
class SplinePath:
    name: str
    points: tuple[tuple[float, float], ...]   # [mm] field frame
    closed: bool = True
    source: str = ""

    def length(self) -> float:                 # [mm]
        pts = list(self.points) + ([self.points[0]] if self.closed else [])
        return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))

    def min_radius(self) -> float:             # [mm]
        n = len(self.points)
        best = float("inf")
        rng = range(n) if self.closed else range(1, n-1)
        for i in rng:
            a = self.points[(i-1) % n]; b = self.points[i]; c = self.points[(i+1) % n]
            ab = math.dist(a, b); bc = math.dist(b, c); ca = math.dist(c, a)
            cross = abs((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]))
            if cross < 1e-9 or ab*bc*ca == 0:
                continue
            best = min(best, ab*bc*ca/(2*cross))
        return best

    def extent(self):
        xs = [p[0] for p in self.points]; ys = [p[1] for p in self.points]
        return min(xs), max(xs), min(ys), max(ys)


def load(path: str | Path) -> SplinePath:
    d = json.loads(Path(path).read_text())
    if d.get("units", "mm") != "mm":
        raise ValueError(f"{path}: units must be mm")
    pts = tuple((float(p[0]), float(p[1])) for p in d["points"])
    if len(pts) < 2:
        raise ValueError(f"{path}: needs at least 2 points")
    return SplinePath(name=d.get("name", Path(path).stem), points=pts,
                      closed=bool(d.get("closed", True)),
                      source=d.get("source", ""))


def save(sp: SplinePath, path: str | Path) -> None:
    Path(path).write_text(json.dumps({
        "name": sp.name, "units": "mm", "frame": "field",
        "closed": sp.closed, "source": sp.source,
        "length_mm": round(sp.length(), 1),
        "min_radius_mm": round(sp.min_radius(), 1),
        "points": [[round(x, 2), round(y, 2)] for x, y in sp.points],
    }, indent=1))


# ---- SVG import ---------------------------------------------------------

_NUM = re.compile(r"[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?")


def _bezier(p0, p1, p2, p3, n):
    out = []
    for i in range(1, n+1):
        t = i/n; u = 1-t
        out.append((u*u*u*p0[0] + 3*u*u*t*p1[0] + 3*u*t*t*p2[0] + t*t*t*p3[0],
                    u*u*u*p0[1] + 3*u*u*t*p1[1] + 3*u*t*t*p2[1] + t*t*t*p3[1]))
    return out


def parse_svg_path(d: str, per_curve: int = 40):
    """Flatten one SVG path 'd' string (M/L/C/S/Z, absolute or relative)."""
    toks = re.findall(r"[MmLlCcSsZzHhVv]|" + _NUM.pattern, d)
    pts, cur, start, i, cmd, prev_c2 = [], (0.0, 0.0), (0.0, 0.0), 0, None, None
    closed = False
    def num():
        nonlocal i
        v = float(toks[i]); i += 1
        return v
    while i < len(toks):
        if re.fullmatch(r"[MmLlCcSsZzHhVv]", toks[i]):
            cmd = toks[i]; i += 1
        if cmd in ("Z", "z"):
            closed = True
            cur = start
            cmd = None
            continue
        rel = cmd.islower()
        if cmd in ("M", "m"):
            x = num(); y = num()
            cur = (cur[0]+x, cur[1]+y) if rel else (x, y)
            start = cur; pts.append(cur); prev_c2 = None
            cmd = "l" if rel else "L"
        elif cmd in ("L", "l"):
            x = num(); y = num()
            cur = (cur[0]+x, cur[1]+y) if rel else (x, y)
            pts.append(cur); prev_c2 = None
        elif cmd in ("H", "h"):
            x = num(); cur = (cur[0]+x, cur[1]) if rel else (x, cur[1])
            pts.append(cur); prev_c2 = None
        elif cmd in ("V", "v"):
            y = num(); cur = (cur[0], cur[1]+y) if rel else (cur[0], y)
            pts.append(cur); prev_c2 = None
        elif cmd in ("C", "c", "S", "s"):
            if cmd in ("C", "c"):
                x1 = num(); y1 = num(); x2 = num(); y2 = num(); x3 = num(); y3 = num()
                c1 = (cur[0]+x1, cur[1]+y1) if rel else (x1, y1)
            else:
                x2 = num(); y2 = num(); x3 = num(); y3 = num()
                c1 = (2*cur[0]-prev_c2[0], 2*cur[1]-prev_c2[1]) if prev_c2 else cur
            c2 = (cur[0]+x2, cur[1]+y2) if rel else (x2, y2)
            p3 = (cur[0]+x3, cur[1]+y3) if rel else (x3, y3)
            pts.extend(_bezier(cur, c1, c2, p3, per_curve))
            cur = p3; prev_c2 = c2
        else:
            raise ValueError(f"unsupported SVG path command {cmd!r}")
    return pts, closed


def import_svg(svg_path: str | Path, name: str, *, fit: bool = True,
               per_curve: int = 40, fill: float | None = None) -> SplinePath:
    """Import the LONGEST path in an SVG, mapped into field coordinates.

    SVG is y-DOWN and in its own units, so the import flips y and (with
    --fit) scales/centres the curve to fill the drivable box. Without --fit
    the numbers are taken as millimetres in the field frame already.
    """
    text = Path(svg_path).read_text()
    ds = re.findall(r'<path[^>]*\bd="([^"]+)"', text)
    if not ds:
        raise ValueError(f"{svg_path}: no <path d=...> found")
    best = None
    for d in ds:
        pts, closed = parse_svg_path(d, per_curve)
        if len(pts) < 2:
            continue
        L = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        if best is None or L > best[0]:
            best = (L, pts, closed)
    _, pts, closed = best
    pts = [(x, -y) for x, y in pts]                 # SVG y-down -> field y-up
    if fit:
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        cx, cy = (min(xs)+max(xs))/2, (min(ys)+max(ys))/2
        w, h = max(xs)-min(xs), max(ys)-min(ys)
        # 0.78, not 0.92: the follower needs room for its own tracking error.
        # At 0.92 the imported path reached y = -356 mm against a 386 mm fence
        # and the camera geofence stopped the run 2 mm outside -- the path
        # itself was legal but left less margin than the cross-track error.
        s = min(2*BOX_X/w if w else 1e9, 2*BOX_Y/h if h else 1e9) * (fill or 0.78)
        pts = [((x-cx)*s, (y-cy)*s) for x, y in pts]
    return SplinePath(name=name, points=tuple(pts), closed=closed,
                      source=f"imported from {Path(svg_path).name}"
                             f"{' (fitted to field)' if fit else ''}")


def _cmd_import(a):
    sp = import_svg(a.svg, a.name, fit=a.fit, per_curve=a.per_curve,
                    fill=a.fill)
    save(sp, a.out)
    _describe(sp, a.out)
    return 0


def _describe(sp: SplinePath, where):
    x0, x1, y0, y1 = sp.extent()
    print(f"  {sp.name}: {len(sp.points)} points, "
          f"{'closed' if sp.closed else 'open'}")
    print(f"  length {sp.length()/10:.1f} cm   min radius {sp.min_radius()/10:.1f} cm")
    print(f"  extent x [{x0/10:+.1f},{x1/10:+.1f}] y [{y0/10:+.1f},{y1/10:+.1f}] cm"
          f"   box +-{BOX_X/10:.1f}/{BOX_Y/10:.1f}")
    fits = (max(abs(x0), abs(x1)) <= BOX_X and max(abs(y0), abs(y1)) <= BOX_Y)
    print(f"  {'FITS the drivable box' if fits else 'DOES NOT FIT'}")
    print(f"  {where}")


def _cmd_info(a):
    _describe(load(a.path), a.path)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="splinefile", description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("import-svg")
    p.add_argument("svg"); p.add_argument("out")
    p.add_argument("--name", required=True)
    p.add_argument("--per-curve", type=int, default=40)
    p.add_argument("--fit", action="store_true", default=True)
    p.add_argument("--fill", type=float, default=None,
                   help="fraction of the drivable box to fill (default 0.78)")
    p.add_argument("--no-fit", dest="fit", action="store_false")
    p.set_defaults(fn=_cmd_import)
    p2 = sub.add_parser("info"); p2.add_argument("path"); p2.set_defaults(fn=_cmd_info)
    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    raise SystemExit(main())
