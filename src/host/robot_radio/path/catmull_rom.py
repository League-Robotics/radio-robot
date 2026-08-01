"""Centripetal Catmull-Rom spline construction and sampling.

Extracted from ``test/follow_path.py`` (sprint 006, ticket 004). This
module used to also carry a pure-pursuit lookahead-target picker and its
circle-intersection helper; both were deleted (128-008, zero callers) once
``pathplan.solver.pursuitTarget()`` shipped the same geometry independently
in sprint 127 -- see that module's header note.
"""

from __future__ import annotations

import math


def catmull_rom(points, samples_per_segment=24, alpha=0.5):
    """Centripetal Catmull–Rom spline through *points*."""
    if len(points) < 2:
        return list(points)
    p0p = (2*points[0][0] - points[1][0], 2*points[0][1] - points[1][1])
    pNp = (2*points[-1][0] - points[-2][0], 2*points[-1][1] - points[-2][1])
    pts = [p0p] + list(points) + [pNp]

    def tj(ti, pi, pj):
        d = math.hypot(pj[0] - pi[0], pj[1] - pi[1])
        return ti + max(d, 1e-6) ** alpha

    def lerp(a, b, ta, tb, t):
        w = (tb - t) / (tb - ta)
        return (w*a[0] + (1-w)*b[0], w*a[1] + (1-w)*b[1])

    out = []
    for i in range(len(pts) - 3):
        p0, p1, p2, p3 = pts[i], pts[i+1], pts[i+2], pts[i+3]
        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)
        for k in range(samples_per_segment):
            u = t1 + (t2 - t1) * (k / samples_per_segment)
            A1 = lerp(p0, p1, t0, t1, u)
            A2 = lerp(p1, p2, t1, t2, u)
            A3 = lerp(p2, p3, t2, t3, u)
            B1 = lerp(A1, A2, t0, t2, u)
            B2 = lerp(A2, A3, t1, t3, u)
            out.append(lerp(B1, B2, t1, t2, u))
    out.append(points[-1])
    return out
