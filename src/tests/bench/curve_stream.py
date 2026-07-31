#!/usr/bin/env python3
"""curve_stream.py -- stream a continuously-varying-curvature path through
Motion::Planner's queue, demonstrating the relaxed at-speed hand-off
(planner.cpp's boundaryLambda()/shapesCompatible() relaxation, out-of-
process 2026-07-30) against a genuinely interesting shape instead of a
rounded square.

    uv run python src/tests/bench/curve_stream.py --sim

Path: a 4-petal cloverleaf (``robot_radio.path.patterns.
four_leaf_waypoints``, dormant since sprint 006's ``follow_path.py``)
sampled through the C1 cubic-Bezier builder (``robot_radio.path.bezier``,
dormant since it was extracted for TestGUI path-preview work). Both are
REUSED as-is, not reimplemented -- see the module docstrings on each for
what they already do. The sampled polyline (arc-length-uniform, one point
every ``--segment-cm`` centimetres) is chunked into consecutive-pair arc
Moves: segment i's heading delta over its own arc length gives that
segment's local curvature, and ``v_x``/``omega`` are derived from it so a
SINGLE constant CRUISE command threads the whole path -- the planner (not
this script) is what locally slows a segment whose curvature is too tight
to hold cruise through, via ``curvatureHandoffLambdaCap()``. That is the
whole point of the firmware change this script demonstrates: it is not
this script's job to plan a speed profile, only to describe the shape.

Segments are streamed with ``move_twist(replace=False, ...)`` (SAME three
constraints ``square_tour.py``'s own CHAIN_DEPTH block documents: replace
explicit, completion detected from the ack RING keyed by ``Move.id``,
unique monotonic ids) keeping ``--depth`` Moves in flight in the
firmware's own 5-deep queue (1 active + 4 pending) at once -- default 4,
inside the stakeholder's requested 3-5 range.

Sim only today. This is a bench/HITL-tool convention module
(``src/tests/CLAUDE.md``: bench/ scripts are plain CLI tools, never
pytest-collected) -- a hardware run needs the robot moved to the stand
first (it is on the playfield as of this writing) and stakeholder sign-off,
per ``.claude/rules/hardware-bench-testing.md``/``playfield-testing.md``.

Produces a two-panel PNG (left/right commanded wheel speed vs time, with
segment-boundary markers) and prints the per-boundary MINIMUM wheel speed
-- the number that shows whether a hand-off ever actually touched rest.
"""
from __future__ import annotations

import argparse
import math
import pathlib
import signal
import sys
import time

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT / "src" / "host"))

import robot_radio.path.bezier  # noqa: E402,F401 -- registers "bezier" (direct submodule import -- see below)
from robot_radio.nav.pose import Pose, Waypoint  # noqa: E402
from robot_radio.path.builder import build_path  # noqa: E402
from robot_radio.path.patterns import four_leaf_waypoints  # noqa: E402
from robot_radio.path.sampled_path import SampledPath  # noqa: E402

# `from robot_radio.path import bezier` (rather than the direct submodule
# import above) recurses infinitely: robot_radio/path/__init__.py's own
# lazy-import __getattr__ re-enters itself for the "bezier" name under that
# import form (a latent bug in that package, unrelated to this script --
# not touched here). Direct submodule import sidesteps it.

CRUISE = 300.0       # [mm/s] nominal commanded speed for every segment --
                      # the planner, not this script, slows a tight one
DEPTH = 4             # [in-flight Moves] active + this many queued ahead,
                      # inside the stakeholder's requested 3-5 range
SEGMENT_CM = 6.0      # [cm] arc-length spacing between path samples --
                      # also each segment's own arc length
MOVE_TIMEOUT_MS = 4000.0   # [ms] per-segment safety backstop
POLL_SLICE = 0.04     # [s] one SimLoop cycle (App::RobotLoop::kCycle)
_ID_BASE = 9000        # segment Move ids: _ID_BASE + index, never reused


class MoveSpec:
    __slots__ = ("v_x", "omega", "distance", "index")

    def __init__(self, v_x: float, omega: float, distance: float, index: int):
        self.v_x = v_x          # [mm/s]
        self.omega = omega      # [rad/s]
        self.distance = distance  # [mm] stop_distance -- mean wheel travel
        self.index = index


def build_cloverleaf(*, tip_cm: float = 30.0, spacing_cm: float = SEGMENT_CM) -> SampledPath:
    """A 4-petal cloverleaf, C1-continuous Bezier through it -- reuses
    ``patterns.four_leaf_waypoints`` (the control-point generator) and the
    registered ``"bezier"`` builder (the C1 sampler) verbatim; neither is
    reimplemented here. Returns arc-length-uniform samples every
    ``spacing_cm``."""
    center = (0.0, 0.0)
    tips = [
        ("NE", (tip_cm, tip_cm)),
        ("NW", (-tip_cm, tip_cm)),
        ("SW", (-tip_cm, -tip_cm)),
        ("SE", (tip_cm, -tip_cm)),
    ]
    pts = four_leaf_waypoints(center, tips, bulge=0.30)
    # pts[0] == pts[-1] == center (the loop closes on itself) -- start/end
    # Pose take the endpoints explicitly, everything between is a Waypoint
    # with an inferred (None) heading, exactly build_path()'s own contract.
    start = Pose(pts[0][0], pts[0][1], 0.0)
    end = Pose(pts[-1][0], pts[-1][1], 0.0)
    waypoints = [Waypoint(x, y, None) for x, y in pts[1:-1]]
    return build_path("bezier", start, end, waypoints, spacing_cm=spacing_cm,
                      tangent_frac=0.35)


def segments_from_path(path: SampledPath, *, cruise: float, track_mm: float,
                       omega_max: float) -> list[MoveSpec]:
    """Consecutive-pair arc Moves: segment i runs from ``path.points[i]``
    to ``path.points[i+1]``, its curvature read straight off the sampled
    HEADING delta over its own (uniform) arc length -- exact for the
    underlying spline's own local tangent, not a re-solved circular-arc
    fit. ``v_x`` is CRUISE dropped only as far as ``omega_max`` demands for
    THIS segment's own curvature (a host-side command-time floor, distinct
    from the firmware's own boundaryLambda() hand-off cap this script is
    demonstrating) -- never globally, per the stakeholder's own "slow for
    that arc, not everywhere" instruction."""
    out: list[MoveSpec] = []
    pts = path.points
    heads = path.headings
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        distanceMm = math.hypot(x1 - x0, y1 - y0) * 10.0  # [cm] -> [mm]
        if distanceMm < 1.0:
            continue  # degenerate (duplicate) sample, skip
        dHeading = math.atan2(math.sin(heads[i + 1] - heads[i]),
                              math.cos(heads[i + 1] - heads[i]))
        curvature = dHeading / distanceMm  # [1/mm]
        vX = cruise
        if abs(curvature) > 1e-9:
            vX = min(cruise, omega_max / abs(curvature))
        omega = vX * curvature
        out.append(MoveSpec(vX, omega, distanceMm, len(out)))
    return out


class SimBackend:
    """Minimal sim harness -- same shape as square_tour.py's own
    SimBackend (not imported: that file is a standalone script, not a
    library), reduced to what this script needs: manual, deterministic
    stepping, no tick thread."""

    def __init__(self, robot_json: "str | pathlib.Path") -> None:
        from robot_radio.config.robot_config import load_robot_config
        from robot_radio.io.sim_config import SimConfigConn
        from robot_radio.io.sim_loop import SimLoop
        from robot_radio.robot.protocol import NezhaProtocol

        robot_config = load_robot_config(robot_json)
        track = robot_config.trackwidth if robot_config.trackwidth is not None else 128.0
        self.track_mm = track
        self._sim = SimLoop(track_width=track)
        self._sim.connect(start_tick_thread=False)
        self._sim.configure_from_robot(robot_config)
        self.proto = NezhaProtocol(SimConfigConn(self._sim))

    def advance(self, seconds: float) -> list:
        frames = []
        for _ in range(max(1, int(round(seconds / POLL_SLICE)))):
            self._sim.step(1)
            self._sim._drain_tlm_into_queue()  # noqa: SLF001 -- matches square_tour.py's own call
            frames.extend(self._sim.read_pending_binary_tlm_frames())
        return frames

    def close(self) -> None:
        try:
            self.proto.estop()
        except Exception:
            pass
        self._sim.disconnect()


def _installEstopSignalHandler(backend: "SimBackend") -> None:
    """estop() on SIGTERM/SIGINT before the process exits -- a bare
    SIGTERM bypasses every Python `finally` block (127-002 lesson,
    hardware incident 2026-07-30), so this is defense in depth alongside
    the try/finally in main(), mirroring square_tour.py's own
    ``_installEstopSignalHandler()``."""

    def _handler(signum: int, _frame) -> None:
        try:
            backend.proto.estop()
        except Exception:
            pass
        sys.exit(1)

    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)


def stream_segments(backend: SimBackend, segments: list[MoveSpec], *,
                    depth: int, timeout_ms: float) -> dict:
    """Keep ``depth`` Moves enqueued in Motion::Planner's own queue at
    once (1 active + up to depth-1 pending): the FIRST segment preempts
    (replace=True, "just drive this now"), every later one enqueues behind
    whatever is already queued (replace=False -- a real MoveQueue enqueue,
    never a preempt; getting this backwards would flush the chain on
    every send). Completion is read off the ack RING keyed by Move.id
    (docs/protocol-v4.md section 7.2), never the `active` flag's rise-
    then-fall edge -- `active` stays asserted continuously across the
    whole chain.

    Returns {"t": [...], "left": [...], "right": [...],
    "boundary_times": [...]} -- the wheel-speed trace and the wall-clock
    (sim) time of each segment's own completion, for the chart and the
    per-boundary minimum-speed report.
    """
    n = len(segments)
    ids = [_ID_BASE + s.index for s in segments]
    nextToSend = 0
    completed = 0
    sentAt: dict[int, float] = {}
    trace_t: list[float] = []
    trace_left: list[float] = []
    trace_right: list[float] = []
    boundary_times: list[float] = []
    elapsed = 0.0

    def sendOne(i: int) -> None:
        seg = segments[i]
        backend.proto.move_twist(
            v_x=seg.v_x, v_y=0.0, omega=seg.omega,
            stop_distance=seg.distance, timeout=timeout_ms,
            replace=(i == 0), move_id=ids[i])

    # Prime the queue.
    while nextToSend < n and nextToSend < depth:
        sendOne(nextToSend)
        nextToSend += 1

    watchdogTicks = 0
    maxTicks = int(60.0 / POLL_SLICE)  # 60s hard ceiling -- never hang forever
    while completed < n and watchdogTicks < maxTicks:
        frames = backend.advance(POLL_SLICE)
        elapsed += POLL_SLICE
        watchdogTicks += 1
        for f in frames:
            if f.vel is not None:
                trace_t.append(elapsed)
                trace_left.append(f.vel[0])
                trace_right.append(f.vel[1])
            for ack in f.acks:
                if ack.corr_id in sentAt or ack.corr_id not in ids:
                    continue
                sentAt[ack.corr_id] = elapsed
                completed += 1
                boundary_times.append(elapsed)
                if not ack.ok:
                    print(f"WARNING: segment corr_id={ack.corr_id} "
                          f"acked err={ack.err_code}")
                if nextToSend < n:
                    sendOne(nextToSend)
                    nextToSend += 1
    if completed < n:
        print(f"WARNING: only {completed}/{n} segments completed before "
              f"the watchdog ceiling")
    return {"t": trace_t, "left": trace_left, "right": trace_right,
            "boundary_times": boundary_times}


def report_and_plot(trace: dict, *, out_path: pathlib.Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = trace["t"]
    left = trace["left"]
    right = trace["right"]
    boundaries = trace["boundary_times"]

    print(f"\n{len(boundaries)} segment boundaries crossed.")
    # The LAST recorded boundary is the final segment's OWN completion --
    # it has no successor, so it correctly lands at rest (nothing to hand
    # off TO) and is not a hand-off dip at all; exclude it from the
    # interior-boundary stat below (it still gets its grey line on the
    # chart -- this only affects the printed summary).
    interior = boundaries[:-1] if len(boundaries) > 1 else boundaries
    mins = []
    for bt in interior:
        # Minimum |wheel speed| within +-150ms of this boundary -- the
        # window a control-cycle-scale hand-off dip would show up in.
        window = [(abs(l), abs(r)) for tt, l, r in zip(t, left, right)
                  if abs(tt - bt) <= 0.15]
        if not window:
            continue
        localMin = min(min(l, r) for l, r in window)
        mins.append(localMin)
    if mins:
        stopped = sum(1 for m in mins if m < 1.0)
        print(f"Interior hand-off boundaries: {len(mins)} (excludes the "
              f"final segment's own natural landing)")
        print(f"  minimum wheel speed at a hand-off: min={min(mins):.1f} mm/s, "
              f"mean={sum(mins) / len(mins):.1f} mm/s")
        print(f"  boundaries that fully landed at rest (<1 mm/s): "
              f"{stopped}/{len(mins)}  (a direction reversal at the clover's "
              f"waist -- shapeDirectionsAgree() correctly still requires rest there)")

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t, left, label="left [mm/s]", linewidth=1.2)
    ax.plot(t, right, label="right [mm/s]", linewidth=1.2)
    for bt in boundaries:
        ax.axvline(bt, color="0.8", linewidth=0.6, zorder=0)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("commanded wheel speed [mm/s]")
    ax.set_title("curve_stream: wheel speed across a streamed 4-petal cloverleaf\n"
                "(grey lines = segment boundaries -- relaxed at-speed hand-off)")
    ax.legend(loc="upper right")
    ax.axhline(0.0, color="black", linewidth=0.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    print(f"chart written to {out_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", action="store_true", default=True,
                        help="run against SimLoop (the only backend today)")
    parser.add_argument("--robot", default=str(_REPO_ROOT / "data" / "robots" / "tovez.json"))
    parser.add_argument("--cruise", type=float, default=CRUISE)
    parser.add_argument("--depth", type=int, default=DEPTH)
    parser.add_argument("--segment-cm", type=float, default=SEGMENT_CM)
    parser.add_argument("--tip-cm", type=float, default=30.0)
    parser.add_argument("--out", default=str(pathlib.Path(__file__).parent / "curve_stream_sim.png"))
    args = parser.parse_args()

    path = build_cloverleaf(tip_cm=args.tip_cm, spacing_cm=args.segment_cm)
    print(f"path: {len(path.points)} points, {path.total_length_cm:.1f} cm total "
          f"({path.builder_name})")

    backend = SimBackend(args.robot)
    _installEstopSignalHandler(backend)
    try:
        omega_max = 8.0  # [rad/s] matches benchLimits()/robot config's own omegaMax ballpark
        segments = segments_from_path(path, cruise=args.cruise,
                                      track_mm=backend.track_mm, omega_max=omega_max)
        print(f"{len(segments)} arc segments, cruise={args.cruise:.0f} mm/s, "
              f"depth={args.depth}")
        trace = stream_segments(backend, segments, depth=args.depth,
                                timeout_ms=MOVE_TIMEOUT_MS)
        report_and_plot(trace, out_path=pathlib.Path(args.out))
    finally:
        backend.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
