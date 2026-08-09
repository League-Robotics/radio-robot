#!/usr/bin/env python3
"""planner_square_tour.py -- the SQUARE TOUR fully ON-ROBOT: 4 x 500 mm
legs + 4 x 90 deg turns, planned AND executed by the firmware's onboard
Motion::Planner (planner-integration build), measured by the real encoders.

The host's only roles are command submission and telemetry capture -- the
control loop is entirely inside the firmware (never split across the
serial transport). Moves are enqueued with ``replace=False`` honoring the
5-deep queue (1 active + 4 pending): the script tops the queue up as
completion acks (``ack_corr == Move.id``) come back on the telemetry ack
ring.

Robot is mounted on a stand with the wheels off the ground (see
`.claude/rules/hardware-bench-testing.md`), so it is safe to drive freely.

Outputs a PASS/FAIL summary (all 8 moves complete; encoder path/heading
near targets) and the same dual-trace plot as the simulation tour:
real wheel speeds (encoders) + the telemetry pose heading.

Usage:
    uv run python src/tests/bench/planner_square_tour.py
    uv run python src/tests/bench/planner_square_tour.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import math
import pathlib
import signal
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

# The heading-trim mechanism under test is the wheels tour's, imported --
# not reimplemented -- so the planner arm and the wheels arm are corrected by
# the SAME code and any difference in closure is the tour, not the trim.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from wheels_square_tour import rest_stats, trim_to_heading  # noqa: E402

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
ACK_TIMEOUT = 1000   # [ms] wait_for_ack() bound for each enqueue ack
BOOT_WAIT = 5.0      # [s] pid-removal firmware boots slower; wait out the preamble
TRACK = 128.0        # [mm]
LEG = 500.0          # [mm]
CRUISE = 150.0       # [mm/s]
TURN = math.pi / 2   # [rad]
OMEGA = 2.4          # [rad/s] pivot rate -> +/-153.6 mm/s per wheel
# Was 1.2 (+/-76.8 mm/s per wheel). Raised 2026-07-28 on bench evidence:
# below ~120 mm/s this plant is not repeatable -- four identical trials at
# 76.8 mm/s spread 26-53 mm/s, and one wheel failed to turn at all on some
# reverse trials -- so a 1.2 pivot ran the wheels inside the dead zone and
# came out asymmetric, arcing through corners instead of rotating. At
# 153.6 mm/s the same measurement is clean (a counter-rotating pair matched
# to 0.2 mm/s). Tour A/B, 4 runs each, NON-OVERLAPPING closures:
#   omega 1.2 -> 9.8, 21.1, 18.0, 18.0 mm (median 18.0)
#   omega 2.4 -> 9.1,  5.1,  3.2,  7.1 mm (median  6.1)
# Heading accuracy is unchanged (+/-1.5 deg either way) -- the turns always
# reached their ANGLE, they just got there by arcing. Override with --omega.
MOVE_TIMEOUT = 30000.0  # [ms] per-move safety backstop
QUEUE_DEPTH = 5      # 1 active + 4 pending
WALL_LIMIT = 120.0   # [s] whole-tour wall-clock bound


def tour(omega: float = OMEGA, id_base: int | None = None,
         cruise: float = CRUISE) -> list[dict]:
    """The 8 tour moves as move_twist() kwargs, in order.

    `omega` [rad/s] sets the pivot rate, i.e. each wheel runs at
    omega*trackWidth/2. The default 2.4 puts the wheels at +/-153.6 mm/s,
    on the well-behaved part of the plant's curve. The previous 1.2 put
    them at +/-76.8 mm/s, inside the erratic near-breakaway region
    (measured 2026-07-28: below ~120 mm/s the two wheels' achieved/commanded
    ratios diverge wildly and are not repeatable, e.g. 0.194 vs 0.539 at
    77 mm/s; at and above ~150 mm/s both directions agree within a few
    percent). See the OMEGA constant above for the A/B numbers. Exposed as
    a flag so the two can still be compared on one robot.

    `id_base` seeds the 8 move IDs (id_base+1 .. id_base+8); default None
    derives a fresh one from wall-clock time. 127-002: the robot does NOT
    necessarily reboot between separate invocations of this script (
    confirmed on the RADIOBRIDGE relay path -- unlike some direct-USB
    reconnects, `!GO` re-handshakes the relay dongle, not the robot, so
    RobotLoop's own uptime and its `acceptedMoveIds_` dedup ring persist
    across runs). The OLD hardcoded 9001-9008 base meant a second
    consecutive run over an unrebooted session sent the exact same IDs
    the first run already recorded as accepted -- every one of them got
    silently deduped (acked OK, never actually enqueued), producing a
    false FAIL with zero real motion and no informative error. Seeding
    from time.time() keeps IDs unique run-to-run without touching
    Move.id's own wire meaning (still just an opaque uint32 label) and
    without changing anything about a SINGLE run's own internal
    consistency (still 8 IDs, 2 per leg/turn pair, well above any
    corr_id this session's SerialConnection will assign).
    """
    if id_base is None:
        id_base = (int(time.time()) % 1_000_000) * 100
    moves = []
    for i in range(4):
        moves.append(dict(v_x=cruise, v_y=0.0, omega=0.0,
                          stop_distance=LEG, timeout=MOVE_TIMEOUT,
                          replace=False, move_id=id_base + 1 + 2 * i))
        moves.append(dict(v_x=0.0, v_y=0.0, omega=omega,
                          stop_angle=TURN, timeout=MOVE_TIMEOUT,
                          replace=False, move_id=id_base + 2 + 2 * i))
    return moves


def _append_frame(log: dict, f, t: float) -> None:
    """Append one telemetry frame to the shared log dict (the same columns
    the pipelined path logs, so all downstream reporting is unchanged)."""
    if f.enc_left is None or f.enc_right is None:
        return
    log["t"].append(t)
    log["velLeft"].append(f.enc_left.velocity)
    log["velRight"].append(f.enc_right.velocity)
    log["posLeft"].append(f.enc_left.position)
    log["posRight"].append(f.enc_right.position)
    log["heading"].append(f.pose[2] / 100.0 if f.pose else float("nan"))
    log["x"].append(f.pose[0] if f.pose else float("nan"))
    log["y"].append(f.pose[1] if f.pose else float("nan"))
    if f.twist is not None:
        v = float(f.twist[0])
        omega = float(f.twist[1]) / 1000.0                    # [rad/s]
        half = 0.5 * TRACK * omega
        log["cmdLeft"].append(v - half)
        log["cmdRight"].append(v + half)
    else:
        log["cmdLeft"].append(float("nan"))
        log["cmdRight"].append(float("nan"))


class SequentialTour:
    """Rest-to-rest planner tour: ONE Move in flight at a time, a settle
    dwell at every segment boundary, and (optionally) a host-side heading
    trim after each turn Move using the wheels tour's own nudge helper.

    The shipped pipelined tour keeps the queue topped up and never rests, so
    it has no moment at which a heading correction could be applied. This
    path exists to answer one question: does the planner, given the same
    per-corner correction the wheels path gets, close like the wheels path?
    """

    SETTLE_TICK = 0.10   # [s]
    LEASE = 400.0        # [ms] the WHEELS lease the settle/nudges re-arm
    ENQUEUE_RETRY_S = 2.0
    ENQUEUE_RETRIES_MAX = 5
    MOVE_WALL = 30.0     # [s] per-move host-side backstop

    def __init__(self, proto, args, log, t0):
        self.proto = proto
        self.args = args
        self.log = log
        self.t0 = t0
        self.acks: "list[tuple[float, object]]" = []
        self.completions: "list[tuple[float, int]]" = []

    # -- telemetry ----------------------------------------------------
    def drain(self, _proto=None, _log=None) -> None:
        """Drain pending frames. The two ignored parameters let this be
        handed straight to stream_segment()/trim_to_heading() as their
        drain_fn, which call it as drain(proto, log)."""
        for f in self.proto.read_pending_binary_tlm_frames():
            t = time.monotonic() - self.t0
            _append_frame(self.log, f, t)
            for entry in f.acks:
                self.acks.append((t, entry))

    def settle(self):
        """Rest dwell at a segment boundary. Returns ((x, y, heading),
        rest-spread stats).

        Two modes, selected by `--settle-mode` (default `passive`):

        `passive` -- wait, draining telemetry, and send NOTHING. This is the
        default, and the correct instrument, for one measured reason: a
        zero-velocity WHEELS command is a TELEOP TAKEOVER, not a no-op.
        `Core::RobotLoop::handleWheels()` calls `planner_.estop()` ("Drive
        takes over motion -- one owner at a time"), and `Planner::estop()`
        clears `carryValid_`, the planner's cumulative-heading intent
        ledger. So a lease held through the dwell destroys that ledger at
        EVERY segment boundary, and the tour then measures the ledger being
        torn down rather than measuring the tour. That is not a firmware
        bug -- the firmware fails closed exactly as designed, because the
        host told it someone else is driving.

        Measured on `tovez` 2026-08-05 (ticket 134-004, report
        `docs/bench-reports/sprint-134-004-bench-acceptance-2026-08-05.md`
        Part II section 10): same firmware, same tour, only the settle
        differs -- passive closed 3.6/8.2 mm with 12/12 corners inside
        `align_tol`; lease closed 34.2 mm with 1/8.

        `lease` -- the historical behaviour: hold `wheels(0, 0)` for the
        whole dwell, re-arming a short lease every tick. Kept because the
        wheels tour's own settle does exactly this, so a planner-vs-wheels
        comparison wants an identical dwell on both sides, and because
        every result in this file's history before 2026-08-05 was measured
        this way.
        """
        mark = len(self.log["heading"])
        end = time.monotonic() + self.args.settle
        lease = self.args.settle_mode == "lease"
        while time.monotonic() < end:
            if lease:
                self.proto.wheels(0.0, 0.0, self.LEASE)
            time.sleep(self.SETTLE_TICK)
            self.drain()
        pose = (self.log["x"][-1], self.log["y"][-1], self.log["heading"][-1])
        return pose, rest_stats(self.log["heading"][mark:])

    # -- one move -----------------------------------------------------
    def run_move(self, spec: dict) -> float:
        """Enqueue one Move, wait for its enqueue ack, then for its
        completion ack. Returns the completion time [s since t0]."""
        # Let any WHEELS lease from the preceding settle/nudge expire before
        # handing motion ownership back to the planner.
        time.sleep(0.5)
        self.drain()
        self.acks.clear()
        move_id = spec["move_id"]
        corr = self.proto.move_twist(**spec)
        enqueued = False
        tries = 0
        enq_deadline = time.monotonic() + self.ENQUEUE_RETRY_S
        deadline = time.monotonic() + self.MOVE_WALL
        while time.monotonic() < deadline:
            self.drain()
            while self.acks:
                _t, entry = self.acks.pop(0)
                if entry.corr_id == corr and not enqueued:
                    if not entry.ok:
                        raise RuntimeError(
                            f"enqueue move {move_id} rejected: "
                            f"err={entry.err_code}")
                    enqueued = True
                elif enqueued and entry.corr_id == move_id:
                    t = time.monotonic() - self.t0
                    self.completions.append((t, move_id))
                    return t
            if not enqueued and time.monotonic() > enq_deadline:
                tries += 1
                if tries > self.ENQUEUE_RETRIES_MAX:
                    raise RuntimeError(f"move {move_id} enqueue never acked")
                corr = self.proto.move_twist(**spec)
                enq_deadline = time.monotonic() + self.ENQUEUE_RETRY_S
                print(f"  (retry {tries} for move {move_id})")
            time.sleep(0.02)
        raise RuntimeError(f"move {move_id} never completed within "
                           f"{self.MOVE_WALL:.0f}s")

    # -- the tour -----------------------------------------------------
    def run(self, moves: "list[dict]"):
        rest_poses = []
        corners = []
        pose, start_stats = self.settle()
        rest_poses.append(pose)
        h_start = pose[2]
        for corner in range(4):
            self.run_move(moves[2 * corner])                  # leg
            pose, leg_stats = self.settle()
            rest_poses.append(pose)
            self.run_move(moves[2 * corner + 1])              # turn
            pose, turn_stats = self.settle()
            rest_poses.append(pose)
            target = h_start + 90.0 * (corner + 1)            # [deg]
            rec = dict(corner=corner + 1, target=target,
                       heading_pre_trim=pose[2],
                       residual_pre_trim=target - pose[2],
                       raw_turn=pose[2] - rest_poses[-2][2],
                       rest_sd_after_turn=turn_stats["sd"],
                       rest_ptp_after_turn=turn_stats["ptp"],
                       rest_sd_after_leg=leg_stats["sd"],
                       nudges=[], trim_seconds=0.0)
            if self.args.trim:
                pose, nudges, elapsed = trim_to_heading(
                    self.proto, self.log, pose, target, self.args.trim_tol,
                    self.args.trim_max_nudges, 1.0,
                    settle_fn=self.settle, drain_fn=self.drain)
                rest_poses[-1] = pose
                rec["nudges"] = nudges
                rec["trim_seconds"] = elapsed
            rec["heading_post_trim"] = rest_poses[-1][2]
            rec["residual_post_trim"] = target - rest_poses[-1][2]
            rec["nudge_count"] = len(rec["nudges"])
            rec["converged"] = abs(rec["residual_post_trim"]) <= self.args.trim_tol
            corners.append(rec)
            print(f"  corner {corner + 1}/4: heading {rest_poses[-1][2]:7.1f} "
                  f"(turn {rest_poses[-1][2] - rest_poses[-2][2]:+6.1f})  "
                  f"x={rest_poses[-1][0]:7.1f} y={rest_poses[-1][1]:7.1f}  "
                  f"resid {rec['residual_pre_trim']:+5.2f} -> "
                  f"{rec['residual_post_trim']:+5.2f} in "
                  f"{rec['nudge_count']} nudge(s) / {rec['trim_seconds']:.1f}s"
                  + ("" if rec["converged"] else "  [NOT CONVERGED]"))
        return rest_poses, corners, start_stats


def _install_estop_signal_handler(proto: NezhaProtocol) -> None:
    """Guarantee `estop()` fires even under an external SIGTERM/SIGINT.

    127-002 (2026-07-30): a hardware batch running this script was killed
    mid-Move by its orchestrating process's own session boundary, and the
    robot was found still driving afterward -- the `try/finally` below
    never ran. Python's default SIGTERM disposition terminates the
    process immediately WITHOUT running `finally` blocks (unlike SIGINT,
    which raises `KeyboardInterrupt` and IS caught by `finally` -- so a
    Ctrl-C was already safe; a bare kill was not). This installs an
    explicit handler for both signals so an external terminate -- not
    just an in-process exception or a clean Ctrl-C -- also estops before
    the process exits. Installed as early as `proto` exists (right after
    connect(), before the boot-wait sleep) so the whole run is covered,
    not just the tour loop.

    This is defense in depth, not a complete guarantee: a SIGKILL cannot
    be caught by any process-level handler. The operational mitigation
    for that remaining gap is procedural, not code -- never run this
    script as an unsupervised background batch; run it in the foreground
    and wait for it to actually exit before considering the robot idle.
    """

    def _handler(signum: int, _frame) -> None:
        try:
            proto.estop()
        except Exception:
            pass
        sys.exit(1)

    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--relay", action="store_true",
                   help="the port is the RADIO RELAY dongle: force the relay "
                        "!GO data-plane handshake instead of banner "
                        "auto-detect (which misclassifies when the relay is "
                        "already forwarding telemetry -- commands then die "
                        "in the relay's control plane)")
    p.add_argument("--out", default=None,
                   help="plot path (default: alongside this script)")
    p.add_argument("--omega", type=float, default=OMEGA,
                   help="[rad/s] pivot rate; each wheel runs at "
                        "omega*trackWidth/2. Default 2.4 = +/-153.6 mm/s, "
                        "on the well-behaved part of the curve; 1.2 = "
                        "+/-76.8 mm/s, inside the plant's erratic "
                        "near-breakaway region (see tour()).")
    p.add_argument("--cruise", type=float, default=CRUISE,
                   help="[mm/s] leg cruise speed handed to the planner "
                        "(default 150, the shipped tour speed)")
    p.add_argument("--outdir", default=None,
                   help="directory for the chart + a per-run rest-pose CSV; "
                        "overrides --out")
    p.add_argument("--label", default="tour",
                   help="run label used in the --outdir filenames")
    p.add_argument("--settle", type=float, default=1.2,
                   help="[s] rest dwell before the first move and after the "
                        "last completion ack. Closure is measured between "
                        "those two REST poses -- the same basis "
                        "wheels_square_tour.py uses, so the two tours are "
                        "comparable. 0 restores the old first-sample/"
                        "last-sample basis.")
    p.add_argument("--settle-mode", choices=("passive", "lease"),
                   default="passive",
                   help="what the SEQUENTIAL tour does during each settle "
                        "dwell. passive (default): wait and send nothing. "
                        "lease: hold wheels(0,0) for the whole dwell -- the "
                        "historical behaviour. A zero-velocity WHEELS "
                        "command is a teleop takeover, so it routes to "
                        "planner_.estop() and clears the planner's "
                        "cumulative-heading ledger; leasing during a settle "
                        "therefore measures the ledger being destroyed "
                        "rather than the tour. See SequentialTour.settle(). "
                        "No effect on the pipelined tour, which never "
                        "leases.")
    p.add_argument("--sequential", action="store_true",
                   help="one Move in flight at a time with a settle dwell at "
                        "every segment boundary, instead of the shipped "
                        "pipelined queue. Implied by --trim (a correction "
                        "needs a rest boundary to be applied at).")
    p.add_argument("--trim", action="store_true",
                   help="after each turn Move completes and settles, close "
                        "the CUMULATIVE heading residual to n*90 deg with "
                        "wheels_square_tour.py's own low-speed pivot nudges, "
                        "then send the next leg. Implies --sequential.")
    p.add_argument("--trim-tol", type=float, default=1.0)   # [deg]
    p.add_argument("--trim-max-nudges", type=int, default=6)
    args = p.parse_args()
    if args.trim:
        args.sequential = True

    conn = SerialConnection(port=args.port,
                            mode="relay" if args.relay else None)
    conn.connect()
    proto = NezhaProtocol(conn)
    _install_estop_signal_handler(proto)
    print(f"connected on {args.port}; waiting {BOOT_WAIT:.0f}s for boot preamble")
    time.sleep(BOOT_WAIT)
    # Telemetry is silent at idle by design, so the pre-first-move and
    # post-last-move REST dwells would see no frames at all without this --
    # the tour itself only ever saw frames because a Move was running.
    proto.tlmOn()
    time.sleep(0.5)
    proto.read_pending_binary_tlm_frames()

    moves = tour(args.omega, cruise=args.cruise)
    next_enqueue = 0
    inflight = 0
    # corr_id -> (move_id, send_time, retries). MEASURED (ticket 135-001,
    # 2026-08-05, tovez): the RADIO RELAY lost 3.5% of inbound enqueues at
    # a steady 20Hz/200-cmd rate (0% on direct serial at the same rate),
    # with the firmware's own command-ring-full fault bit clear throughout
    # -- i.e. genuine link loss, not firmware backpressure. See
    # `src/tests/bench/command_loss_bench.py`'s own module docstring and
    # this sprint's ticket 001 Completion Notes for the full breakdown;
    # this SUPERSEDES the previously-cited, unsourced "~20% of inbound
    # lines are dropped (DAPLink bridge...)" figure. Real, nonzero loss
    # remains, so a lost enqueue would otherwise pin `inflight` forever:
    # unacked sends are RETRIED with a fresh corr_id.
    # A retry after a lost *ack* (rather than a lost command) would
    # double-enqueue that move -- accepted as rare-by-construction: the ack
    # ring rides EVERY telemetry frame, so an ack only vanishes if all
    # frames carrying it drop, while a command is one single line.
    pending_enqueues: dict[int, tuple[int, float, int]] = {}
    ENQUEUE_RETRY_S = 1.2
    ENQUEUE_RETRIES_MAX = 6
    ENQUEUE_SPACING_S = 0.15  # never burst the relay's single-line buffer
    last_send = 0.0
    completions: list[tuple[float, int]] = []  # (t, move_id)
    log = dict(t=[], velLeft=[], velRight=[], posLeft=[], posRight=[],
               heading=[], cmdLeft=[], cmdRight=[], x=[], y=[])
    enc_start = None
    # Rest poses, the same boundary-pose basis wheels_square_tour.py uses:
    # index 0 is the settled pose BEFORE the first move, index -1 the settled
    # pose after the last completion ack. The eight entries between them are
    # the pose sampled AT each completion ack -- the planner runs its queue
    # back to back with no dwell, so those eight are in-motion boundaries,
    # not rest poses, and are labelled as such wherever they are reported.
    rest_poses: "list[tuple[float, float, float]]" = []
    boundary_poses: "list[tuple[int, float, float, float]]" = []
    corners: "list[dict]" = []
    start_stats: dict = {}
    started_wall = time.time()
    t0 = time.monotonic()

    def settle(seconds: float) -> "tuple[float, float, float] | None":
        """Dwell `seconds` draining telemetry, return the last pose seen."""
        end = time.monotonic() + seconds
        last = None
        while time.monotonic() < end:
            for f in proto.read_pending_binary_tlm_frames():
                if f.pose is not None:
                    last = (f.pose[0], f.pose[1], f.pose[2] / 100.0)
            time.sleep(0.02)
        return last

    try:
        if args.sequential:
            seq = SequentialTour(proto, args, log, t0)
            rest_poses, corners, start_stats = seq.run(moves)
            completions = seq.completions
            next_enqueue = len(moves)
            enc_start = (log["posLeft"][0], log["posRight"][0])
            # The 8 boundary poses the per-segment report wants are exactly
            # rest_poses[1:] here -- REST boundaries, not in-motion ones.
            boundary_poses = [
                (moves[i]["move_id"], p[0], p[1], p[2])
                for i, p in enumerate(rest_poses[1:])]
        elif args.settle > 0.0:
            pose = settle(args.settle)
            if pose is not None:
                rest_poses.append(pose)
        while not args.sequential and time.monotonic() - t0 < WALL_LIMIT:
            # Top the queue up (replace=False; leave one slot of headroom
            # against ack latency so ERR_FULL never races the ring). Enqueue
            # acks are matched off the SAME single telemetry drain below --
            # never wait_for_ack(), whose internal drain would eat the
            # velocity samples and completion acks this loop logs.
            now = time.monotonic()
            if (next_enqueue < len(moves) and inflight < QUEUE_DEPTH - 1
                    and now - last_send >= ENQUEUE_SPACING_S):
                corr = proto.move_twist(**moves[next_enqueue])
                pending_enqueues[corr] = (
                    moves[next_enqueue]["move_id"], now, 0)
                next_enqueue += 1
                inflight += 1
                last_send = now
            # Retry any enqueue whose ack never came back (radio drop).
            for corr in list(pending_enqueues):
                move_id, sent, tries = pending_enqueues[corr]
                if time.monotonic() - sent < ENQUEUE_RETRY_S:
                    continue
                del pending_enqueues[corr]
                if tries + 1 > ENQUEUE_RETRIES_MAX:
                    print(f"FAIL: move {move_id} unacked after "
                          f"{ENQUEUE_RETRIES_MAX} retries")
                    return 1
                spec = next(m for m in moves if m["move_id"] == move_id)
                corr2 = proto.move_twist(**spec)
                pending_enqueues[corr2] = (move_id, time.monotonic(), tries + 1)
                last_send = time.monotonic()
                print(f"  (retry {tries + 1} for move {move_id})")

            for f in proto.read_pending_binary_tlm_frames():
                t = time.monotonic() - t0
                if f.enc_left is not None and f.enc_right is not None:
                    if enc_start is None:
                        enc_start = (f.enc_left.position, f.enc_right.position)
                    log["t"].append(t)
                    log["velLeft"].append(f.enc_left.velocity)
                    log["velRight"].append(f.enc_right.velocity)
                    log["posLeft"].append(f.enc_left.position)
                    log["posRight"].append(f.enc_right.position)
                    # pose is (x, y, heading) with heading in centi-degrees
                    log["heading"].append(f.pose[2] / 100.0 if f.pose else float("nan"))
                    log["x"].append(f.pose[0] if f.pose else float("nan"))
                    log["y"].append(f.pose[1] if f.pose else float("nan"))
                    # NOT COMMANDED -- this is MEASURED, decomposed.
                    #
                    # `Telemetry.twist` is documented in telemetry.proto as
                    # "body twist from measured wheel velocities" and is
                    # populated from state.pose.v_x/omega (telemetry.cpp).
                    # Decomposing it per wheel therefore reproduces the
                    # encoder velocities already plotted, by a different
                    # route -- it is the SAME signal, not a second one.
                    #
                    # This block used to be labelled "commanded", and the
                    # chart drew it as a dashed "commanded" trace sitting
                    # exactly on the solid measured trace. That overlay was
                    # an artifact of plotting one signal twice, and was
                    # repeatedly misread as evidence of perfect tracking.
                    #
                    # There is currently NO commanded-velocity telemetry to
                    # plot instead: RobotState::Command::v_x/omega are
                    # unwired (permanently 0.0, see robot_state.h) and the
                    # real per-wheel setpoint `cmd_vel` lived on
                    # TelemetrySecondary, which has been deleted. Exposing it
                    # means adding a field to the primary frame. Until then
                    # the honest thing is to label this what it is -- see
                    # docs/design/2026-07-28-motion-profile-exploration.md.
                    if f.twist is not None:
                        v = float(f.twist[0])
                        omega = float(f.twist[1]) / 1000.0  # [rad/s]
                        half = 0.5 * TRACK * omega
                        log["cmdLeft"].append(v - half)
                        log["cmdRight"].append(v + half)
                    else:
                        log["cmdLeft"].append(float("nan"))
                        log["cmdRight"].append(float("nan"))
                for entry in f.acks:
                    if entry.corr_id in pending_enqueues:
                        move_id, _, _ = pending_enqueues[entry.corr_id]
                        if not entry.ok:
                            print(f"FAIL enqueue move {move_id}: "
                                  f"err={entry.err_code}")
                            return 1
                        del pending_enqueues[entry.corr_id]
                        continue
                    wanted = {m["move_id"] for m in moves[:next_enqueue]}
                    done = {mid for _, mid in completions}
                    if entry.corr_id in wanted and entry.corr_id not in done:
                        completions.append((t, entry.corr_id))
                        inflight -= 1
                        if log["x"]:
                            boundary_poses.append((entry.corr_id, log["x"][-1],
                                                   log["y"][-1],
                                                   log["heading"][-1]))
                        print(f"  move {entry.corr_id} complete at t={t:.1f}s")
            if len(completions) == len(moves):
                break
            time.sleep(0.02)
        if args.settle > 0.0 and not args.sequential:
            pose = settle(args.settle)
            if pose is not None:
                rest_poses.append(pose)
    finally:
        proto.estop()
        time.sleep(0.5)
        proto.read_pending_binary_tlm_frames()
        try:
            proto.tlmOff()
        except Exception:
            pass
        conn.disconnect()

    if pending_enqueues:
        print("WARNING: enqueue acks never seen for: "
              f"{sorted(v[0] for v in pending_enqueues.values())}")
    missing = {m['move_id'] for m in moves} - {mid for _, mid in completions}
    if missing:
        print(f"WARNING: never completed: {sorted(missing)}")
    ok = len(completions) == len(moves)
    if not log["t"]:
        print("FAIL: no telemetry captured")
        return 1
    d_left = log["posLeft"][-1] - enc_start[0]
    d_right = log["posRight"][-1] - enc_start[1]
    path = 0.5 * (d_left + d_right)
    heading_enc = math.degrees((d_right - d_left) / TRACK)
    wall = log["t"][-1]
    print(f"\n{'PASS' if ok else 'FAIL'}: {len(completions)}/{len(moves)} moves "
          f"completed in {wall:.1f} s")
    print(f"  encoders: dL {d_left:+.1f} dR {d_right:+.1f} mm -> "
          f"path {path:.1f} mm (target {4 * LEG:.0f}; error {path - 4 * LEG:+.1f})")
    print(f"  heading (encoder differential) {heading_enc:.1f} deg "
          f"(target 360; error {heading_enc - 360.0:+.1f})")

    # Closure: how far the pose ended from where it started, and how far
    # the final heading is from a full 360 turn. Measured between the two
    # REST poses when --settle > 0 (wheels_square_tour.py's basis), else
    # between the first and last telemetry samples (the old basis).
    if len(rest_poses) >= 2:
        x0, y0, h0 = rest_poses[0]
        x1, y1, h1 = rest_poses[-1]
        basis = f"rest poses, {args.settle:.1f}s settle"
    else:
        x0, y0, h0 = log["x"][0], log["y"][0], log["heading"][0]
        x1, y1, h1 = log["x"][-1], log["y"][-1], log["heading"][-1]
        basis = "first/last telemetry sample (no settle)"
    closure = math.hypot(x1 - x0, y1 - y0)
    heading_sweep = h1 - h0
    print(f"  pose closure {closure:.1f} mm from start "
          f"(finish at {x1 - x0:+.1f}, {y1 - y0:+.1f}) [{basis}]")
    print(f"  heading sweep {heading_sweep:+.1f} deg (target +360)")

    # Per-segment truth, from the pose sampled at each completion ack. The
    # planner never rests between moves, so these are IN-MOTION boundaries.
    seg_lengths: "list[float]" = []
    seg_turns: "list[float]" = []
    seg_drift: "list[float]" = []
    if len(boundary_poses) == 8:
        chain = [(x0, y0, h0)] + [(bx, by, bh) for _, bx, by, bh in boundary_poses]
        for i in range(4):
            a = chain[2 * i]
            b = chain[2 * i + 1]
            c = chain[2 * i + 2]
            seg_lengths.append(math.hypot(b[0] - a[0], b[1] - a[1]))
            seg_turns.append(c[2] - b[2])
            seg_drift.append(b[2] - a[2])
        print("  per-leg length [mm]: "
              + "  ".join(f"{d:6.1f}" for d in seg_lengths)
              + f"   (target {LEG:.0f})")
        print("  per-turn angle [deg]: "
              + "  ".join(f"{d:+6.1f}" for d in seg_turns) + "   (target +90)")
        print("  in-leg heading drift [deg]: "
              + "  ".join(f"{d:+6.1f}" for d in seg_drift))

    if args.outdir:
        import csv as _csv
        import pathlib as _pathlib
        outdir_p = _pathlib.Path(args.outdir)
        outdir_p.mkdir(parents=True, exist_ok=True)
        results = outdir_p / "planner_tour_results.csv"
        new = not results.exists()
        with open(results, "a", newline="") as fh:
            w = _csv.writer(fh)
            if new:
                w.writerow(["label", "cruise", "omega", "closure",
                            "heading_sweep", "path", "wall",
                            "l1", "l2", "l3", "l4", "t1", "t2", "t3", "t4"])
            w.writerow([args.label, args.cruise, args.omega,
                        f"{closure:.1f}", f"{heading_sweep:+.1f}",
                        f"{path:.1f}", f"{wall:.1f}",
                        *([f"{d:.1f}" for d in seg_lengths] or [""] * 4),
                        *([f"{d:+.1f}" for d in seg_turns] or [""] * 4)])
        print(f"  results: {results}")

        # Per-run record, same schema as the wheels arm's, so both arms
        # analyze through one reader.
        import json as _json
        nudge_total = sum(c["nudge_count"] for c in corners)
        trim_total = sum(c["trim_seconds"] for c in corners)
        if corners:
            resid_post = [c["residual_post_trim"] for c in corners]
            print("  cumulative residual after trim [deg]: "
                  + "  ".join(f"{r:+5.2f}" for r in resid_post)
                  + f"   mean|r| {sum(abs(r) for r in resid_post) / 4.0:.2f}")
            print(f"  trim: {nudge_total} nudges / {trim_total:.1f}s over 4 "
                  f"corners; tour wall {wall:.1f}s")
        run = dict(label=args.label,
                   arm="planner-sequential" if args.sequential else "planner",
                   started_wall=started_wall,
                   started_iso=time.strftime("%Y-%m-%dT%H:%M:%S",
                                             time.localtime(started_wall)),
                   settle=args.settle, settle_mode=args.settle_mode,
                   trim=bool(args.trim), trim_tol=args.trim_tol,
                   trim_max_nudges=args.trim_max_nudges,
                   cruise=args.cruise, omega=args.omega,
                   closure=closure, heading_sweep=heading_sweep,
                   wall=wall, nudge_total=nudge_total,
                   trim_seconds=trim_total,
                   seg_lengths=seg_lengths, turn_deltas=seg_turns,
                   leg_heading_drift=seg_drift,
                   rest_poses=[list(p) for p in rest_poses],
                   start_rest_stats=start_stats, corners=corners)
        jpath = outdir_p / f"trimtol_{args.label}.json"
        jpath.write_text(_json.dumps(run, indent=1))
        print(f"  run record: {jpath}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(15, 10))
    grid = fig.add_gridspec(2, 2, height_ratios=[1.15, 1.0],
                            hspace=0.3, wspace=0.2)

    # --- Wheel speeds: commanded vs measured, both wheels ----------------
    ax = fig.add_subplot(grid[0, :])
    ax.plot(log["t"], log["cmdLeft"], color="#1f77b4", lw=1.0, ls="--",
            label="left, twist-derived (MEASURED, not commanded)")
    ax.plot(log["t"], log["velLeft"], color="#1f77b4", lw=1.4,
            label="left measured (encoder)")
    ax.plot(log["t"], log["cmdRight"], color="#d62728", lw=1.0, ls="--",
            label="right, twist-derived (MEASURED, not commanded)")
    ax.plot(log["t"], log["velRight"], color="#d62728", lw=1.4,
            label="right measured (encoder)")
    id_base = moves[0]["move_id"] - 1  # 127-002: IDs are now dynamic (tour()'s own id_base), not
    # hardcoded at 9001 -- recover the same base from the first move actually sent this run.
    for tc, move_id in completions:
        ax.axvline(tc, color="#888888", lw=0.6, alpha=0.5)
        kind = "leg" if move_id % 2 == 1 else "turn"
        ax.annotate(f"{kind} {(move_id - id_base + 1) // 2}",
                    xy=(tc, 0.98), xycoords=("data", "axes fraction"),
                    fontsize=7.5, rotation=90, va="top", ha="right",
                    color="#666666")
    lim = 1.15 * max(max(abs(v) for v in log["velLeft"]),
                     max(abs(v) for v in log["velRight"]), args.cruise)
    ax.set_ylim(-lim, lim)
    ax.axhline(0.0, color="black", lw=0.5)
    ax.axhline(args.cruise, color="#cccccc", lw=0.7, ls=":")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("wheel speed [mm/s]")
    ax.legend(loc="lower right", fontsize=8, ncol=2, framealpha=0.9)
    ax.grid(True, alpha=0.25)
    ax.set_title("Wheel speed: measured (solid) vs the same signal via body twist (dashed) -- NO commanded telemetry exists",
                 fontsize=10)

    # --- The motion (encoder-odometry pose) ------------------------------
    ax2 = fig.add_subplot(grid[1, 0])
    xs = [x - x0 for x in log["x"]]
    ys = [y - y0 for y in log["y"]]
    ideal = [(0, 0), (LEG, 0), (LEG, LEG), (0, LEG), (0, 0)]
    ax2.plot([p[0] for p in ideal], [p[1] for p in ideal], color="#bbbbbb",
             ls="--", lw=1.0, label="ideal square")
    ax2.plot(xs, ys, color="#2b8a3e", lw=1.6)
    ax2.plot([0], [0], marker="o", color="#2b8a3e", ms=7, label="start")
    ax2.plot([xs[-1]], [ys[-1]], marker="X", color="#d62728", ms=10,
             label="finish")
    ax2.annotate(f"closure {closure:.1f} mm", xy=(xs[-1], ys[-1]),
                 xytext=(10, -14), textcoords="offset points",
                 fontsize=9, color="#d62728")
    ax2.set_aspect("equal", adjustable="datalim")
    ax2.set_xlabel("x [mm]")
    ax2.set_ylabel("y [mm]")
    ax2.legend(loc="best", fontsize=8)
    ax2.grid(True, alpha=0.25)
    ax2.set_title("Path (encoder odometry pose)", fontsize=10)

    # --- Heading vs time -------------------------------------------------
    ax3 = fig.add_subplot(grid[1, 1])
    ax3.plot(log["t"], log["heading"], color="#6741d9", lw=1.4)
    for i in range(1, 5):
        ax3.axhline(90.0 * i, color="#cccccc", lw=0.7, ls=":")
    ax3.set_xlabel("time [s]")
    ax3.set_ylabel("heading [deg]")
    ax3.grid(True, alpha=0.25)
    ax3.set_title("Heading (encoder odometry) -- gridlines at n*90 deg",
                  fontsize=10)

    fig.suptitle(
        f"ON-ROBOT square tour -- onboard Motion::Planner "
        f"(per-wheel profiler + velocity trim), real encoders\n"
        f"{len(completions)}/{len(moves)} moves   "
        f"path {path:.1f}/{4 * LEG:.0f} mm ({path - 4 * LEG:+.1f})   "
        f"heading {heading_enc:.1f}/360 deg ({heading_enc - 360.0:+.1f})   "
        f"pose closure {closure:.1f} mm",
        fontsize=12)
    if args.outdir:
        out = str(pathlib.Path(args.outdir) / f"planner_square_tour_{args.label}.png")
    else:
        out = args.out or (__file__.rsplit("/", 1)[0] + "/planner_square_tour.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"wrote {out}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
