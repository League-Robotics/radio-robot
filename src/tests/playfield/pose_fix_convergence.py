#!/usr/bin/env python3
"""pose_fix_convergence.py -- aprilcam end-to-end playfield script (099-009).

The sprint's headline capability, demonstrated end to end: **PING clock-sync
-> tag-pose-to-PoseFix send -> convergence check**. This is the only tier
where the full camera -> binary wire -> `EkfTiny` -> fused-pose loop
actually closes (`clasi/sprints/099-.../sprint.md`'s own Success Criteria;
`architecture-update.md` D5-D8).

Why `src/tests/playfield/`, not `src/tests/bench/` (see `tests/CLAUDE.md`'s
three-domain split): step 2 below -- "observe the robot's true pose" --
needs the camera's CALIBRATED WORLD-FRAME reading (`tag.world_xy` in the
A1-centred cm frame), not a static/known bench pose. A bench script has no
camera-truth source to fix the robot's world pose AGAINST; this
capability's entire point is closing the camera-loop, so it belongs where
the camera's world calibration lives.

Precedent: `tests_old/bench/playfield_camera_run.py` is the closest
pre-rebuild reference for the connect/camera/geofence/safe-stop shape (its
own header flags "square run" motion/odometry as the precondition that had
to come back before it could run again -- this sprint restores exactly
that: `PoseEstimator` ticking live, D5-D8's camera-fix). It predates
Protocol v3 (sprint 095-097) though, so its own drive verbs (`RT`/`G`/`X`)
are sent as raw TEXT -- dead against this firmware (`docs/protocol-v3.md`
S6: only `HELP`/`HELLO`/`PING`/`ID`/`VER`/`STOP` still parse as text). This
script instead drives entirely through `NezhaProtocol`'s BINARY methods
(`ping()`, `distance()`, `stop()`, `set_config()`, `stream()`,
`pose_fix()` -- the last one added by this ticket, see
`src/host/robot_radio/robot/protocol.py`), never raw text, never lock-step
pyserial (prior bench-session lesson, `tests/CLAUDE.md`).

Sequence
--------
1. Connect: robot via `robot_radio.io.serial_conn.SerialConnection`
   (auto-detects direct-USB vs. the radio relay's `!GO` data plane from the
   boot `DEVICE:` banner) + aprilcam daemon (`DaemonControl`).
2. **PING clock-sync** (`clock_sync_burst()`): a burst of binary `ping`
   round trips, recorded into a `ClockSync` (`src/host/robot_radio/robot/
   clock_sync.py`) -- NTP-style min-RTT filtering + skew regression,
   translating host-monotonic time <-> robot-clock time (D6: "host maps
   clocks via PING t= + RTT/2").
3. **Geofence + hop-test precondition** (per `clasi/knowledge/
   vision-geofence-before-driving.md` -- NEVER blind-drive the
   playfield): confirm the robot tag is inside the calibrated playfield
   bounds (inset by `--margin`), then run one small, camera-observed
   forward hop before any extended run. Surface is "playfield," never
   "floor" (`clasi/knowledge/playfield-not-floor.md`).
4. **Tag-pose -> PoseFix send**: read the robot's tag pose via the aprilcam
   daemon (`get_tags`/`dc.list_mobile_tags()` -- `register_mobile_tag()` is
   called once, idempotently, from the active robot config's own
   `vision.tag_offset_mm` if the tag is not yet registered as mobile, so
   `world_xy` reports the robot's CENTRE, not the raw tag), convert to the
   robot's world-frame convention (cm -> mm, radians unchanged -- see
   `camera_pose_to_pose_fix_kwargs()`), and build a `PoseFix{x, y, h, t}`
   with `t` derived from the clock-sync mapping
   (`ClockSync.to_robot_time()`). Sent via `NezhaProtocol.pose_fix()`
   (binary `CommandEnvelope` arm 7 -- see `protocol.py`).
5. **Convergence check** (`wait_for_pose_convergence()`): poll the binary
   `stream` arm for `pose=` (the fused EKF belief), confirm it moves toward
   the camera-observed target within `--tol-mm`/`--tol-deg` inside
   `--converge-timeout` seconds. The camera-fix EKF update (D5) is a
   WEIGHTED Kalman update against `ekf_r_fix_xy`/`ekf_r_fix_theta`, not a
   hard snap -- the tolerance/timeout below are deliberately generous
   (documented, not tuned-to-pass) to allow for that.

Safety (`tests/CLAUDE.md`'s HITL conventions)
----------------------------------------------
Widens the binary config watchdog (`SET sTimeout=<ms>`-equivalent,
`NezhaProtocol.set_config(sTimeout=...)`) for the run; ALWAYS restores it
and sends a binary `stop()` in a `finally` block -- motors are never left
running on an exception or Ctrl-C. Resilient to a single dropped/late
camera frame: `read_cam_pose()` retries within its own timeout window and
returns `None` (never raises) on genuine loss, and every camera read in the
main sequence checks for `None` and reports clearly rather than crashing.

Usage
-----
    uv run python src/tests/playfield/pose_fix_convergence.py \\
        --port /dev/cu.usbmodem2121102

    # Safe pipeline check -- camera + clock-sync + geofence only, no motion,
    # no PoseFix send:
    uv run python src/tests/playfield/pose_fix_convergence.py --plan-only
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_HOST_DIR = _REPO_ROOT / "src" / "host"
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))

from robot_radio.io.serial_conn import SerialConnection  # noqa: E402
from robot_radio.robot.clock_sync import ClockSync  # noqa: E402
from robot_radio.robot.protocol import NezhaProtocol  # noqa: E402

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
DEFAULT_TAG_ID = 100          # matches data/robots/tovez.json's vision.robot_tag_id
RUN_WATCHDOG_MS = 5000        # [ms] widened for the run
BOOT_WATCHDOG_MS = 1000       # [ms] firmware default -- restored on exit
DEFAULT_MARGIN = 14.0         # [cm] geofence inset from the playfield's ArUco-corner extent
DEFAULT_HOP = 60              # [mm] hop-test forward distance
DEFAULT_HOP_SPEED = 80        # [mm/s] hop-test speed
DEFAULT_TOL_POS = 30.0        # [mm] documented convergence position tolerance
DEFAULT_TOL_HEADING = 3.0     # [deg] documented convergence heading tolerance
DEFAULT_CONVERGE_TIMEOUT = 5.0   # [s] documented convergence time bound
DEFAULT_PING_COUNT = 5
STREAM_PERIOD = 50             # [ms] ~20 Hz TLM poll during the convergence check


# ===========================================================================
# Pure logic -- no I/O, no hardware/camera reference. Unit-tested by
# src/tests/unit/test_pose_fix_convergence_pure.py without a robot or camera.
# ===========================================================================

def wrap_deg(angle: float) -> float:  # [deg]
    """Wrap an angle (deg) to [-180, 180) -- same formula as `wrap()` in
    `tests_old/bench/playfield_camera_run.py`."""
    return (angle + 180.0) % 360.0 - 180.0


def camera_pose_to_pose_fix_kwargs(x: float, y: float, heading: float,
                                   t: float) -> dict:  # [cm] [cm] [rad] [ms]
    """Convert an aprilcam world pose to `NezhaProtocol.pose_fix()` kwargs.

    aprilcam's `world_xy` is centimetres, A1-centred; `PoseFix.x`/`.y`
    (`protos/drivetrain.proto`) are millimetres -- a factor-of-10 scale, no
    axis change (both are +x east, +y north). aprilcam's `yaw` and
    `PoseFix.h` are BOTH radians, 0 = east, CCW-positive -- no angular
    conversion needed at all (unlike the old text-plane `SI`'s
    centidegrees; see `robot_radio.robot.sync_pose.pose_to_setpose_line()`
    for that now-superseded conversion). `t` is passed through as the
    already-robot-clock-mapped millisecond timestamp (the caller derives it
    via `ClockSync.to_robot_time()` from a host-side capture time) and
    rounded to the nearest integer (`PoseFix.t` is a wire `uint32`).
    """
    return {
        "x": x * 10.0,
        "y": y * 10.0,
        "h": heading,
        "t": int(round(t)),
    }


def pose_fix_target_mm_cdeg(x: float, y: float,
                            heading: float) -> tuple[float, float, float]:  # [cm][cm][rad]
    """Convert an aprilcam world pose to the SAME (x_mm, y_mm, h_cdeg) shape
    `TLMFrame.pose`/`TLMFrame.otos` report, for direct comparison in
    `pose_error()`. Mirrors `protocol.py`'s own `_ANGLE_SCALE`
    (rad -> cdeg, `kAngleScale` mirror) exactly."""
    return (x * 10.0, y * 10.0, math.degrees(heading) * 100.0)


def pose_error(observed: tuple[float, float, float],
               target: tuple[float, float, float],
               ) -> tuple[float, float]:  # -> (distance [mm], heading_error [deg])
    """Pure comparison between an observed `TLMFrame.pose`-shaped 3-tuple
    (x_mm, y_mm, h_cdeg) and a target of the SAME shape. Returns the
    Euclidean position error (mm) and the absolute wrapped heading error
    (deg)."""
    dx = observed[0] - target[0]
    dy = observed[1] - target[1]
    distance = math.hypot(dx, dy)
    heading_error = abs(wrap_deg((observed[2] - target[2]) / 100.0))
    return distance, heading_error


def pose_converged(distance: float, heading_error: float,  # [mm] [deg]
                   tol_pos: float, tol_heading: float) -> bool:  # [mm] [deg]
    """Pure tolerance check: both position and heading error must be within
    their documented bound (see module docstring's Convergence check step
    for why the default tolerances are deliberately generous -- the
    camera-fix EKF update is a weighted Kalman update, not a hard snap)."""
    return distance <= tol_pos and heading_error <= tol_heading


def geofence_from_playfield(playfield: dict,
                            margin: float) -> tuple[float, float, float, float]:  # [cm]
    """Geofence (x_lo, x_hi, y_lo, y_hi) cm = the playfield's ArUco-corner
    extent inset by `margin`. Ported from `tests_old/bench/
    world_goto_chart.py`'s `geofence_from_playfield()` (pure, unchanged
    logic) -- the table IS the calibrated playfield: its ArUco corner
    markers bound the drivable surface, and insetting by `margin` keeps the
    robot tag (hence the robot body) clear of the edge."""
    xs = [float(u["x"]) for u in playfield.get("aruco_tags", [])]
    ys = [float(u["y"]) for u in playfield.get("aruco_tags", [])]
    if not xs or not ys:
        raise ValueError("playfield has no aruco_tags to derive a geofence from")
    return (min(xs) + margin, max(xs) - margin,
            min(ys) + margin, max(ys) - margin)


def in_fence(x: float, y: float,
            fence: tuple[float, float, float, float]) -> bool:  # [cm]
    """True when (x, y) is inside `fence` (x_lo, x_hi, y_lo, y_hi), cm."""
    x_lo, x_hi, y_lo, y_hi = fence
    return x_lo <= x <= x_hi and y_lo <= y <= y_hi


# ===========================================================================
# Hardware/camera-dependent helpers -- never called at import time. Every
# call site below is reached only from main() (guarded by
# `if __name__ == "__main__":`) or from a function a caller must invoke
# explicitly, so `import pose_fix_convergence` alone touches no hardware.
# ===========================================================================

def open_daemon(cam_pattern: str | None, cam_index: int | None):
    """Connect to the aprilcam daemon and return (dc, cam_name).

    Mirrors `tests_old/bench/world_goto_chart.py`'s `open_daemon()` --
    aprilcam imports deferred to this function (never at module import
    time), matching `robot_radio.robot.sync_pose`'s own documented
    "importable without aprilcam installed" posture, even though aprilcam
    is an ordinary project dependency here (`pyproject.toml`) -- deferring
    keeps a bare `import pose_fix_convergence` free of any daemon/camera
    I/O regardless.
    """
    from aprilcam.client.control import DaemonControl
    from aprilcam.config import Config

    dc = DaemonControl.connect_default(Config.load())
    if cam_pattern is not None or cam_index is not None:
        cam = dc.open_camera(pattern=cam_pattern, index=cam_index or 0)
        return dc, cam
    cams = dc.list_cameras()
    if cams:
        return dc, cams[0]
    cam = dc.open_camera(index=cam_index or 0)
    return dc, cam


def playfield_from_daemon(dc, name: str | None = None) -> dict:
    """Fetch the playfield map from the aprilcam daemon (no local file
    needed). Ported from `tests_old/bench/world_goto_chart.py`'s
    `playfield_from_daemon()` (unchanged logic)."""
    import json
    resp = dc.list_playfields()
    entries = list(resp.playfields)
    if not entries:
        raise RuntimeError(
            "aprilcam daemon has no playfield defined -- calibrate one first "
            "(mcp__aprilcam__calibrate_playfield)")
    entry = entries[0]
    if name is not None:
        match = next((e for e in entries if e.name == name), None)
        if match is None:
            avail = ", ".join(e.name for e in entries) or "(none)"
            raise RuntimeError(f"playfield {name!r} not found on daemon; available: {avail}")
        entry = match
    return json.loads(entry.json_blob)


def ensure_mobile_tag_registered(dc, tag_id: int) -> None:
    """Register `tag_id` as a mobile tag (robot centre, not bare AprilTag
    centre) from the active robot config's own `vision.tag_offset_mm`, if
    it is not already registered. Idempotent -- `register_mobile_tag()` is
    a persisted, once-only call per the aprilcam API guide; a robot whose
    tag is already registered (e.g. `tovez`, tag 100 -- verified via
    `list_mobile_tags` during this ticket's implementation) is a no-op
    here."""
    from robot_radio.config import get_robot_config

    already = {m["tag_id"] for m in dc.list_mobile_tags()}
    if tag_id in already:
        return
    cfg = get_robot_config()
    if cfg is None:
        print(f"  WARN: no active robot config -- cannot auto-register tag {tag_id} "
              f"as mobile; world_xy will report the raw tag, not the robot centre")
        return
    offset = cfg.vision.tag_offset_mm
    dc.register_mobile_tag(tag_id, x_mm=offset.x, y_mm=offset.y,
                           z_cm=offset.z / 10.0, yaw_deg=math.degrees(offset.yaw_rad),
                           owner=cfg.identity.robot_name)
    print(f"  registered tag {tag_id} as mobile (owner={cfg.identity.robot_name!r}, "
          f"offset x={offset.x}mm y={offset.y}mm z={offset.z}mm)")


def read_cam_pose(dc, cam, tag_id: int, timeout_s: float = 2.0,
                  ) -> tuple[float, float, float] | None:  # -> (x_cm, y_cm, yaw_rad) | None
    """Poll the aprilcam daemon for `tag_id`'s calibrated world pose.

    Resilient to a single dropped/late camera frame: `dc.get_tags()`
    itself may raise (a transient daemon/gRPC hiccup) or simply not yet
    have `tag_id` in its latest frame (occluded/motion-blurred) -- both are
    caught/retried in a plain poll loop within `timeout_s`, never
    propagated as a crash. Returns `None` (not an exception) if the tag is
    not seen with a calibrated `world_xy` before the deadline -- callers
    must check for `None` and report clearly (never assume a reading
    succeeded).
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            tag_frame = dc.get_tags(cam)
        except Exception as exc:  # noqa: BLE001 -- one dropped frame must not crash the script
            print(f"  (camera read hiccup, retrying: {exc})")
            time.sleep(0.05)
            continue
        for tag in tag_frame.tags:
            if tag.id == tag_id and tag.world_xy is not None and tag.yaw is not None:
                return float(tag.world_xy[0]), float(tag.world_xy[1]), float(tag.yaw)
        time.sleep(0.03)
    return None


def clock_sync_burst(proto: NezhaProtocol, cs: ClockSync, n: int = DEFAULT_PING_COUNT) -> None:
    """Fire `n` binary `ping` round trips and record each into `cs`.

    `NezhaProtocol.ping()` already sends `CommandEnvelope{ping: Ping{}}` and
    returns `(t_robot, rtt)` in ms -- this wrapper brackets each call with
    its own host-monotonic `t0`/`t1` (ms) so `ClockSync.record_ping()` gets
    the SAME `(t0, t1, t_robot)` triple its offset/skew math is built
    around (D6: "host maps clocks via PING t= + RTT/2"), reusing
    `ClockSync`'s existing min-RTT-filtered estimator rather than
    re-deriving one. A single failed ping (`ping()` returns `None` on
    timeout) is skipped, not fatal -- mirrors `ClockSync.ping_burst()`'s
    own "skip failures, keep the survivors" posture for its text-plane
    sibling.
    """
    for _ in range(n):
        t0 = time.monotonic() * 1000.0
        result = proto.ping()
        t1 = time.monotonic() * 1000.0
        if result is None:
            continue
        t_robot, _rtt = result
        cs.record_ping(t0=t0, t1=t1, t_robot=float(t_robot))


def hop_test(proto: NezhaProtocol, dc, cam, tag_id: int,
            fence: tuple[float, float, float, float],
            hop: int = DEFAULT_HOP, speed: int = DEFAULT_HOP_SPEED,
            ) -> tuple[bool, str]:
    """Small, camera-observed forward hop -- the geofence/hop-test
    precondition `clasi/knowledge/vision-geofence-before-driving.md`
    requires before any extended playfield run. Confirms the robot is
    inside `fence` before moving, drives a short bounded `distance()`
    segment (self-terminating -- no watchdog dependency), polls the camera
    for a geofence breach WHILE it runs (stopping immediately if one
    occurs), and reports whether the robot moved roughly the expected
    amount and stayed on the field.
    """
    before = read_cam_pose(dc, cam, tag_id, timeout_s=2.0)
    if before is None:
        return False, "no camera fix before hop -- aborting, no motion sent"
    x0, y0, _yaw0 = before
    if not in_fence(x0, y0, fence):
        return False, (f"robot at ({x0:+.1f},{y0:+.1f}) cm is OUTSIDE the geofence "
                       f"before the hop -- aborting, no motion sent")

    proto.distance(speed, speed, hop)
    deadline = time.monotonic() + max(2.0, (hop / max(speed, 1)) * 3.0 + 1.0)
    breached = False
    while time.monotonic() < deadline:
        cur = read_cam_pose(dc, cam, tag_id, timeout_s=0.3)
        if cur is not None and not in_fence(cur[0], cur[1], fence):
            breached = True
            break
        time.sleep(0.05)
    proto.stop()   # always stop at the end of the hop, breach or not
    time.sleep(0.3)

    if breached:
        return False, "hop breached the geofence -- stopped"

    after = read_cam_pose(dc, cam, tag_id, timeout_s=1.5)
    if after is None:
        return False, "no camera fix after hop -- cannot confirm motion"
    x1, y1, _yaw1 = after
    moved = math.hypot(x1 - x0, y1 - y0) * 10.0  # [mm]
    if moved < hop * 0.25:
        return False, f"robot barely moved ({moved:.0f}mm of {hop}mm commanded) -- check link/motors"
    return True, f"moved {moved:.0f}mm of {hop}mm commanded, stayed on field"


def send_camera_pose_fix(proto: NezhaProtocol, cs: ClockSync, dc, cam, tag_id: int,
                         ) -> tuple[bool, str, tuple[float, float, float] | None]:
    """Capture the robot's camera-observed pose, map its capture time onto
    the robot's own clock, and send it as a delayed `PoseFix`.

    Returns `(ok, detail, target_mm_cdeg)` -- `target_mm_cdeg` is the SAME
    `(x_mm, y_mm, h_cdeg)` shape `pose_error()` compares against
    `TLMFrame.pose`, or `None` if the send did not happen at all (no camera
    fix, or clock-sync not yet calibrated).
    """
    host_t = time.monotonic() * 1000.0
    cam_pose = read_cam_pose(dc, cam, tag_id, timeout_s=2.0)
    if cam_pose is None:
        return False, "no camera fix -- PoseFix not sent", None

    robot_t = cs.to_robot_time(host_t)
    if robot_t is None:
        return False, "clock-sync not calibrated yet -- PoseFix not sent", None

    x, y, heading = cam_pose
    kwargs = camera_pose_to_pose_fix_kwargs(x, y, heading, robot_t)
    reply = proto.pose_fix(**kwargs)
    target = pose_fix_target_mm_cdeg(x, y, heading)
    if reply is None:
        return False, "PoseFix send timed out (no reply)", target
    if reply.WhichOneof("body") != "ok":
        return False, f"PoseFix rejected: {reply}", target
    return True, (f"PoseFix sent: x={kwargs['x']:.0f}mm y={kwargs['y']:.0f}mm "
                 f"h={math.degrees(kwargs['h']):.1f}deg t={kwargs['t']}"), target


def wait_for_pose_convergence(proto: NezhaProtocol,
                              target: tuple[float, float, float],  # (x_mm, y_mm, h_cdeg)
                              tol_pos: float = DEFAULT_TOL_POS,  # [mm]
                              tol_heading: float = DEFAULT_TOL_HEADING,  # [deg]
                              timeout_s: float = DEFAULT_CONVERGE_TIMEOUT,
                              ) -> tuple[bool, str]:
    """Poll the binary `stream` arm for `pose=` (the fused EKF belief) and
    report whether it converges toward `target` within tolerance/timeout.

    Streams at `STREAM_PERIOD` (~20 Hz), draining every pending frame each
    pass (`read_pending_binary_tlm_frames()`), tracking the MINIMUM
    position/heading error observed across the whole window -- the fix is
    a weighted Kalman update (D5), so the belief may still be settling when
    the window ends; the minimum captures genuine progress even if the
    final frame is not the closest one. Always disarms streaming before
    returning (mirrors `NezhaProtocol.snap()`'s own arm/disarm discipline).
    """
    proto.stream(STREAM_PERIOD)
    best_pos = math.inf
    best_heading = math.inf
    saw_pose = False
    try:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for frame in proto.read_pending_binary_tlm_frames():
                if frame.pose is None:
                    continue
                saw_pose = True
                distance, heading_error = pose_error(frame.pose, target)
                best_pos = min(best_pos, distance)
                best_heading = min(best_heading, heading_error)
                if pose_converged(distance, heading_error, tol_pos, tol_heading):
                    return True, (f"converged: pos_err={distance:.1f}mm "
                                 f"heading_err={heading_error:.1f}deg "
                                 f"(tol {tol_pos:.0f}mm/{tol_heading:.1f}deg)")
            time.sleep(0.05)
    finally:
        proto.stream(0)

    if not saw_pose:
        return False, "no pose= frames observed during the convergence window"
    return False, (f"did not converge within {timeout_s:.1f}s: best pos_err={best_pos:.1f}mm "
                   f"best heading_err={best_heading:.1f}deg (tol {tol_pos:.0f}mm/{tol_heading:.1f}deg)")


# ===========================================================================
# Orchestration
# ===========================================================================

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--tag-id", type=int, default=None,
                   help=f"AprilTag id for the robot (default: active robot config's "
                        f"vision.robot_tag_id, else {DEFAULT_TAG_ID})")
    p.add_argument("--cam-pattern", default=None, help="Camera name/pattern to open")
    p.add_argument("--cam-index", type=int, default=None, help="OS camera index to open")
    p.add_argument("--margin", type=float, default=DEFAULT_MARGIN,
                   help="Geofence inset (cm) from the playfield's ArUco-corner extent")
    p.add_argument("--hop", type=int, default=DEFAULT_HOP, help="Hop-test distance (mm)")
    p.add_argument("--hop-speed", type=int, default=DEFAULT_HOP_SPEED, help="Hop-test speed (mm/s)")
    p.add_argument("--tol-mm", type=float, default=DEFAULT_TOL_POS,
                   help="Convergence position tolerance (mm)")
    p.add_argument("--tol-deg", type=float, default=DEFAULT_TOL_HEADING,
                   help="Convergence heading tolerance (deg)")
    p.add_argument("--converge-timeout", type=float, default=DEFAULT_CONVERGE_TIMEOUT,
                   help="Convergence time bound (s)")
    p.add_argument("--ping-count", type=int, default=DEFAULT_PING_COUNT,
                   help="Number of PINGs in the clock-sync burst")
    p.add_argument("--plan-only", action="store_true",
                   help="Camera + clock-sync + geofence check only -- no motion, no PoseFix send")
    return p.parse_args()


def main() -> int:
    args = _parse_args()

    conn = SerialConnection(port=args.port)
    proto: NezhaProtocol | None = None
    dc = None
    ok = True

    try:
        print(f"connecting to robot on {args.port} ...")
        info = conn.connect()
        if info.get("error"):
            print(f"  ERROR: robot connect failed: {info['error']}")
            return 2
        print(f"  connected (mode={info.get('mode')})")
        proto = NezhaProtocol(conn)

        print("connecting to aprilcam daemon ...")
        dc, cam = open_daemon(args.cam_pattern, args.cam_index)
        print(f"  camera: {cam}")

        from robot_radio.config import get_robot_config
        cfg = get_robot_config()
        tag_id = args.tag_id
        if tag_id is None:
            tag_id = cfg.vision.robot_tag_id if cfg is not None else DEFAULT_TAG_ID
        ensure_mobile_tag_registered(dc, tag_id)

        playfield = playfield_from_daemon(dc)
        fence = geofence_from_playfield(playfield, args.margin)
        print(f"  geofence: x in [{fence[0]:+.0f},{fence[1]:+.0f}] "
              f"y in [{fence[2]:+.0f},{fence[3]:+.0f}] cm (margin {args.margin:.0f}cm)")

        print(f"\n== PING clock-sync ({args.ping_count} pings) ==")
        cs = ClockSync()
        clock_sync_burst(proto, cs, n=args.ping_count)
        if cs.sample_count == 0:
            print("  FAIL: no PING replies -- cannot proceed")
            return 3
        print(f"  samples={cs.sample_count} min_rtt={cs.min_rtt:.1f}ms "
              f"offset={cs.offset:+.1f}ms skew={cs.skew}")

        print("\n== Geofence precondition ==")
        pose = read_cam_pose(dc, cam, tag_id, timeout_s=3.0)
        if pose is None:
            print(f"  FAIL: camera does not see tag {tag_id} (calibrated) -- aborting")
            return 4
        print(f"  robot at ({pose[0]:+.1f},{pose[1]:+.1f}) cm yaw={math.degrees(pose[2]):+.1f} deg")
        if not in_fence(pose[0], pose[1], fence):
            print("  FAIL: robot is OUTSIDE the geofence -- move it toward the "
                  "playfield centre before running this script")
            return 5
        print("  PASS: robot is inside the geofence")

        if args.plan_only:
            print("\nplan-only: skipping hop-test, PoseFix send, and convergence check.")
            return 0

        print(f"\n== Hop-test ({args.hop}mm @ {args.hop_speed}mm/s) ==")
        proto.set_config(sTimeout=RUN_WATCHDOG_MS)
        hop_ok, hop_msg = hop_test(proto, dc, cam, tag_id, fence,
                                   hop=args.hop, speed=args.hop_speed)
        print(f"  {'PASS' if hop_ok else 'FAIL'}: {hop_msg}")
        if not hop_ok:
            ok = False
            return 6

        print("\n== Tag-pose -> PoseFix send ==")
        send_ok, send_msg, target = send_camera_pose_fix(proto, cs, dc, cam, tag_id)
        print(f"  {'PASS' if send_ok else 'FAIL'}: {send_msg}")
        if not send_ok or target is None:
            ok = False
            return 7

        print(f"\n== Convergence check (tol {args.tol_mm:.0f}mm/{args.tol_deg:.1f}deg, "
              f"timeout {args.converge_timeout:.1f}s) ==")
        conv_ok, conv_msg = wait_for_pose_convergence(
            proto, target, tol_pos=args.tol_mm, tol_heading=args.tol_deg,
            timeout_s=args.converge_timeout)
        print(f"  {'PASS' if conv_ok else 'FAIL'}: {conv_msg}")
        ok = ok and conv_ok
        return 0 if ok else 8

    except KeyboardInterrupt:
        print("\ninterrupted -- stopping motors ...")
        ok = False
        return 130
    finally:
        if proto is not None:
            try:
                proto.stream(0)
            except Exception as exc:  # noqa: BLE001
                print(f"  WARN cleanup stream(0): {exc}")
            try:
                proto.stop()
            except Exception as exc:  # noqa: BLE001
                print(f"  WARN cleanup stop(): {exc}")
            try:
                proto.set_config(sTimeout=BOOT_WATCHDOG_MS)
            except Exception as exc:  # noqa: BLE001
                print(f"  WARN cleanup restore watchdog: {exc}")
            print("  [safety] stream off + STOP + watchdog restored to "
                 f"{BOOT_WATCHDOG_MS}ms.")
        if conn.is_open:
            conn.disconnect()
        if dc is not None:
            try:
                dc.close()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    sys.exit(main())
