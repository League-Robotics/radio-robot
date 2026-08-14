"""runner -- the system-test executor: one backend, one destructive
telemetry drain, one record stream.

Executes a parsed Tour (tourfile.py) against a backend (sim today;
hardware backend follows the same surface), recording every wire line both
directions through the Recorder (recorder.py). EXPECT assertions evaluate
against the same in-process record stream the JSONL writer consumes.

Key invariants (from the bench-verified lineage this replaces):
- Exactly ONE destructive drain of the telemetry queue (the executor's
  ack-matching loop). The recorder observes frames via the backend's
  parallel per-frame callback, never by draining.
- Move completion is the ack RING scanned for Move.id (121-002's fix),
  never a timed settle.
- One-leg lookahead inside a motion segment: leg N+1 enqueues while leg N
  is active (first leg replace=True, rest replace=False).
- The safety path is estop(), never the planned stop() (2026-07-29
  measurement: a mid-leg stop() drives the whole leg first).
"""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import jq

sys.path.insert(0, str(Path(__file__).resolve().parent))
from recorder import Recorder  # noqa: E402
from tourfile import (  # noqa: E402
    CamfixStep, DbgStep, DwellStep, ExpectStep, MoveStep, SendStep, SplineStep,
    StopStep, Tour,
)
import splinefile  # noqa: E402

# Move.id base. 1<<20 was chosen to stay clear of auto-assigned corr_ids, but
# on hardware SOME ids are rejected outright -- the Move acks and then never
# goes ACTIVE. Measured 2026-08-14, alternating drive direction so a table edge
# could not confound it:
#     id 0        drives (-137 enc)      id 1048577  DEAD (x2)
#     id 100000   drives (+137 enc)      id 65535    DEAD
# Not a simple magnitude threshold, and not characterised further here. 1000 is
# in the range that demonstrably drives, and still clear of the small
# auto-assigned corr_ids that ack matching has to discriminate against.
_MOVE_ID_BASE = 1000
_ACTIVE_BIT = 1 << 2  # telemetry flags: kFlagActive
_POLL = 0.02  # [s] ack-poll interval
_STOP_SETTLE_FRAMES = 2  # consecutive inactive frames = at rest
_STOP_TIMEOUT = 15.0  # [s] bounded wait for a planned stop to land
_BOX_X, _BOX_Y = 611.0, 386.0  # [mm] drivable box for the robot centre


class TourFailure(Exception):
    """A tour step failed; the run stops (after estop) and reports."""


@dataclass
class StepResult:
    line_no: int
    kind: str
    ok: bool
    detail: str = ""


@dataclass
class TourRunResult:
    tour: str
    ok: bool
    steps: list[StepResult] = field(default_factory=list)

    def summary(self) -> str:
        n_ok = sum(1 for s in self.steps if s.ok)
        status = "PASS" if self.ok else "FAIL"
        lines = [f"{status} {self.tour}: {n_ok}/{len(self.steps)} steps"]
        for s in self.steps:
            mark = "ok " if s.ok else "FAIL"
            lines.append(f"  [{mark}] line {s.line_no:3d} {s.kind}"
                         + (f" -- {s.detail}" if s.detail else ""))
        return "\n".join(lines)


class SimBackend:
    """Drives the real firmware through SimLoop. Satisfies the executor's
    backend surface: move/stop/estop/read_pending_frames/send_line/
    true_pose/firmware_version/close."""

    name = "sim"

    def __init__(self, robot_config: str, *, recorder: Recorder,
                 speed_factor: int = 1):
        from robot_radio.config.robot_config import load_robot_config
        from robot_radio.io.sim_loop import SimLoop

        self._loop = SimLoop()
        self._loop.on_telemetry = recorder.rx_tlm
        self._loop.on_debug = recorder.rx_line
        self._loop.on_cleartext = recorder.rx_line
        self._loop.connect(start_tick_thread=True)
        self._loop.configure_from_robot(load_robot_config(robot_config))
        if speed_factor > 1:
            self._loop.set_speed_factor(speed_factor)

    def move(self, **kwargs: Any) -> int:
        return self._loop.move(**kwargs)

    def stop(self) -> int:
        return self._loop.stop()

    def estop(self) -> int:
        return self._loop.estop()

    def read_pending_frames(self) -> list:
        return self._loop.read_pending_binary_tlm_frames()

    def send_line(self, line: str) -> None:
        """Inject one cleartext wire line. NO terminator: the sim's
        inject path treats the whole buffer as one line (verified: an
        appended 0x0A reaches the firmware verb/data parser as a literal
        byte -- 'STATUS\\n' fails the registry lookup entirely)."""
        self._loop.inject_command(line.encode("ascii"))

    def true_pose(self) -> dict | None:
        return self._loop.get_true_pose()

    def firmware_version(self) -> str:
        try:
            return self._loop.firmware_version()
        except Exception:
            return "?"

    def enable_telemetry(self) -> None:
        pass  # sim pushes telemetry while active; kAuto suffices

    def close(self) -> None:
        try:
            self._loop.estop()
        except Exception:
            pass
        self._loop.disconnect()


def _move_kwargs(step: MoveStep, move_id: int, *, replace: bool) -> dict:
    kw: dict[str, Any] = {"timeout": step.timeout * 1000.0,  # [ms]
                          "replace": replace, "id": move_id}
    if step.variant == "wheels":
        kw["v_left"] = step.v_left
        kw["v_right"] = step.v_right
    else:
        kw["v_x"] = step.v_x
        kw["v_y"] = step.v_y
        kw["omega"] = step.omega
    if step.stop_kind == "time":
        kw["stop_time"] = step.stop_value * 1000.0  # [ms]
    elif step.stop_kind == "dist":
        kw["stop_distance"] = step.stop_value  # [mm]
    else:
        kw["stop_angle"] = step.stop_value  # [rad]
    return kw


class HardwareBackend:
    """Drives the REAL robot over serial/relay. Same backend surface as
    SimBackend, so the executor and every tour file are unchanged.

    true_pose() comes from the overhead camera when one is reachable -- it is
    the independent check CAMFIX needs, and the robot's own odometry cannot
    serve as its own validator. If no camera is available the backend still
    runs; CAMFIX steps then report unavailable rather than silently passing.
    """

    name = "hardware"

    def __init__(self, port: str, *, recorder: Recorder, camera: bool = True):
        from robot_radio.io.serial_conn import SerialConnection
        from robot_radio.robot.protocol import NezhaProtocol

        self._conn = SerialConnection(port=port)
        self._p = NezhaProtocol(self._conn)
        self._rec = recorder
        self._conn.connect()
        # 1.5 s, NOT the 6 s a boot-race theory once suggested. A long idle
        # before TLM:ON lets the inbound buffer fill, and every Move issued
        # afterwards is acked and then never executed -- the exact
        # "acks but never goes ACTIVE" symptom, which this delay CAUSED rather
        # than cured. With 1.5 s, wheels and twists both drive normally on the
        # same connection (measured 2026-08-14: 4/4 commands, ~120 enc counts
        # each).
        time.sleep(1.5)
        self._ensure_data_plane(port)
        self._p.tlmOn()
        time.sleep(0.8)
        self._cam = None
        self._dc = None
        self._skip_guard = False
        # Tag id and its offset come from the ROBOT CONFIG, never a literal:
        # tovez moved from tag 100 (centre-mounted) to tag 52 (front-mounted,
        # 113 mm up) on 2026-08-14, and a hardcoded 100 silently reports "no
        # tag" -- which the pre-move guard correctly turns into a refusal to
        # drive, i.e. the whole suite stops with no obvious reason.
        self._tag_id = 100
        try:
            from robot_radio.config.robot_config import load_robot_config
            cfg = load_robot_config("data/robots/tovez.json")
            self._tag_id = int(cfg.vision.robot_tag_id)
            self._tag_cfg = cfg
        except Exception:
            self._tag_cfg = None
        if camera:
            try:
                from aprilcam.config import Config
                from aprilcam.client.control import DaemonControl
                self._dc = DaemonControl.connect_default(Config.load())
                self._cam = self._dc.list_cameras()[0]
                # Register the mobile-tag offset FROM THE FILE so world_xy
                # reports the robot's centre of rotation, not the raw tag.
                # The daemon keeps no mobile-tag state across restarts.
                if self._tag_cfg is not None:
                    v = self._tag_cfg.vision
                    already = {m["tag_id"] for m in self._dc.list_mobile_tags()}
                    if self._tag_id not in already:
                        self._dc.register_mobile_tag(
                            self._tag_id, x_mm=v.tag_offset_x, y_mm=v.tag_offset_y,
                            z_cm=v.tag_offset_z / 10.0, yaw_deg=v.tag_offset_yaw,
                            owner=self._tag_cfg.identity.robot_name)
            except Exception:
                self._dc = self._cam = None

    def _ensure_data_plane(self, port: str) -> None:
        """Make sure a RELAY is in its data plane before we send commands.

        SerialConnection classifies the device from its HELLO banner, but with
        the robot streaming telemetry on the same channel that banner is
        routinely lost -- the connection then reports mode='direct' and never
        runs _relay_handshake(). Commands go to the relay's CONTROL plane and
        are silently discarded, while robot->host telemetry keeps flowing, so
        every Move looks accepted and never executes. Whether a run worked
        depended on whether some EARLIER !GO had left the relay latched in its
        data plane -- which is the whole "intermittent move drop" this session
        chased through bad Move ids, a dead twist path, boot races and a flat
        battery.

        Sending !GO again is harmless if it is already there.
        """
        if getattr(self._conn, "mode", "") == "relay":
            return
        try:
            for cmd in ("!ECHO OFF", "!MODE RAW250", "!GO"):
                self._conn.send_fast(cmd)
                time.sleep(0.4)
            time.sleep(0.6)
            self._rec.note("relay data-plane handshake sent", port=port)
        except Exception as exc:
            self._rec.note("relay handshake failed", port=port, error=str(exc))

    def _predict(self, kw: dict, pose: dict) -> list[tuple[float, float]]:
        """Sample where this Move will actually take the robot, from the
        CAMERA pose. Straight moves walk along the heading; arcs sweep around
        the instantaneous centre at radius v/omega. Time-bounded moves use
        v * t. Anything unbounded is treated as its timeout's worth of travel,
        because that is what it can do if nothing stops it."""
        x, y, h = pose["x"], pose["y"], pose["heading"]
        v = float(kw.get("v_x", 0.0))
        if "v_left" in kw or "v_right" in kw:
            vl = float(kw.get("v_left", 0.0)); vr = float(kw.get("v_right", 0.0))
            v = 0.5 * (vl + vr)
            omega = (vr - vl) / 115.0
        else:
            omega = float(kw.get("omega", 0.0))
        if "stop_distance" in kw:
            span = abs(float(kw["stop_distance"]))
        elif "stop_angle" in kw and abs(omega) > 1e-6:
            span = abs(float(kw["stop_angle"]) / omega) * abs(v)
        elif "stop_time" in kw:
            span = abs(v) * float(kw["stop_time"]) / 1000.0
        else:
            span = abs(v) * float(kw.get("timeout", 2000.0)) / 1000.0
        span = min(span, 3000.0)
        out, steps = [], 24
        for i in range(steps + 1):
            d = span * i / steps
            if abs(omega) > 1e-6 and abs(v) > 1e-6:
                R = v / omega
                dth = (d / abs(v)) * omega * (1 if v >= 0 else -1)
                cx, cy = x - R * math.sin(h), y + R * math.cos(h)
                th = h + dth
                out.append((cx + R * math.sin(th), cy - R * math.cos(th)))
            else:
                sgn = 1.0 if v >= 0 else -1.0
                out.append((x + sgn * d * math.cos(h), y + sgn * d * math.sin(h)))
        return out

    def move(self, **kwargs: Any) -> int:
        """Same keyword surface as SimLoop.move: ms for times, `id` for the
        Move id -- so _move_kwargs() and the spline executor are backend
        agnostic.

        REFUSES TO ISSUE a move whose predicted path leaves the table. Every
        rail contact this project has had came from a fence that reacted after
        the fact, or that read the robot's own odometry -- the same number the
        driving logic uses, so when it is wrong the fence is wrong the same
        way. Here the check runs BEFORE the command is sent and uses the
        CAMERA, which cannot be wrong in the same direction as odometry. If
        the tag is not visible, the move is refused: driving blind is how the
        robot ends up somewhere nobody predicted."""
        kw = dict(kwargs)
        move_id = int(kw.pop("id", 0))
        if self._dc is not None and not self._skip_guard:
            # Retry: the tag is missed on a frame now and then (measured ~70%
            # detection on some tags), and refusing on a single miss stops a
            # perfectly safe run for no reason.
            pose = None
            for _ in range(6):
                pose = self.quick_pose()
                if pose is not None:
                    break
                time.sleep(0.12)
            if pose is None:
                self.estop()
                raise TourFailure("refused: robot tag not visible -- will not "
                                  "command motion without an independent fix")
            path = self._predict(kw, pose)
            bad = [q for q in path
                   if abs(q[0]) > _BOX_X or abs(q[1]) > _BOX_Y]
            if bad:
                self.estop()
                q = bad[0]
                raise TourFailure(
                    f"refused: predicted path leaves the table at "
                    f"({q[0]:.0f},{q[1]:.0f}) mm from ({pose['x']:.0f},"
                    f"{pose['y']:.0f}) -- limit +-{_BOX_X:.0f}/{_BOX_Y:.0f}")
        if "v_left" in kw or "v_right" in kw:
            return self._p.move_wheels(kw.pop("v_left", 0.0),
                                       kw.pop("v_right", 0.0),
                                       move_id=move_id, **kw)
        return self._p.move_twist(kw.pop("v_x", 0.0), kw.pop("v_y", 0.0),
                                  kw.pop("omega", 0.0), move_id=move_id, **kw)

    def stop(self) -> int:
        return self._p.stop()

    def estop(self) -> int:
        # A halt is a REQUEST: the brick latches its last speed and a lost
        # zero write is permanent, so it is always sent twice.
        r = self._p.estop()
        time.sleep(0.25)
        try:
            self._p.estop()
        except Exception:
            pass
        return r

    def read_pending_frames(self) -> list:
        """Drain telemetry AND pump cleartext replies into the recorder.

        Do NOT also drain cleartext lines here. Reading lines from the same
        buffer STEALS bytes from the binary framing and the decode goes wrong:
        measured 2026-08-14, odometry pose came back as (2879, 16887) mm and
        the robot drove onto the north rail, on a path that had tracked to
        72 mm minutes earlier. Cleartext replies are simply not recoverable
        while binary telemetry streams."""
        frames = self._p.read_pending_binary_tlm_frames()
        for fr in frames:
            self._rec.rx_tlm(fr)
        return frames

    def send_line(self, line: str) -> None:
        """Fire-and-forget. A cleartext REPLY cannot be recovered while
        binary telemetry is streaming (see read_pending_frames), so tours that
        need a liveness assertion on hardware use CAMFIX instead."""
        self._p.send_fast(line)

    def true_pose(self) -> dict | None:
        if not self._dc:
            return None
        xs, ys, ws = [], [], []
        for _ in range(9):
            try:
                tf = self._dc.get_tags(self._cam)
            except Exception:
                return None
            t = next((t for t in tf.tags if t.id == self._tag_id and t.world_xy), None)
            if t:
                xs.append(t.world_xy[0]*10.0); ys.append(t.world_xy[1]*10.0)
                ws.append(t.yaw)
            time.sleep(0.05)
        if not xs:
            return None
        med = lambda v: sorted(v)[len(v)//2]
        # "h" is the key _run_camfix reads; "heading" is kept for seeding.
        return {"x": med(xs), "y": med(ys), "h": ws[0], "heading": ws[0]}

    def seed_from_camera(self) -> dict | None:
        """Put the robot's odometry into FIELD coordinates.

        Pure pursuit compares the robot's pose against path points expressed
        in the field frame, so without this the robot chases a path drawn
        around an arbitrary odometry origin. Camera-seeding once at the start
        is a surveyed start, not closed-loop camera steering -- nothing reads
        the camera again while the path is being followed."""
        pose = self.true_pose()
        if not pose:
            return None
        self._p.send_fast(f"SEED:{pose['x']:.0f},{pose['y']:.0f},"
                          f"{pose['heading']:.4f}")
        time.sleep(1.2)
        self._rec.note("seed", x=round(pose["x"], 1), y=round(pose["y"], 1),
                       heading_deg=round(math.degrees(pose["heading"]), 2))
        return pose

    def quick_pose(self) -> dict | None:
        """ONE camera sample, no averaging -- for the in-loop fence.

        true_pose() takes a median of 9 samples and blocks ~0.45 s. Calling
        that every 0.5 s inside the pure-pursuit loop halved the control rate
        and the robot never finished a lap (measured: 17 mm cross-track, 0/1
        laps). A fence only needs to know roughly where the robot is."""
        if not self._dc:
            return None
        try:
            tf = self._dc.get_tags(self._cam)
        except Exception:
            return None
        t = next((t for t in tf.tags if t.id == self._tag_id and t.world_xy), None)
        if not t:
            return None
        return {"x": t.world_xy[0]*10.0, "y": t.world_xy[1]*10.0,
                "heading": t.yaw}

    def firmware_version(self) -> str:
        try:
            return self._p.version()
        except Exception:
            return "?"

    def enable_telemetry(self) -> None:
        self._p.tlmOn()

    def close(self) -> None:
        try:
            self.estop()
        finally:
            try:
                self._conn.disconnect()
            except Exception:
                pass
            if self._dc:
                try:
                    self._dc.close()
                except Exception:
                    pass


class TourRunner:
    """Executes one Tour against a backend, recording as it goes."""

    def __init__(self, backend: Any, recorder: Recorder, *,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep):
        self._b = backend
        self._r = recorder
        self._clock = clock
        self._sleep = sleep
        self._move_seq = 0
        # All records, appended by bus subscription -- EXPECT's haystack.
        self._records: list[dict] = []
        recorder.subscribe(self._records.append)
        # seq_file of the previous directive's own tx record -- EXPECT's
        # window anchor ("between the previous command and the timeout").
        self._anchor_seq = 0

    # -- helpers --------------------------------------------------------

    def _next_move_id(self) -> int:
        self._move_seq += 1
        return _MOVE_ID_BASE + self._move_seq

    def _drain(self) -> list:
        """THE single destructive telemetry drain."""
        return self._b.read_pending_frames()

    def _wait_for_ack(self, move_id: int, timeout: float) -> bool:  # [s]
        # Backends may fold the Move id to fit the wire (see
        # HardwareBackend.move); match whichever id actually went out.
        deadline = self._clock() + timeout
        # Fallback: treat "was active, then came to rest" as completion.
        # Kept because it is independently sound (a finished move should not
        # be reported as a failure just because an ack was missed), NOT
        # because of the id theory it was first written for -- that theory
        # was WRONG. Every move_id from 0 to 65535 drives fine; the runs that
        # looked like id failures were a robot pinned against the east rail,
        # travelling less each hop until it could not move at all.
        saw_active = False
        quiet = 0
        while self._clock() < deadline:
            for frame in self._drain():
                for ack in (frame.acks or []):
                    if ack.corr_id == move_id:
                        return True
                if frame.flags is not None:
                    if frame.flags & _ACTIVE_BIT:
                        saw_active = True
                        quiet = 0
                    elif saw_active:
                        quiet += 1
                        if quiet >= _STOP_SETTLE_FRAMES:
                            return True
            self._sleep(_POLL)
        return False

    def _await_active(self, timeout: float) -> bool:  # [s]
        """True once telemetry shows the robot actually moving."""
        deadline = self._clock() + timeout
        while self._clock() < deadline:
            for frame in self._drain():
                if frame.flags is not None and (frame.flags & _ACTIVE_BIT):
                    return True
            self._sleep(_POLL)
        return False

    def _wait_at_rest(self, timeout: float = _STOP_TIMEOUT) -> bool:  # [s]
        quiet = 0
        deadline = self._clock() + timeout
        while self._clock() < deadline:
            for frame in self._drain():
                if frame.flags is not None and not (frame.flags & _ACTIVE_BIT):
                    quiet += 1
                    if quiet >= _STOP_SETTLE_FRAMES:
                        return True
                else:
                    quiet = 0
            self._sleep(_POLL)
        return False

    # -- pure pursuit ---------------------------------------------------

    def _pose(self):
        """Latest robot pose (x [mm], y [mm], heading [rad]) or None.

        Drains through the SAME single destructive drain as everything else --
        a second drain would starve the ack loop."""
        got = None
        for frame in self._drain():
            pose = getattr(frame, "pose", None)
            if not pose:
                continue
            # frame.pose is (x_mm, y_mm, heading_CENTIDEGREES) -- the same
            # decode recorder.py does ("x, y, h_cdeg = frame.pose"). Reading
            # the third field as radians yields headings of tens of thousands
            # of degrees and the follower diverges immediately.
            px, py, h_cdeg = pose
            got = (float(px), float(py), math.radians(float(h_cdeg) / 100.0))
        return got

    def _run_spline(self, step: SplineStep, tour_dir: Path) -> StepResult:
        """Follow a stored path with pure pursuit.

        Classic form: find the point on the path one lookahead ahead of the
        robot, transform it into the body frame, and command the arc that
        reaches it -- omega = 2*v*y_body / L^2. Twists are re-issued
        continuously with a short time bound so a lost command expires
        instead of latching (every Move is bounded by construction, and the
        Nezha brick latches its last speed if a zero is lost).
        """
        sp = splinefile.load(tour_dir / step.path)
        pts = list(sp.points)
        n = len(pts)
        v = step.speed
        L = step.lookahead
        target_laps = max(1, step.laps)

        self._r.note("spline", path=step.path, points=n,
                     length_mm=round(sp.length(), 1),
                     min_radius_mm=round(sp.min_radius(), 1),
                     speed=v, lookahead=L, laps=target_laps,
                     interval=step.interval)

        pose = None
        deadline = self._clock() + (sp.length()*target_laps/max(v, 1.0))*3.0 + 30.0
        # start at the path point nearest the robot
        for _ in range(200):
            pose = self._pose()
            if pose:
                break
            self._sleep(_POLL)
        if not pose:
            return StepResult(step.line_no, "spline", False, "no pose telemetry")
        idx = min(range(n), key=lambda i: math.dist(pts[i], pose[:2]))
        # Lap accounting by TOTAL index advance, not by landing exactly on the
        # start index -- the lookahead advance skips indices, so an equality
        # test can miss the wrap entirely and report 0 laps on a run that
        # tracked the path to 2 mm.
        advanced, worst = 0, 0.0
        laps = 0
        cam_checked = 0.0
        # Closure is measured against where THIS RUN started, not against a
        # fixed coordinate: the robot may join a closed path anywhere along
        # it, and pure pursuit then correctly returns to that join point. A
        # CAMFIX on the path's first point fails by 593 mm for a lap that
        # tracked to 43 mm -- it asserts staging, not driving.
        start_fix = (self._b.quick_pose() if hasattr(self._b, "quick_pose")
                     else None)
        start_xy = ((start_fix["x"], start_fix["y"]) if start_fix
                    else (pose[0], pose[1]))
        while self._clock() < deadline:
            pose = self._pose() or pose
            px, py, ph = pose
            # advance the target index while it is closer than the lookahead
            j = idx
            for _ in range(n):
                if math.dist(pts[j], (px, py)) >= L:
                    break
                j = (j + 1) % n
                advanced += 1
                if not sp.closed and j >= n - 1:
                    break
            laps = advanced // n
            if laps >= target_laps:
                break
            if not sp.closed and advanced >= n - 1:
                break
            idx = j
            tx, ty = pts[idx]
            dx, dy = tx - px, ty - py
            # GEOFENCE. The firmware stall detector cannot save us here: it
            # needs ~500 ms of sustained stall, and pure pursuit re-issues a
            # replace=True Move every ~120 ms, which restarts that window
            # every time. Measured 2026-08-14 -- the robot drove into the rail
            # and ground there with stall detection enabled and no flag set.
            # So the follower fences itself.
            if abs(px) > _BOX_X or abs(py) > _BOX_Y:
                self._b.estop()
                return StepResult(step.line_no, "spline", False,
                                  f"geofence: pose ({px:.0f},{py:.0f}) mm "
                                  f"outside +-{_BOX_X:.0f}/{_BOX_Y:.0f}")
            # INDEPENDENT camera fence, ~2 Hz. The odometry check above shares
            # its input with the follower, so a corrupted pose blinds both --
            # which is exactly what put the robot on the north rail. The
            # camera is the only source that cannot be wrong in the same way.
            now = self._clock()
            if now - cam_checked > 0.5 and hasattr(self._b, "quick_pose"):
                cam_checked = now
                tp = self._b.quick_pose()
                if tp:
                    if abs(tp["x"]) > _BOX_X or abs(tp["y"]) > _BOX_Y:
                        self._b.estop()
                        return StepResult(
                            step.line_no, "spline", False,
                            f"camera geofence: ({tp['x']:.0f},{tp['y']:.0f}) mm")
                    # odometry vs camera divergence = the pose is not trustworthy
                    drift = math.hypot(tp["x"]-px, tp["y"]-py)
                    if drift > 400.0:
                        self._b.estop()
                        return StepResult(
                            step.line_no, "spline", False,
                            f"odometry/camera disagree by {drift:.0f} mm -- "
                            f"pose untrustworthy, refusing to steer on it")
            # cross-track error against the NEAREST path point, for the gate
            near = min(range(n), key=lambda i: math.dist(pts[i], (px, py)))
            worst = max(worst, math.dist(pts[near], (px, py)))
            if worst > step.tol:
                self._b.estop()
                return StepResult(step.line_no, "spline", False,
                                  f"cross-track {worst:.0f} mm exceeded "
                                  f"tol {step.tol:.0f} mm")
            # body frame
            y_b = -math.sin(ph)*dx + math.cos(ph)*dy
            x_b = math.cos(ph)*dx + math.sin(ph)*dy
            dist = math.hypot(dx, dy) or 1.0
            omega = 2.0 * v * y_b / (dist*dist)
            if x_b < 0:                      # target behind: turn hard, don't reverse
                omega = math.copysign(max(abs(omega), 1.0), y_b or 1.0)
            # Same units/keys as _move_kwargs: ms, and `id` (not move_id).
            # A FRESH id per twist: reusing one id had the planner treat
            # every re-issue as the same Move, and the robot travelled 3 mm in
            # an entire run while 875 commands went out.
            # stop_time must comfortably outlive the re-issue period,
            # otherwise the Move expires between commands and the robot
            # stutters. timeout is the safety backstop if the host goes quiet.
            hold = max(0.35, step.interval * 2.5)
            kw = dict(v_x=v, v_y=0.0, omega=omega, stop_time=hold*1000.0,
                      timeout=max(2.0, hold*3.0)*1000.0,
                      replace=True, id=self._next_move_id())
            self._b.move(**kw)
            self._r.tx_cmd("SPLINE_TWIST", {"v_x": v, "omega": round(omega, 4),
                                            "target": [round(tx,1), round(ty,1)],
                                            "pose": [round(px,1), round(py,1),
                                                     round(math.degrees(ph),1)],
                                            "line_no": step.line_no})
            self._sleep(step.interval)
        self._b.estop()
        self._sleep(1.0)
        end_fix = (self._b.quick_pose() if hasattr(self._b, "quick_pose")
                   else None)
        end_xy = ((end_fix["x"], end_fix["y"]) if end_fix
                  else ((self._pose() or pose)[0], (self._pose() or pose)[1]))
        closure = math.dist(start_xy, end_xy) if sp.closed else float("nan")
        ok = laps >= target_laps or (not sp.closed)
        detail = (f"laps {laps}/{target_laps}, worst cross-track {worst:.0f} mm")
        if sp.closed:
            detail += f", closure {closure:.0f} mm"
        self._r.note("spline_result", laps=laps, worst_cross_track_mm=round(worst, 1),
                     closure_mm=None if math.isnan(closure) else round(closure, 1))
        return StepResult(step.line_no, "spline", ok, detail)

    # -- step executors -------------------------------------------------

    def _run_segment(self, moves: list[MoveStep]) -> list[StepResult]:
        """Run consecutive MoveSteps with one-leg lookahead."""
        results: list[StepResult] = []
        ids: list[tuple[MoveStep, int]] = []
        for i, step in enumerate(moves):
            move_id = self._next_move_id()
            kw = _move_kwargs(step, move_id, replace=(i == 0))
            corr_id = self._b.move(**kw)
            self._r.tx_cmd("MOVE", {**kw, "corr_id": corr_id,
                                    "line_no": step.line_no})
            # A Move is occasionally accepted (acked) and then never becomes
            # ACTIVE -- the robot simply does not execute it. Measured
            # 2026-08-14 on hardware: the SAME command drives one run and does
            # nothing the next, with no fault flag and no pattern in the Move
            # id (both "id X is bad" theories were noise). Re-issue once if
            # nothing goes active; a Move that IS running is unaffected because
            # the re-issue carries the same id and replace semantics.
            if not self._await_active(1.5):
                self._r.note("move did not go active; re-issuing",
                             line_no=step.line_no, id=move_id)
                self._b.move(**kw)
            self._set_anchor()
            ids.append((step, move_id))
            # Lookahead depth 1: wait for the PREVIOUS move before sending
            # the one after next.
            if i >= 1:
                prev_step, prev_id = ids[i - 1]
                ok = self._wait_for_ack(prev_id, prev_step.timeout + 5.0)
                results.append(StepResult(prev_step.line_no, "move", ok,
                                          "" if ok else "no completion ack"))
                if not ok:
                    raise TourFailure(f"line {prev_step.line_no}: move never "
                                      f"completed (id {prev_id})")
        last_step, last_id = ids[-1]
        ok = self._wait_for_ack(last_id, last_step.timeout + 5.0)
        results.append(StepResult(last_step.line_no, "move", ok,
                                  "" if ok else "no completion ack"))
        if not ok:
            raise TourFailure(
                f"line {last_step.line_no}: move never completed (id {last_id})")
        return results

    def _run_stop(self, step: StopStep) -> StepResult:
        corr_id = self._b.stop()
        self._r.tx_cmd("STOP", {"corr_id": corr_id, "line_no": step.line_no})
        self._set_anchor()
        ok = self._wait_at_rest()
        if ok and step.dwell > 0.0:
            self._sleep(step.dwell)
        return StepResult(step.line_no, "stop", ok,
                          "" if ok else "never came to rest")

    def _run_dbg(self, step: DbgStep) -> StepResult:
        line = "DBG:" + step.text
        self._b.send_line(line)
        self._r.tx_cmd("DBG", {"text": step.text, "line_no": step.line_no},
                       plane="cleartext")
        self._set_anchor()
        return StepResult(step.line_no, "dbg", True, step.text)

    def _run_send(self, step: SendStep) -> StepResult:
        line = step.verb + (":" + step.data if step.data else "")
        self._b.send_line(line)
        self._r.tx_cmd(step.verb, {"data": step.data,
                                   "line_no": step.line_no},
                       plane="cleartext")
        self._set_anchor()
        return StepResult(step.line_no, "send", True, step.verb)

    def _run_expect(self, step: ExpectStep) -> StepResult:
        try:
            program = jq.compile(step.query)
        except Exception as exc:
            return StepResult(step.line_no, "expect", False,
                              f"bad query: {exc}")

        def matches(rec: dict) -> bool:
            try:
                return bool(program.input_value(rec).first())
            except Exception:
                return False

        anchor = self._anchor_seq
        start = self._clock()
        deadline = start + step.timeout
        scanned = 0
        matched: int | None = None
        while self._clock() < deadline and matched is None:
            snapshot = self._records  # append-only; len() is safe
            while scanned < len(snapshot):
                rec = snapshot[scanned]
                scanned += 1
                if rec.get("seq_file", 0) >= anchor and matches(rec):
                    matched = rec["seq_file"]
                    break
            if matched is None:
                self._sleep(_POLL)
        waited = self._clock() - start
        ok = matched is not None
        self._r.expect_result(step.query, ok, matched, waited, step.line_no)
        return StepResult(step.line_no, "expect", ok,
                          step.query if ok else f"no match in {step.timeout}s: "
                          + step.query)

    def _run_camfix(self, step: CamfixStep) -> StepResult:
        pose = self._b.true_pose()
        if pose is None:
            self._r.camera_fix({"ok": False, "error": "no truth source",
                                "line_no": step.line_no})
            return StepResult(step.line_no, "camfix", False, "no truth source")
        dx = pose["x"] - step.x
        dy = pose["y"] - step.y
        err = math.hypot(dx, dy)
        ok = err <= step.radius
        detail = f"pos err {err:.1f} mm (radius {step.radius:.0f})"
        heading_err = None
        if step.heading is not None:
            heading_err = _norm_angle(pose["h"] - step.heading)
            h_ok = abs(heading_err) <= step.tol
            ok = ok and h_ok
            detail += f", yaw err {math.degrees(heading_err):.1f} deg"
        self._r.camera_fix({
            "source": self._b.name, "x": pose["x"], "y": pose["y"],
            "heading": pose["h"],
            "target": {"x": step.x, "y": step.y, "radius": step.radius,
                       "heading": step.heading, "tol": step.tol},
            "error": err, "heading_error": heading_err, "ok": ok,
            "line_no": step.line_no})
        self._set_anchor()
        return StepResult(step.line_no, "camfix", ok, detail)

    def _set_anchor(self) -> None:
        # Next EXPECT's window opens at the record just written (the
        # previous directive's own tx record).
        if self._records:
            self._anchor_seq = self._records[-1]["seq_file"]

    # -- the run --------------------------------------------------------

    def run(self, tour: Tour) -> TourRunResult:
        self._tour_dir = (Path(tour.source).resolve().parent
                          if tour.source not in ("<text>", "") else Path("."))
        result = TourRunResult(tour=tour.name, ok=True)
        steps = list(tour.steps)
        i = 0
        try:
            while i < len(steps):
                step = steps[i]
                if isinstance(step, MoveStep):
                    segment = [step]
                    while i + 1 < len(steps) and isinstance(steps[i + 1],
                                                            MoveStep):
                        i += 1
                        segment.append(steps[i])
                    self._r.tx_step("segment", segment[0].line_no,
                                    {"moves": len(segment)})
                    result.steps.extend(self._run_segment(segment))
                elif isinstance(step, StopStep):
                    r = self._run_stop(step)
                    result.steps.append(r)
                    if not r.ok:
                        raise TourFailure(f"line {step.line_no}: stop failed")
                elif isinstance(step, DwellStep):
                    self._r.tx_step("dwell", step.line_no,
                                    {"seconds": step.seconds})
                    self._sleep(step.seconds)
                    result.steps.append(StepResult(step.line_no, "dwell", True))
                elif isinstance(step, DbgStep):
                    result.steps.append(self._run_dbg(step))
                elif isinstance(step, SendStep):
                    result.steps.append(self._run_send(step))
                elif isinstance(step, ExpectStep):
                    r = self._run_expect(step)
                    result.steps.append(r)
                    if not r.ok:
                        raise TourFailure(
                            f"line {step.line_no}: EXPECT failed: {r.detail}")
                elif isinstance(step, SplineStep):
                    res = self._run_spline(step, self._tour_dir)
                    result.steps.append(res)
                    if not res.ok:
                        raise TourFailure(f"line {step.line_no}: spline "
                                          f"failed -- {res.detail}")
                elif isinstance(step, CamfixStep):
                    r = self._run_camfix(step)
                    result.steps.append(r)
                    if not r.ok:
                        raise TourFailure(
                            f"line {step.line_no}: CAMFIX failed: {r.detail}")
                i += 1
        except TourFailure as exc:
            self._b.estop()
            self._r.note("tour failed; estop sent", reason=str(exc))
            result.ok = False
        except BaseException:
            self._b.estop()
            raise
        return result


def _norm_angle(a: float) -> float:  # [rad] -> (-pi, pi]
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:16]


def _git_sha(repo_root: Path) -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                              cwd=repo_root, capture_output=True, text=True,
                              timeout=5).stdout.strip()
    except Exception:
        return "?"


def build_run_meta(tour: Tour, *, tier: str, robot_config: str,
                   argv: list[str]) -> dict:
    root = Path(__file__).resolve().parents[3]
    cfg = Path(robot_config)
    return {
        "schema_version": 1,
        "tier": tier,
        "tour_file": tour.source,
        "tour_sha256": hashlib.sha256(tour.text.encode()).hexdigest()[:16],
        "tour_text": tour.text,
        "robot_config": str(cfg),
        "robot_config_sha256": _sha256(cfg) if cfg.exists() else "?",
        "host_git_sha": _git_sha(root),
        "started_wall": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "argv": argv,
    }
