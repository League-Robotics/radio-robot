"""NezhaState — hardware state manager for the Nezha robot (protocol v2).

Owns the update cycle: sends S keepalive commands via NezhaProtocol, reads
TLM frames, maintains timestamped sensor state, and supports both synchronous
and async (daemon thread) operation.

Usage (manual):
    proto = NezhaProtocol(conn)
    state = NezhaState(proto)
    state.update(left=100, right=100)
    print(state.encoders, state.otos_pose)

Usage (async):
    state.start_async()
    state.wheel_speeds = [100, 100]
    time.sleep(1.0)
    state.stop_async()
"""

from __future__ import annotations

import math
import threading
import time

from robot_radio.nav.pose import Pose
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame
from robot_radio.robot.robot_state import RobotState


class NezhaState:
    """Hardware state manager for the Nezha robot (protocol v2).

    Maintains the most recent TLM sensor values.  All public state attributes
    are protected by ``_lock``.  Callers needing a consistent snapshot should
    copy under the lock:

        with state._lock:
            enc = state.encoders
            pose = state.otos_pose

    ``update()`` is the unit of work: sends current wheel speeds and reads
    available TLM lines for ~40 ms.  Call it manually in a tight control loop,
    or use ``start_async()`` / ``stop_async()`` to run it on a daemon thread at
    ~25 Hz.
    """

    def __init__(self, proto: NezhaProtocol) -> None:
        self._proto = proto
        self._lock = threading.Lock()
        self._wheel_speeds: list[int] = [0, 0]
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        # Sensor state — always access under _lock
        self.encoders: tuple[int, int] = (0, 0)
        # otos_pose stores (x, y, heading) — centi-degrees for heading
        self.otos_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
        # heading_rad: CCW-positive heading in radians, derived from TLM pose cdeg
        self.heading_rad: float = 0.0
        self.line_sensor: tuple[int, int, int, int] = (255, 255, 255, 255)
        self.color: tuple[int, int, int, int] = (0, 0, 0, 0)
        self.last_tlm_t: int | None = None   # robot clock of last TLM frame (ms)
        self.last_update_s: float = 0.0
        self.dt_s: float = 0.0
        # Composite motion state — built from TLM frames that carry pose=.
        # twist= is optional: when absent, v=0 and omega=0 are used.
        self.robot_state: RobotState | None = None
        # EKF gate rejection count — cumulative across all channels.
        # Populated from TLM ekf_rej= field (sprint 024-005). None if not yet received.
        self.ekf_rej: int | None = None

    # ------------------------------------------------------------------
    # Wheel speed property
    # ------------------------------------------------------------------

    @property
    def wheel_speeds(self) -> list[int]:
        """Current commanded wheel speeds [left, right]."""
        with self._lock:
            return list(self._wheel_speeds)

    @wheel_speeds.setter
    def wheel_speeds(self, speeds: list[int]) -> None:
        with self._lock:
            self._wheel_speeds[0] = speeds[0]
            self._wheel_speeds[1] = speeds[1]

    # ------------------------------------------------------------------
    # Update cycle
    # ------------------------------------------------------------------

    def update(self, left: int | None = None, right: int | None = None) -> None:  # [mm/s]
        """Send current wheel speeds and read available TLM frames for 40 ms.

        If ``left`` or ``right`` are provided they are stored as the
        new commanded wheel speeds before sending.

        097-003: telemetry arrives over the binary plane now (``stream()``
        is binary-only -- see its own docstring), so this reads
        ``NezhaProtocol.read_binary_tlm_frames()`` (already-parsed
        ``TLMFrame`` objects) rather than raw text lines.
        """
        if left is not None or right is not None:
            with self._lock:
                if left is not None:
                    self._wheel_speeds[0] = left
                if right is not None:
                    self._wheel_speeds[1] = right

        with self._lock:
            l, r = self._wheel_speeds[0], self._wheel_speeds[1]

        self._proto.drive(l, r)

        now = time.monotonic()
        for tlm in self._proto.read_binary_tlm_frames(duration=40):
            self._apply_tlm(tlm)

        with self._lock:
            self.dt_s = now - self.last_update_s if self.last_update_s > 0 else 0.0
            self.last_update_s = now

    def _apply_tlm(self, tlm: TLMFrame) -> None:
        """Update matching state attributes from one already-parsed TLMFrame."""
        with self._lock:
            if tlm.enc is not None:
                self.encoders = tlm.enc
            if tlm.pose is not None:
                x, y, heading = tlm.pose  # [mm], [mm], [cdeg]
                self.otos_pose = (float(x), float(y), float(heading))
                # Convert centidegrees to radians: cdeg / 18000.0 * math.pi
                self.heading_rad = heading / 18000.0 * math.pi

                # Build RobotState from pose (mandatory) + twist (optional).
                # Pose.x/y are in centimetres (nav convention); firmware x in mm.
                pose_obj = Pose(
                    x=float(x) / 10.0,
                    y=float(y) / 10.0,
                    heading=heading / 18000.0 * math.pi,
                )
                if tlm.twist is not None:
                    v_mmps, omega_mradps = tlm.twist
                    v_f     = float(v_mmps)
                    omega_f = float(omega_mradps) / 1000.0  # mrad/s → rad/s
                else:
                    v_f = 0.0
                    omega_f = 0.0
                self.robot_state = RobotState(
                    pose=pose_obj,
                    v=v_f,
                    omega=omega_f,
                    accel=None,
                    stamp=time.monotonic(),
                )

            if tlm.line is not None:
                self.line_sensor = tlm.line
            if tlm.color is not None:
                self.color = tlm.color
            if tlm.t is not None:
                self.last_tlm_t = tlm.t
            if tlm.ekf_rej is not None:
                self.ekf_rej = tlm.ekf_rej

    # ------------------------------------------------------------------
    # One-shot commands
    # ------------------------------------------------------------------

    def zero_otos(self) -> None:
        """Zero the OTOS position at the current location (ZERO pose command)."""
        self._proto.zero_otos()
        with self._lock:
            x, y, h = self.otos_pose
            self.otos_pose = (0.0, 0.0, h)

    def zero_encoders(self) -> None:
        """Zero all encoder counters (ZERO enc command)."""
        self._proto.zero_encoders()
        with self._lock:
            self.encoders = (0, 0)

    def stop(self) -> None:
        """Stop motors immediately (STOP command)."""
        with self._lock:
            self._wheel_speeds = [0, 0]
        self._proto.stop()

    def set_world_pose(self, x: float, y: float, heading: float) -> None:  # [mm], [mm], [deg]
        """Set OTOS world-frame pose (OV command). Heading in degrees."""
        xi, yi = int(round(x)), int(round(y))
        # Firmware OV expects centi-degrees for heading
        wire_heading = int(round(heading * 100))  # [cdeg]
        self._proto.otos_set_position(xi, yi, wire_heading)
        with self._lock:
            self.otos_pose = (float(xi), float(yi), float(wire_heading))
            self.heading_rad = wire_heading / 18000.0 * math.pi

    def enable_stream(self, period: int = 40) -> None:  # [ms]
        """Enable TLM streaming at the given period (STREAM <ms>)."""
        self._proto.stream(period)

    def disable_stream(self) -> None:
        """Disable TLM streaming (STREAM 0)."""
        self._proto.stream(0)

    # ------------------------------------------------------------------
    # Async operation
    # ------------------------------------------------------------------

    def start_async(self, interval_s: float = 0.040) -> None:
        """Start a daemon thread calling ``update()`` in a loop at ~25 Hz."""
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(interval_s,),
            daemon=True,
            name="NezhaState-async",
        )
        self._thread.start()

    def stop_async(self) -> None:
        """Signal the async thread to exit and wait for it to join (up to 2 s)."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self, interval_s: float) -> None:
        sleep_s = max(0.0, interval_s - 0.040)
        while not self._stop_event.is_set():
            try:
                self.update()
            except Exception:
                pass
            if sleep_s > 0:
                time.sleep(sleep_s)
