"""Nezha — robot driver for the Nezha2 firmware, protocol v2.

Commands (full v2 reference in source/app/CommandProcessor.cpp):

Motion
------
  S <l> <r>         streaming speed burst (watchdog-based)
  T <l> <r> <ms>    timed speed burst; robot sends EVT done T when done
  D <l> <r> <mm>    drive by distance;  robot sends EVT done D when done
  G <x> <y> <speed> go-to XY arc;       robot sends EVT done G when done
  STOP              stop immediately

Gripper
-------
  GRIP <deg>        set gripper servo angle
  GRIP              query current angle (returns OK grip deg=<val>)

Encoders / pose
---------------
  ZERO enc          zero encoder counters
  ZERO pose         zero OTOS pose tracking
  ZERO enc pose     zero both

Telemetry
---------
  STREAM <ms>       set TLM period (0=off)
  STREAM fields=…   subscribe to a subset of TLM fields
  SNAP              request one immediate TLM frame

OTOS sensor
-----------
  OI    init signal processing
  OZ    zero position at current location
  OR    reset Kalman filters
  OP    query position -> OK pos x= y= h=
  OV    set world-frame position

Config
------
  GET [keys…]       read config (all or named subset) -> CFG key=val ...
  SET key=val …     write config keys               -> OK set key=val ...
"""

from __future__ import annotations

import math
import time
from typing import Any, Generator

from robot_radio.nav.pose import Pose
from robot_radio.robot.robot import Robot
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame, ParsedResponse, parse_response
from robot_radio.robot.robot_state import RobotState


class RobotNotFoundError(ConnectionError):
    """Raised when the robot does not respond to liveness preflight (PING/ID)."""


class Nezha(Robot):
    """Driver for the DFRobot Nezha2 via serial running protocol v2 firmware.

    The canonical robot state snapshot is available as ``robot.state``
    (a frozen :class:`RobotState`), updated by ``_apply_tlm`` on every
    incoming TLM frame.  The legacy per-attribute names
    (``encoders``, ``otos_pose``, ``line_sensor``, ``color``) remain
    available as thin properties over ``state`` so all existing callers
    are unaffected.

    Streaming drive interface:
      - ``stream_drive(speeds, ...)`` — generator; yields ParsedResponse objects
        and updates robot state (encoders, otos_pose, line_sensor, color)
      - ``vw(v, omega, ...)`` — body-velocity generator; yields None
        each TLM tick; state updated before yield; break sends STOP + STREAM 0
      - ``send_drive(left, right)``   — fire-and-forget S keepalive for manual loops

    Typical streaming loop::

        speeds = [200, 200]
        for resp in robot.stream_drive(speeds, period=40):
            # 097-003: binary telemetry frames arrive already parsed, on
            # resp.tlm -- see ParsedResponse.tlm's own docstring.
            tlm = resp.tlm if resp.tag == "TLM" else None
            if tlm and tlm.enc:
                print(tlm.enc)
            # mutate speeds to steer
            speeds[0] = new_left
            speeds[1] = new_right

    ``otos_pose`` stores ``(x, y, yaw_rad)`` with CCW-positive yaw in radians.
    ``encoders`` stores ``(left, right)`` cumulative totals.
    """

    # Below this, neither wheel reliably rotates (deadband).
    MIN_SPEED = 12  # [mm/s]

    def __init__(self, proto: NezhaProtocol) -> None:
        self._proto = proto

        # Unified live sensor state — updated by _apply_tlm on every TLM frame.
        # Legacy per-attribute names (encoders, otos_pose, line_sensor, color)
        # are thin properties over this object so existing callers are unaffected.
        self.state: RobotState = RobotState(
            pose=Pose(x=0.0, y=0.0, heading=0.0),
            v=0.0,
            omega=0.0,
            accel=None,
            stamp=time.monotonic(),
            encoders=None,
            twist=None,
            line=None,
            color=None,
            world_pose=None,
        )

    # ------------------------------------------------------------------
    # Back-compat sensor properties (thin wrappers over self.state)
    # ------------------------------------------------------------------

    @property
    def encoders(self) -> tuple[int, int]:
        """Cached encoder totals (left, right). Returns (0, 0) until first TLM frame."""
        return self.state.encoders or (0, 0)

    @property
    def otos_pose(self) -> tuple[float, float, float]:
        """Cached OTOS pose (x, y, yaw_rad). Returns (0.0, 0.0, 0.0) until first TLM."""
        p = self.state.pose
        return (p.x, p.y, p.heading)

    @property
    def line_sensor(self) -> tuple[int, int, int, int]:
        """Cached line sensor values (g1, g2, g3, g4). Returns (255,255,255,255) until first TLM."""
        return self.state.line or (255, 255, 255, 255)

    @property
    def color(self) -> tuple[int, int, int, int]:
        """Cached colour sensor values (r, g, b, c). Returns (0, 0, 0, 0) until first TLM."""
        return self.state.color or (0, 0, 0, 0)

    # ------------------------------------------------------------------
    # Robot interface — connection
    # ------------------------------------------------------------------

    def is_connected(self) -> bool:
        return self._proto.is_open

    def connect(self, attempts: int = 5) -> dict[str, str]:
        """Run liveness preflight: PING then ID.

        Returns the robot identity kv dict on success.
        Raises RobotNotFoundError if either step times out or returns no response.

        Each step is retried up to ``attempts`` times.  PING and ID are
        idempotent reads, and an occasional reply is lost when a "+" keepalive
        gets merged with the command by the relay's RAW250 framing (the
        keepalive idle-gate in SerialConnection minimises this, but a lone
        command issued right after a "+" can still collide).  The rapid retries
        form a command burst, during which the idle-gate suppresses "+" entirely,
        so a corrupted preflight reliably recovers on the next attempt rather
        than aborting the whole session.
        """
        if not any(self._proto.ping() is not None for _ in range(attempts)):
            raise RobotNotFoundError(
                "Robot did not respond to PING — check cable, relay, and power."
            )
        for _ in range(attempts):
            id_result = self._proto.get_id()
            if id_result is not None:
                return id_result
        raise RobotNotFoundError(
            "Robot did not respond to ID — relay may be present but robot is silent."
        )

    # ------------------------------------------------------------------
    # Robot interface — motion
    # ------------------------------------------------------------------

    def stop(self) -> None:
        """Stop motors immediately (STOP command)."""
        self._proto.stop()

    def grip(self, angle: int) -> None:
        """Set gripper servo angle (GRIP <deg> command)."""
        self._proto.grip(angle)

    def speed(self, left: int, right: int) -> Generator[tuple[int, int], None, None]:  # [mm/s]
        """Non-blocking encoder streaming. Yields (left, right) encoder totals.

        Sends S keepalives to maintain streaming. Close the generator to stop.
        """
        def _clamp(v: int) -> int:
            return 0 if v == 0 else max(self.MIN_SPEED, abs(v)) * (1 if v > 0 else -1)

        l, r = _clamp(left), _clamp(right)
        speeds = [l, r]
        try:
            for resp in self._proto.stream_drive(speeds, period=40, watchdog=200):
                # 097-003: binary telemetry frames arrive already parsed, on
                # resp.tlm -- see ParsedResponse.tlm's own docstring.
                tlm = resp.tlm if resp.tag == "TLM" else None
                if tlm and tlm.enc:
                    self._apply_tlm(tlm)
                    yield tlm.enc
        except GeneratorExit:
            pass

    def speed_for_time(self, left: int, right: int, ms: int) -> tuple[int, int]:  # [mm/s]
        """Blocking timed drive (T command). Returns final encoder totals (mm).

        Waits for EVT done T or a conservative host-side timeout.
        """
        def _clamp(v: int) -> int:
            return 0 if v == 0 else max(self.MIN_SPEED, abs(v)) * (1 if v > 0 else -1)

        l, r = _clamp(left), _clamp(right)
        self._proto.timed(l, r, ms)
        self._proto.wait_for_evt_done("T", ms + 2000)  # outcome unused; discard reason
        return self.encoders

    def speed_for_distance(self, left: int, right: int, mm: int) -> tuple[int, int]:  # [mm/s]
        """Blocking distance drive (D command). Returns final encoder totals (mm)."""
        def _clamp(v: int) -> int:
            return 0 if v == 0 else max(self.MIN_SPEED, abs(v)) * (1 if v > 0 else -1)

        l, r = _clamp(left), _clamp(right)
        remaining = abs(int(mm))  # [mm]
        if remaining == 0:
            return self.encoders

        cruise = max(abs(l), abs(r), 1)  # [mm/s]
        hop_max = max(40, int(cruise * 1.5))  # [mm]

        while remaining > 0:
            hop = min(remaining, hop_max)
            self._proto.read_pending_lines()
            self._proto.distance(l, r, hop)

            outcome, _ = self._proto.wait_for_evt_done("D", timeout=6000)
            if outcome == "timeout":
                self._proto.stop()
                raise TimeoutError(
                    f"Distance hop timed out: target={hop}mm at speeds {l},{r}"
                )
            remaining -= hop

        return self.encoders

    def _run_until_done(
        self, verb: str, on_tick: Any, timeout_s: float
    ) -> str:
        """Private tick loop for callback-driven go_to / turn.

        Drains binary telemetry frames (``self._proto.
        read_pending_binary_tlm_frames()``, 097-003 -- ``stream()`` is
        binary-only now, see that method's own docstring), updates robot
        state from each, and calls ``on_tick(self)`` after each update. EVT
        lines (``done <verb>``/``safety_stop``) still arrive as text, read
        via ``self._proto._conn.read_lines(duration=50)`` as before -- EVT
        emission is unaffected by ``stream()``'s binary conversion.

        Returns one of: ``"done"``, ``"safety_stop"``, ``"aborted"``,
        ``"timeout"``.

        Termination rules (in evaluation order per iteration):
        1. ``on_tick`` returns ``False`` → send ``X``, disable stream,
           return ``"aborted"``.
        2. ``EVT done <verb>`` arrives → disable stream, return ``"done"``.
        3. ``EVT safety_stop`` arrives → disable stream, return
           ``"safety_stop"``.
        4. Wall-clock ``timeout_s`` exceeded → send ``X``, disable stream,
           return ``"timeout"``.
        5. No TLM arrived and keepalive interval elapsed → send ``+``
           keepalive (safety belt; the SerialConnection daemon also does
           this).
        """
        _KEEPALIVE_INTERVAL = 0.200  # seconds between keepalives
        deadline = time.monotonic() + timeout_s
        last_keepalive = time.monotonic()
        had_tlm = False

        # Settle-based completion (radio robustness): the firmware's
        # ``EVT done <verb>`` rides the same lossy radio link as TLM and is
        # easily dropped during/after motion, which would otherwise hang this
        # loop until the full timeout.  So we also detect arrival independently:
        # once the robot has actually moved and then holds ~zero velocity for
        # _SETTLE_S, the motion is over → return "settled".  The caller reports
        # the final pose so the operator sees where it actually stopped.
        _SETTLE_V = 20.0      # mm/s linear "stopped" threshold
        _SETTLE_W = 0.15      # rad/s yaw-rate "stopped" threshold
        _SETTLE_S = 1.5       # s of continuous stop (after moving) → settled
        moved = False
        stopped_since: float | None = None

        while time.monotonic() < deadline:
            had_tlm = False

            for tlm in self._proto.read_pending_binary_tlm_frames():
                self._apply_tlm(tlm)
                had_tlm = True
                result = on_tick(self)
                if result is False:
                    self._proto._conn.send_fast("X")
                    self._proto.stream(0)
                    return "aborted"

            lines = self._proto._conn.read_lines(duration=50)

            for raw_line in lines:
                r = parse_response(raw_line)
                if r is None:
                    continue

                if r.tag == "EVT":
                    tokens = r.tokens
                    if tokens and tokens[0] == "done":
                        if len(tokens) < 2 or tokens[1] == verb:
                            self._proto.stream(0)
                            return "done"
                    elif tokens and tokens[0] == "safety_stop":
                        self._proto.stream(0)
                        return "safety_stop"

            # Settle detection: once the robot has moved and then holds ~zero
            # velocity for _SETTLE_S, treat the move as complete even if the
            # EVT done was never received (dropped over radio).
            moving = (abs(self.state.v) > _SETTLE_V or
                      abs(self.state.omega) > _SETTLE_W)
            if moving:
                moved = True
                stopped_since = None
            elif moved:
                if stopped_since is None:
                    stopped_since = time.monotonic()
                elif time.monotonic() - stopped_since >= _SETTLE_S:
                    self._proto.stream(0)
                    return "settled"

            # Keepalive safety belt (daemon normally covers this).
            now = time.monotonic()
            if not had_tlm and (now - last_keepalive) >= _KEEPALIVE_INTERVAL:
                self._proto._conn.send_fast("+")
                last_keepalive = now

        # Deadline exceeded.
        self._proto._conn.send_fast("X")
        self._proto.stream(0)
        return "timeout"

    def go_to(self, x: int, y: int,  # [mm]
              speed: int,  # [mm/s]
              on_tick: Any = None,
              timeout_s: float = 15.0) -> tuple[int, int, str]:
        """Blocking or callback-driven go-to (G command).

        Returns ``(left_enc, right_enc, outcome)`` where ``outcome``
        is one of ``"done"``, ``"settled"``, ``"safety_stop"``, ``"aborted"``,
        or ``"timeout"``.  ``"settled"`` (callback path only) means the robot
        moved then held ~zero velocity for ~1.5 s without an ``EVT done`` — used
        when that event is dropped over the radio link.

        Parameters
        ----------
        x, y:
            Target position in robot-relative mm (forward, left).
        speed:
            Cruise speed in mm/s (clamped to ≥ 1).
        on_tick:
            When ``None`` (default), the method blocks using
            ``wait_for_evt_done`` — identical to the pre-sprint behaviour.
            No STREAM is enabled.  This is the path used by Navigator.

            When a callable is provided, ``STREAM 80`` is enabled before
            issuing ``G``, and ``on_tick(robot)`` is called after each TLM
            tick.  Returning ``False`` from ``on_tick`` aborts the move
            (sends ``X``, outcome ``"aborted"``).
        timeout_s:
            Maximum wall-clock seconds to wait.
        """
        speed = max(abs(speed), 1)
        if on_tick is None:
            # Back-compat blocking path — behaviour unchanged from pre-sprint.
            self._proto.go_to(x, y, speed)
            timeout = int(timeout_s * 1000)
            outcome, _ = self._proto.wait_for_evt_done("G", timeout)
            time.sleep(0.2)
            return self.encoders[0], self.encoders[1], outcome
        else:
            self._proto.stream(80)
            self._proto.go_to(x, y, speed)
            outcome = self._run_until_done("G", on_tick, timeout_s)
            return self.encoders[0], self.encoders[1], outcome

    def turn(self, heading: int, on_tick: Any = None,  # [cdeg]
             eps: int | None = None,  # [cdeg]
             timeout_s: float = 10.0) -> str:
        """Rotate to an absolute heading (TURN command).

        Returns outcome string: ``"done"``, ``"safety_stop"``,
        ``"aborted"``, or ``"timeout"``.

        Parameters
        ----------
        heading:
            Target heading in centi-degrees.  Positive = CCW (matches OTOS
            CCW convention).  Range −18000 … +18000.
        on_tick:
            When ``None`` (default), blocks using ``wait_for_evt_done``.
            When a callable, enables ``STREAM 80``, issues ``TURN``, and
            calls ``on_tick(robot)`` after each TLM tick.  Return ``False``
            from ``on_tick`` to abort.
        eps:
            Optional heading tolerance in centi-degrees (default 300 = 3°).
        timeout_s:
            Maximum wall-clock seconds to wait.
        """
        if on_tick is None:
            self._proto.turn(heading, eps=eps)
            timeout = int(timeout_s * 1000)
            outcome, _ = self._proto.wait_for_evt_done("TURN", timeout)
            return outcome
        else:
            self._proto.stream(80)
            self._proto.turn(heading, eps=eps)
            return self._run_until_done("TURN", on_tick, timeout_s)

    def read_encoders(self) -> tuple[int, int]:
        """Return cached encoder state (updated by streaming TLM frames)."""
        return self.encoders

    def zero_encoders(self) -> None:
        """Zero encoder counters (ZERO enc command)."""
        self._proto.zero_encoders()
        self.state = RobotState(
            pose=self.state.pose,
            v=self.state.v,
            omega=self.state.omega,
            accel=self.state.accel,
            stamp=self.state.stamp,
            encoders=(0, 0),
            twist=self.state.twist,
            line=self.state.line,
            color=self.state.color,
            world_pose=self.state.world_pose,
            otos_pose=self.state.otos_pose,
        )

    def send(self, message: str, read_timeout: int = 500) -> dict[str, Any]:  # [ms]
        """Send arbitrary v2 command string, return raw response dict."""
        return self._proto.send(message, read_timeout)

    # ------------------------------------------------------------------
    # Streaming drive
    # ------------------------------------------------------------------

    def vw(
        self,
        v: int,  # [mm/s]
        omega: int,  # [mrad/s]
        *,
        period: int = 40,  # [ms]
    ) -> Generator[None, None, None]:
        """Body-velocity streaming generator.  Yields ``None`` once per TLM tick.

        Before each yield, ``robot.state`` is updated from the incoming TLM frame
        via ``_apply_tlm``.  The caller reads ``robot.state`` directly after each
        ``yield``::

            for _ in robot.vw(200, 500):
                print(robot.state.encoders)
                if close_enough:
                    break  # sends STOP + STREAM 0 cleanly

        Protocol sequence
        -----------------
        1. ``STREAM <period>`` — enable TLM at the requested period (binary,
           097-003 -- see ``NezhaProtocol.stream()``'s own docstring).
        2. ``VW <v> <omega>`` — start body-velocity drive.
        3. Loop: drain binary telemetry frames non-blocking (097-003), call
           ``_apply_tlm`` then ``yield`` for each; then read text lines for
           50 ms for ``EVT safety_stop`` (EVT emission is unaffected by
           ``stream()``'s binary conversion, still text); exit naturally on
           ``EVT safety_stop``.
        4. Re-send ``VW`` as a keepalive whenever
           ``period * 0.30 / 1000`` seconds have elapsed since the last
           send (≤30% of the firmware watchdog window).
        5. On ``GeneratorExit`` (caller ``break``): send ``STOP`` then
           ``STREAM 0``; suppress exceptions so the generator exits cleanly.

        Parameters
        ----------
        v:
            Forward speed in mm/s (−1000 … +1000).
        omega:
            Yaw rate in milli-radians/s (−3142 … +3142); positive = CCW.
        period:
            TLM streaming period and keepalive base interval in milliseconds.
            Default 40 ms (25 Hz).
        """
        vw_cmd = f"VW {v} {omega}"
        keepalive_s = period * 0.30 / 1000.0

        self._proto.stream(period)

        try:
            self._proto._conn.send_fast(vw_cmd)
            last_send = time.monotonic()
            while True:
                for tlm in self._proto.read_pending_binary_tlm_frames():
                    self._apply_tlm(tlm)
                    yield None
                    now = time.monotonic()
                    if now - last_send >= keepalive_s:
                        self._proto._conn.send_fast(vw_cmd)
                        last_send = now
                for raw_line in self._proto._conn.read_lines(duration=50):
                    r = parse_response(raw_line)
                    if r is None:
                        continue
                    if r.tag == "EVT" and r.tokens and r.tokens[0] == "safety_stop":
                        return
                    now = time.monotonic()
                    if now - last_send >= keepalive_s:
                        self._proto._conn.send_fast(vw_cmd)
                        last_send = now
                now = time.monotonic()
                if now - last_send >= keepalive_s:
                    self._proto._conn.send_fast(vw_cmd)
                    last_send = now
        except GeneratorExit:
            try:
                self._proto._conn.send_fast("STOP")
                self._proto.stream(0)
            except Exception:
                pass

    def send_drive(self, left: int, right: int) -> None:  # [mm/s]
        """Fire-and-forget S keepalive for manual control loops."""
        self._proto.drive(left, right)

    def stream_drive(
        self,
        speeds: list[int],
        *,
        period: int = 40,  # [ms]
        watchdog: int = 500,  # [ms]
    ) -> Generator[ParsedResponse, None, None]:
        """Streaming drive generator. Yields ParsedResponse for each incoming line.

        Updates self.encoders, self.otos_pose, self.line_sensor, self.color
        before yielding each parsed TLM frame. Mutate ``speeds`` in the caller
        loop to change velocity mid-stream. Ends naturally on EVT safety_stop.
        """
        for resp in self._proto.stream_drive(
            speeds, period=period, watchdog=watchdog
        ):
            if resp.tag == "TLM":
                # 097-003: binary telemetry frames arrive already parsed, on
                # resp.tlm -- see ParsedResponse.tlm's own docstring.
                if resp.tlm:
                    self._apply_tlm(resp.tlm)
            yield resp

    def _apply_tlm(self, tlm: TLMFrame) -> None:
        """Construct a new frozen RobotState from the incoming TLMFrame.

        Fields absent from the frame retain their previous values from
        ``self.state`` (partial-frame handling).
        """
        prev = self.state

        # Pose: update if the frame carries a pose= field.
        if tlm.pose is not None:
            x, y, heading = tlm.pose  # [mm], [mm], [cdeg]
            # heading is centi-degrees (integer); convert to radians CCW-positive
            yaw_rad = math.radians(heading / 100.0)
            new_pose = Pose(x=float(x), y=float(y), heading=yaw_rad)
        else:
            new_pose = prev.pose

        # Fused body-frame velocity (optional twist= field).
        if tlm.twist is not None:
            v_mmps, omega_mradps = tlm.twist
            new_v = float(v_mmps)
            new_omega = float(omega_mradps) / 1000.0  # mrad/s → rad/s
        else:
            new_v = prev.v
            new_omega = prev.omega

        # Raw OTOS pose (optional otos= field): x,y mm and heading centi-degrees,
        # same encoding as pose=. Kept separate from the fused pose above.
        if tlm.otos is not None:
            ox, oy, oh = tlm.otos  # [mm], [mm], [cdeg]
            new_otos = (float(ox), float(oy), math.radians(oh / 100.0))
        else:
            new_otos = prev.otos_pose

        self.state = RobotState(
            pose=new_pose,
            v=new_v,
            omega=new_omega,
            accel=prev.accel,
            stamp=time.monotonic(),
            encoders=tlm.enc if tlm.enc is not None else prev.encoders,
            twist=tlm.twist if tlm.twist is not None else prev.twist,
            line=tlm.line if tlm.line is not None else prev.line,
            color=tlm.color if tlm.color is not None else prev.color,
            world_pose=prev.world_pose,
            otos_pose=new_otos,
        )

    # ------------------------------------------------------------------
    # Ping / identity
    # ------------------------------------------------------------------

    def ping(self) -> tuple[int, float] | None:
        """Send PING, return (t_robot, rtt) or None."""
        return self._proto.ping()

    def get_id(self) -> dict[str, str] | None:
        """Send ID command, return identity kv dict or None."""
        return self._proto.get_id()

    # ------------------------------------------------------------------
    # Config
    # ------------------------------------------------------------------

    def get_config(self, *keys: str) -> dict[str, str] | None:
        """Read config (GET [keys...]). Returns key->value dict or None."""
        return self._proto.get_config(*keys)

    def set_config(self, **kwargs: Any) -> dict[str, str] | None:
        """Write config keys (SET key=val ...). Returns applied kv or None."""
        return self._proto.set_config(**kwargs)

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def stream_tlm(self, period: int) -> None:  # [ms]
        """Enable TLM streaming at the given period (STREAM <ms>). 0 = off."""
        self._proto.stream(period)

    def snap(self) -> None:
        """Request one immediate TLM frame (SNAP command).

        Low-level fire-and-forget wrapper; the returned frame is discarded.
        Use ``refresh()`` to issue SNAP and capture the result into ``state``.
        """
        self._proto.snap()

    def refresh(self) -> "RobotState":
        """Issue a one-shot SNAP and return the updated robot state.

        Calls ``self._proto.snap()`` which synchronously returns a parsed
        ``TLMFrame`` (or ``None`` if the robot sent no TLM in its response).
        When a frame is received it is fed through ``_apply_tlm`` to update
        ``self.state``; when no frame is received the prior ``self.state`` is
        returned unchanged.

        This is the idle-state-query path — no streaming, no background thread.

        Returns
        -------
        RobotState
            The current (possibly just-refreshed) robot state.
        """
        tlm = self._proto.snap()
        if tlm is not None:
            self._apply_tlm(tlm)
        return self.state

    # ------------------------------------------------------------------
    # OTOS sensor management
    # ------------------------------------------------------------------

    def zero_otos(self) -> None:
        """Zero the OTOS position at the current location (ZERO pose command)."""
        self._proto.zero_otos()
        prev = self.state
        self.state = RobotState(
            pose=Pose(x=0.0, y=0.0, heading=prev.pose.heading),
            v=prev.v,
            omega=prev.omega,
            accel=prev.accel,
            stamp=prev.stamp,
            encoders=prev.encoders,
            twist=prev.twist,
            line=prev.line,
            color=prev.color,
            world_pose=prev.world_pose,
            otos_pose=prev.otos_pose,
        )

    def init_otos(self) -> None:
        """Initialise the OTOS sensor (OI command). Robot must be still."""
        self._proto.otos_init()

    def reset_otos_tracking(self) -> None:
        """Reset OTOS Kalman filters (OR command)."""
        self._proto.otos_reset_tracking()

    def set_world_pose(self, x: int, y: int, heading: int) -> None:  # [mm], [mm], [cdeg]
        """Set OTOS world-frame pose (OV command). Heading in centi-degrees."""
        self._proto.otos_set_position(x, y, heading)

    def update_world_pose(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
        """Set the world-frame pose from camera-native units and record it in state.

        Converts camera-native units to firmware wire units and calls
        ``set_internal_pose`` (SI command -> Odometry::setPose), which anchors the
        motion controller's pose (poseX/poseY/poseHrad) so getPose/telemetry then
        report WORLD coordinates directly.  Records the camera-native values in
        ``self.state.world_pose`` as ``(x_cm, y_cm, yaw_rad)`` so callers can read
        the last-set world pose back without unit conversion.

        (Was OV — the raw-OTOS-chip nudge — which does NOT set the controller pose
        and lands rotated by the OTOS mount angle; that was the +90° trace bug.)

        Unit conventions
        ----------------
        Input  (camera-native): centimetres for x/y, radians for heading.
        Wire   (firmware):      ``x = round(x_cm * 10)``,
                                ``y = round(y_cm * 10)``,
                                ``heading = round(degrees(yaw_rad) * 100)``.
        Stored (state):         ``(x_cm, y_cm, yaw_rad)`` — camera units, unchanged.

        Parameters
        ----------
        x_cm:
            World x-position in centimetres.
        y_cm:
            World y-position in centimetres.
        yaw_rad:
            World heading in radians (CCW-positive, 0 = +x/east, camera frame).
        """
        x = round(x_cm * 10)
        y = round(y_cm * 10)
        heading = round(math.degrees(yaw_rad) * 100)
        self._proto.set_internal_pose(x, y, heading)
        prev = self.state
        self.state = RobotState(
            pose=prev.pose,
            v=prev.v,
            omega=prev.omega,
            accel=prev.accel,
            stamp=prev.stamp,
            encoders=prev.encoders,
            twist=prev.twist,
            line=prev.line,
            color=prev.color,
            world_pose=(x_cm, y_cm, yaw_rad),
            otos_pose=prev.otos_pose,
        )

    def read_otos_pose(self) -> tuple[int, int, int] | None:
        """Query current OTOS pose (OP command). Returns (x, y, heading) or None."""
        return self._proto.otos_get_position()

    def set_otos_linear_scalar(self, val: int) -> int | None:
        """Set OTOS linear scalar (OL <val>). Returns confirmed value or None."""
        return self._proto.otos_set_linear_scalar(val)

    def get_otos_linear_scalar(self) -> int | None:
        """Read back OTOS linear scalar (OL). Returns value or None."""
        return self._proto.otos_get_linear_scalar()

    def set_otos_angular_scalar(self, val: int) -> int | None:
        """Set OTOS angular scalar (OA <val>). Returns confirmed value or None."""
        return self._proto.otos_set_angular_scalar(val)

    def get_otos_angular_scalar(self) -> int | None:
        """Read back OTOS angular scalar (OA). Returns value or None."""
        return self._proto.otos_get_angular_scalar()

    # ------------------------------------------------------------------
    # J-port I/O
    # ------------------------------------------------------------------

    def port_read(self, port: int) -> int | None:
        """Read digital J-port (P <port>). Returns 0/1 or None."""
        return self._proto.port_read(port)

    def port_write(self, port: int, value: bool) -> None:
        """Write digital J-port (P <port> <val>)."""
        self._proto.port_write(port, value)

    def port_read_analog(self, port: int) -> int | None:
        """Read analog J-port (PA <port>). Returns 0-1023 or None."""
        return self._proto.port_read_analog(port)

    def port_write_analog(self, port: int, value: int) -> None:
        """Write PWM (0-1023) to J-port (PA <port> <val>)."""
        self._proto.port_write_analog(port, value)

    # ------------------------------------------------------------------
    # Connection access
    # ------------------------------------------------------------------

    @property
    def connection(self) -> NezhaProtocol:
        return self._proto
