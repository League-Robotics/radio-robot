"""push_calibration — send calibration values to firmware.

Resolves the interface duality between the MCP path (NezhaProtocol) and the
CLI path (SerialConnection):

- When passed a ``NezhaProtocol``: if the proto already has a
  ``push_calibration`` method, delegates to it.  Otherwise extracts the
  underlying ``SerialConnection`` (``proto._conn``) and falls through to the
  direct-SET path.  This ensures forward-compatibility with the wiring that
  ticket 028-003 adds to NezhaProtocol.

- When passed a ``SerialConnection``: constructs and sends the v2 SET command
  sequence directly.

Both paths return a result dict with at minimum a ``"status"`` key.

Note: this module does NOT wire push_calibration into cli.py or robot_mcp.py.
That is ticket 028-003.
"""

from __future__ import annotations

import math
import sys
from typing import Any



def push_calibration(conn_or_proto: Any, config: Any) -> dict[str, Any]:
    """Push calibration values to firmware.

    Parameters
    ----------
    conn_or_proto:
        Either a :class:`robot_radio.robot.protocol.NezhaProtocol` or a
        :class:`robot_radio.io.serial_conn.SerialConnection`.
    config:
        A :class:`robot_radio.config.robot_config.RobotConfig` (or any object
        with the same attribute structure).

    Returns
    -------
    dict
        ``{"status": "ok", ...}`` on success.  The dict may carry additional
        diagnostic keys (e.g. ``"commands"`` listing the verbs that were sent).
    """
    # Resolve duality: NezhaProtocol vs SerialConnection.
    from robot_radio.robot.protocol import NezhaProtocol
    from robot_radio.io.serial_conn import SerialConnection

    if isinstance(conn_or_proto, NezhaProtocol):
        proto = conn_or_proto
        # If NezhaProtocol has its own push_calibration method (added by
        # ticket 028-003), delegate to it.  Check both the instance and the
        # class so that either a monkey-patched instance attribute or a real
        # class method is found.
        try:
            _push_fn = object.__getattribute__(proto, "push_calibration")
        except AttributeError:
            _push_fn = getattr(type(proto), "push_calibration", None)
        if _push_fn is not None and callable(_push_fn):
            return _push_fn(config)
        # Otherwise extract the underlying connection and fall through.
        conn = proto._conn
    elif isinstance(conn_or_proto, SerialConnection):
        conn = conn_or_proto
    else:
        raise TypeError(
            f"push_calibration expects NezhaProtocol or SerialConnection, "
            f"got {type(conn_or_proto).__name__}"
        )

    return _push_via_conn(conn, config)


def calibration_kwargs(config: Any) -> dict[str, float | int]:
    """Select the Tier-1 (already-wire-covered) calibration field set from
    *config*, as a flat wire-key kwargs dict (``{"ml": ..., "pid.kp": ...,
    ...}``) — the SAME wire-key vocabulary ``protocol.py``'s
    ``_MOTOR_PID_KEYS``/``_DRIVETRAIN_KEYS`` curate.

    Pure, side-effect-free field SELECTION only — no text formatting, no
    transport. This is what ``calibration_commands()`` (below) formats into
    ``SET key=value`` strings for the hardware/CLI path; ticket 113-005's
    ``SimLoop`` calls this directly and passes the dict straight to
    ``NezhaProtocol.set_config(**kwargs)``, skipping the text round trip
    entirely.

    Covers, in order:
      - ``ml``/``mr`` — mm_per_wheel_deg_left/right (wheel-diameter-derived
        default when uncalibrated).
      - ``tw`` — trackwidth mm.
      - ``rotSlip`` — calibration.rotational_slip.  ALWAYS present: an
        uncalibrated config (rotational_slip null/missing) resolves to the
        documented "no correction" sentinel ``0`` (``effectiveSlip()`` maps
        0 -> 1.0), so a no-calibration robot NEUTRALIZES whatever value is
        baked into the firmware's compiled-in DefaultConfig instead of
        silently inheriting it.  This is what makes "select tovez nocal →
        turns are geometry-pure" true in the sim.
      - ``pid.kp/ki/kff/iMax/kaw`` — the velocity-PID gains (``control.vel_*``
        → ``MotorConfigPatch`` Gains, applied to BOTH motors by
        ``RobotLoop::handleConfig``). Stakeholder 2026-07-18: selecting a
        robot must be authoritative for the CONTROL gains too — the sim
        binary bakes its own harness gains (``SimHarness::makeMotorConfig()``)
        and real firmware bakes whatever robot JSON was active at build time
        (``gen_boot_config.py``), and neither may silently leak into a
        session. Each key is present only when the config carries a value
        (``ControlConfig``'s own contract: None → firmware boot default
        kept). ``control.vel_filt`` has NO live ``SET`` key and is
        deliberately not pushed — it reaches firmware at build time only
        (``gen_boot_config.py`` → ``MotorConfig.setVelFiltAlpha()``).
        ``headingKp``/``headingKd``/``minSpeed``/``distanceKp``/
        ``arriveDwell`` (113-003's own ``PlannerConfigPatch`` additions) —
        DELETED (115-003, gut-to-minimal-firmware S1 motion-stack excision):
        `PlannerConfigPatch`/`ConfigDelta.planner` and the `App::Pilot` that
        applied them are gone; none of these five keys are valid
        `set_config()` wire keys any more (see `protocol.py`'s own
        `_ALL_SET_KEYS`, which no longer contains them) — pushing any of
        them fails the WHOLE `set_config()` call (returns `None`, no wire
        transmission at all), which is exactly why this function must NOT
        still select them.

    ``OI``/``OL``/``OA`` (OTOS) are deliberately OUT of this dict — they are
    not ``SET key=value`` verbs at all (see ``calibration_commands()``'s own
    docstring on ``otos_config()`` being a separate mechanism), so they have
    no place in a flat kwargs dict.  ``calibration_commands()`` builds them
    directly, unchanged.

    Does NOT select ``config.geometry.odometry_offset_mm`` (the OTOS
    mounting-offset/lever-arm) — see ``calibration_commands()``'s own
    docstring for the full rationale (unregistered `SET` keys, ``ERR
    badkey``).
    """
    kwargs: dict[str, float | int] = {}

    # ── Wheel encoder calibration and trackwidth ──────────────────────────
    cal = getattr(config, "calibration", None)

    wd = getattr(getattr(config, "wheels", None), "wheel_diameter_mm", None)
    default_wheel_travel_calib = (math.pi * wd / 360.0) if wd is not None else None  # [mm/deg]

    wheel_travel_calib_left  = getattr(cal, "mm_per_wheel_deg_left",  None) if cal else None
    wheel_travel_calib_right = getattr(cal, "mm_per_wheel_deg_right", None) if cal else None
    wheel_travel_calib_left  = wheel_travel_calib_left  if wheel_travel_calib_left  is not None else default_wheel_travel_calib
    wheel_travel_calib_right = wheel_travel_calib_right if wheel_travel_calib_right is not None else default_wheel_travel_calib

    if wheel_travel_calib_left is not None:
        kwargs["ml"] = float(wheel_travel_calib_left)
    if wheel_travel_calib_right is not None:
        kwargs["mr"] = float(wheel_travel_calib_right)

    geom = getattr(config, "geometry", None)
    tw = getattr(geom, "trackwidth", None) if geom else None
    if tw is not None:
        kwargs["tw"] = int(round(float(tw)))

    # ── Rotational slip: always present, uncalibrated -> sentinel 0 ───────
    rot_slip = getattr(cal, "rotational_slip", None) if cal else None
    kwargs["rotSlip"] = float(rot_slip) if rot_slip is not None else 0.0

    # ── Velocity-PID gains: present when set ───────────────────────────────
    # See this function's docstring.  Wire keys are protocol.py's own
    # vocabulary (_MOTOR_PID_KEYS); both hardware
    # (binary_bridge.translate_command → NezhaProtocol.set_config) and Sim
    # (SimTransport._handle_config_set → NezhaProtocol.config) accept them.
    # headingKp/headingKd/minSpeed/distanceKp/arriveDwell -- DELETED
    # (115-003, gut S1 motion-stack excision): see this function's own
    # docstring.
    ctrl = getattr(config, "control", None)
    for wire_key, attr in (
        ("pid.kp", "vel_kp"),
        ("pid.ki", "vel_ki"),
        ("pid.kff", "vel_kff"),
        ("pid.iMax", "vel_imax"),
        ("pid.kaw", "vel_kaw"),
    ):
        value = getattr(ctrl, attr, None) if ctrl is not None else None
        if value is not None:
            kwargs[wire_key] = float(value)

    return kwargs


# Wire keys formatted with a plain "%.6f" (matches the pre-113-003 text
# implementation's own ml/mr formatting exactly) rather than the "%g" every
# other SET key below uses.
_SIX_DECIMAL_KEYS = frozenset({"ml", "mr"})


def calibration_commands(config: Any) -> list[tuple[str, int]]:
    """Build the v2 calibration wire-command sequence for *config*.

    Pure function — returns ``(command, read_timeout)`` pairs and sends nothing,
    so any transport (SerialConnection, NezhaProtocol, or the TestGUI's
    Transport) can push the same sequence.  Mirrors the logic in
    ``robot_radio.io.cli._push_calibration``; changes there should be
    ported here.

    113-003: a thin formatting wrapper over ``calibration_kwargs()`` (above)
    — that function SELECTS which fields to push; this one FORMATS each
    selected item into a ``SET key=value`` text command (``tw`` as a plain
    int, ``ml``/``mr`` to 6 decimal places, everything else via ``%g``,
    matching the pre-113-003 text implementation's own per-key formatting
    exactly), then appends the OTOS ``OI``/``OL``/``OA`` sequence, which
    ``calibration_kwargs()`` deliberately does not cover. Behavior-preserving
    for every existing caller (``cli.py``, ``turn_shape.py``, ``__main__.py``'s
    manual robot-select): byte-identical output to the pre-113-003
    implementation for every existing config shape.

    The sequence:
      1. ``SET ml=<float>``  — mm_per_wheel_deg_left
      2. ``SET mr=<float>``  — mm_per_wheel_deg_right
      3. ``SET tw=<int>``    — trackwidth mm
      4. ``SET rotSlip=<float>`` — calibration.rotational_slip.  ALWAYS
         sent: an uncalibrated config (rotational_slip null/missing) pushes
         the documented "no correction" sentinel ``0`` (``effectiveSlip()``
         maps 0 -> 1.0), so a no-calibration robot NEUTRALIZES whatever
         value is baked into the firmware's compiled-in DefaultConfig
         instead of silently inheriting it.  This is what makes "select
         tovez nocal → turns are geometry-pure" true in the sim.
      5. ``SET pid.kp/ki/kff/iMax/kaw=<float>`` — the velocity-PID gains
         (``control.vel_*`` → ``MotorConfigPatch`` Gains, applied to BOTH
         motors by ``RobotLoop::handleConfig``). See ``calibration_kwargs()``'s
         own docstring for the full field-selection rationale (including why
         the former ``headingKp``/``headingKd``/``minSpeed``/``distanceKp``/
         ``arriveDwell`` steps are gone -- 115-003 deleted
         ``PlannerConfigPatch`` wholesale).
      6. ``OI``              — OTOS init (must precede OL/OA)
      7. ``OL <int8>``       — otos_linear_scale encoded
      8. ``OA <int8>``       — otos_angular_scale encoded

    109-004 RESTORES steps 5-7 (dropped 2026-07-16, out-of-process, when
    ``OI``/``OL``/``OA`` had no path over the current binary wire at all —
    see this function's own git history / issue
    ``otos-calibration-config-message.md``): ``binary_bridge.py``'s
    ``translate_command()`` now intercepts these three verbs and constructs/
    sends an ``OtosConfigPatch`` ``ConfigDelta`` directly
    (``NezhaProtocol.otos_config()``), on both hardware and Sim transports
    (``SimTransport._handle_otos_patch()``) — so the push below reaches a
    real firmware consumer again (``RobotLoop::handleConfig``'s new OTOS
    case) instead of producing "not supported"/"nodev" noise on every
    connect. ``OL``/``OA`` still carry the chip's RAW int8 register scalar
    (``scale_to_int8()``, NOT the raw ``otos_linear_scale``/
    ``otos_angular_scale`` multiplier itself) — the exact same encoding the
    pre-2026-07-16 text-plane push used, unchanged by this restoration
    (``Otos::setLinearScalar()``/``setAngularScalar()`` still expect the raw
    register value — see ``otos.h``'s own OL/OA doc comment).

    Does NOT push ``config.geometry.odometry_offset_mm`` (the OTOS
    mounting-offset/lever-arm): ``odomOffX``/``odomOffY``/``odomYaw`` are not
    in ``config_commands.cpp``'s registered `SET` key table
    (architecture-update.md (084) Decision 2's closed 15-key surface) — a
    push of any of them gets ``ERR badkey`` from the current firmware/sim
    (ticket 085-005 finding; also observed independently during ticket
    085-002/003's manual runs). This is not new drift 084 introduced: the
    OTOS lever-arm has no real hardware driver in this program at all, and
    OTOS pose is otherwise configured entirely via ``OI``/``OL``/``OA``/``OV``,
    never `SET` — so this function still does not push it (109-004's
    ``OtosConfigPatch`` DOES carry offset_x/y/yaw wire capacity, but no host
    verb sends them yet — a future ticket's scope, not this restoration's).
    ``config.geometry.odometry_offset_mm`` itself (e.g.
    ``data/robots/tovez.json``'s non-zero ``x: -47.7``) is left as-is in the
    schema — this function is simply not (yet) one of its consumers.
    """
    cmds: list[tuple[str, int]] = []

    for key, value in calibration_kwargs(config).items():
        if key == "tw":
            cmds.append((f"SET tw={value}", 200))
        elif key in _SIX_DECIMAL_KEYS:
            cmds.append((f"SET {key}={value:.6f}", 200))
        else:
            cmds.append((f"SET {key}={value:g}", 200))

    # ── OTOS init + scalars: RESTORED (109-004) ───────────────────────────
    # Dropped 2026-07-16 (out-of-process) because OI/OL/OA had no path over
    # the binary wire at all; 109-004 gives them one (OtosConfigPatch,
    # RobotLoop::handleConfig's new OTOS case, binary_bridge.py's/
    # SimTransport's direct-patch-send interception of these three verbs) --
    # see this function's own docstring for the restoration's full
    # rationale. OI must precede OL/OA (chip init before the scale writes).
    # ALWAYS pushed, same "uncalibrated -> neutral sentinel" discipline as
    # rotSlip above: an uncalibrated config (otos_linear_scale/
    # otos_angular_scale null/missing) pushes the 1.0 "no correction"
    # default explicitly, overwriting whatever DefaultConfig.cpp baked in,
    # rather than silently omitting the write.
    from robot_radio.calibration.helpers import scale_to_int8

    cal = getattr(config, "calibration", None)
    lin_scale = getattr(cal, "otos_linear_scale",  None) if cal else None
    ang_scale = getattr(cal, "otos_angular_scale", None) if cal else None
    lin_scale = float(lin_scale) if lin_scale is not None else 1.0
    ang_scale = float(ang_scale) if ang_scale is not None else 1.0

    cmds.append(("OI", 500))
    cmds.append((f"OL {scale_to_int8(lin_scale)}", 200))
    cmds.append((f"OA {scale_to_int8(ang_scale)}", 200))

    # (The NOTE about OTOS mounting-offset never being pushable — `odomOff*`
    # aren't registered SET keys — still holds; that stays out too.)

    return cmds


def _push_via_conn(conn: Any, config: Any) -> dict[str, Any]:
    """Send ``calibration_commands(config)`` over *conn* (SerialConnection).

    Returns a dict with ``"status": "ok"`` and ``"commands"`` listing sent verbs.
    """
    sent: list[str] = []
    for cmd, read_timeout in calibration_commands(config):
        conn.send(cmd, read_timeout=read_timeout)
        sent.append(cmd)
    return {"status": "ok", "commands": sent}
