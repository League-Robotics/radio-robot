"""push_calibration — send calibration values to firmware.

Resolves the interface duality between the MCP path (NezhaProtocol) and the
CLI path (SerialConnection):

- When passed a ``NezhaProtocol``: pushes every selected field as its own
  ``set_config_field()`` round trip (132-014) -- ``calibration_kwargs()``'s
  ml/mr/pid.* keys plus ``otos_kwargs()``'s OTOS offset/scale fields, the
  SAME field set ``calibration_commands()`` below formats as text, just sent
  over the real binary CONFIG-arm wire instead of a text line the current
  firmware has no parser for.

- When passed a ``SerialConnection``: constructs and sends the v2 SET
  command sequence (``calibration_commands()``) as plain text lines. This
  path is KEPT for a bare ``SerialConnection`` caller, but does not reach a
  live firmware handler on the current wire (the P4 single-loop firmware's
  command plane is binary-only, ``docs/protocol-v5.md`` -- see
  ``NezhaProtocol.send()``'s own doc comment) -- a pre-existing gap from the
  102-107 rebuild, orthogonal to sprint 132's own changes, not fixed here.

Both paths return a result dict with at minimum a ``"status"`` key.

132-014 (migrate host NezhaProtocol + calibration/push.py + TestGUI onto the
new config surface): retargets every field SELECTOR below off the retired
flat ``config.control``/``config.calibration`` sections (removed when
``RobotConfig`` adopted ``robot_config.proto``'s consumer-grouped shape,
132-020) onto the new ``config.motors``/``config.wheel_control``/
``config.otos``/``config.estimator``/``config.planner`` groups, and retargets
every SENDER off the retired ``*ConfigPatch``/``ConfigDelta`` wire shapes
(deleted 132-013) onto ``NezhaProtocol.set_config_field()``.

KNOWN GAP, mid-sprint (unchanged by this ticket -- ticket 017's job): the
robot JSONs on disk (``data/robots/*.json``) are still in the OLD 13-section
shape, so ``config.motors``/``config.wheel_control``/etc. read their proto3
zero defaults for a REAL loaded robot config until ticket 017 reshapes the
files -- see ``robot_config.py``'s own "KNOWN GAP, mid-sprint" doc comment.
This module's job is field SELECTION from whatever shape ``config`` actually
carries; it does not, and must not, work around the JSON still being
unreshaped (sprint.md Design Rationale Decision 2 / Test Strategy).
"""

from __future__ import annotations

import math
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
        ``{"status": "ok"|"partial", ...}``.  The dict may carry additional
        diagnostic keys (e.g. ``"applied"``/``"rejected"`` listing the
        fields that succeeded/failed for the ``NezhaProtocol`` path,
        ``"commands"`` listing the verbs sent for the ``SerialConnection``
        path).
    """
    from robot_radio.robot.protocol import NezhaProtocol
    from robot_radio.io.serial_conn import SerialConnection

    if isinstance(conn_or_proto, NezhaProtocol):
        return _push_via_proto(conn_or_proto, config)
    elif isinstance(conn_or_proto, SerialConnection):
        conn = conn_or_proto
    else:
        raise TypeError(
            f"push_calibration expects NezhaProtocol or SerialConnection, "
            f"got {type(conn_or_proto).__name__}"
        )

    return _push_via_conn(conn, config)


def calibration_kwargs(config: Any) -> "dict[str, float]":
    """Select the Tier-1 (already-wire-covered) calibration field set from
    *config*, as a flat ``{wire_key: value}`` dict (``{"ml": ...,
    "pid.kp": ..., ...}``) -- the SAME wire-key vocabulary
    ``protocol.py``'s ``_SET_KEY_TARGETS`` curates, each resolving to a
    ``(ConfigGroupTarget, field_name)`` pair.

    Pure, side-effect-free field SELECTION only -- no wire I/O.
    ``SimLoop.configure_from_robot()`` (Tier 1) and
    ``_push_via_proto()``/``calibration_commands()`` below all call this
    directly.

    132-014: retargeted off the retired ``config.control``/
    ``config.calibration`` flat sections onto ``config.motors``/
    ``config.wheel_control`` (``robot_config.proto``'s consumer-grouped
    shape, 132-020). ``tw``/``rotSlip`` (``config.geometry.trackwidth``/
    ``rotational_slip``) are DROPPED, not migrated: GEOMETRY is a
    boot-only ``ConfigGroupTarget`` (configurator.h's own re-appliability
    table -- "trackWidth has no post-construction setter anywhere"), so a
    live push has NEVER taken effect (the old DrivetrainConfigPatch arm was
    ``ERR_UNIMPLEMENTED`` on the firmware side for the same reason,
    sprint.md's own Problem section) -- pushing them from HERE would also
    be the exact ``Config::Robot`` GEOMETRY-divergence hazard
    ``SimLoop.configure_from_robot()``'s own doc comment flags (trackWidth
    is one of the sim's justified ``BootOverrides`` divergences, never to
    be overwritten from a live push). ``set_config()``/the ``SET`` verb
    still recognize ``tw``/``rotSlip`` directly (``protocol._SET_KEY_TARGETS``)
    for a human/hardware caller that wants the honest ``ERR_NOT_LIVE`` --
    this function simply never selects them into a Tier-1/connect-time push.

    Covers, in order:
      - ``ml``/``mr`` -- ``config.motors.travel_calib_left/right``
        (``Motors.travel_calib_left/right``), falling back to the
        wheel-diameter-derived default (``config.wheels.wheel_diameter_mm``,
        a host-only section unaffected by the reshape) when the group field
        reads its proto3 zero default (0.0 is never a valid calibrated
        value -- ``robot_config.proto``'s own ``(min) = 0.0001`` bound).
      - ``pid.kp/ki/kff/iMax/kaw`` -- App::Drive's unified wheel-speed
        controller's Stage B fast-PID gains (``WheelControl.pid_kp/pid_ki/
        pid_kaff/pid_i_max/pid_max``), the direct successor of
        ``control.wheel_pid_*`` -- unchanged consumer (130-005), only the
        source group/field names moved.

    ``OI``/``OL``/``OA`` (OTOS) are deliberately OUT of this dict -- see
    ``otos_kwargs()`` below, a separate selector for a separate
    ``ConfigGroupTarget`` (OTOS, not MOTORS/WHEEL_CONTROL).
    """
    kwargs: "dict[str, float]" = {}

    # ── Wheel travel calibration (Motors group), diameter-derived fallback
    motors = getattr(config, "motors", None)
    wd = getattr(getattr(config, "wheels", None), "wheel_diameter_mm", None)
    default_travel_calib = (math.pi * wd / 360.0) if wd is not None else None  # [mm/deg]

    travel_calib_left = getattr(motors, "travel_calib_left", None) if motors is not None else None
    travel_calib_right = getattr(motors, "travel_calib_right", None) if motors is not None else None
    if not travel_calib_left and default_travel_calib is not None:
        travel_calib_left = default_travel_calib
    if not travel_calib_right and default_travel_calib is not None:
        travel_calib_right = default_travel_calib

    if travel_calib_left is not None:
        kwargs["ml"] = float(travel_calib_left)
    if travel_calib_right is not None:
        kwargs["mr"] = float(travel_calib_right)

    # ── App::Drive's unified wheel-speed controller: Stage B fast-PID gains
    wheel_control = getattr(config, "wheel_control", None)
    for wire_key, attr in (
        ("pid.kp", "pid_kp"),
        ("pid.ki", "pid_ki"),
        ("pid.kff", "pid_kaff"),
        ("pid.iMax", "pid_i_max"),
        ("pid.kaw", "pid_max"),
    ):
        value = getattr(wheel_control, attr, None) if wheel_control is not None else None
        if value is not None:
            kwargs[wire_key] = float(value)

    return kwargs


def otos_kwargs(config: Any) -> "dict[str, float]":
    """Select ``config.otos``'s OFFSET/SCALE fields as a flat
    ``{field_name: value}`` dict, ready for
    ``proto.set_config_field(robot_config_pb2.OTOS, field_name, value)``
    per entry -- the OTOS counterpart of ``calibration_kwargs()`` above,
    selecting from a DIFFERENT ``ConfigGroupTarget`` (OTOS, not
    MOTORS/WHEEL_CONTROL).

    132-014 (new function, replacing the OI/OL/OA text-verb-specific
    handling ``calibration_commands()`` used to own alone): ``linear_scale``/
    ``angular_scale`` here are already in the config MULTIPLIER domain
    (1.0 = no correction) -- unlike the pre-132 text-plane ``OL``/``OA``
    verbs (which carried the chip's raw int8 register scalar,
    ``scale_to_int8()``-encoded), the live wire push applies NO such
    encoding: ``App::configureOtos()`` (132-010, trap 3 closed) converts
    the multiplier through ``Devices::scaleToRegister()`` FIRMWARE-side
    now, so a live push and a boot bake finally agree on what a given
    multiplier means. ``binary_bridge.py``'s ``OL <scale>``/``OA <scale>``
    verbs already pass their argument straight through as the multiplier
    (no host-side encoding) -- this function's values are pushed the same
    way.

    Fields not present on *config* (``getattr`` returns ``None``) are
    simply omitted -- there is no "uncalibrated -> neutral sentinel"
    push here (unlike ``rotSlip``'s old sentinel discipline): an absent
    OTOS field means "leave whatever ``loadBaked()``/the last push
    already installed alone."
    """
    kwargs: "dict[str, float]" = {}
    otos = getattr(config, "otos", None)
    for attr in ("offset_x", "offset_y", "offset_yaw", "linear_scale", "angular_scale"):
        value = getattr(otos, attr, None) if otos is not None else None
        if value is not None:
            kwargs[attr] = float(value)
    return kwargs


# ESTIMATOR_FIELDS / PLANNER_SHAPER_FIELDS -- exported (no leading
# underscore) so estimator_kwargs()'s callers (sim_loop.py Tier 3,
# testgui/__main__.py's _push_estimator_config()) can partition its single
# flat return dict back into its two wire targets without re-deriving the
# field lists.
ESTIMATOR_FIELDS = ("weight_heading_otos", "weight_omega_otos", "staleness")
PLANNER_SHAPER_FIELDS = ("a_max", "a_decel", "alpha_max", "alpha_decel", "jerk_max", "yaw_jerk_max")


def estimator_kwargs(config: Any) -> "dict[str, float]":
    """Select the fusion-weight + shaper-ceiling field set from *config*, as
    a flat ``{field_name: value}`` dict spanning TWO ``ConfigGroupTarget``s
    -- ``ESTIMATOR_FIELDS`` (``config.estimator.*``) and
    ``PLANNER_SHAPER_FIELDS`` (``config.planner_shaper.*``). Callers push
    each key via ``set_config_field(ESTIMATOR or PLANNER_SHAPER, key,
    value)``, selecting the target with ``key in ESTIMATOR_FIELDS``/``key
    in PLANNER_SHAPER_FIELDS`` (module-level constants above).

    132-014: retargeted off the retired single ``EstimatorConfigPatch``
    binary arm (``config.proto``, one ``ConfigDelta.estimator`` patch
    carrying all nine fields in ONE envelope) -- that whole patch type is
    deleted (132-013). The nine fields now live on TWO different
    ``robot_config.proto`` groups instead of one Patch message:

      - ``config.estimator.weight_heading_otos``/``weight_omega_otos``/
        ``staleness`` -- unchanged consumer intent (``App::StateEstimator``'s
        complementary-blend fusion weights), but that consumer was ALREADY
        deleted as dead code before this sprint (sprint 128 ticket 016) --
        ``ESTIMATOR`` decodes these for read-back but ``install(ESTIMATOR)``
        permanently returns ``ERR_UNIMPLEMENTED`` (configurator.h). A push
        here is honestly rejected, not silently dropped (the old
        ``EstimatorConfigPatch``'s own "acks 0, lands nowhere" trap,
        closed) -- this function still selects them (read-back honesty),
        it is the CALLER's job to log/tolerate the rejection.
      - ``config.planner_shaper.a_max``/``a_decel``/``alpha_max``/
        ``alpha_decel``/``jerk_max``/``yaw_jerk_max`` --
        ``Motion::Planner``'s accel/jerk ceilings, formerly their OWN
        dedicated always-live wire arm (riding the SAME
        ``EstimatorConfigPatch`` envelope as a "smallest coherent path"
        convenience, config.proto's own doc comment), then folded into
        ``robot_config.proto``'s ``Planner`` message (a BOOT-ONLY
        ``ConfigGroupTarget`` in full, 132-002 through 132-013) -- a real,
        if temporary, capability REGRESSION this sprint's own architecture
        introduced (a live push got the honest ``ERR_NOT_LIVE`` every
        time, not a renaming).

        FIXED, 132-017 (JSON reshape ticket, stakeholder-sanctioned
        mid-sprint scope addition): the six fields split OUT of Planner
        into their own ``PlannerShaper`` group/``PLANNER_SHAPER``
        ``ConfigGroupTarget``, which IS live (its own setter,
        ``Motion::Planner::applyShaperLimits()``, was already one of the
        issue's eight safely re-appliable setters -- the coarse PLANNER
        grouping was the only thing forcing it boot-only). Selected from
        ``config.planner_shaper`` now, not ``config.planner`` -- the
        pydantic model shape moved with the split (``robot_config.py``'s
        own ``planner_shaper`` field).

    Each key is present only when *config* carries a non-``None`` value for
    it (``getattr`` returns ``None`` when the source group itself is
    ``None`` or the attribute is absent). Returns ``{}`` if *config* carries
    none of the nine fields at all -- the caller must treat that as
    "nothing to push."
    """
    kwargs: "dict[str, float]" = {}

    est = getattr(config, "estimator", None)
    for attr in ESTIMATOR_FIELDS:
        value = getattr(est, attr, None) if est is not None else None
        if value is not None:
            kwargs[attr] = float(value)

    planner_shaper = getattr(config, "planner_shaper", None)
    for attr in PLANNER_SHAPER_FIELDS:
        value = getattr(planner_shaper, attr, None) if planner_shaper is not None else None
        if value is not None:
            kwargs[attr] = float(value)

    return kwargs


# Wire keys formatted with a plain "%.6f" (matches the pre-113-003 text
# implementation's own ml/mr formatting exactly) rather than the "%g" every
# other SET key below uses.
_SIX_DECIMAL_KEYS = frozenset({"ml", "mr"})

# Per-command read timeouts for calibration_commands()'s pushed sequence.
_SET_READ_TIMEOUT_MS = 200
_OTOS_INIT_READ_TIMEOUT_MS = 500


def calibration_commands(config: Any) -> "list[tuple[str, int]]":
    """Build the v2 calibration wire-command sequence for *config*, as
    plain TEXT ``SET key=value``/``OI``/``OL``/``OA`` lines.

    Pure function -- returns ``(command, read_timeout)`` pairs and sends
    nothing. KEPT for ``_push_via_conn()``'s ``SerialConnection`` path and
    for direct callers that still want the text-line shape (e.g.
    ``__main__.py``'s per-command push-loop UI, which logs each line's
    reply) -- see this module's own header comment for why this text plane
    does not reach a live firmware handler on the current wire regardless
    of this function's own correctness (pre-existing, orthogonal gap from
    the 102-107 rebuild).

    132-014: a thin formatting wrapper over ``calibration_kwargs()``/
    ``otos_kwargs()`` (above) -- unchanged shape, retargeted field sources.
    ``OL``/``OA`` now format the MULTIPLIER directly (``otos_kwargs()``'s
    own domain -- see that function's docstring for why ``scale_to_int8()``
    is no longer applied here: the live wire push does the register
    conversion firmware-side now, 132-010, and this text path mirrors that
    for consistency even though it reaches no parser today). ``OI`` is
    still emitted (an inert no-op token on the current wire regardless --
    see this module's header comment) for continuity with the pre-132
    command shape; nothing consumes it as a trigger any more.

    The sequence:
      1. ``SET ml=<float>``/``SET mr=<float>`` -- wheel travel calib
      2. ``SET pid.kp/ki/kff/iMax/kaw=<float>`` -- Stage B fast-PID gains
      3. ``OI``              -- inert legacy token, see docstring above
      4. ``OL <float>``/``OA <float>`` -- OTOS linear/angular scale, MULTIPLIER
    """
    cmds: "list[tuple[str, int]]" = []

    for key, value in calibration_kwargs(config).items():
        if key in _SIX_DECIMAL_KEYS:
            cmds.append((f"SET {key}={value:.6f}", _SET_READ_TIMEOUT_MS))
        else:
            cmds.append((f"SET {key}={value:g}", _SET_READ_TIMEOUT_MS))

    otos = otos_kwargs(config)
    cmds.append(("OI", _OTOS_INIT_READ_TIMEOUT_MS))
    if "linear_scale" in otos:
        cmds.append((f"OL {otos['linear_scale']:g}", _SET_READ_TIMEOUT_MS))
    if "angular_scale" in otos:
        cmds.append((f"OA {otos['angular_scale']:g}", _SET_READ_TIMEOUT_MS))

    return cmds


def _push_via_proto(proto: Any, config: Any) -> "dict[str, Any]":
    """Push *config*'s calibration to *proto* (a real ``NezhaProtocol``)
    over the LIVE binary wire -- one ``set_config_field()`` round trip per
    selected field (``calibration_kwargs()``'s ml/mr/pid.* plus
    ``otos_kwargs()``'s OTOS offset/scale fields), instead of the dead
    ``SerialConnection``-text path ``_push_via_conn()`` below still serves.

    132-014 (new function): this is what makes a REAL ``NezhaProtocol``
    caller of ``push_calibration()`` (``io/robot_mcp.py``) actually reach
    firmware -- before this ticket, every ``NezhaProtocol`` call fell
    through to ``_push_via_conn(proto._conn, config)``, sending plain text
    the current firmware's binary-only command plane cannot parse (see
    this module's own header comment) -- a pre-existing gap, unrelated to
    sprint 132, closed here as a natural consequence of giving
    ``push_calibration()`` a real binary implementation.

    Returns ``{"status": "ok"|"partial", "applied": [...], "rejected": [...]}``
    -- ``applied``/``rejected`` name each pushed field (``"ml"``,
    ``"otos.linear_scale"``, ...), never silently swallowing a rejection.
    """
    from robot_radio.robot import protocol as protocol_mod
    from robot_radio.robot.pb2 import robot_config_pb2

    applied: "list[str]" = []
    rejected: "list[str]" = []

    for key, value in calibration_kwargs(config).items():
        target, field_name = protocol_mod._SET_KEY_TARGETS[key]
        ack = proto.set_config_field(target, field_name, value)
        (applied if ack is not None else rejected).append(key)

    for field_name, value in otos_kwargs(config).items():
        ack = proto.set_config_field(robot_config_pb2.OTOS, field_name, value)
        (applied if ack is not None else rejected).append(f"otos.{field_name}")

    return {
        "status": "ok" if not rejected else "partial",
        "applied": applied,
        "rejected": rejected,
    }


def _push_via_conn(conn: Any, config: Any) -> "dict[str, Any]":
    """Send ``calibration_commands(config)`` over *conn* (SerialConnection)
    as plain text lines.

    Returns a dict with ``"status": "ok"`` and ``"commands"`` listing sent verbs.
    """
    sent: "list[str]" = []
    for cmd, read_timeout in calibration_commands(config):
        conn.send(cmd, read_timeout=read_timeout)
        sent.append(cmd)
    return {"status": "ok", "commands": sent}
