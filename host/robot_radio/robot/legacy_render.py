"""legacy_render.py -- M5 rogo Translator Proxy (097-004), reverse direction.

The other half of the translator: binary ``pb2`` replies -> text-v2 reply
lines. Pure, stateless functions -- no ``SerialConnection``/socket/PTY
reference anywhere in this module, matching ``legacy_verbs.py``'s own
"pure" posture. Every renderer is transcribed from the firmware's own
``snprintf`` format strings, cited by file:line against the pre-097-006/
007/008 sources (this sprint's earlier tickets gutted the TEXT HANDLERS
that owned these format strings -- ``motion_commands.cpp``/
``system_commands.cpp`` (097-006), ``config_commands.cpp`` (097-007),
``telemetry_commands.cpp``/``tlm_frame.cpp`` (097-008) -- but the wire
SHAPE those handlers produced is exactly what a legacy text client still
expects, so this module reconstructs it host-side from the equivalent
binary reply fields). Citations below reference the pre-gut commit each
handler was deleted in (``git show <gut-commit>^:<path>``), not a file
that exists in the working tree today.

Two format-string primitives every renderer below is built from, both
transcribed from ``CommandProcessor::replyOK()``/``replyErr()``
(``source/commands/command_processor.cpp`` -- NOT gutted, still the
generic reply-line assembler every retained command uses): 4 spacing
variants each, selected by whether a body/detail and a corr-id are
present.
"""

from __future__ import annotations

import math

from robot_radio.robot.pb2 import envelope_pb2, planner_pb2, telemetry_pb2

# ---------------------------------------------------------------------------
# Generic OK/ERR line assembly -- CommandProcessor::replyOK()/replyErr()
# mirror (command_processor.cpp:284-322, pre-gut motion_commands.cpp etc.
# citations below all funnel through these two).
# ---------------------------------------------------------------------------


def render_ok(verb: str, body: str | None, corr_id: int | str | None) -> str:
    """``CommandProcessor::replyOK()`` mirror: ``"OK %s %s #%s"`` /
    ``"OK %s %s"`` / ``"OK %s #%s"`` / ``"OK %s"``, selected by whether
    ``body``/``corr_id`` are present (falsy == absent, matching the
    firmware's own ``body && body[0] != '\\0'`` / ``id && id[0] != '\\0'``
    guards)."""
    if body:
        return f"OK {verb} {body} #{corr_id}" if corr_id else f"OK {verb} {body}"
    return f"OK {verb} #{corr_id}" if corr_id else f"OK {verb}"


def render_err(code: str, detail: str | None, corr_id: int | str | None) -> str:
    """``CommandProcessor::replyErr()`` mirror: same 4-variant shape as
    ``render_ok()`` above, with ``code``/``detail`` in place of
    ``verb``/``body``."""
    if detail:
        return f"ERR {code} {detail} #{corr_id}" if corr_id else f"ERR {code} {detail}"
    return f"ERR {code} #{corr_id}" if corr_id else f"ERR {code}"


# ---------------------------------------------------------------------------
# ERR code mapping -- binary Error{code, field} -> text (code, detail).
# Per the implementation spec (clasi/issues/rogo-translator-proxy-text-v2-
# binary-bridge-on-a-pty.md): mirrors the text plane's existing
# unknown/badarg/range/full codes, plus binary-only DECODE/UNIMPLEMENTED/
# OVERSIZE folded onto the closest text-plane equivalent (no text-plane
# code ever existed for "malformed wire bytes" or "arm not wired yet").
# ---------------------------------------------------------------------------

ERR_CODE_TEXT: dict[int, str] = {
    envelope_pb2.ERR_NONE: "none",
    envelope_pb2.ERR_UNKNOWN: "unknown",
    envelope_pb2.ERR_BADARG: "badarg",
    envelope_pb2.ERR_RANGE: "range",
    envelope_pb2.ERR_FULL: "full",
    envelope_pb2.ERR_DECODE: "badarg",
    envelope_pb2.ERR_UNIMPLEMENTED: "unsupported",
    envelope_pb2.ERR_OVERSIZE: "unsupported",
}


def field_name_for_error(field_number: int) -> str | None:
    """Map an ``Error.field`` number back to its ``CommandEnvelope`` oneof
    field NAME via the schema itself (``CommandEnvelope.DESCRIPTOR.
    fields_by_number``) -- per the implementation spec's own instruction,
    not a hand-maintained duplicate table. ``0`` (not field-specific, e.g.
    ``ERR_UNKNOWN``/``ERR_DECODE``) returns ``None``."""
    if not field_number:
        return None
    fd = envelope_pb2.CommandEnvelope.DESCRIPTOR.fields_by_number.get(field_number)
    return fd.name if fd else None


def render_error(error: envelope_pb2.Error, corr_id: int | str | None) -> str:
    """Render a binary ``Error`` reply as ``"ERR <code> [<field-name>]
    [#<id>]"``."""
    code = ERR_CODE_TEXT.get(error.code, "unknown")
    detail = field_name_for_error(error.field)
    return render_err(code, detail, corr_id)


# ---------------------------------------------------------------------------
# Liveness/identity -- system_commands.cpp (pre-097-006; gut commit
# 18ba84d8). handleId()/handleVer()/formatDeviceAnnouncement().
# ---------------------------------------------------------------------------


def render_id_line(device_id: "envelope_pb2.DeviceId", corr_id: int | str | None) -> str:
    """``handleId()`` mirror: bare ``"ID model=NEZHA2 name=%s serial=%lu
    fw=%s proto=%d [#%s]"`` -- NOT wrapped in ``OK``; ``ID`` is its own
    reply taxonomy, like ``DEVICE:``."""
    base = (f"ID model=NEZHA2 name={device_id.name} serial={device_id.serial} "
            f"fw={device_id.fw_version} proto={device_id.proto_version}")
    return f"{base} #{corr_id}" if corr_id else base


def render_ver_body(device_id: "envelope_pb2.DeviceId") -> str:
    """``handleVer()`` mirror: ``"fw=%s proto=%d"`` -- the BODY only; wrap
    with ``render_ok("ver", body, corr_id)`` for the full line (VER has no
    independent binary arm -- its content is a strict subset of ID's
    reply, see ``envelope_for_id()``, legacy_verbs.py)."""
    return f"fw={device_id.fw_version} proto={device_id.proto_version}"


def render_device_banner(device_id: "envelope_pb2.DeviceId") -> str:
    """``formatDeviceAnnouncement()`` mirror: ``"DEVICE:NEZHA2:robot:%s:%lu"``
    -- the SAME banner ``handleHello()`` and the firmware's own boot
    announcement both emit. The proxy answers ``HELLO`` LOCALLY from a
    startup-cached ``DeviceId`` (see io/proxy.py) -- upstream passthrough
    can't work, ``SerialConnection``'s reader drops ``DEVICE:`` lines."""
    return f"DEVICE:NEZHA2:robot:{device_id.name}:{device_id.serial}"


# ---------------------------------------------------------------------------
# Motion verbs -- motion_commands.cpp (pre-097-006; gut commit 18ba84d8).
# Each handler's body construction used the ORIGINAL request's own
# (unconverted) token values, not a round trip through the binary Ack --
# transcribed the same way here: these renderers take the CLIENT's own
# positional/kv tokens (the same ``pos``/``kv`` ``legacy_verbs`` parsed to
# build the request envelope) plus the reply ``Ack``, exactly mirroring
# each handler's own mix of "echo the request" + "report q=/rem=/t= from
# runtime state".
# ---------------------------------------------------------------------------


def render_ok_for_verb(verb: str, pos: list[str], kv: dict[str, str],
                       ack: "envelope_pb2.Ack", corr_id: int | str | None) -> str:
    """Render the ``OK`` reply line for a verb whose binary reply is a
    plain ``Ack{q, rem, t}`` -- ``S``/``T``/``D``/``RT``/``SEG``/``R``/
    ``TURN``/``G``/``MOVE``/``MOVER``/``PING``/``STOP`` (``R``/``TURN``/
    ``G`` added 097; ``SEG`` added 100-007, THE CUTOVER -- see
    ``legacy_translate.py``'s ``segment_for_arc()``/``segment_for_turn()``/
    ``segment_for_goto_relative()``/``segment_for_seg()`` for the
    translations these render the reply for). Raises ``ValueError`` for any
    other verb (no Ack-shaped reply defined for it)."""
    if verb == "S":
        # handleS(): "OK drive l=%d r=%d"
        return render_ok("drive", f"l={int(float(pos[0]))} r={int(float(pos[1]))}", corr_id)
    if verb == "T":
        # handleT(): "OK drive l=%d r=%d ms=%d"
        body = f"l={int(float(pos[0]))} r={int(float(pos[1]))} ms={int(float(pos[2]))}"
        return render_ok("drive", body, corr_id)
    if verb == "D":
        # handleD(): "OK drive l=%d r=%d mm=%d"
        body = f"l={int(float(pos[0]))} r={int(float(pos[1]))} mm={int(float(pos[2]))}"
        return render_ok("drive", body, corr_id)
    if verb == "RT":
        # handleRT(): "OK rt rot=%d"
        return render_ok("rt", f"rot={int(float(pos[0]))}", corr_id)
    if verb == "SEG":
        # segment_for_seg() (100-007, THE CUTOVER): "OK seg arc=%d dh=%d
        # exit=%d q=%u rem=%d" -- q/rem come from the Ack, exit from the
        # RAW kv value (no live handler to derive it from, same posture as
        # TURN's eps/MOVER's t/v/w below).
        from robot_radio.robot.legacy_verbs import kvfloat
        exit_speed = int(kvfloat(kv, "exit_speed"))
        body = (f"arc={int(float(pos[0]))} dh={int(float(pos[1]))} "
                f"exit={exit_speed} q={ack.q} rem={int(ack.rem)}")
        return render_ok("seg", body, corr_id)
    if verb == "R":
        # handleR(): "OK arc speed=%d radius=%d"
        body = f"speed={int(float(pos[0]))} radius={int(float(pos[1]))}"
        return render_ok("arc", body, corr_id)
    if verb == "TURN":
        # handleTURN(): "OK turn heading=%d eps=%d"
        from robot_radio.robot.legacy_verbs import kvfloat
        eps = int(kvfloat(kv, "eps"))
        return render_ok("turn", f"heading={int(float(pos[0]))} eps={eps}", corr_id)
    if verb == "G":
        # handleG(): "OK goto x=%d y=%d speed=%d"
        body = f"x={int(float(pos[0]))} y={int(float(pos[1]))} speed={int(float(pos[2]))}"
        return render_ok("goto", body, corr_id)
    if verb == "MOVE":
        # handleMove(): "OK move dist=%d dir=%d fh=%d q=%u rem=%d"
        # -- q/rem come from the Ack (runtime queue depth / remaining
        # translation), NOT re-derived from the request.
        body = (f"dist={int(float(pos[0]))} dir={int(float(pos[1]))} "
                f"fh={int(float(pos[2]))} q={ack.q} rem={int(ack.rem)}")
        return render_ok("move", body, corr_id)
    if verb == "MOVER":
        # handleMover(): "OK mover t=%d v=%d w=%d q=%u" -- t/v/w are the
        # RAW kv values (no unit conversion in the text handler either).
        from robot_radio.robot.legacy_verbs import kvfloat
        t = int(kvfloat(kv, "t"))
        v = int(kvfloat(kv, "v"))
        w = int(kvfloat(kv, "w"))
        body = f"t={t} v={v} w={w} q={ack.q}"
        return render_ok("mover", body, corr_id)
    if verb == "PING":
        # handlePing(): "OK pong t=%lu"
        return render_ok("pong", f"t={ack.t}", corr_id)
    if verb == "STOP":
        # handleStop(): "OK stop" (no body)
        return render_ok("stop", None, corr_id)
    raise ValueError(f"no Ack-shaped OK renderer for verb {verb}")


# Verbs whose Ack success arms the EVT-done watch (T/D/RT/SEG/MOVE only --
# S/MOVER/PING/STOP do not; SEG added 100-007, THE CUTOVER, alongside its
# T/D/RT/MOVE segment-arm siblings; see io/proxy.py's _EvtWatcher wiring).
EVT_ARMING_VERBS = frozenset({"T", "D", "RT", "SEG", "MOVE"})


# ---------------------------------------------------------------------------
# EVT synthesis -- CommandProcessor::emitEvent() GOAL_DONE-branch mirror
# (command_processor.cpp, NOT gutted -- still the one place "EVT ..." wire
# text is assembled, even though nothing calls it today; see io/proxy.py's
# _EvtWatcher docstring for why the PROXY, not the firmware, is the one
# producing these now).
# ---------------------------------------------------------------------------


def render_evt_done(verb: str, corr_id: int | str | None, reason: str = "idle") -> str:
    """``"EVT done <VERB> [#<id>] reason=<reason>"``."""
    tail = f"#{corr_id} reason={reason}" if corr_id else f"reason={reason}"
    return f"EVT done {verb} {tail}"


# ---------------------------------------------------------------------------
# Telemetry -- tlm_frame.cpp's buildTlmFrame() (pre-097-008; gut commit
# bce277c7) for the periodic STREAM/SNAP line; motion_commands.cpp's
# handleTlm() (pre-097-006) for the one-shot `TLM` verb's bench body.
# ---------------------------------------------------------------------------

# kAngleScale mirror (tlm_frame.cpp): radians -> centidegrees.
_ANGLE_SCALE = 5729.5779513  # [cdeg/rad]

# modeChar() mirror (tlm_frame.cpp): msg::DriveMode -> single wire char.
# VELOCITY has no dedicated character (falls to the 'I' default, same as
# protocol.py's own _DRIVE_MODE_CHAR.get() fallback).
_MODE_CHAR = {
    planner_pb2.IDLE: "I",
    planner_pb2.STREAMING: "S",
    planner_pb2.TIMED: "T",
    planner_pb2.DISTANCE: "D",
    planner_pb2.GO_TO: "G",
}


def render_tlm_line(telemetry: "telemetry_pb2.Telemetry") -> str:
    """``buildTlmFrame()`` mirror: the periodic STREAM/SNAP wire line --
    ``"TLM t=%lu mode=%c seq=%u"`` plus each conditionally-present field
    group in the SAME order ``buildTlmFrame()`` appends them:
    enc/vel/cmd/pose/otos(+otosconn)/twist. ``encpose=`` is never emitted
    -- ``telemetry.proto`` does not carry it (096-001's trim; a documented,
    permanent gap, not an oversight here). Every integer field uses
    Python's ``int()`` truncate-toward-zero, matching C++'s
    ``static_cast<int>`` on a float exactly (both truncate toward zero, not
    floor)."""
    parts = [f"TLM t={int(telemetry.now)} mode={_MODE_CHAR.get(telemetry.mode, 'I')} "
             f"seq={int(telemetry.seq)}"]
    if telemetry.has_enc:
        parts.append(f"enc={int(telemetry.enc_left)},{int(telemetry.enc_right)}")
    if telemetry.has_vel:
        parts.append(f"vel={int(telemetry.vel_left)},{int(telemetry.vel_right)}")
    if telemetry.has_cmd_vel:
        parts.append(f"cmd={int(telemetry.cmd_vel_left)},{int(telemetry.cmd_vel_right)}")
    if telemetry.has_pose:
        parts.append(f"pose={int(telemetry.pose.x)},{int(telemetry.pose.y)},"
                     f"{int(telemetry.pose.h * _ANGLE_SCALE)}")
    if telemetry.has_otos:
        parts.append(f"otos={int(telemetry.otos.x)},{int(telemetry.otos.y)},"
                     f"{int(telemetry.otos.h * _ANGLE_SCALE)}")
        parts.append(f"otosconn={1 if telemetry.otos_connected else 0}")
    if telemetry.has_twist:
        parts.append(f"twist={int(telemetry.twist.v_x)},{int(telemetry.twist.omega * 1000.0)}")
    return " ".join(parts)


def _lround(value: float) -> int:
    """``lroundf()`` mirror: round HALF AWAY FROM ZERO -- NOT Python's
    built-in ``round()``, which rounds half-to-even (banker's rounding) and
    disagrees with C's ``lroundf()`` on exact ``.5`` boundaries."""
    return math.floor(value + 0.5) if value >= 0 else -math.floor(-value + 0.5)


def _format_tenths(value: float) -> str:
    """``formatTenths()`` mirror (``handleTlm()``'s own lambda,
    motion_commands.cpp): one-decimal fixed-point via integer math only --
    the firmware's newlib-nano ``snprintf`` has no float-conversion support
    (``%f`` silently emits nothing on-hardware), so the ORIGINAL handler
    never used it either; reproduced here for wire-format fidelity, not
    because Python has the same limitation."""
    t = _lround(value * 10.0)
    sign = "-" if t < 0 else ""
    t = abs(t)
    return f"{sign}{t // 10}.{t % 10}"


def render_tlm_one_shot_body(telemetry: "telemetry_pb2.Telemetry") -> str:
    """``handleTlm()`` mirror: the one-shot ``TLM`` verb's bench-diagnostic
    body -- ``"enc=%s,%s vel=%s,%s cmd=%d,%d acc=%d,%d active=%d conn=%d,%d
    glitch=%u,%u ts=%u,%u now=%u"``. Binary ``Telemetry`` carries every one
    of these fields UNCONDITIONALLY (the bench-diagnostic block,
    telemetry.proto fields 20-28) -- no ``has_*`` gating needed, matching
    ``handleTlm()``'s own "never omits them" posture. Wrap with
    ``render_ok("tlm", body, corr_id)`` for the full line."""
    enc_l = _format_tenths(telemetry.enc_left if telemetry.has_enc else 0.0)
    enc_r = _format_tenths(telemetry.enc_right if telemetry.has_enc else 0.0)
    vel_l = _format_tenths(telemetry.vel_left if telemetry.has_vel else 0.0)
    vel_r = _format_tenths(telemetry.vel_right if telemetry.has_vel else 0.0)
    cmd_l = int(telemetry.cmd_vel_left) if telemetry.has_cmd_vel else 0
    cmd_r = int(telemetry.cmd_vel_right) if telemetry.has_cmd_vel else 0
    return (f"enc={enc_l},{enc_r} vel={vel_l},{vel_r} cmd={cmd_l},{cmd_r} "
            f"acc={int(telemetry.acc_left)},{int(telemetry.acc_right)} "
            f"active={1 if telemetry.active else 0} "
            f"conn={1 if telemetry.conn_left else 0},{1 if telemetry.conn_right else 0} "
            f"glitch={int(telemetry.glitch_left)},{int(telemetry.glitch_right)} "
            f"ts={int(telemetry.ts_left)},{int(telemetry.ts_right)} "
            f"now={int(telemetry.now)}")


# ---------------------------------------------------------------------------
# Config -- config_commands.cpp (pre-097-007; gut commit a61ffb6e).
# formatConfigKeyFromBb()/kAllKeys/formatFixed().
# ---------------------------------------------------------------------------

# kAllKeys mirror (config_commands.cpp) -- the complete, ordered key list
# GET's "dump all" path uses. SAME order protocol.py's own _ALL_GET_KEYS
# already uses (097-002); duplicated here (not imported) so this module
# stays free of a protocol.py dependency -- both copies cite the same
# firmware source and must be kept in sync if kAllKeys ever changes.
ALL_GET_KEYS = (
    "tw", "ml", "mr",
    "pid.kp", "pid.ki", "pid.kff", "pid.iMax", "pid.kaw",
    "rotSlip",
    "ekfQxy", "ekfQtheta", "ekfROtosXy", "ekfROtosTheta",
    "minSpeed", "sTimeout",
)

# Per-key format kind, transcribed from formatConfigKeyFromBb()'s own
# per-key branch: "int" -> static_cast<int>(v) (truncate toward zero,
# no decimals); "fixed3" -> formatFixed(v, 3); "uint" -> unsigned, no
# decimals (sTimeout only -- the one non-float registered key).
_KEY_FORMAT_KIND = {
    "tw": "int", "ml": "fixed3", "mr": "fixed3",
    "pid.kp": "fixed3", "pid.ki": "fixed3", "pid.kff": "fixed3",
    "pid.iMax": "fixed3", "pid.kaw": "fixed3",
    "rotSlip": "fixed3",
    "ekfQxy": "fixed3", "ekfQtheta": "fixed3",
    "ekfROtosXy": "fixed3", "ekfROtosTheta": "fixed3",
    "minSpeed": "int", "sTimeout": "uint",
}


def _format_fixed(value: float, decimals: int = 3) -> str:
    """``formatFixed()`` mirror (config_commands.cpp): fixed-point decimal
    via integer math, round-half-away-from-zero (``lroundf``)."""
    negative = value < 0.0
    scale = 10 ** decimals
    scaled = _lround(abs(value) * scale)
    int_part, frac_part = divmod(scaled, scale)
    sign = "-" if negative else ""
    if decimals <= 0:
        return f"{sign}{int_part}"
    return f"{sign}{int_part}.{frac_part:0{decimals}d}"


def format_config_value(key: str, value: float) -> str:
    """Format one registered config key's RAW numeric value into its exact
    firmware wire representation (``formatConfigKeyFromBb()`` mirror).
    Unrecognized keys default to ``fixed3`` (defensive; every key this is
    ever called with is already validated against ``ALL_GET_KEYS``/
    ``protocol.py``'s ``_TARGET_FOR_KEY`` upstream)."""
    kind = _KEY_FORMAT_KIND.get(key, "fixed3")
    if kind in ("int", "uint"):
        return str(int(value))
    return _format_fixed(value, 3)


def render_cfg_line(values: dict[str, float], corr_id: int | str | None,
                    keys: tuple[str, ...] | None = None) -> str:
    """``handleGet()`` mirror: ``"CFG k=v k=v ... [#id]"``, keys in
    ``ALL_GET_KEYS`` order (or the caller-supplied ``keys`` order, for a
    targeted ``GET <k1> <k2>``). ``values`` holds RAW numeric values (not
    pre-formatted strings) -- this function owns the firmware-exact
    formatting via ``format_config_value()``. A key absent from ``values``
    (its target's round trip timed out) is silently omitted, matching
    ``handleGet()``'s own "skip an unrecognized/unavailable key" behavior."""
    order = keys if keys is not None else ALL_GET_KEYS
    parts = [f"{k}={format_config_value(k, values[k])}" for k in order if k in values]
    line = "CFG " + " ".join(parts) if parts else "CFG"
    return f"{line} #{corr_id}" if corr_id else line
