"""robot_radio.testgui.binary_bridge — in-process text-v2 -> binary
translation for the TestGUI transports (097, min-viable migration).

Firmware is binary-only plus a 6-verb text rump (HELP/HELLO/PING/ID/VER/
STOP, docs/protocol-v3.md section 6) — every motion/config/telemetry text
verb the TestGUI's ``commands.COMMANDS`` schema or its Operations panel
still builds (``S``/``T``/``D``/``RT``/``SET``/``GET``/``STREAM``/``SNAP``/
...) gets ``ERR unknown`` if sent as literal text. This module is the ONE
place both transports (``transport.py``'s ``_HardwareTransport`` and
``SimTransport``) route every outbound command line through so a text line
never reaches the wire un-translated.

It is the SAME routing table ``io/proxy.py``'s ``ProtocolBridge.
_handle_client_line`` already implements for the ``rogo proxy`` PTY bridge
— reused here (not re-derived): ``legacy_verbs.tokenize_send_line()``/
``BINARY_DISPATCH`` for the tokenizer + one-arm verb builders,
``legacy_render`` for reply-line formatting, ``protocol.py``'s
``NezhaProtocol``/config key-target maps for SET/GET/STREAM. Trimmed to
what the TestGUI needs and stripped of the PTY/EVT-watcher machinery that
module owns (the GUI has its own tour idle-detection via live telemetry,
not a synthesized ``EVT``).

``translate_command(proto, raw_line)`` is the single entry point. For a
verb with no binary arm at all — the legacy pose/otos family (SI/ZERO/OZ/
OI/OR/OP/OV/OL/OA — envelope.proto's ``pose``/``otos`` oneof arms are
declared-only until 098), GRIP/QLEN (never had a binary arm), and the
legacy ``DEV`` debug family (retired with the rest of the text plane, no
binary arm was ever planned for it) — it returns a typed ``ERR ...`` reply
line WITHOUT sending anything on the wire. The OTOS-chip hardware verbs
(OI/OL/OA/OV/OP/OR) render with the ``nodev`` code specifically (not
``unsupported``): this preserves ``calibration_commands()``'s existing
NODEV-tolerant push loop (``__main__.py``'s ``_push_robot_calibration`` /
``push.py``'s own docstring) — those three verbs already meant "no
physical OTOS chip attached" in Sim mode on the old text plane, and reads
the same way today for the same physical reason (096/098 have not wired a
real detection path either way).

R/TURN/G (097, this ticket): UN-GATED — each now translates to an
open-loop ``segment``/``replace`` envelope via ``legacy_translate.
segment_for_arc()``/``segment_for_turn()``/``segment_for_goto_relative()``
(``legacy_verbs.envelope_for_r/turn/g``). None of the three need the
Planner motion oneof arm (still reserved, not declared, per envelope.proto
field 5's own comment) — they are approximations of their original
closed-loop/continuous firmware behavior, each documented at its
translator function. The genuinely fused-pose/OTOS-chip/camera-dependent
verbs above stay gated.

107-003 launch-unblock: ``legacy_render``/``legacy_verbs`` were deleted
wholesale by commit ``129cbcb3`` (104-002) with no replacement, and were
never re-pointed here — every name below was already dead-verb residue
(see ``clasi/issues/binary-bridge-segment-replace-arms-deleted.md``), but
the unconditional module-level import made this an ``ImportError`` that
prevented ``transport.py`` — and therefore the whole TestGUI — from
importing at all, blocking sprint 107's own tour path along with
everything else (docs/issue updated to record this). Both imports are now
guarded: when unavailable, ``translate_command()`` short-circuits to a
single explicit ``ERR`` line (``_LEGACY_UNAVAILABLE_REPLY``) for every
verb, and ``render_log_line()`` falls back to ``google.protobuf.
text_format`` rendering (already its fallback for reply kinds
``legacy_render`` never covered). This is a MINIMAL launch-unblock only —
it does not restore legacy verb translation or rewrite this module's
segment/replace logic; that remains the filed issue's own, separate scope.
"""

from __future__ import annotations

from typing import Any

from robot_radio.robot import protocol
from robot_radio.robot.pb2 import envelope_pb2
from robot_radio.robot.protocol import NezhaProtocol

try:
    from robot_radio.robot import legacy_render as render
    from robot_radio.robot import legacy_verbs
    _LEGACY_TRANSLATION_AVAILABLE = True
except ImportError:
    render = None  # type: ignore[assignment]
    legacy_verbs = None  # type: ignore[assignment]
    _LEGACY_TRANSLATION_AVAILABLE = False

# Fixed reply for every verb when legacy_render/legacy_verbs are missing
# (see this module's own docstring, "107-003 launch-unblock"). Deliberately
# NOT built via `render.render_err()` -- render itself may be the thing
# that's unavailable.
_LEGACY_UNAVAILABLE_REPLY = (
    "ERR unavailable legacy verb translation removed -- see "
    "clasi/issues/binary-bridge-segment-replace-arms-deleted.md"
)

# kStreamFloorMs mirror (source/commands/telemetry_commands.cpp /
# binary_channel.cpp) — same floor protocol.py's own NezhaProtocol.stream()/
# snap() and io/proxy.py's ProtocolBridge use.
_STREAM_FLOOR_MS = 20  # [ms]

# Verbs with NO binary arm at all, ever. GRIP/QLEN never had any binary arm
# planned. (097, this ticket: G/R/TURN REMOVED from this set -- they now
# translate to open-loop segment/replace envelopes; see this module's own
# docstring and legacy_translate.py's segment_for_arc()/segment_for_turn()/
# segment_for_goto_relative(). io/proxy.py's OWN, separate
# _ALWAYS_UNSUPPORTED_VERBS copy for the rogo proxy PTY bridge is untouched
# -- out of this ticket's scope.)
_ALWAYS_UNSUPPORTED_VERBS = frozenset({"QLEN", "GRIP"})

# Pose-reset verbs: envelope.proto's `pose`/`otos` oneof arms exist
# (SetPose/OdometerCommand) but are declared-only -- BinaryChannel replies
# ERR_UNIMPLEMENTED for them until sprint 098. SI/ZERO/OZ specifically
# reset SOFTWARE pose state (fused EKF pose / encoder counters / the OTOS
# chip's own zero reference) -- not "is a physical device attached"
# questions, so these render generic "unsupported", unlike the OTOS-chip
# verbs below.
_POSE_RESET_VERBS = frozenset({"SI", "ZERO", "OZ"})

# OTOS-chip hardware verbs: OI (init)/OL (linear scalar)/OA (angular
# scalar)/OV (raw position write)/OP (position query)/OR (Kalman reset).
# Same "declared-only until 098" status as the pose-reset verbs above, but
# rendered as "nodev" -- see this module's own docstring for why
# (calibration_commands() NODEV-tolerance).
_OTOS_DEVICE_VERBS = frozenset({"OI", "OL", "OA", "OV", "OP", "OR"})

_UNSUPPORTED_REASON = "requires sprint 098 (no binary arm yet)"


def translate_command(proto: NezhaProtocol, raw_line: str) -> str:
    """Translate one text-v2 line to binary, send it, and return the
    rendered text-v2 reply line.

    Empty/whitespace-only input returns ``""`` (no verb to dispatch, no
    wire traffic) -- mirrors ``ProtocolBridge._handle_client_line``'s own
    "not stripped, no verb -> None" short-circuit, translated to this
    module's "always return a string" contract.

    107-003 launch-unblock: if ``legacy_render``/``legacy_verbs`` are not
    importable (see this module's own docstring), every non-empty line
    short-circuits to ``_LEGACY_UNAVAILABLE_REPLY`` instead of dispatching
    -- parsing the line at all is itself `legacy_verbs`' job, so there is
    no partial/degraded dispatch to fall back to.
    """
    if not _LEGACY_TRANSLATION_AVAILABLE:
        return _LEGACY_UNAVAILABLE_REPLY if raw_line.strip() else ""

    stripped, corr_id_str = legacy_verbs.split_corr_id(raw_line)
    corr_id = int(corr_id_str) if corr_id_str else None
    verb, pos, kv = legacy_verbs.tokenize_send_line(stripped)
    if not verb:
        return ""

    if verb in _OTOS_DEVICE_VERBS:
        return render.render_err("nodev", f"{verb} {_UNSUPPORTED_REASON}", corr_id)
    if verb in _POSE_RESET_VERBS or verb in _ALWAYS_UNSUPPORTED_VERBS or verb.startswith("DEV"):
        return render.render_err("unsupported", f"{verb} {_UNSUPPORTED_REASON}", corr_id)

    if verb == "SET":
        return _handle_set(proto, kv, corr_id)
    if verb == "GET":
        return _handle_get(proto, pos, corr_id)
    if verb == "STREAM":
        return _handle_stream(proto, pos, corr_id)
    if verb == "SNAP":
        return _handle_snap(proto, corr_id)

    if verb in legacy_verbs.BINARY_DISPATCH:
        return _handle_binary_verb(proto, verb, pos, kv, corr_id)

    # Unknown verb (P/PA/DBG/anything else the text plane used to answer) --
    # never forwarded as raw text (the firmware would just ERR unknown on
    # it anyway); a typed reply here at least tells the caller why.
    return render.render_err("unsupported", verb, corr_id)


# ---------------------------------------------------------------------------
# One-arm binary verbs (S/D/T/RT/MOVE/MOVER/ECHO/PING/STOP/ID/HELLO/VER/HELP)
# ---------------------------------------------------------------------------


def _handle_binary_verb(proto: NezhaProtocol, verb: str, pos: list[str],
                        kv: dict[str, str], corr_id: int | None) -> str:
    try:
        envs = legacy_verbs.BINARY_DISPATCH[verb](pos, kv)
    except ValueError as exc:
        return render.render_err("badarg", str(exc), corr_id)

    # (100-007, THE CUTOVER) envs may hold up to 3 envelopes (MOVE's
    # <=3-primitive decomposition) or 2 (G's) -- send every one IN ORDER,
    # stopping at the first rejected/timed-out reply. NezhaProtocol.
    # _send_envelope() normalizes SerialConnection's (dict-wrapped) vs.
    # SimConnection's (bare ReplyEnvelope) different send_envelope() return
    # shapes -- see that method's own docstring.
    reply = None
    for env in envs:
        reply = proto._send_envelope(env, read_timeout=500)
        if reply is None:
            return render.render_err("unknown", "timeout", corr_id)
        if reply.WhichOneof("body") == "err":
            return render.render_error(reply.err, corr_id)

    if reply is None:
        return render.render_ok_for_verb(verb, pos, kv, envelope_pb2.Ack(), corr_id)

    which = reply.WhichOneof("body")
    if which == "echo":
        return render.render_ok("echo", reply.echo.payload.decode("utf-8", "replace"), corr_id)
    if which == "id":
        if verb == "VER":
            return render.render_ok("ver", render.render_ver_body(reply.id), corr_id)
        return render.render_id_line(reply.id, corr_id)
    if which == "helptext":
        return render.render_ok("help", reply.helptext.text, corr_id)
    if which == "ok":
        return render.render_ok_for_verb(verb, pos, kv, reply.ok, corr_id)
    return render.render_err("unknown", None, corr_id)


# ---------------------------------------------------------------------------
# Config (SET/GET) -- reuses NezhaProtocol/protocol.py's own key-target maps
# rather than reimplementing the fan-out (mirrors io/proxy.py's
# ProtocolBridge._handle_set/_handle_get exactly).
# ---------------------------------------------------------------------------


def _handle_set(proto: NezhaProtocol, kv: dict[str, str], corr_id: int | None) -> str:
    if not kv:
        return render.render_err("badarg", "no key=value pairs", corr_id)
    bad = [k for k in kv if k not in protocol._ALL_SET_KEYS]
    if bad:
        return render.render_err("badkey", bad[0], corr_id)
    try:
        kwargs = {k: float(v) for k, v in kv.items()}
    except ValueError:
        return render.render_err("badarg", "bad value", corr_id)
    applied = proto.set_config(**kwargs)
    if applied is None:
        return render.render_err("badarg", "set failed", corr_id)
    body = " ".join(f"{k}={v}" for k, v in applied.items())
    return render.render_ok("set", body, corr_id)


def _handle_get(proto: NezhaProtocol, pos: list[str], corr_id: int | None) -> str:
    requested = tuple(pos) if pos else render.ALL_GET_KEYS
    bad = [k for k in requested if k not in protocol._TARGET_FOR_KEY]
    if bad:
        return render.render_err("badkey", bad[0], corr_id)

    from robot_radio.io.proxy import _raw_config_snapshot_value

    targets = sorted({protocol._TARGET_FOR_KEY[k] for k in requested})
    snapshots: dict[int, Any] = {}
    for target in targets:
        snapshot = proto.get_config_binary(target)
        if snapshot is not None:
            snapshots[target] = snapshot

    values: dict[str, float] = {}
    for key in requested:
        snapshot = snapshots.get(protocol._TARGET_FOR_KEY[key])
        if snapshot is None:
            continue
        raw = _raw_config_snapshot_value(key, snapshot)
        if raw is not None:
            values[key] = raw
    return render.render_cfg_line(values, corr_id, keys=requested)


# ---------------------------------------------------------------------------
# Telemetry (STREAM/SNAP)
# ---------------------------------------------------------------------------


def _handle_stream(proto: NezhaProtocol, pos: list[str], corr_id: int | None) -> str:
    """``STREAM <period>`` -> ``StreamControl{binary: true, period}``.

    This is the ONE place ``STREAM 50``/``STREAM 0`` (connect-time arm in
    ``__main__.py``, the Operations panel's STREAM toggle, and
    ``SimTransport``'s own tick-thread startup) turns into a binary
    envelope -- no call site needs to build the envelope itself.
    """
    if not pos:
        return render.render_err("badarg", "period", corr_id)
    try:
        requested = int(float(pos[0]))
    except ValueError:
        return render.render_err("badarg", "period", corr_id)
    period = 0 if requested <= 0 else max(_STREAM_FLOOR_MS, requested)
    proto.stream(period)
    return render.render_ok("stream", f"period={period}", corr_id)


def _handle_snap(proto: NezhaProtocol, corr_id: int | None) -> str:
    """One-shot ``SNAP`` -- arm-wait-disarm synthesis (NezhaProtocol.snap()'s
    own strategy, architecture-update.md (097) Decision 4), reimplemented
    here instead of called directly because it needs the RAW
    ``telemetry_pb2.Telemetry`` (for ``legacy_render.render_tlm_line()``),
    not ``snap()``'s own parsed ``TLMFrame``.

    The only caller today is ``_TourRunner._wait_for_idle``'s fire-and-forget
    ``send("SNAP")`` nudge (``__main__.py``) -- implemented anyway so a
    stray/future ``SNAP`` never silently falls through to the catch-all
    "unsupported" reply.
    """
    import time as _time

    conn = proto._conn
    conn.drain_binary_tlm()
    proto.stream(_STREAM_FLOOR_MS)
    read_binary_tlm = getattr(conn, "read_binary_tlm", None)
    if read_binary_tlm is not None:
        frames = read_binary_tlm(duration=400)
    else:
        # SimConnection path (no read_binary_tlm): frames only exist after
        # whoever ticks the sim has ticked (SimTransport's tick-thread when
        # the GUI is connected), so a single non-blocking drain immediately
        # after arming RACES that thread -- which drains each frame into the
        # trace pipeline first -- and lost every time: the recurring
        # "ERR unknown snap-timeout" console noise. Poll briefly instead,
        # matching read_binary_tlm()'s own 400ms window.
        frames = conn.drain_binary_tlm()
        deadline = _time.monotonic() + 0.4
        while not frames and _time.monotonic() < deadline:
            _time.sleep(0.02)
            frames = conn.drain_binary_tlm()
    if not frames:
        return render.render_err("unknown", "snap-timeout", corr_id)
    return render.render_tlm_line(frames[0].tlm)


# ---------------------------------------------------------------------------
# Serial/message monitor filtering (097, Goal 4) -- translates every raw
# `*B<base64>` wire log line SerialConnection's on_send/on_recv hooks
# deliver (see io/serial_conn.py's `_reader_loop`/`send_envelope`
# docstrings: on_recv fires for EVERY decoded line before any
# classification, including the high-rate binary telemetry push stream)
# into readable text for the TestGUI's log pane, instead of an opaque
# base64 blob a human cannot read. `_HardwareTransport`'s `_on_send`/
# `_on_recv` closures (transport.py) are the only callers -- SimTransport
# never needs this: its own send()/command()/_drain_cmd_queue() already log
# translate_command()'s human-readable return value (or the original text
# line), and its tick-thread never logs the raw armored telemetry stream at
# all (see transport.py's `_tick_loop`).
# ---------------------------------------------------------------------------

_BINARY_ARMOR_PREFIX = "*B"


def render_log_line(raw_line: str, *, outbound: bool) -> str | None:
    """Translate one raw wire log line for display in the message monitor.

    ``outbound=True``: ``raw_line`` is a sent line -- a plain text-v2
    command line, OR (095-002 armor) a ``*B<base64>``-encoded
    ``CommandEnvelope``. ``outbound=False``: ``raw_line`` is a received
    line -- a ``*B<base64>``-encoded ``ReplyEnvelope`` (the firmware is
    binary-only plus the 6-verb text rump; a received rump reply, e.g.
    ``DEVICE:...``, is plain text and passes through unchanged the same as
    any other non-armored line).

    Returns:
      - ``None`` to mean "drop this line entirely" -- a ``ReplyEnvelope{tlm}``
        push frame, the high-rate telemetry stream that floods the console
        with no per-line operator value (broken out into the telemetry
        panel instead, same rationale as ``telemetry_panel.
        is_telemetry_log_line()``'s text-plane precedent).
      - ``raw_line`` unchanged, for any non-armored line, or an armored
        line that fails to base64-decode/protobuf-parse (defensive; never
        raises out of a log hook).
      - Otherwise, a single-line human-readable rendering of the decoded
        envelope: a received reply uses ``legacy_render``'s own
        context-free renderers where one exists for that oneof arm
        (``err``/``id``/``echo``/``helptext``); ``ok`` (Ack) and ``cfg``
        (ConfigSnapshot) have no verb-agnostic ``legacy_render`` renderer
        (``render_ok_for_verb()``/``render_cfg_line()`` both need the
        ORIGINAL request's verb/keys, which a bare reply line does not
        carry) -- rendered instead via ``google.protobuf.text_format``, the
        same "readable text instead of raw armor" outcome without
        inventing a verb-guessing scheme. A sent command (outbound) has no
        ``legacy_render`` equivalent at all (that module renders replies,
        not requests) -- always rendered via ``text_format``.
    """
    if not raw_line.startswith(_BINARY_ARMOR_PREFIX):
        return raw_line

    import base64

    from google.protobuf import text_format  # type: ignore[import-untyped]

    from robot_radio.robot.pb2 import envelope_pb2

    try:
        raw_bytes = base64.b64decode(raw_line[len(_BINARY_ARMOR_PREFIX):])
    except Exception:
        return raw_line

    if outbound:
        try:
            cmd = envelope_pb2.CommandEnvelope.FromString(raw_bytes)
        except Exception:
            return raw_line
        return text_format.MessageToString(cmd, as_one_line=True).strip() or raw_line

    try:
        reply = envelope_pb2.ReplyEnvelope.FromString(raw_bytes)
    except Exception:
        return raw_line

    which = reply.WhichOneof("body")
    corr_id = reply.corr_id or None
    if which == "tlm":
        return None
    # 107-003 launch-unblock: render may be None (legacy_render unavailable
    # -- see this module's own docstring); every branch below falls through
    # to the text_format fallback in that case, same as "ok"/"cfg"/"evt".
    if render is not None:
        if which == "err":
            return render.render_error(reply.err, corr_id)
        if which == "id":
            return render.render_id_line(reply.id, corr_id)
        if which == "echo":
            return render.render_ok("echo", reply.echo.payload.decode("utf-8", "replace"), corr_id)
        if which == "helptext":
            return render.render_ok("help", reply.helptext.text, corr_id)
    # "ok" (Ack)/"cfg" (ConfigSnapshot)/"evt" (EventNotify) -- no verb-
    # agnostic legacy_render renderer exists (see this function's own
    # docstring); text_format gives readable text without guessing. Also
    # reached for err/id/echo/helptext when render itself is unavailable.
    return text_format.MessageToString(reply, as_one_line=True).strip() or raw_line
