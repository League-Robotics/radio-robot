"""robot_radio.testgui.binary_bridge — direct-call helpers for the TestGUI's
in-process text-verb entry points, plus the message-monitor line renderer.

This module is the ONE place both hardware transports (``transport.py``'s
``_HardwareTransport``) route every outbound command line NOT already
intercepted by ``_dispatch_managed_move()`` (``D``/``RT``/``SEG 0 <cdeg>``,
which go straight to ``NezhaProtocol.move_twist()``/``move_wheels()``) --
``translate_command(proto, raw_line)`` is the single entry point.

Of the legacy text-v2 verb surface the TestGUI still builds wire strings
for, exactly THREE verbs still have a live binary-plane translation, each a
short, direct call to a live ``NezhaProtocol`` method -- no intermediate
translation layer, table, or reply-rendering module:

- ``OL``/``OA`` -- OTOS calibration, via ``NezhaProtocol.
  set_config_field(robot_config_pb2.OTOS, "linear_scale"/"angular_scale",
  value)`` (``_handle_otos_patch()``). 132-014 retargets this off the
  retired ``OtosConfigPatch``/``otos_config()`` direct-patch-send mechanism
  (109-004) now that ``config.proto`` is deleted (132-013). ``OI`` is
  recognized but has NO live retarget -- see ``_handle_otos_patch()``'s own
  doc comment for why (no successor field in ``robot_config.proto``'s
  ``Otos`` group).
- ``SET key=value ...`` -- config push, via ``NezhaProtocol.
  set_config_field()`` per key (``_handle_set_patch()``), resolved through
  ``protocol._SET_KEY_TARGETS``. Stakeholder bench finding 2026-07-22: SET
  was found rejecting nearly every pushed calibration value against real
  hardware (log: ``'SET pid.kaw=0' rejected: ERR unavailable legacy verb
  translation removed``) -- the original direct-call fix; 132-014 carries
  it onto the new per-field wire primitive.
- ``STREAM <period>`` -- telemetry mode, via ``NezhaProtocol.tlmOn()``/
  ``tlmOff()`` (``_handle_stream_patch()``, 128-004). The current wire has
  no per-call streaming PERIOD control (``TLM:ON``/``TLM:OFF`` are flat
  mode switches) -- a nonzero period means ON, zero means OFF, preserving
  the on/off semantic the GUI's STREAM button/connect-time push already
  used (``operations.py``'s toggle, ``__main__.py``'s connect-time
  ``STREAM 50``).

Every other verb this module used to translate through a shared
verb-tokenizer/reply-renderer module pair (the text-v2 translation
machinery deleted wholesale by commit ``129cbcb3``, 104-002) -- ``S``/``T``/
``D``/``R``/``TURN``/``RT``/``G`` (the ``commands.py`` COMMANDS schema,
itself deleted 128-004; ``D``/``RT`` are still live, but via
``_dispatch_managed_move()``, never through this module), ``GET``,
``SNAP``, the pose-reset family ``SI``/``ZERO``/``OZ``, ``GRIP``/``QLEN``,
and the OTOS-device verbs ``OV``/``OP``/``OR`` -- has no binary-plane arm
at all on the current wire (``docs/protocol-v5.md``: exactly three
``CommandEnvelope`` arms, ``move``/``config``/``stop``, plus ``wheels``/
``estop``; no ``get``/``stream``/``pose_fix``/``otos`` reply arms survive
either, see ``src/protos/envelope.proto``'s own reserved-field history).
``translate_command()`` returns a generic ``ERR unsupported <verb>`` reply
for these without ever touching the wire -- the exact same "parsed, never
sent, replies ERR" outcome every one of these verbs already had (a
permanent unavailable-gate flag this module used to carry made that
outcome universal, not verb-specific).

History (128-004): the two modules this module used to dispatch the WHOLE
text-v2 surface through were deleted wholesale by commit ``129cbcb3``
(104-002), leaving a permanent "translation unavailable" gate flag over
~250 lines that could never execute (``_handle_binary_verb()``,
``_handle_set()``/``_handle_get()``, ``_handle_stream()``,
``_handle_snap()``, and the reply-renderer-specific branches of
``render_log_line()``). 109-004 (OI/OL/OA) and the 2026-07-22 stakeholder
bench fix (SET) had already carved live exceptions out from under that
gate via the same direct-call pattern this module now uses exclusively;
128-004 deletes the dead gate and everything behind it, adds the STREAM
direct call ticket 003 called for, and repoints the old
``ERR unavailable ...`` reply's ``clasi/issues/
binary-bridge-segment-replace-arms-deleted.md`` citation (a file that was
never actually filed) at this docstring instead.
"""

from __future__ import annotations

from robot_radio.robot import protocol
from robot_radio.robot.pb2 import envelope_pb2, robot_config_pb2
from robot_radio.robot.protocol import NezhaProtocol

# OTOS-chip calibration verbs (109-004, Architecture Revision 1). 132-014
# retargets OL/OA onto set_config_field(OTOS, ...) now that OtosConfigPatch/
# ConfigDelta (config.proto) are deleted (132-013) -- see
# _handle_otos_patch()'s own doc comment for OI's fate (no successor field).
# OV (raw position write)/OP (position query)/OR (Kalman reset) have no
# config-group field to set and are not in this set -- they fall to the
# generic unsupported reply below, same as before.
_OTOS_PATCH_VERBS = frozenset({"OI", "OL", "OA"})

_OI_NO_LIVE_TRIGGER_REPLY = (
    "ERR unsupported OI -- robot_config.proto's Otos group has no live "
    "chip-reinit trigger field (offset/scale values only); OtosConfigPatch's "
    "old init trigger has no successor on the current wire (see "
    "configurator.h's OTOS re-appliability-table row and binary_bridge.py's "
    "own module docstring)"
)

_UNSUPPORTED_REPLY_FMT = (
    "ERR unsupported {verb} -- no binary-plane translation on the current "
    "wire (docs/protocol-v5.md; see testgui/binary_bridge.py's own module "
    "docstring)"
)


def _handle_otos_patch(proto: NezhaProtocol, verb: str, pos: list[str]) -> str:
    """``OL <scale>``/``OA <scale>`` -> ``set_config_field(OTOS,
    "linear_scale"/"angular_scale", value)`` (132-014, retargeted off the
    retired ``OtosConfigPatch``/``otos_config()`` direct-patch-send
    mechanism, 109-004 -- ``OtosConfigPatch``/``ConfigDelta`` themselves are
    deleted, 132-013). Same envelope-building path (``NezhaProtocol.
    set_config_field()``) every transport now shares.

    ``OI`` (chip re-init/Kalman-reset trigger) has NO retarget: robot_
    config.proto's ``Otos`` group declares ``offset_x``/``offset_y``/
    ``offset_yaw``/``linear_scale``/``angular_scale`` only -- the old
    ``OtosConfigPatch.init`` fire-and-forget trigger has no
    ``Config::Robot``-shaped successor field to address (confirmed by
    reading ``robot_config.proto`` in full; ``configurator.h``'s own OTOS
    re-appliability-table row calls this out explicitly: "its 6th field,
    init, was a fire-and-forget trigger with no Config::Robot-shaped
    successor and was never persisted either"). No wire traffic is sent
    for ``OI`` -- it gets the SAME honest, immediate, no-round-trip
    "unsupported" treatment ``OV``/``OP``/``OR`` already get below, not a
    fabricated success.

    Returns a plain, hand-rolled ``OK``/``ERR`` reply line.
    """
    if verb == "OI":
        return _OI_NO_LIVE_TRIGGER_REPLY

    field_name = "linear_scale" if verb == "OL" else "angular_scale"
    if not pos:
        return f"ERR badarg {verb} requires <scale>"
    try:
        value = float(pos[0])
    except ValueError as exc:
        return f"ERR badarg {exc}"

    try:
        ack = proto.set_config_field(robot_config_pb2.OTOS, field_name, value)
    except ConnectionError as exc:
        return f"ERR unknown {exc}"

    if ack is None:
        return f"ERR unknown {verb} timeout or rejected"
    return f"OK {verb.lower()}"


def _parse_set_kv(tokens: list[str]) -> dict[str, str]:
    """Parse ``SET``'s own ``key=value key2=value2 ...`` positional tokens
    into a kv dict. Raises ``ValueError`` on a malformed token (no ``=``)."""
    kv: dict[str, str] = {}
    for tok in tokens:
        if "=" not in tok:
            raise ValueError(f"malformed SET arg (expected key=value): {tok!r}")
        key, _, value = tok.partition("=")
        kv[key] = value
    return kv


def _handle_set_patch(proto: NezhaProtocol, tokens: list[str]) -> str:
    """``SET key=value ...`` -> one ``set_config_field()`` round trip per
    key (132-014, retargeted off the retired ``set_config()``-via-
    ``ConfigDelta`` mechanism), resolving each flat key to its
    ``(ConfigGroupTarget, field_name)`` pair via the SAME
    ``protocol._SET_KEY_TARGETS`` vocabulary ``NezhaProtocol.set_config()``
    itself now uses -- called DIRECTLY here (not through ``set_config()``)
    so a per-key rejection (unknown field, ``ERR_NOT_LIVE`` on a boot-only
    target, ``ERR_BUSY`` while a motor is moving) names the OFFENDING key
    rather than collapsing every key's outcome into one generic failure.
    The SAME method ``SimTransport._handle_config_set()`` (``transport.py``)
    now also calls, for Sim.

    Returns a plain, hand-rolled ``OK set ...``/``ERR ...`` reply line.
    """
    try:
        kv = _parse_set_kv(tokens)
    except ValueError as exc:
        return f"ERR badarg {exc}"
    if not kv:
        return "ERR badarg no key=value pairs"
    bad = [k for k in kv if k not in protocol._SET_KEY_TARGETS]
    if bad:
        return f"ERR badkey {bad[0]}"
    try:
        kwargs = {k: float(v) for k, v in kv.items()}
    except ValueError:
        return "ERR badarg bad value"

    applied: dict[str, str] = {}
    for key, value in kwargs.items():
        target, field_name = protocol._SET_KEY_TARGETS[key]
        ack = proto.set_config_field(target, field_name, value)
        if ack is None:
            return f"ERR badarg set failed at {key}"
        applied[key] = protocol._format_config_value(value)

    body = " ".join(f"{k}={v}" for k, v in applied.items())
    return f"OK set {body}"


def _handle_stream_patch(proto: NezhaProtocol, pos: list[str]) -> str:
    """``STREAM <period>`` -> ``NezhaProtocol.tlmOn()``/``tlmOff()`` direct
    call (128-004, per ticket 003's own note). A nonzero period means ON, a
    zero (or unparsable) period means OFF -- the v5 wire has no per-call
    streaming PERIOD control any more (``tlmOn()``/``tlmOff()`` are flat
    ``TLM:ON``/``TLM:OFF`` mode switches, see their own docstrings); this
    only preserves the on/off SEMANTIC the GUI's STREAM controls already
    used (``operations.py``'s toggle sends ``STREAM 50``/``STREAM 0``,
    ``__main__.py``'s connect-time push sends ``STREAM 50``).

    Returns a plain, hand-rolled ``OK stream ...`` reply line -- this call
    is fire-and-forget on the wire (``send_fast()``, no ack), so unlike
    ``_handle_otos_patch()``/``_handle_set_patch()`` there is no ack to
    wait on and no ``ERR nak``/``ERR unknown timeout`` outcome.
    """
    if not pos:
        return "ERR badarg period"
    try:
        requested = int(float(pos[0]))
    except ValueError:
        return "ERR badarg period"
    if requested <= 0:
        proto.tlmOff()
        return "OK stream period=0"
    proto.tlmOn()
    return f"OK stream period={requested}"


def translate_command(proto: NezhaProtocol, raw_line: str) -> str:
    """Translate one text-v2 line to binary, send it, and return the
    rendered text-v2 reply line.

    Empty/whitespace-only input returns ``""`` (no verb to dispatch, no
    wire traffic).

    ``OI``/``OL``/``OA`` (109-004), ``SET`` (stakeholder bench fix,
    2026-07-22), and ``STREAM`` (128-004) are the only verbs with a live
    binary-plane translation -- each constructs and sends a real envelope
    (or, for STREAM, a real cleartext ``TLM:ON``/``TLM:OFF`` line) directly
    via a live ``NezhaProtocol`` method. Every other verb returns a generic
    ``ERR unsupported <verb>`` reply without ever touching the wire -- see
    this module's own docstring for why (no binary-plane arm exists for it
    on the current wire).
    """
    stripped = raw_line.strip()
    if not stripped:
        return ""

    tokens = stripped.split()
    verb = tokens[0].upper()
    pos = tokens[1:]

    if verb in _OTOS_PATCH_VERBS:
        return _handle_otos_patch(proto, verb, pos)
    if verb == "SET":
        return _handle_set_patch(proto, pos)
    if verb == "STREAM":
        return _handle_stream_patch(proto, pos)

    return _UNSUPPORTED_REPLY_FMT.format(verb=verb)


# ---------------------------------------------------------------------------
# Serial/message monitor filtering (097, Goal 4) -- translates every raw
# wire log line SerialConnection's on_send/on_recv hooks deliver (see
# io/serial_conn.py's `_reader_loop`/`send_envelope` docstrings: on_recv
# fires for EVERY decoded line/frame before any classification, including
# the high-rate binary telemetry push stream) into readable text for the
# TestGUI's log pane, instead of an opaque blob a human cannot read.
# `_HardwareTransport`'s `_on_send`/`_on_recv` closures (transport.py) are
# the only callers -- SimTransport never needs this: its own
# send()/command()/_drain_cmd_queue() already log translate_command()'s
# human-readable return value (or the original text line), and its
# tick-thread never logs the raw framed telemetry stream at all (see
# transport.py's `_tick_loop`).
#
# 123-002/003 COBS+CRC cutover: a binary frame is no longer distinguished by
# a `*B` TEXT prefix -- ``SerialConnection`` now hands this function raw
# ``bytes`` for a binary frame (post-delimiter-strip, still COBS+CRC-encoded)
# and a plain ``str`` for a text-plane line, so the dispatch below is by
# Python TYPE, not by string prefix.
#
# 128-004: the reply-renderer-specific rendering this function used to fall
# back onto for ``err``/``id``/``echo``/``helptext`` replies was already
# permanently unreachable (the module it called into was deleted wholesale
# by 104-002, and ``id``/``echo``/``helptext`` no longer exist as
# ``ReplyEnvelope.body`` oneof fields at all -- only ``ok``/``err``/``tlm``
# do). Every reply now renders via ``google.protobuf.text_format``
# unconditionally -- this is the SAME text_format fallback this function
# already fell to for every reply in practice, just without the dead
# "renderer available" branch that could never be taken.
# ---------------------------------------------------------------------------


def render_log_line(raw_line: "str | bytes", *, outbound: bool) -> str | None:
    """Translate one raw wire log line/frame for display in the message
    monitor.

    ``raw_line`` is a plain ``str`` for a cleartext-plane line (HELLO/PING/
    ID/VER and their replies -- returned unchanged) or raw ``bytes`` for a
    binary wire LINE (124-005: the FULL ``<COMMAND>':'<COBS+CRC bytes>``
    content -- ``io/serial_conn.py``'s ``on_send``/``on_recv`` hooks pass
    the whole line now, not the bare COBS body, precisely so this function
    can recover the verb its own ``decode_frame()`` call needs to scope the
    CRC correctly): ``outbound=True`` means a sent ``CommandEnvelope``,
    ``outbound=False`` means a received ``ReplyEnvelope``.

    Returns:
      - ``None`` to mean "drop this line entirely" -- a ``ReplyEnvelope{tlm}``
        push frame, the high-rate telemetry stream that floods the console
        with no per-line operator value (broken out into the telemetry
        panel instead, same rationale as ``telemetry_panel.
        is_telemetry_log_line()``'s text-plane precedent).
      - The original text line unchanged, for a ``str`` input, or a hex
        fallback (``<binary: malformed, N bytes>``) for a binary frame that
        fails to COBS/CRC-decode or protobuf-parse (defensive; never raises
        out of a log hook).
      - Otherwise, a single-line human-readable rendering of the decoded
        envelope via ``google.protobuf.text_format`` (128-004: unconditional
        -- see this section's own header comment for why there is no other
        rendering path any more).

    Received (``outbound=False``) frames are exactly one message shape now
    (124-009, robot-state-blackboard-...md, issue's own "TelemetrySecondary
    dies"): ``pb2.ReplyEnvelope``. ``pb2.TelemetrySecondary`` -- its former
    sibling, a slower ~5Hz diagnostic frame emitted as its own bare,
    independently-framed frame -- is DELETED outright, along with the
    ReplyEnvelope-vs-TelemetrySecondary disambiguation fallback this
    function used to need (see ``io/serial_conn.py``'s
    ``_handle_binary_reply()`` docstring for the historical rationale). A
    ``ReplyEnvelope`` parse that raises, or succeeds with an EMPTY body
    oneof, is now simply malformed -- there is no second shape to retry.
    """
    if isinstance(raw_line, str):
        return raw_line

    from google.protobuf import text_format  # type: ignore[import-untyped]

    from robot_radio.io.wire_codec import decode_frame

    def _malformed_marker() -> str:
        return f"<binary: malformed, {len(raw_line)} bytes>"

    # 124-005: `raw_line` is the FULL wire line -- split off the leading
    # `<COMMAND>':'` prefix (protocol v5's own grammar, issue §1) so
    # decode_frame() can scope its CRC check over the real verb, matching
    # how the line was actually encoded (Comms::sendReply()/
    # Comms::decodeBinaryFrame(), comms.cpp). No ':' at all is itself
    # malformed -- every binary verb line always carries one.
    command, sep, cobs_body = raw_line.partition(b":")
    if not sep:
        return _malformed_marker()

    raw_bytes = decode_frame(cobs_body, command=command)
    if raw_bytes is None:
        return _malformed_marker()

    if outbound:
        try:
            cmd = envelope_pb2.CommandEnvelope.FromString(raw_bytes)
        except Exception:
            return _malformed_marker()
        return text_format.MessageToString(cmd, as_one_line=True).strip() or _malformed_marker()

    try:
        reply = envelope_pb2.ReplyEnvelope.FromString(raw_bytes)
    except Exception:
        reply = None

    if reply is None or reply.WhichOneof("body") is None:
        # Not a (recognizable) ReplyEnvelope -- 124-009: there is no second
        # shape to retry any more (TelemetrySecondary is deleted outright,
        # robot-state-blackboard-...md).
        return _malformed_marker()

    which = reply.WhichOneof("body")
    if which == "tlm":
        return None
    return text_format.MessageToString(reply, as_one_line=True).strip() or _malformed_marker()
