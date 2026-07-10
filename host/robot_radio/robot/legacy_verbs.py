"""legacy_verbs.py -- M5 rogo Translator Proxy (097-004).

Pure tokenizer + verb-to-envelope dispatch tables, extending ticket 002's
``legacy_translate.py`` motion builders (S/D/T reused verbatim; RT/MOVE/
MOVER/ECHO/PING/STOP/ID added here) to the FULL text-v2 verb surface a
legacy client might still send. No ``SerialConnection``/socket/PTY
reference anywhere in this module -- every function takes plain tokens and
returns a plain ``pb2.CommandEnvelope`` (or raises ``ValueError`` on a
malformed/incomplete command); the caller (``cli.py``'s ``cmd_send``, or
``io/proxy.py``'s ``ProtocolBridge``) owns the wire round trip.

Two independent callers, two independent vocabularies -- do not conflate
them:

  - ``cli.py``'s ``cmd_send`` (the original "rogo REPL Translator" scope,
    delivered here at near-zero incremental cost via thin aliases): a verb
    in ``PROTOCOL_VERBS`` translates to binary; a verb in ``RUMP_VERBS``
    (the SAME five-verb safety rump architecture-update.md (097) Step 1
    names: PING/ID/HELLO/HELP/STOP) is sent as plain TEXT, unchanged; any
    other verb (R/TURN/G/DEV/unrecognized) ALSO falls through to plain
    text, unchanged -- ``cmd_send`` never invents a translation firmware
    never had.
  - ``io/proxy.py``'s ``ProtocolBridge`` has its OWN, wider routing table
    (``BINARY_DISPATCH`` covers PING/STOP/ID/VER too, since the proxy talks
    ONLY binary to the robot -- there is no text rump on the wire for it to
    fall back to). ``ProtocolBridge._handle_client_line`` consults
    ``BINARY_DISPATCH`` directly for the verbs it knows how to translate,
    and layers proxy-only local/typed-error handling (HELLO/HELP answered
    locally, SET/GET/STREAM/SNAP/TLM given their own multi-round-trip
    handlers reusing ``protocol.py``, R/TURN/G/GRIP/DEV/QLEN/pose-otos ->
    typed ``ERR unsupported``) on top -- see that module's own docstring.

Transcription source (never re-derived -- 095 Decision 5's "transcribe,
don't re-derive" discipline): every builder below cites the firmware
function/file it ports, same as ``legacy_translate.py``'s own header.
"""

from __future__ import annotations

from typing import Callable

from robot_radio.robot import legacy_translate
from robot_radio.robot.pb2 import envelope_pb2


# ---------------------------------------------------------------------------
# Tokenizer -- CommandProcessor::parseTokens()/parseKV() mirror
# (command_processor.cpp).
# ---------------------------------------------------------------------------


def tokenize_send_line(raw: str) -> tuple[str, list[str], dict[str, str]]:
    """Split ``raw`` into ``(VERB, positional_tokens, kv_dict)``.

    Mirrors ``parseTokens()``'s verb-only upper-casing (positional/kv
    tokens keep their original case) and ``parseKV()``'s "any token
    containing '=' is a kv pair, everything else is positional" split.
    Unlike the firmware, this does NOT strip a trailing ``#<id>`` token --
    that is ``split_corr_id()``'s own job (called separately by
    ``ProtocolBridge``; ``cmd_send`` never needs it since ``rogo send``
    lets ``SerialConnection.send()``/``send_envelope()`` own corr-id
    assignment). An empty/whitespace-only line returns ``("", [], {})``.
    """
    parts = raw.split()
    if not parts:
        return "", [], {}
    verb = parts[0].upper()
    positional: list[str] = []
    kv: dict[str, str] = {}
    for tok in parts[1:]:
        if "=" in tok:
            k, _, v = tok.partition("=")
            if k:
                kv[k] = v
                continue
        positional.append(tok)
    return verb, positional, kv


def split_corr_id(raw: str) -> tuple[str, str | None]:
    """Split a trailing ``#<digits>`` correlation-id token off ``raw``.

    Mirrors ``parseTokens()``'s own corr-id extraction (the last token,
    IFF it is ``#`` followed by one or more digits and nothing else).
    Returns ``(line_without_corr_id, corr_id_or_None)``; the returned line
    is stripped but otherwise untouched (still needs ``tokenize_send_line``
    to split verb/positional/kv).
    """
    stripped = raw.strip()
    if not stripped:
        return stripped, None
    head, sep, tail = stripped.rpartition(" ")
    if sep and tail.startswith("#") and tail[1:].isdigit():
        return head, tail[1:]
    if not sep and stripped.startswith("#") and stripped[1:].isdigit():
        # A bare "#<id>" with no verb at all -- degenerate, but parseTokens()
        # would still strip it, leaving an empty line.
        return "", stripped[1:]
    return stripped, None


def kvfloat(kv: dict[str, str], key: str, default: float = 0.0) -> float:
    """``kvFloat()`` mirror (arg_parse.cpp): parse ``kv[key]`` as a float;
    an absent key or an unparsable value both fall back to ``default``
    (matching the firmware's own "malformed kv value silently defaults"
    posture for optional MOVE/MOVER modifiers)."""
    if key not in kv:
        return default
    try:
        return float(kv[key])
    except (TypeError, ValueError):
        return default


# ---------------------------------------------------------------------------
# Verb -> CommandEnvelope builders. Each raises ValueError(usage message)
# on too few positional args -- callers (cmd_send/_handle_binary_verb)
# catch it and render a typed usage error, never a raw traceback.
# ---------------------------------------------------------------------------


def envelope_for_drive(pos: list[str], kv: dict[str, str]) -> envelope_pb2.CommandEnvelope:
    """``S <left> <right>`` -> ``{drive: DrivetrainCommand{wheels}}``
    (``handleS()``, motion_commands.cpp, via ``legacy_translate.
    wheel_targets_for_drive()``, ticket 002)."""
    if len(pos) < 2:
        raise ValueError("S requires <left> <right>")
    wheels = legacy_translate.wheel_targets_for_drive(float(pos[0]), float(pos[1]))
    env = envelope_pb2.CommandEnvelope()
    env.drive.wheels.CopyFrom(wheels)
    return env


def envelope_for_timed(pos: list[str], kv: dict[str, str]) -> envelope_pb2.CommandEnvelope:
    """``T <left> <right> <ms>`` -> ``{segment: MotionSegment}``
    (``handleT()``, via ``legacy_translate.segment_for_timed()``, ticket 002)."""
    if len(pos) < 3:
        raise ValueError("T requires <left> <right> <ms>")
    seg = legacy_translate.segment_for_timed(float(pos[0]), float(pos[1]), int(float(pos[2])))
    return envelope_pb2.CommandEnvelope(segment=seg)


def envelope_for_distance(pos: list[str], kv: dict[str, str]) -> envelope_pb2.CommandEnvelope:
    """``D <left> <right> <mm>`` -> ``{segment: MotionSegment}``
    (``handleD()``, via ``legacy_translate.segment_for_distance()``, ticket 002)."""
    if len(pos) < 3:
        raise ValueError("D requires <left> <right> <mm>")
    seg = legacy_translate.segment_for_distance(float(pos[0]), float(pos[1]), int(float(pos[2])))
    return envelope_pb2.CommandEnvelope(segment=seg)


def envelope_for_rt(pos: list[str], kv: dict[str, str]) -> envelope_pb2.CommandEnvelope:
    """``RT <relAngle_cdeg>`` -> ``{segment: MotionSegment}`` (final_heading
    only) (``handleRT()``, via ``legacy_translate.segment_for_rt()``)."""
    if len(pos) < 1:
        raise ValueError("RT requires <relAngle>")
    seg = legacy_translate.segment_for_rt(float(pos[0]))
    return envelope_pb2.CommandEnvelope(segment=seg)


def envelope_for_move(pos: list[str], kv: dict[str, str]) -> envelope_pb2.CommandEnvelope:
    """``MOVE <distance> <direction> <finalHeading> [v=][a=][j=][w=][wa=]
    [wj=][s=]`` -> ``{segment: MotionSegment}`` (``handleMove()``, via
    ``legacy_translate.segment_for_move()``)."""
    if len(pos) < 3:
        raise ValueError("MOVE requires <distance> <direction> <finalHeading>")
    seg = legacy_translate.segment_for_move(
        float(pos[0]), float(pos[1]), float(pos[2]),
        speed_max=kvfloat(kv, "v"), accel_max=kvfloat(kv, "a"),
        jerk_max=kvfloat(kv, "j"), yaw_rate_max=kvfloat(kv, "w"),
        yaw_accel_max=kvfloat(kv, "wa"), yaw_jerk_max=kvfloat(kv, "wj"),
        stream=kvfloat(kv, "s") > 0.5)
    return envelope_pb2.CommandEnvelope(segment=seg)


def envelope_for_mover(pos: list[str], kv: dict[str, str]) -> envelope_pb2.CommandEnvelope:
    """``MOVER <distance> <direction> <finalHeading> [t=][v=][w=][a=][j=]
    [wa=][wj=]`` -> ``{replace: MotionSegment}`` (``handleMover()``, via
    ``legacy_translate.segment_for_mover()``) -- REPLACE semantics, not
    ``segment``."""
    if len(pos) < 3:
        raise ValueError("MOVER requires <distance> <direction> <finalHeading>")
    seg = legacy_translate.segment_for_mover(
        float(pos[0]), float(pos[1]), float(pos[2]),
        time=kvfloat(kv, "t"), v=kvfloat(kv, "v"), accel_max=kvfloat(kv, "a"),
        jerk_max=kvfloat(kv, "j"), omega=kvfloat(kv, "w"),
        yaw_accel_max=kvfloat(kv, "wa"), yaw_jerk_max=kvfloat(kv, "wj"))
    return envelope_pb2.CommandEnvelope(replace=seg)


def envelope_for_echo(pos: list[str], kv: dict[str, str]) -> envelope_pb2.CommandEnvelope:
    """``ECHO <text...>`` -> ``{echo: Echo{payload}}``. Every positional
    token is re-joined space-separated (mirrors ``handleEcho()``'s own
    token reassembly, motion_commands.cpp/system_commands.cpp)."""
    payload = " ".join(pos)
    env = envelope_pb2.CommandEnvelope()
    env.echo.payload = payload.encode("utf-8")
    return env


def envelope_for_ping(pos: list[str], kv: dict[str, str]) -> envelope_pb2.CommandEnvelope:
    """``PING`` -> ``{ping: Ping{}}`` (zero-field arm)."""
    env = envelope_pb2.CommandEnvelope()
    env.ping.SetInParent()
    return env


def envelope_for_stop(pos: list[str], kv: dict[str, str]) -> envelope_pb2.CommandEnvelope:
    """``STOP`` -> ``{stop: Stop{}}`` (zero-field arm, "cannot be malformed")."""
    env = envelope_pb2.CommandEnvelope()
    env.stop.SetInParent()
    return env


def envelope_for_id(pos: list[str], kv: dict[str, str]) -> envelope_pb2.CommandEnvelope:
    """``ID`` or ``VER`` -> ``{id: DeviceId{}}`` (empty request, Decision 4
    -- VER's content is a strict subset of ID's reply, no independent
    binary ``ver`` arm exists)."""
    env = envelope_pb2.CommandEnvelope()
    env.id.SetInParent()
    return env


# ---------------------------------------------------------------------------
# Dispatch tables
# ---------------------------------------------------------------------------

_EnvelopeBuilder = Callable[[list[str], dict[str, str]], envelope_pb2.CommandEnvelope]

# BINARY_DISPATCH: every verb with a 1:1 CommandEnvelope translation. Wider
# than PROTOCOL_VERBS below -- the proxy (io/proxy.py) also routes
# PING/STOP/ID/VER through here (it has no text rump to fall back to);
# cmd_send only consults this table for verbs in PROTOCOL_VERBS.
BINARY_DISPATCH: dict[str, _EnvelopeBuilder] = {
    "S": envelope_for_drive,
    "D": envelope_for_distance,
    "T": envelope_for_timed,
    "RT": envelope_for_rt,
    "MOVE": envelope_for_move,
    "MOVER": envelope_for_mover,
    "ECHO": envelope_for_echo,
    "PING": envelope_for_ping,
    "STOP": envelope_for_stop,
    "ID": envelope_for_id,
    "VER": envelope_for_id,
}

# PROTOCOL_VERBS: the seven verbs `rogo send` translates to binary --
# architecture-update.md (097) Step 1's own list, reused verbatim by the
# committed test_cli_send_translator.py.
PROTOCOL_VERBS = frozenset({"S", "D", "T", "RT", "MOVE", "MOVER", "ECHO"})

# RUMP_VERBS: architecture-update.md (097) Step 1's five-verb safety rump
# (PING/ID/HELLO/HELP/STOP) -- `rogo send` sends these as plain text,
# unchanged (no translation invented for verbs a bare terminal already
# speaks natively). NOTE: this is `cmd_send`'s OWN vocabulary, distinct
# from what the CURRENT firmware text rump actually retains post-097-006
# (3 verbs: STOP/PING/HELLO) -- `cmd_send` doesn't special-case that; it
# just forwards the text line and lets the firmware (or its absence)
# answer. The proxy (`ProtocolBridge`) does NOT use this table at all --
# it routes PING/STOP/ID/VER through BINARY_DISPATCH and answers
# HELLO/HELP locally (see io/proxy.py).
RUMP_VERBS = frozenset({"PING", "ID", "HELLO", "HELP", "STOP"})


# ---------------------------------------------------------------------------
# decode_reply_body() -- pretty-printer for `rogo send --decode`.
# ---------------------------------------------------------------------------


def decode_reply_body(reply: envelope_pb2.ReplyEnvelope) -> str:
    """Render a populated ``ReplyEnvelope``'s oneof body as
    ``"<arm>:\\n  field = value\\n  ...\\n  corr_id = <id>"`` -- a
    human-readable alternative to the raw protobuf text-format dump
    (``str(reply)``, ``"ok {\\n  q: 3\\n...}"``). Returns
    ``"(empty reply)  corr_id=<id>"`` when no oneof arm is set."""
    which = reply.WhichOneof("body")
    if which is None:
        return f"(empty reply)  corr_id={reply.corr_id}"
    body = getattr(reply, which)
    lines = [f"{which}:"]
    for field in body.DESCRIPTOR.fields:
        value = getattr(body, field.name)
        lines.append(f"  {field.name} = {value}")
    lines.append(f"  corr_id = {reply.corr_id}")
    return "\n".join(lines)
