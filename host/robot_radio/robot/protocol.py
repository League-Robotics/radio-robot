"""NezhaProtocol — v2 wire-protocol adapter for the Nezha firmware.

Owns the SerialConnection and is the only code that touches the serial port.
All command encoding and response parsing lives here; higher-level objects
(NezhaState, Nezha) delegate every wire operation to this class.

Wire format — protocol v2
--------------------------
Requests:
  One '\n'-terminated line, whitespace-delimited tokens.
  Verb is upper-cased by the firmware; remaining tokens preserve case.
  Optional trailing '#<id>' for request correlation.
  Example: "S 200 150\n", "SET ml=0.487\n", "T 200 200 1000 #7\n"

Responses:
  OK   — command accepted:       "OK pong t=12345"
  ERR  — rejected:               "ERR badarg missing key"
  EVT  — async event:            "EVT done T", "EVT done T #12", "EVT safety_stop"
                                 May carry a trailing reason= token, e.g.:
                                 "EVT done T reason=time", "EVT safety_stop reason=watchdog"
  TLM  — telemetry frame:        "TLM t=12345 enc=1024,1019 pose=350,-12,1780"
  CFG  — config dump:            "CFG ml=0.487 mr=0.481 ..."
  ID   — identity/capabilities:  "ID model=Nezha2 name=GUTOV ..."

EVT done T/D/G and EVT safety_stop carry a trailing '#<id>' when the
originating T/D/G command included one.  Bare events (no id) are unchanged.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Generator

from robot_radio.io.serial_conn import SerialConnection

# Binary-plane pb2 bindings (096-007, M6 Host Config/Telemetry Client). Safe
# to import at module level here (unlike robot_radio.io.serial_conn.py --
# see that module's own _get_envelope_pb2() docstring for the circular-
# import hazard it avoids): robot_radio.robot.pb2 has no dependency back onto
# robot_radio.robot or robot_radio.io, so importing it while
# robot_radio.robot's own __init__.py is still mid-execution (which is
# always the case when this module is first loaded -- __init__.py imports
# this module itself) never re-enters a partially-initialized module.
from robot_radio.robot.pb2 import config_pb2, drivetrain_pb2, envelope_pb2, planner_pb2, telemetry_pb2

# legacy_translate (097-002, M4 Legacy Verb Translator): pure/stateless
# verb -> pb2-message functions, no SerialConnection/I/O reference. Same
# "no circular-import hazard" reasoning as the robot.pb2 import above --
# robot_radio.robot.legacy_translate depends only on robot_radio.robot.pb2,
# never back onto robot_radio.robot's own __init__.py or this module.
from robot_radio.robot import legacy_translate


# ---------------------------------------------------------------------------
# Public data types
# ---------------------------------------------------------------------------

# kAngleScale mirror (source/telemetry/tlm_frame.cpp): 18000/pi, converting
# radians (telemetry.proto's Pose2D.h, common.proto) to centidegrees (the
# int this dataclass's pose/otos fields carry, matching the historical
# text-plane TLM parser's own units -- see TLMFrame.from_pb2()'s docstring).
# Same scale factor, same truncate-toward-zero int() cast the firmware's
# static_cast<int> applies -- see TLMFrame.from_pb2().
_ANGLE_SCALE = 5729.5779513  # [cdeg/rad]

# kStreamFloorMs mirror (source/commands/telemetry_commands.cpp /
# binary_channel.cpp): the firmware's own minimum non-zero STREAM period --
# handleStream() clamps any 1..19 request up to this floor. snap()'s
# arm-wait-disarm synthesis (097-003, M3, architecture-update.md Decision 4)
# arms at this floor so its one-shot wait is as short as the firmware allows;
# no host-side clamping semantics are implied beyond "this is what the
# firmware will actually use".
_STREAM_FLOOR_MS = 20  # [ms]

# modeChar() mirror (source/telemetry/tlm_frame.cpp): maps msg::DriveMode
# (telemetry.mode, planner.proto) to the SAME single-character mode= wire
# value the historical text-plane TLM parser read off a text STREAM/SNAP
# frame's "mode=" token. VELOCITY has no dedicated character (modeChar()'s
# own `default: return 'I';` case) -- mirrored here via .get()'s fallback in
# from_pb2(), not a dict entry, so a future DriveMode value added to
# planner.proto without a matching modeChar() case falls back the same way
# on both sides.
_DRIVE_MODE_CHAR = {
    planner_pb2.IDLE: "I",
    planner_pb2.STREAMING: "S",
    planner_pb2.TIMED: "T",
    planner_pb2.DISTANCE: "D",
    planner_pb2.GO_TO: "G",
}


@dataclass
class TLMFrame:
    """Parsed TLM telemetry frame from the firmware.

    All fields are optional — only sensors present in the frame are populated.
    ``t`` is the robot clock in milliseconds at sensor-sample time.
    ``seq`` is the D10 sequence counter (uint16, wrapping at 65535); absent on
    pre-028-005 firmware.  Use ``tlm_drop_rate(frames)`` to estimate packet loss.
    ``pose`` heading is in centi-degrees (integer), positions in mm.
    ``vel`` is per-wheel measured speed in mm/s:
      - Differential build: 2-tuple (vL_mmps, vR_mmps).
      - Mecanum build:      4-tuple (vFR_mmps, vFL_mmps, vBR_mmps, vBL_mmps).
    ``twist`` is fused body-frame velocity:
      - Differential build: 2-tuple (v_mmps, omega_mradps).
      - Mecanum build:      3-tuple (vx_mmps, vy_mmps, omega_mradps).
    The vy field in the mecanum twist is the lateral body velocity from OTOS.
    ``wedge`` is the per-wheel encoder-wedge detector latch state (064-004):
    (left, right), each 0 (healthy) or 1 (latched). Unconditional on the
    firmware side (not gated by STREAM fields=) — always present on any
    firmware new enough to emit it; absent (None) on older firmware.
    ``encpose`` is the encoder-only dead-reckoned world pose (068-001):
    (x, y, heading) in (mm, mm, cdeg), arc-integrated from wheel deltas only —
    same shape/units as ``otos``/``pose``. Gated by STREAM fields=; absent
    (None) on older firmware or when explicitly excluded from the
    subscription.
    ``otos_health`` is the OTOS fusion-gate health state (074-004):
    (status, blocked) — ``status`` is the raw OTOS STATUS byte (0 = clean),
    ``blocked`` is ``Drive::_otosFusionBlocked`` (0/1) as a bool. Note:
    ``otos`` above is the raw, last-successfully-read pose and does NOT go
    stale or change meaning when fusion is blocked — ``otos_health`` is what
    tells a host fusion is currently blocked. Unconditional on the firmware
    side (not gated by freshness, matching ``wedge``'s precedent) — always
    present on any firmware new enough to emit it; absent (None) on older
    firmware.
    """
    t: int | None = None
    mode: str | None = None
    seq: int | None = None                       # D10 sequence counter (uint16, wraps at 65535)
    enc: tuple[int, int] | None = None          # (left, right) [mm]
    pose: tuple[int, int, int] | None = None    # (x, y, heading) [mm, mm, cdeg]
    vel: tuple[int, ...] | None = None          # differential: (vL, vR); mecanum: (vFR, vFL, vBR, vBL) mm/s
    cmd_vel: tuple[int, int] | None = None      # (left, right) COMMANDED per-wheel velocity (PID setpoint) mm/s
    twist: tuple[int, ...] | None = None        # differential: (v, omega_mrad); mecanum: (vx, vy, omega_mrad)
    otos: tuple[int, int, int] | None = None    # (x, y, heading) [mm, mm, cdeg] — raw OTOS pose
    line: tuple[int, int, int, int] | None = None   # (g1, g2, g3, g4)
    color: tuple[int, int, int, int] | None = None  # (r, g, b, c)
    ekf_rej: int | None = None                   # cumulative EKF gate rejection count
    wedge: tuple[int, int] | None = None         # (left, right) wedge-latch state, 0/1 each (064-004)
    encpose: tuple[int, int, int] | None = None  # (x, y, heading) [mm, mm, cdeg] — encoder-only pose (068-001)
    otos_health: tuple[int, bool] | None = None  # (raw STATUS byte, fusion_blocked) — OTOS health (074-004)

    @classmethod
    def from_pb2(cls, telemetry: "telemetry_pb2.Telemetry") -> "TLMFrame":
        """Build a TLMFrame from a binary-plane ``pb2.Telemetry`` message
        (``ReplyEnvelope.body.tlm``, envelope.proto/telemetry.proto).

        Adapts telemetry.proto's wire shape onto this SAME dataclass shape
        the historical text-plane TLM parser (retired 097-003; see
        ``tests/unit/test_protocol_binary_client.py`` for the frozen
        reference copy used to verify field-for-field parity) already
        produced from a text STREAM/SNAP line — for every field the two
        formats share, ``from_pb2(telemetry)`` is field-for-field equal to
        what that historical parser produced from the matching text line.
        This is an ADAPTER, not a redesign: TLMFrame's existing fields/shape
        are unchanged; pb2's shape bends to fit them, never the reverse.

        Truncation matches the firmware's own text formatter exactly
        (``buildTlmFrame()``'s ``static_cast<int>``, i.e. truncate-toward-
        zero) — Python's ``int()`` on a float does the same, so a frame
        built from the same underlying sensor values agrees across either
        wire format.

        Fields left at this dataclass's own default (``None``) because
        telemetry.proto and TLMFrame do not share them:
          - ``wedge``, ``encpose``, ``otos_health`` — parsed by the
            historical text-plane parser from the text plane's
            ``wedge=``/``encpose=``/``otos_health=`` tokens, but
            telemetry.proto declares no matching field (``encpose`` was
            trimmed at 096-001 — see telemetry.proto's own file header;
            ``wedge``/``otos_health`` were never part of the STREAM/SNAP
            field union telemetry.proto curates). NOTE (097-003): this is a
            genuine, permanent gap for any consumer that needs ``encpose``
            from a LIVE binary telemetry stream — see
            ``host/robot_radio/calibration/fit_sim_error_model.py``'s
            module docstring for the one consumer this ticket found that
            structurally depends on it, and why that consumer stays on the
            text plane rather than silently losing the field.
          - OTOS connectivity — telemetry.proto DOES carry
            ``has_otos``/``otos``/``otos_connected``, but the historical
            text-plane parser never parsed the text plane's own
            ``otosconn=`` token into any TLMFrame field either, so
            ``otos_connected`` is dropped here too, for parity with the
            text path this dataclass already models.
          - ``acc_left``/``acc_right``/``active``/``conn_left``/
            ``conn_right``/``glitch_left``/``glitch_right``/``ts_left``/
            ``ts_right`` — telemetry.proto ALSO curates these from the
            separate one-shot ``TLM`` verb's ``OK tlm ...`` reply
            (``handleTlm()``, motion_commands.cpp) — a DIFFERENT text wire
            shape than the STREAM/SNAP ``TLM t=... mode=...`` line this
            dataclass models. TLMFrame has no slot for these; they are
            silently dropped here, like any other field this dataclass does
            not declare.

        ``vel``/``cmd_vel``/``twist`` are always built as the DIFFERENTIAL
        tuple shape (matching this build's differential-only drivetrain and
        ``buildTlmFrame()``'s own ``"%d,%d"`` formatting): telemetry.proto's
        ``twist`` is a ``BodyTwist3`` (``v_x``, ``v_y``, ``omega``), but the
        firmware always zero-fills ``v_y`` for a differential build
        (tlm_frame.cpp), so ``v_y`` is dropped here exactly as the text
        plane's own 2-value ``twist=%d,%d`` already drops it.
        """
        frame = cls()
        frame.t = telemetry.now
        frame.mode = _DRIVE_MODE_CHAR.get(telemetry.mode, "I")
        frame.seq = telemetry.seq

        if telemetry.has_enc:
            frame.enc = (int(telemetry.enc_left), int(telemetry.enc_right))

        if telemetry.has_vel:
            frame.vel = (int(telemetry.vel_left), int(telemetry.vel_right))

        if telemetry.has_cmd_vel:
            frame.cmd_vel = (int(telemetry.cmd_vel_left), int(telemetry.cmd_vel_right))

        if telemetry.has_pose:
            frame.pose = (
                int(telemetry.pose.x),
                int(telemetry.pose.y),
                int(telemetry.pose.h * _ANGLE_SCALE),
            )

        if telemetry.has_otos:
            frame.otos = (
                int(telemetry.otos.x),
                int(telemetry.otos.y),
                int(telemetry.otos.h * _ANGLE_SCALE),
            )

        if telemetry.has_twist:
            frame.twist = (
                int(telemetry.twist.v_x),
                int(telemetry.twist.omega * 1000.0),
            )

        return frame


@dataclass
class ParsedResponse:
    """Structured representation of a single response line from the firmware.

    ``tlm`` (097-003): populated only when this ``ParsedResponse`` was
    synthesized by ``NezhaProtocol.stream_drive()`` from a binary-plane
    telemetry push frame (``tag="TLM"``, every other field at its default) --
    ``stream_drive()`` arms streaming via ``stream()``, which is binary-only
    after this ticket, so no text ``TLM ...`` line is ever produced for it to
    parse (``raw``/``tokens``/``kv`` stay empty for a binary-sourced frame).
    Every text-sourced ``ParsedResponse`` (``OK``/``ERR``/``EVT``/...) leaves
    ``tlm`` at its default ``None``.
    """
    tag: str          # "OK", "ERR", "EVT", "TLM", "CFG", "ID"
    tokens: list[str] = field(default_factory=list)  # plain tokens after tag
    kv: dict[str, str] = field(default_factory=dict) # key=value pairs
    corr_id: str | None = None                       # trailing #<id>, if any
    raw: str = ""                                    # original stripped line
    tlm: "TLMFrame | None" = None                     # 097-003: binary-sourced frame, if any


# ---------------------------------------------------------------------------
# Stop clause builder
# ---------------------------------------------------------------------------

class Stop:
    """Builder for stop= clause tokens sent with motion commands.

    Each class method returns a formatted stop= string that can be passed
    in the stop=[...] list argument to motion command methods (vw, drive,
    arc, timed, distance, turn).

    Grammar matches the firmware mc_parseStopToken dispatch table:
      stop=t:<ms>
      stop=d:<mm>
      stop=line:<ge|le>:<thr>
      stop=sensor:<ch>:<ge|le>:<thr>
      stop=color:<h>:<s>:<v>:<dist>
      stop=heading:<heading>:<eps>  (cdeg, cdeg)
      stop=rot:<arc>  (mm)
    """

    @classmethod
    def time(cls, duration: int) -> str:  # [ms]
        """Stop after ``duration`` milliseconds."""
        return f"stop=t:{duration}"

    @classmethod
    def dist(cls, distance: int) -> str:  # [mm]
        """Stop after ``distance`` millimetres of travel."""
        return f"stop=d:{distance}"

    @classmethod
    def line(cls, cmp: str, threshold: int) -> str:
        """Stop when the line sensor crosses the threshold.

        Args:
            cmp: ``'ge'`` (>=) or ``'le'`` (<=).
            threshold: Raw sensor count.
        """
        return f"stop=line:{cmp}:{threshold}"

    @classmethod
    def sensor(cls, channel: str, cmp: str, threshold: int) -> str:
        """Stop when a named sensor channel crosses the threshold.

        Args:
            channel: One of line0–line3, colorR, colorG, colorB, colorC,
                     analogIn0–analogIn3.
            cmp: ``'ge'`` (>=) or ``'le'`` (<=).
            threshold: Raw sensor count.
        """
        return f"stop=sensor:{channel}:{cmp}:{threshold}"

    @classmethod
    def color(cls, h: float, s: float, v: float, dist: float) -> str:
        """Stop when the color sensor matches (h, s, v) within ``dist``."""
        return f"stop=color:{h}:{s}:{v}:{dist}"

    @classmethod
    def heading(cls, heading: int, eps: int) -> str:  # [cdeg]
        """Stop when the robot reaches heading ``heading`` ± ``eps`` (centi-degrees)."""
        return f"stop=heading:{heading}:{eps}"

    @classmethod
    def rot(cls, arc_length: int) -> str:  # [mm]
        """Stop after ``arc_length`` millimetres of arc travel."""
        return f"stop=rot:{arc_length}"


# ---------------------------------------------------------------------------
# Module-level parse functions (can be used without a NezhaProtocol instance)
# ---------------------------------------------------------------------------

_RESPONSE_TAGS = frozenset(("OK", "ERR", "EVT", "TLM", "CFG", "ID"))


def _strip_relay(line: str) -> str:
    """Strip relay prefix characters and surrounding whitespace."""
    return line.strip().lstrip("<# ").strip()


def parse_response(line: str) -> ParsedResponse | None:
    """Parse one v2 response line into a ParsedResponse, or None if unrecognised.

    Handles relay prefix stripping, optional trailing '#<id>' correlation token,
    and key=value pair extraction.
    """
    s = _strip_relay(line)
    if not s:
        return None

    parts = s.split()
    if not parts:
        return None

    tag = parts[0].upper()
    if tag not in _RESPONSE_TAGS:
        return None

    rest = parts[1:]

    # Extract trailing corr_id: '#' followed by digits only.
    corr_id: str | None = None
    if rest and rest[-1].startswith("#") and rest[-1][1:].isdigit():
        corr_id = rest[-1][1:]
        rest = rest[:-1]

    # Parse key=value pairs; remainder are plain positional tokens.
    kv: dict[str, str] = {}
    plain: list[str] = []
    for tok in rest:
        if "=" in tok and not tok.startswith("="):
            k, _, v = tok.partition("=")
            kv[k] = v
        else:
            plain.append(tok)

    return ParsedResponse(
        tag=tag,
        tokens=plain,
        kv=kv,
        corr_id=corr_id,
        raw=s,
    )


def tlm_drop_rate(frames: "list[TLMFrame]") -> float:
    """Estimate the TLM frame drop rate from a sequence of TLMFrame objects.

    Uses the ``seq`` field (D10, firmware 028-005+) to detect gaps.  The
    uint16 seq counter wraps at 65535; wrap-around is handled correctly.

    Returns the fraction of expected sequence numbers that are absent:
      0.0 — no drops detected (or fewer than 2 frames, or no seq fields).
      1.0 — every possible intermediate frame was dropped.

    Returns 0.0 for fewer than 2 frames or when all ``seq`` fields are None
    (pre-D10 firmware).

    Args:
        frames: List of TLMFrame objects (in order received).
    """
    seq_frames = [f for f in frames if f.seq is not None]
    if len(seq_frames) < 2:
        return 0.0

    expected_span = 0
    drops = 0
    for i in range(1, len(seq_frames)):
        prev = seq_frames[i - 1].seq
        curr = seq_frames[i].seq
        # Gap accounting with uint16 wrap-around (modulo 65536).
        gap = (curr - prev) & 0xFFFF  # type: ignore[operator]
        expected_span += gap
        if gap > 1:
            drops += gap - 1

    if expected_span == 0:
        return 0.0
    return drops / expected_span


# ---------------------------------------------------------------------------
# Config key <-> binary target/field mapping (097-002, M2 NezhaProtocol Core
# Conversion). NezhaProtocol.get_config()/.set_config() keep their text-plane
# **kwargs/*keys signature -- a flat "wire key" vocabulary -- but the binary
# plane's ConfigDelta/ConfigGet/ConfigSnapshot (config.proto, 096-001) are
# typed, per-SLICE Patch messages, not a generic key/value map. This table is
# the translation between the two: it curates the SAME 15 keys
# config_commands.cpp's kAllKeys registers on the text plane (that file's own
# list, transcribed here -- config.proto's own header comment already
# establishes the 1:1 correspondence between kAllKeys and the three curated
# Patch messages, so this map does not invent anything new). A key not in
# this table has no binary wire target at all (mirrors the text plane's own
# ERR badkey -- see get_config()/set_config()'s own docstrings for the
# resulting behavior).
# ---------------------------------------------------------------------------

# tw/rotSlip/ekfQxy/ekfQtheta/ekfROtosXy/ekfROtosTheta -> DrivetrainConfigPatch
# field, config_pb2.CONFIG_DRIVETRAIN.
_DRIVETRAIN_KEYS = {
    "tw": "trackwidth",
    "rotSlip": "rotational_slip",
    "ekfQxy": "ekf_q_xy",
    "ekfQtheta": "ekf_q_theta",
    "ekfROtosXy": "ekf_r_otos_xy",
    "ekfROtosTheta": "ekf_r_otos_theta",
}

# pid.kp/ki/kff/iMax/kaw -> MotorConfigPatch Gains field. Applied to BOTH
# bound motors from a SINGLE patch server-side (handleConfigMotor(),
# binary_channel.cpp -- "any present Gains field applies to BOTH bound
# motors unconditionally... never a per-side Gains split"), so a set_config()
# call needs only ONE motor envelope carrying these, never two. Read back
# from the LEFT motor's snapshot (config_commands.cpp's own
# formatConfigKeyFromBb() comment: "pid.* reads the LEFT bound motor's
# published config").
_MOTOR_PID_KEYS = {
    "pid.kp": "kp",
    "pid.ki": "ki",
    "pid.kff": "kff",
    "pid.iMax": "i_max",
    "pid.kaw": "kaw",
}

# minSpeed -> PlannerConfigPatch.min_speed, config_pb2.CONFIG_PLANNER.
_PLANNER_KEYS = {"minSpeed": "min_speed"}

# ml/mr and sTimeout are handled specially, not via a plain field-name map:
#   - ml/mr both patch MotorConfigPatch.travel_calib, disambiguated by
#     `side` (Decision 5, config.proto) -- ml=LEFT, mr=RIGHT.
#   - sTimeout is ConfigDelta's bare `watchdog` oneof arm (uint32, NOT a
#     message-typed Patch -- Open Question 4, config.proto), routed
#     straight to bb.streamWatchdogWindowIn, never bb.configIn.
_ALL_SET_KEYS = frozenset(
    set(_DRIVETRAIN_KEYS) | set(_MOTOR_PID_KEYS) | set(_PLANNER_KEYS)
    | {"ml", "mr", "sTimeout"})

# get_config()'s target-per-key lookup (which ConfigGet.target a given key's
# CURRENT value is read from). pid.* reads LEFT (see _MOTOR_PID_KEYS above).
_TARGET_FOR_KEY: dict[str, int] = {}
_TARGET_FOR_KEY.update({k: config_pb2.CONFIG_DRIVETRAIN for k in _DRIVETRAIN_KEYS})
_TARGET_FOR_KEY.update({k: config_pb2.CONFIG_MOTOR_LEFT for k in _MOTOR_PID_KEYS})
_TARGET_FOR_KEY["ml"] = config_pb2.CONFIG_MOTOR_LEFT
_TARGET_FOR_KEY["mr"] = config_pb2.CONFIG_MOTOR_RIGHT
_TARGET_FOR_KEY.update({k: config_pb2.CONFIG_PLANNER for k in _PLANNER_KEYS})
_TARGET_FOR_KEY["sTimeout"] = config_pb2.CONFIG_WATCHDOG

# get_config() with no keys: the full dump, in kAllKeys' own order
# (config_commands.cpp) -- not required for correctness (dict order is not a
# documented part of get_config()'s contract) but keeps a printed/iterated
# result stable and diffable against the text plane's own dump order.
_ALL_GET_KEYS = (
    "tw", "ml", "mr",
    "pid.kp", "pid.ki", "pid.kff", "pid.iMax", "pid.kaw",
    "rotSlip",
    "ekfQxy", "ekfQtheta", "ekfROtosXy", "ekfROtosTheta",
    "minSpeed", "sTimeout",
)


def _format_config_value(value: Any) -> str:
    """Format a set_config() kwarg value into the SAME string shape the
    text plane's set_config() already produced -- floats to 6 significant
    digits, everything else via str(). Reused as-is from the pre-097-002
    text implementation (the formatting rule itself did not change)."""
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def _read_config_snapshot_value(key: str, snapshot: "envelope_pb2.ConfigSnapshot") -> str | None:
    """Read one get_config() key's current value out of the ConfigSnapshot
    for the target _TARGET_FOR_KEY[key] names. BinaryChannel's handleGet()
    (binary_channel.cpp) always populates EVERY field of a target's Patch on
    a GET reply (no partial-presence case to handle here -- unlike a SET
    delta, a GET snapshot is a full read of the target's current state)."""
    if key in _DRIVETRAIN_KEYS:
        return _format_config_value(getattr(snapshot.drivetrain, _DRIVETRAIN_KEYS[key]))
    if key in ("ml", "mr"):
        return _format_config_value(snapshot.motor.travel_calib)
    if key in _MOTOR_PID_KEYS:
        return _format_config_value(getattr(snapshot.motor, _MOTOR_PID_KEYS[key]))
    if key in _PLANNER_KEYS:
        return _format_config_value(snapshot.planner.min_speed)
    if key == "sTimeout":
        return str(snapshot.watchdog)
    return None


# ---------------------------------------------------------------------------
# NezhaProtocol
# ---------------------------------------------------------------------------

class NezhaProtocol:
    """Wire protocol v2 adapter for the Nezha firmware.

    Owns a SerialConnection and exposes one method per firmware command group.
    All response parsing delegates to module-level parse_* functions so callers
    can reuse them on lines received through other paths (streaming generators).

    v2 protocol rules:
    - Commands are whitespace-separated tokens, verb upper-cased only.
    - Integer values are literal mm (no implicit scaling, no sign prefix).
    - Optional trailing '#<id>' for request/response correlation.
    - Response tags: OK, ERR, EVT, TLM, CFG, ID.
    """

    def __init__(self, conn: SerialConnection) -> None:
        self._conn = conn

    # ------------------------------------------------------------------
    # Connection delegation
    # ------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._conn.is_open

    @property
    def mode(self) -> str | None:
        return self._conn.mode

    def _send_envelope(self, envelope: "envelope_pb2.CommandEnvelope",
                       read_timeout: int = 500,  # [ms]
                       ) -> "envelope_pb2.ReplyEnvelope | None":
        """Send ``envelope``; return the decoded ``ReplyEnvelope`` (or
        ``None`` on timeout/not-connected), normalizing the two different
        ``send_envelope()`` return shapes this tree's two connection
        backends use.

        ``SerialConnection.send_envelope()`` (``robot_radio/io/
        serial_conn.py``) returns a dict --
        ``{"sent": ..., "mode": ..., "reply": ReplyEnvelope | None}`` --
        because a real serial link's request/reply is genuinely
        asynchronous (a background reader thread fills a corr-id-keyed
        queue that could just as easily be filled by an unrelated frame
        first). ``SimConnection.send_envelope()`` (``robot_radio/io/
        sim_conn.py``, 097) returns the decoded ``ReplyEnvelope`` (or
        ``None``) DIRECTLY -- its own docstring explains why: the sim call
        is already synchronous (one in-process C call, no interleaving is
        possible), so there is no dict wrapper to build. Every
        ``NezhaProtocol`` method below sends over EITHER connection type
        (``testgui.binary_bridge`` — 097's TestGUI transport migration —
        constructs one ``NezhaProtocol`` per connection, hardware or sim,
        and routes every command through it), so this is the ONE place
        that reconciles the two shapes rather than each call site
        special-casing ``isinstance(result, dict)`` itself.
        """
        result = self._conn.send_envelope(envelope, read_timeout=read_timeout)
        if isinstance(result, dict):
            return result.get("reply")
        return result

    def send(self, cmd: str, read_timeout: int = 500) -> dict:  # [ms]
        """Send a v2 command, return raw response dict (for ad-hoc / pass-through)."""
        return self._conn.send(cmd, read_timeout)

    def send_fast(self, cmd: str) -> None:
        """Fire-and-forget send with no response reading."""
        self._conn.send_fast(cmd)

    def read_lines(self, duration: int) -> list[str]:  # [ms]
        """Blocking read for up to duration milliseconds."""
        return self._conn.read_lines(duration)

    def read_pending_lines(self) -> list[str]:
        """Drain the pending queues without blocking."""
        return self._conn.read_pending_lines()

    # ------------------------------------------------------------------
    # Static parse helpers (reusable on raw lines from streaming callers)
    # ------------------------------------------------------------------

    @staticmethod
    def parse_response(line: str) -> ParsedResponse | None:
        """Parse a v2 response line. Delegates to module-level parse_response()."""
        return parse_response(line)

    # ------------------------------------------------------------------
    # Liveness / identity
    # ------------------------------------------------------------------

    def ping(self, corr_id: str | None = None) -> tuple[int, float] | None:
        """Send PING, parse the reply's robot-clock timestamp (ms).

        Returns (t_robot, rtt) or None if no valid response, both in ms.
        rtt is the round-trip time measured by this call.

        Binary implementation (097-002, M2 NezhaProtocol Core Conversion):
        CommandEnvelope{ping: Ping{}} via send_envelope() -- the Ack reply's
        `t` field (set only by BinaryChannel's ping arm, mirroring text
        PING's `OK pong t=<ms>`) is `t_robot`. `corr_id` is accepted for
        signature compatibility but has no binary wire home -- the
        envelope's own corr_id field is fully owned/overwritten by
        SerialConnection.send_envelope() for reply routing, unlike the text
        plane's independent trailing '#<id>' token. No caller in this tree
        passes it today (grep-verified); contract (return type/shape)
        unchanged.
        """
        envelope = envelope_pb2.CommandEnvelope(ping=envelope_pb2.Ping())
        t0 = time.monotonic()
        reply = self._send_envelope(envelope, read_timeout=500)
        t1 = time.monotonic()
        rtt = (t1 - t0) * 1000.0  # [ms]

        if reply is not None and reply.WhichOneof("body") == "ok":
            return (int(reply.ok.t), rtt)
        return None

    def echo(self, payload: str) -> str | None:
        """Send ECHO <payload>, return echoed payload string or None.

        Binary implementation (097-002): CommandEnvelope{echo: Echo{payload}}
        via send_envelope(); the reply's echo arm carries the payload back
        verbatim (envelope.proto's own "reuse the request-side message on
        the reply side" pattern). Contract (return type/shape) unchanged.
        """
        envelope = envelope_pb2.CommandEnvelope(
            echo=envelope_pb2.Echo(payload=payload.encode("utf-8")))
        reply = self._send_envelope(envelope, read_timeout=500)
        if reply is not None and reply.WhichOneof("body") == "echo":
            return reply.echo.payload.decode("utf-8")
        return None

    def get_id(self) -> dict[str, str] | None:
        """Send ID command. Returns kv dict (model, name, serial, fw, proto, caps) or None.

        Binary implementation (097-002): CommandEnvelope{id: DeviceId{}}
        (empty request, envelope.proto Decision 4) via send_envelope(); the
        reply's DeviceId fields (model/name/serial/fw_version/proto_version)
        map onto the SAME kv keys the text ID reply's `handleId()`
        (system_commands.cpp) emits today -- `caps=` is not emitted by
        either plane (that field was dropped pre-097; see handleId()'s own
        comment). Contract (return type/shape) unchanged.
        """
        envelope = envelope_pb2.CommandEnvelope(id=envelope_pb2.DeviceId())
        reply = self._send_envelope(envelope, read_timeout=500)
        if reply is not None and reply.WhichOneof("body") == "id":
            d = reply.id
            return {
                "model": d.model,
                "name": d.name,
                "serial": str(d.serial),
                "fw": d.fw_version,
                "proto": str(d.proto_version),
            }
        return None

    def get_ver(self) -> dict[str, str] | None:
        """Send VER command. Returns kv dict (fw, proto) or None.

        Binary implementation (097-002): VER's content is a strict SUBSET of
        ID's reply -- no independent binary `ver` arm exists or is added
        (architecture-update.md (097) M2). Sends the SAME CommandEnvelope{id:
        DeviceId{}} get_id() does and reads only fw_version/proto_version off
        the reply. Contract (return type/shape) unchanged.
        """
        envelope = envelope_pb2.CommandEnvelope(id=envelope_pb2.DeviceId())
        reply = self._send_envelope(envelope, read_timeout=500)
        if reply is not None and reply.WhichOneof("body") == "id":
            return {"fw": reply.id.fw_version, "proto": str(reply.id.proto_version)}
        return None

    def get_help(self) -> str | None:
        """Send HELP. Returns the verb-list string or None."""
        resp_dict = self._conn.send("HELP", read_timeout=500)
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "help":
                return " ".join(r.tokens[1:])
        return None

    # ------------------------------------------------------------------
    # Config: GET / SET
    # ------------------------------------------------------------------

    def get_config(self, *keys: str) -> dict[str, str] | None:
        """Send GET [keys...], parse CFG response into key->value dict.

        With no keys, returns the full config dump (all registered keys).
        Returns None if no CFG line was received.

        Binary implementation (097-002): thin wrapper over get_config_binary()
        (096-007). The text plane's single "GET [keys]" line becomes one
        get_config_binary() round trip PER DISTINCT ConfigTarget the
        requested keys span (module-level _TARGET_FOR_KEY) -- ConfigGet only
        names one target per request (config.proto), unlike the text plane's
        single free-form key list, so a multi-target request (or the full,
        no-args dump, which spans all 5 targets) costs multiple round trips.
        A key outside the module-level _ALL_SET_KEYS vocabulary has no
        binary wire target -- returns None (mirrors the text plane's own ERR
        badkey producing no CFG line). A target whose round trip times out
        contributes no keys to the result (best-effort merge, matching the
        text plane's own "merge every CFG line received" behavior across a
        multi-line dump) rather than failing the whole call.
        """
        requested = keys if keys else _ALL_GET_KEYS
        if any(k not in _TARGET_FOR_KEY for k in requested):
            return None

        targets = sorted({_TARGET_FOR_KEY[k] for k in requested})
        snapshots: dict[int, envelope_pb2.ConfigSnapshot] = {}
        for target in targets:
            snapshot = self.get_config_binary(target)
            if snapshot is not None:
                snapshots[target] = snapshot

        result: dict[str, str] = {}
        for key in requested:
            snapshot = snapshots.get(_TARGET_FOR_KEY[key])
            if snapshot is None:
                continue
            value = _read_config_snapshot_value(key, snapshot)
            if value is not None:
                result[key] = value
        return result if result else None

    def set_config(self, **kwargs: Any) -> dict[str, str] | None:
        """Send SET key=value ..., parse OK set response.

        Returns dict of applied keys (from OK set response) or None.
        Floats are formatted with up to 6 significant digits.

        Binary implementation (097-002): thin wrapper over set_config_binary()
        (096-007). Unlike the text plane's single atomic SET line, a
        ConfigDelta's oneof carries only ONE Patch at a time (config.proto),
        so kwargs spanning multiple targets (e.g. tw= + sTimeout=) become
        MULTIPLE set_config_binary() round trips, one per touched target --
        NOT atomic across targets the way the single text SET line was
        (flagged, not silently reconciled, per this project's "transcribe,
        never re-derive; flag genuine gaps" discipline: a true cross-target
        atomic SET is not achievable without new binary wire capability,
        out of this sprint's scope). Any kwarg key outside the module-level
        _ALL_SET_KEYS vocabulary fails the WHOLE call (returns None, no wire
        traffic at all) -- mirrors the text plane's own atomic-SET "one bad
        key rejects the whole line" posture (config_commands.cpp's own file
        header). If every touched target's round trip Acks, the returned
        dict echoes the kwargs actually sent (formatted the same way the
        pre-097-002 text implementation formatted them) -- the binary Ack
        carries no per-key echo the way the text "OK set ..." reply did, so
        this is the closest same-shape substitute, not a wire round trip of
        the applied value.
        """
        if not kwargs:
            return None
        if any(k not in _ALL_SET_KEYS for k in kwargs):
            return None

        drivetrain_patch: dict[str, float] = {}
        motor_left_patch: dict[str, float] = {}
        motor_right_patch: dict[str, float] = {}
        planner_patch: dict[str, float] = {}
        watchdog_value: int | None = None

        for key, value in kwargs.items():
            if key in _DRIVETRAIN_KEYS:
                drivetrain_patch[_DRIVETRAIN_KEYS[key]] = float(value)
            elif key == "ml":
                motor_left_patch["travel_calib"] = float(value)
            elif key == "mr":
                motor_right_patch["travel_calib"] = float(value)
            elif key in _MOTOR_PID_KEYS:
                # Applied to BOTH bound motors server-side from ONE patch
                # (handleConfigMotor(), binary_channel.cpp) -- carried on the
                # LEFT envelope only; see _MOTOR_PID_KEYS' own comment.
                motor_left_patch[_MOTOR_PID_KEYS[key]] = float(value)
            elif key in _PLANNER_KEYS:
                planner_patch[_PLANNER_KEYS[key]] = float(value)
            elif key == "sTimeout":
                watchdog_value = int(value)

        ok = True
        if drivetrain_patch:
            delta = envelope_pb2.ConfigDelta(
                drivetrain=config_pb2.DrivetrainConfigPatch(**drivetrain_patch))
            if self.set_config_binary(delta) is None:
                ok = False
        if motor_left_patch:
            delta = envelope_pb2.ConfigDelta(motor=config_pb2.MotorConfigPatch(
                side=config_pb2.LEFT, **motor_left_patch))
            if self.set_config_binary(delta) is None:
                ok = False
        if motor_right_patch:
            delta = envelope_pb2.ConfigDelta(motor=config_pb2.MotorConfigPatch(
                side=config_pb2.RIGHT, **motor_right_patch))
            if self.set_config_binary(delta) is None:
                ok = False
        if planner_patch:
            delta = envelope_pb2.ConfigDelta(
                planner=config_pb2.PlannerConfigPatch(**planner_patch))
            if self.set_config_binary(delta) is None:
                ok = False
        if watchdog_value is not None:
            delta = envelope_pb2.ConfigDelta(watchdog=watchdog_value)
            if self.set_config_binary(delta) is None:
                ok = False

        if not ok:
            return None
        return {key: _format_config_value(value) for key, value in kwargs.items()}

    # ------------------------------------------------------------------
    # Config: binary GET / SET (096-007, M6 Host Config/Telemetry Client)
    #
    # Alongside the text SET/GET wrappers above, NOT a replacement for
    # them — same public-API-stability posture 095 established for
    # drive/segment/replace (this class's existing methods/signatures are
    # untouched; these are new, additive envelope builders). Both build a
    # CommandEnvelope and hand it to SerialConnection.send_envelope() (095's
    # corr-id-correlated binary round trip), then unwrap the ONE reply arm
    # each request can produce — mirroring BinaryChannel's own CONFIG/GET
    # arms (source/commands/binary_channel.cpp) exactly: CONFIG replies
    # Ack on success, GET replies exactly one ConfigSnapshot. Neither
    # method distinguishes a rejected (``Error``) reply from a plain
    # timeout/not-connected — both return None, matching get_config()/
    # set_config()'s own above "no failure detail" posture on the text
    # plane.
    # ------------------------------------------------------------------

    def set_config_binary(self, delta: "envelope_pb2.ConfigDelta",
                          read_timeout: int = 500) -> "envelope_pb2.Ack | None":  # [ms]
        """Send CommandEnvelope{config: delta}; return the Ack reply, or
        None (timeout, not connected, or an Error reply).

        ``delta`` is a fully-built ``pb2.ConfigDelta`` — its own oneof
        (``drivetrain``/``motor``/``planner``/``watchdog``) selects which
        config slice is being patched, mirroring BinaryChannel's CONFIG arm
        1:1. Building ``delta`` is the caller's job (e.g.
        ``envelope_pb2.ConfigDelta(drivetrain=config_pb2.
        DrivetrainConfigPatch(trackwidth=128.0))``) — this method's only
        job is the envelope round trip.
        """
        envelope = envelope_pb2.CommandEnvelope(config=delta)
        reply = self._send_envelope(envelope, read_timeout=read_timeout)
        if reply is not None and reply.WhichOneof("body") == "ok":
            return reply.ok
        return None

    def get_config_binary(self, target: int,
                          read_timeout: int = 500) -> "envelope_pb2.ConfigSnapshot | None":  # [ms]
        """Send CommandEnvelope{get: ConfigGet{target}}; return the
        ConfigSnapshot reply (exactly one slice, per BinaryChannel's GET
        arm), or None (timeout, not connected, or an Error reply).

        ``target`` is one of ``config_pb2.CONFIG_DRIVETRAIN`` /
        ``CONFIG_MOTOR_LEFT`` / ``CONFIG_MOTOR_RIGHT`` / ``CONFIG_PLANNER``
        / ``CONFIG_WATCHDOG``.
        """
        envelope = envelope_pb2.CommandEnvelope(get=envelope_pb2.ConfigGet(target=target))
        reply = self._send_envelope(envelope, read_timeout=read_timeout)
        if reply is not None and reply.WhichOneof("body") == "cfg":
            return reply.cfg
        return None

    # ------------------------------------------------------------------
    # Drive commands
    # ------------------------------------------------------------------

    def stop(self) -> None:
        """Stop motors immediately (STOP command).

        Binary implementation (097-002): CommandEnvelope{stop: Stop{}} via
        send_envelope() -- a zero-field oneof arm that "cannot be
        malformed" (envelope.proto Decision 3), byte-identical in spirit to
        the text handleStop()'s own NEUTRAL/BRAKE construction. The STOP
        bytes reach the wire immediately (send_envelope()'s write happens
        before it blocks on the reply queue) -- no less safety-responsive
        than the prior fire-and-forget send_fast("STOP"); only the Python
        call itself now blocks briefly (<=300ms) for the discarded Ack.
        Return type (None) unchanged.
        """
        envelope = envelope_pb2.CommandEnvelope(stop=envelope_pb2.Stop())
        self._conn.send_envelope(envelope, read_timeout=300)

    def cancel(self) -> None:
        """Cancel the active motion command (hard stop). Sends X."""
        self._conn.send_fast("X")

    def arc(self, speed: int, radius: int,  # [mm/s], [mm]
            corr_id: str | None = None,
            stop: list[str] | None = None) -> None:
        """Send R arc command — sets body arc motion (open-ended, no built-in timeout).

        Format: R <speed> <radius> [stop=<kind>:<args> ...] [#id]
        - ``speed``: forward speed in mm/s (−1000 … +1000).
        - ``radius``: arc radius in mm (−10000 … +10000; 0 = straight).
          **Sign convention: positive radius ⇒ CCW (left arc).**
          Matches BodyKinematics::inverse where CCW-positive ω gives vL < vR.
        - ``corr_id``: optional correlation id; echoed in EVT done R.
        - ``stop``: optional list of stop= clause strings from the Stop builder.

        Uses fire-and-forget (send_fast). The arc runs until the host sends X
        (hard cancel) or R 0 <r> (speed=0 triggers SOFT ramp-down + EVT done R).
        To use as a keepalive-driven command, re-send within the firmware sTimeout
        window; the firmware does NOT have a built-in keepalive watchdog for R.

        Robot replies ``OK arc speed=… radius=…`` synchronously. On soft-stop
        (speed=0), the firmware emits ``EVT done R`` asynchronously.
        """
        cmd = f"R {speed} {radius}"
        if stop:
            cmd += " " + " ".join(stop)
        if corr_id is not None:
            cmd += f" #{corr_id}"
        self._conn.send_fast(cmd)

    def vw(self, v: int, omega: int,  # [mm/s], [mrad/s]
           corr_id: str | None = None,
           stop: list[str] | None = None) -> None:
        """Send a VW command — sets body-twist velocity, resets system watchdog.

        Format: VW <v> <omega> [stop=<kind>:<args> ...] [#id]
        - ``v``: forward speed in mm/s (−1000 … +1000).
        - ``omega``: yaw rate in milli-radians/s (−3142 … +3142).
          Positive = CCW (left turn).
        - ``corr_id``: optional correlation id; echoed in EVT safety_stop.
        - ``stop``: optional list of stop= clause strings from the Stop builder.

        Uses fire-and-forget (send_fast) so it can be called at streaming
        rate without blocking.  The firmware echoes ``OK vw v=… omega=…``
        synchronously, but callers driving at high frequency typically ignore
        the per-frame reply.

        **Do not use VW as a keepalive during non-VW commands (TURN, G, T,
        D, R, RT).**  Since firmware 027-003, the firmware detects an active
        non-VW command and replies ``OK vw busy=<origin>`` without updating
        the command target, so a ``VW 0 0`` keepalive will NOT reset the
        watchdog for those commands.  Non-VW commands have a built-in TIME
        stop net and do not require keepalives.
        """
        cmd = f"VW {v} {omega}"
        if stop:
            cmd += " " + " ".join(stop)
        if corr_id is not None:
            cmd += f" #{corr_id}"
        self._conn.send_fast(cmd)

    def drive(self, left: int, right: int,  # [mm/s]
              stop: list[str] | None = None) -> None:
        """Send an S streaming command — sets streaming wheel speeds, resets watchdog.

        Format: S <l> <r> [stop=<kind>:<args> ...]  (space-separated integers, literal mm/s)
        - ``stop``: optional list of stop= clause strings from the Stop builder.

        **Do not use S as a keepalive during non-VW commands (TURN, G, T,
        D, R, RT).**  S converts to a VW command internally; since firmware
        027-003 the firmware detects an active non-VW command and replies
        ``OK vw busy=<origin>`` without updating the command target.  Non-VW
        commands have a built-in TIME stop net and do not require keepalives.

        Binary implementation (097-002): CommandEnvelope{drive:
        DrivetrainCommand{wheels}} via send_envelope(), the per-wheel-speed
        target legacy_translate.wheel_targets_for_drive() builds (handleS()'s
        own construction, transcribed). ``stop`` has no binary wire home --
        WheelTargets/DrivetrainCommand carry no stop-clause capability, and
        the CURRENT text S handler (parseS(), motion_commands.cpp, 093-001)
        already rejects any stop=/sensor= kv outright with ERR badarg (no
        motor effect) -- and drive()'s prior fire-and-forget send_fast()
        never read that ERR reply either, so passing ``stop`` was ALREADY a
        silent no-motor-effect call before this conversion. This binary
        implementation preserves that "no motor effect" outcome by sending
        no envelope at all when ``stop`` is given, rather than silently
        starting to drive (which the old text-plane behavior never did
        either) -- kept as an unused-but-signature-compatible parameter.
        """
        if stop:
            return
        wheels = legacy_translate.wheel_targets_for_drive(left, right)
        envelope = envelope_pb2.CommandEnvelope(
            drive=drivetrain_pb2.DrivetrainCommand(wheels=wheels))
        self._conn.send_envelope(envelope, read_timeout=300)

    def timed(self, left: int, right: int,  # [mm/s]
             duration: int,  # [ms]
             sensor: str | None = None,
             stop: list[str] | None = None) -> list[str]:
        """Send T command; return initial response lines.

        Format: T <l> <r> <ms> [sensor=<ch>:<op>:<thr>] [stop=<kind>:<args> ...]
        Robot replies OK drive ...; later sends EVT done T.

        Optional ``sensor`` modifier stops the drive early when a sensor crosses
        a threshold.  Format: ``"<ch>:<op>:<thr>"`` where ch ∈ line0–line3,
        colorR/G/B/C; op ∈ ge|le; thr is an integer raw ADC count.
        Example: sensor="line0:ge:512"

        Optional ``stop`` is a list of stop= clause strings from the Stop builder.
        Multiple conditions are appended space-separated before any '#id'.

        Binary implementation (097-002): CommandEnvelope{segment:
        MotionSegment} via send_envelope(), built by
        legacy_translate.segment_for_timed() (handleT()'s own l/r-sign-then-
        distance computation via BodyKinematics::forward(), transcribed).
        ``sensor``/``stop`` are accepted but inert on BOTH planes today --
        the CURRENT handleT() (motion_commands.cpp, post-093/094) never
        reads past args[0..2] (l/r/ms), so a text-plane sensor=/stop= token
        was already silently ignored before this conversion; MotionSegment
        has no matching field either. Returns a synthesized single-line list
        (``["OK drive ..."]`` on Ack, ``[]`` on timeout/error) reproducing
        the pre-conversion contract's SHAPE (a list of response-line
        strings) -- no caller in this tree inspects the actual line text
        (grep-verified).
        """
        seg = legacy_translate.segment_for_timed(left, right, duration)
        envelope = envelope_pb2.CommandEnvelope(segment=seg)
        reply = self._send_envelope(envelope, read_timeout=300)
        if reply is not None and reply.WhichOneof("body") == "ok":
            return [f"OK drive l={left} r={right} ms={duration} "
                    f"q={reply.ok.q} rem={reply.ok.rem:.1f}"]
        return []

    def distance(self, left: int, right: int,  # [mm/s]
                travel: int,  # [mm]
                sensor: str | None = None,
                stop: list[str] | None = None) -> list[str]:
        """Send D command; return initial response lines.

        Format: D <l> <r> <mm> [sensor=<ch>:<op>:<thr>] [stop=<kind>:<args> ...]
        Robot replies OK drive ...; later sends EVT done D.

        Optional ``sensor`` modifier stops the drive early when a sensor crosses
        a threshold.  Format: ``"<ch>:<op>:<thr>"`` (same as timed()).
        Example: sensor="colorC:ge:800"

        Optional ``stop`` is a list of stop= clause strings from the Stop builder.

        Binary implementation (097-002): CommandEnvelope{segment:
        MotionSegment} via send_envelope(), built by
        legacy_translate.segment_for_distance() (handleD()'s own
        sign-then-distance computation via BodyKinematics::forward(),
        transcribed). ``sensor``/``stop`` are accepted but inert on BOTH
        planes today, same reasoning as timed() above. Return-value shape
        unchanged (synthesized single-line list on Ack, [] on timeout/error;
        same "no caller inspects the text" note as timed()).
        """
        seg = legacy_translate.segment_for_distance(left, right, travel)
        envelope = envelope_pb2.CommandEnvelope(segment=seg)
        reply = self._send_envelope(envelope, read_timeout=300)
        if reply is not None and reply.WhichOneof("body") == "ok":
            return [f"OK drive l={left} r={right} mm={travel} "
                    f"q={reply.ok.q} rem={reply.ok.rem:.1f}"]
        return []

    def go_to(self, x: int, y: int,  # [mm]
              speed: int) -> list[str]:  # [mm/s]
        """Send G go-to command; return initial response lines.

        Format: G <x> <y> <speed>
        Robot replies OK goto ...; later sends EVT done G.
        """
        resp = self._conn.send(f"G {x} {y} {speed}", read_timeout=300)
        return resp.get("responses", [])

    def turn(self, heading: int, eps: int | None = None,  # [cdeg]
             corr_id: str | None = None,
             sensor: str | None = None,
             stop: list[str] | None = None) -> list[str]:
        """Send TURN command — rotate to an absolute heading and stop within eps.

        Format: TURN <heading> [eps=<cdeg>] [sensor=<ch>:<op>:<thr>]
                     [stop=<kind>:<args> ...] [#id]
        - ``heading``: target heading in centidegrees (−18000 … +18000 = ±180°).
          Positive values are CCW (matches OTOS CCW convention).
        - ``eps``: optional tolerance in centidegrees (default 300 = 3°;
          range 10–1800). Pass a tighter value for calibration use (e.g. 100 = 1°).
        - ``sensor``: optional early-stop modifier; format ``"<ch>:<op>:<thr>"``
          (same as timed() / distance()). Example: sensor="line0:ge:512"
        - ``corr_id``: optional correlation id; echoed in EVT done TURN.
        - ``stop``: optional list of stop= clause strings from the Stop builder.

        Robot replies ``OK turn heading=<cdeg> eps=<cdeg>`` synchronously.
        On arrival within eps (or sensor trip): ``EVT done TURN [#<id>]`` emitted async.

        To wait for completion, use ``wait_for_evt_done("TURN", timeout)``.
        Example::

            proto.turn(9000, eps=100, corr_id="1")  # turn to +90° (CCW), 1° eps
            result, reason = proto.wait_for_evt_done("TURN", timeout=10000, corr_id="1")
        """
        cmd = f"TURN {heading}"
        if eps is not None:
            cmd += f" eps={eps}"
        if sensor is not None:
            cmd += f" sensor={sensor}"
        if stop:
            cmd += " " + " ".join(stop)
        if corr_id is not None:
            cmd += f" #{corr_id}"
        resp = self._conn.send(cmd, read_timeout=300)
        return resp.get("responses", [])

    def drive_until_sensor(self, left: int, right: int,  # [mm/s]
                           duration: int,  # [ms]
                           channel: str, threshold: int,
                           op: str = "ge") -> list[str]:
        """Drive timed until a sensor crosses a threshold (or duration expires).

        Convenience wrapper around T with a ``sensor=`` modifier.  The drive stops
        at whichever comes first: the sensor condition or the time limit.

        Args:
            left:   Left wheel speed in mm/s (−1000 … +1000).
            right:  Right wheel speed in mm/s (−1000 … +1000).
            duration: Maximum duration in ms (1 … 30000). Acts as a safety timeout.
            channel:    Sensor channel name: line0–line3, colorR, colorG, colorB, colorC.
            threshold:  Integer threshold in raw sensor units (uint16_t ADC counts).
            op:         Comparison operator: "ge" (≥, default) or "le" (≤).

        Returns:
            Initial response lines from the firmware (OK drive … or ERR …).
            EVT done T is emitted asynchronously; wait with wait_for_evt_done("T").

        Wire format: ``T <left> <right> <duration> sensor=<channel>:<op>:<threshold>``

        Example::

            proto.drive_until_sensor(200, 200, 10000, "line0", 512)
            result, reason = proto.wait_for_evt_done("T", timeout=12000)
            # result is "done" (sensor tripped) or "timeout"; reason is e.g. "sensor" or None
        """
        sensor_token = f"{channel}:{op}:{threshold}"
        return self.timed(left, right, duration, sensor=sensor_token)

    def grip(self, angle: int | None = None) -> int | None:  # [deg]
        """Send GRIP [angle] command. Returns confirmed degree or None.

        Format: GRIP <angle>  or  GRIP (query only)
        Robot replies OK grip deg=<deg>.
        """
        cmd = f"GRIP {angle}" if angle is not None else "GRIP"
        resp = self._conn.send(cmd, read_timeout=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "grip":
                try:
                    return int(r.kv["deg"])
                except (KeyError, ValueError):
                    pass
        return None

    def zero_encoders(self) -> None:
        """Zero encoders (ZERO enc command)."""
        self._conn.send("ZERO enc", read_timeout=200)

    def zero_otos(self) -> None:
        """Zero OTOS pose tracking (ZERO pose command)."""
        self._conn.send("ZERO pose", read_timeout=200)

    def zero_all(self) -> None:
        """Zero both encoders and OTOS pose (ZERO enc pose command)."""
        self._conn.send("ZERO enc pose", read_timeout=200)

    # ------------------------------------------------------------------
    # Telemetry streaming
    # ------------------------------------------------------------------

    def stream(self, period: int) -> None:  # [ms]
        """Set TLM streaming period in ms (0 = off).

        Binary implementation (097-003, M3 NezhaProtocol Telemetry
        Conversion): ``period`` maps 1:1 onto ``CommandEnvelope{stream:
        StreamControl{period, binary: true}}`` via ``send_envelope()`` --
        handleStream()'s own 20ms floor (``binary_channel.cpp``, mirrored
        from ``kStreamFloorMs``) applies firmware-side exactly as it did for
        the text plane, so no host-side clamping is needed here.
        ``binary=true`` selects ``telemetryEmitBinary()`` over the text
        emitter for every periodic frame this stream produces from now on
        -- the only telemetry plane ``stream()``/``snap()`` speak after this
        ticket. The Ack reply is read and discarded (return type ``None``
        unchanged), matching ``stop()``'s "block briefly for the Ack, but
        return nothing" posture.
        """
        envelope = envelope_pb2.CommandEnvelope(
            stream=envelope_pb2.StreamControl(period=period, binary=True))
        self._conn.send_envelope(envelope, read_timeout=300)

    def stream_fields(self, fields: str) -> None:
        """Set TLM streaming with a field subset.

        Format: STREAM fields=enc,pose,line
        ``fields`` is a comma-separated string of field names.
        """
        self._conn.send(f"STREAM fields={fields}", read_timeout=300)

    def read_binary_tlm_frames(self, duration: int) -> "list[TLMFrame]":  # [ms]
        """Block for up to ``duration`` ms, returning every binary telemetry
        frame received during that window as ``TLMFrame`` objects (097-003).

        The binary-plane counterpart of the pre-097-003 text-plane idiom of
        reading ``read_lines(duration)`` and parsing each ``TLM`` line into a
        ``TLMFrame`` -- reads ``SerialConnection.read_binary_tlm()``
        (``_binary_tlm_queue``) instead of ``_tlm_queue``, and adapts each
        raw ``pb2.ReplyEnvelope`` via ``TLMFrame.from_pb2()``.
        """
        return [TLMFrame.from_pb2(reply.tlm)
                for reply in self._conn.read_binary_tlm(duration)]

    def read_pending_binary_tlm_frames(self) -> "list[TLMFrame]":
        """Non-blocking drain of every currently-queued binary telemetry
        frame as ``TLMFrame`` objects (097-003) -- the binary-plane
        counterpart of ``read_pending_lines()``.
        """
        return [TLMFrame.from_pb2(reply.tlm)
                for reply in self._conn.drain_binary_tlm()]

    def snap(self) -> "TLMFrame | None":
        """Request ONE telemetry frame synchronously and return it (parsed).

        Binary implementation (097-003, M3, architecture-update.md Decision
        4): SYNTHESIZED host-side from the existing binary ``stream`` arm --
        no new firmware wire capability is added (096 Open Question 2
        deferred exactly this resolution to 097). There is no binary
        one-shot SNAP arm; instead:

        1. Drain any stale frames already queued in ``_binary_tlm_queue``
           (leftovers from a previous ``stream()``/``snap()`` session), so a
           stale snapshot is never returned.
        2. Arm streaming at the firmware's own 20ms floor via ``stream()``
           (``StreamControl{period=_STREAM_FLOOR_MS, binary=true}``).
        3. Block on ``_binary_tlm_queue`` for up to 400 ms for exactly ONE
           frame.
        4. Disarm via ``stream(0)`` (``StreamControl{period=0,
           binary=true}``), regardless of whether step 3 timed out.

        This costs a two-round-trip latency profile (arm ack, then wait for
        a frame) instead of a true single request/reply -- a documented,
        accepted trade (architecture-update.md (097) Decision 4
        Consequences). Contract (``TLMFrame | None``) unchanged.
        """
        self._conn.drain_binary_tlm()
        self.stream(_STREAM_FLOOR_MS)
        try:
            frames = self._conn.read_binary_tlm(duration=400)
        finally:
            self.stream(0)
        if not frames:
            return None
        return TLMFrame.from_pb2(frames[0].tlm)

    # ------------------------------------------------------------------
    # OTOS sensor
    # ------------------------------------------------------------------

    def otos_init(self) -> None:
        """Enable OTOS signal processing (OI command)."""
        self._conn.send("OI", read_timeout=500)

    def otos_zero(self) -> None:
        """Zero OTOS position to current location (OZ command)."""
        self._conn.send("OZ", read_timeout=200)

    def otos_reset_tracking(self) -> None:
        """Reset OTOS Kalman filters (OR command)."""
        self._conn.send("OR", read_timeout=200)

    def otos_get_position(self) -> tuple[int, int, int] | None:
        """Query OTOS position (OP command). Returns (x, y, heading) or None
        (x, y in mm, heading in cdeg)."""
        resp = self._conn.send("OP", read_timeout=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "pos":
                try:
                    return (int(r.kv["x"]), int(r.kv["y"]), int(r.kv["h"]))
                except (KeyError, ValueError):
                    pass
        return None

    def otos_set_position(self, x: int, y: int,  # [mm]
                          heading: int) -> None:  # [cdeg]
        """Set OTOS world-frame position (OV command) — nudges the RAW OTOS chip
        only; does NOT set the motion controller's pose.  Prefer set_internal_pose
        (SI) for a camera fix.  NOTE: OV writes the chip's raw registers, which
        readTransformed then rotates by the OTOS mount angle (odomYawDeg) — so a
        world (x,y) passed here lands rotated; that mismatch is why OV must not be
        used to anchor the world pose."""
        self._conn.send(f"OV {x} {y} {heading}", read_timeout=300)

    def set_internal_pose(self, x: int, y: int,  # [mm]
                          heading: int) -> None:  # [cdeg]
        """Set the motion controller's onboard pose from an external (camera) fix
        (SI command -> Odometry::setPose).  This writes poseX/poseY/poseHrad — the
        pose getPose/telemetry report and G/D/TURN drive against — so the robot
        tracks in WORLD coordinates.  Heading is centi-degrees in the camera world
        frame (0 = +x/east, CCW-positive)."""
        self._conn.send(f"SI {x} {y} {heading}", read_timeout=300)

    def otos_set_linear_scalar(self, val: int) -> int | None:
        """Set OTOS linear scalar (OL <val> command). Returns confirmed value or None."""
        resp = self._conn.send(f"OL {val}", read_timeout=500)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "linear":
                try:
                    return int(r.kv["scalar"])
                except (KeyError, ValueError):
                    pass
        return None

    def otos_get_linear_scalar(self) -> int | None:
        """Read back OTOS linear scalar (OL no-arg command). Returns value or None."""
        resp = self._conn.send("OL", read_timeout=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "linear":
                try:
                    return int(r.kv["scalar"])
                except (KeyError, ValueError):
                    pass
        return None

    def otos_set_angular_scalar(self, val: int) -> int | None:
        """Set OTOS angular scalar (OA <val> command). Returns confirmed value or None."""
        resp = self._conn.send(f"OA {val}", read_timeout=500)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "angular":
                try:
                    return int(r.kv["scalar"])
                except (KeyError, ValueError):
                    pass
        return None

    def otos_get_angular_scalar(self) -> int | None:
        """Read back OTOS angular scalar (OA no-arg command). Returns value or None."""
        resp = self._conn.send("OA", read_timeout=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "angular":
                try:
                    return int(r.kv["scalar"])
                except (KeyError, ValueError):
                    pass
        return None

    # ------------------------------------------------------------------
    # J-port I/O
    # ------------------------------------------------------------------

    def port_read(self, port: int) -> int | None:
        """Read digital J-port (P <port> command). Returns 0/1 or None."""
        resp = self._conn.send(f"P {port}", read_timeout=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "port":
                try:
                    return int(r.kv["v"])
                except (KeyError, ValueError):
                    pass
        return None

    def port_write(self, port: int, value: bool) -> None:
        """Write digital J-port (P <port> <val> command)."""
        self._conn.send(f"P {port} {1 if value else 0}", read_timeout=200)

    def port_read_analog(self, port: int) -> int | None:
        """Read analog J-port (PA <port> command). Returns 0-1023 or None."""
        resp = self._conn.send(f"PA {port}", read_timeout=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "aport":
                try:
                    return int(r.kv["v"])
                except (KeyError, ValueError):
                    pass
        return None

    def port_write_analog(self, port: int, value: int) -> None:
        """Write PWM (0-1023) to J-port (PA <port> <val> command)."""
        self._conn.send(f"PA {port} {value}", read_timeout=200)

    # ------------------------------------------------------------------
    # Blocking drive helpers (wait for EVT done or safety_stop)
    # ------------------------------------------------------------------

    def wait_for_evt_done(self, verb: str, timeout: int,  # [ms]
                          corr_id: str | None = None) -> tuple[str, str | None]:
        """Block until 'EVT done <verb>' or 'EVT safety_stop' arrives.

        Returns ``(outcome, reason)`` where:
          ``outcome``: ``"done"``, ``"safety_stop"``, or ``"timeout"``.
          ``reason``: the ``reason=`` token from the EVT line, or ``None`` if
                      absent (e.g. pre-052 firmware or EVT safety_stop without
                      ``reason=watchdog``).

        If ``corr_id`` is provided, only EVT lines carrying that id (or bare
        EVT lines without any id) are accepted.  This lets the host distinguish
        completions when multiple correlated drives are in flight.
        """
        deadline = time.time() + timeout / 1000.0
        while time.time() < deadline:
            for raw_line in self._conn.read_lines(duration=100):
                r = parse_response(raw_line)
                if r is None:
                    continue
                if r.tag == "EVT":
                    # When a corr_id filter is specified, skip EVT lines that
                    # carry a *different* id.  Bare EVT lines (r.corr_id None)
                    # are always accepted.
                    if corr_id is not None and r.corr_id is not None:
                        if r.corr_id != corr_id:
                            continue
                    reason = r.kv.get("reason")  # None if absent
                    if r.tokens and r.tokens[0] == "done":
                        # Accept if verb matches or no verb given in EVT.
                        if len(r.tokens) < 2 or r.tokens[1] == verb:
                            return "done", reason
                    elif r.tokens and r.tokens[0] == "safety_stop":
                        return "safety_stop", reason
        return "timeout", None

    # ------------------------------------------------------------------
    # Streaming drive generator
    # ------------------------------------------------------------------

    def stream_drive(
        self,
        speeds: list[int],
        *,
        period: int = 40,  # [ms]
        watchdog: int = 500,  # [ms]
    ) -> Generator[ParsedResponse, None, None]:
        """Streaming drive generator. Yields ParsedResponse for each incoming line.

        Enables TLM streaming on entry, sends S keepalives, disables streaming
        on GeneratorExit. Mutate ``speeds`` in the caller loop to change velocity.
        Ends naturally on EVT safety_stop.

        Args:
            speeds: Mutable [left, right] list (mm/s); mutate to steer.
            period: TLM streaming period in ms.
            watchdog: S keepalive deadline (ms); must re-send within firmware
                watchdog timeout or motors stop.

        097-003: ``stream()`` is binary-only now, so telemetry no longer
        arrives as text ``TLM ...`` lines through ``read_lines()`` -- EVT
        lines (``safety_stop``) still do, unaffected. Each pass also drains
        ``_binary_tlm_queue`` and yields one ``ParsedResponse(tag="TLM",
        tlm=<TLMFrame>)`` per frame (see ``ParsedResponse.tlm``'s own
        docstring) -- callers that used to text-parse ``resp.raw`` when
        ``resp.tag == "TLM"`` now read ``resp.tlm`` instead; the frame is
        already parsed, not re-derived from text.
        """
        self.stream(period)
        keepalive_s = watchdog * 0.30 / 1000.0

        def _resend_if_due(last: float) -> float:
            now = time.monotonic()
            if now - last >= keepalive_s:
                self._conn.send_fast(f"S {speeds[0]} {speeds[1]}")
                return now
            return last

        try:
            self._conn.send_fast(f"S {speeds[0]} {speeds[1]}")
            last_send = time.monotonic()
            while True:
                for raw_line in self._conn.read_lines(duration=50):
                    r = parse_response(raw_line)
                    if r is None:
                        continue
                    if r.tag == "EVT" and r.tokens and r.tokens[0] == "safety_stop":
                        return
                    yield r
                    last_send = _resend_if_due(last_send)
                for reply in self._conn.drain_binary_tlm():
                    yield ParsedResponse(tag="TLM", tlm=TLMFrame.from_pb2(reply.tlm))
                    last_send = _resend_if_due(last_send)
                last_send = _resend_if_due(last_send)
        except GeneratorExit:
            try:
                self._conn.send_fast("STOP")
                self.stream(0)
            except Exception:
                pass
