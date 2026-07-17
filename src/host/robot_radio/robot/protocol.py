"""NezhaProtocol — binary wire-protocol adapter for the P4 single-loop
Nezha firmware (103-001 onward).

Owns the SerialConnection and is the only code that touches the serial port.
All command encoding and response parsing lives here; higher-level objects
(NezhaState, Nezha) delegate every wire operation to this class.

Wire format — P4 (single-loop firmware, 103-001)
-------------------------------------------------
The command plane is binary-only: one ``CommandEnvelope`` (protobuf,
``protos/envelope.proto``) per outbound command, armored as a `*B<base64>`
line. ``CommandEnvelope``'s ``cmd`` oneof carries exactly THREE arms —
``twist``/``config``/``stop`` — every earlier arm (ping/echo/id/hello/ver/
help/get/drive/segment/replace/motion/pose_fix/otos/stream/plan_dump) was
pruned by 103-001's schema prune and is `reserved`, not reused (see
``envelope.proto``'s own header comment). There is no per-command
synchronous reply for ``twist``/``config``/``stop`` — a command's outcome
rides the ack ring inside the next ``Telemetry`` push (``wait_for_ack()``).

Telemetry is always-on (no STREAM arm to arm first): the firmware pushes a
``ReplyEnvelope{tlm: Telemetry}`` frame unconditionally at ~25 Hz — see
``read_binary_tlm_frames()``/``read_pending_binary_tlm_frames()``.

104-002 deleted every method targeting a now-reserved arm (ping/echo/get_id/
get_ver/get_help/get_config/get_config_binary/pose_fix/drive/timed/distance/
arc/vw/turn/go_to/grip/zero_*/otos_*/port_*/stream/stream_fields/snap/
stream_drive/wait_for_evt_done/cancel and the ``Stop`` stop-clause-token
builder) — see this ticket's completion notes for the full disposition
table. ``set_config()``/``set_config_binary()`` survive (target the still-
live ``config`` arm); there is no live config READ-back path any more (the
``get`` arm is reserved) — a genuine, permanent wire-schema gap until a
future sprint adds one.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any

from robot_radio.io.serial_conn import SerialConnection

# Binary-plane pb2 bindings (096-007, M6 Host Config/Telemetry Client). Safe
# to import at module level here (unlike robot_radio.io.serial_conn.py --
# see that module's own _get_envelope_pb2() docstring for the circular-
# import hazard it avoids): robot_radio.robot.pb2 has no dependency back onto
# robot_radio.robot or robot_radio.io, so importing it while
# robot_radio.robot's own __init__.py is still mid-execution (which is
# always the case when this module is first loaded -- __init__.py imports
# this module itself) never re-enters a partially-initialized module.
from robot_radio.robot.pb2 import config_pb2, envelope_pb2, planner_pb2, telemetry_pb2


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


@dataclass(frozen=True)
class AckEntry:
    """One ack-ring entry from a ``Telemetry`` push (``telemetry.proto``
    ``AckEntry``, adapted onto a plain host-side shape the same way
    ``TLMFrame`` adapts ``Telemetry`` itself).

    Reports the outcome of ONE previously-sent command (matched by
    ``corr_id``) via the ack ring riding inside every ``Telemetry`` frame
    (103-009, Decision 2's "telemetry-only return path") — the P4 wire has
    no per-command synchronous ``ReplyEnvelope`` for ``twist``/``stop``/
    ``config``, so this is the ONLY place their outcome is reported.

    The ring is depth 3 (``telemetry.proto``'s own comment on
    ``Telemetry.acks``): the SAME ``corr_id`` legitimately rides more than
    one consecutive ``Telemetry`` push in a row. That is ring
    RE-DELIVERY, not a duplicate ack or an error — ``NezhaProtocol.
    wait_for_ack()`` tolerates it by construction (it simply returns on the
    first frame where a match is found; a caller never sees the re-delivered
    copies at all).
    """
    corr_id: int
    ok: bool
    err_code: int  # raw ErrCode (envelope.proto) value when ok is False, else 0 (ERR_NONE)

    @classmethod
    def from_pb2(cls, entry: "telemetry_pb2.AckEntry") -> "AckEntry":
        """Adapt one ``telemetry_pb2.AckEntry`` onto this dataclass."""
        return cls(
            corr_id=int(entry.corr_id),
            ok=(entry.status == telemetry_pb2.ACK_STATUS_OK),
            err_code=int(entry.err_code),
        )


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
    ``active`` (097-this-ticket) is ``bb.drivetrain.busy`` — TRUE while a
    segment/replace-arm motion is in progress, FALSE once it self-
    terminates. Populated from every binary STREAM/SNAP frame (see
    ``from_pb2()``'s own docstring for why this is the reliable motion-
    complete signal, unlike ``mode``).
    ``acks`` (103-009) is the ack ring riding this frame — up to 3
    ``AckEntry`` entries (``telemetry.proto`` ``Telemetry.acks``, depth 3),
    the ONLY way a P4 ``twist()``/``stop()``/``config`` command's outcome is
    reported (no per-command synchronous reply — see ``NezhaProtocol.
    wait_for_ack()``). Always present (an empty tuple, never None, once a
    frame has gone through ``from_pb2()`` — unconditional like ``active``).
    ``fault_bits``/``event_bits`` (103-009) are the raw firmware bitmasks
    (``telemetry.proto`` ``Telemetry.fault_bits``/``event_bits``) — see
    ``telemetry.proto``'s own comment for the bit numbering. Also always
    present once a frame has gone through ``from_pb2()``.
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
    active: bool | None = None                   # bb.drivetrain.busy — motion in progress (097, this ticket)
    acks: tuple[AckEntry, ...] | None = None      # ack-ring entries riding this frame, depth 3 (103-009)
    fault_bits: int | None = None                 # bitmask — see telemetry.proto Telemetry.fault_bits (103-009)
    event_bits: int | None = None                 # bitmask — see telemetry.proto Telemetry.event_bits (103-009)

    @classmethod
    def from_pb2(cls, telemetry: "telemetry_pb2.Telemetry") -> "TLMFrame":
        """Build a TLMFrame from a binary-plane ``pb2.Telemetry`` message
        (``ReplyEnvelope.body.tlm``, envelope.proto/telemetry.proto).

        Adapts telemetry.proto's wire shape onto this SAME dataclass shape
        the historical text-plane TLM parser (retired 097-003; see
        ``robot_radio.robot._legacy_tlm_text.parse_historical_tlm_line`` for
        the frozen reference copy kept for parity-testing and the narrow set
        of non-``SerialConnection`` consumers that module's own docstring
        names) already produced from a text STREAM/SNAP line — for every
        field the two formats share, ``from_pb2(telemetry)`` is field-for-
        field equal to what that historical parser produced from the
        matching text line. This is an ADAPTER, not a redesign: TLMFrame's
        existing fields/shape are unchanged; pb2's shape bends to fit them,
        never the reverse.

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
            ``src/host/robot_radio/calibration/fit_sim_error_model.py``'s
            module docstring for the one consumer this ticket found that
            structurally depends on it, and why that consumer stays on the
            text plane rather than silently losing the field.
          - OTOS connectivity — telemetry.proto DOES carry
            ``has_otos``/``otos``/``otos_connected``, but the historical
            text-plane parser never parsed the text plane's own
            ``otosconn=`` token into any TLMFrame field either, so
            ``otos_connected`` is dropped here too, for parity with the
            text path this dataclass already models.
          - ``cmd_vel`` (103-009, permanent gap): 103-001's prune
            (telemetry.proto's own file header) moved
            ``has_cmd_vel``/``cmd_vel_left``/``cmd_vel_right`` OUT of the
            primary ``Telemetry`` message to the new, slower
            ``TelemetrySecondary`` message — the primary ``Telemetry`` this
            method decodes no longer declares any of the three fields at
            all. Like ``encpose`` above, this is a genuine, permanent gap
            for a caller polling the PRIMARY telemetry stream — the
            velocity PID setpoint is still available, just on
            ``TelemetrySecondary`` (own cadence, own decode path — no
            ``TLMFrame`` adapter exists for it yet as of this ticket).
          - ``acc_left``/``acc_right``/``glitch_left``/``glitch_right``/
            ``ts_left``/``ts_right`` — 103-001 moved these to
            ``TelemetrySecondary`` too (telemetry.proto's own file header);
            same permanent-gap treatment as ``cmd_vel`` above.
          - ``active`` is the ONE exception to the paragraph above (097,
            this ticket): unlike the other bench-diagnostic fields,
            ``telemetry.active`` (``bb.drivetrain.busy`` — motion in
            progress, telemetry.proto field 18) is ALSO present,
            unconditionally, on every telemetry frame. It is populated
            below because it is the ONE reliable motion-complete signal —
            ``mode`` does not reliably track it for every drive path.
            ``__main__.py``'s ``_TourRunner._wait_for_idle`` uses it for
            exactly this reason.

        ``vel``/``cmd_vel``/``twist`` are always built as the DIFFERENTIAL
        tuple shape (matching this build's differential-only drivetrain and
        ``buildTlmFrame()``'s own ``"%d,%d"`` formatting): telemetry.proto's
        ``twist`` is a ``BodyTwist3`` (``v_x``, ``v_y``, ``omega``), but the
        firmware always zero-fills ``v_y`` for a differential build
        (tlm_frame.cpp), so ``v_y`` is dropped here exactly as the text
        plane's own 2-value ``twist=%d,%d`` already drops it.

        ``acks``/``fault_bits``/``event_bits`` (103-009) are populated
        unconditionally, the same "always present, not gated by a has_*
        flag" treatment as ``active`` above — ``telemetry.acks`` is a plain
        (possibly empty) repeated field, and ``fault_bits``/``event_bits``
        are plain uint32 fields, so there is no presence flag to check.
        """
        frame = cls()
        frame.t = telemetry.now
        frame.mode = _DRIVE_MODE_CHAR.get(telemetry.mode, "I")
        frame.seq = telemetry.seq
        frame.active = bool(telemetry.active)
        frame.acks = tuple(AckEntry.from_pb2(entry) for entry in telemetry.acks)
        frame.fault_bits = int(telemetry.fault_bits)
        frame.event_bits = int(telemetry.event_bits)

        if telemetry.has_enc:
            frame.enc = (int(telemetry.enc_left), int(telemetry.enc_right))

        if telemetry.has_vel:
            frame.vel = (int(telemetry.vel_left), int(telemetry.vel_right))

        # cmd_vel: NOT read here (103-009) -- telemetry.proto no longer
        # declares has_cmd_vel/cmd_vel_left/cmd_vel_right on the primary
        # Telemetry message (moved to TelemetrySecondary); see this
        # method's own docstring "Fields left at this dataclass's own
        # default" list for the permanent-gap explanation. frame.cmd_vel
        # stays at its dataclass default (None).

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
    """Structured representation of a single text-plane response line.

    Retained as generic line-parsing infrastructure (``parse_response()``
    below) — not itself a "verb" method targeting a specific
    ``CommandEnvelope`` oneof arm, so out of 104-002's dead-verb-deletion
    scope. No ``NezhaProtocol`` method in this file constructs one any
    more (the binary-only P4 command plane has no per-command
    ``ReplyEnvelope`` line to parse this way) — surviving callers are
    outside this file (e.g. relay-transport EVT/keepalive line handling).
    """
    tag: str          # "OK", "ERR", "EVT", "TLM", "CFG", "ID"
    tokens: list[str] = field(default_factory=list)  # plain tokens after tag
    kv: dict[str, str] = field(default_factory=dict) # key=value pairs
    corr_id: str | None = None                       # trailing #<id>, if any
    raw: str = ""                                    # original stripped line
    tlm: "TLMFrame | None" = None                     # binary-sourced frame, if any


# ---------------------------------------------------------------------------
# Module-level parse functions (can be used without a NezhaProtocol instance)
# ---------------------------------------------------------------------------

_RESPONSE_TAGS = frozenset(("OK", "ERR", "EVT", "TLM", "CFG", "ID"))


def _strip_relay(line: str) -> str:
    """Strip relay prefix characters and surrounding whitespace."""
    return line.strip().lstrip("<# ").strip()


def parse_response(line: str) -> ParsedResponse | None:
    """Parse one text-plane response line into a ParsedResponse, or None if
    unrecognised.

    Handles relay prefix stripping, optional trailing '#<id>' correlation
    token, and key=value pair extraction. See ``ParsedResponse``'s own
    docstring for why this parser survives 104-002's dead-verb sweep.
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
# Conversion). NezhaProtocol.set_config()/.config() keep a flat "wire key"
# vocabulary -- a flat "wire key" vocabulary -- but the binary plane's
# ConfigDelta (config.proto, 096-001) is typed, per-SLICE Patch messages,
# not a generic key/value map. This table is the translation between the
# two: it curates the SAME 15 keys config_commands.cpp's kAllKeys used to
# register on the (now-retired) text plane, transcribed here, PLUS
# headingKp/headingKd (098-005). There is no live config READ-back path
# (the binary `get` arm was pruned by 103-001) -- this table now serves
# `set_config()`/`config()` only.
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
# call needs only ONE motor envelope carrying these, never two.
_MOTOR_PID_KEYS = {
    "pid.kp": "kp",
    "pid.ki": "ki",
    "pid.kff": "kff",
    "pid.iMax": "i_max",
    "pid.kaw": "kaw",
}

# minSpeed -> PlannerConfigPatch.min_speed, config_pb2.CONFIG_PLANNER.
# headingKp/headingKd (098-005): the two outer heading-loop PD gains added to
# PlannerConfigPatch by ticket 098-005 -- there is no legacy text key for
# these two; added directly here so set_config(headingKp=...) reaches the
# SAME PlannerConfigPatch.heading_kp field binary_channel.cpp's
# handleConfigPlanner() decodes (src/firm/commands/binary_channel.cpp).
_PLANNER_KEYS = {
    "minSpeed": "min_speed",
    "headingKp": "heading_kp",
    "headingKd": "heading_kd",
}

# ml/mr and sTimeout are handled specially, not via a plain field-name map:
#   - ml/mr both patch MotorConfigPatch.travel_calib, disambiguated by
#     `side` (Decision 5, config.proto) -- ml=LEFT, mr=RIGHT.
#   - sTimeout is ConfigDelta's bare `watchdog` oneof arm (uint32, NOT a
#     message-typed Patch -- Open Question 4, config.proto), routed
#     straight to bb.streamWatchdogWindowIn, never bb.configIn.
_ALL_SET_KEYS = frozenset(
    set(_DRIVETRAIN_KEYS) | set(_MOTOR_PID_KEYS) | set(_PLANNER_KEYS)
    | {"ml", "mr", "sTimeout"})


def _format_config_value(value: Any) -> str:
    """Format a set_config() kwarg value into the SAME string shape the
    text plane's set_config() already produced -- floats to 6 significant
    digits, everything else via str(). Reused as-is from the pre-097-002
    text implementation (the formatting rule itself did not change)."""
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


# ---------------------------------------------------------------------------
# NezhaProtocol
# ---------------------------------------------------------------------------

class NezhaProtocol:
    """Binary wire-protocol adapter for the P4 single-loop Nezha firmware.

    Owns a SerialConnection and exposes one method per firmware command group
    (``twist``/``stop``/``config``) plus telemetry read accessors. All
    response parsing delegates to module-level parse_* functions so callers
    can reuse them on lines received through other paths (streaming
    generators).
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
        first). A historical ctypes sim connection backend used to return
        the decoded ``ReplyEnvelope`` (or ``None``) DIRECTLY instead, since
        that sim call was already synchronous -- see this method's own git
        history for the reconciliation this reflects. 108-006: that
        backend is deleted; its ctypes successor
        (``robot_radio.io.sim_loop.SimLoop``) is a ``TwistTransport``
        implementation, not a ``SerialConnection``-shaped object
        ``NezhaProtocol`` wraps, so this dict-vs-direct reconciliation is
        now purely a ``SerialConnection`` implementation detail this method
        still normalizes defensively.
        """
        result = self._conn.send_envelope(envelope, read_timeout=read_timeout)
        if isinstance(result, dict):
            return result.get("reply")
        return result

    def send(self, cmd: str, read_timeout: int = 500) -> dict:  # [ms]
        """Send a text-plane command, return raw response dict (for ad-hoc /
        pass-through). NOTE: the P4 firmware has no text-plane command
        parser (``main.cpp``'s dispatch switch decodes binary
        ``CommandEnvelope`` only) -- this passthrough survives as generic
        transport plumbing (``SerialConnection.send()``), not a verb this
        ticket's dead-arm sweep targets, but a text line sent through it
        reaches no live firmware handler."""
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
        """Parse a text-plane response line. Delegates to module-level
        parse_response()."""
        return parse_response(line)

    # ------------------------------------------------------------------
    # Config: SET (096-007/097-002, M2/M6). No live READ-back path -- the
    # binary `get` arm was pruned by 103-001 (envelope.proto reserves it);
    # get_config()/get_config_binary() were deleted by 104-002 alongside it.
    # ------------------------------------------------------------------

    def set_config(self, **kwargs: Any) -> dict[str, str] | None:
        """Send SET key=value ..., parse OK set response.

        Returns dict of applied keys (from OK set response) or None.
        Floats are formatted with up to 6 significant digits.

        Binary implementation (097-002): thin wrapper over set_config_binary()
        (096-007). Unlike the (retired) text plane's single atomic SET line, a
        ConfigDelta's oneof carries only ONE Patch at a time (config.proto),
        so kwargs spanning multiple targets (e.g. tw= + sTimeout=) become
        MULTIPLE set_config_binary() round trips, one per touched target --
        NOT atomic across targets (flagged, not silently reconciled, per this
        project's "transcribe, never re-derive; flag genuine gaps"
        discipline: a true cross-target atomic SET is not achievable without
        new binary wire capability). Any kwarg key outside the module-level
        _ALL_SET_KEYS vocabulary fails the WHOLE call (returns None, no wire
        traffic at all). If every touched target's round trip Acks, the
        returned dict echoes the kwargs actually sent (formatted the same
        way the pre-097-002 text implementation formatted them) -- the
        binary Ack carries no per-key echo the way the text "OK set ..."
        reply did, so this is the closest same-shape substitute, not a wire
        round trip of the applied value.

        See also ``config()`` (104-001): a stricter, single-envelope-per-
        call builder for the same ``ConfigDelta`` arm — raises ``ValueError``
        instead of silently no-op'ing on a multi-target or unknown-key
        call. Both survive; neither supersedes the other (see 104-002
        completion notes).
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
    # Config: binary SET (096-007, M6 Host Config/Telemetry Client)
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

    # ------------------------------------------------------------------
    # Drive commands
    # ------------------------------------------------------------------

    def twist(self, v_x: float, omega: float, duration: float) -> int:  # [mm/s] [rad/s] [ms]
        """Command a body-frame twist setpoint for ``duration`` ms — the P4
        wire's ONLY motion command (``CommandEnvelope{twist: Twist{v_x,
        omega, duration}}``, envelope.proto). The host computes the whole
        trajectory and streams twist setpoints; the robot only servos to
        them and arms the unified Deadman for ``duration`` (envelope.proto's
        own ``Twist`` doc comment).

        Fire-and-poll, NOT fire-and-wait (103-009, Decision 2's
        "telemetry-only return path"): the P4 wire has no per-command
        synchronous ``ReplyEnvelope`` for ``twist`` — this call's own outcome
        arrives later, riding the ack ring inside a subsequent ``Telemetry``
        push (see ``wait_for_ack()``). This method returns as soon as the
        bytes reach the wire; it never blocks waiting for a reply that will
        not come.

        Returns the corr_id assigned to this command — pass it to
        ``wait_for_ack()`` to confirm the firmware accepted it. Raises
        ``ConnectionError`` if not connected (``send_envelope_fast()``'s own
        not-open contract).
        """
        envelope = envelope_pb2.CommandEnvelope(
            twist=envelope_pb2.Twist(v_x=v_x, omega=omega, duration=duration))
        return self._conn.send_envelope_fast(envelope)

    def stop(self) -> int:
        """Panic-stop the drivetrain (``CommandEnvelope{stop: Stop{}}``) — a
        zero-field oneof arm that "cannot be malformed" (envelope.proto
        Decision 3).

        Fire-and-poll, the SAME shape as ``twist()`` (103-009, see its
        docstring for why): the P4 firmware reports ``stop``'s outcome via
        the ack ring (``wait_for_ack()``), not a synchronous reply, so this
        call writes the STOP bytes and returns immediately rather than
        blocking on a reply that will not come.

        Returns the corr_id assigned to this command — pass it to
        ``wait_for_ack()`` to confirm the firmware accepted it. Raises
        ``ConnectionError`` if not connected. Every existing caller in this
        tree calls ``stop()`` as a bare statement and ignores the return
        value.
        """
        envelope = envelope_pb2.CommandEnvelope(stop=envelope_pb2.Stop())
        return self._conn.send_envelope_fast(envelope)

    def config(self, **deltas: Any) -> int:
        """Build and send a ``ConfigDelta`` envelope (``CommandEnvelope{
        config: delta}``, ``envelope.proto`` field 6) — the P4 wire's THIRD
        and last ``cmd`` oneof arm, alongside ``twist()``/``stop()``
        (``envelope.proto``'s own oneof comment: "config/stop keep their
        pre-102 field numbers... twist is genuinely new"). 104-001 is what
        gives ``config`` a host-side builder — before it, every OTHER
        oneof arm (``twist``/``stop``) had one but ``config`` did not,
        despite being schema-defined since 103-001.

        Fire-and-poll, the SAME shape as ``twist()``/``stop()`` (103-009,
        Decision 2's "telemetry-only return path"): a ``config`` command's
        outcome rides the ack ring inside a subsequent ``Telemetry`` push,
        never a synchronous ``ReplyEnvelope`` — see ``wait_for_ack()``. This
        method writes the bytes and returns immediately.

        ``deltas`` reuses the SAME flat wire-key vocabulary ``set_config()``
        curates (module-level ``_DRIVETRAIN_KEYS``/``_MOTOR_PID_KEYS``/
        ``_PLANNER_KEYS``/``ml``/``mr``/``sTimeout`` — together
        ``_ALL_SET_KEYS``), so a key added to one map is automatically
        available to the other; nothing here re-derives that vocabulary.
        UNLIKE ``set_config()``, which fans a multi-target kwargs dict out
        into MULTIPLE round trips (one per touched ``ConfigDelta.patch``
        oneof arm, since a single ``ConfigDelta`` carries only one patch at
        a time), ``config()`` builds and sends exactly ONE
        ``CommandEnvelope`` carrying exactly ONE ``ConfigDelta`` — matching
        ``twist()``/``stop()``'s own "one call, one envelope, one corr_id"
        shape. Passing kwargs that span more than one ``ConfigDelta.patch``
        target (e.g. ``tw=`` and ``pid.kp=`` together — drivetrain vs. motor)
        is a caller error: raises ``ValueError``, since no single
        ``ConfigDelta`` could carry both. Same for empty ``deltas`` or a key
        outside the known vocabulary. ``pid.*`` keys and ``ml``/``mr`` may be
        mixed freely in one call — both target the SAME ``MotorConfigPatch``
        oneof arm (mirroring ``set_config()``'s own ``motor_left_patch``/
        ``motor_right_patch`` grouping); ``side`` selects ``travel_calib``'s
        target only and is meaningless for the ``pid.*`` fields
        (``config.proto``'s own ``MotorConfigPatch.side`` comment), so a
        pure-``pid.*`` call (no ``ml``/``mr``) still needs SOME side value on
        the wire — it defaults to ``LEFT``, the same default
        ``set_config()``'s own ``motor_left_patch`` branch always used.

        Confirmed against the merged 103 tree's ``main.cpp`` (resolving
        103's Step 7 Open Question 3, ``architecture-update.md``): the
        firmware's dispatch switch decodes ``CONFIG`` successfully but does
        NOT apply it — the ``CmdKind::CONFIG`` case acks
        ``ACK_STATUS_ERR``/``ERR_UNIMPLEMENTED`` unconditionally
        ("ConfigDelta runtime application deferred this sprint"). This
        method still builds and sends the envelope regardless — ``config()``
        is a wire builder, not a promise the firmware applies the delta;
        pass the returned corr_id to ``wait_for_ack()`` to observe today's
        ``ERR_UNIMPLEMENTED`` outcome, which will flip to a live-apply Ack
        once a future ticket wires the runtime side.

        Returns the corr_id assigned to this command. Raises
        ``ConnectionError`` if not connected (``send_envelope_fast()``'s own
        not-open contract); raises ``ValueError`` for empty, unknown-key, or
        multi-target ``deltas``.
        """
        if not deltas:
            raise ValueError("config() requires at least one key=value delta")
        unknown = sorted(k for k in deltas if k not in _ALL_SET_KEYS)
        if unknown:
            raise ValueError(f"config(): unknown key(s) {unknown!r}")

        drivetrain_patch: dict[str, float] = {}
        motor_patch: dict[str, float] = {}
        motor_side = config_pb2.LEFT
        planner_patch: dict[str, float] = {}
        watchdog_value: int | None = None

        for key, value in deltas.items():
            if key in _DRIVETRAIN_KEYS:
                drivetrain_patch[_DRIVETRAIN_KEYS[key]] = float(value)
            elif key == "ml":
                motor_patch["travel_calib"] = float(value)
                motor_side = config_pb2.LEFT
            elif key == "mr":
                motor_patch["travel_calib"] = float(value)
                motor_side = config_pb2.RIGHT
            elif key in _MOTOR_PID_KEYS:
                motor_patch[_MOTOR_PID_KEYS[key]] = float(value)
            elif key in _PLANNER_KEYS:
                planner_patch[_PLANNER_KEYS[key]] = float(value)
            elif key == "sTimeout":
                watchdog_value = int(value)

        targets_touched = sum([
            bool(drivetrain_patch), bool(motor_patch),
            bool(planner_patch), watchdog_value is not None,
        ])
        if targets_touched > 1:
            raise ValueError(
                "config(): kwargs span more than one ConfigDelta.patch "
                f"target (got {sorted(deltas)}) — a single ConfigDelta "
                "carries only one patch; call config() once per target")

        if drivetrain_patch:
            delta = envelope_pb2.ConfigDelta(
                drivetrain=config_pb2.DrivetrainConfigPatch(**drivetrain_patch))
        elif motor_patch:
            delta = envelope_pb2.ConfigDelta(motor=config_pb2.MotorConfigPatch(
                side=motor_side, **motor_patch))
        elif planner_patch:
            delta = envelope_pb2.ConfigDelta(
                planner=config_pb2.PlannerConfigPatch(**planner_patch))
        else:
            delta = envelope_pb2.ConfigDelta(watchdog=watchdog_value)

        envelope = envelope_pb2.CommandEnvelope(config=delta)
        return self._conn.send_envelope_fast(envelope)

    def otos_config(self, *, linear_scale: float | None = None,
                    angular_scale: float | None = None,
                    offset_x: float | None = None,
                    offset_y: float | None = None,
                    offset_yaw: float | None = None,
                    init: bool = False) -> int:
        """Build and send an ``OtosConfigPatch`` ``ConfigDelta`` envelope
        (``CommandEnvelope{config: ConfigDelta{otos: ...}}``, 109-004) — the
        ``OL``/``OA``/``OI`` wire-verb family's direct-patch-send mechanism
        (sprint 109's Architecture Revision 1: "OL/OA/OI construct and send
        an OtosConfigPatch directly", never through the dead
        ``binary_bridge.translate_command()`` legacy-verb layer).

        A SEPARATE method from ``config()`` rather than folding OTOS keys
        into ``_ALL_SET_KEYS``: OL/OA/OI were never flat ``SET key=value``
        text verbs (unlike ``tw``/``pid.kp``/... — they are their own
        one-or-zero-positional-argument verbs), so there is no existing flat
        wire-key vocabulary to extend; this mirrors ``config()``'s own
        "build exactly ONE envelope carrying exactly ONE patch" shape
        instead of that method's kwargs-to-flat-key mapping.

        ``linear_scale``/``angular_scale`` map 1:1 to
        ``Otos::setLinearScalar()``/``setAngularScalar()`` (OL/OA);
        ``offset_x``/``offset_y``/``offset_yaw`` map to ``Otos::
        setOffset()`` (no wire verb sends these yet this ticket — schema
        capacity for a future OV-equivalent); ``init=True`` maps to
        ``Otos::init()`` (OI) — a plain trigger flag, not a value, so it has
        no corresponding keyword default other than ``False``.

        Fire-and-poll, the SAME shape as ``twist()``/``stop()``/``config()``
        (103-009's "telemetry-only return path"): this call writes the bytes
        and returns immediately; its outcome rides the ack ring
        (``wait_for_ack()``).

        Returns the corr_id assigned to this command. Raises
        ``ConnectionError`` if not connected; raises ``ValueError`` if no
        field is set at all (every kwarg ``None`` and ``init`` falsy — an
        empty patch is a caller error, mirroring ``config()``'s own empty-
        ``deltas`` rejection).
        """
        fields: dict[str, Any] = {}
        if linear_scale is not None:
            fields["linear_scale"] = float(linear_scale)
        if angular_scale is not None:
            fields["angular_scale"] = float(angular_scale)
        if offset_x is not None:
            fields["offset_x"] = float(offset_x)
        if offset_y is not None:
            fields["offset_y"] = float(offset_y)
        if offset_yaw is not None:
            fields["offset_yaw"] = float(offset_yaw)
        if init:
            fields["init"] = True

        if not fields:
            raise ValueError(
                "otos_config() requires at least one field (linear_scale/"
                "angular_scale/offset_x/offset_y/offset_yaw/init)")

        delta = envelope_pb2.ConfigDelta(
            otos=config_pb2.OtosConfigPatch(**fields))
        envelope = envelope_pb2.CommandEnvelope(config=delta)
        return self._conn.send_envelope_fast(envelope)

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "AckEntry | None":  # [ms]
        """Poll incoming ``Telemetry`` pushes for an ack-ring entry matching
        ``corr_id``, for up to ``timeout`` ms. Returns the matched
        ``AckEntry``, or ``None`` if the deadline passes with no match —
        this wait is always bounded, never infinite.

        The ack-ring matcher for the P4 "telemetry-only return path"
        (103-009, Decision 2): ``twist()``/``stop()``/``config()`` (104-001
        — every ``CommandEnvelope`` oneof arm now uses this same shape) get
        no synchronous ``ReplyEnvelope`` of their own — their outcome rides
        the ack ring (``Telemetry.acks``, depth 3) inside the next one or
        more regular ``Telemetry`` pushes after the command reaches the
        firmware. Because the ring is depth 3 and telemetry pushes at
        ~25 Hz, the SAME ``corr_id`` legitimately appears in more than one
        polled frame in a row — that is ring RE-DELIVERY, not an error and
        not a duplicate ack — and tolerating it (returning on the FIRST
        frame where ``corr_id`` is found, so a caller never sees or has to
        dedupe the re-delivered copies) is part of the matcher's contract.

        104-003: the actual poll/match/timeout loop is no longer inline
        here — it was promoted to ``SerialConnection.wait_for_ack()`` (see
        that method's own docstring for the full algorithm, including the
        ring-wrap-is-just-a-timeout note) so every future caller reading
        telemetry directly off ``SerialConnection`` — not just
        ``NezhaProtocol`` — gets the identical matching guarantee without a
        second copy of the algorithm. This method is now a thin adapter:
        delegate to the shared implementation, then wrap the raw
        ``telemetry_pb2.AckEntry`` result in this module's own ``AckEntry``
        dataclass (the same adaptation ``TLMFrame.from_pb2()`` performs for
        telemetry frames generally).
        """
        raw_ack = self._conn.wait_for_ack(corr_id, timeout=timeout)
        if raw_ack is None:
            return None
        return AckEntry.from_pb2(raw_ack)

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def read_binary_tlm_frames(self, duration: int) -> "list[TLMFrame]":  # [ms]
        """Block for up to ``duration`` ms, returning every binary telemetry
        frame received during that window as ``TLMFrame`` objects (097-003).

        Telemetry is always-on in the P4 design (no arming step) — reads
        ``SerialConnection.read_binary_tlm()`` (``_binary_tlm_queue``) and
        adapts each raw ``pb2.ReplyEnvelope`` via ``TLMFrame.from_pb2()``.
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
