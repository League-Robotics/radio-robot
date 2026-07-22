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
``ReplyEnvelope{tlm: Telemetry}`` frame unconditionally every loop iteration
(primary period == cycle period, 20 ms — frame v2, 115-003) — see
``read_binary_tlm_frames()``/``read_pending_binary_tlm_frames()``.

Telemetry frame v2 (115-003, gut-to-minimal-firmware S1 — implements
``telemetry-frame-tightening-amendment-to-gut-s1.md``): a clean, incompatible
rewrite of the ``Telemetry`` message. Per-source reading objects
(``EncoderReading``/``OtosReading``) replace the old flat ``enc_*``/``vel_*``
floats and the bare ``Pose2D otos``; one ``flags`` bit-string replaces every
standalone status bool plus the ``fault_bits``/``event_bits`` masks; a single
``ack_corr``/``ack_err`` slot replaces the depth-3 ``AckEntry`` ring; packed
``line``/``color`` sensor words are new. See ``TLMFrame``'s own docstring for
the host-side adaptation.

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
from robot_radio.robot.pb2 import config_pb2, envelope_pb2, telemetry_pb2


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
# (telemetry.mode, telemetry.proto -- DriveMode relocated in from the deleted
# planner.proto by 115-003, unchanged shape) to the SAME single-character
# mode= wire value the historical text-plane TLM parser read off a text
# STREAM/SNAP frame's "mode=" token. VELOCITY has no dedicated character
# (modeChar()'s own `default: return 'I';` case) -- mirrored here via .get()'s
# fallback in from_pb2(), not a dict entry, so a future DriveMode value added
# to telemetry.proto without a matching modeChar() case falls back the same
# way on both sides.
_DRIVE_MODE_CHAR = {
    telemetry_pb2.IDLE: "I",
    telemetry_pb2.STREAMING: "S",
    telemetry_pb2.TIMED: "T",
    telemetry_pb2.DISTANCE: "D",
    telemetry_pb2.GO_TO: "G",
}


@dataclass(frozen=True)
class AckEntry:
    """The single ack slot from a ``Telemetry`` push (``telemetry.proto``
    ``Telemetry.ack_corr``/``ack_err``), adapted onto a plain host-side
    shape the same way ``TLMFrame`` adapts ``Telemetry`` itself.

    Reports the outcome of ONE previously-sent command (matched by
    ``corr_id``) via the single ack slot riding inside every ``Telemetry``
    frame (103-009 Decision 2's "telemetry-only return path", narrowed from
    a depth-3 ring to one slot by 115-003) — the P4 wire has no per-command
    synchronous ``ReplyEnvelope`` for ``twist``/``stop``/``config``, so this
    is the ONLY place their outcome is reported.

    115-003 frame v2: the depth-3 ``AckEntry`` ring (and the ``AckStatus``
    enum it carried — OK/ERR/DONE/TRIVIAL/SUPERSEDED/FLUSHED/TIMEOUT/
    SOLVE_FAIL, the executor's own per-command completion taxonomy) is
    DELETED — there is no wire ``AckEntry`` message any more, no executor,
    and no completion-status taxonomy beyond plain OK/ERR. ``ack_err == 0``
    means OK; nonzero is the raw ``ErrCode`` (envelope.proto) value — the
    SAME two-value shape ``TWIST``/``STOP``/``CONFIG`` always produced, so
    nothing downstream of `ok`/`err_code` loses information. A command
    acked within the same primary period as another OVERWRITES the slot
    (stakeholder-accepted tradeoff, amendment issue's own "ack-depth-1
    tradeoff" note) — rare at bench rates; ``wait_for_ack()``'s own
    timeout+retry covers it.
    """
    corr_id: int
    ok: bool
    err_code: int  # raw ErrCode (envelope.proto) value when ok is False, else 0 (ERR_NONE)

    @classmethod
    def from_telemetry(cls, telemetry: "telemetry_pb2.Telemetry") -> "AckEntry":
        """Build an ``AckEntry`` from ``telemetry``'s single ack slot
        (``ack_corr``/``ack_err``). Call only when ``flags`` bit 5
        (``ack_fresh``) is set — this method does not itself check the bit
        (``TLMFrame.from_pb2()``/``NezhaProtocol.wait_for_ack()`` are the
        two call sites, and both already gate on it)."""
        return cls(
            corr_id=int(telemetry.ack_corr),
            ok=(telemetry.ack_err == 0),
            err_code=int(telemetry.ack_err),
        )


# ---------------------------------------------------------------------------
# Reading objects (115-003 frame v2) -- host-side adapters for the wire's
# per-source EncoderReading/OtosReading messages (telemetry.proto), mirroring
# AckEntry's own "plain dataclass adapted from a pb2 message" shape. Named the
# SAME as their telemetry_pb2 counterparts; no collision since telemetry_pb2
# is always accessed as a qualified module attribute (telemetry_pb2.
# EncoderReading), never imported bare.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class EncoderReading:
    """One wheel's encoder sample -- position AND velocity together, stamped
    with the sample's own collect time (robot clock, same domain as
    ``TLMFrame.t``). Adapts ``telemetry_pb2.EncoderReading``."""
    position: float  # [mm] accumulated
    velocity: float  # [mm/s] signed, measured
    time: int        # [ms] robot clock at sample collect

    @classmethod
    def from_pb2(cls, reading: "telemetry_pb2.EncoderReading") -> "EncoderReading":
        return cls(position=float(reading.position), velocity=float(reading.velocity),
                   time=int(reading.time))


@dataclass(frozen=True)
class OtosReading:
    """Everything the OTOS supplies in one burst read: position, heading,
    AND the measured velocities (v_x/v_y/omega -- previously read by the
    driver and dropped on the floor), stamped with the burst's own read
    time. Adapts ``telemetry_pb2.OtosReading``. Valid iff ``TLMFrame.
    otos_present`` (flags bit 0) -- see ``TLMFrame.otos_reading``."""
    x: float        # [mm]
    y: float        # [mm]
    heading: float  # [rad]
    v_x: float      # [mm/s]
    v_y: float      # [mm/s]
    omega: float    # [rad/s]
    time: int       # [ms] robot clock at burst read

    @classmethod
    def from_pb2(cls, reading: "telemetry_pb2.OtosReading") -> "OtosReading":
        return cls(x=float(reading.x), y=float(reading.y), heading=float(reading.heading),
                   v_x=float(reading.v_x), v_y=float(reading.v_y), omega=float(reading.omega),
                   time=int(reading.time))


# ---------------------------------------------------------------------------
# flags bit layout (telemetry.proto Telemetry.flags -- 115-003). Mirrors the
# proto's own bit-table comment exactly; TLMFrame's presence/status/fault/
# event properties below are computed from these constants, never a second
# hand-copied numbering.
# ---------------------------------------------------------------------------
_FLAG_OTOS_PRESENT = 1 << 0
_FLAG_OTOS_CONNECTED = 1 << 1
_FLAG_ACTIVE = 1 << 2
_FLAG_CONN_LEFT = 1 << 3
_FLAG_CONN_RIGHT = 1 << 4
_FLAG_ACK_FRESH = 1 << 5
_FLAG_FAULT_I2C_SAFETY_NET = 1 << 6
_FLAG_FAULT_WEDGE_LATCH = 1 << 7
_FLAG_FAULT_I2C_NAK_TIMEOUT = 1 << 8
_FLAG_FAULT_MALFORMED_FRAME = 1 << 9
_FLAG_EVENT_DEADMAN_EXPIRED = 1 << 10
_FLAG_EVENT_BOOT_READY = 1 << 11
_FLAG_EVENT_CONFIG_APPLIED = 1 << 12
_FLAG_LINE_PRESENT = 1 << 13
_FLAG_COLOR_PRESENT = 1 << 14
_FLAG_FAULT_MOVE_TIMEOUT = 1 << 15


def _unpack_channels4(word: "int | None") -> "tuple[int, int, int, int] | None":
    """Unpack a packed 4-channel sensor word (``telemetry.proto``'s ``line``/
    ``color`` fields share this exact packing: one byte per channel, channel
    1 in the low byte) into a ``(ch1, ch2, ch3, ch4)`` tuple. Returns
    ``None`` for ``None`` (not-present) input -- callers gate on ``line``/
    ``color`` returning None."""
    if word is None:
        return None
    return (word & 0xFF, (word >> 8) & 0xFF, (word >> 16) & 0xFF, (word >> 24) & 0xFF)


@dataclass
class TLMFrame:
    """Parsed TLM telemetry frame from the firmware (frame v2, 115-003).

    All fields are optional — a frame built without going through
    ``from_pb2()`` (a hand-built test double, e.g.) leaves every field at
    this dataclass's own ``None`` default, distinguishing "never decoded"
    from "decoded as the wire's zero value". ``t`` is the robot clock in
    milliseconds at frame-assembly time. ``seq`` is the D10 sequence
    counter (uint16, wrapping at 65535). Use ``tlm_drop_rate(frames)`` to
    estimate packet loss. ``pose``/``otos`` heading is in centi-degrees
    (integer), positions in mm — this dataclass's own historical unit
    convention, kept unchanged by the frame v2 rewrite so every existing
    downstream reader (e.g. ``testgui/telemetry_panel.py``) keeps working.

    ``enc``/``vel`` are ``(left, right)`` — position [mm] / velocity [mm/s]
    per wheel, now DERIVED from the wire's own ``EncoderReading`` messages
    (``enc_left``/``enc_right``, each carrying position+velocity+its own
    collect time together — see ``enc_left``/``enc_right`` below for the
    full reading including ``time``). Always present on the wire (no
    presence gate), so always populated by ``from_pb2()``.

    ``twist`` is fused body-frame velocity, 2-tuple ``(v_mmps,
    omega_mradps)`` — the wire's ``BodyTwist3`` always zero-fills ``v_y``
    for this differential build (``tlm_frame.cpp``), so ``v_y`` is dropped
    here exactly as before. Always present on the wire.

    ``otos`` is the raw OTOS pose, ``(x, y, heading)`` in (mm, mm, cdeg) —
    valid iff ``otos_present`` (flags bit 0); ``otos_reading`` (below)
    carries the SAME burst's fuller shape (velocities + its own read
    time) for a caller that wants more than the legacy 3-tuple.

    ``line``/``color`` are ``(ch1..ch4)`` / ``(r, g, b, c)`` — NEWLY wired
    this ticket: frame v2 packs both sensors into one ``uint32`` word each
    (one byte per channel); previously these fields existed on this
    dataclass but were never populated by the binary decode path (only the
    retired text-plane parser ever set them). Valid iff ``line_present``/
    ``color_present`` (flags bits 13/14).

    ``ekf_rej``, ``wedge``, ``encpose``, ``otos_health`` remain permanent
    gaps for the binary decode path — telemetry.proto never declared
    matching fields even before this rewrite (see the retired text-plane
    parser, ``robot_radio.robot._legacy_tlm_text``, for the only place
    these were ever populated) — frame v2 does not change that.
    ``cmd_vel``/``acc_*``/``glitch_*``/``ts_*`` remain on
    ``TelemetrySecondary`` (own cadence, own decode path — 103-001,
    untouched by this ticket) — same permanent gap on the PRIMARY frame
    this class decodes.

    ``active`` is ``bb.drivetrain.busy`` (flags bit 2) — TRUE while a
    motion is in progress. The one reliable motion-complete signal
    (``mode`` does not track it for every drive path).

    ``flags`` is the raw wire bit-string (``telemetry.proto``
    ``Telemetry.flags`` — see that message's own bit-table comment for the
    authoritative numbering) — always populated. Every other
    presence/status/fault/event signal below is a ``@property`` DERIVED
    from ``flags``, never a second field to keep in sync:
      - ``otos_present`` (bit 0), ``otos_connected`` (bit 1) — OTOS
        freshness/connectivity.
      - ``conn_left``/``conn_right`` (bits 3/4) — per-motor bus
        connectivity.
      - ``ack_fresh`` (bit 5) — whether ``ack``/``ack_corr``/``ack_err``
        (below) carry a NEW ack this frame (see ``ack``'s own note).
      - ``fault_i2c_safety_net``/``fault_wedge_latch``/
        ``fault_i2c_nak_timeout``/``fault_malformed_frame`` (bits 6-9) —
        the four fault bits.
      - ``event_deadman_expired``/``event_boot_ready``/
        ``event_config_applied`` (bits 10-12) — the three (one-shot,
        transition-cycle) event bits.
      - ``line_present``/``color_present`` (bits 13/14) — packed-word
        freshness.
      - ``fault_move_timeout`` (bit 15) — declared now, not wired until
        sprint 116's MOVE protocol lands (S1 has no MOVE command to time
        out); always False until then.
    These properties are the ticket's own "existing downstream consumer
    keeps working unchanged" surface — grep ``src/host/robot_radio/`` for
    every attribute name the pre-115 standalone bool/bitmask fields
    exposed before renaming or removing any of them.

    ``ack_corr``/``ack_err`` are the wire's single ack slot (raw, always
    populated — replaces the pre-115 depth-3 ``acks`` ring), valid iff
    ``ack_fresh``. ``ack`` is the SAME slot pre-decoded into an
    ``AckEntry`` (``None`` unless ``ack_fresh`` was set on this frame) —
    the convenience shape most callers want; ``wait_for_ack()`` returns
    the identical ``AckEntry`` shape.

    ``enc_left``/``enc_right`` (``EncoderReading | None``) and
    ``otos_reading`` (``OtosReading | None``, valid iff ``otos_present``)
    are the full per-source reading objects the wire now carries — richer
    than ``enc``/``vel``/``otos`` above (they add each reading's OWN
    collect/burst time), for a caller (e.g. ticket 008's ``tlm_log.py``)
    that wants the raw per-sample stamps rather than the legacy tuples.
    """
    t: int | None = None
    mode: str | None = None
    seq: int | None = None                       # D10 sequence counter (uint16, wraps at 65535)
    flags: int | None = None                      # raw bit-string -- see telemetry.proto Telemetry.flags (115-003)
    enc: tuple[int, int] | None = None          # (left, right) [mm] -- derived from enc_left/enc_right.position
    pose: tuple[int, int, int] | None = None    # (x, y, heading) [mm, mm, cdeg]
    vel: tuple[int, int] | None = None          # (left, right) [mm/s] -- derived from enc_left/enc_right.velocity
    cmd_vel: tuple[int, int] | None = None      # (left, right) COMMANDED per-wheel velocity (PID setpoint) mm/s -- permanent gap, TelemetrySecondary only
    twist: tuple[int, int] | None = None        # (v, omega_mrad)
    otos: tuple[int, int, int] | None = None    # (x, y, heading) [mm, mm, cdeg] — raw OTOS pose; valid iff otos_present
    line: tuple[int, int, int, int] | None = None   # (ch1, ch2, ch3, ch4); valid iff line_present
    color: tuple[int, int, int, int] | None = None  # (r, g, b, c); valid iff color_present
    ekf_rej: int | None = None                   # cumulative EKF gate rejection count -- permanent binary-decode gap
    wedge: tuple[int, int] | None = None         # (left, right) wedge-latch state, 0/1 each -- permanent binary-decode gap
    encpose: tuple[int, int, int] | None = None  # (x, y, heading) [mm, mm, cdeg] -- permanent binary-decode gap
    otos_health: tuple[int, bool] | None = None  # (raw STATUS byte, fusion_blocked) -- permanent binary-decode gap
    active: bool | None = None                   # bb.drivetrain.busy — motion in progress (flags bit 2)
    ack_corr: int | None = None                  # raw ack_corr [uint32]; valid iff ack_fresh (115-003, replaces the depth-3 acks ring)
    ack_err: int | None = None                   # raw ack_err (envelope.proto ErrCode); valid iff ack_fresh
    ack: "AckEntry | None" = None                 # ack_corr/ack_err pre-decoded, populated ONLY when ack_fresh is set
    enc_left: "EncoderReading | None" = None      # full per-wheel reading (position/velocity/time) -- always present on the wire
    enc_right: "EncoderReading | None" = None
    otos_reading: "OtosReading | None" = None      # full OTOS burst (adds v_x/v_y/omega/time over `otos`); valid iff otos_present

    # ------------------------------------------------------------------
    # flags-derived properties (115-003) -- see this class's own docstring.
    # ------------------------------------------------------------------

    def _flag(self, bit: int) -> bool:
        return bool(self.flags is not None and (self.flags & bit))

    @property
    def otos_present(self) -> bool:
        return self._flag(_FLAG_OTOS_PRESENT)

    @property
    def otos_connected(self) -> bool:
        return self._flag(_FLAG_OTOS_CONNECTED)

    @property
    def conn_left(self) -> bool:
        return self._flag(_FLAG_CONN_LEFT)

    @property
    def conn_right(self) -> bool:
        return self._flag(_FLAG_CONN_RIGHT)

    @property
    def ack_fresh(self) -> bool:
        return self._flag(_FLAG_ACK_FRESH)

    @property
    def fault_i2c_safety_net(self) -> bool:
        """Known-benign boot one-shot (telemetry.proto's own bit-6 comment)
        -- only a bit that flips DURING driving, not just at boot, is
        actionable."""
        return self._flag(_FLAG_FAULT_I2C_SAFETY_NET)

    @property
    def fault_wedge_latch(self) -> bool:
        return self._flag(_FLAG_FAULT_WEDGE_LATCH)

    @property
    def fault_i2c_nak_timeout(self) -> bool:
        return self._flag(_FLAG_FAULT_I2C_NAK_TIMEOUT)

    @property
    def fault_malformed_frame(self) -> bool:
        return self._flag(_FLAG_FAULT_MALFORMED_FRAME)

    @property
    def fault_move_timeout(self) -> bool:
        """Bit 15 -- declared now, wired by sprint 116's MOVE protocol; S1
        has no MOVE command to time out, so this is always False today."""
        return self._flag(_FLAG_FAULT_MOVE_TIMEOUT)

    @property
    def event_deadman_expired(self) -> bool:
        return self._flag(_FLAG_EVENT_DEADMAN_EXPIRED)

    @property
    def event_boot_ready(self) -> bool:
        return self._flag(_FLAG_EVENT_BOOT_READY)

    @property
    def event_config_applied(self) -> bool:
        return self._flag(_FLAG_EVENT_CONFIG_APPLIED)

    @property
    def line_present(self) -> bool:
        return self._flag(_FLAG_LINE_PRESENT)

    @property
    def color_present(self) -> bool:
        return self._flag(_FLAG_COLOR_PRESENT)

    @classmethod
    def from_pb2(cls, telemetry: "telemetry_pb2.Telemetry") -> "TLMFrame":
        """Build a TLMFrame from a binary-plane ``pb2.Telemetry`` message
        (``ReplyEnvelope.body.tlm``, envelope.proto/telemetry.proto,
        frame v2 -- 115-003).

        Adapts telemetry.proto's wire shape onto this SAME dataclass shape
        pre-115 callers already read (``t``/``mode``/``seq``/``enc``/
        ``vel``/``pose``/``otos``/``twist``/``active``/``line``/``color``) —
        the decode INTERNALS move (nested readings, one flags bit-string,
        one ack slot), the dataclass's own public field names do not. This
        is an ADAPTER, not a redesign.

        Truncation matches the firmware's own text formatter exactly
        (``buildTlmFrame()``'s ``static_cast<int>``, i.e. truncate-toward-
        zero) — Python's ``int()`` on a float does the same.

        ``enc_left``/``enc_right``/``pose``/``twist`` are ALWAYS present on
        the wire (no presence gate, message-typed fields with proto3
        zero-value defaults when genuinely absent) — populated
        unconditionally, unlike pre-115's ``has_enc``/``has_vel``/
        ``has_pose``/``has_twist``-gated decode. ``otos``/``otos_reading``/
        ``line``/``color`` stay gated, now on ``flags`` bits (0/13/14)
        instead of ``has_otos``-style bool fields.

        Permanent gaps unchanged by this rewrite (telemetry.proto declares
        no matching field): ``wedge``, ``encpose``, ``otos_health``,
        ``ekf_rej``, ``cmd_vel`` (lives on ``TelemetrySecondary`` — own
        cadence, own decode path, untouched by 115-003).
        """
        frame = cls()
        frame.t = telemetry.now
        frame.mode = _DRIVE_MODE_CHAR.get(telemetry.mode, "I")
        frame.seq = telemetry.seq
        frame.flags = int(telemetry.flags)
        frame.active = bool(frame._flag(_FLAG_ACTIVE))

        frame.enc_left = EncoderReading.from_pb2(telemetry.enc_left)
        frame.enc_right = EncoderReading.from_pb2(telemetry.enc_right)
        frame.enc = (int(frame.enc_left.position), int(frame.enc_right.position))
        frame.vel = (int(frame.enc_left.velocity), int(frame.enc_right.velocity))

        frame.pose = (
            int(telemetry.pose.x),
            int(telemetry.pose.y),
            int(telemetry.pose.h * _ANGLE_SCALE),
        )
        frame.twist = (
            int(telemetry.twist.v_x),
            int(telemetry.twist.omega * 1000.0),
        )

        if frame.otos_present:
            frame.otos_reading = OtosReading.from_pb2(telemetry.otos)
            frame.otos = (
                int(telemetry.otos.x),
                int(telemetry.otos.y),
                int(telemetry.otos.heading * _ANGLE_SCALE),
            )

        if frame.line_present:
            frame.line = _unpack_channels4(int(telemetry.line))
        if frame.color_present:
            frame.color = _unpack_channels4(int(telemetry.color))

        frame.ack_corr = int(telemetry.ack_corr)
        frame.ack_err = int(telemetry.ack_err)
        if frame.ack_fresh:
            frame.ack = AckEntry.from_telemetry(telemetry)

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

# PlannerConfigPatch/CONFIG_PLANNER -- DELETED (115-003, gut-to-minimal-
# firmware S1 motion-stack excision): minSpeed/headingKp/headingKd/
# distanceKp/arriveDwell all patched PlannerConfigPatch (config.proto),
# deleted wholesale alongside Motion::Executor/App::Pilot, the subsystems
# that read them. There is no live config target left for any of the five
# -- they are simply no longer valid set_config()/config() keys (both
# raise/return the same "unknown key" outcome as any other bogus key).

# ml/mr and sTimeout are handled specially, not via a plain field-name map:
#   - ml/mr both patch MotorConfigPatch.travel_calib, disambiguated by
#     `side` (Decision 5, config.proto) -- ml=LEFT, mr=RIGHT.
#   - sTimeout is ConfigDelta's bare `watchdog` oneof arm (uint32, NOT a
#     message-typed Patch -- Open Question 4, config.proto), routed
#     straight to bb.streamWatchdogWindowIn, never bb.configIn.
_ALL_SET_KEYS = frozenset(
    set(_DRIVETRAIN_KEYS) | set(_MOTOR_PID_KEYS) | {"ml", "mr", "sTimeout"})


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
        (``drivetrain``/``motor``/``watchdog``/``otos``) selects which
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
        ``ml``/``mr``/``sTimeout`` — together ``_ALL_SET_KEYS``), so a key
        added to one map is automatically
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

        Historically (sprint 103, resolving 103's Step 7 Open Question 3):
        the firmware's dispatch switch decoded ``CONFIG`` successfully but
        did NOT apply it — acked ``ack_err=ERR_UNIMPLEMENTED`` unconditionally
        ("ConfigDelta runtime application deferred this sprint"). This
        method still builds and sends the envelope regardless — ``config()``
        is a wire builder, not a promise the firmware applies the delta;
        pass the returned corr_id to ``wait_for_ack()`` to observe the
        current apply outcome (``AckEntry.ok``/``err_code``).

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
            elif key == "sTimeout":
                watchdog_value = int(value)

        targets_touched = sum([
            bool(drivetrain_patch), bool(motor_patch), watchdog_value is not None,
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

    # ------------------------------------------------------------------
    # Move -- DELETED (115-003, gut-to-minimal-firmware S1 motion-stack
    # excision). The sprint-109 host-adoption ticket's move()/
    # wait_for_move_terminal() sent envelope_pb2.Move (CommandEnvelope's
    # move oneof arm) and polled AckEntry.status against telemetry_pb2.
    # ACK_STATUS_DONE/friends -- both the Move message and the AckStatus
    # completion-status taxonomy are deleted along with Motion::Executor
    # (envelope.proto/telemetry.proto's own now-removed-message doc
    # comments). S1 has no MOVE command and no completion-event taxonomy to
    # poll; sprint 116's MOVE-protocol cutover reintroduces a `Move`-shaped
    # arm at a FRESH field number (21, never 20) with its own host builder.
    # ------------------------------------------------------------------

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "AckEntry | None":  # [ms]
        """Poll incoming ``Telemetry`` pushes for the single ack slot
        matching ``corr_id``, for up to ``timeout`` ms. Returns the matched
        ``AckEntry``, or ``None`` if the deadline passes with no match —
        this wait is always bounded, never infinite.

        The single-ack-slot matcher for the P4 "telemetry-only return path"
        (103-009 Decision 2, narrowed from a depth-3 ring to one slot by
        115-003 frame v2): ``twist()``/``stop()``/``config()`` get no
        synchronous ``ReplyEnvelope`` of their own — their outcome rides
        ``Telemetry.ack_corr``/``ack_err`` (valid iff ``flags`` bit 5,
        ``ack_fresh``) inside a subsequent ``Telemetry`` push. Because a
        command acked within the same primary period as another OVERWRITES
        the slot (the "ack-depth-1 tradeoff", stakeholder-accepted), a
        caller that fires closely-spaced commands may occasionally see this
        method time out for one of them — the documented, accepted
        consequence of dropping the depth-3 ring; retry on timeout covers
        it, matching this method's own bounded-wait contract.

        104-003: the actual poll/match/timeout loop is no longer inline
        here — it lives in ``SerialConnection.wait_for_ack()`` (see that
        method's own docstring for the full algorithm) so every future
        caller reading telemetry directly off ``SerialConnection`` — not
        just ``NezhaProtocol`` — gets the identical matching guarantee
        without a second copy of the algorithm. This method is a thin
        adapter: delegate to the shared implementation, then wrap the
        matched raw ``telemetry_pb2.Telemetry`` frame's ack slot in this
        module's own ``AckEntry`` dataclass (the same adaptation
        ``TLMFrame.from_pb2()`` performs for telemetry frames generally).
        """
        raw_telemetry = self._conn.wait_for_ack(corr_id, timeout=timeout)
        if raw_telemetry is None:
            return None
        return AckEntry.from_telemetry(raw_telemetry)

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
