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
``move``/``config``/``stop`` — every earlier arm (ping/echo/id/hello/ver/
help/get/drive/segment/replace/motion/pose_fix/otos/stream/plan_dump) was
pruned by 103-001's schema prune and is `reserved`, not reused (see
``envelope.proto``'s own header comment). 116-001 (MOVE protocol cutover)
replaced the interim ``twist`` arm (103-001) with ``move`` — a single
bounded motion command (velocity variant + stop condition + required
``timeout`` backstop + a ``replace`` flag against a small on-chip queue);
``twist`` (field 19) is `reserved`, not reused — see ``move_twist()``/
``move_wheels()`` below. There is no per-command synchronous reply for
``move``/``config``/``stop`` — a command's outcome rides the ack ring
inside the next ``Telemetry`` push (``wait_for_ack()``).

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
# planner.proto by 115-003, unchanged shape) to a single-character mode= wire
# value the historical text-plane TLM parser read off a text STREAM/SNAP
# frame's "mode=" token. Any DriveMode value with no entry here falls back to
# "I" via .get()'s own default in from_pb2(), so a future DriveMode value
# added to telemetry.proto without a matching entry here falls back safely
# instead of raising.
#
# 116-007: VELOCITY (set by RobotLoop while a MOVE is actively driving,
# `driving_ ? VELOCITY : IDLE`) previously had no entry and silently fell
# back to "I" -- the SAME character IDLE produces -- so a host-side reader
# (e.g. tlm_log.py's `mode` column) could never distinguish "driving" from
# "idle" by this column alone (confirmed on the sim dry-run, see
# docs/bench-checklists/sprint-115-gut-s1.md). "V" is unused by every other
# entry below (I/S/T/D/G) and is now VELOCITY's own dedicated character.
_DRIVE_MODE_CHAR = {
    telemetry_pb2.IDLE: "I",
    telemetry_pb2.STREAMING: "S",
    telemetry_pb2.TIMED: "T",
    telemetry_pb2.DISTANCE: "D",
    telemetry_pb2.GO_TO: "G",
    telemetry_pb2.VELOCITY: "V",
}


@dataclass(frozen=True)
class AckEntry:
    """One command's ack outcome, adapted onto a plain host-side shape the
    same way ``TLMFrame`` adapts ``Telemetry`` itself — either the single
    "freshest ack" scalar slot (``Telemetry.ack_corr``/``ack_err``,
    ``from_telemetry()`` below) or one entry from the bounded ack ring
    (``Telemetry.acks``, 120, ``from_ring_entry()`` below).

    Reports the outcome of ONE previously-sent command (matched by
    ``corr_id``) — the P4 wire has no per-command synchronous
    ``ReplyEnvelope`` for ``move``/``stop``/``config``, so telemetry is the
    ONLY place their outcome is reported. ``err_code == 0`` (``ok=True``)
    means OK; nonzero is the raw ``ErrCode`` (envelope.proto) value — the
    same two-value shape every command outcome has always produced.

    115-003 frame v2 deleted the pre-115 depth-3 wire ``AckEntry``
    ring/``AckStatus`` enum (OK/ERR/DONE/TRIVIAL/SUPERSEDED/FLUSHED/
    TIMEOUT/SOLVE_FAIL, the deleted executor's own completion taxonomy) in
    favor of one scalar slot. 120 (bench-single-ack-slot-observability-
    collapses-at-40ms.md) brings a wire ``AckEntry`` message back — a
    bounded, depth-4, corr_id+err ring, NOT a revival of the old
    ``AckStatus`` taxonomy (still plain OK/ERR) or of "overwrite is a
    tradeoff, not a bug" (the ring simply does not overwrite until it is
    genuinely full and evicting the OLDEST entry, so a same-primary-period
    collision that used to overwrite the single slot now survives in the
    ring instead). ``ack_corr``/``ack_err``/``flags`` bit 5 keep their
    EXACT prior meaning unchanged — this dataclass now has two possible
    origins, not two possible meanings.
    """
    corr_id: int
    ok: bool
    err_code: int  # raw ErrCode (envelope.proto) value when ok is False, else 0 (ERR_NONE)

    @classmethod
    def from_telemetry(cls, telemetry: "telemetry_pb2.Telemetry") -> "AckEntry":
        """Build an ``AckEntry`` from ``telemetry``'s single "freshest ack"
        scalar slot (``ack_corr``/``ack_err``) — UNCHANGED by 120. Call only
        when ``flags`` bit 5 (``ack_fresh``) is set — this method does not
        itself check the bit (``TLMFrame.from_pb2()`` is the one remaining
        call site, and it already gates on it)."""
        return cls(
            corr_id=int(telemetry.ack_corr),
            ok=(telemetry.ack_err == 0),
            err_code=int(telemetry.ack_err),
        )

    @classmethod
    def from_ring_entry(cls, entry: "telemetry_pb2.AckEntry") -> "AckEntry":
        """Build an ``AckEntry`` from one entry of the bounded ack ring
        (``Telemetry.acks``, 120) — ``entry`` is a raw
        ``telemetry_pb2.AckEntry`` (``corr_id``/``err``), not the whole
        ``Telemetry`` frame it rode in on. Unlike ``from_telemetry()``
        above, no freshness gate applies: a corr_id present in the ring was
        genuinely pushed by ``App::Telemetry::ack()`` at some point — see
        this class's own docstring and ``SerialConnection.wait_for_ack()``'s
        docstring for why the ring needs no ``ack_fresh``-style bit."""
        return cls(
            corr_id=int(entry.corr_id),
            ok=(entry.err == 0),
            err_code=int(entry.err),
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
_FLAG_FAULT_SHAPING_DISABLED = 1 << 16


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
      - ``fault_shaping_disabled`` (bit 16) — a MOVE is active AND both
        angular and linear ``ShaperLimits`` axes are disabled (119 ticket
        001, kill-the-silent-off-shaping-config-boundary.md) — the loud
        off-state for the shaping/anticipation silent-off config
        boundary: with no taper, the land-at-zero completion path can
        never fire and the threshold/timeout backstop is the ONLY
        completion path.
    These properties are the ticket's own "existing downstream consumer
    keeps working unchanged" surface — grep ``src/host/robot_radio/`` for
    every attribute name the pre-115 standalone bool/bitmask fields
    exposed before renaming or removing any of them.

    ``ack_corr``/``ack_err`` are the wire's single "freshest ack" slot
    (raw, always populated), valid iff ``ack_fresh``. ``ack`` is the SAME
    slot pre-decoded into an ``AckEntry`` (``None`` unless ``ack_fresh``
    was set on this frame) — the convenience shape most callers want.
    UNCHANGED by 120 — every existing reader of these three keeps working.

    ``acks`` (120, ADDITIVE) is the bounded ack ring
    (``telemetry.proto``'s ``Telemetry.acks``, depth 4) decoded as a plain
    ``list[AckEntry]``, oldest-pushed first, ALWAYS populated (may be
    empty) — independent of ``ack_fresh``, since a ring entry needs no
    freshness gate (see ``AckEntry.from_ring_entry()``'s own docstring).
    ``wait_for_ack()`` (``SerialConnection``/``NezhaProtocol``) scans this
    ring, not the single slot, to find a specific ``corr_id`` reliably
    across a bounded-but-real burst of rapid-fire commands; ``acks`` is
    exposed here too for a caller (bench scripts, ``tlm_log.py``) that
    wants to inspect the whole ring directly, per-frame.

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
    ack_corr: int | None = None                  # raw ack_corr [uint32]; valid iff ack_fresh (the single "freshest ack" slot, UNCHANGED by 120)
    ack_err: int | None = None                   # raw ack_err (envelope.proto ErrCode); valid iff ack_fresh
    ack: "AckEntry | None" = None                 # ack_corr/ack_err pre-decoded, populated ONLY when ack_fresh is set
    acks: "list[AckEntry]" = field(default_factory=list)  # bounded ack ring (120), oldest-first, ALWAYS populated (may be empty), no freshness gate
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
    def fault_shaping_disabled(self) -> bool:
        """Bit 16 (119 ticket 001) -- a MOVE is active AND both angular and
        linear ``ShaperLimits`` axes are disabled -- see this class's own
        docstring and ``telemetry.h``'s ``kFlagFaultShapingDisabled`` doc
        comment for the full rationale."""
        return self._flag(_FLAG_FAULT_SHAPING_DISABLED)

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

        # Bounded ack ring (120, ADDITIVE) -- ALWAYS populated (may be
        # empty), independent of ack_fresh; oldest-pushed first, matching
        # the wire's own push/evict order (telemetry.cpp's ack() doc
        # comment).
        frame.acks = [AckEntry.from_ring_entry(entry) for entry in telemetry.acks]

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

# ml/mr are handled specially, not via a plain field-name map: both patch
# MotorConfigPatch.travel_calib, disambiguated by `side` (Decision 5,
# config.proto) -- ml=LEFT, mr=RIGHT.
#
# sTimeout -- DELETED (116-001, MOVE protocol cutover): patched ConfigDelta's
# bare `watchdog` oneof arm (uint32 sTimeout, the pre-116 StreamingDrive
# Watchdog window), which is itself deleted -- every Move is now
# self-bounding (its own stop condition or required `timeout`), so the
# separate deadman/watchdog window this key configured is gone along with
# `App::Deadman` (see envelope.proto's own `ConfigDelta.watchdog` doc
# comment). `sTimeout` is simply no longer a valid set_config()/config() key
# -- both now raise/return the same "unknown key" outcome any other bogus
# key already produced.
_ALL_SET_KEYS = frozenset(
    set(_DRIVETRAIN_KEYS) | set(_MOTOR_PID_KEYS) | {"ml", "mr"})


def _format_config_value(value: Any) -> str:
    """Format a set_config() kwarg value into the SAME string shape the
    text plane's set_config() already produced -- floats to 6 significant
    digits, everything else via str(). Reused as-is from the pre-097-002
    text implementation (the formatting rule itself did not change)."""
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


# ---------------------------------------------------------------------------
# Move builder support (116-001, MOVE protocol cutover) -- shared by
# NezhaProtocol.move_twist()/move_wheels(), which differ only in which
# Move.velocity oneof arm (twist/wheels) they build. Move.stop is itself a
# oneof (time/distance/angle, envelope.proto) -- both host builders expose it
# as three separate, mutually-exclusive keyword-only args (stop_time/
# stop_distance/stop_angle) rather than a single generic "kind+value" arg, so
# each carries its own unit as a `# [unit]` tag on its own parameter (project
# naming convention, .claude/rules/coding-standards.md) instead of one
# ambiguous-unit parameter whose meaning depends on a second value.
# ---------------------------------------------------------------------------

def _build_move_stop_kwargs(*, stop_time: float | None, stop_distance: float | None,
                            stop_angle: float | None) -> dict[str, float]:
    """Validate and translate a move_twist()/move_wheels() caller's
    stop_time/stop_distance/stop_angle kwargs into the single
    ``{"time"|"distance"|"angle": value}`` kwarg ``envelope_pb2.Move()``'s
    own ``stop`` oneof constructor expects.

    Raises ``ValueError`` (no wire traffic) unless EXACTLY ONE of the three
    is given -- ``Move.stop`` is a oneof, so zero is an underspecified Move
    and more than one is unrepresentable on the wire."""
    candidates = {"time": stop_time, "distance": stop_distance, "angle": stop_angle}
    given = {k: v for k, v in candidates.items() if v is not None}
    if len(given) != 1:
        raise ValueError(
            "move requires exactly one stop condition (stop_time/"
            f"stop_distance/stop_angle), got {sorted(given)!r}")
    (key, value), = given.items()
    return {key: float(value)}


# ---------------------------------------------------------------------------
# NezhaProtocol
# ---------------------------------------------------------------------------

class NezhaProtocol:
    """Binary wire-protocol adapter for the P4 single-loop Nezha firmware.

    Owns a SerialConnection and exposes one method per firmware command group
    (``move``/``stop``/``config`` — ``move_twist()``/``move_wheels()`` build
    the ``move`` arm's two velocity variants) plus telemetry read accessors.
    All response parsing delegates to module-level parse_* functions so
    callers can reuse them on lines received through other paths (streaming
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
        so kwargs spanning multiple targets (e.g. tw= + pid.kp=) become
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

        if not ok:
            return None
        return {key: _format_config_value(value) for key, value in kwargs.items()}

    # ------------------------------------------------------------------
    # Config: binary SET (096-007, M6 Host Config/Telemetry Client)
    # ------------------------------------------------------------------

    def set_config_binary(self, delta: "envelope_pb2.ConfigDelta",
                          read_timeout: int = 500) -> "AckEntry | None":  # [ms]
        """Send CommandEnvelope{config: delta}; return the matched AckEntry,
        or None (timeout, not connected, or a NAK reply).

        ``delta`` is a fully-built ``pb2.ConfigDelta`` — its own oneof
        (``drivetrain``/``motor``/``watchdog``/``otos``) selects which
        config slice is being patched, mirroring BinaryChannel's CONFIG arm
        1:1. Building ``delta`` is the caller's job (e.g.
        ``envelope_pb2.ConfigDelta(drivetrain=config_pb2.
        DrivetrainConfigPatch(trackwidth=128.0))``) — this method's only
        job is the envelope round trip.

        Bench fix (2026-07-22, stakeholder finding): this method used to
        send via the BLOCKING ``_send_envelope()``/``send_envelope()`` (a
        synchronous request/reply, matched by ``envelope.corr_id`` in
        ``SerialConnection._reply_queues``) and look for a synchronous
        ``ReplyEnvelope{ok: ...}``. The current single-loop firmware never
        sends one for ANY command — ``config``'s outcome, like
        ``move``/``stop``, rides the single ack slot inside the NEXT
        ``Telemetry`` push instead (``docs/protocol-v4.md`` sec 7.1: "a
        wire sniffer will never observe a ``ReplyEnvelope{ok:...}``... only
        ``ReplyEnvelope{tlm: Telemetry}``"), so this method's old
        implementation timed out on EVERY call against real hardware —
        confirmed on the bench: a robot-select calibration push logged 9/12
        ``SET`` keys "ERR badarg set failed" (all 9 routed through this
        method; the other 3, ``OI``/``OL``/``OA``, use the ALREADY fire-
        and-poll ``otos_config()`` and worked). This predates the 103-009
        fire-and-poll migration ``move_twist()``/``move_wheels()``/
        ``stop()`` went through, and was never carried forward when
        ``config()`` (104-001, written AFTER 103-009) got it from day one
        — see that method's own docstring. Fixed the same way: send via
        ``send_envelope_fast()``, then poll for the completion via the ack
        ring.

        Duck-typed ack lookup, not a hardcoded ``self.wait_for_ack()``
        call: ``self._conn`` may be a ``_SimConfigConn`` (``io/
        sim_config.py``, backing ``SimLoop.configure_from_robot()``'s own
        Tier-1 push), which deliberately has NO ``wait_for_ack()`` of its
        own (its own docstring: ``NezhaProtocol.wait_for_ack()`` expects a
        RAW ``telemetry_pb2.Telemetry`` off ``self._conn.wait_for_ack()``,
        but ``SimLoop`` only ever hands back already-adapted ``TLMFrame``/
        ``AckEntry`` dataclasses) — it exposes the SAME ack-ring lookup as
        its own ``poll_ack(corr_id, timeout)``, already returning an
        ``AckEntry``. A connection exposing ``poll_ack`` (Sim) is polled
        that way; anything else (a real ``SerialConnection``) goes through
        ``self.wait_for_ack()`` as usual.
        """
        envelope = envelope_pb2.CommandEnvelope(config=delta)
        corr_id = self._conn.send_envelope_fast(envelope)
        poll_ack = getattr(self._conn, "poll_ack", None)
        ack = poll_ack(corr_id, timeout=read_timeout) if poll_ack is not None \
            else self.wait_for_ack(corr_id, timeout=read_timeout)
        if ack is None or not ack.ok:
            return None
        return ack

    # ------------------------------------------------------------------
    # Drive commands
    # ------------------------------------------------------------------

    def move_twist(self, v_x: float, v_y: float, omega: float, *,
                   stop_time: float | None = None,       # [ms]
                   stop_distance: float | None = None,   # [mm]
                   stop_angle: float | None = None,       # [rad]
                   timeout: float,                        # [ms]
                   replace: bool = True,
                   move_id: int = 0) -> int:
        """Enqueue (or preempt-and-start) a bounded body-frame twist MOVE —
        one of the P4 wire's two ``Move`` velocity variants
        (``CommandEnvelope{move: Move{twist: MoveTwist{v_x, v_y, omega},
        ...}}}``, envelope.proto arm 21, 116-001 MOVE protocol cutover).
        Supersedes the deleted 103-era ``twist()`` (bare ``v_x``/``omega`` +
        deadman-arming ``duration``): every ``Move`` is now bounded by its
        own stop condition and a required ``timeout`` backstop instead of a
        separate watchdog module (``App::Deadman`` no longer exists).

        ``v_y`` is accepted and wire-forwarded but ignored server-side on
        this differential build (``MoveTwist.v_y``'s own doc comment) —
        pass ``0.0`` unless a future holonomic drivetrain needs it.

        Exactly ONE of ``stop_time``/``stop_distance``/``stop_angle`` selects
        this Move's stop condition (``Move``'s ``stop`` oneof) — elapsed
        time since activation, |path arc length| since activation (encoder
        odometry), or |heading change| since activation (encoder odometry),
        respectively. Passing zero or more than one raises ``ValueError``,
        no wire traffic sent — the oneof can carry only one.

        ``timeout`` is the REQUIRED safety backstop (envelope.proto: "<=0 ->
        ERR_BADARG") that fires the Move even if the stop condition can
        never be reached (e.g. stalled wheels) — validated host-side
        (``ValueError`` for a non-positive value) to avoid a wasted wire
        round trip for a command the firmware would reject anyway.

        ``replace`` selects queue semantics against ``App::MoveQueue`` (1
        active + 4 pending): ``True`` (the default — matches every existing
        caller's own pre-Move "just drive this now" usage) flushes pending
        and preempts the active Move, starting this one immediately;
        ``False`` enqueues behind the active Move (``ERR_FULL`` if 4 already
        pending). ``move_id`` is echoed back in this Move's own COMPLETION
        ack (``Move.id`` — distinct from the enqueue ack, which echoes this
        envelope's ``corr_id`` as usual); the default ``0`` is fine for a
        caller that does not need to distinguish completion acks.

        Fire-and-poll, NOT fire-and-wait (103-009, Decision 2's
        "telemetry-only return path", unchanged by the Move cutover): the
        P4 wire has no per-command synchronous ``ReplyEnvelope`` for
        ``move`` — this call's own ENQUEUE outcome arrives later, riding the
        ack ring inside a subsequent ``Telemetry`` push (see
        ``wait_for_ack()``). This method returns as soon as the bytes reach
        the wire; it never blocks waiting for a reply that will not come.

        Returns the corr_id assigned to this command — pass it to
        ``wait_for_ack()`` to confirm the firmware accepted it. Raises
        ``ConnectionError`` if not connected (``send_envelope_fast()``'s own
        not-open contract).
        """
        stop_kwargs = _build_move_stop_kwargs(
            stop_time=stop_time, stop_distance=stop_distance, stop_angle=stop_angle)
        if timeout <= 0:
            raise ValueError(f"move_twist(): timeout must be > 0, got {timeout!r}")
        move = envelope_pb2.Move(
            twist=envelope_pb2.MoveTwist(v_x=v_x, v_y=v_y, omega=omega),
            timeout=timeout, replace=replace, id=move_id, **stop_kwargs)
        envelope = envelope_pb2.CommandEnvelope(move=move)
        return self._conn.send_envelope_fast(envelope)

    def move_wheels(self, v_left: float, v_right: float, *,
                    stop_time: float | None = None,       # [ms]
                    stop_distance: float | None = None,   # [mm]
                    stop_angle: float | None = None,       # [rad]
                    timeout: float,                        # [ms]
                    replace: bool = True,
                    move_id: int = 0) -> int:
        """Enqueue (or preempt-and-start) a bounded per-wheel-speed MOVE —
        the ``Move`` velocity variant's OTHER branch alongside
        ``move_twist()`` (``CommandEnvelope{move: Move{wheels: MoveWheels{
        v_left, v_right}, ...}}}``, envelope.proto arm 21). Stages directly
        through ``Drive::setWheels()`` firmware-side — never translated
        through a twist round trip (sprint 116's architecture-update.md
        Decision 3) — the bench rig's own per-motor-pair driving idiom
        (``.clasi/knowledge/bench-test-rig-layout.md``).

        ``stop_time``/``stop_distance``/``stop_angle``/``timeout``/
        ``replace``/``move_id`` share the SAME contract as ``move_twist()``'s
        own (exactly one stop condition; ``timeout`` required and validated
        > 0 host-side; ``replace`` defaults ``True``) — see that method's
        docstring for the full rationale; not re-derived here.

        Fire-and-poll, the SAME shape as ``move_twist()``/``stop()`` (103-009
        Decision 2's "telemetry-only return path"): this call writes the
        bytes and returns immediately; its ENQUEUE outcome rides the ack
        ring (``wait_for_ack()``).

        Returns the corr_id assigned to this command. Raises
        ``ConnectionError`` if not connected; raises ``ValueError`` for a
        missing/ambiguous stop condition or a non-positive ``timeout``.
        """
        stop_kwargs = _build_move_stop_kwargs(
            stop_time=stop_time, stop_distance=stop_distance, stop_angle=stop_angle)
        if timeout <= 0:
            raise ValueError(f"move_wheels(): timeout must be > 0, got {timeout!r}")
        move = envelope_pb2.Move(
            wheels=envelope_pb2.MoveWheels(v_left=v_left, v_right=v_right),
            timeout=timeout, replace=replace, id=move_id, **stop_kwargs)
        envelope = envelope_pb2.CommandEnvelope(move=move)
        return self._conn.send_envelope_fast(envelope)

    def move(self, *, v_x: float = 0.0, v_y: float = 0.0, omega: float = 0.0,
             v_left: float | None = None, v_right: float | None = None,
             stop_time: float | None = None,       # [ms]
             stop_distance: float | None = None,   # [mm]
             stop_angle: float | None = None,       # [rad]
             timeout: float,                        # [ms]
             replace: bool = True, id: int | None = None) -> int:
        """Single-entry-point ``Move`` builder mirroring
        ``robot_radio.io.sim_loop.SimLoop.move()``'s own kwargs exactly
        (testgui-motion-paths-dead-after-move-cutover fix, planner.tour
        revival) -- ``planner.tour``'s ``MoveTransport`` Protocol calls
        `.move(**kwargs)` on whatever `.protocol` a transport exposes;
        ``_HardwareTransport.protocol`` returns THIS class, so without this
        method a live hardware connection could not run a tour (only
        ``SimTransport.protocol`` -- a ``SimLoop``, which already had
        ``.move()`` -- could). A thin dispatcher over the two methods this
        class already has: a velocity variant of ``v_left``/``v_right``
        (BOTH given) calls ``move_wheels()``; the default (``v_x``/``v_y``/
        ``omega``, ``v_left``/``v_right`` both ``None``) calls
        ``move_twist()``. Raises ``ValueError`` if only one of
        ``v_left``/``v_right`` is given -- mirrors ``SimLoop.move()``'s own
        guard.

        ``id`` maps to ``move_twist()``/``move_wheels()``'s own
        ``move_id`` parameter (``Move.id`` -- the key THIS Move's own
        COMPLETION ack echoes, per ``docs/protocol-v4.md`` section 7.2);
        defaults to ``0`` (their own default) when omitted. UNLIKE
        ``SimLoop.move()``, this does NOT also become the envelope's own
        ``corr_id`` -- ``move_twist()``/``move_wheels()`` auto-assign that
        separately (``send_envelope_fast()``'s own connection-scoped
        counter), so the RETURNED value here is that auto-assigned
        envelope ``corr_id`` (the ENQUEUE ack's own key), not ``id``. A
        caller polling for a Move's own completion (e.g. ``planner.tour``)
        must poll on ``id`` itself, never this return value -- see
        ``MoveTransport``'s own docstring (``planner/tour.py``) for why
        that distinction is transparent to a tour.

        ``stop_time``/``stop_distance``/``stop_angle``/``timeout``/
        ``replace`` share the SAME contract as ``move_twist()``'s own --
        see that method's docstring. Raises ``ConnectionError`` if not
        connected; raises ``ValueError`` for a missing/ambiguous stop
        condition, a non-positive ``timeout``, or a lone
        ``v_left``/``v_right``.
        """
        move_id = id if id is not None else 0
        if v_left is not None or v_right is not None:
            if v_left is None or v_right is None:
                raise ValueError(
                    "move(): v_left and v_right must both be given for a "
                    "wheels Move (got only one)")
            return self.move_wheels(
                v_left, v_right, stop_time=stop_time, stop_distance=stop_distance,
                stop_angle=stop_angle, timeout=timeout, replace=replace, move_id=move_id)
        return self.move_twist(
            v_x, v_y, omega, stop_time=stop_time, stop_distance=stop_distance,
            stop_angle=stop_angle, timeout=timeout, replace=replace, move_id=move_id)

    def stop(self) -> int:
        """Panic-stop the drivetrain (``CommandEnvelope{stop: Stop{}}``) — a
        zero-field oneof arm that "cannot be malformed" (envelope.proto
        Decision 3).

        Fire-and-poll, the SAME shape as ``move_twist()``/``move_wheels()``
        (103-009, see their docstrings for why): the P4 firmware reports
        ``stop``'s outcome via the ack ring (``wait_for_ack()``), not a
        synchronous reply, so this call writes the STOP bytes and returns
        immediately rather than blocking on a reply that will not come.

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
        config: delta}``, ``envelope.proto`` field 6) — one of the P4 wire's
        three ``cmd`` oneof arms, alongside ``move_twist()``/``move_wheels()``/
        ``stop()`` (``envelope.proto``'s own oneof comment: "config/stop keep
        their pre-102 field numbers... move is genuinely new"). 104-001 is
        what gives ``config`` a host-side builder — before it, every OTHER
        oneof arm had one but ``config`` did not, despite being schema-defined
        since 103-001.

        Fire-and-poll, the SAME shape as ``move_twist()``/``move_wheels()``/
        ``stop()`` (103-009, Decision 2's "telemetry-only return path"): a
        ``config`` command's outcome rides the ack ring inside a subsequent
        ``Telemetry`` push, never a synchronous ``ReplyEnvelope`` — see
        ``wait_for_ack()``. This method writes the bytes and returns
        immediately.

        ``deltas`` reuses the SAME flat wire-key vocabulary ``set_config()``
        curates (module-level ``_DRIVETRAIN_KEYS``/``_MOTOR_PID_KEYS``/
        ``ml``/``mr`` — together ``_ALL_SET_KEYS``), so a key added to one map
        is automatically available to the other; nothing here re-derives that
        vocabulary. UNLIKE ``set_config()``, which fans a multi-target kwargs
        dict out into MULTIPLE round trips (one per touched
        ``ConfigDelta.patch`` oneof arm, since a single ``ConfigDelta``
        carries only one patch at a time), ``config()`` builds and sends
        exactly ONE ``CommandEnvelope`` carrying exactly ONE ``ConfigDelta``
        — matching ``move_twist()``/``move_wheels()``/``stop()``'s own "one
        call, one envelope, one corr_id" shape. Passing kwargs that span more
        than one ``ConfigDelta.patch`` target (e.g. ``tw=`` and ``pid.kp=``
        together — drivetrain vs. motor) is a caller error: raises
        ``ValueError``, since no single ``ConfigDelta`` could carry both.
        Same for empty ``deltas`` or a key outside the known vocabulary.
        ``pid.*`` keys and ``ml``/``mr`` may be mixed freely in one call —
        both target the SAME ``MotorConfigPatch`` oneof arm (mirroring
        ``set_config()``'s own ``motor_left_patch``/``motor_right_patch``
        grouping); ``side`` selects ``travel_calib``'s target only and is
        meaningless for the ``pid.*`` fields (``config.proto``'s own
        ``MotorConfigPatch.side`` comment), so a pure-``pid.*`` call (no
        ``ml``/``mr``) still needs SOME side value on the wire — it defaults
        to ``LEFT``, the same default ``set_config()``'s own
        ``motor_left_patch`` branch always used.

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

        targets_touched = sum([bool(drivetrain_patch), bool(motor_patch)])
        if targets_touched > 1:
            raise ValueError(
                "config(): kwargs span more than one ConfigDelta.patch "
                f"target (got {sorted(deltas)}) — a single ConfigDelta "
                "carries only one patch; call config() once per target")

        if drivetrain_patch:
            delta = envelope_pb2.ConfigDelta(
                drivetrain=config_pb2.DrivetrainConfigPatch(**drivetrain_patch))
        else:
            delta = envelope_pb2.ConfigDelta(motor=config_pb2.MotorConfigPatch(
                side=motor_side, **motor_patch))

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

        Fire-and-poll, the SAME shape as ``move_twist()``/``move_wheels()``/
        ``stop()``/``config()`` (103-009's "telemetry-only return path"):
        this call writes the bytes and returns immediately; its outcome
        rides the ack ring (``wait_for_ack()``).

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

    def estimator_config(self, *, weight_heading_otos: float | None = None,
                          weight_omega_otos: float | None = None,
                          staleness_ms: float | None = None,
                          a_max: float | None = None,
                          a_decel: float | None = None,
                          alpha_max: float | None = None,
                          alpha_decel: float | None = None,
                          j_max: float | None = None,
                          yaw_jerk_max: float | None = None) -> int:
        """Build and send an ``EstimatorConfigPatch`` ``ConfigDelta`` envelope
        (``CommandEnvelope{config: ConfigDelta{estimator: ...}}``, 117 ticket
        003) — the live-tuning surface for ``App::StateEstimator``'s v1
        complementary-blend fusion weights, mirroring ``otos_config()``'s own
        "build exactly ONE envelope carrying exactly ONE patch" shape
        exactly.

        ``weight_heading_otos``/``weight_omega_otos`` map 1:1 to
        ``App::StateEstimator::FusionWeights::headingOtos``/``omegaOtos``
        (dimensionless ``[0..1]`` complementary-blend weights, baked
        fail-closed to ``0.0`` by ``Config::defaultEstimatorConfig()`` this
        sprint — encoder-only v1, per stakeholder decision);
        ``staleness_ms`` maps to ``FusionWeights::staleness`` (the max age,
        ms, a fresh OTOS reading may carry and still be eligible to blend).

        ``a_max``/``a_decel``/``alpha_max``/``alpha_decel``/``j_max``/
        ``yaw_jerk_max`` (decel-into-the-goal campaign, jerk-limited
        S-curve stage) map to ``App::MoveQueue::shaperLimits_``
        (``App::MoveQueue::setShaperLimits()``, ``App::ShaperLimits``) —
        riding the SAME ``CONFIG_ESTIMATOR`` wire arm as the three fields
        above, the "smallest coherent path" ``EstimatorConfigPatch``'s own
        doc comment (``config.proto``) gives; ``a_max``/``a_decel`` are
        ``[mm/s^2]`` linear accel-ramp/decel-taper ceilings, ``alpha_max``/
        ``alpha_decel`` are ``[rad/s^2]`` angular ones, ``j_max``/
        ``yaw_jerk_max`` are ``[mm/s^3]``/``[rad/s^3]`` jerk ceilings —
        how fast the commanded ACCELERATION itself may change
        (``Motion::VelocityShaper``'s own limits). Any subset may be set
        alone.

        A former field targeting ``App::MoveQueue``'s own time-lead
        anticipation constant was DELETED (118 ticket 004,
        land-at-zero-completion-delete-stop-lead.md) — the completion
        mechanism it drove no longer exists (see
        ``App::MoveQueue::tick()``'s own doc comment for the land-at-zero
        predicate that replaces it); the wire field is ``reserved`` in
        ``config.proto``, not reused.

        UNLIKE ``otos_config()``, a patch sent through this method is NEVER
        persisted on the robot side — ``RobotLoop::handleConfig()``'s
        ``ESTIMATOR`` branch applies it live but never writes it into
        ``persistedTuning_``/flash (Design Rationale Decision 4, sprint
        117's overlay ``design/design.md``): a reboot always reverts to the
        baked JSON default, never this method's last-sent value.

        Fire-and-poll, the SAME shape as ``move_twist()``/``move_wheels()``/
        ``stop()``/``config()``/``otos_config()`` (103-009's "telemetry-only
        return path"): this call writes the bytes and returns immediately;
        its outcome rides the ack slot (``wait_for_ack()``).

        Returns the corr_id assigned to this command. Raises
        ``ConnectionError`` if not connected; raises ``ValueError`` if no
        field is set at all (every kwarg ``None`` — an empty patch is a
        caller error, mirroring ``otos_config()``'s own empty-patch
        rejection).
        """
        fields: dict[str, Any] = {}
        if weight_heading_otos is not None:
            fields["weight_heading_otos"] = float(weight_heading_otos)
        if weight_omega_otos is not None:
            fields["weight_omega_otos"] = float(weight_omega_otos)
        if staleness_ms is not None:
            fields["staleness_ms"] = float(staleness_ms)
        if a_max is not None:
            fields["a_max"] = float(a_max)
        if a_decel is not None:
            fields["a_decel"] = float(a_decel)
        if alpha_max is not None:
            fields["alpha_max"] = float(alpha_max)
        if alpha_decel is not None:
            fields["alpha_decel"] = float(alpha_decel)
        if j_max is not None:
            fields["j_max"] = float(j_max)
        if yaw_jerk_max is not None:
            fields["yaw_jerk_max"] = float(yaw_jerk_max)

        if not fields:
            raise ValueError(
                "estimator_config() requires at least one field "
                "(weight_heading_otos/weight_omega_otos/staleness_ms/"
                "a_max/a_decel/alpha_max/alpha_decel/j_max/yaw_jerk_max)")

        delta = envelope_pb2.ConfigDelta(
            estimator=config_pb2.EstimatorConfigPatch(**fields))
        envelope = envelope_pb2.CommandEnvelope(config=delta)
        return self._conn.send_envelope_fast(envelope)

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "AckEntry | None":  # [ms]
        """Poll incoming ``Telemetry`` pushes' bounded ack ring for an entry
        matching ``corr_id``, for up to ``timeout`` ms. Returns the matched
        ``AckEntry``, or ``None`` if the deadline passes with no match —
        this wait is always bounded, never infinite.

        The ack-ring matcher (120, bench-single-ack-slot-observability-
        collapses-at-40ms.md — replaces the pre-120 single-scalar-slot
        matcher this method used): ``move_twist()``/``move_wheels()``/
        ``stop()``/``config()`` get no synchronous ``ReplyEnvelope`` of
        their own — their outcome rides ``Telemetry.acks`` (a depth-4
        ring, each entry a real, once-pushed ``App::Telemetry::ack()``
        call) inside a subsequent ``Telemetry`` push. The pre-120 single
        slot (``ack_corr``/``ack_err``, valid iff ``flags`` bit 5) lost a
        command's ack the instant a LATER command's ack landed within the
        same primary period, before the host's next read — bench-measured
        as 12/43 lost transient acks at the real 40ms cycle / ~15Hz host
        read rate. The ring survives up to ``kAckRingDepth`` (4) OTHER
        acks landing before this one is read; only a burst of MORE than 4
        unread acks for corr_ids other than this one would still time this
        out (unchanged bounded-wait contract; retry covers even that rare
        case).

        104-003: the actual poll/match/timeout loop is no longer inline
        here — it lives in ``SerialConnection.wait_for_ack()`` (see that
        method's own docstring for the full ring-scan algorithm) so every
        future caller reading telemetry directly off ``SerialConnection``
        — not just ``NezhaProtocol`` — gets the identical matching
        guarantee without a second copy of the algorithm. This method is a
        thin adapter: delegate to the shared implementation, then wrap the
        matched raw ``telemetry_pb2.AckEntry`` ring entry in this module's
        own ``AckEntry`` dataclass (``AckEntry.from_ring_entry()`` — NOT
        ``from_telemetry()``, which reads the single scalar slot instead;
        reading ``ack_corr``/``ack_err`` off the matched FRAME here would be
        wrong whenever a DIFFERENT, later command's ack became that frame's
        own "freshest ack" by the time it was read).
        """
        matched_entry = self._conn.wait_for_ack(corr_id, timeout=timeout)
        if matched_entry is None:
            return None
        return AckEntry.from_ring_entry(matched_entry)

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
