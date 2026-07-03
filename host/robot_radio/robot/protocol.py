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


# ---------------------------------------------------------------------------
# Public data types
# ---------------------------------------------------------------------------

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
    (x_mm, y_mm, heading_cdeg), arc-integrated from wheel deltas only —
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
    enc: tuple[int, int] | None = None          # (left_mm, right_mm)
    pose: tuple[int, int, int] | None = None    # (x_mm, y_mm, heading_cdeg)
    vel: tuple[int, ...] | None = None          # differential: (vL, vR); mecanum: (vFR, vFL, vBR, vBL) mm/s
    twist: tuple[int, ...] | None = None        # differential: (v, omega_mrad); mecanum: (vx, vy, omega_mrad)
    otos: tuple[int, int, int] | None = None    # (x_mm, y_mm, heading_cdeg) — raw OTOS pose
    line: tuple[int, int, int, int] | None = None   # (g1, g2, g3, g4)
    color: tuple[int, int, int, int] | None = None  # (r, g, b, c)
    ekf_rej: int | None = None                   # cumulative EKF gate rejection count
    wedge: tuple[int, int] | None = None         # (left, right) wedge-latch state, 0/1 each (064-004)
    encpose: tuple[int, int, int] | None = None  # (x_mm, y_mm, heading_cdeg) — encoder-only pose (068-001)
    otos_health: tuple[int, bool] | None = None  # (raw STATUS byte, fusion_blocked) — OTOS health (074-004)


@dataclass
class ParsedResponse:
    """Structured representation of a single response line from the firmware."""
    tag: str          # "OK", "ERR", "EVT", "TLM", "CFG", "ID"
    tokens: list[str] = field(default_factory=list)  # plain tokens after tag
    kv: dict[str, str] = field(default_factory=dict) # key=value pairs
    corr_id: str | None = None                       # trailing #<id>, if any
    raw: str = ""                                    # original stripped line


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
      stop=heading:<cdeg>:<eps_cdeg>
      stop=rot:<arc_mm>
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


def parse_tlm(line: str) -> TLMFrame | None:
    """Parse a TLM frame line into a TLMFrame dataclass, or None if not TLM."""
    resp = parse_response(line)
    if resp is None or resp.tag != "TLM":
        return None

    frame = TLMFrame()
    kv = resp.kv

    if "t" in kv:
        try:
            frame.t = int(kv["t"])
        except ValueError:
            pass

    if "mode" in kv:
        frame.mode = kv["mode"]

    if "seq" in kv:
        try:
            frame.seq = int(kv["seq"])
        except ValueError:
            pass

    if "wedge" in kv:
        try:
            parts = kv["wedge"].split(",")
            if len(parts) == 2:
                frame.wedge = (int(parts[0]), int(parts[1]))
        except ValueError:
            pass

    if "enc" in kv:
        try:
            parts = kv["enc"].split(",")
            if len(parts) == 2:
                frame.enc = (int(parts[0]), int(parts[1]))
        except ValueError:
            pass

    if "pose" in kv:
        try:
            parts = kv["pose"].split(",")
            if len(parts) == 3:
                frame.pose = (int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError:
            pass

    if "encpose" in kv:
        try:
            parts = kv["encpose"].split(",")
            if len(parts) == 3:
                frame.encpose = (int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError:
            pass

    if "vel" in kv:
        try:
            parts = kv["vel"].split(",")
            if len(parts) == 2:
                # Differential: (vL_mmps, vR_mmps)
                frame.vel = (int(parts[0]), int(parts[1]))
            elif len(parts) == 4:
                # Mecanum: (vFR_mmps, vFL_mmps, vBR_mmps, vBL_mmps)
                frame.vel = (int(parts[0]), int(parts[1]),
                             int(parts[2]), int(parts[3]))
        except ValueError:
            pass

    if "twist" in kv:
        try:
            parts = kv["twist"].split(",")
            if len(parts) == 2:
                # Differential: (v_mmps, omega_mradps)
                frame.twist = (int(parts[0]), int(parts[1]))
            elif len(parts) == 3:
                # Mecanum: (vx_mmps, vy_mmps, omega_mradps)
                frame.twist = (int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError:
            pass

    if "otos" in kv:
        try:
            parts = kv["otos"].split(",")
            if len(parts) == 3:
                frame.otos = (int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError:
            pass

    if "line" in kv:
        try:
            parts = kv["line"].split(",")
            if len(parts) == 4:
                frame.line = (int(parts[0]), int(parts[1]),
                              int(parts[2]), int(parts[3]))
        except ValueError:
            pass

    if "color" in kv:
        try:
            parts = kv["color"].split(",")
            if len(parts) == 4:
                frame.color = (int(parts[0]), int(parts[1]),
                               int(parts[2]), int(parts[3]))
        except ValueError:
            pass

    if "ekf_rej" in kv:
        try:
            frame.ekf_rej = int(kv["ekf_rej"])
        except ValueError:
            pass

    if "otos_health" in kv:
        try:
            parts = kv["otos_health"].split(",")
            if len(parts) == 2:
                frame.otos_health = (int(parts[0]), bool(int(parts[1])))
        except ValueError:
            pass

    return frame


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


def parse_cfg(line: str) -> dict[str, str] | None:
    """Parse a CFG response line into a key->value dict, or None if not CFG."""
    resp = parse_response(line)
    if resp is None or resp.tag != "CFG":
        return None
    return dict(resp.kv)


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

    @staticmethod
    def parse_tlm(line: str) -> TLMFrame | None:
        """Parse a TLM line into a TLMFrame. Delegates to module-level parse_tlm()."""
        return parse_tlm(line)

    @staticmethod
    def parse_cfg(line: str) -> dict[str, str] | None:
        """Parse a CFG line into a key->value dict. Delegates to parse_cfg()."""
        return parse_cfg(line)

    # ------------------------------------------------------------------
    # Liveness / identity
    # ------------------------------------------------------------------

    def ping(self, corr_id: str | None = None) -> tuple[int, float] | None:
        """Send PING, parse OK pong t=<robot_ms>.

        Returns (t_robot, rtt) or None if no valid response, both in ms.
        rtt is the round-trip time measured by this call.
        """
        cmd = "PING" if corr_id is None else f"PING #{corr_id}"
        t0 = time.monotonic()
        resp_dict = self._conn.send(cmd, read_timeout=500)
        t1 = time.monotonic()
        rtt = (t1 - t0) * 1000.0  # [ms]

        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "pong":
                if "t" in r.kv:
                    try:
                        return (int(r.kv["t"]), rtt)
                    except ValueError:
                        pass
        return None

    def echo(self, payload: str) -> str | None:
        """Send ECHO <payload>, return echoed payload string or None."""
        resp_dict = self._conn.send(f"ECHO {payload}", read_timeout=500)
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "echo":
                # Payload follows "OK echo " in the stripped line.
                prefix = "OK echo "
                s = _strip_relay(raw_line)
                if s.startswith(prefix):
                    return s[len(prefix):].rstrip()
        return None

    def get_id(self) -> dict[str, str] | None:
        """Send ID command. Returns kv dict (model, name, serial, fw, proto, caps) or None."""
        resp_dict = self._conn.send("ID", read_timeout=500)
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "ID":
                return dict(r.kv)
        return None

    def get_ver(self) -> dict[str, str] | None:
        """Send VER command. Returns kv dict (fw, proto) or None."""
        resp_dict = self._conn.send("VER", read_timeout=500)
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "ver":
                return dict(r.kv)
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
        """
        cmd = ("GET " + " ".join(keys)) if keys else "GET"
        resp_dict = self._conn.send(cmd, read_timeout=500)
        result: dict[str, str] = {}
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "CFG":
                result.update(r.kv)
        return result if result else None

    def set_config(self, **kwargs: Any) -> dict[str, str] | None:
        """Send SET key=value ..., parse OK set response.

        Returns dict of applied keys (from OK set response) or None.
        Floats are formatted with up to 6 significant digits.
        """
        if not kwargs:
            return None
        pairs = []
        for k, v in kwargs.items():
            if isinstance(v, float):
                pairs.append(f"{k}={v:.6g}")
            else:
                pairs.append(f"{k}={v}")
        cmd = "SET " + " ".join(pairs)
        resp_dict = self._conn.send(cmd, read_timeout=500)
        result: dict[str, str] = {}
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "set":
                result.update(r.kv)
        return result if result else None

    # ------------------------------------------------------------------
    # Drive commands
    # ------------------------------------------------------------------

    def stop(self) -> None:
        """Stop motors immediately (STOP command)."""
        self._conn.send_fast("STOP")

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
        """
        cmd = f"S {left} {right}"
        if stop:
            cmd += " " + " ".join(stop)
        self._conn.send_fast(cmd)

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
        """
        cmd = f"T {left} {right} {duration}"
        if sensor is not None:
            cmd += f" sensor={sensor}"
        if stop:
            cmd += " " + " ".join(stop)
        resp = self._conn.send(cmd, read_timeout=300)
        return resp.get("responses", [])

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
        """
        cmd = f"D {left} {right} {travel}"
        if sensor is not None:
            cmd += f" sensor={sensor}"
        if stop:
            cmd += " " + " ".join(stop)
        resp = self._conn.send(cmd, read_timeout=300)
        return resp.get("responses", [])

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

        Format: STREAM <ms>
        """
        self._conn.send(f"STREAM {period}", read_timeout=300)

    def stream_fields(self, fields: str) -> None:
        """Set TLM streaming with a field subset.

        Format: STREAM fields=enc,pose,line
        ``fields`` is a comma-separated string of field names.
        """
        self._conn.send(f"STREAM fields={fields}", read_timeout=300)

    def snap(self) -> "TLMFrame | None":
        """Request ONE telemetry frame synchronously and return it (parsed).

        SNAP returns the reply as a TLM frame (corr-id-less) routed to the
        TLM queue — NOT to the corr-id reply queue.  The old send()-based
        implementation waited on the corr-id queue and always timed out.

        Implementation:
        1. Drain any stale TLM frames already queued (to avoid returning a
           stale snapshot from a previous SNAP or stream burst).
        2. Fire SNAP via send_fast() — no corr-id wait.
        3. Poll read_lines() for up to 400 ms, which drains _tlm_queue, and
           return the first line that parse_tlm() accepts.
        """
        # Step 1: drain stale frames so we get a fresh snapshot.
        self._conn.read_pending_lines()
        # Step 2: fire SNAP with no corr-id reply wait.
        self._conn.send_fast("SNAP")
        # Step 3: read from the TLM queue path until we get a parseable frame.
        lines = self._conn.read_lines(duration=400)
        for ln in lines:
            f = parse_tlm(ln)
            if f is not None:
                return f
        return None

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
                last_send = _resend_if_due(last_send)
        except GeneratorExit:
            try:
                self._conn.send_fast("STOP")
                self.stream(0)
            except Exception:
                pass
