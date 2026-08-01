"""recorder -- the system-test JSONL dataset writer and in-process record bus.

One JSON object per line, EVERY line both directions: binary telemetry
frames, DBG/cleartext lines, and everything the host sent. The EXPECT
evaluator subscribes to the same in-process stream the file writer
consumes, so assertions run against byte-identical data to what lands in
the dataset.

Record envelope (system-test-minimal-system-test issue):
    seq_file   monotonic per-run index, host-assigned
    t_host     time.monotonic() [s] at record time
    dir        "rx" | "tx"
    transport  "sim" | "serial"
    plane      "binary" | "cleartext" | "host"   (host = no wire traffic)
    verb       wire verb or None
    raw        literal text for cleartext lines; None where the decoded
               object is all we have (TLM via TLMFrame)
    decode_ok  False only for lines that failed decode
    type       record kind: run_meta | tlm | dbg | cleartext | cmd | step |
               expect | camera_fix | note
    payload    nested, type-specific
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any, Callable

# Authoritative telemetry flag-bit names, mirrored from firmware
# (src/firm/app/telemetry.h:223-264) -- includes bits 17-20, which the host
# TLMFrame exposes no properties for. RESERVED bits are named so an
# unexpected set bit is visible rather than silently dropped.
FLAG_NAMES = [
    "OTOS_PRESENT",            # 0
    "OTOS_CONNECTED",          # 1
    "ACTIVE",                  # 2
    "CONN_LEFT",               # 3
    "CONN_RIGHT",              # 4
    "RESERVED_5",              # 5
    "FAULT_I2C_SAFETY_NET",    # 6
    "FAULT_WEDGE_LATCH",       # 7
    "FAULT_I2C_NAK",           # 8
    "FAULT_COMMS_MALFORMED",   # 9
    "EVENT_DEADMAN_EXPIRED",   # 10
    "RESERVED_11",             # 11
    "EVENT_CONFIG_APPLIED",    # 12
    "LINE_PRESENT",            # 13
    "COLOR_PRESENT",           # 14
    "FAULT_MOVE_TIMEOUT",      # 15
    "FAULT_SHAPING_DISABLED",  # 16
    "FAULT_POSITION_CLAMPED",  # 17
    "FAULT_COMMANDS_DROPPED",  # 18
    "FAULT_WHEEL_FROZEN_LEFT",   # 19
    "FAULT_WHEEL_FROZEN_RIGHT",  # 20
]

_CDEG = 0.0174532925199433 / 100.0  # [rad/cdeg]
_MRAD = 1.0e-3  # [rad/mrad]


def flag_names(flags: int) -> list[str]:
    names = [name for bit, name in enumerate(FLAG_NAMES) if flags & (1 << bit)]
    extra = flags >> len(FLAG_NAMES)
    bit = len(FLAG_NAMES)
    while extra:
        if extra & 1:
            names.append(f"BIT_{bit}")
        extra >>= 1
        bit += 1
    return names


def tlm_payload(frame: Any) -> dict:
    """Nested payload from a decoded TLMFrame. Mirrors the wire structure;
    flattening is the analyzer's concern, never the dataset's."""
    p: dict[str, Any] = {
        "t": frame.t,
        "seq": frame.seq,
        "mode": frame.mode,
        "flags_raw": frame.flags,
        "flags": flag_names(frame.flags or 0),
    }
    for side in ("enc_left", "enc_right"):
        r = getattr(frame, side, None)
        if r is not None:
            p[side] = {"position": r.position, "velocity": r.velocity,
                       "age": r.age, "position_epoch": r.position_epoch}
    if frame.pose is not None:
        x, y, h_cdeg = frame.pose
        p["pose"] = {"x": float(x), "y": float(y),
                     "heading": h_cdeg * _CDEG}  # [rad]
    if getattr(frame, "twist", None) is not None:
        v, omega_mrad = frame.twist
        p["twist"] = {"v_x": float(v), "omega": omega_mrad * _MRAD}  # [rad/s]
    otos = getattr(frame, "otos_reading", None)
    if otos is not None:
        p["otos"] = {"x": otos.x, "y": otos.y, "heading": otos.heading,
                     "v_x": otos.v_x, "v_y": otos.v_y, "omega": otos.omega,
                     "age": otos.age}
    if getattr(frame, "line", None) is not None:
        p["line"] = list(frame.line)
    if getattr(frame, "color", None) is not None:
        p["color"] = list(frame.color)
    p["acks"] = [{"corr_id": a.corr_id, "ok": a.ok, "err": a.err_code}
                 for a in (frame.acks or [])]
    if getattr(frame, "cycle_busy", None) is not None:
        p["cycle_busy"] = frame.cycle_busy  # [us]
    if getattr(frame, "cycle_period", None) is not None:
        p["cycle_period"] = frame.cycle_period  # [us]
    return p


def parse_cleartext_payload(verb: str, data: str) -> dict:
    """Structure a cleartext line's data field so EXPECT can query it.
    STATUS's k=v:k=v shape becomes a dict with ints where they parse;
    everything else is carried as {"text": data}."""
    if verb == "STATUS" and data:
        out: dict[str, Any] = {}
        for pair in data.split(":"):
            key, sep, raw = pair.partition("=")
            if not sep:
                continue
            try:
                out[key] = int(raw, 0)
            except ValueError:
                out[key] = raw
        return out
    return {"text": data}


class Recorder:
    """JSONL writer + subscriber bus. Thread-safe: taps fire on the sim
    tick thread (or serial reader thread) while the executor records tx on
    the main thread."""

    def __init__(self, out_path: str | Path, transport: str):
        self._path = Path(out_path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self._path.open("w", encoding="utf-8")
        self._transport = transport
        self._lock = threading.Lock()
        self._seq = 0
        self._subscribers: list[Callable[[dict], None]] = []

    @property
    def path(self) -> Path:
        return self._path

    def subscribe(self, fn: Callable[[dict], None]) -> None:
        self._subscribers.append(fn)

    def close(self) -> None:
        with self._lock:
            self._file.close()

    # -- core ----------------------------------------------------------

    def record(self, rec: dict) -> dict:
        with self._lock:
            rec.setdefault("t_host", time.monotonic())
            rec["seq_file"] = self._seq
            self._seq += 1
            rec.setdefault("transport", self._transport)
            self._file.write(json.dumps(rec, separators=(",", ":")) + "\n")
            self._file.flush()
            subscribers = list(self._subscribers)
        for fn in subscribers:
            try:
                fn(rec)
            except Exception:  # a raising subscriber must never kill a tap
                pass
        return rec

    # -- rx constructors ------------------------------------------------

    def rx_tlm(self, frame: Any) -> dict:
        return self.record({
            "type": "tlm", "dir": "rx", "plane": "binary", "verb": "TLM",
            "raw": None, "decode_ok": True, "payload": tlm_payload(frame)})

    def rx_line(self, line: str) -> dict:
        """Any cleartext line, DBG included -- routes on the verb."""
        verb, sep, data = line.partition(":")
        if not sep and " " not in verb:
            data = ""
        if verb == "DBG":
            return self.record({
                "type": "dbg", "dir": "rx", "plane": "cleartext",
                "verb": "DBG", "raw": line, "decode_ok": True,
                "payload": {"text": data}})
        return self.record({
            "type": "cleartext", "dir": "rx", "plane": "cleartext",
            "verb": verb, "raw": line, "decode_ok": True,
            "payload": parse_cleartext_payload(verb, data)})

    # -- tx constructors ------------------------------------------------

    def tx_cmd(self, verb: str, payload: dict, *, plane: str = "binary") -> dict:
        return self.record({
            "type": "cmd", "dir": "tx", "plane": plane, "verb": verb,
            "raw": None, "decode_ok": True, "payload": payload})

    def tx_step(self, step_type: str, line_no: int, detail: dict) -> dict:
        return self.record({
            "type": "step", "dir": "tx", "plane": "host", "verb": None,
            "raw": None, "decode_ok": True,
            "payload": {"step": step_type, "line_no": line_no, **detail}})

    # -- host-side event constructors -----------------------------------

    def expect_result(self, query: str, ok: bool,
                      matched_seq_file: int | None, waited: float,
                      line_no: int) -> dict:
        return self.record({
            "type": "expect", "dir": "tx", "plane": "host", "verb": None,
            "raw": None, "decode_ok": True,
            "payload": {"query": query, "ok": ok,
                        "matched_seq_file": matched_seq_file,
                        "waited": waited, "line_no": line_no}})  # [s]

    def camera_fix(self, payload: dict) -> dict:
        return self.record({
            "type": "camera_fix", "dir": "rx", "plane": "host", "verb": None,
            "raw": None, "decode_ok": True, "payload": payload})

    def note(self, text: str, **detail: Any) -> dict:
        return self.record({
            "type": "note", "dir": "tx", "plane": "host", "verb": None,
            "raw": None, "decode_ok": True,
            "payload": {"text": text, **detail}})

    def run_meta(self, meta: dict) -> dict:
        return self.record({
            "type": "run_meta", "dir": "tx", "plane": "host", "verb": None,
            "raw": None, "decode_ok": True, "payload": meta})
