#!/usr/bin/env python3
"""tests/bench/rig_dev.py — bench "Rig" helper for the binary twist/config/
stop plane (104-006), replacing the pre-103 DeviceBus-era `DEV`-grammar
`Rig` client (`M <port> VEL`/`SERVO`/`ODIAG`/`ODO`/`LINE`/`COLOR`, sprint 101).

The P4 single-loop firmware's wire surface has exactly THREE `cmd` oneof
arms — `twist`/`config`/`stop` (`protos/envelope.proto`, `NezhaProtocol` —
see `host/robot_radio/robot/protocol.py`'s own module docstring) — and no
per-port addressing, no `SERVO` verb, and no `ODO SETPOSE` arm at all. The
rig's two bench motors (port 1 = drum: OTOS/line/color; port 2 = high-
inertia 3-wheel cluster — see `.clasi/knowledge/bench-test-rig-layout.md`)
are driven the SAME way the real robot's own two drive wheels are: as the
firmware's own left/right drivetrain, via `twist(v_x, omega, duration)`.
There is no independent per-port VEL/SERVO control left on this wire at
all — a `v_x`-only twist drives both rig motors symmetrically forward/back;
an `omega`-only twist drives them in opposition. This is a REAL, permanent
capability loss versus the DeviceBus-era rig grammar (no more raw per-motor
setpoints, no more sweeping the 360° OTOS servo) — not an oversight, and
not something this ticket's wire surface can restore; see this ticket's own
completion notes for the disposition of the three downstream scripts
(`rig_drive.py`/`rig_stress.py`/`otos_drift.py`) that depended on that
finer-grained control and are left broken by this rewrite.

Provides:
  - `Rig` — thin operator-facing wrapper over `SerialConnection` +
    `NezhaProtocol` (connect once via `Rig.open()`, then `twist()`/`stop()`/
    `config()`/`wait_for_ack()`/`read_tlm()`/`read_secondary_tlm()`). Used
    both interactively (a Python REPL/notebook, matching the pre-103 `Rig`
    class's own notebook-driving purpose) and by `rig_soak.py`'s sustained
    loop.
  - `waveform()` — pure sine/square reference-velocity generator (kept from
    the pre-103 `rig_dev.py`, unchanged in shape), reused by `rig_soak.py`
    to vary the commanded twist smoothly (no zero-dwell reversal — see
    `rig_soak.py`'s own module docstring for why that matters on this rig).
  - `secondary_to_dict()` — pure adapter from a `telemetry_pb2.
    TelemetrySecondary` frame to a plain dict: the binary-plane counterpart
    of the diagnostics the DeviceBus-era STLM/`M <port> STATE` replies used
    to carry (`glitch`/`acc`/`ts`/`cmd_vel`), now riding `TelemetrySecondary`
    (103-001) instead.

Run directly for a smoke-level bench verification (this ticket's own
Acceptance Criterion 4 — a single connect/twist/config/stop pass, distinct
from `rig_soak.py`'s sustained run):
    uv run python tests/bench/rig_dev.py
    uv run python tests/bench/rig_dev.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import AckEntry, NezhaProtocol, TLMFrame

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
ACK_TIMEOUT = 500  # [ms] wait_for_ack() bound for each command's ack


# ---------------------------------------------------------------------------
# Pure helpers — no I/O, no hardware. Unit-tested directly in
# tests/unit/test_rig_dev.py.
# ---------------------------------------------------------------------------

def waveform(kind: str, t: float, period: float, amp: float) -> float:  # [s] [s]
    """Reference value at time `t` for a 'sine' or 'square' waveform of the
    given `period` [s] and `amp`litude. Used to vary a commanded twist
    component (`v_x` or `omega`) smoothly over a soak run — a sine always
    crosses zero WITH a dwell either side (never an instantaneous zero-dwell
    reversal, the encoder-wedge trigger — see `.clasi/knowledge/
    encoder-wedge-boundary-latch.md`)."""
    phase = (t % period) / period
    if kind == "square":
        return amp if phase < 0.5 else -amp
    return amp * math.sin(2.0 * math.pi * phase)


def secondary_to_dict(secondary: Any) -> dict[str, Any]:
    """Adapt one `telemetry_pb2.TelemetrySecondary` frame to a plain dict —
    the ~5 Hz diagnostic frame (`acc`/`glitch`/`ts`/`cmd_vel`, moved off the
    primary `Telemetry` message by 103-001) that is the binary-plane
    counterpart of the DeviceBus-era STLM/`M <port> STATE` diagnostics.
    `secondary` is typed `Any` (not `telemetry_pb2.TelemetrySecondary`) so
    this module never imports `robot_radio.robot.pb2` at module scope purely
    for a type hint — the only real dependency is duck-typed field access,
    matching `TLMFrame.from_pb2()`'s own adapter style."""
    return {
        "t": int(secondary.now),
        "cmd_vel_left": float(secondary.cmd_vel_left) if secondary.has_cmd_vel else None,
        "cmd_vel_right": float(secondary.cmd_vel_right) if secondary.has_cmd_vel else None,
        "acc_left": float(secondary.acc_left),      # [mm/s^2]
        "acc_right": float(secondary.acc_right),     # [mm/s^2]
        "glitch_left": int(secondary.glitch_left),
        "glitch_right": int(secondary.glitch_right),
        "ts_left": int(secondary.ts_left),            # [ms]
        "ts_right": int(secondary.ts_right),           # [ms]
    }


# ---------------------------------------------------------------------------
# Rig — operator-facing wrapper over SerialConnection + NezhaProtocol
# ---------------------------------------------------------------------------

class Rig:
    """Binary-plane (twist/config/stop) operator wrapper. Construct via
    `Rig.open()` for real hardware (does the connect); the bare constructor
    takes an already-connected `conn`/`proto` pair so tests can inject fakes
    without touching a serial port at all (see `tests/unit/test_rig_dev.py`).
    """

    def __init__(self, conn: SerialConnection, proto: NezhaProtocol | None = None) -> None:
        self.conn = conn
        self.proto = proto if proto is not None else NezhaProtocol(conn)

    @classmethod
    def open(cls, port: str = DEFAULT_PORT, mode: str | None = None,
             settle: float = 2.5) -> "Rig":  # [s]
        """Connect to the robot and return a ready `Rig`. `mode` is `None`
        (auto-detect direct-USB vs relay from the boot `DEVICE:` banner —
        `SerialConnection.connect()`'s own default), `"direct"`, or
        `"relay"`. `settle` gives the firmware's boot `Preamble` device
        detection time to finish before the first command is sent; any
        telemetry frames queued during that window are drained before
        returning so a caller's first `read_tlm()` only sees fresh pushes.
        """
        conn = SerialConnection(port=port, mode=mode)
        info = conn.connect()
        if info.get("status") not in ("connected", "already_connected"):
            raise ConnectionError(f"connect failed: {info}")
        time.sleep(settle)
        rig = cls(conn)
        rig.proto.read_pending_binary_tlm_frames()
        return rig

    # --- commands (the wire's only three cmd oneof arms) -------------------

    def twist(self, v_x: float, omega: float, duration: float) -> int:  # [mm/s] [rad/s] [ms]
        """Fire-and-poll body-frame twist — see `NezhaProtocol.twist()`.
        Returns the corr_id; pass it to `wait_for_ack()` to confirm."""
        return self.proto.twist(v_x=v_x, omega=omega, duration=duration)

    def stop(self) -> int:
        """Fire-and-poll panic-stop — see `NezhaProtocol.stop()`."""
        return self.proto.stop()

    def config(self, **deltas: Any) -> int:
        """Fire-and-poll `ConfigDelta` — see `NezhaProtocol.config()`. Today
        the firmware acks every `config` with `ERR_UNIMPLEMENTED`
        (`ConfigDelta` decode succeeds, runtime application is deferred —
        `source/main.cpp`'s `CmdKind::CONFIG` case); this wrapper still
        sends it so the wire round trip itself is exercised."""
        return self.proto.config(**deltas)

    def wait_for_ack(self, corr_id: int, timeout: int = ACK_TIMEOUT) -> AckEntry | None:  # [ms]
        return self.proto.wait_for_ack(corr_id, timeout=timeout)

    # --- telemetry -----------------------------------------------------------

    def read_tlm(self) -> list[TLMFrame]:
        """Non-blocking drain of every queued PRIMARY telemetry frame."""
        return self.proto.read_pending_binary_tlm_frames()

    def read_secondary_tlm(self) -> list[dict[str, Any]]:
        """Non-blocking drain of every queued SECONDARY (slow diagnostic)
        telemetry frame, adapted via `secondary_to_dict()`."""
        return [secondary_to_dict(s) for s in self.conn.drain_binary_secondary_tlm()]

    def close(self) -> None:
        """Guaranteed stop + disconnect — motors must never be left
        running (`.claude/rules/hardware-bench-testing.md`)."""
        try:
            self.proto.stop()
        except Exception:
            pass
        try:
            self.conn.disconnect()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Smoke verification — this ticket's own Acceptance Criterion 4
# ---------------------------------------------------------------------------

class Result:
    def __init__(self) -> None:
        self.checks: list[tuple[str, bool, str]] = []

    def record(self, name: str, ok: bool, detail: str = "") -> None:
        self.checks.append((name, ok, detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    def ok(self) -> bool:
        passed = sum(1 for _, k, _ in self.checks if k)
        print(f"\n==== {passed}/{len(self.checks)} checks passed ====")
        return passed == len(self.checks)


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--relay", action="store_true",
                   help="port is a radio relay dongle (default: auto-detect)")
    p.add_argument("--v-x", type=float, default=120.0,  # [mm/s]
                   help="body-frame forward velocity for the twist() check")
    p.add_argument("--omega", type=float, default=0.8,  # [rad/s]
                   help="body-frame yaw rate for the second twist() check")
    p.add_argument("--duration", type=float, default=600.0,  # [ms]
                   help="deadman arm window for each twist command")
    p.add_argument("--watch", type=float, default=0.5,  # [s]
                   help="how long to watch telemetry for encoder movement "
                        "after each twist's ack lands")
    return p.parse_args()


def main() -> int:
    args = _args()
    result = Result()
    mode = "relay" if args.relay else None

    try:
        rig = Rig.open(port=args.port, mode=mode)
    except ConnectionError as exc:
        print(f"ERROR: {exc}")
        return 2
    result.record("connect()", True, f"mode={rig.conn.mode}")

    # Bench finding (104-006, hardware verification session): the live wire
    # occasionally drops/delays an individual command's ack-ring entry even
    # when well-separated from any other command (confirmed via ad-hoc
    # single-command diagnostics against bare NezhaProtocol.twist()/
    # wait_for_ack() -- NOT caused by this script's own logic) -- see this
    # ticket's completion notes / clasi/issues/ for the filed follow-up.
    # `wait_for_ack()`'s own docstring already names this class of outcome
    # ("Ring-wrap... a real, bounded failure, not a bug") -- one bounded
    # retry absorbs the common case without masking a genuinely dead link
    # (both attempts timing out).
    def wait_for_ack_retrying(corr_id: int, attempts: int = 2) -> "object | None":
        ack = None
        for _ in range(attempts):
            ack = rig.wait_for_ack(corr_id)
            if ack is not None:
                return ack
        return ack

    # Running "last known enc" snapshot, updated by every read_tlm() drain
    # below -- NOT re-established fresh after wait_for_ack(), because
    # wait_for_ack() destructively drains the same telemetry queue while
    # searching for its own corr_id match (its own docstring: "draining is
    # DESTRUCTIVE"), so any frame it inspects along the way is gone by the
    # time a caller tries to read it afterward. Capturing the baseline
    # BEFORE each twist() call (matching twist_drive.py's own proven order)
    # is the fix.
    last_enc: list = [None]

    def drain() -> None:
        for frame in rig.read_tlm():
            if frame.enc is not None:
                last_enc[0] = frame.enc

    def watch_enc_moves(label: str, enc_before) -> None:
        deadline = time.monotonic() + args.watch
        moved = False
        while time.monotonic() < deadline:
            drain()
            if enc_before is not None and last_enc[0] is not None and last_enc[0] != enc_before:
                moved = True
            time.sleep(0.02)
        result.record(f"encoders moving during {label}", moved,
                       f"before={enc_before} after={last_enc[0]}")

    try:
        # Establish a fresh enc baseline before the first command. A single
        # drain() right after Rig.open() can race the firmware's own ~40 ms
        # telemetry cadence and see nothing yet (matching twist_drive.py's
        # own "give the firmware one push cycle" fallback) -- retry a few
        # times, a short sleep apart, before accepting enc_before as None.
        for _ in range(5):
            drain()
            if last_enc[0] is not None:
                break
            time.sleep(0.1)

        # --- twist(): forward -------------------------------------------
        enc_before = last_enc[0]
        corr_id = rig.twist(v_x=args.v_x, omega=0.0, duration=args.duration)
        ack = wait_for_ack_retrying(corr_id)
        result.record("twist(v_x) ack confirmed via ack ring", ack is not None and ack.ok,
                       f"ack={ack}")
        watch_enc_moves("twist(v_x)", enc_before)

        # --- twist(): turn -------------------------------------------------
        enc_before = last_enc[0]
        corr_id = rig.twist(v_x=0.0, omega=args.omega, duration=args.duration)
        ack = wait_for_ack_retrying(corr_id)
        result.record("twist(omega) ack confirmed via ack ring", ack is not None and ack.ok,
                       f"ack={ack}")
        watch_enc_moves("twist(omega)", enc_before)

        # --- stop() ----------------------------------------------------------
        corr_id = rig.stop()
        ack = wait_for_ack_retrying(corr_id)
        result.record("stop() ack confirmed via ack ring", ack is not None and ack.ok,
                       f"ack={ack}")

        # --- config(): round trip only -- firmware acks ERR_UNIMPLEMENTED
        # today (source/main.cpp's CmdKind::CONFIG case); a genuine ack
        # (ok=False, err_code=ERR_UNIMPLEMENTED) counts as the round trip
        # working -- only a timeout (ack is None) fails this check.
        corr_id = rig.config(sTimeout=1000)
        ack = wait_for_ack_retrying(corr_id)
        result.record("config() round trip acked (ERR_UNIMPLEMENTED expected today)",
                       ack is not None, f"ack={ack}")

        # --- secondary telemetry (glitch/acc/ts/cmd_vel diagnostics) -----
        time.sleep(0.3)  # TelemetrySecondary rides its own ~5 Hz cadence
        secondary = rig.read_secondary_tlm()
        result.record("secondary telemetry (glitch/acc/ts diagnostics) received",
                       len(secondary) > 0,
                       f"{secondary[-1] if secondary else {}}")
    finally:
        rig.close()

    return 0 if result.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
