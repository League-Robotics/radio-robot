"""tests/bench/rig_dev.py — DEV client + waveform driver for the DeviceBus
bench test rig (sprint 101).

The rig runs the DeviceBus bring-up image (source/devices/bringup_main.cpp): a
text DEV command surface driving the device subsystem DIRECTLY (no planner).
This module is the host-side counterpart used by the sprint-101 notebooks and
the device soak test.

Rig layout (see memory `bench-test-rig-layout`):
  - Motor 1 (port 1)  -> drum: OTOS (above), line sensor (0..7 count + ch1
    index), color sensor.
  - Motor 2 (port 2)  -> 3 wheels, HIGH INERTIA (velocity-PID stress).
  - Servo on pin 5 (P1 = Nezha J1/S1), 360deg continuous: SERVO 5 90 = stop,
    <90 / >90 rotate. (pin 0 = P0 = micro:bit speaker -- do NOT drive it.)

DEV commands used: PING, RUNNING, STOP, ODIAG, ODO, LINE, COLOR,
  M <port> {VEL|DUTY|PID|NEUTRAL|RESET|STATE}, SERVO <pin> <angle>.
Replies are correlation-id matched (bring-up echoes ' #<cid>') for reliability.
"""
from __future__ import annotations

import re
import time

import serial

ROBOT_PORT = "/dev/cu.usbmodem2121102"
SERVO_PIN = 5  # P1 = Nezha J1/S1 (found by OTOS-heading response, 101-001)

# The fiber neutralizes a motor whose last setVelocity() is older than this
# (DeviceBus::kVelocityStaleUs). Re-send VEL comfortably faster than this to
# keep driving; stream_capture() feeds it every VEL_FEED_S.
VELOCITY_STALE_S = 0.300  # [s] RX-watchdog neutralize horizon
VEL_FEED_S = 0.100        # [s] setpoint re-send period (< VELOCITY_STALE_S)

_KV = re.compile(r"(\w+)=(-?[\d.]+)")


class Rig:
    """Correlation-id-matched, retry-on-timeout DEV client for the rig."""

    def __init__(self, port: str = ROBOT_PORT, settle: float = 2.5) -> None:
        self.ser = serial.Serial(port, 115200, timeout=0.02)
        self.ser.dtr = True
        time.sleep(settle)  # DeviceBus preamble incl. OTOS retry (~2s)
        self.ser.reset_input_buffer()
        self._cid = 0
        self._buf = ""  # persistent RX buffer (corr-id demux, never flushed mid-run)

    def send(self, s: str) -> int:
        """Fire-and-forget one command; returns its correlation id."""
        self._cid += 1
        self.ser.write((f"{s} #{self._cid}\r\n").encode())
        return self._cid

    def flush(self) -> None:
        """Drop the RX buffer + any queued serial data (resync after a lag)."""
        self._buf = ""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def stream(self, ms: int) -> None:  # [ms]
        """Set the firmware's unsolicited TLM push period (0 = off)."""
        self.cmd(f"STREAM {int(ms)}")

    def read_tlm(self) -> list[dict]:
        """Drain available serial and return parsed unsolicited TLM frames
        since the last call. Shares the persistent buffer with await_reply()
        (TLM lines start with 'TLM', command replies with 'OK'/'ERR', so the
        two never claim the same line). Each dict carries the raw wire keys:
        t (emit [ms]), 1p/1v/1a/1t and 2p/2v/2a/2t per motor."""
        self._buf += self.ser.read(self.ser.in_waiting or 1).decode(errors="replace")
        out: list[dict] = []
        if "\n" in self._buf:
            lines = self._buf.split("\n")
            self._buf = lines[-1]
            for ln in lines[:-1]:
                ln = ln.strip()
                if ln.startswith("TLM"):
                    out.append({m[0]: float(m[1]) for m in _KV.findall(ln)})
        return out

    def await_reply(self, cid: int, timeout: float = 0.6) -> tuple[str, dict] | tuple[None, dict]:
        """Poll the persistent buffer for the reply matching `cid`."""
        tag = f"#{cid}"
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self._buf += self.ser.read(self.ser.in_waiting or 1).decode(errors="replace")
            if "\n" in self._buf:
                lines = self._buf.split("\n")
                self._buf = lines[-1]  # keep the partial tail
                for ln in lines[:-1]:
                    toks = ln.rstrip().split()
                    if toks and toks[-1] == tag and (ln.startswith("OK") or ln.startswith("ERR")):
                        return ln.rstrip(), {m[0]: float(m[1]) for m in _KV.findall(ln)}
            time.sleep(0.002)
        return None, {}

    def cmd(self, s: str, timeout: float = 0.8, retries: int = 2) -> tuple[str, dict]:
        """Reliable one-off: send + await, retrying on timeout."""
        for _ in range(retries + 1):
            cid = self.send(s)
            ln, d = self.await_reply(cid, timeout)
            if ln is not None:
                return ln, d
        return "(timeout)", {}

    # --- device helpers ---------------------------------------------------
    def ping(self) -> bool:
        return self.cmd("PING")[0].startswith("OK")

    def odiag(self) -> dict:
        return self.cmd("ODIAG")[1]

    def odo(self) -> dict:
        return self.cmd("ODO")[1]

    def line(self) -> dict:
        return self.cmd("LINE")[1]

    def color(self) -> dict:
        return self.cmd("COLOR")[1]

    def mstate(self, port: int) -> dict:
        return self.cmd(f"M {port} STATE")[1]

    def mvel(self, port: int, v: float) -> None:  # [mm/s]
        self.send(f"M {port} VEL {v:.1f}")  # fire-and-forget for high-rate drive

    def neutral(self, port: int) -> None:
        self.cmd(f"M {port} NEUTRAL")

    def reset(self, port: int) -> None:
        self.cmd(f"M {port} RESET")

    def servo(self, angle: int, pin: int = SERVO_PIN) -> None:
        self.cmd(f"SERVO {pin} {int(angle)}")

    def stop(self) -> None:
        self.cmd("STOP")

    def close(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


def waveform(kind: str, t: float, period: float, amp: float) -> float:
    """Reference velocity [mm/s] at time t for 'sine' or 'square'."""
    import math
    phase = (t % period) / period
    if kind == "square":
        return amp if phase < 0.5 else -amp
    return amp * math.sin(2.0 * math.pi * phase)


def run_waveform(rig: Rig, port: int, kind: str, period: float, amp: float,
                 cycles: float = 3.0, offset: float = 0.0):
    """Drive motor `port` with a sine/square velocity reference (plus a DC
    `offset` -- use a positive offset >= amp to keep the drive one-directional,
    i.e. no reversal) and capture commanded vs measured (encoder) velocity.
    Returns a list of dict rows: {t, cmd, vel, pos, applied, wedged, glitch}.
    Sends VEL then reads STATE each iteration."""
    rig.reset(port)
    rows = []
    t0 = time.monotonic()
    dur = period * cycles
    while True:
        t = time.monotonic() - t0
        if t >= dur:
            break
        cmd = offset + waveform(kind, t, period, amp)
        rig.send(f"M {port} VEL {cmd:.1f}")     # fire-and-forget setpoint
        cid = rig.send(f"M {port} STATE")       # then read the state back
        _, st = rig.await_reply(cid, timeout=0.15)
        rows.append({
            "t": t, "cmd": cmd,
            "vel": st.get("vel"), "pos": st.get("pos"),
            "applied": st.get("applied"),
            "wedged": st.get("wedged"), "glitch": st.get("glitch"),
        })
    rig.neutral(port)
    return rows


def stream_capture(rig: Rig, port: int, drive, duration: float,
                   stream_ms: int = 80, offset: float = 0.0):
    """Characterization capture over the RELIABLE telemetry-stream path
    (vs. run_waveform's lossy per-sample STATE polling). Turns on the
    firmware's 80 ms TLM push, drives motor `port` with velocity setpoint
    `drive(t)` re-sent every VEL_FEED_S (feeds the RX watchdog), and records
    ONLY the pushed TLM frames -- so the result is gap-free at the encoder's
    own ~80 ms refresh cadence. Returns rows keyed off the firmware's own emit
    stamp: {t [s], cmd, pos, vel, applied, stamp [us]}."""
    pk, vk, ak, tk = f"{port}p", f"{port}v", f"{port}a", f"{port}t"
    rig.stream(stream_ms)
    rig.reset(port)
    rig.cmd(f"M {port} PID 1")
    # Let the STAGED reset + encoder boot-anchor settle before recording.
    # The Nezha brick reports its huge lifetime-accumulated raw count for a
    # frame or two after RESET (nezha_motor.cpp documents "~-33526mm on first
    # contact, NORMAL") until the software offset zeroes it; capturing through
    # that would blow out the position axis. Settle, then flush the transient.
    time.sleep(0.4)
    rig.flush()
    rows: list[dict] = []
    t0 = time.monotonic()
    last_feed = -1.0
    t_emit0 = None
    while time.monotonic() - t0 < duration:
        t = time.monotonic() - t0
        if t - last_feed >= VEL_FEED_S:
            last_feed = t
            rig.send(f"M {port} VEL {offset + drive(t):.1f}")
        for d in rig.read_tlm():
            if pk not in d or "t" not in d:
                continue
            # Defensive: drop any residual boot-anchor / corrupt outlier frame
            # (no real bench run travels tens of metres) so it never distorts
            # the plot's autoscale.
            if abs(d.get(pk, 0.0)) > 20000.0:
                continue
            if t_emit0 is None:
                t_emit0 = d["t"]
            rows.append({
                "t": (d["t"] - t_emit0) / 1000.0,        # [s] firmware emit clock
                "cmd": offset + drive((d["t"] - t_emit0) / 1000.0),
                "pos": d.get(pk), "vel": d.get(vk),
                "applied": d.get(ak), "stamp": d.get(tk),
            })
        time.sleep(0.005)
    rig.neutral(port)
    return rows
