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
                 cycles: float = 3.0):
    """Drive motor `port` with a sine/square velocity reference and capture
    commanded vs measured (encoder) velocity. Returns a list of dict rows:
    {t, cmd, vel, pos, applied}. Sends VEL then reads STATE each iteration."""
    rig.reset(port)
    rows = []
    t0 = time.monotonic()
    dur = period * cycles
    while True:
        t = time.monotonic() - t0
        if t >= dur:
            break
        cmd = waveform(kind, t, period, amp)
        rig.send(f"M {port} VEL {cmd:.1f}")     # fire-and-forget setpoint
        cid = rig.send(f"M {port} STATE")       # then read the state back
        _, st = rig.await_reply(cid, timeout=0.15)
        rows.append({
            "t": t, "cmd": cmd,
            "vel": st.get("vel"), "pos": st.get("pos"),
            "applied": st.get("applied"),
        })
    rig.neutral(port)
    return rows
