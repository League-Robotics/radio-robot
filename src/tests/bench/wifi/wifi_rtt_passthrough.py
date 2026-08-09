#!/usr/bin/env python3
"""Passthrough-mode UDP RTT: write payload, time until first echo byte.

Between trips the pipe is silent, so time-to-first-downstream-byte after a
write IS the round trip (robust to the bridge's char drops).
"""
import sys
import time

import serial

PORT = sys.argv[1]
PEER = sys.argv[2] if len(sys.argv) > 2 else "192.168.1.40"
N = int(sys.argv[3]) if len(sys.argv) > 3 else 30

ser = serial.Serial(PORT, 115200, timeout=0.02)

def drain(seconds: float) -> bytes:
    out = b""
    deadline = time.time() + seconds
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            out += chunk
    return out

def cmd(text: str, wait: float = 1.5) -> str:
    ser.write(text.encode() + b"\r\n")
    return drain(wait).decode("utf-8", "replace")

drain(1.0)
# In case a previous run left passthrough active: guard-time escape first.
time.sleep(1.1)
ser.write(b"+++")
time.sleep(1.1)
cmd("AT+CIPMODE=0")
cmd("AT+CIPCLOSE")
print(cmd(f'AT+CIPSTART="UDP","{PEER}",5005,5006,0', 2.0))
print(cmd("AT+CIPMODE=1"))
ser.write(b"AT+CIPSEND\r\n")
got = drain(2.0)
print(f"passthrough: {got!r}")

rtts = []  # [ms]
misses = 0
for i in range(N):
    payload = f"p{i:03d}".encode()
    t0 = time.time()
    ser.write(payload)
    first = None
    deadline = time.time() + 2.0
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            first = time.time()
            break
    if first is None:
        misses += 1
    else:
        rtts.append((first - t0) * 1000.0)
    drain(0.25)  # swallow the rest of the echo

if rtts:
    rtts.sort()
    n = len(rtts)
    print(f"{n}/{N} round trips, {misses} missed")
    print(f"RTT ms  min={rtts[0]:.1f}  median={rtts[n//2]:.1f}  "
          f"p90={rtts[int(n*0.9)]:.1f}  max={rtts[-1]:.1f}")
else:
    print(f"ALL {N} trips failed")

time.sleep(1.1)
ser.write(b"+++")
time.sleep(1.1)
print(cmd("AT+CIPMODE=0"))
ser.close()
