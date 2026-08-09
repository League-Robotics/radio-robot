#!/usr/bin/env python3
"""Latency + passthrough streaming test for the Ai-WB2 module via atbridge.

Test A: N UDP round trips in normal AT mode (CIPSEND handshake each time),
        timing payload-write -> +IPD-echo-seen at the host serial side.
Test B: transparent passthrough (CIPMODE=1): stream chunks upstream, the
        UDP server's byte count is ground truth for the host->module serial
        leg + WiFi; echoes come back concurrently.

The bridge serial link drops ~5-10% of chars module->host, so Test A
matches echoes loosely (any +IPD burst counts) and Test B trusts the
server log, not the serial readback.
"""
import re
import sys
import time

import serial

PORT = sys.argv[1]
PEER = sys.argv[2] if len(sys.argv) > 2 else "192.168.1.40"
N = int(sys.argv[3]) if len(sys.argv) > 3 else 20

ser = serial.Serial(PORT, 115200, timeout=0.05)

def drain(seconds: float) -> str:
    out = b""
    deadline = time.time() + seconds
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            out += chunk
    return out.decode("utf-8", "replace")

def cmd(text: str, wait: float = 1.5) -> str:
    ser.write(text.encode() + b"\r\n")
    return drain(wait)

def wait_for(pattern: str, timeout: float) -> tuple[bool, str]:
    got = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            got += chunk
            if re.search(pattern.encode(), got):
                return True, got.decode("utf-8", "replace")
    return False, got.decode("utf-8", "replace")

print("--- setup: fresh UDP socket")
drain(1.0)
cmd("AT+CIPCLOSE")
out = cmd(f'AT+CIPSTART="UDP","{PEER}",5005,5006,0', 2.0)
print(out)

print(f"--- Test A: {N} UDP round trips (AT mode)")
rtts = []  # [ms]
misses = 0
for i in range(N):
    payload = f"ping-{i:04d}-x"  # 11 bytes
    cmd_ok = wait_for
    ser.write(f"AT+CIPSEND={len(payload)}\r\n".encode())
    ok, _ = wait_for(">", 2.0)
    if not ok:
        misses += 1
        drain(0.5)
        continue
    t0 = time.time()
    ser.write(payload.encode())
    # The echo is "echo:"+payload inside an +IPD burst; with serial loss,
    # match any +IPD fragment ("IPD" less one char still has "PD,")
    ok, _ = wait_for(r"IPD|PD,|ech", 3.0)
    if ok:
        rtts.append((time.time() - t0) * 1000.0)
    else:
        misses += 1
    drain(0.15)

if rtts:
    rtts.sort()
    n = len(rtts)
    print(f"    {n}/{N} completed, {misses} missed")
    print(f"    RTT ms  min={rtts[0]:.1f}  median={rtts[n//2]:.1f}  "
          f"p90={rtts[int(n*0.9)]:.1f}  max={rtts[-1]:.1f}")
    print("    NOTE: includes 2x 115200 serial legs + AT parse, not just WiFi")
else:
    print(f"    ALL {N} trips failed")

print("--- Test B: passthrough streaming upstream (CIPMODE=1)")
print(cmd("AT+CIPMODE=1"))
ser.write(b"AT+CIPSEND\r\n")
ok, out = wait_for(">", 3.0)
print(f"    passthrough entered: {ok}")
if ok:
    total = 0
    t0 = time.time()
    for i in range(8):
        chunk = (f"S{i}:" + "abcdefghij" * 25 + "|").encode()  # 254 B
        ser.write(chunk)
        total += len(chunk)
        time.sleep(0.05)
    dt = time.time() - t0
    print(f"    streamed {total} B upstream in {dt:.2f}s "
          f"({total/dt:.0f} B/s at the serial side)")
    drain(1.5)
    # exit passthrough: 1s guard, +++, 1s guard
    time.sleep(1.1)
    ser.write(b"+++")
    time.sleep(1.1)
    print(cmd("AT"))
    print(cmd("AT+CIPMODE=0"))
print("--- done (check udp_echo.log byte counts for ground truth)")
ser.close()
