#!/usr/bin/env python3
"""UDP echo server for the WiFi-module bench test.

Logs every datagram (peer, length, payload) to stdout and echoes it back
with an "echo:" prefix so the module side can prove the return path.
"""
import socket
import sys
import time

port = int(sys.argv[1]) if len(sys.argv) > 1 else 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", port))
print(f"udp echo listening on :{port}", flush=True)
while True:
    data, peer = sock.recvfrom(2048)
    print(f"[{time.strftime('%H:%M:%S')}] {peer} {len(data)}B: {data!r}",
          flush=True)
    sock.sendto(b"echo:" + data, peer)
