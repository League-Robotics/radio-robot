#!/usr/bin/env python3
"""TCP echo server for the WiFi-module bench test."""
import socket
import sys
import time

port = int(sys.argv[1]) if len(sys.argv) > 1 else 5007
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", port))
srv.listen(1)
print(f"tcp echo listening on :{port}", flush=True)
while True:
    conn, peer = srv.accept()
    print(f"[{time.strftime('%H:%M:%S')}] connect from {peer}", flush=True)
    while True:
        data = conn.recv(2048)
        if not data:
            break
        print(f"[{time.strftime('%H:%M:%S')}] {len(data)}B: {data!r}",
              flush=True)
        conn.sendall(b"echo:" + data)
    conn.close()
    print(f"[{time.strftime('%H:%M:%S')}] disconnect {peer}", flush=True)
