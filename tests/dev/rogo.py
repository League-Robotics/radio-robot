#!/usr/bin/env python3
"""rogo — command-line test harness for the radio-robot-c firmware.

Connects directly to the robot over USB serial (direct mode, no relay prefix).
Default port: /dev/cu.usbmodem2143202  (the robot, NOT the relay at usbmodem21431202)

Usage examples:
  python3 tests/rogo.py hello
  python3 tests/rogo.py drive 200 200 --ms 2000
  python3 tests/rogo.py timed 150 150 1500
  python3 tests/rogo.py distance 200 200 300
  python3 tests/rogo.py enc
  python3 tests/rogo.py enc-zero
  python3 tests/rogo.py stop
  python3 tests/rogo.py go-to 300 0 200
  python3 tests/rogo.py grip 90
  python3 tests/rogo.py line
  python3 tests/rogo.py color
  python3 tests/rogo.py k
  python3 tests/rogo.py k-set KP 300
  python3 tests/rogo.py send "ENC"
"""

import argparse
import glob
import sys
import time
import serial

BAUD_RATE = 115200
DEFAULT_PORT = "/dev/cu.usbmodem2143202"
READ_TIMEOUT_S = 0.12


def sign(v: int) -> str:
    return f"+{v}" if v >= 0 else str(v)


# ---------------------------------------------------------------------------
# Serial connection (direct mode — no relay prefix)
# ---------------------------------------------------------------------------

class Conn:
    def __init__(self, port: str, verbose: bool = False):
        self._port = port
        self._verbose = verbose
        self._ser: serial.Serial | None = None
        self.mode: str = "direct"
        self.announcement: dict | None = None

    def connect(self) -> None:
        self._ser = serial.Serial(
            baudrate=BAUD_RATE,
            timeout=READ_TIMEOUT_S,
            dsrdtr=False,
            rtscts=False,
        )
        self._ser.port = self._port
        self._ser.dtr = False
        self._ser.rts = False
        self._ser.open()
        time.sleep(0.3)
        self._ser.reset_input_buffer()
        self._write("HELLO")
        lines = self._read_lines(1500)
        for line in lines:
            if line.startswith("DEVICE:"):
                parts = line.split(":")
                if len(parts) >= 5:
                    self.announcement = {
                        "role": parts[1],
                        "common_name": parts[2],
                        "device_name": parts[3],
                        "serial": ":".join(parts[4:]),
                    }
                    role = parts[1].upper()
                    if "RELAY" in role or "BRIDGE" in role:
                        self.mode = "relay"
                    else:
                        self.mode = "direct"
        if self.announcement:
            print(f"[connected] port={self._port} mode={self.mode} "
                  f"device={self.announcement.get('common_name', '?')} "
                  f"({self.announcement.get('device_name', '?')})")
        else:
            print(f"[connected] port={self._port} mode={self.mode} (no announcement)")

    def _write(self, msg: str) -> None:
        line = f"{msg}\n" if self.mode == "direct" else f">{msg}\n"
        if self._verbose:
            print(f"  >> {line.rstrip()}")
        self._ser.reset_input_buffer()
        self._ser.write(line.encode("utf-8"))
        self._ser.flush()

    def _read_lines(self, duration_ms: int = 500) -> list[str]:
        lines: list[str] = []
        deadline = time.time() + duration_ms / 1000.0
        while time.time() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", "ignore").strip()
            if text:
                lines.append(text)
        return lines

    def send(self, msg: str, read_ms: int = 600) -> list[str]:
        self._write(msg)
        lines = self._read_lines(read_ms)
        for line in lines:
            print(f"  << {line}")
        return lines

    def send_fast(self, msg: str) -> None:
        line = f"{msg}\n" if self.mode == "direct" else f">{msg}\n"
        if self._verbose:
            print(f"  >> {line.rstrip()}")
        self._ser.write(line.encode("utf-8"))
        self._ser.flush()

    def read_lines(self, duration_ms: int = 500) -> list[str]:
        lines = self._read_lines(duration_ms)
        for line in lines:
            print(f"  << {line}")
        return lines

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()


# ---------------------------------------------------------------------------
# Port helpers
# ---------------------------------------------------------------------------

def list_ports() -> list[str]:
    return sorted(glob.glob("/dev/cu.usbmodem*"))


def resolve_port(port: str | None) -> str:
    if port:
        return port
    ports = list_ports()
    if not ports:
        print("ERROR: no USB modem ports found", file=sys.stderr)
        sys.exit(1)
    if len(ports) == 1:
        return ports[0]
    # Multiple ports — prefer the known robot port
    if DEFAULT_PORT in ports:
        return DEFAULT_PORT
    print(f"Multiple ports found: {ports}")
    print(f"Using {ports[0]}. Use --port to specify.")
    return ports[0]


# ---------------------------------------------------------------------------
# Command handlers
# ---------------------------------------------------------------------------

def cmd_hello(conn: Conn, args) -> None:
    conn.send("HELLO", read_ms=1200)


def cmd_drive(conn: Conn, args) -> None:
    left, right = args.left, args.right
    ms = args.ms
    cmd = f"S{sign(left)}{sign(right)}"
    print(f"Streaming S{sign(left)}{sign(right)} for {ms}ms — Ctrl-C to stop early")
    deadline = time.time() + ms / 1000.0
    try:
        conn.send_fast(cmd)
        while time.time() < deadline:
            conn.read_lines(100)
            conn.send_fast(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        conn.send("X", read_ms=300)
        print("[stopped]")


def cmd_timed(conn: Conn, args) -> None:
    left, right, ms = args.left, args.right, args.ms
    cmd = f"T{sign(left)}{sign(right)}{sign(ms)}"
    print(f"T command: {cmd} (blocking ~{ms + 500}ms)")
    conn.send(cmd, read_ms=ms + 2000)


def cmd_distance(conn: Conn, args) -> None:
    left, right, mm = args.left, args.right, args.mm
    cmd = f"D{sign(left)}{sign(right)}{sign(mm)}"
    min_speed = max(abs(left), abs(right), 1)
    timeout_ms = int(mm / min_speed * 1000) + 3000
    print(f"D command: {cmd} (timeout {timeout_ms}ms)")
    conn.send(cmd, read_ms=min(timeout_ms, 9000))


def cmd_enc(conn: Conn, args) -> None:
    conn.send("ENC", read_ms=300)


def cmd_enc_zero(conn: Conn, args) -> None:
    conn.send("EZ", read_ms=300)
    print("[encoders zeroed]")


def cmd_stop(conn: Conn, args) -> None:
    conn.send("X", read_ms=300)
    print("[stopped]")


def cmd_go_to(conn: Conn, args) -> None:
    x, y, speed = args.x, args.y, args.speed
    timeout_s = args.timeout
    cmd = f"G{sign(x)}{sign(y)}{sign(speed)}"
    print(f"G command: {cmd} (timeout {timeout_s}s)")
    conn._write(cmd)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        lines = conn._read_lines(duration_ms=200)
        done = False
        for line in lines:
            print(f"  << {line}")
            s = line.lstrip("<# ")
            if s.startswith("G+DONE") or s.startswith("G+TIMEOUT"):
                done = True
                break
        if done:
            break
    else:
        print("[host timeout]")
    time.sleep(0.2)
    conn.send("ENC", read_ms=300)


def cmd_grip(conn: Conn, args) -> None:
    angle = args.angle
    cmd = f"G{sign(angle)}"
    conn.send(cmd, read_ms=400)


def cmd_line(conn: Conn, args) -> None:
    conn.send("LS", read_ms=400)


def cmd_color(conn: Conn, args) -> None:
    conn.send("CS", read_ms=400)


def cmd_k(conn: Conn, args) -> None:
    conn.send("K", read_ms=500)


def cmd_k_set(conn: Conn, args) -> None:
    key = args.key.lstrip("Kk")   # accept "KCP" or "CP" — strip leading K
    value = args.value
    cmd = f"K{key}+{value}" if float(value) >= 0 else f"K{key}{value}"
    conn.send(cmd, read_ms=400)


def cmd_otos(conn: Conn, args) -> None:
    conn.send("O", read_ms=400)


def cmd_otos_init(conn: Conn, args) -> None:
    conn.send("OI", read_ms=600)


def cmd_otos_zero(conn: Conn, args) -> None:
    conn.send("OZ", read_ms=400)


def cmd_send(conn: Conn, args) -> None:
    conn.send(args.message, read_ms=args.ms)


def cmd_ports(conn_unused, args) -> None:
    ports = list_ports()
    if ports:
        for p in ports:
            marker = "  <-- DEFAULT ROBOT PORT" if p == DEFAULT_PORT else ""
            print(f"  {p}{marker}")
    else:
        print("  (none found)")


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rogo",
        description="Command-line test harness for radio-robot-c firmware",
    )
    p.add_argument("--port", default=None,
                   help=f"Serial port (default: {DEFAULT_PORT})")
    p.add_argument("-v", "--verbose", action="store_true",
                   help="Print sent commands")

    sub = p.add_subparsers(dest="command", required=True)

    sub.add_parser("hello", help="Send HELLO and print announcement")
    sub.add_parser("ports", help="List available USB modem ports")

    dr = sub.add_parser("drive", help="Stream S command for --ms milliseconds")
    dr.add_argument("left", type=int, help="Left speed mm/s")
    dr.add_argument("right", type=int, help="Right speed mm/s")
    dr.add_argument("--ms", type=int, default=2000, help="Duration ms (default 2000)")

    ti = sub.add_parser("timed", help="T: blocking drive for fixed time")
    ti.add_argument("left", type=int)
    ti.add_argument("right", type=int)
    ti.add_argument("ms", type=int)

    di = sub.add_parser("distance", help="D: blocking drive for fixed distance")
    di.add_argument("left", type=int, help="Speed mm/s")
    di.add_argument("right", type=int, help="Speed mm/s")
    di.add_argument("mm", type=int, help="Distance mm")

    sub.add_parser("enc", help="Read encoders (ENC)")
    sub.add_parser("enc-zero", help="Zero encoders (EZ)")
    sub.add_parser("stop", help="Stop motors (X)")

    gt = sub.add_parser("go-to", help="G: go to relative XY (mm) at speed mm/s")
    gt.add_argument("x", type=int, help="X offset mm")
    gt.add_argument("y", type=int, help="Y offset mm")
    gt.add_argument("speed", type=int, help="Speed mm/s")
    gt.add_argument("--timeout", type=float, default=15.0, help="Host timeout s")

    gr = sub.add_parser("grip", help="Set gripper angle (degrees)")
    gr.add_argument("angle", type=int)

    sub.add_parser("line", help="Read line sensor (LS)")
    sub.add_parser("color", help="Read color sensor (CS)")
    sub.add_parser("k", help="Dump calibration params (K)")

    ks = sub.add_parser("k-set", help="Set one calibration param (e.g. k-set KP 300)")
    ks.add_argument("key", help="Param name, e.g. KP KI KD KCC KLF KRF ...")
    ks.add_argument("value", help="Float value")

    sub.add_parser("otos", help="Read OTOS pose (O)")
    sub.add_parser("otos-init", help="Init OTOS (OI)")
    sub.add_parser("otos-zero", help="Zero OTOS pose (OZ)")

    raw = sub.add_parser("send", help="Send raw command string")
    raw.add_argument("message", help="Command string (no newline)")
    raw.add_argument("--ms", type=int, default=600, help="Read timeout ms")

    return p


HANDLERS = {
    "hello": cmd_hello,
    "ports": cmd_ports,
    "drive": cmd_drive,
    "timed": cmd_timed,
    "distance": cmd_distance,
    "enc": cmd_enc,
    "enc-zero": cmd_enc_zero,
    "stop": cmd_stop,
    "go-to": cmd_go_to,
    "grip": cmd_grip,
    "line": cmd_line,
    "color": cmd_color,
    "k": cmd_k,
    "k-set": cmd_k_set,
    "otos": cmd_otos,
    "otos-init": cmd_otos_init,
    "otos-zero": cmd_otos_zero,
    "send": cmd_send,
}


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "ports":
        cmd_ports(None, args)
        return

    port = resolve_port(args.port)
    conn = Conn(port, verbose=args.verbose)
    try:
        conn.connect()
        handler = HANDLERS[args.command]
        handler(conn, args)
    except KeyboardInterrupt:
        print("\n[interrupted]")
    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        conn.close()


if __name__ == "__main__":
    main()
