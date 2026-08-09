#!/usr/bin/env python3
"""Apply a robot's configured fixed WiFi IP to its Planet X module.

Reads ``connection.wifi_ip`` from the robot's JSON (the one config file --
see robot_config.proto's field comment). The module learns gateway and
netmask from its DHCP join; ONLY the address itself comes from the file
(stakeholder directive 2026-08-08). A robot with no ``wifi_ip`` (or no
WiFi module at all) is a clean no-op: the module is an optional
peripheral.

Talks to the module through the atbridge firmware (see README.md) on the
robot's USB serial port. The bridge's module->host leg drops ~5-10% of
chars (DAPLink CDC outbound loss, sprint 123's measurement -- baud-
invariant), so every value read back over it must be seen IDENTICALLY
TWICE before it is trusted, and the applied IP is verified end-to-end
with a host-side ping rather than by readback alone.

Usage:
  uv run python src/tests/bench/wifi/wifi_setup.py --robot gopiv \
      --port /dev/cu.usbmodem2121302 [--ssid "Busboom Mesh" --password PW]
"""
import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

import serial

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT / "src" / "host"))

from robot_radio.config.robot_config import load_robot_config  # noqa: E402

IP_RE = re.compile(r'(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})')


class Bridge:
    """Thin loss-aware AT driver over the atbridge serial pipe."""

    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.drain(1.0)

    def drain(self, seconds: float) -> str:
        out = b""
        deadline = time.time() + seconds
        while time.time() < deadline:
            chunk = self.ser.read(4096)
            if chunk:
                out += chunk
        return out.decode("utf-8", "replace")

    def cmd(self, text: str, wait: float = 1.5) -> str:
        self.ser.write(text.encode() + b"\r\n")
        return self.drain(wait)

    def alive(self) -> bool:
        for _ in range(3):
            if "OK" in self.cmd("AT", 1.0):
                return True
        return False

    def read_cipsta(self, tries: int = 20,
                    wait: float = 1.0) -> "dict[str, str] | None":
        """Majority-vote ip/gateway/netmask out of repeated AT+CIPSTA? reads.

        The serial leg drops chars independently per read, BUT identical
        corruptions can repeat ("255.255.255.0" -> "25.255.255.0" arises
        from three different single-char drops), so seen-twice is not
        enough: a field's value wins only with >=3 sightings AND a lead of
        >=2 over the runner-up. Netmask candidates must also be structurally
        valid masks (contiguous ones), which rejects "25.255.255.0" outright.

        Field labels arrive holey too ("gatway") -- match the label TOKEN
        between the colons by subsequence, never the whole line ("ip" is a
        subsequence of "+CIPSTA" itself, which burned the first version of
        this parser).
        """
        tally: dict[str, dict[str, int]] = {"ip": {}, "gateway": {}, "netmask": {}}

        def winner(field: str) -> "str | None":
            counts = sorted(tally[field].values(), reverse=True)
            if not counts or counts[0] < 3:
                return None
            if len(counts) > 1 and counts[0] - counts[1] < 2:
                return None
            return max(tally[field], key=lambda q: tally[field][q])

        for _ in range(tries):
            out = self.cmd("AT+CIPSTA?", wait)
            for line in out.splitlines():
                parts = line.split(":")
                if len(parts) < 3:
                    continue
                token = parts[1].strip().lower()
                m = IP_RE.search(":".join(parts[2:]))
                if not m:
                    continue
                quad = m.group(1)
                field = None
                if token and len(token) <= 2 and _is_subsequence(token, "ip"):
                    field = "ip"
                elif len(token) >= 4 and _is_subsequence(token, "gateway"):
                    field = "gateway"
                elif len(token) >= 4 and _is_subsequence(token, "netmask"):
                    field = "netmask"
                if field == "netmask" and not _valid_mask(quad):
                    continue
                if field:
                    tally[field][quad] = tally[field].get(quad, 0) + 1
            got = {f: winner(f) for f in tally}
            if all(got.values()):
                return got  # type: ignore[return-value]
        return None


def _is_subsequence(needle: str, haystack: str) -> bool:
    it = iter(haystack.lower())
    return all(c in it for c in needle.lower())


def _valid_mask(quad: str) -> bool:
    try:
        v = 0
        for octet in quad.split("."):
            n = int(octet)
            if not 0 <= n <= 255:
                return False
            v = (v << 8) | n
    except ValueError:
        return False
    # contiguous ones from the MSB: v | (v-1) over 32 bits must be all-ones
    return v != 0 and ((v | (v - 1)) & 0xFFFFFFFF) == 0xFFFFFFFF


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True, help="robot JSON stem, e.g. gopiv")
    ap.add_argument("--port", required=True, help="atbridge serial port")
    ap.add_argument("--ssid", help="join this AP first (else rely on the module's saved AP)")
    ap.add_argument("--password", help="AP password (with --ssid)")
    args = ap.parse_args()

    cfg = load_robot_config(REPO_ROOT / "data" / "robots" / f"{args.robot}.json")
    ip = cfg.connection.wifi_ip
    if not ip:
        print(f"{args.robot}: no connection.wifi_ip configured -- nothing to do")
        return 0
    print(f"{args.robot}: configured fixed IP {ip}")

    br = Bridge(args.port)
    if not br.alive():
        print("ERROR: no AT/OK through the bridge -- is the atbridge flashed "
              "and the module plugged in?")
        return 1

    if args.ssid:
        print(f"joining {args.ssid!r}...")
        out = br.cmd(f'AT+CWJAP="{args.ssid}","{args.password or ""}"', 15.0)
        if "GOT IP" not in out.replace(" ", ""):
            # loss-tolerant: accept holey "WIFI GOT IP" via subsequence
            if not _is_subsequence("GOTIP", out):
                print(f"ERROR: join failed: {out!r}")
                return 1

    # Re-enable station DHCP first: a previous static apply (including a
    # bad one) leaves no DHCP truth to read, and this makes the tool
    # idempotent from any prior module state. The lease takes a moment.
    br.cmd("AT+CWDHCP=1,1", 2.0)
    time.sleep(4.0)

    # DHCP gives the network shape; only the address comes from config.
    net = br.read_cipsta()
    if net is None:
        print("ERROR: could not majority-read gateway/netmask from AT+CIPSTA? "
              "-- is the module associated? (try --ssid/--password)")
        return 1
    gw, mask = net["gateway"], net["netmask"]
    print(f"DHCP network: ip {net['ip']}  gateway {gw}  netmask {mask}")

    print(f"applying static IP {ip} (gateway {gw}, netmask {mask})")
    br.cmd(f'AT+CIPSTA="{ip}","{gw}","{mask}"', 3.0)

    applied = br.read_cipsta()
    if applied is None or applied["ip"] != ip:
        print(f"ERROR: readback says {applied!r}, wanted ip {ip!r}")
        return 1
    print(f"module readback confirms {applied['ip']}")

    # End-to-end truth: the readback rode the lossy serial; the ping does not.
    r = subprocess.run(["ping", "-c", "3", ip], capture_output=True, text=True)
    print(r.stdout.strip().splitlines()[-1] if r.stdout else r.stderr.strip())
    if r.returncode == 0:
        print(f"OK: {args.robot}'s module answers at {ip}")
        return 0
    print(f"WARNING: applied and read back, but {ip} does not answer ping "
          "from this host -- check subnet/routing (DHCP net vs configured IP).")
    return 2


if __name__ == "__main__":
    sys.exit(main())
