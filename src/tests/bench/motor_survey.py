#!/usr/bin/env python3
"""motor_survey.py -- run the distance-fidelity gate across a fleet of boards.

Answers a question one robot cannot: **how much do motors and wheels vary
between builds?** Flashes every listed board with the SAME firmware image --
so the same code and the same baked calibration run everywhere and the only
remaining variable is the hardware -- then runs
`velocity_profile_gate.py` on each, cold (first motion after the flash's
reset) and warm, appending every result to one shared summary CSV.

    uv run python src/tests/bench/motor_survey.py \\
        --uids <uid>,<uid>,<uid> --out src/tests/bench/output/motor_survey_YYYYMMDD

Per board it writes `<label>_<run>.png` / `<label>_<run>_samples.csv` and
appends to `summary.csv`. `analyze_motor_survey.py` turns that summary into
the cross-board comparison.

**The ratio is wheel-diameter independent.** Both sides of
delivered/commanded are expressed in the firmware's own mm units, which come
from one baked `mm_per_wheel_deg`; a board with different wheels reports a
different *absolute* travel but the same *ratio* for the same control
behaviour. That is what makes boards with unlike wheels comparable here --
and it is why absolute mm from this survey mean nothing across boards.

SAFETY. Flashing the wrong board is the expensive mistake on a shared hub
(`.claude/rules/hardware-bench-testing.md`), so:
  - boards are addressed ONLY by UID, never by port (ports move on every
    re-enumeration, including the one the flash itself causes -- this script
    re-resolves each board's port AFTER flashing it, never before);
  - a board whose role is RADIOBRIDGE is refused outright: that is the radio
    relay the untethered/field path depends on, and reflashing it with robot
    firmware takes the field path down. `--allow-relay` overrides, and
    exists only so the refusal is a decision rather than a wall.
"""
from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys
import time

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_GATE = _REPO_ROOT / "src" / "tests" / "bench" / "velocity_profile_gate.py"

_UID_RE = re.compile(r"\b([0-9a-f]{48,64})\b")
_PORT_RE = re.compile(r"(/dev/[^\s]+)")


class Device:
    def __init__(self, uid: str, port: "str | None", role: str, name: str) -> None:
        self.uid, self.port, self.role, self.name = uid, port, role, name

    def __repr__(self) -> str:
        return f"Device({self.name or '<unnamed>'} {self.uid[:12]}… {self.port} {self.role})"


def enumerate_devices() -> "list[Device]":
    """Live view of what is attached right now (`mbdeploy list`, never
    `probe` -- probe prints the stale registry too, including devices that
    are not connected)."""
    result = subprocess.run(["uv", "run", "mbdeploy", "list"], cwd=_REPO_ROOT,
                            capture_output=True, text=True, timeout=180)
    devices: "list[Device]" = []
    for line in result.stdout.splitlines():
        uid_match = _UID_RE.search(line)
        if not uid_match:
            continue
        uid = uid_match.group(1)
        port_match = _PORT_RE.search(line)
        tail = line[uid_match.end():]
        if port_match:
            tail = line[port_match.end():]
        fields = tail.split()
        role = fields[0] if fields else ""
        name = fields[1] if len(fields) > 1 else ""
        devices.append(Device(uid, port_match.group(1) if port_match else None, role, name))
    return devices


def find(uid: str, devices: "list[Device]") -> "Device | None":
    for device in devices:
        if device.uid == uid:
            return device
    return None


def label_for(uid: str, device: "Device | None") -> str:
    """A stable, human-checkable board label. Every board flashed from one
    image announces the SAME baked device name, so the announcement cannot
    identify a board in this survey -- the UID tail can."""
    if device is not None and device.name:
        return device.name
    return f"board-{uid[16:24]}"


def flash(uid: str, hex_path: pathlib.Path, *, allow_relay: bool) -> bool:
    devices = enumerate_devices()
    device = find(uid, devices)
    if device is None:
        print(f"  SKIP {uid[:16]}…: not attached")
        return False
    if device.role.upper() == "RADIOBRIDGE" and not allow_relay:
        print(f"  REFUSED {label_for(uid, device)} ({uid[:16]}…): role is RADIOBRIDGE — "
              f"this is the radio relay the field path depends on. Use --allow-relay "
              f"only if you mean to convert it into a robot.")
        return False
    print(f"  flashing {label_for(uid, device)} ({uid[:16]}…) at {device.port} …")
    result = subprocess.run(
        ["uv", "run", "mbdeploy", "deploy", "--hex", str(hex_path), uid],
        cwd=_REPO_ROOT, capture_output=True, text=True, timeout=600)
    if result.returncode != 0:
        print(f"  FLASH FAILED rc={result.returncode}")
        print((result.stdout + result.stderr)[-1500:])
        return False
    print("  flashed ok")
    return True


def run_gate(port: str, board: str, run_tag: str, out: pathlib.Path,
             summary: pathlib.Path, extra: "list[str]") -> "int":
    chart = out / f"{board}_{run_tag}.png"
    samples = out / f"{board}_{run_tag}_samples.csv"
    command = [
        "uv", "run", "python", str(_GATE),
        "--port", port, "--board", board, "--run", run_tag,
        "--chart", str(chart), "--csv", str(samples),
        "--summary-csv", str(summary), *extra,
    ]
    result = subprocess.run(command, cwd=_REPO_ROOT, capture_output=True,
                            text=True, timeout=900)
    tail = [line for line in result.stdout.splitlines()
            if any(key in line for key in ("plateau tracking", "left :", "right:",
                                            "FAULTS", "PASS", "FAIL"))]
    for line in tail[-8:]:
        print(f"      {line.strip()}")
    if result.returncode not in (0, 1):  # 1 == gate FAIL, which is a real result
        print(f"      gate errored rc={result.returncode}")
        print((result.stdout + result.stderr)[-1200:])
    return result.returncode


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--uids", required=True,
                   help="comma-separated board UIDs, in survey order")
    p.add_argument("--out", required=True, help="survey directory (created)")
    p.add_argument("--hex", default=str(_REPO_ROOT / "MICROBIT.hex"))
    p.add_argument("--runs", default="cold,warm1,warm2",
                   help="run tags in order; the FIRST is taken immediately after "
                        "the flash's reset and is the only genuinely cold one")
    p.add_argument("--skip-flash", action="store_true",
                   help="characterize whatever is already on each board (no cold run)")
    p.add_argument("--allow-relay", action="store_true")
    p.add_argument("--boot-wait", type=float, default=6.0,  # [s]
                   help="seconds after flashing before the cold run (default 6)")
    p.add_argument("--gate-arg", action="append", default=[],
                   help="extra argument forwarded to the gate (repeatable)")
    args = p.parse_args()

    uids = [u.strip() for u in args.uids.split(",") if u.strip()]
    runs = [r.strip() for r in args.runs.split(",") if r.strip()]
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    summary = out / "summary.csv"
    hex_path = pathlib.Path(args.hex)
    if not args.skip_flash and not hex_path.exists():
        print(f"ERROR: no firmware image at {hex_path} — run `just build` first")
        return 2

    print(f"\n=== motor survey: {len(uids)} board(s) x {len(runs)} run(s) ===")
    print(f"  image:   {hex_path if not args.skip_flash else '(--skip-flash)'}")
    print(f"  output:  {out}")

    surveyed = 0
    for index, uid in enumerate(uids, 1):
        devices = enumerate_devices()
        device = find(uid, devices)
        board = label_for(uid, device)
        print(f"\n--- [{index}/{len(uids)}] {board}  {uid[:16]}… ---")
        if device is None:
            print("  SKIP: not attached")
            continue

        if not args.skip_flash:
            if not flash(uid, hex_path, allow_relay=args.allow_relay):
                continue
            # The flash resets and re-enumerates the board; its port can move,
            # so resolve it AFTER flashing, by UID, every time.
            time.sleep(args.boot_wait)
            device = find(uid, enumerate_devices())
            if device is None or not device.port:
                print("  SKIP: board did not come back after flashing")
                continue
            print(f"  back up at {device.port}")

        if not device.port:
            print("  SKIP: no serial port")
            continue

        for run_tag in runs:
            print(f"    run {run_tag}:")
            run_gate(device.port, board, run_tag, out, summary, args.gate_arg)
        surveyed += 1

    print(f"\n=== surveyed {surveyed}/{len(uids)} board(s) ===")
    print(f"  summary: {summary}")
    return 0 if surveyed else 1


if __name__ == "__main__":
    sys.exit(main())
