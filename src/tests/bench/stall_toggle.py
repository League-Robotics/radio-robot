"""Turn the stall detector on/off at RUNTIME, without reflashing.

    uv run python src/tests/bench/stall_toggle.py --port PORT off
    uv run python src/tests/bench/stall_toggle.py --port PORT on
    uv run python src/tests/bench/stall_toggle.py --port PORT show

Why a whole-group push: set_config_group() sends the target group's ENTIRE
message, so any field you leave out is transmitted as its protobuf default --
i.e. ZERO. Pushing one field is not a partial update, it is a wipe of every
other field in the group. That is not hypothetical: a single-field vel_kff push
earlier in this project silently zeroed the rest of MOTORS, distance moves went
to 2.09x commanded, and the damage SURVIVED a DTR reset -- only a reflash
cleared it, and several hours of "firmware" debugging were spent on it first.

So this tool always sends the complete wheel_control group, read from the robot
JSON (data/robots/<robot>.json), with only stall_window overridden. The file
stays the source of truth; "off" is stall_window = 0, the documented inert
state, and "on" restores the file's own value.
"""
import argparse
import json
import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO / "src" / "host"))

from robot_radio.io.serial_conn import SerialConnection      # noqa: E402
from robot_radio.robot.protocol import NezhaProtocol         # noqa: E402
from robot_radio.robot.pb2 import robot_config_pb2 as pb     # noqa: E402

FIELDS = ("v_min", "bias_max", "tau_adapt", "a_steady", "deficit_threshold",
          "deficit_window", "pid_kp", "pid_ki", "pid_i_max", "pid_kaff",
          "pid_max", "pos_err_max", "stall_speed", "stall_demand",
          "stall_window")


def group_from_file(robot: str) -> dict:
    d = json.loads((_REPO / "data" / "robots" / f"{robot}.json").read_text())
    wc = d["wheel_control"]
    missing = [f for f in FIELDS if f not in wc]
    if missing:
        raise SystemExit(f"{robot}.json wheel_control is missing {missing}")
    return {f: float(wc[f]) for f in FIELDS}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("action", choices=["on", "off", "show"])
    ap.add_argument("--port", default="/dev/cu.usbmodem214102")
    ap.add_argument("--robot", default="tovez")
    args = ap.parse_args()

    fields = group_from_file(args.robot)
    baked = fields["stall_window"]
    if args.action == "show":
        print(f"  {args.robot}.json wheel_control:")
        for f in FIELDS:
            mark = "  <- stall" if f.startswith("stall") else ""
            print(f"    {f:18} {fields[f]}{mark}")
        return 0

    fields["stall_window"] = 0.0 if args.action == "off" else baked

    conn = SerialConnection(port=args.port)
    p = NezhaProtocol(conn)
    try:
        conn.connect(); time.sleep(1.5)
        p.tlmOn(); time.sleep(0.6)
        ack = p.set_config_group(pb.WHEEL_CONTROL, **fields)
        ok = bool(ack) and getattr(ack, "ok", False)
        print(f"  stall detector {args.action.upper()}  "
              f"(stall_window = {fields['stall_window']}, file value {baked})")
        print(f"  whole wheel_control group pushed, {len(FIELDS)} fields; "
              f"ack {'OK' if ok else ack}")
        # read back -- an ack is not evidence the value landed
        try:
            snap = p.get_config(pb.WHEEL_CONTROL)
            got = getattr(snap, "stall_window", None)
            if got is not None:
                print(f"  read-back stall_window = {got}  "
                      f"{'CONFIRMED' if abs(got - fields['stall_window']) < 1e-6 else 'MISMATCH'}")
        except Exception as exc:
            print(f"  read-back unavailable ({type(exc).__name__}) -- "
                  f"behaviour is the only check")
        return 0 if ok else 1
    finally:
        try: p.estop()
        except Exception: pass
        try: conn.disconnect()
        except Exception: pass


if __name__ == "__main__":
    raise SystemExit(main())
