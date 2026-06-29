#!/usr/bin/env python3
"""set_linear_cal.py — apply linear odometry calibration at RUNTIME (no flash).

Sends ``SET ml/mr`` (encoder mm-per-wheel-degree) and optionally the OTOS linear
scale to the robot live, and persists the new values to data/robots/tovez.json so
the next boot's DefaultConfig matches.

    # scale the current encoder mm/deg by a measured factor (truth/reported):
    uv run python tests/_infra/calibrate/set_linear_cal.py --enc-factor 0.862
    # or set explicit values:
    uv run python tests/_infra/calibrate/set_linear_cal.py --ml 0.6176 --mr 0.6100
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from robot_radio.robot.nezha import Nezha
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
from robot_radio.config.robot_config import load_robot_config

_REPO = Path(__file__).resolve().parents[3]
_TOVEZ = _REPO / "data" / "robots" / "tovez.json"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--enc-factor", type=float, default=None,
                    help="multiply current ml AND mr by this (truth/reported)")
    ap.add_argument("--ml", type=float, default=None)
    ap.add_argument("--mr", type=float, default=None)
    ap.add_argument("--otos-scale", type=float, default=None,
                    help="new OTOS linear scale (also pushed via OL)")
    ap.add_argument("--port", default=None)
    ap.add_argument("--no-write", action="store_true")
    a = ap.parse_args(argv)

    cfg = load_robot_config(_TOVEZ)
    ml = cfg.calibration.mm_per_wheel_deg_left
    mr = cfg.calibration.mm_per_wheel_deg_right
    otos = cfg.calibration.otos_linear_scale
    print(f"current: ml={ml:.5f} mr={mr:.5f} otos_scale={otos:.4f}")

    if a.enc_factor is not None:
        ml *= a.enc_factor
        mr *= a.enc_factor
    if a.ml is not None:
        ml = a.ml
    if a.mr is not None:
        mr = a.mr
    if a.otos_scale is not None:
        otos = a.otos_scale
    print(f"new    : ml={ml:.5f} mr={mr:.5f} otos_scale={otos:.4f}")

    port = a.port or (list_serial_ports() or [None])[0]
    if port is None:
        print("no serial port"); return 2
    conn = SerialConnection(port=port); conn.connect()
    proto = NezhaProtocol(conn); robot = Nezha(proto)
    print("connected:", robot.connect())
    res = proto.set_config(ml=round(ml, 5), mr=round(mr, 5))
    print("SET ml/mr ->", res)
    if a.otos_scale is not None:
        # OTOS linear float scale -> int8 register: (scale-1.0)/0.001, clamped.
        i8 = max(-128, min(127, round((otos - 1.0) / 0.001)))
        rb = proto.otos_set_linear_scalar(i8)
        print(f"SET OTOS linear scale={otos:.4f} int8={i8:+d} (readback {rb})")
    conn.disconnect()

    if not a.no_write:
        d = json.loads(_TOVEZ.read_text())
        d.setdefault("calibration", {})
        d["calibration"]["mm_per_wheel_deg_left"] = round(ml, 5)
        d["calibration"]["mm_per_wheel_deg_right"] = round(mr, 5)
        if a.otos_scale is not None:
            d["calibration"]["otos_linear_scale"] = round(otos, 4)
        _TOVEZ.write_text(json.dumps(d, indent=2) + "\n")
        print(f"wrote {_TOVEZ}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
