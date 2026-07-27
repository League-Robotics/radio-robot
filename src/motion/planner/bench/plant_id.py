#!/usr/bin/env python3
"""plant_id.py -- identify the real duty->wheel-velocity plant, so the
host-side duty-plane simulation uses MEASURED dynamics, not guesses.

Method: reduce the firmware velocity PID to pure feedforward (CONFIG
pid.kp=0, pid.ki=0 -- duty = kff * v_target exactly), then step the duty
via move_wheels(v_equiv = duty/kff) and record each wheel's telemetry
velocity time series. Fit per step: steady-state gain [mm/s per duty],
time constant tau [s] (63% rise), and dead time [s] (first motion).
Gains are restored afterward.

    uv run python src/motion/planner/bench/plant_id.py \
        --port /dev/cu.usbmodem2121102
"""

import argparse
import statistics
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

FIRMWARE_KFF = 0.0008  # [duty/(mm/s)] tovez.json vel_kff


def captureStep(proto: NezhaProtocol, duty: float,
                seconds: float) -> list[tuple[float, float, float]]:
    """Command a duty step and record (hostTime, velLeft, velRight)."""
    proto.read_pending_binary_tlm_frames()  # drain
    t0 = time.monotonic()
    vEquiv = duty / FIRMWARE_KFF
    proto.move_wheels(v_left=vEquiv, v_right=vEquiv,
                      stop_time=seconds * 1000.0 + 500.0,
                      timeout=seconds * 1000.0 + 1000.0, replace=True)
    series = []
    while time.monotonic() - t0 < seconds:
        for f in proto.read_pending_binary_tlm_frames():
            if f.enc_left is None:
                continue
            series.append((time.monotonic() - t0,
                           f.enc_left.velocity, f.enc_right.velocity))
        time.sleep(0.01)
    proto.estop()
    return series


def fitStep(series, duty: float, wheelIndex: int) -> dict | None:
    velocities = [(t, row[wheelIndex]) for t, *row in
                  [(t, l, r) for t, l, r in series]]
    if len(velocities) < 20:
        return None
    steady = statistics.mean(v for t, v in velocities if t > 1.6)
    if abs(steady) < 20.0:
        return None
    gain = steady / duty
    deadTime = next((t for t, v in velocities if abs(v) > 0.10 * abs(steady)),
                    None)
    riseTime = next((t for t, v in velocities if abs(v) > 0.63 * abs(steady)),
                    None)
    tau = (riseTime - deadTime) if (riseTime and deadTime) else None
    return dict(duty=duty, steady=steady, gain=gain, deadTime=deadTime,
                tau=tau)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    args = parser.parse_args()

    conn = SerialConnection(port=args.port)
    info = conn.connect()
    if info.get("status") != "connected":
        raise ConnectionError(f"connect failed: {info}")
    proto = NezhaProtocol(conn)
    try:
        proto.config(**{"pid.kp": 0.0, "pid.ki": 0.0})
        time.sleep(0.4)
        print("firmware PID reduced to kff -- open-loop duty steps")
        results = []
        for duty in (0.2, 0.35, 0.5, -0.35):
            time.sleep(1.0)  # rest between steps
            series = captureStep(proto, duty, 2.2)
            for wheel, name in ((0, "L"), (1, "R")):
                fit = fitStep(series, duty, wheel)
                if fit:
                    results.append(fit)
                    print(f"  duty {duty:+.2f} {name}: steady "
                          f"{fit['steady']:+7.1f} mm/s  gain "
                          f"{fit['gain']:6.1f} mm/s/duty  dead "
                          f"{(fit['deadTime'] or 0) * 1000:5.0f} ms  tau "
                          f"{(fit['tau'] or 0) * 1000:5.0f} ms")
        if results:
            gains = [r["gain"] for r in results]
            taus = [r["tau"] for r in results if r["tau"]]
            deads = [r["deadTime"] for r in results if r["deadTime"]]
            print(f"SUMMARY: gain median {statistics.median(gains):.0f} "
                  f"mm/s/duty; tau median "
                  f"{statistics.median(taus) * 1000:.0f} ms; dead median "
                  f"{statistics.median(deads) * 1000:.0f} ms over "
                  f"{len(results)} fits")
    finally:
        proto.estop()
        proto.config(**{"pid.kp": 0.0016, "pid.ki": 0.005})
        time.sleep(0.3)
        conn.disconnect()


if __name__ == "__main__":
    main()
