#!/usr/bin/env python3
"""watchdog_sim.py — verify the system safety-stop watchdog in simulation.

After the fix, the watchdog must cover EVERY motion (S/T/D/G/VW), not just
streaming. With the host silent it must emit `EVT safety_stop` within sTimeoutMs
and actually stop the wheels; with continuous `+` keepalives it must NOT fire.
"""
import math
import sys

from robot_radio.io.sim_conn import SimConnection

ST = 500          # sTimeout ms
DT = 20           # tick step ms
KA = 150          # keepalive period ms


def speed_mm_s(c):
    """Instantaneous true speed from two close exact-pose reads."""
    a = c.get_exact_pose(); c.tick(DT); b = c.get_exact_pose()
    return math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / (DT / 1000.0)


def run(cmd, keepalive, total_ms=2500):
    c = SimConnection(); c.connect()
    c.set_slip(0.0, 0.0); c.set_encoder_noise(0.0)
    c.send(f"SET sTimeout={ST}")
    c.set_enc(0.0, 0.0); c.tick(50)
    c.send(cmd)
    t = 0
    ka_acc = 0
    safety_t = None
    moving_before = 0.0
    speed_after = None
    while t < total_ms:
        lines = c.tick(DT); t += DT
        for ln in lines:
            if "safety_stop" in ln and safety_t is None:
                safety_t = t
        if safety_t is None:
            # sample motion while (supposedly) driving
            p = c.get_exact_pose()
            moving_before = max(moving_before, abs(p["x"]) + abs(p["y"]))
        elif speed_after is None and t > safety_t + 200:
            speed_after = speed_mm_s(c)   # speed ~200ms after the stop fired
        if keepalive:
            ka_acc += DT
            if ka_acc >= KA:
                c.send("+"); ka_acc = 0
    if safety_t is not None and speed_after is None:
        speed_after = speed_mm_s(c)
    c.disconnect()
    return safety_t, moving_before, speed_after


def main():
    print(f"sTimeout={ST}ms — expect safety_stop at ~{ST}-{ST+100}ms when silent\n")
    cases = [
        ("G 0 800 200", "G (go-to, far-left → pre-rotate)"),
        ("S 150 150",   "S (streaming)"),
        ("T 150 150 5000", "T (timed 5s)"),
        ("D 150 150 4000", "D (distance 4m)"),
        ("VW 0 1500",   "VW (raw spin)"),
    ]
    ok = True
    print("== SILENT (no keepalive): safety_stop MUST fire, wheels MUST stop ==")
    for cmd, label in cases:
        st, moved, after = run(cmd, keepalive=False)
        fired = st is not None
        stopped = (after is not None and after < 5.0)
        verdict = "OK" if (fired and stopped) else "FAIL"
        if not (fired and stopped):
            ok = False
        print(f"  [{verdict}] {label:34s} safety_stop@{st}ms  "
              f"moved={moved:.0f}  speed_after={after if after is None else round(after,1)}mm/s")

    print("\n== KEEPALIVE every 150ms: safety_stop must NOT fire ==")
    for cmd, label in [("G 0 800 200", "G (go-to)"), ("S 150 150", "S (streaming)"),
                       ("T 150 150 5000", "T (timed)")]:
        st, moved, after = run(cmd, keepalive=True)
        verdict = "OK" if st is None else "FAIL"
        if st is not None:
            ok = False
        print(f"  [{verdict}] {label:34s} safety_stop@{st}  moved={moved:.0f}")

    print("\nRESULT:", "ALL PASS" if ok else "FAILURES ABOVE")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
