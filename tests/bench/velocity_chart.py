#!/usr/bin/env python3
"""velocity_chart.py — interactive real-time robot bench dashboard.

REINVIGORATED for ticket 077-006: this is the same tool as
`tests_old/bench/velocity_chart.py`, rewired from the old TLM-streaming
telemetry path onto the `DEV` protocol (`docs/protocol-v2.md` §16). It
drives via `DEV DT WHEELS <left> <right>` (per-wheel velocity targets,
ratio-governed — the same command family as `DEV DT VW`, chosen here because
this tool's whole UX is stepping a per-wheel ratio, which `WHEELS` sets
directly without going through body-twist kinematics) and samples
`DEV M <left-port> STATE` / `DEV M <right-port> STATE` for velocity and
applied duty. There is no TLM push-streaming in this bench-only dev
firmware — the worker thread polls STATE in a tight loop instead; every DEV
line it sends also feeds the serial-silence watchdog, so no separate
keepalive stream is needed.

PANELS (left column shares the time axis):
  - Wheel velocity (vL, vR) with 200 ms moving-average overlay + commanded setpoints  [KEPT]
  - Jitter metric: rolling RMSE of each wheel vs its own 200 ms moving average       [KEPT]
  - Applied duty (aL, aR) — NEW, replaces the old colour-sensor strip: the DEV
    protocol's Motor faceplate has no colour/line sensor exposed (only Motor is
    implemented this sprint — see the sprint-077 issue's Step 3), but it does
    give us `applied=` directly, which is arguably more useful here anyway: this
    tool's purpose is tuning the in-motor velocity PID, and applied-duty-vs-
    velocity is exactly the step-response signal for that.
  - Status strip: wedged/conn flags per side — NEW, replaces the old 4-channel
    line-sensor strip (same reason: no line sensor exposed over DEV).
RIGHT column:
  - Phase plot: vR vs vL (the wheel-ratio diagonal) with commanded-target dot  [KEPT —
    this is explicitly the ratio governor's diagonal made visible, per the sprint issue]
  - Position delta panel — NEW, replaces the old OTOS-odometry panel: DEV exposes
    no world pose (no OTOS/odometry faceplate implemented this sprint), but
    `DEV M <n> STATE`'s `pos=` (onboard encoder position, degrees) is available.
    Plotted relative to each side's first sample after connect (a raw absolute
    reading isn't bounded around zero the way OTOS was), so this becomes a
    wheel-rotation/drift-since-connect view instead of a world-frame trace.

KEYS (focus the plot window) — unchanged from the old tool:
  SPACE        connect / disconnect (fresh serial each time — survives reboots)
  1..9         set speed: 1 = below the dead-zone (~crawl), 9 = max wheel speed
  0            stop (speed 0)
  LEFT / RIGHT step the wheel-ratio: RIGHT walks the phase diagonal toward
               "right wheel only" (vL→0, vertical); LEFT toward "left wheel only"
               (vR→0, horizontal). Centre = balanced (vL=vR). 5 stops each side.

Dependencies: matplotlib, numpy (both already project dependencies via the
`calibrate` uv dependency-group — see pyproject.toml). Only imported in the
interactive (non-headless, non---selftest) path.

Safety: the worker widens the serial-silence watchdog (`DEV WD 3000`) on
connect and always sends `DEV DT STOP` + `DEV STOP` + restores `DEV WD 1000`
on disconnect/SPACE-stop/exit — motors must never be left running.

Usage:
    uv run python tests/bench/velocity_chart.py [--port DEV] [--speed MMPS]
        [--window S] [--max-speed MMPS] [--left-port N] [--right-port N]
        [--poll-period S]

    # No GUI: run the worker for SECS seconds, print a text summary.
    uv run python tests/bench/velocity_chart.py --headless 5

    # No hardware, no GUI: exercise arg parsing + the ratio/jitter math only
    # (this is the testable-without-hardware part per ticket 077-006).
    uv run python tests/bench/velocity_chart.py --selftest
"""

from __future__ import annotations

import argparse
import collections
import queue
import sys
import threading
import time

RATIO_STOPS = 5          # arrow steps from balanced to one-wheel-only (each side)
MEASURE_AGE = 1.0         # seconds-ago mark where the jitter RMSE is read (settled)
SESSION_WATCHDOG_WINDOW = 3000    # [ms]
BOOT_WATCHDOG_WINDOW = 1000       # [ms] firmware default — restored on disconnect


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Interactive robot bench dashboard (DEV protocol)")
    p.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    p.add_argument("--left-port", type=int, default=1, help="DEV DT PORTS left (default 1)")
    p.add_argument("--right-port", type=int, default=2, help="DEV DT PORTS right (default 2)")
    p.add_argument("--speed", type=int, default=200,
                   help="Initial wheel speed mm/s (default 200)")
    p.add_argument("--window", type=float, default=8.0,
                   help="Rolling window seconds (default 8)")
    p.add_argument("--max-speed", type=int, default=400,
                   help="Speed for key 9 / max wheel speed mm/s (default 400)")
    p.add_argument("--poll-period", type=float, default=0.03,
                   help="Target seconds between DEV M STATE polls (default 0.03; actual "
                        "rate is bounded by serial round-trip latency, not this value — "
                        "there is no TLM push-stream in this bench firmware)")
    p.add_argument("--headless", type=float, default=None, metavar="SECS",
                   help="No GUI: run the stream worker for SECS seconds and print "
                        "the collected samples + summary.")
    p.add_argument("--selftest", action="store_true",
                   help="No GUI, no hardware: exercise argument parsing, the ratio-target "
                        "math, and the moving-average/jitter math against synthetic data, "
                        "then exit. The testable-without-hardware part of this tool.")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Pure helpers — shared by the worker, the GUI, and --selftest.
# ---------------------------------------------------------------------------

def ratio_targets(base_speed: float, ratio_idx: int,
                   ratio_stops: int = RATIO_STOPS) -> tuple[int, int]:
    """Map (base_speed, ratio_idx) to (left, right) wheel targets, mm/s.

    ratio_idx in [-ratio_stops, +ratio_stops]: 0 = balanced (vL=vR=base_speed);
    +ratio_stops -> right-wheel-only (vL=0); -ratio_stops -> left-wheel-only (vR=0).
    """
    frac = ratio_idx / float(ratio_stops)      # -1..+1
    if frac >= 0:                              # toward right-wheel-only (vL→0)
        vL, vR = base_speed * (1.0 - frac), base_speed
    else:                                      # toward left-wheel-only (vR→0)
        vL, vR = base_speed, base_speed * (1.0 + frac)
    return int(round(vL)), int(round(vR))


def dev_send(proto, cmd: str, timeout: int = 300):  # [ms]
    """Send one DEV command, return the first OK/ERR reply line, parsed."""
    from robot_radio.robot.protocol import parse_response
    resp = proto.send(cmd, timeout)
    for raw in resp.get("responses", []):
        r = parse_response(raw)
        if r is not None and r.tag in ("OK", "ERR"):
            return r
    return None


def _kv_float(r, key: str) -> float | None:
    if r is None or key not in r.kv:
        return None
    try:
        return float(r.kv[key])
    except ValueError:
        return None


# ---------------------------------------------------------------------------
# Streaming worker — one instance per SPACE-start; torn down on SPACE-stop.
# ---------------------------------------------------------------------------

def _stream_worker(
    port: str | None,
    cmd_box: list,                     # cmd_box[0] = (vL, vR) target, updated by main
    data_queue: "queue.Queue",         # pushes (t, vL, vR, aL, aR, pL, pR, wL, wR) tuples
    stop_event: threading.Event,
    status_queue: "queue.Queue[str]",
    left_port: int = 1,
    right_port: int = 2,
    poll_period: float = 0.03,
) -> None:
    """Open a fresh serial connection, drive cmd_box, push DEV STATE samples.

    Polls `DEV M <left_port> STATE` / `DEV M <right_port> STATE` in a tight
    loop and re-issues `DEV DT WHEELS <vL> <vR>` whenever cmd_box changes (so
    the GUI can change speed/ratio on the fly). Every DEV line sent — polls
    included — resets the firmware's serial-silence watchdog, so this loop
    alone keeps the connection alive; no separate keepalive is needed.
    """
    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol

    def _cmd():
        try:
            return cmd_box[0]
        except Exception:
            return (0, 0)

    conn = None
    proto = None
    try:
        conn = SerialConnection(port=port)   # mode=None: auto-detect direct vs. relay
        status_queue.put("CONNECTING")
        info = conn.connect()
        if "error" in info:
            status_queue.put(f"FAILED: {info['error']}")
            return
        if stop_event.is_set():
            return

        proto = NezhaProtocol(conn)
        dev_send(proto, f"DEV WD {SESSION_WATCHDOG_WINDOW}")
        dev_send(proto, f"DEV DT PORTS {left_port} {right_port}")

        vL, vR = _cmd()
        dev_send(proto, f"DEV DT WHEELS {vL} {vR}")

        status_queue.put(f"RUNNING — mode={info.get('mode')}")

        pos_baseline: tuple[float | None, float | None] | None = None

        while not stop_event.is_set():
            t = time.monotonic()
            left_state = dev_send(proto, f"DEV M {left_port} STATE", timeout=200)
            right_state = dev_send(proto, f"DEV M {right_port} STATE", timeout=200)

            vl, vr = _kv_float(left_state, "vel"), _kv_float(right_state, "vel")
            al, ar = _kv_float(left_state, "applied"), _kv_float(right_state, "applied")
            pl, pr = _kv_float(left_state, "pos"), _kv_float(right_state, "pos")
            wl = left_state.kv.get("wedged") if left_state else None
            wr = right_state.kv.get("wedged") if right_state else None
            cl = left_state.kv.get("conn") if left_state else None
            cr = right_state.kv.get("conn") if right_state else None

            if pos_baseline is None:
                pos_baseline = (pl, pr)
            dl = None if pl is None or pos_baseline[0] is None else pl - pos_baseline[0]
            dr = None if pr is None or pos_baseline[1] is None else pr - pos_baseline[1]

            data_queue.put((t, vl, vr, al, ar, dl, dr, wl, wr, cl, cr))

            nvL, nvR = _cmd()
            if (nvL, nvR) != (vL, vR):
                dev_send(proto, f"DEV DT WHEELS {nvL} {nvR}", timeout=200)
                vL, vR = nvL, nvR

            time.sleep(poll_period)

    except OSError as exc:
        if getattr(exc, "errno", None) == 6:
            status_queue.put("DISCONNECTED — power cycle detected, press SPACE to reconnect")
        else:
            status_queue.put(f"ERROR: {exc}")
    except Exception as exc:
        status_queue.put(f"ERROR: {exc}")
    finally:
        status_queue.put("STOPPED")
        if proto is not None:
            try:
                dev_send(proto, "DEV DT STOP")
                dev_send(proto, "DEV STOP")
                dev_send(proto, f"DEV WD {BOOT_WATCHDOG_WINDOW}")
            except Exception:
                pass
        if conn is not None:
            try:
                conn.disconnect()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# --selftest — no hardware, no GUI: exercise parsing + math only.
# ---------------------------------------------------------------------------

def _run_selftest() -> int:
    from robot_radio.robot.protocol import parse_response

    checks: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        checks.append((name, ok, detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    # 1. Ratio-target math at the extremes and centre.
    check("ratio_targets balanced", ratio_targets(200, 0) == (200, 200))
    check("ratio_targets full-right (vL->0)", ratio_targets(200, RATIO_STOPS) == (0, 200))
    check("ratio_targets full-left (vR->0)", ratio_targets(200, -RATIO_STOPS) == (200, 0))
    mid = ratio_targets(200, RATIO_STOPS // 2)
    check("ratio_targets midpoint is between balanced and full-right",
          0 < mid[0] < 200 and mid[1] == 200, str(mid))

    # 2. DEV STATE reply parsing (the same shape the worker's dev_send() reads).
    line = "OK DEV M 1 pos=123.4 vel=80.1 applied=0.28 wedged=0 conn=1"
    r = parse_response(line)
    check("parse_response reads DEV M STATE reply", r is not None and r.tag == "OK", str(r))
    check("kv fields extracted",
          r is not None and _kv_float(r, "vel") == 80.1 and _kv_float(r, "applied") == 0.28,
          str(r.kv) if r else "")
    check("wedged/conn tokens readable",
          r is not None and r.kv.get("wedged") == "0" and r.kv.get("conn") == "1")

    err_line = "ERR unsupported volt"
    er = parse_response(err_line)
    check("parse_response reads ERR reply", er is not None and er.tag == "ERR", str(er))

    # 3. Moving-average / jitter math (the exact numpy calls _update() uses),
    #    run against a synthetic velocity series.
    import numpy as np
    t = np.linspace(0, 8, 400)
    v = 200 + 5 * np.sin(t * 3.0)   # noisy-ish synthetic wheel velocity
    n200 = max(2, int(round(0.2 / max(1e-3, float(np.median(np.diff(t)))))))
    k = np.ones(n200) / n200
    ma = np.convolve(v, k, mode="same")
    check("moving average shape matches input", ma.shape == v.shape)
    rmse = np.sqrt(np.convolve((v - ma) ** 2, k, mode="same"))
    check("jitter RMSE is finite and non-negative", bool(np.all(np.isfinite(rmse)) and np.all(rmse >= 0)))

    passed = sum(1 for _, ok, _ in checks if ok)
    total = len(checks)
    print(f"\n{passed}/{total} selftest checks passed")
    return 0 if passed == total else 1


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse_args()

    if args.selftest:
        return _run_selftest()

    if args.port is None:
        from robot_radio.io.serial_conn import list_serial_ports
        ports = list_serial_ports()
        if not ports:
            print("ERROR: no USB modem serial ports found.")
            return 2
        port = ports[0]
    else:
        port = args.port
    print(f"  port: {port}   speed: {args.speed} mm/s   window: {args.window} s"
          f"   DEV DT PORTS {args.left_port} {args.right_port}")

    # ---- headless diagnostic ----
    if args.headless is not None:
        cmd_box = [(args.speed, args.speed)]
        data_q: "queue.Queue" = queue.Queue()
        status_q: "queue.Queue[str]" = queue.Queue()
        stop_ev = threading.Event()
        th = threading.Thread(
            target=_stream_worker,
            args=(port, cmd_box, data_q, stop_ev, status_q,
                  args.left_port, args.right_port, args.poll_period),
            daemon=True,
        )
        th.start()
        t_end = time.monotonic() + args.headless
        samples = []
        while time.monotonic() < t_end:
            try:
                while True:
                    samples.append(data_q.get_nowait())
            except queue.Empty:
                pass
            try:
                while True:
                    print(f"  [status] {status_q.get_nowait()}")
            except queue.Empty:
                pass
            time.sleep(0.1)
        stop_ev.set()
        th.join(timeout=3.0)
        vel = [(t, s[0], s[1]) for (t, *s) in samples if s[0] is not None or s[1] is not None]
        print(f"\n  collected {len(samples)} samples ({len(vel)} with vel)")
        if vel:
            t0 = vel[0][0]
            for ts, vl, vr in vel[::max(1, len(vel) // 30)]:
                print(f"    t={ts - t0:5.2f}s  vL={vl}  vR={vr} mm/s")
            vls = [v for _, v, _ in vel if v is not None]
            vrs = [v for _, _, v in vel if v is not None]
            nz = sum(1 for _, vl, vr in vel if vl or vr)
            if vls and vrs:
                print(f"  vL range [{min(vls)}, {max(vls)}]  vR range [{min(vrs)}, {max(vrs)}]")
            print(f"  nonzero: {nz}/{len(vel)}  "
                  + (">>> WHEELS DROVE" if nz else ">>> ALL ZERO"))
        else:
            print("  no vel samples — check connection")
        return 0

    print("  Keys: SPACE connect | 1-9 speed | 0 stop | LEFT/RIGHT wheel-ratio")

    import matplotlib
    import platform
    if platform.system() == "Darwin":
        matplotlib.use("MacOSX")
    else:
        matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import numpy as np

    plt.style.use("dark_background")

    window_s = args.window
    maxlen = int(window_s * 60)          # generous: up to 60 Hz of samples
    # Speed map for keys 1..9: 1 = below dead-zone crawl, 9 = max wheel speed.
    SPEEDS = list(np.linspace(15, args.max_speed, 9).round().astype(int))

    # ---- shared command state (read by worker, written by key handler) ----
    cmd_box = [(args.speed, args.speed)]
    base_speed = [args.speed]              # mm/s magnitude
    ratio_idx = [0]                         # -RATIO_STOPS..+RATIO_STOPS

    def _recompute_cmd():
        cmd_box[0] = ratio_targets(base_speed[0], ratio_idx[0])
    _recompute_cmd()

    # ---- telemetry buffers ----
    t_buf = collections.deque(maxlen=maxlen)
    vL_buf = collections.deque(maxlen=maxlen)
    vR_buf = collections.deque(maxlen=maxlen)
    aL_buf = collections.deque(maxlen=maxlen)
    aR_buf = collections.deque(maxlen=maxlen)
    pL_buf = collections.deque(maxlen=maxlen)
    pR_buf = collections.deque(maxlen=maxlen)
    # carry-forward holders for fields missing on a given poll
    last = {"vel": (0.0, 0.0), "applied": (0.0, 0.0), "pos": (0.0, 0.0),
            "wedged": ("0", "0"), "conn": ("0", "0")}

    data_queue = queue.Queue()
    status_queue = queue.Queue()
    worker_state = {"thread": None, "stop": None}

    # ---- figure / layout (same shape as the old dashboard; contents swapped
    # per the module docstring's panel notes) ----
    fig = plt.figure(figsize=(14, 9))
    title_text = fig.suptitle("Robot bench dashboard  [SPACE = connect]",
                              color="white", fontsize=12)
    gs = fig.add_gridspec(4, 2, width_ratios=[2.2, 1.0],
                          height_ratios=[3.0, 1.6, 0.9, 0.7],
                          hspace=0.55, wspace=0.22)
    ax_vel     = fig.add_subplot(gs[0, 0])
    ax_jit     = fig.add_subplot(gs[1, 0], sharex=ax_vel)
    ax_applied = fig.add_subplot(gs[2, 0], sharex=ax_vel)
    ax_status  = fig.add_subplot(gs[3, 0])
    ax_phase   = fig.add_subplot(gs[0:2, 1])
    ax_pos     = fig.add_subplot(gs[2:4, 1])

    # velocity panel
    ax_vel.set_title("Wheel velocity (mm/s)", fontsize=10)
    ax_vel.set_ylabel("mm/s", fontsize=8)
    ax_vel.set_xlim(window_s, 0)   # newest (0 s ago) on the RIGHT, older to the left
    ax_vel.set_ylim(-50, args.max_speed + 60)
    ax_vel.grid(True, alpha=0.3)
    (ln_vL,)  = ax_vel.plot([], [], color="deepskyblue", lw=1.2, label="vL")
    (ln_vR,)  = ax_vel.plot([], [], color="tomato",      lw=1.2, label="vR")
    (ln_vLa,) = ax_vel.plot([], [], color="deepskyblue", lw=2.0, alpha=0.45)
    (ln_vRa,) = ax_vel.plot([], [], color="tomato",      lw=2.0, alpha=0.45)
    set_vL = ax_vel.axhline(cmd_box[0][0], color="deepskyblue", ls=":", lw=1.0, alpha=0.7)
    set_vR = ax_vel.axhline(cmd_box[0][1], color="tomato",      ls=":", lw=1.0, alpha=0.7)
    ax_vel.legend(fontsize=7, loc="upper right", ncol=2)

    # jitter panel
    ax_jit.set_title("Jitter — rolling RMSE vs 200 ms moving avg (mm/s)", fontsize=9)
    ax_jit.set_ylabel("RMSE", fontsize=8)
    ax_jit.set_xlim(window_s, 0)
    ax_jit.set_ylim(0, 30)
    ax_jit.grid(True, alpha=0.3)
    ax_jit.axvline(MEASURE_AGE, color="white", ls=":", lw=1.0, alpha=0.6)
    (ln_jL,) = ax_jit.plot([], [], color="deepskyblue", lw=1.2, label="vL")
    (ln_jR,) = ax_jit.plot([], [], color="tomato",      lw=1.2, label="vR")
    jit_text = ax_jit.text(0.01, 0.92, "", transform=ax_jit.transAxes,
                           fontsize=8, color="white", va="top")
    ax_jit.legend(fontsize=7, loc="upper right", ncol=2)

    # applied-duty panel (replaces the old colour-sensor strip — see docstring)
    ax_applied.set_title("Applied duty (fraction, DEV M STATE applied=)", fontsize=9)
    ax_applied.set_ylabel("duty", fontsize=8)
    ax_applied.set_xlim(window_s, 0)
    ax_applied.set_ylim(-1.05, 1.05)
    ax_applied.grid(True, alpha=0.3)
    (ln_aL,) = ax_applied.plot([], [], color="deepskyblue", lw=1.2, label="aL")
    (ln_aR,) = ax_applied.plot([], [], color="tomato",      lw=1.2, label="aR")
    ax_applied.legend(fontsize=7, loc="upper right", ncol=2)

    # status strip (replaces the old 4-channel line-sensor strip — see docstring)
    ax_status.set_title("Status (wedged / conn)", fontsize=9)
    ax_status.set_xlabel("seconds ago", fontsize=8)
    ax_status.set_xticks([])
    ax_status.set_yticks([])
    status_text = ax_status.text(0.02, 0.5, "", transform=ax_status.transAxes,
                                 fontsize=9, color="white", va="center", family="monospace")

    # phase plot
    lim = args.max_speed + 40
    ax_phase.set_title("Phase: vR vs vL (ratio diagonal)", fontsize=10)
    ax_phase.set_xlabel("vL (mm/s)", fontsize=8)
    ax_phase.set_ylabel("vR (mm/s)", fontsize=8)
    ax_phase.set_xlim(-lim, lim)
    ax_phase.set_ylim(-lim, lim)
    ax_phase.set_aspect("equal")
    ax_phase.grid(True, alpha=0.3)
    ax_phase.plot([-lim, lim], [-lim, lim], color="lightgrey", ls="--", lw=1.0,
                  alpha=0.55, label="1:1 (vR=vL)")
    (ratio_line,)  = ax_phase.plot([], [], color="yellow", ls=":", lw=1.6,
                                   alpha=0.85, label="ratio")
    (phase_trace,) = ax_phase.plot([], [], color="grey", lw=0.8, alpha=0.6)
    (phase_dot,)   = ax_phase.plot([], [], "o", color="red", ms=8, label="actual")
    (phase_cmd,)   = ax_phase.plot([], [], "X", color="orange", ms=9, label="cmd pt")
    ax_phase.legend(fontsize=7, loc="upper left")

    def _update_ratio_line():
        # Dotted line through the origin at the commanded wheel-ratio:
        # balanced -> 45° diagonal; RIGHT -> vertical (vL→0); LEFT -> horizontal (vR→0).
        frac = ratio_idx[0] / float(RATIO_STOPS)
        if frac >= 0:
            dL, dR = (1.0 - frac), 1.0
        else:
            dL, dR = 1.0, (1.0 + frac)
        md = max(dL, dR, 1e-6)
        ratio_line.set_data([-lim * dL / md, lim * dL / md],
                            [-lim * dR / md, lim * dR / md])
    _update_ratio_line()

    # position-delta panel (replaces the old OTOS-odometry panel — see docstring)
    ax_pos.set_title("Position Δ since connect (deg, DEV M STATE pos=)", fontsize=10)
    ax_pos.set_xlabel("seconds ago", fontsize=8)
    ax_pos.set_ylabel("deg", fontsize=8)
    ax_pos.set_xlim(window_s, 0)
    ax_pos.grid(True, alpha=0.3)
    (ln_pL,) = ax_pos.plot([], [], color="limegreen", lw=1.2, label="L")
    (ln_pR,) = ax_pos.plot([], [], color="violet",    lw=1.2, label="R")
    ax_pos.legend(fontsize=7, loc="upper left", ncol=2)

    # ---- key handler ----
    def _start_worker():
        stop_ev = threading.Event()
        worker_state["stop"] = stop_ev
        worker_state["thread"] = threading.Thread(
            target=_stream_worker,
            args=(port, cmd_box, data_queue, stop_ev, status_queue,
                  args.left_port, args.right_port, args.poll_period),
            daemon=True,
        )
        worker_state["thread"].start()

    def _on_key(event):
        k = event.key
        if k == " ":
            th = worker_state["thread"]
            if th is not None and th.is_alive():
                worker_state["stop"].set()
                title_text.set_text("Robot bench dashboard  [SPACE = connect]")
            else:
                _start_worker()
                title_text.set_text("Robot bench dashboard  [connecting…]")
        elif k in "123456789":
            base_speed[0] = SPEEDS[int(k) - 1]
            _recompute_cmd()
        elif k == "0":
            base_speed[0] = 0
            _recompute_cmd()
        elif k == "right":
            ratio_idx[0] = min(RATIO_STOPS, ratio_idx[0] + 1)
            _recompute_cmd()
        elif k == "left":
            ratio_idx[0] = max(-RATIO_STOPS, ratio_idx[0] - 1)
            _recompute_cmd()
        else:
            return
        set_vL.set_ydata([cmd_box[0][0], cmd_box[0][0]])
        set_vR.set_ydata([cmd_box[0][1], cmd_box[0][1]])
        _update_ratio_line()
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("key_press_event", _on_key)

    def _movavg(v, n):
        if n < 2 or len(v) < n:
            return v
        k = np.ones(n) / n
        return np.convolve(v, k, mode="same")

    def _safe_set(line, x, y):
        # Trim x and y to the SAME (shortest) length before set_data(). The
        # worker thread appends to the plot deques while we read them here, so
        # a single time-series can still be momentarily 1 sample out of step
        # even after the n=min snapshot below — and matplotlib crashes when it
        # later recaches a line whose x/y lengths differ.
        m = min(len(x), len(y))
        line.set_data(x[-m:], y[-m:])

    # ---- update ----
    def _update():
        while not status_queue.empty():
            msg = status_queue.get_nowait()
            if msg.startswith("RUNNING"):
                title_text.set_text(f"Robot bench dashboard  ▶ {msg}")
            elif msg.startswith(("DISCONNECTED", "ERROR", "FAILED")):
                title_text.set_text(f"Robot bench dashboard  ⚠ {msg}  [SPACE=retry]")

        try:
            while True:
                t, vl, vr, al, ar, dl, dr, wl, wr, cl, cr = data_queue.get_nowait()
                if vl is not None or vr is not None:
                    last["vel"] = (vl if vl is not None else last["vel"][0],
                                    vr if vr is not None else last["vel"][1])
                if al is not None or ar is not None:
                    last["applied"] = (al if al is not None else last["applied"][0],
                                        ar if ar is not None else last["applied"][1])
                if dl is not None or dr is not None:
                    last["pos"] = (dl if dl is not None else last["pos"][0],
                                    dr if dr is not None else last["pos"][1])
                if wl is not None or wr is not None:
                    last["wedged"] = (wl if wl is not None else last["wedged"][0],
                                       wr if wr is not None else last["wedged"][1])
                if cl is not None or cr is not None:
                    last["conn"] = (cl if cl is not None else last["conn"][0],
                                     cr if cr is not None else last["conn"][1])
                t_buf.append(t)
                vL_buf.append(last["vel"][0])
                vR_buf.append(last["vel"][1])
                aL_buf.append(last["applied"][0])
                aR_buf.append(last["applied"][1])
                pL_buf.append(last["pos"][0])
                pR_buf.append(last["pos"][1])
        except queue.Empty:
            pass

        # update commanded markers (in case ratio changed)
        phase_cmd.set_data([cmd_box[0][0]], [cmd_box[0][1]])
        ax_vel.set_title(
            f"Wheel velocity (mm/s)   cmd L={cmd_box[0][0]} R={cmd_box[0][1]}"
            f"   speed={base_speed[0]}  ratio={ratio_idx[0]:+d}/{RATIO_STOPS}",
            fontsize=10)
        status_text.set_text(
            f"wedged  L={last['wedged'][0]}  R={last['wedged'][1]}\n"
            f"conn    L={last['conn'][0]}  R={last['conn'][1]}")

        if not t_buf:
            return

        # Snapshot the buffers to a COMMON length first (see _safe_set's note).
        tl, vll, vrl = list(t_buf), list(vL_buf), list(vR_buf)
        all_, arl = list(aL_buf), list(aR_buf)
        pll, prl = list(pL_buf), list(pR_buf)
        n = min(len(tl), len(vll), len(vrl), len(all_), len(arl), len(pll), len(prl))
        if n < 1:
            return
        t_arr = np.array(tl[-n:])
        now = t_arr[-1]
        age = now - t_arr            # 0 = newest (right edge), grows to the left
        vl = np.array(vll[-n:], dtype=float)
        vr = np.array(vrl[-n:], dtype=float)
        al = np.array(all_[-n:], dtype=float)
        ar = np.array(arl[-n:], dtype=float)
        pl_arr = np.array(pll[-n:], dtype=float)
        pr_arr = np.array(prl[-n:], dtype=float)

        _safe_set(ln_vL, age, vl)
        _safe_set(ln_vR, age, vr)
        _safe_set(ln_aL, age, al)
        _safe_set(ln_aR, age, ar)

        # 200 ms moving average + rolling RMSE jitter metric. Measured at the
        # 1 s-ago mark (MEASURE_AGE) where the averaging window is fully settled
        # (the newest edge is still filling, so its RMSE is unreliable).
        if len(t_arr) > 3:
            dt = max(1e-3, float(np.median(np.diff(t_arr))))
            n200 = max(2, int(round(0.2 / dt)))
            mret = max(3, int(round(1.0 / dt)))          # 1 s rolling RMSE window
            maL, maR = _movavg(vl, n200), _movavg(vr, n200)
            _safe_set(ln_vLa, age, maL)
            _safe_set(ln_vRa, age, maR)
            kk = np.ones(mret) / mret
            rmsL = np.sqrt(np.convolve((vl - maL) ** 2, kk, mode="same"))
            rmsR = np.sqrt(np.convolve((vr - maR) ** 2, kk, mode="same"))
            _safe_set(ln_jL, age, rmsL)
            _safe_set(ln_jR, age, rmsR)
            idx1 = int(np.argmin(np.abs(age - MEASURE_AGE)))
            jit_text.set_text(
                f"RMSE @{MEASURE_AGE:.0f}s  vL={rmsL[idx1]:4.1f}  vR={rmsR[idx1]:4.1f} mm/s")
            top = max(8.0, float(np.nanmax(np.concatenate([rmsL, rmsR]))) * 1.2)
            ax_jit.set_ylim(0, top)

        # phase
        _safe_set(phase_trace, vl, vr)
        phase_dot.set_data([vl[-1]], [vr[-1]])

        # position delta
        _safe_set(ln_pL, age, pl_arr)
        _safe_set(ln_pR, age, pr_arr)
        pmin = min(pl_arr.min(), pr_arr.min())
        pmax = max(pl_arr.max(), pr_arr.max())
        if pmax > pmin:
            pad = 0.1 * (pmax - pmin) + 5
            ax_pos.set_ylim(pmin - pad, pmax + pad)

    plt.ion()
    plt.show(block=False)
    try:
        while plt.fignum_exists(fig.number):
            _update()
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(0.033)
    except KeyboardInterrupt:
        pass
    finally:
        if worker_state["stop"] is not None:
            worker_state["stop"].set()
        th = worker_state["thread"]
        if th is not None:
            th.join(timeout=2.0)
        plt.close("all")

    return 0


if __name__ == "__main__":
    sys.exit(main())
