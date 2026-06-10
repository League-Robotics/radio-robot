#!/usr/bin/env python3
"""velocity_chart.py — interactive real-time robot bench dashboard.

Streams live telemetry and renders a multi-panel matplotlib dashboard while you
drive and physically interfere with the robot (grab wheels, slide colors/lines
under the sensors). Designed for tuning the velocity loop and watching the
sensors at the same time.

PANELS (left column shares the time axis):
  - Wheel velocity (vL, vR) with 200 ms moving-average overlay + commanded setpoints
  - Jitter metric: rolling RMSE of each wheel vs its own 200 ms moving average
  - Colour strip: reconstructed RGB under the colour sensor (0x43)
  - Line strip: 4 line-sensor channels as on/off bands (adaptive threshold)
RIGHT column:
  - Phase plot: vR vs vL (the wheel-ratio diagonal) with commanded-target dot
  - Odometry: OTOS pose x,y (mm) vs time

KEYS (focus the plot window):
  SPACE        connect / disconnect (fresh serial each time — survives reboots)
  1..9         set speed: 1 = below the dead-zone (~crawl), 9 = max wheel speed
  0            stop (speed 0)
  LEFT / RIGHT step the wheel-ratio: RIGHT walks the phase diagonal toward
               "right wheel only" (vL→0, vertical); LEFT toward "left wheel only"
               (vR→0, horizontal). Centre = balanced (vL=vR). 5 stops each side.

Usage:
    uv run python tests/bench/velocity_chart.py [--port DEV] [--speed MMPS]
        [--window S] [--max-speed MMPS] [--stream-ms MS]
"""

import argparse
import collections
import queue
import sys
import threading
import time

RATIO_STOPS = 5          # arrow steps from balanced to one-wheel-only (each side)
MEASURE_AGE = 1.0        # seconds-ago mark where the jitter RMSE is read (settled)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Interactive robot bench dashboard")
    p.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    p.add_argument("--speed", type=int, default=200,
                   help="Initial wheel speed mm/s (default 200)")
    p.add_argument("--window", type=float, default=8.0,
                   help="Rolling window seconds (default 8)")
    p.add_argument("--max-speed", type=int, default=400,
                   help="Speed for key 9 / max wheel speed mm/s (default 400 = vWheelMax)")
    p.add_argument("--stream-ms", type=int, default=40,
                   help="TLM stream period ms (default 40 = 25 Hz, for jitter resolution)")
    p.add_argument("--headless", type=float, default=None, metavar="SECS",
                   help="No GUI: run the stream worker for SECS seconds and print "
                        "the collected (vL,vR) samples + summary.")
    p.add_argument("--set", dest="sets", action="append", default=[], metavar="K=V",
                   help="Apply a SET override on connect (repeatable), e.g. "
                        "--set sync=1 --set vel.kI=0.15.")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Streaming worker — one instance per SPACE-start; torn down on SPACE-stop.
# ---------------------------------------------------------------------------

def _stream_worker(
    port: str,
    cmd_box: list,                     # cmd_box[0] = (vL, vR) target, updated by main
    data_queue: "queue.Queue",         # pushes (t, vel, pose, line, color) tuples
    stop_event: threading.Event,
    status_queue: "queue.Queue[str]",
    sets: "list[str] | None" = None,
    stream_ms: int = 40,
) -> None:
    """Open a fresh serial connection, drive cmd_box, push telemetry samples.

    Same robust connect/calibrate/zero logic as before; differences:
      - drives the LIVE cmd_box[0] = (vL, vR) (not a fixed speed) so the GUI can
        change speed / wheel-ratio on the fly.
      - streams ALL fields (enc,vel,pose,line,color) so the dashboard can show
        the sensors, not just velocity.
    """
    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol, parse_response, parse_tlm
    from robot_radio.robot.nezha import Nezha

    KEEPALIVE_S = 0.150

    def _cmd():
        try:
            return cmd_box[0]
        except Exception:
            return (0, 0)

    conn = None
    try:
        conn = SerialConnection(port=port, mode="direct")
        conn.connect(skip_ping=True)

        proto = NezhaProtocol(conn)
        nezha = Nezha(proto)

        status_queue.put("CONNECTING")
        deadline = time.monotonic() + 12.0
        identity = None
        while time.monotonic() < deadline and not stop_event.is_set():
            try:
                identity = nezha.connect()
                break
            except Exception:
                time.sleep(0.4)

        if identity is None or stop_event.is_set():
            status_queue.put("FAILED")
            return

        # Flush stale buffer.
        try:
            conn.send_fast("STOP")
            conn.send_fast("STREAM 0")
            time.sleep(0.05)
            conn._ser.reset_input_buffer()
        except Exception:
            pass

        # Push calibration (DTR reset boots uncalibrated → wheels won't drive).
        try:
            from robot_radio.io.cli import _push_calibration
            _push_calibration(conn)
            status_queue.put("calibration pushed")
        except Exception as exc:
            status_queue.put(f"WARN: calibration push failed: {exc}")

        # Zero encoders (fresh boot freezes readback at 0 until ZERO enc).
        try:
            proto.zero_encoders()
        except Exception as exc:
            status_queue.put(f"WARN: zero encoders failed: {exc}")

        # Live SET overrides.
        for kv in (sets or []):
            try:
                r = proto.send(f"SET {kv}", 250)
                status_queue.put(f"SET {kv} -> {r.get('responses', ['?'])[-1]}")
            except Exception as exc:
                status_queue.put(f"WARN: SET {kv} failed: {exc}")

        # Generous watchdog.
        try:
            proto.send("SET sTimeout=10000", 300)
        except Exception:
            pass

        # Stream ALL fields so the sensors show up, then start streaming.
        try:
            proto.stream_fields("enc,vel,pose,line,color")
        except Exception as exc:
            status_queue.put(f"WARN: stream_fields failed: {exc}")
        proto.stream(stream_ms)

        status_queue.put(f"RUNNING — {identity.get('name', '?')}")

        vL, vR = _cmd()
        conn.send_fast(f"S {vL} {vR}")
        last_send = time.monotonic()

        # Request raw OTOS position every N read-loop iterations so the odom
        # panel shows actual optical-flow data rather than fused pose (which
        # includes encoder dead-reckoning and drifts when wheels spin freely).
        _op_interval = 5   # send OP every 5 iterations ≈ every 150 ms
        _op_counter = 0

        while not stop_event.is_set():
            _op_counter += 1
            if _op_counter >= _op_interval:
                _op_counter = 0
                conn.send_fast("OP")

            for raw_line in conn.read_lines(duration_ms=30):
                r = parse_response(raw_line)
                if r is None:
                    continue
                if r.tag == "EVT" and r.tokens and r.tokens[0] == "safety_stop":
                    vL, vR = _cmd()
                    conn.send_fast(f"S {vL} {vR}")
                    last_send = time.monotonic()
                    continue
                # Raw OTOS position reply — use this for the odom panel.
                if r.tag == "OK" and r.tokens and r.tokens[0] == "op":
                    try:
                        x = int(r.kv["x"])
                        y = int(r.kv["y"])
                        data_queue.put((time.monotonic(), None, (x, y, 0),
                                        None, None))
                    except (KeyError, ValueError):
                        pass
                    continue
                if r.tag == "TLM":
                    tlm = parse_tlm(r.raw)
                    if tlm is not None:
                        data_queue.put((time.monotonic(), tlm.vel, None,
                                        tlm.line, tlm.color))

            now = time.monotonic()
            if now - last_send >= KEEPALIVE_S:
                vL, vR = _cmd()
                conn.send_fast(f"S {vL} {vR}")
                last_send = now

    except OSError as exc:
        if exc.errno == 6:
            status_queue.put("DISCONNECTED — power cycle detected, press SPACE to reconnect")
        else:
            status_queue.put(f"ERROR: {exc}")
    except Exception as exc:
        status_queue.put(f"ERROR: {exc}")
    finally:
        status_queue.put("STOPPED")
        if conn is not None:
            try:
                for _ in range(3):
                    conn.send_fast("STOP")
                    time.sleep(0.05)
                from robot_radio.robot.protocol import NezhaProtocol
                NezhaProtocol(conn).stream(0)
            except Exception:
                pass
            try:
                conn.disconnect()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse_args()

    if args.port is None:
        from robot_radio.io.serial_conn import list_serial_ports
        ports = list_serial_ports()
        if not ports:
            print("ERROR: no USB modem serial ports found.")
            return 2
        port = ports[0]
    else:
        port = args.port
    print(f"  port: {port}   speed: {args.speed} mm/s   window: {args.window} s")

    # ---- headless diagnostic ----
    if args.headless is not None:
        cmd_box = [(args.speed, args.speed)]
        data_q: "queue.Queue" = queue.Queue()
        status_q: "queue.Queue[str]" = queue.Queue()
        stop_ev = threading.Event()
        th = threading.Thread(
            target=_stream_worker,
            args=(port, cmd_box, data_q, stop_ev, status_q, args.sets, args.stream_ms),
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
        vel = [(t, v) for (t, v, *_rest) in samples if v is not None]
        print(f"\n  collected {len(samples)} samples ({len(vel)} with vel)")
        if vel:
            t0 = vel[0][0]
            for ts, v in vel[::max(1, len(vel) // 30)]:
                print(f"    t={ts - t0:5.2f}s  vL={v[0]:>5}  vR={v[1]:>5} mm/s")
            vls = [v[0] for _, v in vel]
            vrs = [v[1] for _, v in vel]
            nz = sum(1 for _, v in vel if v[0] or v[1])
            print(f"  vL range [{min(vls)}, {max(vls)}]  vR range [{min(vrs)}, {max(vrs)}]")
            print(f"  nonzero: {nz}/{len(vel)}  "
                  + (">>> WHEELS DROVE" if nz else ">>> ALL ZERO"))
        else:
            print("  no vel samples — check connection/fields")
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
    maxlen   = int(window_s * 60)          # generous: up to 60 Hz of samples
    # Speed map for keys 1..9: 1 = below dead-zone crawl, 9 = max wheel speed.
    SPEEDS = list(np.linspace(15, args.max_speed, 9).round().astype(int))

    # ---- shared command state (read by worker, written by key handler) ----
    cmd_box   = [(args.speed, args.speed)]
    base_speed = [args.speed]              # mm/s magnitude
    ratio_idx  = [0]                       # -RATIO_STOPS..+RATIO_STOPS

    def _recompute_cmd():
        b = base_speed[0]
        frac = ratio_idx[0] / float(RATIO_STOPS)   # -1..+1
        if frac >= 0:                              # toward right-wheel-only (vL→0)
            vL, vR = b * (1.0 - frac), b
        else:                                      # toward left-wheel-only (vR→0)
            vL, vR = b, b * (1.0 + frac)
        cmd_box[0] = (int(round(vL)), int(round(vR)))
    _recompute_cmd()

    # ---- telemetry buffers ----
    t_buf  = collections.deque(maxlen=maxlen)
    vL_buf = collections.deque(maxlen=maxlen)
    vR_buf = collections.deque(maxlen=maxlen)
    px_buf = collections.deque(maxlen=maxlen)
    py_buf = collections.deque(maxlen=maxlen)
    col_buf = collections.deque(maxlen=maxlen)        # (r,g,b) floats 0..1
    line_buf = collections.deque(maxlen=maxlen)       # (4,) raw ints
    # carry-forward holders for fields missing on a given frame
    last = {"vel": (0, 0), "pose": (0, 0, 0), "line": (0, 0, 0, 0),
            "color": (0, 0, 0, 0)}
    line_lo = [1e9, 1e9, 1e9, 1e9]
    line_hi = [1.0, 1.0, 1.0, 1.0]

    data_queue   = queue.Queue()
    status_queue = queue.Queue()
    worker_state = {"thread": None, "stop": None}

    # ---- figure / layout ----
    fig = plt.figure(figsize=(14, 9))
    title_text = fig.suptitle("Robot bench dashboard  [SPACE = connect]",
                              color="white", fontsize=12)
    gs = fig.add_gridspec(4, 2, width_ratios=[2.2, 1.0],
                          height_ratios=[3.0, 1.6, 0.7, 0.9],
                          hspace=0.55, wspace=0.22)
    ax_vel   = fig.add_subplot(gs[0, 0])
    ax_jit   = fig.add_subplot(gs[1, 0], sharex=ax_vel)
    ax_color = fig.add_subplot(gs[2, 0], sharex=ax_vel)
    ax_line  = fig.add_subplot(gs[3, 0], sharex=ax_vel)
    ax_phase = fig.add_subplot(gs[0:2, 1])
    ax_odom  = fig.add_subplot(gs[2:4, 1])

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

    # colour strip (imshow updated each frame)
    ax_color.set_title("Colour under sensor (0x43)", fontsize=9)
    ax_color.set_yticks([])
    img_color = ax_color.imshow(np.zeros((1, 2, 3)), aspect="auto",
                                extent=[window_s, 0, 0, 1], origin="lower",
                                interpolation="nearest", vmin=0, vmax=1)

    # line-sensor strip (4 channels, on/off)
    ax_line.set_title("Line sensor 0x1A — 4 ch on/off (adaptive thresh)", fontsize=9)
    ax_line.set_xlabel("seconds ago", fontsize=8)
    ax_line.set_yticks([0.5, 1.5, 2.5, 3.5])
    ax_line.set_yticklabels(["c0", "c1", "c2", "c3"], fontsize=7)
    img_line = ax_line.imshow(np.zeros((4, 2)), aspect="auto",
                              extent=[window_s, 0, 0, 4], origin="lower",
                              interpolation="nearest", cmap="inferno", vmin=0, vmax=1)

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

    # odometry
    ax_odom.set_title("OTOS raw position (mm)", fontsize=10)
    ax_odom.set_xlabel("seconds ago", fontsize=8)
    ax_odom.set_ylabel("mm", fontsize=8)
    ax_odom.set_xlim(window_s, 0)
    ax_odom.grid(True, alpha=0.3)
    (ln_px,) = ax_odom.plot([], [], color="limegreen", lw=1.2, label="x")
    (ln_py,) = ax_odom.plot([], [], color="violet",    lw=1.2, label="y")
    ax_odom.legend(fontsize=7, loc="upper left", ncol=2)

    # ---- key handler ----
    def _start_worker():
        stop_ev = threading.Event()
        worker_state["stop"] = stop_ev
        worker_state["thread"] = threading.Thread(
            target=_stream_worker,
            args=(port, cmd_box, data_queue, stop_ev, status_queue, args.sets,
                  args.stream_ms),
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

    # ---- update ----
    def _update():
        while not status_queue.empty():
            msg = status_queue.get_nowait()
            if msg.startswith("RUNNING"):
                title_text.set_text(f"Robot bench dashboard  ▶ {msg}")
            elif msg.startswith(("DISCONNECTED", "ERROR")):
                title_text.set_text(f"Robot bench dashboard  ⚠ {msg}  [SPACE=retry]")

        try:
            while True:
                t, vel, pose, line, color = data_queue.get_nowait()
                if vel is not None:
                    last["vel"] = vel
                if pose is not None:
                    last["pose"] = pose
                if line is not None:
                    last["line"] = line
                if color is not None:
                    last["color"] = color
                t_buf.append(t)
                vL_buf.append(last["vel"][0])
                vR_buf.append(last["vel"][1])
                px_buf.append(last["pose"][0])
                py_buf.append(last["pose"][1])
                # reconstruct colour (assume channel order R,G,B,C)
                c = last["color"]
                m = max(c[0], c[1], c[2], 1)
                col_buf.append((min(c[0] / m, 1.0), min(c[1] / m, 1.0),
                                min(c[2] / m, 1.0)))
                ln = last["line"][:4]
                for i in range(4):
                    line_lo[i] = min(line_lo[i], ln[i])
                    line_hi[i] = max(line_hi[i], ln[i])
                line_buf.append(tuple(ln))
        except queue.Empty:
            pass

        # update commanded markers (in case ratio changed)
        phase_cmd.set_data([cmd_box[0][0]], [cmd_box[0][1]])
        title_text.set_text(title_text.get_text())  # keep
        ax_vel.set_title(
            f"Wheel velocity (mm/s)   cmd L={cmd_box[0][0]} R={cmd_box[0][1]}"
            f"   speed={base_speed[0]}  ratio={ratio_idx[0]:+d}/{RATIO_STOPS}",
            fontsize=10)

        if not t_buf:
            return

        t_arr = np.array(t_buf)
        now = t_arr[-1]
        age = now - t_arr            # 0 = newest (right edge), grows to the left
        vl = np.array(vL_buf, dtype=float)
        vr = np.array(vR_buf, dtype=float)

        ln_vL.set_data(age, vl)
        ln_vR.set_data(age, vr)

        # 200 ms moving average + rolling RMSE jitter metric. Measured at the
        # 1 s-ago mark (MEASURE_AGE) where the averaging window is fully settled
        # (the newest edge is still filling, so its RMSE is unreliable).
        if len(t_arr) > 3:
            dt = max(1e-3, float(np.median(np.diff(t_arr))))
            n200 = max(2, int(round(0.2 / dt)))
            mret = max(3, int(round(1.0 / dt)))          # 1 s rolling RMSE window
            maL, maR = _movavg(vl, n200), _movavg(vr, n200)
            ln_vLa.set_data(age, maL)
            ln_vRa.set_data(age, maR)
            kk = np.ones(mret) / mret
            rmsL = np.sqrt(np.convolve((vl - maL) ** 2, kk, mode="same"))
            rmsR = np.sqrt(np.convolve((vr - maR) ** 2, kk, mode="same"))
            ln_jL.set_data(age, rmsL)
            ln_jR.set_data(age, rmsR)
            idx1 = int(np.argmin(np.abs(age - MEASURE_AGE)))
            jit_text.set_text(
                f"RMSE @{MEASURE_AGE:.0f}s  vL={rmsL[idx1]:4.1f}  vR={rmsR[idx1]:4.1f} mm/s")
            top = max(8.0, float(np.nanmax(np.concatenate([rmsL, rmsR]))) * 1.2)
            ax_jit.set_ylim(0, top)

        # colour strip (columns oldest→newest; extent maps to age, newest at 0)
        if col_buf:
            cimg = np.array(col_buf, dtype=float).reshape(1, -1, 3)
            img_color.set_data(cimg)
            img_color.set_extent([age[0], age[-1], 0, 1])

        # line strip — adaptive midpoint threshold per channel → on/off
        if line_buf:
            larr = np.array(line_buf, dtype=float)          # (N,4)
            on = np.zeros((4, larr.shape[0]))
            for i in range(4):
                mid = (line_lo[i] + line_hi[i]) / 2.0
                on[i] = (larr[:, i] > mid).astype(float)
            img_line.set_data(on)
            img_line.set_extent([age[0], age[-1], 0, 4])

        # phase
        phase_trace.set_data(vl, vr)
        phase_dot.set_data([vl[-1]], [vr[-1]])

        # odometry
        ln_px.set_data(age, np.array(px_buf, dtype=float))
        ln_py.set_data(age, np.array(py_buf, dtype=float))
        pmin = min(min(px_buf), min(py_buf))
        pmax = max(max(px_buf), max(py_buf))
        if pmax > pmin:
            pad = 0.1 * (pmax - pmin) + 5
            ax_odom.set_ylim(pmin - pad, pmax + pad)

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
