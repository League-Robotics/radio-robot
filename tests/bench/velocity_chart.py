#!/usr/bin/env python3
"""velocity_chart.py — real-time velocity strip charts + phase plot bench tool.

Streams live telemetry from the robot and renders three updating matplotlib
plots in a window:
  - Left wheel velocity (mm/s) strip chart
  - Right wheel velocity (mm/s) strip chart
  - Phase plot: vR vs vL with reference line and current-point dot

SPACE — connect to robot and start wheels / disconnect and stop wheels.
Each press opens a fresh serial connection, making it resilient to robot
reboots and encoder wedge states.

Usage:
    uv run python tests/bench/velocity_chart.py [--port DEV] [--speed MMPS] [--window S]
"""

import argparse
import collections
import queue
import sys
import threading
import time


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Real-time robot velocity charts")
    p.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    p.add_argument("--speed", type=int, default=200,
                   help="Wheel speed mm/s (default 200)")
    p.add_argument("--window", type=float, default=8.0,
                   help="Rolling window seconds (default 8)")
    p.add_argument("--headless", type=float, default=None, metavar="SECS",
                   help="No GUI: run the stream worker for SECS seconds and "
                        "print the collected (vL,vR) samples + summary. Used to "
                        "diagnose the direct-path drive without a SPACE press.")
    p.add_argument("--set", dest="sets", action="append", default=[], metavar="K=V",
                   help="Apply a SET override on connect (repeatable), e.g. "
                        "--set sync=1 --set vel.kP=0.2. Lets you grab-test gains/"
                        "coupling live without reflashing.")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Streaming worker — one instance per SPACE-start; torn down on SPACE-stop.
# ---------------------------------------------------------------------------

def _stream_worker(
    port: str,
    speed: int,
    data_queue: "queue.Queue[tuple[float, int, int]]",
    stop_event: threading.Event,   # set externally to tear down
    status_queue: "queue.Queue[str]",  # push status strings to main thread
    sets: "list[str] | None" = None,   # extra "key=value" SET overrides on connect
) -> None:
    """Open a fresh serial connection, connect, drive, push (t,vL,vR) tuples.

    Custom robust streaming loop — does NOT use NezhaProtocol.stream_drive so
    it can re-arm on EVT safety_stop rather than exiting on the first one.

    Protocol:
      1. Flush (STOP + STREAM 0 + reset_input_buffer) — drops any stale
         EVT safety_stop left in the serial buffer from a prior run.
      2. SET sTimeout=10000 — generous firmware watchdog; host sends S every
         150 ms which is far inside 10 s.
      3. STREAM 100 — ~10 Hz TLM; low serial load avoids TX buffer overflow
         during GUI stalls.
      4. Send S <speed> <speed> immediately, then re-send every 150 ms
         measured by time.monotonic() — independent of how many lines were
         read in each iteration.
      5. On EVT safety_stop: log, immediately re-send S, continue (re-arm).
         Only exit on stop_event or a real serial OSError.
    """
    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol, parse_response, parse_tlm
    from robot_radio.robot.nezha import Nezha

    KEEPALIVE_S = 0.150   # re-send S every 150 ms; safely inside 10 s watchdog

    conn = None
    try:
        conn = SerialConnection(port=port, mode="direct")
        conn.connect(skip_ping=True)

        proto = NezhaProtocol(conn)
        nezha = Nezha(proto)

        # Retry PING until robot answers (may need up to ~9 s after port reset).
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

        # 1. Flush — drop any stale EVT safety_stop (or leftover TLM) from a
        #    prior session so the streaming loop starts on a clean buffer.
        try:
            conn.send_fast("STOP")
            conn.send_fast("STREAM 0")
            time.sleep(0.05)
            conn._ser.reset_input_buffer()
        except Exception:
            pass

        # 1b. Push calibration — REQUIRED for the wheels to actually drive.
        #     Opening the serial port pulses DTR and resets the micro:bit, so the
        #     robot boots UNCALIBRATED every connect. An uncalibrated robot ACKs
        #     "S <l> <r>" ("OK drive ...") but produces no motor output — the
        #     wheels never turn and the encoders read a constant 0, which looks
        #     exactly like an encoder failure. _make_robot/rogo push this on every
        #     connect; this custom worker must too.
        try:
            from robot_radio.io.cli import _push_calibration
            _push_calibration(conn)
            status_queue.put("calibration pushed")
        except Exception as exc:
            status_queue.put(f"WARN: calibration push failed: {exc}")

        # 1c. Zero the encoders — REQUIRED before driving on a fresh connection.
        #     A freshly-booted Nezha (every serial reconnect pulses DTR and
        #     resets the micro:bit) has its encoder READBACK frozen at 0 until a
        #     ZERO enc (vendor 0x1D zero-position) re-initialises it. Without
        #     this, "S <l> <r>" is ACKed but the encoders read a constant 0, the
        #     firmware's wedge detector fires (EVT enc_wedged ... n=10), and the
        #     wheels never spin up. enc_watch / rogo do this; the chart must too.
        try:
            proto.zero_encoders()
        except Exception as exc:
            status_queue.put(f"WARN: zero encoders failed: {exc}")

        # 1d. Apply any live SET overrides (e.g. sync=1, vel.kP=0.2) so gains /
        #     coupling can be grab-tested without reflashing. These persist on the
        #     firmware until reboot.
        for kv in (sets or []):
            try:
                r = proto.send(f"SET {kv}", 250)
                status_queue.put(f"SET {kv} -> {r.get('responses', ['?'])[-1]}")
            except Exception as exc:
                status_queue.put(f"WARN: SET {kv} failed: {exc}")

        # 2. Generous firmware S-watchdog (10 s). The host sends S every 150 ms,
        #    well within the window even during multi-second GUI/GIL stalls.
        try:
            proto.send("SET sTimeout=10000", 300)
        except Exception:
            pass

        # 3. Enable TLM streaming at 10 Hz (low serial load).
        proto.stream(100)

        status_queue.put(f"RUNNING — {identity.get('name', '?')}")

        # 4. Send S immediately, then every KEEPALIVE_S.
        conn.send_fast(f"S {speed} {speed}")
        last_send = time.monotonic()

        # Robust streaming loop — re-arms on EVT safety_stop.
        while not stop_event.is_set():
            for raw_line in conn.read_lines(duration_ms=50):
                r = parse_response(raw_line)
                if r is None:
                    continue

                # 5. Re-arm on safety_stop — do NOT exit; restart the wheels.
                if r.tag == "EVT" and r.tokens and r.tokens[0] == "safety_stop":
                    status_queue.put("REARM — safety_stop received, re-sending S")
                    conn.send_fast(f"S {speed} {speed}")
                    last_send = time.monotonic()
                    continue

                # 6. Collect velocity telemetry.
                if r.tag == "TLM":
                    tlm = parse_tlm(r.raw)
                    if tlm and tlm.vel is not None:
                        data_queue.put((time.monotonic(), tlm.vel[0], tlm.vel[1]))

            # 4 (cont). Time-driven keepalive — re-send S if 150 ms elapsed,
            # regardless of how many lines were read this iteration.
            now = time.monotonic()
            if now - last_send >= KEEPALIVE_S:
                conn.send_fast(f"S {speed} {speed}")
                last_send = now

    except OSError as exc:
        if exc.errno == 6:   # Device not configured — robot power-cycled
            status_queue.put("DISCONNECTED — power cycle detected, press SPACE to reconnect")
        else:
            status_queue.put(f"ERROR: {exc}")
    except Exception as exc:
        status_queue.put(f"ERROR: {exc}")
    finally:
        status_queue.put("STOPPED")
        if conn is not None:
            try:
                # Best-effort stop before closing.
                for _ in range(3):
                    conn.send_fast("STOP")
                    time.sleep(0.05)
                from robot_radio.robot.protocol import NezhaProtocol
                proto = NezhaProtocol(conn)
                proto.stream(0)
            except Exception:
                pass
            try:
                conn.disconnect()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Main — matplotlib window + spacebar connection control
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

    # ---- headless diagnostic: run the worker, print samples, exit (no GUI) ----
    if args.headless is not None:
        data_q: "queue.Queue[tuple[float,int,int]]" = queue.Queue()
        status_q: "queue.Queue[str]" = queue.Queue()
        stop_ev = threading.Event()
        th = threading.Thread(
            target=_stream_worker,
            args=(port, args.speed, data_q, stop_ev, status_q, args.sets),
            daemon=True,
        )
        th.start()
        t_end = time.monotonic() + args.headless
        samples: list[tuple[float, int, int]] = []
        t0 = None
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
        print(f"\n  collected {len(samples)} velocity samples")
        if samples:
            t0 = samples[0][0]
            for ts, vl, vr in samples[::max(1, len(samples) // 30)]:
                print(f"    t={ts - t0:5.2f}s  vL={vl:>5}  vR={vr:>5} mm/s")
            vls = [s[1] for s in samples]
            vrs = [s[2] for s in samples]
            nonzero = sum(1 for s in samples if s[1] != 0 or s[2] != 0)
            print(f"  vL range [{min(vls)}, {max(vls)}]  vR range [{min(vrs)}, {max(vrs)}]")
            print(f"  nonzero samples: {nonzero}/{len(samples)}")
            print("  >>> WHEELS DROVE (firmware vel field works on this path)"
                  if nonzero else
                  "  >>> ALL ZERO — wheels not driving on this path (vel field stayed 0)")
        else:
            print("  no samples — robot never streamed vel (check connection/fields)")
        return 0

    print("  Press SPACE in the plot window to connect/disconnect.")

    import matplotlib
    import platform
    # TkAgg crashes on macOS when daemon threads + numpy interact during teardown.
    # MacOSX (AppKit) backend is stable on macOS; TkAgg is fine on Linux.
    if platform.system() == "Darwin":
        matplotlib.use("MacOSX")
    else:
        matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    import numpy as np

    plt.style.use("dark_background")

    window_s = args.window
    maxlen   = int(window_s * 50)
    cmd_speed = args.speed

    times_buf: "collections.deque[float]" = collections.deque(maxlen=maxlen)
    vL_buf:    "collections.deque[int]"   = collections.deque(maxlen=maxlen)
    vR_buf:    "collections.deque[int]"   = collections.deque(maxlen=maxlen)

    data_queue:   "queue.Queue[tuple[float,int,int]]" = queue.Queue()
    status_queue: "queue.Queue[str]"                  = queue.Queue()

    # Worker state — replaced on each SPACE-start.
    worker_state = {"thread": None, "stop": None}

    # ---- figure ----
    fig = plt.figure(figsize=(12, 7))
    title_text = fig.suptitle("Robot wheel velocity  [SPACE = connect]",
                               color="white", fontsize=12)
    gs  = gridspec.GridSpec(2, 2, figure=fig, width_ratios=[1, 1.2],
                            hspace=0.45, wspace=0.35)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[1, 0])
    ax3 = fig.add_subplot(gs[:, 1])

    for ax, label in ((ax1, "Left wheel velocity (mm/s)"),
                      (ax2, "Right wheel velocity (mm/s)")):
        ax.set_title(label, fontsize=10)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylabel("mm/s", fontsize=8)
        ax.set_xlim(0, window_s)
        ax.set_ylim(-350, 350)
        ax.grid(True, alpha=0.3)

    ax1.axhline(cmd_speed, color="yellow", linestyle="--", linewidth=1.0,
                alpha=0.7, label=f"cmd={cmd_speed}")
    ax2.axhline(cmd_speed, color="yellow", linestyle="--", linewidth=1.0,
                alpha=0.7, label=f"cmd={cmd_speed}")
    ax1.legend(fontsize=7, loc="upper right")
    ax2.legend(fontsize=7, loc="upper right")

    (line_vL,) = ax1.plot([], [], color="deepskyblue", linewidth=1.2)
    (line_vR,) = ax2.plot([], [], color="tomato",      linewidth=1.2)

    ax3.set_title("Phase plot: vR vs vL (mm/s)", fontsize=10)
    ax3.set_xlabel("vL (mm/s)", fontsize=8)
    ax3.set_ylabel("vR (mm/s)", fontsize=8)
    ax3.set_xlim(-350, 350)
    ax3.set_ylim(-350, 350)
    ax3.set_aspect("equal")
    ax3.grid(True, alpha=0.3)
    ax3.plot([-350, 350], [-350, 350], color="dodgerblue", linestyle="--",
             linewidth=1.0, alpha=0.8, label="vR=vL (reference)")
    ax3.legend(fontsize=7, loc="upper left")

    (phase_trace,) = ax3.plot([], [], color="grey",  linewidth=0.8, alpha=0.6)
    (phase_dot,)   = ax3.plot([], [], "o", color="red", markersize=8)

    # ---- spacebar handler ----
    def _on_key(event):
        if event.key != " ":
            return
        th = worker_state["thread"]
        if th is not None and th.is_alive():
            # Disconnect: stop the running worker.
            worker_state["stop"].set()
            title_text.set_text("Robot wheel velocity  [SPACE = connect]")
        else:
            # Connect: fresh stop event + fresh thread.
            stop_ev = threading.Event()
            worker_state["stop"]   = stop_ev
            worker_state["thread"] = threading.Thread(
                target=_stream_worker,
                args=(port, args.speed, data_queue, stop_ev, status_queue, args.sets),
                daemon=True,
            )
            worker_state["thread"].start()
            title_text.set_text("Robot wheel velocity  [connecting…]")
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("key_press_event", _on_key)

    # ---- animation update ----
    def _update(_frame):
        # Consume status messages from worker.
        while not status_queue.empty():
            msg = status_queue.get_nowait()
            if msg.startswith("RUNNING"):
                title_text.set_text(
                    f"Robot wheel velocity  ▶ {msg}  [SPACE = disconnect]")
            elif msg.startswith("DISCONNECTED"):
                title_text.set_text(
                    f"Robot wheel velocity  ⚡ {msg}")
            elif msg in ("STOPPED", "FAILED", "CONNECTING"):
                pass
            elif msg.startswith("ERROR"):
                title_text.set_text(
                    f"Robot wheel velocity  ⚠ {msg}  [SPACE = retry]")

        # Drain data queue into buffers.
        try:
            while True:
                t, vl, vr = data_queue.get_nowait()
                times_buf.append(t)
                vL_buf.append(vl)
                vR_buf.append(vr)
        except queue.Empty:
            pass

        if not times_buf:
            return line_vL, line_vR, phase_trace, phase_dot

        t_arr  = np.array(times_buf)
        vl_arr = np.array(vL_buf)
        vr_arr = np.array(vR_buf)
        now    = t_arr[-1]
        rel    = np.clip(t_arr - (now - window_s), 0, window_s)

        line_vL.set_data(rel, vl_arr)
        line_vR.set_data(rel, vr_arr)
        phase_trace.set_data(vl_arr, vr_arr)
        phase_dot.set_data([vl_arr[-1]], [vr_arr[-1]])

        return line_vL, line_vR, phase_trace, phase_dot

    plt.ion()
    plt.show(block=False)

    try:
        while plt.fignum_exists(fig.number):
            _update(None)
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
