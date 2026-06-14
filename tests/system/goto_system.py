#!/usr/bin/env python3
"""goto_system.py — single-G-command telemetry demonstrator.

Drives ONE firmware G (go-to) command against sim, bench, or production,
collects telemetry per-tick via the Nezha.go_to(on_tick=...) callback API,
prints a one-line sample each tick, and plots the resulting path as a
matplotlib Agg PNG.

Targets
-------
  sim         — in-process firmware sim (no hardware).  DBG OTOS BENCH enables
                bench-OTOS so the pose advances with the commanded motion.
  bench       — real robot on a stand with bench-OTOS enabled (DBG OTOS BENCH).
  production  — real robot on the playfield with real OTOS + optional camera
                ground-truth pose (supply --camera <name>).
  playfield   — alias for production.

Usage
-----
  uv run python tests/system/goto_system.py --target sim --full-speed
  uv run python tests/system/goto_system.py --target bench --port /dev/cu.usbmodem...
  uv run python tests/system/goto_system.py \\
      --target production --port /dev/cu.usbmodem... --camera arducam-ov9782 \\
      --fwd 300 --left 0 --speed 160

CLI flags
---------
  --target {sim,bench,production,playfield}   (default: sim)
  --fwd MM          Forward distance mm (default: 300)
  --left MM         Lateral distance mm (default: 0)
  --speed MMPS      Cruise speed mm/s (default: 160)
  --timeout SECS    Max seconds to wait for G completion (default: 15)
  --port PORT       Serial port for bench/production
  --camera CAM      Camera name for production camera-pose runs
  --real-time       (sim) pace sim to wall-clock speed
  --full-speed      (sim) run as fast as possible (default)
  --image PATH      Override output PNG path
  --no-plot         Skip PNG rendering entirely

Notes
-----
- matplotlib and aprilcam are imported LAZILY inside plot() so that
  ``python goto_system.py --target sim`` works headless without a display
  or camera daemon.
- bench and production targets require a connected robot and are deferred
  to the team-lead for hardware verification.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Single-G telemetry demonstrator (sim / bench / production)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--target",
        choices=["sim", "bench", "production", "playfield"],
        default="sim",
        help="Target: sim | bench | production | playfield (default: sim)",
    )
    p.add_argument("--fwd", type=int, default=300, metavar="MM",
                   help="Forward distance in mm (default: 300)")
    p.add_argument("--left", type=int, default=0, metavar="MM",
                   help="Lateral distance in mm (default: 0)")
    p.add_argument("--speed", type=int, default=160, metavar="MMPS",
                   help="Cruise speed in mm/s (default: 160)")
    p.add_argument("--timeout", type=float, default=15.0, metavar="SECS",
                   help="Max seconds to wait for G completion (default: 15)")
    p.add_argument("--port", default=None,
                   help="Serial port for bench/production (auto-detect if omitted)")
    p.add_argument("--camera", default=None,
                   help="Camera name for production camera-pose runs")
    p.add_argument("--image", default=None, metavar="PATH",
                   help="Override output PNG path (default: tests/system/out/goto_<target>.png)")

    rt_group = p.add_mutually_exclusive_group()
    rt_group.add_argument(
        "--real-time", dest="real_time", action="store_true",
        help="(sim) pace sim to wall-clock speed",
    )
    rt_group.add_argument(
        "--full-speed", dest="real_time", action="store_false",
        help="(sim) run as fast as possible (default)",
    )
    p.set_defaults(real_time=False)

    p.add_argument("--no-plot", action="store_true",
                   help="Skip PNG rendering")

    return p.parse_args(argv)


# ---------------------------------------------------------------------------
# Telemetry record
# ---------------------------------------------------------------------------

class TelemetrySample:
    """One per-tick snapshot recorded in on_tick."""

    __slots__ = (
        "t", "x_mm", "y_mm", "heading_rad",
        "enc_l", "enc_r",
        "v_mmps", "omega_mradps",
        "cam_x_cm", "cam_y_cm", "cam_yaw_rad",
    )

    def __init__(
        self,
        t: float,
        x_mm: float, y_mm: float, heading_rad: float,
        enc_l: int, enc_r: int,
        v_mmps: float, omega_mradps: float,
        cam_x_cm: float | None = None,
        cam_y_cm: float | None = None,
        cam_yaw_rad: float | None = None,
    ) -> None:
        self.t = t
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.heading_rad = heading_rad
        self.enc_l = enc_l
        self.enc_r = enc_r
        self.v_mmps = v_mmps
        self.omega_mradps = omega_mradps
        self.cam_x_cm = cam_x_cm
        self.cam_y_cm = cam_y_cm
        self.cam_yaw_rad = cam_yaw_rad


# ---------------------------------------------------------------------------
# Plot (lazy matplotlib — headless Agg, no display required)
# ---------------------------------------------------------------------------

def _render_png(
    path: str,
    samples: list[TelemetrySample],
    target: str,
    fwd_mm: int,
    left_mm: int,
    outcome: str,
) -> None:
    """Render the recorded path(s) to a matplotlib Agg PNG.

    Shows:
    - OTOS/firmware pose path (cyan) — always present.
    - Camera ground-truth path (green) — production only, when available.

    Modelled on tests/bench/old/world_goto_chart.py::render_png:
    field outline, equal aspect, labeled lines, dark background.
    """
    import matplotlib  # noqa: PLC0415
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415

    import os  # noqa: PLC0415
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_facecolor("#101418")
    fig.patch.set_facecolor("#101418")

    # Gather firmware-OTOS path (mm → cm for display).
    xs_otos = [s.x_mm / 10.0 for s in samples]
    ys_otos = [s.y_mm / 10.0 for s in samples]

    # Gather camera path (already in cm).
    cam_samples = [s for s in samples if s.cam_x_cm is not None]
    xs_cam = [s.cam_x_cm for s in cam_samples]
    ys_cam = [s.cam_y_cm for s in cam_samples]

    # Target point in cm (relative to start).
    tx_cm = fwd_mm / 10.0
    ty_cm = left_mm / 10.0

    # Plot OTOS path.
    if len(xs_otos) >= 2:
        ax.plot(xs_otos, ys_otos, "-o", color="#00becf", ms=2.5, lw=1.8,
                label="OTOS/firmware pose")
    elif len(xs_otos) == 1:
        ax.plot(xs_otos, ys_otos, "o", color="#00becf", ms=4,
                label="OTOS/firmware pose")

    # Plot camera path (production).
    if len(xs_cam) >= 2:
        ax.plot(xs_cam, ys_cam, "-o", color="#3cdc5a", ms=2.5, lw=1.8,
                label="camera ground-truth")
    elif len(xs_cam) == 1:
        ax.plot(xs_cam, ys_cam, "o", color="#3cdc5a", ms=4,
                label="camera ground-truth")

    # Start + target markers.
    ax.plot(0.0, 0.0, "o", color="#4080ff", ms=12, zorder=5, label="start")
    ax.plot(tx_cm, ty_cm, "*", color="#ffd60a", ms=18, zorder=5,
            label=f"target ({tx_cm:+.0f},{ty_cm:+.0f}) cm")

    # Planned straight line: start → target.
    ax.plot([0, tx_cm], [0, ty_cm], "--", color="#666", lw=1, alpha=0.6,
            label="planned straight")

    # Axes + labels.
    ax.axhline(0, color="#334", lw=0.7)
    ax.axvline(0, color="#334", lw=0.7)
    ax.set_aspect("equal")
    ax.set_xlabel("x (cm, OTOS forward)", color="#aaa")
    ax.set_ylabel("y (cm, OTOS left)", color="#aaa")
    ax.tick_params(colors="#aaa")
    title = f"goto_system — target={target}  G({fwd_mm},{left_mm}) mm — {outcome}"
    ax.set_title(title, color="white", fontsize=10)
    ax.legend(loc="upper right", fontsize=8, facecolor="#202428", labelcolor="white")

    # Auto-pad view.
    all_x = xs_otos + xs_cam + [0.0, tx_cm]
    all_y = ys_otos + ys_cam + [0.0, ty_cm]
    pad = max(5.0, (max(all_x) - min(all_x)) * 0.15, (max(all_y) - min(all_y)) * 0.15)
    ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
    ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

    fig.tight_layout()
    fig.savefig(path, dpi=120, facecolor="#101418")
    plt.close(fig)
    print(f"[goto_system] saved PNG: {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)

    # Normalise "playfield" → "production".
    target = "production" if args.target == "playfield" else args.target

    fwd_mm: int = args.fwd
    left_mm: int = args.left
    speed_mms: int = args.speed
    timeout_s: float = args.timeout

    # Default output path: tests/system/out/goto_<target>.png
    _repo = pathlib.Path(__file__).resolve().parent.parent.parent
    if args.image is not None:
        png_path = args.image
    else:
        out_dir = pathlib.Path(__file__).resolve().parent / "out"
        png_path = str(out_dir / f"goto_{target}.png")

    # ------------------------------------------------------------------ #
    # Open connection via make_target                                      #
    # ------------------------------------------------------------------ #
    from robot_radio.testkit import make_target  # noqa: PLC0415

    camera_arg = args.camera if target == "production" else None

    print(
        f"[goto_system] target={target}  G({fwd_mm},{left_mm}) mm  "
        f"speed={speed_mms} mm/s  timeout={timeout_s}s  "
        f"real_time={args.real_time}"
    )

    tr = make_target(
        target,
        real_time=args.real_time,
        port=args.port,
        camera=camera_arg,
    )
    robot = tr.robot

    # ------------------------------------------------------------------ #
    # Enable the full telemetry field set explicitly.                      #
    # (go_to(on_tick=...) will enable STREAM 80; this ensures all fields  #
    # are subscribed before the G is issued so "all telemetry" is         #
    # intentional and not left to the default field mask.)                 #
    # ------------------------------------------------------------------ #
    robot.send("STREAM fields=enc,pose,vel,twist,otos,line,color", 300)

    # ------------------------------------------------------------------ #
    # Build the on_tick callback.                                          #
    # ------------------------------------------------------------------ #
    samples: list[TelemetrySample] = []
    t0 = time.monotonic()

    def on_tick(r) -> bool:
        """Called by go_to once per TLM tick.

        Reads robot.state and records + prints a one-line telemetry sample.
        For production with camera, reads the camera ground-truth pose too.

        Return True  → continue driving.
        Return False → abort (sends X, outcome = "aborted").
                       (Shown here as a comment to illustrate the API.)
        """
        t = time.monotonic() - t0
        state = r.state
        p = state.pose
        enc = state.encoders or (0, 0)
        twist = state.twist  # (v_mmps, omega_mradps) or None
        v = twist[0] if twist is not None else 0
        omega = twist[1] if twist is not None else 0

        # Camera ground-truth (production with camera only).
        cam_x = cam_y = cam_yaw = None
        if target == "production" and tr.playfield is not None:
            try:
                cam_x, cam_y, cam_yaw = tr.pose.read()
            except Exception:
                pass  # transient tag dropout — keep driving

        sample = TelemetrySample(
            t=t,
            x_mm=p.x, y_mm=p.y, heading_rad=p.heading,
            enc_l=enc[0], enc_r=enc[1],
            v_mmps=float(v), omega_mradps=float(omega),
            cam_x_cm=cam_x, cam_y_cm=cam_y, cam_yaw_rad=cam_yaw,
        )
        samples.append(sample)

        # Print one-line telemetry summary per tick.
        cam_str = ""
        if cam_x is not None:
            cam_str = f"  cam=({cam_x:+.1f},{cam_y:+.1f})cm yaw={math.degrees(cam_yaw):.1f}°"
        print(
            f"  t={t:5.2f}s  pose=({p.x:+5.0f},{p.y:+5.0f})mm  "
            f"h={math.degrees(p.heading):+.1f}°  "
            f"enc=({enc[0]:+d},{enc[1]:+d})mm  "
            f"v={v:.0f}mm/s  ω={omega:.0f}mrad/s"
            + cam_str
        )

        # Return True → keep driving.
        # Return False → abort (sends X, outcome = "aborted").
        return True

    # ------------------------------------------------------------------ #
    # Issue the single G command with on_tick callback.                   #
    # ------------------------------------------------------------------ #
    print(f"[goto_system] issuing G {fwd_mm} {left_mm} {speed_mms} ...")
    enc_l, enc_r, outcome = robot.go_to(
        fwd_mm, left_mm, speed_mms,
        on_tick=on_tick,
        timeout_s=timeout_s,
    )

    # ------------------------------------------------------------------ #
    # Summary.                                                             #
    # ------------------------------------------------------------------ #
    final = robot.state.pose
    print(
        f"\n[goto_system] outcome={outcome}  samples={len(samples)}  "
        f"enc=({enc_l},{enc_r})mm"
    )
    print(
        f"[goto_system] final pose: x={final.x:+.1f}mm  y={final.y:+.1f}mm  "
        f"h={math.degrees(final.heading):+.1f}°"
    )

    # ------------------------------------------------------------------ #
    # Plot.                                                                #
    # ------------------------------------------------------------------ #
    if not args.no_plot:
        if samples:
            _render_png(png_path, samples, target, fwd_mm, left_mm, outcome)
        else:
            print("[goto_system] no telemetry samples recorded — skipping plot")

    # Cleanup.
    try:
        tr.conn.disconnect()
    except Exception:
        pass
    if tr.playfield is not None:
        try:
            tr.playfield.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
