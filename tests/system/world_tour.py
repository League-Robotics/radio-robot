#!/usr/bin/env python3
"""world_tour.py — stepped playfield tour that draws each leg on AprilCamView.

Drives the robot through a sequence of N playfield features (the colored squares
by default), one leg at a time.  Each leg re-localizes off the overhead camera,
closed-loop drives to the target, and draws TWO overlays on the AprilCamView:

    RED   = onboard odometry  (the firmware's fused pose)
    GREEN = camera ground truth

Two ways to run it:

  * Interactive (default): press **SPACE** to erase the view and drive the next
    leg, then look at the traces it left.  Press SPACE again → erase + next leg.
    Press **q** (or Esc / Ctrl-C) to quit.  The robot only moves when you press
    SPACE, so you control the pace and can study each trace.

  * One-shot (``--auto``): runs the whole tour back-to-back with a short pause to
    view each trace.  Use this for testing.

The robot's AprilTag is id 100 (id 1 is the field-centre reference) and the field
is 134.3x89.3 cm (A1-centred).  The squares sit at +-35/+-24 and the cardinal dots
at +-50/+-30, so the default safe box (--field-x 58 --field-y 40) reaches them all.

    uv run python tests/system/world_tour.py                 # interactive, 6-square loop
    uv run python tests/system/world_tour.py -n 4            # interactive, first 4 legs
    uv run python tests/system/world_tour.py --auto          # one-shot (testing)
    uv run python tests/system/world_tour.py --targets "black square,red dot,green dot,blue square"
"""
from __future__ import annotations

import argparse
import sys
import time

# A clockwise loop of the six colored squares — draws a hexagon-ish pattern.
DEFAULT_ORDER = [
    "black square",      # N   ( 0, +24)
    "orange square",     # NE  (+35,+24)
    "green square",      # SE  (+35,-24)
    "magenta square",    # S   ( 0, -24)
    "blue square",       # SW  (-35,-24)
    "purple square",     # NW  (-35,+24)
]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("-n", "--n", type=int, default=None,
                   help="number of legs to drive (default: one full pass of the "
                        "target list; if larger, the list cycles)")
    p.add_argument("--targets", default=None,
                   help="comma-separated where-queries to visit in order "
                        "(default: the six colored squares as a loop). Each may be "
                        "a square ('blue square'), a dot ('red dot', "
                        "'northeast orange dot'), or a unique bare color.")
    p.add_argument("--random", action="store_true",
                   help="each leg drives to a random colored SQUARE on the opposite "
                        "n/s side (crosses the x-axis; 3 choices); ignores --targets")
    p.add_argument("--seed", type=int, default=None, help="(--random) RNG seed")
    p.add_argument("--auto", action="store_true",
                   help="run the whole tour automatically (no SPACE stepping)")
    p.add_argument("--pause", type=float, default=1.0,
                   help="(--auto) seconds to view each trace before the next leg "
                        "erases it (default 1.0)")
    p.add_argument("--speed", type=int, default=160,
                   help="G arc speed mm/s (default 160)")
    p.add_argument("--arrive", type=float, default=8.0,
                   help="arrival tolerance cm (default 8)")
    p.add_argument("--field-x", type=float, default=58.0,
                   help="playfield safe half-width x cm (default 58)")
    p.add_argument("--field-y", type=float, default=40.0,
                   help="playfield safe half-width y cm (default 40)")
    p.add_argument("--robot-tag", type=int, default=100,
                   help="AprilTag id on the robot (default 100)")
    p.add_argument("--camera", default=None,
                   help="camera name (default: first camera in the aprilcam daemon)")
    p.add_argument("--port", default=None, help="serial port (auto-detect if omitted)")
    return p.parse_args(argv)


def _read_key() -> str:
    """Block for a single keypress (no Enter). Returns the character."""
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def _flush_input() -> None:
    """Drop any buffered keystrokes (so key-repeat during a drive doesn't queue
    up extra legs — one deliberate press = one leg)."""
    try:
        import termios
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except Exception:
        pass


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    from robot_radio.testkit import make_target
    from robot_radio.field.playfield import Playfield
    from robot_radio.testkit.pose import CameraPose
    try:                                            # run directly or as a module
        from goto_world import (drive_to_target, resolve_where,
                                get_rectangles, pick_random_rect, _cleanup)
    except ImportError:
        from tests.system.goto_world import (drive_to_target, resolve_where,
                                             get_rectangles, pick_random_rect, _cleanup)

    rng = None
    legs: list[tuple[str, str, float, float]] | None = None
    if args.random:
        import random
        rng = random.Random(args.seed)
        n = args.n if (args.n and args.n > 0) else 8
    else:
        order = ([s.strip() for s in args.targets.split(",") if s.strip()]
                 if args.targets else list(DEFAULT_ORDER))
        if not order:
            print("[tour] no targets.")
            return 2
        n = args.n if (args.n and args.n > 0) else len(order)

    # --- Connect robot + camera once ---------------------------------------
    tr = make_target("production", port=args.port)
    robot = tr.robot
    fw = robot.connect().get("fw")
    print(f"[tour] connected: fw={fw}")
    pf = Playfield.open(args.camera)
    pose_src = CameraPose(pf, tag_id=args.robot_tag)

    if args.random:
        rects = get_rectangles(pf)
        if not rects:
            print("[tour] --random: no colored rectangles on the playfield.")
            _cleanup(tr, pf)
            return 2
        print(f"[tour] {n} RANDOM leg(s) — each a colored square on the opposite n/s "
              f"side ({len(rects)} squares; 3 choices per leg).")
    else:
        # Resolve the whole target list up front (fail fast on a bad name).
        legs = []
        for i in range(n):
            q = order[i % len(order)]
            slug, tx, ty = resolve_where(pf, q)
            legs.append((q, slug, tx, ty))
        print(f"[tour] {n} leg(s):")
        for i, (q, slug, tx, ty) in enumerate(legs, 1):
            print(f"   {i:2}. {q:20} -> {slug:24} ({tx:+.0f},{ty:+.0f}) cm")

    interactive = not args.auto
    if interactive:
        print("\n[tour] SPACE = erase view + drive next leg;  q = quit.")

    results: list[tuple[str, dict]] = []
    try:
        pf.clear_paths()                            # start with a clean view
        for i in range(1, n + 1):
            if interactive:
                _flush_input()
                preview = "random crossing" if args.random else \
                    f"{legs[i - 1][0]} ({legs[i - 1][1]})"
                print(f"\n[tour] leg {i}/{n} -> {preview}. "
                      f"SPACE to drive, q to quit ...", flush=True)
                k = _read_key()
                if k in ("q", "Q", "\x03", "\x1b"):
                    print("[tour] quit.")
                    break

            pf.clear_paths()                        # erase the previous trace
            if args.random:
                rx, ry, _ = pose_src.read()
                slug, tx, ty = pick_random_rect(rects, ry, rng)
                desc = f"random {slug} @ ({tx:+.0f},{ty:+.0f})"
            else:
                q, slug, tx, ty = legs[i - 1]
                desc = f"{q} ({slug}) @ ({tx:+.0f},{ty:+.0f})"
            print(f"[tour] === leg {i}/{n}: -> {desc} cm ===")
            res = drive_to_target(
                robot, pf, pose_src, tx, ty,
                speed=args.speed, arrive=args.arrive,
                field_x=args.field_x, field_y=args.field_y,
                draw_paths=True, label="tour", verbose=True,
            )
            results.append((desc, res))
            print(f"[tour] leg {i}: {'ARRIVED' if res['arrived'] else 'NOT-ARRIVED'} "
                  f"err={res['error']:.1f}cm — trace drawn (RED=onboard, GREEN=camera).")
            if res["aborted"]:
                print("[tour] BOUNDS-ABORT — stopping the tour for safety.")
                break
            if args.auto:
                time.sleep(args.pause)
    except KeyboardInterrupt:
        print("\n[tour] interrupted.")
    finally:
        _cleanup(tr, pf)

    if results:
        ok = sum(1 for _, r in results if r["arrived"])
        mean = sum(r["error"] for _, r in results) / len(results)
        print(f"\n[tour] done: {ok}/{len(results)} arrived  (mean error {mean:.1f}cm)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
