"""systest -- THE system test. One program, tour files, one JSONL dataset.

Usage:
    uv run python src/tests/system/systest.py run --tier sim \
        [--robot-config data/robots/tovez_nocal.json] [--speed N] \
        [--out out/] TOUR [TOUR...]

    uv run python src/tests/system/systest.py plot DATASET.jsonl [--out DIR]
    uv run python src/tests/system/systest.py compare DATASET.jsonl --goldens DIR
    uv run python src/tests/system/systest.py bless DATASET.jsonl \
        [--runs R.jsonl ...] --goldens DIR

`run` executes the tours in order, gating each on the previous one's
success, and writes one JSONL dataset per run:
    out/<tour>_<tier>_<timestamp>.jsonl
Exit status is nonzero on any failure. Tours are plain text scripts --
see tours/*.tour and tourfile.py for the grammar.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from recorder import Recorder  # noqa: E402
from runner import (HardwareBackend, SimBackend, TourRunner,  # noqa: E402
                    build_run_meta)
from tourfile import parse_tour_file  # noqa: E402

_DEFAULT_ROBOT = "data/robots/tovez_nocal.json"


def _cmd_run(args: argparse.Namespace) -> int:
    out_dir = Path(args.out)
    exit_code = 0
    for tour_path in args.tours:
        tour = parse_tour_file(tour_path)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        out_path = out_dir / f"{tour.name}_{args.tier}_{stamp}.jsonl"
        recorder = Recorder(out_path, transport=args.tier)
        recorder.run_meta(build_run_meta(tour, tier=args.tier,
                                         robot_config=args.robot_config,
                                         argv=sys.argv[1:]))
        if args.tier == "sim":
            backend = SimBackend(args.robot_config, recorder=recorder,
                                 speed_factor=args.speed)
        else:
            backend = HardwareBackend(args.port, recorder=recorder,
                                      camera=(args.tier == "playfield"))
        recorder.note("firmware", version=backend.firmware_version())
        if args.tier == "playfield" and hasattr(backend, "seed_from_camera"):
            seeded = backend.seed_from_camera()
            if seeded:
                print(f"  seeded from camera: ({seeded['x']:.0f},"
                      f"{seeded['y']:.0f}) mm")
            else:
                print("  WARNING: no camera fix; odometry frame is not the "
                      "field frame and any SPLINE will chase the wrong path")
        try:
            runner = TourRunner(backend, recorder)
            result = runner.run(tour)
        finally:
            backend.close()
            recorder.close()
        print(result.summary())
        print(f"dataset: {out_path}")
        if not result.ok:
            exit_code = 1
            print(f"gate: {tour.name} failed -- not running later tours")
            break
    return exit_code


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="systest", description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_run = sub.add_parser("run", help="run tour file(s)")
    p_run.add_argument("tours", nargs="+", help="tour file paths, in order")
    p_run.add_argument("--tier", choices=["sim", "bench", "playfield"],
                       default="sim")
    p_run.add_argument("--robot-config", default=_DEFAULT_ROBOT)
    p_run.add_argument("--speed", type=int, default=1,
                       help="sim speed factor (1..20); wall-clock dwells "
                            "and EXPECT timeouts do not scale")
    p_run.add_argument("--port", default="/tmp/tovez-tty",
                       help="serial port (bench/playfield tiers)")
    p_run.add_argument("--out", default="src/tests/system/out")
    p_run.set_defaults(fn=_cmd_run)

    for name in ("plot", "compare", "bless"):
        p = sub.add_parser(name, help=f"{name} signals vs goldens")
        p.add_argument("dataset")
        p.add_argument("--goldens", default="src/tests/system/goldens")
        p.add_argument("--runs", nargs="*", default=[],
                       help="bless: extra no-change datasets for "
                            "suggest_tolerance()")
        p.add_argument("--out", default="src/tests/system/out")
        p.set_defaults(fn=_cmd_golden, which=name)

    args = parser.parse_args(argv)
    return args.fn(args)


def _cmd_golden(args: argparse.Namespace) -> int:
    import goldens
    return goldens.dispatch(args)


if __name__ == "__main__":
    raise SystemExit(main())
