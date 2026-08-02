#!/usr/bin/env python3
"""square_tour_sim.py -- RETIRED (130-007).

This script used to run a SQUARE TOUR (4 x 500 mm legs, 4 x 90 deg turns)
through the REAL Motion::Planner (libmotionplanner via ctypes) against a
simulated pair of mismatched motors, closing the loop through the
planner's own M4 duty stage (Motion::WheelPid/Planner::stageDuty()): its
``SimWheel`` plant was driven ENTIRELY off the per-cycle ``plannerDuty()``
read (a first-order duty->velocity model with measured stiction/breakaway,
schedule-faithful staggered sense/plan/act offsets), and its whole point
was to chart commanded duty per wheel alongside true wheel speed.

Sprint 130 ticket 007 deletes the M4 duty stage outright
(``Motion::WheelPid``, ``Planner::stageDuty()``, ``commandedDutyLeft/
Right()``, the ``plannerDuty()`` C API) -- App::Drive's own unified
wheel-speed controller (tickets 004/005) is the proven, shipped law now,
and the parked duty stage has no future (reversing sprint 128 Decision
2's PARK). That deletion removes the ONE value this script's entire
simulation was built around: there is no live duty read left to drive the
plant with, and App::Drive's own controller is not reachable from this
host-side ctypes harness at all (it lives in the firmware/sim C++ build,
not in ``libmotionplanner``) -- there is no "live value" this script can
repoint at without a substantially different harness, out of scope for
this ticket.

This is the "print a clear message, do not leave a bench script silently
reading zeros" resolution from
``bench-duty-readers-see-zero-after-stageduty-park.md`` (its own listed
option, generalized: since duty read WAS the whole script here, not one
mode among several, retiring the whole script is the honest analogue of
"drop the duty-read mode"). See ``src/tests/bench/hil_drive.py``'s
module docstring for the sibling ``--duty`` mode, dropped the same way.

git history has the full prior implementation (stiction plant, dual-axis
duty/speed plot, schedule-faithful staggered sampling) for any future
duty-sink cutover -- see ``src/motion/DESIGN.md``'s "Wheel control
generations" note for the current owner/status of that question.
"""

import sys


def main() -> None:
    print(
        "square_tour_sim.py is RETIRED (130-007): the M4 duty stage "
        "(Motion::WheelPid/Planner::stageDuty()) this script's entire "
        "simulation was built around is deleted outright -- App::Drive's "
        "own unified wheel-speed controller is the proven, shipped law now "
        "and is not reachable from this host-side ctypes harness. See this "
        "file's own module docstring for the full rationale and "
        "src/motion/DESIGN.md's \"Wheel control generations\" note. "
        "git history has the prior implementation.",
        file=sys.stderr,
    )
    sys.exit(1)


if __name__ == "__main__":
    main()
