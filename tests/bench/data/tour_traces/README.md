# Curated tour bench traces (107-005)

This directory holds a **curated, committed subset** of the trace files
`tests/bench/tour_bench_run.py` produced during the 107-005 bench session
against the real robot (Tovez, UID
`9906360200052820a8fdb5e413abb276000000006e052820`) on its stand, wheels off
the ground, per `.claude/rules/hardware-bench-testing.md`.

The script's own primary output location is `tests/bench/out/` (gitignored,
per every other bench script's own "HITL run artifacts, not committed"
convention) -- this directory exists ONLY so ticket 006's notebook
(`tests/notebooks/tour_closure_and_ramps.ipynb`) can load representative
trace data from a **fresh checkout**, where `tests/bench/out/` will not
exist. The full raw set (21 run attempts across both tours, including every
fault/overshoot stop, not just the curated ones below) is documented by
filename/timestamp in ticket 005's own Completion Notes
(`clasi/sprints/107-testgui-revival-tours-execute-and-close/tickets/done/
005-bench-tour-runs-trace-capture.md`) but is NOT committed in full -- only
this representative subset is.

## Curated files

**TOUR_1** (13 legs; `D 200 200 345`, `RT 9000` x6 same-direction 90 deg
turns, alternating with straight legs):

| Timestamp | Outcome | Notes |
|---|---|---|
| `20260715T201348Z` | fault, stopped leg 1 | genuine `kFaultWedgeLatch` trip entering the first turn (reversal-adjacent wedge-latch family) |
| `20260715T201724Z` | overshoot, stopped leg 1 | narrow bounded-overshoot trip at the DEFAULT `overshoot_bound_angular=0.1rad` (pre-widening) |
| `20260715T202308Z` | COMPLETE | clean pass, position_delta=502.8mm heading_delta=-12.70deg |
| `20260715T202452Z` | COMPLETE | clean pass, position_delta=353.6mm heading_delta=73.44deg |
| `20260715T202538Z` | COMPLETE | clean pass, position_delta=32.0mm heading_delta=-176.95deg |

**TOUR_2** (15 legs; includes a `RT -21700` = -217 deg turn, more than half
a revolution, and mixed turn directions):

| Timestamp | Outcome | Notes |
|---|---|---|
| `20260715T202706Z` | fault, stopped leg 0 | genuine `kFaultWedgeLatch` trip in leg 1 (same reversal-adjacent family) |
| `20260715T202642Z` | overshoot, stopped leg 5 | narrow bounded-overshoot trip on the -217 deg turn at `overshoot_bound_angular=0.35rad` |
| `20260715T202802Z` | COMPLETE | clean pass, position_delta=114.6mm heading_delta=138.28deg |
| `20260715T202905Z` | COMPLETE | clean pass, position_delta=715.6mm heading_delta=77.15deg (largest observed closure error, both tours) |

See ticket 005's own Completion Notes for the full run-statistics table
(completion rates, mean/spread, the chosen closure tolerance and its
rationale, and the human resonance-ringing review) across all 21 captured
attempts.
