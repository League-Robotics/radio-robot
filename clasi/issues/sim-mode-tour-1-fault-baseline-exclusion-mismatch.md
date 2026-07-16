---
status: pending
---

# Sim-mode Tour 1 reliably trips `kFaultWedgeLatch` mid-tour; real hardware doesn't (108-007 discovery)

Found during sprint 108 ticket 007 (`SimTransport` rewired onto
`robot_radio.io.sim_loop.SimLoop`, un-gating the Sim tour buttons) while
writing the ticket's own headless "Tour 1 closure" regression test
(`tests/testgui/test_sim_transport_tour1.py`).

## Problem

Running `planner.tour.run_tour()` against `SimTransport.protocol` (a live
`SimLoop` driving the REAL compiled firmware, `TestSim::SimHarness`/
`SimPlant`) with `TOUR_1`'s real 13-leg geometry faults out (`RunOutcome
.FAULT`, `kFaultWedgeLatch`) on nearly every attempt — **8/8 observed** in
a quick repro loop, always at a turn leg or the distance leg immediately
following one (never the first straight leg). Default
`DEFAULT_INTER_LEG_SETTLE=1.0s` (the bench-proven value from 107-005 /
the `tour1-freeze-investigation-2026-07-15.md` fix) is already in effect.

The SAME tour, run on REAL hardware the same day (`tests/bench/data/
tour_traces/tour_tour_1_20260715T202538Z.json`, `tests/bench/tour_bench_run.py`),
**completed** (`stopped_at: null`) despite also observing fault bits at
some point (`fault_bits_ever: 3`) — `StreamingExecutor`'s baseline-exclusion
(107-001) correctly treated those bits as pre-existing/not-new on hardware.
In Sim, the same mechanism treats the bits as NEW almost every leg boundary.

## What's NOT the cause (ruled out)

A raw `SimLoop` straight -> stop -> turn sequence, driven directly with
continuous `twist()` calls every 150ms (no `run_tour()`/`StreamingExecutor`
in the loop, no 1.0s idle settle window), never faults over 20+ manual
reps. The fault is specific to `run_tour()`'s own leg-boundary flow: a
`stop()`, then a genuinely idle ~1.0s settle window, then the next leg's
`begin()` snapshotting a fault-bit "baseline" before its first `twist()`.

This is NOT a regression from ticket 007's transport rewire itself — the
rewire is a thin, verified-correct plumbing change (`SimTransport.protocol`
correctly exposes a live, connected `SimLoop` satisfying `TwistTransport`;
`twist()`/`stop()`/telemetry all flow correctly in isolation, per this
same test file's `test_sim_transport_connects_and_exposes_a_live_protocol`
and the raw-`SimLoop` repro above).

## Hypothesis

Points at a timing/synchronization difference between `SimLoop`'s own
tick-thread (fixed 50ms sim-cycle granularity, `_CYCLE_DURATION_S` in
`sim_loop.py`) and the REAL robot's much-faster native loop tick, during
the exact window `begin()` snapshots its fault-bit baseline vs. when a
flickering `kFaultWedgeLatch` bit (already documented, real, and NOT sim-
specific — see `wedge-latch-flickers-during-active-motion.md` and
`.clasi/knowledge/encoder-wedge-boundary-latch.md`) happens to be set or
clear. Sim's lower tick rate may make `begin()`'s baseline snapshot land
on a "wrong side" of a flicker far more often than real hardware's finer
polling does, or `SimPlant`'s idle-state encoder-read behavior may
differ subtly from the real motor's.

## Why this matters / recommended direction

This is a NEW, deterministically-reproducible-in-sim lead on the SAME
family of bug the two issues above already track on hardware (where it
was hard to reproduce reliably) — Sim reproduces it at a much higher rate
than real hardware,making it a much cheaper harness for the actual root-
cause investigation those issues call for. Recommended: a dedicated
investigation (probably folds into
`wedge-latch-flickers-during-active-motion.md`'s own "Recommended
direction") using `SimLoop`'s `read_hook()`/raw per-tick `fault_bits`
(this file's own manual repro script, not committed) to capture the exact
tick sequence at a leg boundary and compare it against `begin()`'s
baseline-snapshot timing in `executor.py`.

## Current disposition (108-007)

Ticket 108-007's own headless acceptance test
(`tests/testgui/test_sim_transport_tour1.py::
test_tour_1_runs_to_completion_with_finite_small_closure`) retries the
full tour up to 5 times, then is marked `xfail(strict=False)` referencing
THIS issue — it stays informative (an unexpected pass is reported, not
hidden) without permanently reddening `uv run python -m pytest` on a bug
this ticket's own scope (transport plumbing) cannot fix. A separate,
narrower test in the same file
(`test_tour_shaped_sequence_via_direct_twist_calls_never_faults`) drives
the SAME leg geometry directly via `SimLoop.twist()`/`.stop()` (bypassing
`run_tour()`/`StreamingExecutor`'s baseline-exclusion machinery
entirely) and passes reliably — proving the rewired seam itself
(`SimTransport` -> `SimLoop` -> real firmware -> plant -> telemetry ->
closure math) is correct; only the `run_tour()`-specific baseline-
snapshot timing interaction is blocked on this issue. See ticket 108-007's
own Completion Notes for the full writeup.
