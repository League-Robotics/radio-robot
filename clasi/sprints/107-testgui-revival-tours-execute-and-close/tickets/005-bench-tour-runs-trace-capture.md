---
id: "005"
title: "Bench tour runs + trace capture"
status: open
use-cases: [SUC-036]
depends-on: ["003"]
github-issue: ""
issue: ""
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench tour runs + trace capture

## Description

With tickets 001-003 landed, Tour 1 and Tour 2 can run end-to-end against
real hardware — but no one has proven it on the actual bench rig, and
there is no captured evidence (trace files) for the notebook (ticket 006)
to chart. This ticket runs both tours for real on the bench rig
(`.claude/rules/hardware-bench-testing.md`, wheels off the ground),
through ticket 002's tour driver, capturing the full per-leg
commanded-vs-measured telemetry trace — mirroring `profiled_motion_verify.
py`'s own `LegResult`/CSV+JSON-sidecar convention (106-006), promoted to
cover a whole multi-leg tour rather than one isolated leg — to
`tests/bench/out/tour_<name>_<timestamp>.{csv,json}`, plus the tour's own
closure numbers (final pose vs. pre-leg-1 baseline, position and heading
delta), with an explicit closure tolerance chosen from the captured runs
themselves (not assumed from the pre-098 tours' now-inapplicable 100mm
figure, measured against a completely different, open-loop control
scheme). Serves SUC-036. This IS the sprint's own bench-runnable proof
that the stakeholder's stated acceptance ("demonstrate that the tours...
actually execute... I want those tours to be closed") is real.

Known risk carried forward from ticket 001: the heading-loop retune fixed
the WORST overshoot case but left a documented `+15.75°` single-turn
outlier possibility. Tour 2 chains 7 turns — this ticket's own closure
tolerance must be set with that compounding risk in view, from actually
observed runs, not assumed away.

## Acceptance Criteria

- [ ] The standing bench verification gate
      (`.claude/rules/hardware-bench-testing.md`) is satisfied before any
      tour run: sensors alive, wheels drive both directions with encoders
      incrementing, round-trip confirmed over the real link (mirror
      `profiled_motion_verify.py`'s own `preflight()` function/pattern).
- [ ] Both Tour 1 and Tour 2 run to completion on the bench rig with no leg
      timing out, through the real (not simulated) live wire surface,
      via ticket 002's `planner.tour.run_tour()`.
- [ ] A captured trace (CSV + JSON sidecar) exists per tour run under
      `tests/bench/out/`, recording every leg's commanded-vs-measured
      velocity/heading over time (per-tick rows, mirroring
      `profiled_motion_verify.py`'s own row schema: `tick_index`,
      `elapsed_s`, `sent_v_x`, `sent_omega`, `enc_l/r`, `vel_l/r`,
      `pose_x/y/h_cdeg`, `fault_bits`, `event_bits`, plus a `leg_index`/
      `leg_kind` column identifying which tour leg each row belongs to).
- [ ] Tour closure (final pose vs. pre-leg-1 baseline) is measured,
      recorded in the JSON sidecar, and checked against an explicitly
      stated tolerance chosen from the captured runs' own numbers (with
      documented headroom, matching 106-006/086-004's own "measure then
      set tolerance" precedent) — a real pass/fail judgment, not "it
      looked right."
- [ ] At least 2-3 repeat runs per tour are captured (matching 106-006's
      own "repeat runs" practice for characterizing run-to-run variance,
      especially given the carried-forward `+15.75°` outlier risk) before
      the closure tolerance is finalized.
- [ ] A human reviews the captured traces for visible resonance ringing on
      accel/decel phases (matching 106-006's own AC #3 "human trace
      review" convention) and records that pass/fail judgment in this
      ticket's own Completion Notes.
- [ ] Findings (measured closure numbers, chosen tolerance and why, any
      outlier observed, human ringing judgment) are recorded in this
      ticket's own Completion Notes — this is the sprint's primary
      evidence artifact alongside the notebook.

## Implementation Plan

### Approach

New `tests/bench/tour_bench_run.py`, structured like
`profiled_motion_verify.py` (106-006) but driving a whole tour instead of
one leg:
1. Connect (`SerialConnection`/`NezhaProtocol`), run the standing
   preflight gate.
2. Build `PlannerParams()` (ticket 001's new defaults apply automatically
   — no CLI override needed, though the script should still expose
   overrides for iteration, mirroring `profiled_motion_verify.py`'s own
   CLI-flag convention) and a `HeadingCorrector` with `otos_untrusted=True`
   (this rig's OTOS is on a mechanically decoupled mount — same convention
   every other bench script in this tree uses).
3. For each tour (`planner.tour.TOUR_1`, `TOUR_2`): parse to legs, call
   `run_tour()` with a row-callback that accumulates the full per-tick
   trace (tagged with leg index/kind), record the returned closure delta.
4. Write CSV + JSON sidecar per tour run to `tests/bench/out/`
   (`tour_<name>_<timestamp>.{csv,json}`), including planner params, tour
   name, per-leg outcomes, and the closure numbers in the JSON sidecar.
5. Gate-check: every leg `COMPLETED`, zero NEW fault bits (baseline-
   relative — free now via ticket 001's fix), closure within the
   tolerance chosen from the FIRST session's own repeat runs (a two-pass
   process: run once loose/unchecked to gather numbers, then set the
   tolerance and re-run to confirm — document both passes in Completion
   Notes).
6. `STOP` always sent in a `finally` block (mirrors every other bench
   script's safety convention).

### Files to Create

- `tests/bench/tour_bench_run.py`

### Testing Plan

- This ticket's OWN verification is the bench session itself — there is no
  meaningful unit-test surface beyond what ticket 002 already covers
  (`test_planner_tour.py`). Run `uv run python tests/bench/
  tour_bench_run.py` against the real robot on the stand, repeated 2-3x
  per tour per the acceptance criteria.
- `uv run python -m pytest` (full suite) stays green — this ticket adds no
  new pytest-collected file.

### Documentation Updates

- This ticket's own Completion Notes are the primary documentation
  artifact (closure numbers, chosen tolerance and rationale, outlier
  observations, human ringing-review judgment) — ticket 006's notebook
  reads the trace FILES this ticket writes, so file paths/naming must be
  stable and referenced accurately in Completion Notes for ticket 006 to
  pick up.
