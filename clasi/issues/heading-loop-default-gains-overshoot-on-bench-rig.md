---
status: pending
sprint: '106'
tickets:
- 106-006
---

# PlannerParams' default heading-loop gains overshoot on the bench rig's turn leg

## Problem

`PlannerParams`' field defaults (`host/robot_radio/planner/model.py`) —
`heading_kp=2.0`, `heading_omega_clamp=0.5` (rad/s) — were carried forward
from ticket 106-005 as "a starting point" (that ticket's own AC #10 docstring
explicitly anticipated ticket 006's bench session would need to measure the
actual achievable correction bandwidth before treating any gain here as
final).

During ticket 106-006's real bench session (`tests/bench/profiled_motion_verify.py`,
robot on the stand), running a profiled 60° turn with the DEFAULT gains
against the real firmware caused the heading-correction trim to saturate at
its full `+heading_omega_clamp` (0.5 rad/s) for several consecutive ticks —
adding substantial EXTRA rotation on top of the profile's own already-complete
open-loop trajectory — and the turn landed at ~79° against the 60° target
(+19°, roughly +32% overshoot). The likely mechanism: `commanded_heading`
(the profile's own advancing plan, integrated open-loop from each setpoint's
`omega`) keeps advancing every tick regardless of the real plant's actual
progress; if the real plant (this rig's high-inertia proxy load, per
`.clasi/knowledge/bench-test-rig-layout.md`) lags the idealized profile,
the corrector keeps adding speed trying to catch up to an ever-advancing
target it can never quite reach, then overshoots once it does catch up.

## What was found empirically

Reducing to `heading_kp=0.4`, `heading_omega_clamp=0.2` (this ticket's own
bench finding, exposed as `--heading-kp`/`--heading-omega-clamp` CLI
overrides on `profiled_motion_verify.py`, never as a change to
`PlannerParams`' own field defaults) worked much better: turn-landing errors
across 4 clean-gain bench runs were -4.09°, -1.18°, +2.10°, and +15.75°
(one outlier — see the ticket's own Completion Notes "run-to-run
variability" section). This is a real improvement but not a perfectly solved
gain — the +15.75° outlier shows the saturation/overshoot failure mode is
reduced, not fully eliminated, at these gains.

## Recommended follow-up

A dedicated gain-tuning session (or a small sweep script, mirroring
`tests/bench/velocity_step_response.py`'s own live-`config()`-sweep
convention from ticket 106-002) to find a heading_kp/heading_omega_clamp/
latency_tau combination that holds turn-landing error reliably inside a
tight tolerance (e.g. ±3°) across repeated runs, before `PlannerParams`'
own field defaults are updated. Candidate directions: lower
`heading_omega_clamp` further to bound the maximum "catch-up" contribution;
investigate whether `commanded_heading`'s own open-loop advance should be
paced against MEASURED progress rather than purely time-integrated, for a
high-inertia load specifically.

## Evidence

- `clasi/sprints/106-host-trajectory-planner-profiled-twists-straights-and-turns/tickets/006-bench-gate-sim-validated-then-real-profiled-straight-turn-captured-traces.md`'s
  own Completion Notes (finding 2, "run-to-run variability").
- Captured bench traces: `tests/bench/out/profiled_{straight,turn}_20260715T151949Z.*`
  (default gains, +19.12° overshoot) vs.
  `tests/bench/out/profiled_{straight,turn}_20260715T152339Z.*` (tuned gains,
  +15.75° outlier) vs. `..._152402Z.*`/`..._152424Z.*` (tuned gains, clean).
