---
status: done
---

# Navigator segments must outlast the replace interval (forward motion is a sawtooth)

## Description

**Measured on the playfield 2026-08-06, first real closed-loop field run of
`Motion::Navigator` (sprint 135).** Turns are smooth; forward motion is
violently herky-jerky — the stakeholder's words: "the most jerky,
interrupted motion ever."

Full-rate telemetry capture of one GO_TO leg (chart:
`src/tests/bench/output/goto_trace.png`, capture script pattern in that
session's scratchpad `goto_trace.py`):

- **Pivot phase (t=0.2-2.4s): clean.** One smooth ramp up, flat hold,
  smooth ramp down. Wheels at opposite signs, as expected.
- **Forward phase (t=2.7-5.6s): sawtooth.** Wheel velocity oscillates
  between ~60 and ~150 mm/s continuously, never holding cruise.
- **`kFlagActive` collapses to 0 and back ~15 times in 2.2s** — one dip
  roughly every 150 ms (every ~3 control cycles).
- Path geometry is FINE — the XY trace is a smooth curve onto the target.
  This is purely a velocity-profile defect, not a steering/solver defect.

Why sim and the stand both missed it: the ctest asserts velocity
continuity against `IdealPlant` (135-003), and the stand gate (135-006)
runs with wheels off the ground where the OTOS is frozen — so the
Navigator never actually re-solved. This regime had never been exercised.

## Cause

Two mechanisms, both pointing the same direction. Confirm which dominates
before tuning.

**1. Every internal segment plans a decel to rest, and the replace
cadence is short enough that the profile is always in that decel.**
`activateNext()` resets `activeBoundary_ = 0` on every activation
(`planner.cpp:1049`), so a Move only carries a nonzero exit velocity when
a COMPATIBLE SUCCESSOR IS ALREADY QUEUED (`boundaryLambda()`,
`planner.cpp:1496-1564`). The Navigator drives with `replace=true`, which
flushes the queue (`planner.cpp:266-277`) — so there is never a queued
successor and every segment plans a full stop.

**2. The one-cycle empty Move slot on each replace may be commanding an
actual zero.** 135-005 documented that `kFlagActive` blips false for
exactly one telemetry cycle on every internal-segment replace, because it
tracks Planner Move-slot occupancy. But `RobotLoop::zeroUnownedMotion()`
(`robot_loop.cpp:325-330`) zeroes BOTH `cmdVelocity` fields when neither
`planner_.active()` nor `drive_.owns()` — so if that runs on the blip
cycle, the wheels are commanded to zero for one full 50 ms cycle on every
single replace. At a 150 ms replace cadence that is a zero command every
third cycle, which would produce exactly the observed sawtooth.
**Check this first — it may be the whole story, and it is a different fix
(suppress the zeroing across a Navigator-owned handoff) than the cadence
change below.**

## Proposed fix

Stakeholder's directive (2026-08-06), and it is the cheap one: **do not
change the land-at-rest plan and do not touch `boundaryLambda()`. Instead
make each committed segment long enough that the next replace always
lands during CRUISE, before the decel ever begins.**

Sizing, from `data/robots/tovez.json` as of this writing (cruise
`navigator.speed` = 150 mm/s, `planner_shaper.a_decel` = 250 mm/s²,
`planner.control_period` = 50 ms):

| quantity | value |
|---|---|
| decel time from cruise `v / a_decel` | **0.6 s** |
| decel distance `v² / (2·a_decel)` | **45 mm** |
| segment commit, ~2.5x decel time | **1.5 s ≈ 225 mm** |
| replace cadence, ~1x decel time | **0.6 s ≈ 90 mm** (12 cycles) |

**The invariant to enforce and to assert in a test:**

```
replace_interval  <  segment_duration - decel_time
```

Here 0.6 s < (1.5 - 0.6) = 0.9 s, so a replace always arrives with ≥0.3 s
of cruise still ahead of it. Derive all four numbers from config at
runtime — do NOT hardcode them; `speed` and `a_decel` are per-robot config
and this must re-derive if either changes (same discipline
`arc_solver.h`'s `maxWheelStep` already follows).

**Terminal behaviour:** once remaining distance ≤ one segment (~225 mm),
STOP replacing entirely and let the final Move play out to rest at the
target. That is the one place the decel-to-zero plan is supposed to
execute, and it is also what fixes the current terminal accuracy (the
robot presently arrives mid-sawtooth).

Replace the current material-change throttle (`kNavOmegaReplaceThreshold`
/ `kNavArcLengthReplaceThreshold` / `kNavRefreshFraction`, ported verbatim
from the host loop in 135-003) with this time-based cadence — those
thresholds were tuned for a 100 ms host loop that did not have this
problem, and a material-change trigger is the wrong control variable here
anyway: the issue is not "has the solution changed" but "is the current
segment about to start decelerating."

## Verification

- **Sim first:** a ctest against a plant that actually re-solves (the
  135-003 `IdealPlant` harness, extended so pose advances) asserting the
  invariant directly — across a multi-segment leg, no commanded wheel
  velocity sample drops below cruise·0.8 between the initial accel and
  the terminal decel.
- **The zeroing check:** assert `cmdVelocity` is never zero on a
  Navigator-owned replace cycle (mechanism 2 above).
- **Hardware, and this is the real gate:** re-run the same capture that
  produced this issue's chart —
  `goto_trace.py <x> <y>` — on tovez over the relay, and compare the
  charts side by side. Accept when the forward phase holds cruise with
  no visible sawtooth and `kFlagActive` stops picket-fencing.
- Re-run the 5-waypoint field tour; expect the ~140 mm median arrival
  error to drop substantially, since much of it is arriving mid-lurch.

## RESOLVED 2026-08-06 — it was mechanism 2, and it was one line

Diagnosed and fixed out-of-process the same session (commit `8454456e`,
firmware `v0.20260806.5`). **Mechanism 2 was the whole story; the segment/
cadence rework in "Proposed fix" above was NOT needed and was not done.**

`RobotLoop::zeroUnownedMotion()` guarded on `planner_.active() ||
drive_.owns()` and did not know about the Navigator. Because
`Planner::move(replace=true)` clears `active_.occupied` immediately while
`Navigator::tick()` (which owns the re-activating `planner_.tick()`) runs
at the END of `cycle()`, `planner_.active()` reads false through the whole
top half of every replace cycle. `zeroUnownedMotion()` runs at the TOP of
`cycle()`, two statements before `drive_.tick()` actuates — so every
replace commanded a full 50 ms of zero. Fix: add `|| navigator_.active()`.

Measured on tovez over the relay, same capture script both sides:

| | before | after |
|---|---|---|
| forward-phase velocity | sawtooth, 60↔150 mm/s | steady cruise plateau, ±10 mm/s ripple |
| single-leg arrival error | 95.3 mm | **11.8 mm** |
| 5-waypoint tour, median | 142.8 mm | **8.5 mm** |
| tour worst leg | 195.8 mm | **17.7 mm** |

`kFlagActive` still picket-fences during a goto — that is 135-005's
documented Move-slot-occupancy artifact, not motion — but it no longer
costs a commanded zero.

**Left undone deliberately:** the time-based segment/cadence policy this
issue originally proposed. Cruise now holds flat, so there is no measured
problem left for it to solve; re-open only if a future regime (higher
cruise, tighter curvature) reintroduces a dip. The stakeholder's sizing
rule and the derived numbers are preserved above for that case.

## Related

- `src/tests/bench/output/goto_trace.png` — the chart this issue is built
  on (pivot smooth / straight sawtooth / active picket-fencing).
- Sprint 135 (closed) — shipped `Motion::Navigator`; its ctest velocity-
  continuity assertion passes against `IdealPlant` and did not catch this.
- `clasi/issues/later/135-006-goto-playfield-ab-and-relay-leg-not-attempted.md`
  — the owed playfield A/B; this issue is the first real finding from it.
- Measured A/B for context: `goto_world.py` (camera-closed, host-driven,
  re-fixes every pass) lands **4.3 mm**; firmware GO_TO on OTOS with one
  seed per leg lands **~140 mm median**. Some of that gap is OTOS drift,
  but the sawtooth is a separate and fixable contributor.
