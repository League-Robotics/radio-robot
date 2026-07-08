---
id: 089
title: "Planner motion planning via vendored Ruckig \u2014 build integration and D/T\
  \ terminal-overshoot fix"
status: done
branch: sprint/089-planner-motion-planning-via-vendored-ruckig-build-integration-and-d-t-terminal-overshoot-fix
use-cases: []
issues:
- planner-motion-planning-via-vendored-ruckig.md
- rt-open-loop-overshoot-under-synchronous-update.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 089: Planner motion planning via vendored Ruckig — build integration and D/T terminal-overshoot fix

## Goals

Integrate the already-vendored Ruckig library (`libraries/ruckig/`, MIT
community, proven by `tests/sim/unit/test_ruckig_smoke.py`) into
`Subsystems::Planner` so the Planner produces a real, inspectable jerk-limited
motion plan instead of a per-tick velocity chase, and use it to eliminate the
confirmed hardware terminal-overshoot/reverse-spin on `D`/`T`.

## Problem

Hardware-confirmed 2026-07-07: `D 200 200 1000` overshoots to ~292 mm/s
(commanded 200) then reverses ~16 mm after `EVT done`; `T 200 200 1000`
reverses ~23 mm after `EVT done`. Root cause: `Motion::VelocityRamp` is a
per-tick `approach()` toward a velocity target; on a stop the Planner sets
target `(0,0)` and keeps emitting a velocity twist. The velocity servo
(`Hal::MotorVelocityPid`) then sees `err = 0 - measured < 0` on a
still-coasting wheel and commands a reverse duty to brake, overshooting zero.
086/087 reduced this (086-002 integrator reset, 086-003/087-009 stop-distance
anticipation) but never eliminated it — the fix belongs in the motion plan
itself, not another servo/anticipation patch.

## Solution

Ruckig's `Trajectory` (computed via `calculate()`, sampled via `at_time()`)
is a first-class, inspectable jerk-limited motion plan that can be told to
arrive at the goal AT REST (`target_velocity = 0`), which by construction
never crosses zero into reverse. The Planner replaces `Motion::VelocityRamp` +
`Planner::applyStopAnticipation()` with Ruckig-generated trajectories for the
goal kinds that produced the confirmed bug or share its exact mechanism
(`DISTANCE`/`TIMED`/`VELOCITY`/`STREAM`/`TURN`/`ROTATION` — i.e. the
`D`/`T`/`R`/bare-`S`/`TURN`/`RT` wire verbs). **[Revision, post-stakeholder-
review]** Scope was originally staged narrower (`D`/`T`/`R`/`S` only,
`TURN`/`RT`/`G` all deferred); the stakeholder reviewed and EXPANDED it to
include `TURN`/`RT`, explicitly accepting the added risk of re-verifying
turn accuracy. Only `GOTO_GOAL` (`G`) remains deferred this sprint — it
needs a structurally different online/per-tick solve pattern whose cost is
unmeasured (see architecture-update.md Decision 5/9 for the full revised
rationale, including how `TURN`/`RT` map onto the rotational Ruckig channel
and what happens to their 086/087 accuracy-calibration surface).

## Success Criteria

- Ruckig's 11 core solver sources build as part of the REAL firmware CMake
  build and the host-sim CMake build (not just the standalone smoke-test
  subprocess compile).
- The Planner holds an inspectable Ruckig `Trajectory` per active
  `DISTANCE`/`TIMED`/`VELOCITY`/`STREAM`/`TURN`/`ROTATION` goal.
- Sim tests assert the sampled/commanded velocity/rotation profile for
  `D`/`T`/`TURN`/`RT` never goes negative and arrives at rest at the target
  — not just that the sim plant's own position converges (the sim plant
  today masks the reverse).
- On the stand (`.claude/rules/hardware-bench-testing.md`): `D 200 200 1000`
  and `T 200 200 1000` complete with no reverse and no 292-vs-200 overshoot.
- **[Revision]** On the stand: `TURN`/`RT` complete with no reverse AND
  their 086/087 heading/rotation accuracy tolerance bars are not regressed
  (re-verified against the same numeric bars those sprints established, per
  architecture-update.md Decision 9 — a stronger bar than "unregressed
  pass/xfail status," since their code path genuinely changes this sprint).
- `GOTO_GOAL` (`G`) is not regressed (existing sim tests keep their current
  pass status; `VelocityRamp`/`pursueSteer()` are byte-for-byte unchanged).

## Scope

### In Scope

- Vendored-Ruckig build integration into the ARM firmware CMake build and the
  host-sim CMake build.
- A new `Motion::` wrapper class around `Ruckig<1>`/`Trajectory<1>` (linear
  and rotational channels) usable from `Subsystems::Planner`.
- Migrating `DISTANCE`/`TIMED`/`VELOCITY`/`STREAM`/`TURN`/`ROTATION` goal
  kinds (`D`/`T`/`R`/`S`/`TURN`/`RT` wire verbs) onto Ruckig-generated
  trajectories, replacing `Motion::VelocityRamp` + `Planner::
  applyStopAnticipation()` IN FULL (all three branches — `STOP_DISTANCE`,
  `STOP_HEADING`, `STOP_ROTATION`) for those goal kinds. **[Revision: was
  `D`/`T`/`R`/`S` only, `STOP_DISTANCE` branch only.]**
- `TURN`/`RT`'s rotational-channel target resolution (reading the
  already-resolved heading delta / relative angle the existing wire-layer
  handlers compute) and the disposition of their 086/087 calibration
  surface (`rotational_slip`, `rotation_gain_pos/neg`, `rotation_offset(_neg)`
  — see architecture-update.md Decision 9 for which stay untouched and why).
- Wiring `msg::PlannerConfig`'s existing `j_max`/`yaw_jerk_max` fields (already
  present, currently unused — `j_max == 0` today means "trapezoid, no
  S-curve") as Ruckig's per-channel `max_jerk`, preserving that sentinel.
- Sim tests proving the no-reverse/rest-terminating property of the sampled
  plan for `D`/`T`/`TURN`/`RT`.
- Bench verification on the stand for `D`/`T`/`TURN`/`RT` per
  `.claude/rules/hardware-bench-testing.md`, including re-verifying `TURN`/
  `RT`'s existing 086/087 accuracy tolerance bars (not just no-reverse).

### Out of Scope

- **[Revision]** Migrating `GOTO_GOAL` (`G` wire verb) onto Ruckig. It keeps
  using `Motion::VelocityRamp` + `Planner::pursueSteer()`/`enterPursue()`'s
  `PRE_ROTATE`/`PURSUE` state machine unchanged — deferred because it needs
  a structurally different online/per-tick solve pattern (Decision 2) whose
  per-tick solve cost is unmeasured, not for scope-discipline reasons (unlike
  `TURN`/`RT`, which the stakeholder pulled INTO this sprint's scope — see
  architecture-update.md Decision 5's revision). A natural follow-on sprint.
- Retiring `Motion::VelocityRamp`/`applyStopAnticipation()` outright —
  `VelocityRamp` stays load-bearing for `GOTO_GOAL` this sprint (see
  architecture-update Migration Concerns for the resulting, now-smaller,
  temporary dual-mechanism state); `applyStopAnticipation()` itself IS fully
  removed this sprint (**[Revision]** — every goal kind that called it is
  now migrated).
- Extending the Planner→Drivetrain wire edge (`msg::BodyTwist3`/
  `DrivetrainCommand`) with an acceleration field ("command by acceleration"
  read literally) — **[Revision] CONFIRMED deferred by the stakeholder** —
  see architecture-update.md's Decision 3; add only if bench tracking shows
  velocity-only is insufficient.
- Any change to `docs/protocol-v2.md`'s wire grammar — no verb/argument
  changes.

## Test Strategy

Sim-level (`tests/sim/unit/`): unit tests on the new `Motion::` Ruckig wrapper
(offline solve-to-rest, sample past duration holds at final state, jerk=0
sentinel maps to unlimited jerk) plus Planner-level tests asserting the
sampled/commanded velocity/rotation trace for `D`/`T`/`TURN`/`RT` scenarios
never goes negative and converges to the commanded distance/duration/angle
at rest — checking the PLAN, not only the sim plant's position (the sim
plant currently masks the reverse symptom, and cannot fully validate real
turn accuracy either). Existing `test_motion_commands*.py`/
`test_motion_overshoot_regression.py` suites must stay green. **[Revision]**
RT's existing `xfail`s in `test_motion_commands_arc_turn.py`/
`test_tour_geometry.py` are allowed, but not required, to flip this sprint —
un-`xfail`ing them is an accepted possible side effect of the Ruckig
migration (Decision 9), not a requirement; if they stay `xfail`, that alone
is not a sprint failure, but a NEW `xfail` or a regressed accuracy bar is.
Bench-level: `.claude/rules/hardware-bench-testing.md`'s standing gate,
extended with `D`/`T` velocity-profile observation (encoder-derived) proving
no reverse and no overshoot past the commanded speed, AND `TURN`/`RT`
heading/rotation-accuracy re-verification against 086/087's existing
tolerance bars (not just no-reverse) — the sim cannot carry this part of the
gate, only the stand can.

## Architecture Notes

See architecture-update.md for the full design, including the DoF-mapping,
offline-vs-online-per-verb, Planner→Drivetrain edge, config, build-integration,
and goal-kind-migration decisions. **[Revision, post-stakeholder-review]**
Decision 3 (accel edge) is CONFIRMED velocity-only, unchanged. Decision 5
(scope) is EXPANDED to include `TURN`/`RT`; only `GOTO_GOAL` remains
deferred — see the new Decision 9 for how `TURN`/`RT` map onto the
rotational Ruckig channel and what happens to their 086/087 calibration
surface. The architecture self-review verdict is APPROVE.

## GitHub Issues

(none tracked yet)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan (approved with two decisions —
      accel edge CONFIRMED velocity-only; scope EXPANDED to include TURN/RT)

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Vendored-Ruckig build integration (ARM firmware + host-sim CMake) and footprint measurement | — |
| 002 | `Motion::JerkTrajectory` wrapper class and unit tests | 001 |
| 003 | Planner: migrate `DISTANCE` (D) onto `JerkTrajectory` | 002 |
| 004 | Planner: migrate `TIMED`/`VELOCITY`/`STREAM` (T/R/S) onto `JerkTrajectory` | 003 |
| 005 | Planner: migrate `TURN`/`ROTATION` (TURN/RT) onto the rotational `JerkTrajectory` channel | 003 |
| 006 | Sim tests: no-reverse trajectory-sampling proof for D/T/TURN/RT and G-unregressed check | 004, 005 |
| 007 | Bench verification on the stand: D/T/TURN/RT motion accuracy and G spot-check | 006 |

Tickets execute serially in the order listed. 005's dependency is 003 only
(not 004) in the dependency graph, but serial numbered execution means 004
lands first in practice — 005's own final cleanup step (deleting
`applyStopAnticipation()` in full, collapsing `Planner::tick()`'s dispatch)
relies on that ordering, documented explicitly in ticket 005 itself.
