---
status: pending
filed: 2026-07-25
filed_by: team-lead (stakeholder-directed, parallel motion effort)
related:
- motion-library-development-kickoff-parallel-effort.md
---

# Motion planner: Motion::Planner — discrete-exact profiling, estimation, host-only development

## Summary

Build out the standalone motion planner designed and stakeholder-reviewed
2026-07-25 — **design of record:
[docs/design/motion-planner-sketch.md](../../docs/design/motion-planner-sketch.md)**.
One subsystem, `Motion::Planner`, owning everything between "a Move
arrived" and "here are the wheel velocity targets for the next 50 ms":
move queue + lookahead, discrete-exact trapezoid profiling, encoder/OTOS
state estimation, and odometry. Sequestered: host-buildable with zero
firmware dependency except the `types/` directory (`RobotState`), tested
entirely on the host (C++ unit/scenario tiers + a ctypes shared library
driven from Python).

Correctness bar: **in a zero-error simulation the motion is exact** — a
distance Move travels exactly its threshold, a rotation turns exactly its
angle, chains leak zero boundary error — with **no tuned margin
constants** (replaces the land-at-zero 0.48/0.67/0.92 machinery).

## Status: v1 already implemented (this checkout, `motion-planner` branch)

Commit `4aea58c1` on the `motion-planner` branch, self-contained under
`src/motion/planner/` (new files only — no collision with sprints
124-126's territory):

- `profile.{h,cpp}` — discrete trapezoid policy: exact staircase braking
  accounting, exact terminal step (landing within one decel step of the
  boundary), `maxEntryVelocity` lookahead.
- `estimation.{h,cpp}` — EMA wheel-velocity filter (fresh-sample gated),
  ZOH predict-to-now, arc-exact odometry, v1 OTOS heading blend.
- `planner.{h,cpp}` — `Motion::Planner`: `move()`/`stop()`;
  `tick(const RobotState&)` computes (cannot touch the blackboard);
  `update(RobotState&) const` saves (computes nothing new); 5-deep
  queue; same-axis carry via **cumulative chain baselines** (next Move
  measures from `previousBaseline + threshold`, so sub-tick boundary
  residual is debited to the successor — zero chain leak); timeout
  backstop; replace/flush.
- `types/robot_state.h` — MIRROR of the sprint-124 RobotState sketch,
  to be deleted and swapped for `src/firm/types/robot_state.h` at the
  joint checkpoint.
- Tests: 3 ctest suites (profile sweep, estimation units, 11 end-to-end
  zero-error plant scenarios), all passing. Measured: 500 mm lands with
  0.011 µm error; 90° turn with 0.014 arcsec.
- `capi.cpp` + `py/planner_harness.py` — ctypes tier with a
  struct-layout guard.

## Remaining work (sprint scope candidates)

1. **Noise/lag scenario tier** — the self-correction property: injected
   encoder noise, stale-sample cadence (~80 ms refresh vs 50 ms loop),
   actuation delay ≠ 0; assert bounded (not exact) error.
2. **Settle-confirm completion option** (M1) — `requireSettle`:
   completion additionally waits for |remaining| ≤ ε and |v| ≤ ε within
   a bounded window; default on for hardware, coincides with
   profile-complete in sim.
3. **Bench measurement** — characterize the encoder refresh interval
   (histogram of raw 0x46 count-change intervals, per wheel, 2-3 speeds)
   before tuning the EMA weight. The ~80 ms figure is bench folklore,
   never formally characterized.
4. **Heading hold on Distance moves** (M3) — P loop on the uncommanded
   axis.
5. **RobotState joint checkpoint** — swap the types mirror for the real
   sprint-124 header; reconcile field lists.
6. **Duty-plane back end** (M4, joint checkpoint with the main repo) —
   velocity PID moves into the planner's output stage; per-wheel duty
   out, observer estimates in, per
   `docs/design/base-explicit-loop-sketch.md`.
7. **Wheels-Move stop conditions beyond Time**, if protocol demand
   materializes (v1 deliberately rejects them).
8. **Open design questions** (sketch §8): `Motion::Move` field shape
   1:1 vs simplified; `BodyTwist3` into `types/` vs dropping the array
   overloads.

## Constraints

- Parallel-effort ground rules per
  [motion-library-development-kickoff-parallel-effort.md](motion-library-development-kickoff-parallel-effort.md):
  this work runs on the `motion-planner` branch in the parallel
  checkout, out-of-process, while sprints 124-126 execute in the main
  environment. Only new files under `src/motion/planner/`; never edit
  `src/firm` or `src/motion/state_estimator.*` here.
- Boundary-header changes require sign-off from both efforts once 124
  lands.
- Gates: the goal-doc bars (`docs/design/goal-exact-tours.md`) govern —
  motion-test numbers must hold AND the full-sim gate must reproduce
  them at the M6 integration checkpoint.
