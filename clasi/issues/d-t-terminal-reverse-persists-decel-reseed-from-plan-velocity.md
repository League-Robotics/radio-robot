---
status: pending
---

# D/T terminal reverse-motion persists on hardware — decel re-solve seeded from plan-believed velocity, not measured

## Description

Sprint 089 migrated `D`/`T` onto `Motion::JerkTrajectory` (vendored Ruckig)
specifically to eliminate the confirmed terminal reverse-spin/overshoot bug.
The ticket 007 bench pass (2026-07-07, robot Tovez on the stand, fw
0.20260707.18) shows the reverse motion is **reduced but not eliminated**:

| Verb | Post-`EVT done` reverse | Original bug | Completion reason |
|---|---|---|---|
| `D 200 200 1000` | **11–21 mm** (3 runs, 2 independent measurement paths) | ~16 mm | `dist` (own stop cond — good) |
| `T 200 200 1000` | **19–23 mm** (2 runs) | ~23 mm | `time` (own stop cond — good) |

Cross-checked via both fused TLM `enc=` and the raw per-motor `DEV M n STATE`
register to rule out a telemetry-fusion artifact — both agree a real,
reproducible position regression occurs after `EVT done`. The completion-mode
criterion **passes** (Decision 10's replan machinery works; nothing falls
through to the `STOP_TIME` net), so this is narrowly the terminal-reseed
behavior, not a completion failure.

## Root cause (read from source, not guessed)

Decision 8's seeding contract — "never seed a re-solve from measured state"
(correct in principle; it is exactly what avoids the 087-009 limit-cycle
bug) — assumes the channel's own theoretical belief tracks the real plant
closely. On this hardware that assumption breaks:

1. `JerkTrajectory::sample()` always overwrites `lastVelocity_`/`lastPosition_`
   with the **plan's** theoretical value at the sampled elapsed time, never
   the measured speed (`source/motion/jerk_trajectory.cpp:170-179`).
2. The bench-tuned velocity PID (`source/config/boot_config.cpp`, unchanged by
   this sprint — `Hal::MotorVelocityPid` is out of Decision 3's scope) tracks
   loosely: measured wheel speed runs ~250–310 mm/s during cruise on a
   commanded 200.
3. So the real encoder crosses `STOP_DISTANCE` **before** the plan's own
   position state would begin decelerating. `Motion::remainingToStop()`'s
   divergence replan correctly does NOT fire (its no-reverse-target guard
   skips when the plant has already reached/passed target), so
   `armDistanceStopDecel()` fires and seeds `solveToVelocity(0, ...)` from
   `lastVelocity_` — which `sample()` just set to the plan's lower ~200 belief
   **this same tick**, not the real ~250–310 measured speed.
4. The decel plan then commands down from ~200 while the real wheel is still
   faster, so the PID sees persistent negative error and brakes, producing the
   11–21 mm reverse creep. Smaller than the original bug (which stepped to
   zero) — a genuine partial improvement — but not "no reverse encoder motion."

This is a real interaction between Decision 8's seeding policy and the plant's
pre-existing PID looseness; the sim's idealized zero-tracking-error plant
cannot reproduce it (exactly the Grounding section's warning, and why the
bench gate exists).

## Options (for the follow-on sprint to decide — not pre-judged here)

- A **bounded** correction toward measured speed specifically at the
  stop-decel handoff, without reopening the 087-009 limit-cycle risk Decision 8
  cites.
- Retune / tighten the velocity PID so plan-belief tracks the plant (touches
  the sprint-077-tuned defaults — its own scoped change).
- An explicit stakeholder-accepted terminal-tolerance decision (accept
  ~10–25 mm terminal reverse creep as within bar).

## Evidence

- Sprint 089 `bench-verification-log.md` §1–2 (commit `2f809195`) — full traces.
- Captured raw trace: `tests/bench/out/bench_089_007_smoke.json`.

## Scope note

Blocks sprint 089's own acceptance (ticket 007). The peak-speed overshoot
(~300 vs commanded 200) is the same untouched PID's tracking characteristic,
**not** a Ruckig/plan-shape regression — the plan's own velocity ceiling was
confirmed correctly capped at the commanded speed.
