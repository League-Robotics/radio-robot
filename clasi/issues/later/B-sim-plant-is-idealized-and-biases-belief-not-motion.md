---
status: pending
priority: medium
---

# The sim's plant is an idealized toy and its error knobs bias belief, not motion

## Description

2026-08-02 post-130 review, **F3 (CRITICAL for G2)** plus Part 5. This is the
central blocker for the stated goal of *a hardware-characterized sim the motion
planner can be developed against*.

## Two structural problems

**1. The plant is ideal.** `src/tests/sim/plant/wheel_plant.h:44-45` is one
shared compile-time gain (`kDutyVelMax = 500`), linear from duty 0, symmetric,
with no breakaway, no saturation knee, and no two-wheel coupling. Worse,
`src/sim/sim_harness.h:130` then installs the plant's **exact inverse** as the
controller's feedforward — so the sim controller lives in a zero-map-error world
by construction. The one thing the 130 controller exists to correct cannot occur
in sim.

**2. Every error knob biases belief, not motion.** `sim_plant.h:117-157`'s
encoder freeze/jitter/scale and OTOS drift all apply in `reported*()`. True
motion is untouched **by design**. That is exactly backwards for characterizing
a plant: the real robot's wheels genuinely do the wrong thing; they do not
merely misreport.

## What the real plant does that the sim cannot represent

| Measured behavior | In sim? |
|---|---|
| Per-wheel gain asymmetry, breakaway, saturation knee | **no** (planner-tier `NoisyPlant` has it; the full sim does not) |
| Session-to-session gain drift ±25% | **no** — no mechanism |
| Two-wheel load coupling / derating | **no** — wheels step independently (`sim_plant.cpp:249-250`) |
| Deadband + copysign boost | **no** — the compensation runs against a plant lacking the defect it corrects |
| Delivered 54 ms vs believed 50 | **no** — one constexpr serves both |
| Actuation lag ~120–140 ms | partial — first-order τ = 0.13 hardcoded, not per-wheel, not configurable |
| Encoder quantization, I2C wedge, OTOS drift | yes — the best-covered area |

## The fix path is short because the seams exist

Per the review's Part 5, in order:

1. **Parameterize `WheelPlant`** (per-wheel gain/τ, breakaway band, saturation
   knee) from a pushed config: one new `sim_configure_wheel_plant()` ctypes
   export plus a `sim_plant` section in the robot JSON via the existing
   `configure_from_robot()`. The numbers already exist in `tovez.json`'s
   130-001 note — finish the right wheel first
   ([[A-next-physical-bench-session-checklist]] item 3). Drop or make opt-in the
   exact-inverse feedforward.
2. **Decouple delivered from believed period**: a settable step dt (default
   `kCycle`) plus an optional deterministic jitter sequence, with
   `PlannerLimits.controlPeriod` staying the JSON value. Feed it the measured
   54.0 — see [[A-nominal-50ms-vs-delivered-54ms]].
3. **Add the true-motion error half**: per-wheel achieved-gain error,
   load-coupled derating f(|dutyL| + |dutyR|), turn scrub. The vocabulary
   already exists twice — `NoisyPlant`
   (`src/motion/planner/tests/test_support.h:95-165`) and the dead 069
   `PhysicsWorld`'s `bodyRotScrub`/`motorOffsetL/R`.
4. **Retire `fit_sim_error_model.py`** — it targets the deleted SIMSET registry
   and a `src/sim/firmware.py` that no longer exists; zero live callers.
   Re-implement "fit" as a `systest.py` subcommand over a hardware JSONL vs the
   same tour's sim JSONL (`signals.py` is already transport-symmetric), emitting
   a versioned error-model JSON the sim loads.
5. **Fix the Python config-path drift** — `SimLoop.configure_from_robot()`
   pushes the JSON's stale `duty_per_speed` (`sim_loop.py:715-719`) while
   hardware runs the baked constant, so a TestGUI sim session runs a *third*
   feedforward. Same disease 130-002 killed in C++. Folded into
   [[B-one-owner-per-constant-speed-floor-and-duty-per-speed]].
6. **One injection surface**: fold all error-model application into
   `configure_from_robot()` so pytest, systest, and TestGUI share it — the
   standing one-Sim-object rule — pruning the dead Sim-Errors knobs
   (warn-and-skip today) in the same pass.

Determinism is currently excellent but **seedless**, so dispersion studies are
impossible; the fit doctrine needs noise sigmas, so add a seed while in here.

## Verification

- The sim reproduces the measured plant: commanded 150 mm/s delivers 70–85% from
  a cold start, breakaway around the measured duty, saturation near the measured
  ceiling.
- A gain error introduced in the plant is visible as real position error, not
  merely as a misreported encoder.
- The sim can be stepped at 54 ms and a marginal loop (heading hold at gain 2.0)
  destabilizes there and not at 50 — closing the "passes in sim, breaks on
  hardware" gap that this exact bug walked through.
- One hardware square-tour dataset and its sim counterpart, compared by the fit
  subcommand, produce an error model the sim loads.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F3, Part 5,
  S1–S5.
- [[B-system-test-minimal-remaining-phases]] — the hardware tier is what
  produces the fit's input dataset. These two unlock each other.
- `docs/post-mortem/2026-08-02-sprint-130-wheels-solid.md` root cause 5.
