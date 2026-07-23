---
status: done
sprint: '120'
tickets:
- 120-002
---

# On-chip fake/simulated OTOS test device for the bench (synthetic OTOS pose, heading from encoders)

## Context

**PREMISE CORRECTED 2026-07-23 (phase-B bench, v0.20260723.2 on the stand).**
Two earlier premises are empirically WRONG and are the reason this issue kept
getting mis-scoped:

- NOT "OTOS on a servo port, `otos.present()` false." Bench telemetry shows
  `flag_otos_present` AND `flag_otos_connected` True on **every** frame, and
  `frame.otos` populated — the real OTOS is live on the **I2C bus** and read
  each cycle. (This matches the standing correction that the OTOS is I2C-bus
  mounted, never on a servo.)
- NOT "nothing reports OTOS" (the later/ triage note): `frame.otos` is
  reported every frame. What's true is narrower — nothing CONSUMES OTOS for
  control yet (StateEstimator OTOS fusion weights are committed 0.0), so
  heading currently comes from encoders through the estimator.

There is still **no firmware-side fake OTOS**: `MICROBIT.hex` always reads the
real chip; `SimPlant` is host-only in `libfirmware_host` and can't run on the
micro:bit.

## Why a fake OTOS is still wanted (the real, stakeholder-stated reason)

On the **stand** the wheels spin free and the robot does not translate through
space, so the **real** OTOS reports a nearly-static pose while the encoders
count — `frame.otos` is useless for verifying that a bench *tour* went where it
should. A build-selectable fake OTOS that synthesizes pose from encoder
kinematics makes `frame.otos` track the commanded motion on the stand, so bench
tour-closure can be verified through the SAME OTOS-present path the robot uses
on the table. Stakeholder (2026-07-23): build the fake for bench use now, so
that "when we put it on the table, you're actually using the real one" — i.e.
a compile/runtime seam that swaps synthetic-on-stand for real-on-table.

## What we want

A firmware-side, **build-selectable fake OTOS "test version"** — a
`Devices::Otos`-compatible test device that synthesizes an OTOS pose on-chip
instead of reading the I2C chip, so the robot can **report a synthetic OTOS
pose** (`frame.otos`) and drive heading from it.

Per stakeholder: it is **used for heading, and the heading is derived from the
encoders** — i.e. the synthetic OTOS pose is generated from the wheel/encoder
kinematic model (the same forward kinematics `App::Odometry` and the host
`SimPlant` use). This gives the bench a self-consistent **OTOS-present** regime
(matching how the sim closure gate validates with OTOS heading) without a
physical OTOS.

## Scope / design notes (for planning)

- **Build seam is missing.** The ARM composition root hard-wires the real bus:
  `src/firm/main.cpp:91` constructs `MicroBitI2CBus` and `main.cpp:122-128`
  constructs `Devices::Otos` against it, with no way to substitute a fake.
  Needs either a compile-time build variant (e.g. a `FAKE_OTOS` / bench-test
  macro) or a constructor seam.
- **Pose source.** The fake OTOS derives its `frame.otos` pose from encoder
  odometry (`BodyKinematics::forward` over real encoder deltas), so the
  reported OTOS pose tracks the commanded motion on the stand. It feeds the
  same `frame.otos` slot + `otos_present/connected` flags the real device
  drives, so any consumer (today: bench verification; later: StateEstimator
  fusion if a weight is raised) sees an OTOS-present regime.
- **NOTE the deleted references.** `App::HeadingSource` was removed in the 115
  gut — do NOT design against it. The live seam is `Devices::Otos` (the leaf
  the composition root builds) and `App::Odometry`/`App::StateEstimator`
  (`src/firm/app/`); the host mirror is `SimPlant` (`src/sim/sim_plant.*`).
- **Keep production untouched.** The real `Devices::Otos` path stays unchanged;
  the fake is a build-selectable variant (macro or constructor seam) that
  swaps synthetic-on-stand for real-on-table.
- **Do not reuse** the stale worktree `.claude/worktrees/bench-otos/` — it is
  the pre-rebuild `source/` layout, not the live tree.

## Priority

Pulled back to the active pool 2026-07-23 by direct stakeholder instruction
(overnight bench session): build the fake OTOS now for bench tour bring-up.
Supersedes the earlier later/ triage note below.

## Triage note (SUPERSEDED 2026-07-23 by stakeholder directive)

~~Moved to later/: as written this targets `App::HeadingSource` and the
OTOS-driven heading path, both deleted in the 115 gut.~~ The HeadingSource
observation stands (design against `Devices::Otos`/estimator instead), but the
"revisit later" disposition is reversed: the stakeholder wants the bench fake
OTOS built now (see corrected Context/Why above). `frame.otos` IS reported
today; the fake makes it meaningful on a stand.
