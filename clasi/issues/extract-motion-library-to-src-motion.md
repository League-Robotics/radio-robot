---
status: pending
filed: 2026-07-24
filed_by: team-lead (stakeholder restructuring directive)
related:
- firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
- replan-sprints-122-plus-to-close-goal-exact-tours.md
tickets: []
---

# Extract the motion library: move all motion-control code from src/firm to src/motion

## Stakeholder directive (2026-07-24)

Split the codebase into a hardened FIRMWARE BASE and a separate MOTION
LIBRARY so the two can be developed and tested independently. This repo
focuses on the firmware base from here on; the motion library will be
developed on its own branch/worktree (likely its own repo later) — planned
here, executed there. This issue is the mechanical extraction that makes the
split real. **It is a pure reorganization: zero behavior change, zero wire
change — the firmware image still links the motion library, so every gate
and test passes unchanged before/after.**

## What moves (src/firm → src/motion)

- `firm/motion/stop_condition.*`, `firm/motion/velocity_shaper.*` (already
  pure — they anchor the new library).
- `firm/kinematics/body_kinematics.*` (pure math).
- `firm/app/move_queue.*` (queue, completion semantics, shaping dispatch —
  the code all the exactness work lives in).
- `firm/app/state_estimator.*` (estimation-for-control).
- `firm/app/odometry.*` (pose integration is kinematics, not hardware).
- The TWIST half of `firm/app/drive.*`: `setTwist()`/`BodyKinematics`
  staging moves up; the base keeps a plain wheel-target sink
  (`setWheels()`/`stop()`/`tick()` writing motor velocity targets).
- Their DESIGN.md content, unit tests, and the relevant harnesses.

## What stays in src/firm (the base)

`devices/`, `com/`, `messages/`, `config/`, `app/robot_loop.*`,
`app/comms.*`, `app/telemetry.*`, `app/preamble.*`, the wheel-target sink,
and (per the companion issue) the bounded wheel-move executor and the
per-wheel observer once built.

## The interface (the actual design work in this issue)

`src/motion` must compile with NO includes from `src/firm` except
`messages/` (generated wire structs are shared vocabulary). Define one
narrow boundary header (owned by motion, implemented by the base):

- **In:** per-wheel state (position, velocity, sample time — later the
  observer's estimate), a wheel-command sink, `now`, and plain config
  values (trackwidth, shaper limits) injected at construction.
- **Out:** `handleMove(env) → ack`, per-cycle `tick(now)`, completion
  events, and the pose/twist telemetry contributions the frame carries
  today (staged via the boundary, not by reaching into `Telemetry`).

RobotLoop composes base + motion exactly as `main.cpp`/`SimHarness` do
today; only include paths and construction change.

## Build targets

1. `firmware` — links base + motion (unchanged behavior, unchanged wire).
2. `motion_tests` — **standalone**: plain cmake + the motion library + a
   model plant. Reuse `src/tests/sim/plant/wheel_plant.*` (already pure) as
   the test plant. No SimHarness, no ctypes, no Python — completion/shaping
   logic iterates at unit-test speed. This target existing and passing IS
   the point of the split.
3. `libfirmware_host` (sim) — unchanged, links both.

## Repo-extraction readiness (the stakeholder's separate-repo intent)

`src/motion/` gets its own CMakeLists and its own DESIGN.md, imports nothing
from `firm/` but `messages/`, and its tests run from the directory alone —
so lifting it into a separate repo (or developing it in a worktree) later is
`git subtree split` + one include-path change, not a redesign. Until then it
lives here and the firmware links it by path.

## Acceptance

- Full suite + closure gates green with numbers UNCHANGED from the
  pre-extraction baseline (record before/after — this is a refactor gate,
  not an accuracy gate).
- `motion_tests` builds and runs with no sim library and no Python, and
  carries at least the existing StopCondition/VelocityShaper tests plus one
  end-to-end model-plant scenario (enqueue two chained moves, verify
  completion sequence) proving the boundary is sufficient.
- `src/motion` include-graph is clean (`firm/` appears only as `messages/`);
  CI-greppable.
- CLAUDE.md / design docs updated to name the two layers and the boundary.

## Sequencing

This supersedes the current sprint 122 course: 122 closes with 001's
already-decided revert-to-margin-baseline (keep the tau_plant plumbing and
the falsified-analytic finding); terminal-settle completion, same-axis
carry, heading-hold, and the rest of the exactness work move INTO the
motion library's own plan and proceed after extraction, developed against
`motion_tests` first, sim gates second. The companion base-hardening issue
runs in this repo in parallel once the boundary exists.
