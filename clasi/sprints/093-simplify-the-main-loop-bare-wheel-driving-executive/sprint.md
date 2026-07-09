---
id: 093
title: "Simplify the main loop — bare wheel-driving executive"
status: ticketing
branch: sprint/093-simplify-the-main-loop-bare-wheel-driving-executive
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004]
issues:
- simplify-the-main-loop-strip-it-to-bare-wheel-driving.md
- get-wire-output-events-telemetry-out-of-the-main-loop.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 093: Simplify the main loop — bare wheel-driving executive

## Goals

Strip `Rt::MainLoop::tick()` down from a 280+-line orchestrator of seven
concerns (safety watchdogs, odometer, pose fusion, Planner goal-closure,
telemetry, two-plane output routing) to a four-line command→wheels
executive that ticks only `Hardware` and `Drivetrain`. Reduce the live wire
command surface from ~30 verbs across six families to exactly four:
`PING`, `HELLO`, `S`, `STOP`. Stakeholder framing (2026-07-08, verbatim):
*"a real simple main loop... It should read like a shopping list, not like
a Tolstoy novel."*

## Problem

The loop no longer reflects its own intended design. `S`, the simplest
drive verb, doesn't even drive the wheels directly — it converts wheel
speeds to a body twist, hands off to `Planner`, which ramps and converts
back to wheels. Every other verb family (`config`, `pose`, `otos`,
`telemetry`, `dev`) adds more state the loop must service every pass, none
of it needed for the bench bring-up work this sprint targets.

## Solution

Gut, don't refactor. `MainLoop` shrinks to two subsystem references
(`Hardware&`, `Drivetrain&`); `tick()` becomes: tick hardware, tick
drivetrain, commit `bb.motors[]`/`bb.drivetrain`, route drivetrain's own
output back to `bb.motorIn[]`. `S` posts a `msg::DrivetrainCommand{WHEELS}`
straight to `bb.driveIn` (no kinematics, no ramp, no stop-clauses); `STOP`
posts the canonical neutral. `Rt::CommandRouter`'s table shrinks to
`PING`/`HELLO`/`S`/`STOP`. Removed classes (`Planner`, `PoseEstimator`,
`EkfTiny`, `Rt::Configurator`, both watchdogs) and command-family files stay
on disk, un-wired — reversible, least-churn, per the greenfield-rebuild
precedent (park, don't delete). See
`clasi/sprints/093-simplify-the-main-loop-bare-wheel-driving-executive/
architecture-update.md` for the full design, diagrams, and design-rationale
decisions (notably: loop-originated `EVT`/`TLM` output is removed by
removing its producers rather than built as a queue — this also resolves
the companion `get-wire-output-events-telemetry-out-of-the-main-loop.md`
issue).

## Success Criteria

- `Rt::MainLoop::tick()` reads as the four-step sequence above — no
  watchdog, no pose, no planner, no telemetry code in the loop.
- Exactly four verbs are reachable on the wire: `PING`, `HELLO`, `S`, `STOP`.
- `S`/`STOP` drive/neutralize both wheels directly, verified both in sim
  and on the physical bench (stand-mounted, wheels off the ground).
- The CLASI close-gate (`uv run python -m pytest`) is 100% green against a
  deliberately narrowed, curated `tests/sim/` suite — not the pre-sprint
  suite's full scope.
- The safety-watchdog removal, the test-parking scheme, and the "no ramp"
  behavior change are each documented as explicit, examined decisions (not
  discovered as side effects) in `architecture-update.md`.

## Scope

### In Scope

- `source/runtime/main_loop.{h,cpp}` — the loop gut.
- `source/main.cpp` / `tests/_infra/sim/sim_api.cpp` — composition-root
  slimming, in lockstep (the 1:1 sim-mirror invariant).
- `source/runtime/command_router.cpp` — table reduction to four verbs.
- `source/commands/motion_commands.cpp` — `handleS`/`parseS`/`handleStop`
  rewrite.
- `tests/sim/` — parking obsoleted tests, a small focused suite for the new
  surface, `conftest.py`'s `sim` fixture fix (drop the dead `DEV WD` widen).
- A hardware bench-gate verification pass on the stand.

### Out of Scope

- Relocating motion planning (Ruckig / goal closure) into `Drivetrain` —
  that is a separate, already-drafted future sprint
  (`drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md`,
  `communicator-drivetrain-motion-command-segment.md`). This sprint leaves
  the seam clean but does not touch it.
- `docs/protocol-v2.md` currency (deferred — team-lead disposition).
- A boot-banner / `HELLO`-reply note about the missing watchdog (deferred —
  team-lead disposition).
- Any host-side change (`robot_radio`, TestGUI) to track the reduced verb
  surface (accepted fallout — team-lead disposition; bench verification
  uses a bare serial script instead).

## Test Strategy

Sim (`tests/sim/`) stays the CI/close-gate, but its scope narrows sharply:
most of the pre-sprint suite exercises verbs/subsystems this sprint
unwires. Obsoleted files are moved to `tests/sim/parked-093/` (excluded via
`pyproject.toml`'s `norecursedirs`, not deleted) rather than dragged along
red or discarded outright. A small, curated suite proves the four-verb
surface: `S` drives both wheels correctly (direction + magnitude), `STOP`
neutralizes, `PING`/`HELLO` reply correctly, and an out-of-surface verb
correctly gets `ERR unknown`. The sprint closes with a mandatory hardware
bench-gate pass on the stand (`.claude/rules/hardware-bench-testing.md`),
using a bare serial script rather than TestGUI/`robot_radio` (both of which
still expect the pre-sprint verb surface).

## Architecture Notes

See `architecture-update.md` for the full 7-step design. Headline decisions:
1. Loop-originated wire output (EVT/TLM) is removed by removing its
   producers, not replaced by a queue — resolves the companion
   `get-wire-output-events-telemetry-out-of-the-main-loop.md` issue by
   subsumption; that issue's queue/drain design is deferred-or-obsolete.
2. The safety-watchdog removal is a deliberate, stakeholder-owned decision,
   acceptable only because the robot runs stand-mounted with wheels off the
   ground for the duration of this command surface's operating envelope.
3. Removed classes/command families are parked (unwired, left on disk), not
   deleted — reversible, least-churn.
4. `sim_get_async_evts()` is kept as a permanent no-op stub rather than
   deleted, to avoid a host-side ABI break this sprint does not otherwise
   touch.

## GitHub Issues

(None — this sprint is sourced from `clasi/issues/`, not GitHub.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Minimal command table + handleS/handleStop rewrite | — |
| 002 | MainLoop::tick() gut + main.cpp slim + sim_api.cpp lockstep | 001 |
| 003 | Sim-test parking + focused S/STOP/PING/HELLO suite | 002 |
| 004 | Bench-gate verification on the stand | 003 |

Tickets execute serially in the order listed.
