---
id: '060'
title: 'Ordered-tick cutover: close parity gaps, make default, delete legacy control
  path'
status: done
branch: sprint/060-ordered-tick-cutover-close-parity-gaps-make-default-delete-legacy-control-path
use-cases: []
issues:
- make-ordered-tick-the-default-close-parity-gaps.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 060: Ordered-tick cutover: close parity gaps, make default, delete legacy control path

## Goals

Complete the message-architecture migration so there is NO legacy control path left.
Close the 3 parity gaps (TLM source, motor-output authority, sensor schedule), make
`USE_ORDERED_TICK` the only compile-time path, delete the legacy loop and all dead
legacy members, and rename the `2` scaffolding to clean names. Land everything on
master in a single sprint.

## Problem

Sprints 054-059 built the message-based architecture additively — Drive2, Sensors,
MotionController2, BusDrain, bottom-up config — but never became the live default
because ticket 059-005 left 3 documented parity gaps unresolved. The legacy loop
(`#ifndef USE_ORDERED_TICK`, lines 57-159 of `LoopTickOnce.cpp`) is still the
production control path. The `2`-suffix names (drive2, bvc2, MotionController2,
planner) are temporary migration scaffolding. Until this sprint lands, the
message-driven architecture is present and unit-tested but is not live.

## Solution

Seven sequenced tickets that each leave the build green:

1. Rewire `buildTlmFrame` to read from `drive2.state()` / `sensors.state()` and
   regenerate the golden-TLM capture with stakeholder review.
2. Resolve the `MotorController::setCommandsRef` authority conflict so Drive2's
   `_outputs` is the single motor-command sink.
3. Unify the sensor lag-timer schedule so `sensors.tick()` is the sole driver of
   line/color reads.
4. Flip `USE_ORDERED_TICK` to the default; run full host suite green with the
   ordered tick live.
5. Delete the legacy `#ifndef` branch and all dead members from Robot.
6. Rename `bvc2→bvc`, `Drive2→subsystems::Drive`, `MotionController2→MotionController`,
   `planner` stays `planner`.
7. Build the firmware, run the parity bench checklist (human-operated, host-side
   preparation only).

## Success Criteria

- `USE_ORDERED_TICK` is the only path; legacy loop branch and dead members deleted.
- `grep -r USE_ORDERED_TICK source/ tests/` returns nothing.
- Full host suite green with the ordered tick live (excluding 2 known-baseline
  config-golden failures: `tag_offset_mm.z` schema gap + DefaultConfig golden drift).
- Golden-TLM snapshot regenerated and diff reviewed.
- Bench checklist produced; physical tovez parity run executed by stakeholder.

## Scope

### In Scope

- Close gap #1: rewire `buildTlmFrame` to `drive2.state()` / `sensors.state()`; regenerate golden.
- Close gap #2: single `setCommandsRef` authority for motor output.
- Close gap #3: unify sensor lag timers under `sensors.tick()`.
- Flip `USE_ORDERED_TICK` to default.
- Delete legacy loop branch (`LoopTickOnce.cpp:57-159`) and dead Robot members.
- Rename: `bvc2→bvc`, `subsystems::Drive2→subsystems::Drive`, `MotionController2→MotionController`, de-`2` planner type.
- Bench-parity preparation: firmware build + documented checklist (physical run by stakeholder).

### Out of Scope

- Issue 2: `tag-offset-mm-z-field-schema-mismatch.md` — deferred. The 2 baseline config-golden failures are the accepted known baseline for this sprint.
- `Ports2` / full Ports subsystem migration — `ports.periodic()` stays and is documented as remaining scaffolding.
- Any physical bench run by an agent — the bench ticket produces artifacts; the human executes on hardware.

## Test Strategy

Each ticket runs `uv run python -m pytest` (NOT `uv run pytest`). Sprint acceptance
is green across the full host simulation suite except the 2 known-baseline config
failures. The golden-TLM test is the primary behavior oracle: it must be regenerated
in ticket 001 (with stakeholder diff review) and must remain green for all subsequent
tickets. The parity tests (`test_059_ordered_tick_parity.py`) and planner-isolation
tests must also remain green throughout.

## Architecture Notes

- `LoopTickOnce.cpp` is the only file containing `USE_ORDERED_TICK` today.
- Robot.h member declaration order is load-bearing: `bvc` before `drive` before `sensors` before `planner`.
- Drive2 holds its own `_outputs`; after gap #2 that becomes the sole motor sink.
- Sensors facade (`sensors.tick()`) already has its own lag timers; gap #3 removes the redundant legacy calls.
- After deletion, `subsystems::Drive` (the renamed Drive2) is the only Drive class; the old `subsystems::Drive` header (`drive/Drive.h`) is deleted.
- After rename, `MotionController` refers to the new-arch class; the old imperative `MotionController` is deleted (it was already wrapped and only called via MotionController2).

## GitHub Issues

(None linked yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Close gap 1: rewire TLM to Drive2/Sensors state; regenerate golden | — |
| 002 | Close gap 2: single motor-output authority (setCommandsRef) | 001 |
| 003 | Close gap 3: unify sensor lag schedule under sensors.tick() | 002 |
| 004 | Flip USE_ORDERED_TICK to default; full host suite green | 003 |
| 005 | Delete legacy loop branch and dead Robot members | 004 |
| 006 | Rename bvc2/Drive2/MotionController2 to clean names | 005 |
| 007 | Bench-parity prep: firmware build + parity checklist | 006 |

Tickets execute serially in the order listed.
