---
id: '053'
title: "Stop conditions \u2014 Phase 2 (collapse open-loop family)"
status: done
branch: sprint/053-stop-conditions-phase-2-collapse-open-loop-family
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- stop-conditions-as-a-first-class-system-primitive.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 053: Stop conditions — Phase 2 (collapse open-loop family)

## Goals

Remove the verb-identity mirroring from the firmware motion pipeline so the
internals run purely on *twist + stop conditions*. After this sprint, the system
has no stringify/re-parse round-trip for the open-loop family, no separate Goal
variants for STREAM/TIMED/DISTANCE/ARC/VELOCITY, and a shrunken Origin enum whose
only remaining job is the VW-keepalive retargetability flag. Sprint 052 delivered
the additive Phase 1 (stop= parser, reason= reporting, host Stop builders); this
sprint is the structural Phase 2 that resolves the issue.

## Problem

After Phase 1, S/T/D/R/VW accept `stop=` clauses and report `reason=` on
completion. However, the underlying routing still:

1. Stringifies verb identity (`t=`, `dist=`, `stream=`, `radius=`, `h=`, `rot=`)
   into a fake VW ParsedCommand and pushes it on the queue, then re-parses those
   strings in handleVW to demux back into 8 separate Goal variants and begin*()
   calls. The T/D path additionally round-trips (v, ω) back to (vL, vR) via
   `inverse()` so beginTimed/beginDistance can accept wheel speeds.
2. DriveMode::STREAMING (the S command) bypasses MotionCommand entirely, so
   stop= clauses attached to S are parsed but can never fire (Phase 1 left this
   as a known gap, noted in the STREAM branch comment).
3. `MotionCommand::Origin` carries 7 variants (VW, TURN, G, T, D, R, RT) even
   though only the VW vs. non-VW distinction is used for the keepalive guard.

## Solution

Four coordinated changes to `source/commands/MotionCommands.cpp`,
`source/superstructure/Superstructure.h/.cpp`,
`source/control/MotionControllerBegin.cpp`, and `source/commands/MotionCommand.h`:

1. **Eliminate the stringify/re-parse round-trip for S/T/D/R**: each handler now
   calls `begin*()` directly (via GoalRequest on the Superstructure seam) and
   passes StopConditions to `mc_applyStopClauses` after the begin call, rather
   than packing KV strings for handleVW to re-parse. Remove the `packKVArg`
   helper and the inverse() round-trip for T/D. Remove `argsHasKey`/`argsScanKV`
   demux for t=/dist=/stream=/radius= in handleVW.
2. **Migrate S (DriveMode::STREAMING) onto a MotionCommand velocity goal** with
   a `streamSeed` flag so stop= clauses actually fire on S. Extend `GoalRequest`
   with `StopCondition stops[4]; uint8_t nStops; bool streamSeed;
   const char* doneLabel;` and route STREAM through it. `beginStream` is
   reworked to configure a MotionCommand that seeds the BVC immediately (no ramp)
   and exits STREAMING mode; `DriveMode::STREAMING` becomes purely a label for
   the BVC-seeded velocity path.
3. **Collapse Goal::{STREAM,TIMED,DISTANCE,ARC,VELOCITY} into a single VELOCITY
   goal** via the `streamSeed` and `doneLabel` fields in GoalRequest. Keep
   Goal::GOTO, Goal::TURN, Goal::ROTATE (closed-loop controllers).
4. **Shrink `MotionCommand::Origin`** to two values: `RETARGETABLE` (VW-origin)
   and `FIXED` (everything else). The keepalive guard in handleVW is preserved
   unchanged in effect.
5. **Re-baseline the golden-TLM canary** as a deliberate, reviewed step.

## Success Criteria

- `uv run --with pytest python -m pytest tests/simulation -q` passes with
  exactly 2 known failures (test_default_config_pin, test_robot_config schema).
- S with `stop=` clauses fires and reports `reason=`.
- `EVT done T/D/R/G/TURN/RT` labels are preserved on the wire.
- VW keepalive guard works: non-retargetable active commands reply `busy=`.
- D encoder reset: `distanceDrive` still resets encoders before snapshotting baseline.
- `python build.py --clean` exits 0.
- Golden-TLM canary diff is reviewed and re-baselined deliberately.
- The stop-conditions issue is closed (completes_issue: true on ticket 006).

## Scope

### In Scope

- Eliminate packKVArg / argsHasKey / argsScanKV for the S/T/D/R/stream/t/dist/radius keys.
- Remove the inverse() round-trip for T/D (forward kinematics was already done in handleT/handleD before pushing; handleVW re-did the inverse — remove that).
- Migrate S onto MotionCommand with a streamSeed flag in GoalRequest.
- Collapse STREAM/TIMED/DISTANCE/ARC/VELOCITY into one VELOCITY path.
- Shrink Origin enum to RETARGETABLE / FIXED.
- Extend GoalRequest with stops[], nStops, streamSeed, doneLabel.
- Re-baseline the golden-TLM canary.
- Sim tests confirming stop= fires on S, wire labels preserved, keepalive guard works.
- Firmware clean build (`python build.py --clean`).

### Out of Scope

- G/TURN/RT routing changes (closed-loop controllers; h= and rot= demux in
  handleVW stays as-is for TURN and RT, which remain closed-loop).
- New stop condition kinds.
- Host-side protocol.py changes (stop= builders done in Phase 1).
- HaltController changes.

## Test Strategy

Canonical test command: `uv run --with pytest python -m pytest tests/simulation -q`

Known pre-existing failures (2, do not regress):
- `tests/simulation/unit/test_default_config_pin.py::test_default_robot_config_unchanged`
- `tests/simulation/unit/test_robot_config.py::TestSchemaValidation::test_tovez_validates_against_schema`

New tests needed:
- S with stop=d:300 fires and reports reason=dist.
- S with stop=line:ge:512 fires and reports reason=line.
- T/D/R continue to fire existing stops and the Origin guard tests pass.
- Wire label preservation: EVT done T/D/R confirmed.
- Keepalive guard: non-retargetable active command → busy= reply.
- Golden-TLM canary: re-baseline with diff review (dedicated ticket).
- Firmware clean build: `python build.py --clean` exits 0.

## Architecture Notes

- `mc_applyStopClauses` and `mc_parseStopToken` (Phase 1) are retained.
- The D11 reply-suppression rule (converter already replied) still applies for
  TURN/RT/G which remain on the KV push path.
- `beginStream` in MotionControllerBegin.cpp is reworked; `DriveMode::STREAMING`
  label is preserved for the BVC-seeded path.
- Distance encoder reset: `Robot::distanceDrive` atomic reset (beginDistance +
  resetEncoders) is preserved by routing DISTANCE through GoalRequest, which
  continues to call `robot->distanceDrive`.
- The golden-TLM canary change is INTENTIONAL and expected. A dedicated ticket
  captures the diff-review step.

## GitHub Issues

(No GitHub issues linked yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Extend GoalRequest: stops[], streamSeed, doneLabel | — |
| 002 | Shrink Origin enum to RETARGETABLE/FIXED | 001 |
| 003 | Migrate S onto MotionCommand (streamSeed path) | 001, 002 |
| 004 | Eliminate T/D stringify+inverse round-trip | 001, 002 |
| 005 | Eliminate R stringify round-trip; collapse VELOCITY goal | 001, 002 |
| 006 | Golden-TLM re-baseline, validation, firmware clean build | 003, 004, 005 |

Tickets execute serially in the order listed.
