---
id: "105"
title: "Sim rebuild around the steppable loop"
status: roadmap
branch: sprint/105-sim-rebuild-around-the-steppable-loop
use-cases: []
issues: []
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 105: Sim rebuild around the steppable loop

## Goals

ROADMAP-STAGE ENTRY (not yet detailed — no architecture-update.md content,
no tickets). This sprint is P7 of
`clasi/issues/single-loop-firmware-p3-p7-continuation.md`, the third and
final sprint in the single-loop firmware arc. Sprint 102 deleted the old
sim build (`tests/_infra/{sim,drive}`) alongside the rest of the Elite
orchestration stack, explicitly deferring simulation to "its own later
phase" once a steppable loop exists to build it around. Sprints 103/104
build and prove that loop on real hardware; this sprint rebuilds simulation
against it:

- A thin `sim_api` layer around the new steppable loop, using the existing
  `HOST_BUILD` scripted-fake surface the `devices/` leaves and `I2CBus`
  already carry (`i2c_bus.h`'s `scriptWrite()`/`scriptRead()`/`setClock()`,
  already host-buildable — confirmed present and unchanged through sprint
  102/103).
- Fault injection, per `clasi/issues/later/sim-hardware-fault-injection.md`
  (retarget that issue's scope onto the new loop once this sprint is
  detailed — do not assume its pre-rebuild design carries over unchanged).
- Restore a green, runnable pytest sim tier (retired by sprint 102's
  deletion of `tests/_infra/sim`).
- TestGUI revival is explicitly a LATER follow-on, not this sprint's scope
  (per the continuation issue's P7 note: "sim rebuild ... own sprint; ...
  then testgui revival").

## Problem

There is currently no simulation build for the single-loop firmware — sim
was deleted wholesale in sprint 102 rather than migrated, because migrating
a simulation harness built for the discarded Elite/fiber architecture would
have been wasted effort. Firmware changes since 102 (103's new loop, 104's
host realignment) have had no sim coverage; every verification has been
real-hardware-only.

## Solution

Design deferred to this sprint's own detail-mode planning pass. The
continuation issue's steer: build `sim_api` thin (a host-buildable seam
over the SAME production loop code — not a parallel model), reusing the
`HOST_BUILD` scripted-fake primitives already in `devices/` rather than
inventing a new mocking layer.

## Success Criteria

A sim pytest tier exists, is green, and is runnable in CI without hardware;
it exercises the real `source/app/` loop code (not a duplicate model) via
`HOST_BUILD`.

## Scope

### In Scope

- `sim_api` around the steppable loop.
- Fault injection (retargeted from the parked `later/` issue).
- Sim pytest tier restoration.

### Out of Scope

- TestGUI revival (explicit later follow-on, not this sprint).
- Any new hardware-facing firmware behavior (this sprint is host/test-only
  unless detail-mode planning finds a real gap).

## Test Strategy

(Deferred to detail-mode planning for this sprint.)

## Architecture Notes

Depends on sprints 103 and 104 being complete (a stable, bench-proven
single loop and fully realigned host tooling to build the sim harness
against). Full architecture-update.md is written when this sprint is
detailed.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
