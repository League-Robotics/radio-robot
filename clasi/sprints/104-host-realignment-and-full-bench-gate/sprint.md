---
id: "104"
title: "Host realignment and full bench gate"
status: roadmap
branch: sprint/104-host-realignment-and-full-bench-gate
use-cases: []
issues: []
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 104: Host realignment and full bench gate

## Goals

ROADMAP-STAGE ENTRY (not yet detailed — no architecture-update.md content,
no tickets). This sprint is P5 (remainder) + P6 of
`clasi/issues/single-loop-firmware-p3-p7-continuation.md`, the second sprint
in the single-loop firmware arc. Sprint 103 ("the loop, the wire, and first
drive") lands the on-robot loop plus a MINIMAL host twist-sender sufficient
to prove the loop on the stand — it deliberately does not fully realign the
host tooling. This sprint finishes that realignment and proves the whole
stack under sustained load:

- **P5 remainder** — all host builders beyond the minimal
  `NezhaProtocol.twist()/stop()` slice sprint 103 lands (e.g. `config`
  arm builder, any remaining ack-ring ergonomics), deletion of the legacy
  text-plane / segment-era translators and dead builders in
  `host/robot_radio/robot/protocol.py` and `host/robot_radio/io/serial_conn.py`
  that no longer have a firmware target after 103's protocol prune, and a
  rewrite of the `tests/bench/rig_dev.py`/`rig_soak.py` family onto the
  binary twist/stop plane (they currently assume the pre-103 segment/drive
  wire surface and DeviceBus-era bringup — both retired by sprint 103).
- **P6** — soak testing: zero I2C errors, zero wedge latch, TLM drop-rate
  measured over BOTH USB and the radio relay under sustained load (not just
  the short bench-gate windows sprint 103's own gate uses), deadman
  kill-test repeated under soak conditions, and verification that the
  I2CBus `readyAt` safety net (added as a fault bit in 103 ticket 002) never
  fires during a clean soak run.

## Problem

Sprint 103 proves the new loop CAN be driven and CAN be trusted for one
bench session (the hard scoping rule: every sprint ends bench-runnable).
It does not prove the full host tooling surface works, and it does not
prove the loop survives sustained/soak conditions on both transports. The
old host tooling (`rig_dev.py`, `rig_soak.py`, the legacy text-plane
translators) still assumes the retired wire surface and will not run
against 103's firmware without rewriting.

## Solution

Rewrite the host tooling onto the pruned twist/config/stop + ack-ring wire
plane sprint 103 ships, delete what no longer has a firmware target, and
run sustained soak verification over both transports. High-level design
(module list, sequencing, ticket breakdown) is deferred to this sprint's own
detail-mode planning pass — do not treat this roadmap entry as a design
commitment beyond what is stated here.

## Success Criteria

Full host tooling (calibration, bench scripts, rig_dev/rig_soak) drives the
robot over the new binary plane on both USB and relay; a sustained soak run
(both transports) is clean — zero I2C errors, zero wedge latch, a measured
(not assumed) TLM drop rate, and the I2CBus safety-net fault bit never
fires.

## Scope

### In Scope

- Host: remaining P5 builders, legacy translator deletion, `serial_conn.py`
  ack-ring matcher hardening (built minimally in 103; hardened here),
  `rig_dev.py`/`rig_soak.py` family rewrite to the binary plane.
- Firmware: none planned (this sprint is host-side; if soak testing finds a
  firmware defect, a firmware ticket may be added during this sprint's own
  detail-mode planning — not decided here).
- Bench: soak-duration (not smoke-duration) verification over USB and relay.

### Out of Scope

- Sim rebuild (sprint 105, P7).
- Any new wire protocol fields beyond what sprint 103 ships (no schema
  changes planned; if soak testing reveals a real budget/field gap, that is
  a finding for this sprint's own planning, not pre-decided here).

## Test Strategy

(Deferred to detail-mode planning for this sprint.)

## Architecture Notes

Builds directly on sprint 103's `source/app/{Comms,Telemetry,Deadman,Drive,
Odometry}` and the pruned `protos/{envelope,telemetry}.proto` — no firmware
architecture change is anticipated, only host-side. Full architecture-update.md
is written when this sprint is detailed.

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
