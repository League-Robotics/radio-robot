---
id: '104'
title: Host realignment and full bench gate
status: done
branch: sprint/104-host-realignment-and-full-bench-gate
use-cases: []
issues:
- rig-persistent-otos-distrust.md
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

`uv run python -m pytest tests/unit -q` must report 0 failed/0 errors by
ticket 002 and stay that way through ticket 007 (baseline measured
2026-07-14 against the merged 103 tree: 112 failed, 5 errors, 297
passed). Firmware-side (ticket 004) gets `HOST_BUILD` unit coverage for
the new fault bit. No sprint in this arc closes on tests alone — ticket
007's real-hardware soak session (both transports, sustained duration) is
this sprint's actual Definition of Done, per
`.claude/rules/hardware-bench-testing.md`.

## Architecture Notes

Builds directly on sprint 103's `source/app/{Comms,Telemetry,Deadman,Drive,
Odometry}` and the pruned `protos/{envelope,telemetry}.proto`. One small
firmware touch this sprint after all (ticket 004: `kFaultCommsMalformed`
bit + `kFaultI2CSafetyNet` doc correction) — additive/documentation-only,
no wire schema change, no behavioral change to the loop or dispatch path.
Everything else is host-side (`host/robot_radio/`, `tests/bench/`,
`data/robots/*.json`). See `architecture-update.md` for the full 7-step
document (module list, diagrams, design rationale, open questions).

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Host command surface completed — NezhaProtocol.config() | — |
| 002 | Legacy translator and dead-verb deletion | 001 |
| 003 | serial_conn ack-ring matcher hardening + TelemetrySecondary consumption | 002 |
| 004 | Firmware fault-bit follow-ups — kFaultCommsMalformed + kFaultI2CSafetyNet characterization | — |
| 005 | Rig profile — persistent OTOS-untrusted marker | — |
| 006 | Bench script family rewritten to the binary twist/config/stop plane | 001, 003 |
| 007 | P6 soak gate — sustained dual-transport bench-runnable verification | 001, 002, 003, 004, 005, 006 |

Tickets execute serially in the order listed. 004 and 005 have no
dependency on 001-003 and could run in parallel with them in a future
execution pass that chooses to parallelize despite this plan's serial
order (architecture-update.md Migration Concerns); 007 is strictly last.
