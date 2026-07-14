---
id: '102'
title: 'Single-loop firmware: spikes, archive, and delete to stub (P0-P2)'
status: done
branch: sprint/102-single-loop-firmware-spikes-archive-and-delete-to-stub-p0-p2
use-cases:
- SUC-001
- SUC-003
- SUC-004
- SUC-005
issues:
- single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 102: Single-loop firmware: spikes, archive, and delete to stub (P0-P2)

## Goals

Execute phases P0, P1, and P2 (exactly — no P3+ work) of the stakeholder-approved
plan in
`clasi/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`:

- **P0 — de-risk before deleting anything.** Two spikes, no destructive changes:
  (a) measure whether the radio relay's `!GO` data plane sustains a pushed ~30 Hz
  binary telemetry stream or silently drops it (push vs. host-paced-poll return
  path decision) — this measurement also sets the shared telemetry rate budget
  for both the radio and serial transports (stakeholder decision 2026-07-14: no
  baud change, see Problem/Success Criteria below); (b) dry-run the pruned
  wire-protocol proto budget (`gen_messages.py` + `wire.h` static_asserts) with
  no hardware involved. (A third spike, the serial baud ceiling, was dropped by
  stakeholder decision — see Tickets.)
- **P1 — a way back.** Tag the pre-deletion state and archive two known-good
  flashable hexes (default + devicebus-bringup), each proven by a real reflash.
- **P2 — delete the Elite plumbing, in one commit.** Remove the ~15,900 remaining
  lines of `runtime/`, `subsystems/`, `commands/`, `drive/`, `telemetry/`, `hal/`,
  `com/i2c_bus*`, `estimation/`, dead `types/`, fiber/staging machinery, and
  orphaned vendored libs; replace `source/main.cpp` with a ~50-line banner-only
  stub (motors never energized); prune the matching test/build/host surface.
  Leaves the tree in a flashable, bootable, but functionally inert state — P3
  (build the real loop) is a later sprint.

This sprint does **not** build the new single-loop firmware, the new wire
protocol, or any host changes — those are sprint 103 (P3-P4) and sprint 104
(P5-P6), noted as roadmap successors only, not detailed here.

## Problem

The 2026-07-13/14 code review
(`docs/code_review/2026-07-13-devices-drive-review.md`) found that nearly every
major DeviceBus bug lives at the fiber boundary (staging, stale gates, ring
stamps, adapter seam) and that the Elite architecture's plumbing (blackboard,
router, Hal seam, Configurator) is where behavior hides (no-op ticks, blind
wedge flags, silent segment drops) across ~7,100 lines of dead code. The
stakeholder decided (2026-07-14) to remove on-robot trajectory planning
entirely: host plans, robot follows as a velocity/yaw follower with continuous,
honest telemetry. The stakeholder decision is to delete the old stack up front
(fallback = git tag + archived hex, not parked code) rather than incrementally
migrate it — but two unknowns must be resolved *before* an irreversible
delete: relay telemetry-push behavior (which also sets the shared telemetry
rate budget for serial and radio, per the 2026-07-14 stakeholder decision
below), and whether the pruned wire frame actually fits its budget. A third
unknown, the serial baud ceiling, is no longer resolved by measurement — the
stakeholder decided (2026-07-14) the radio is the robot's production
interface and its throughput is fixed, so serial matches whatever cadence
the radio sustains rather than being raised independently.

## Solution

Two independent P0 spike tickets (current firmware, no destructive changes)
answer the open questions and record verdicts in
`.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md` and this sprint's
tickets. (A third spike ticket, the serial baud ceiling, was dropped by
stakeholder decision 2026-07-14 — see Tickets.) A P1 ticket creates the
safety net (tag + two archived, reflash-proven hexes) that makes the P2
delete reversible without parked code. The P2 ticket executes the full
delete/keep inventory from the issue as one commit, replacing
`source/main.cpp` with a banner-only stub, so the "no working firmware" window
is exactly one commit and every commit on either side of it is flashable.

## Success Criteria

- Relay push-vs-poll verdict recorded, with the 2026-06-12 knowledge note
  updated to confirm or retract "async STREAM frames dropped by the bridge"
  against current relay firmware.
- Sustainable telemetry frame rate measured through the relay and, separately,
  over direct USB at the fixed 115200 baud (ticket 001); the recommended
  common telemetry cadence for both transports is the minimum of the two,
  with headroom, recorded as the rate budget P4/P5 must honor. No baud change
  anywhere in the stack (stakeholder decision 2026-07-14: the radio is the
  production interface and sets the budget; the bench must never enjoy
  bandwidth the field doesn't have).
- Pruned envelope/telemetry proto draft passes `wire.h`'s static_asserts
  (worst case must fit the 186 B envelope ceiling) — no hardware required, no
  merge to the live protos this sprint.
- `pre-single-loop` annotated tag pushed; two archived hexes (default,
  devicebus-bringup) each proven by a real reflash on the bench rig.
- P2 delete lands as one commit: `just build` produces a hex, the stub flashes
  and banners on the stand, the surviving pytest subset is green, and a grep
  for every deleted header returns nothing under `source/`, `tests/`, `host/`.

## Scope

### In Scope

- P0 spikes (relay push telemetry + shared rate-budget measurement,
  wire-frame budget dry run) — measurement and documentation only, against
  CURRENT firmware. (Ticket 002, a serial baud-ceiling spike, was dropped by
  stakeholder decision 2026-07-14 — see Tickets.)
- P1 tag + hex archival with reflash proof.
- P2 full delete inventory (source, tests, build, host fallout) + banner-only
  stub `main.cpp`, per the issue's delete/keep list, corrected for what
  `3c4a8c0a` already deleted on this branch (see Architecture Notes).

### Out of Scope (deferred to sprint 103 / 104, roadmap only)

- P3: writing the new single-loop `main()`, `Comms`, `Telemetry`+ack ring,
  `Deadman`, `Drive`, `Odometry`, `Preamble` driver in `source/app/`.
- P4: the actual pruned wire protocol landing in `protos/` + regenerated
  messages (the wire-frame budget spike only dry-runs this on a scratch
  branch; it does not merge).
- P5: host-side `twist`/`config`/`stop` builders, ack-ring matcher, applying
  the common telemetry cadence ticket 001 measures. No baud raise anywhere —
  dropped by stakeholder decision 2026-07-14 (serial stays 115200
  permanently, matching whatever the radio sustains).
- P6: bench-gate rig rewrite (`rig_dev`/`rig_soak` binary-plane soak).
- P7 (separate future sprint): sim rebuild + testgui revival.

## Test Strategy

P0 spikes are themselves measurement exercises (relay/USB telemetry rate and
common-cadence recommendation, `wire.h` static_assert pass/fail) — their
"tests" are the measurement runs. P1's test is a real reflash of each
archived hex. P2's gate is: `just build` succeeds, the flashed stub banners on
the stand (hardware bench gate per
`.claude/rules/hardware-bench-testing.md` — connect only, no drive
verification since the stub never energizes motors), the surviving pytest
subset stays green, and a repo-wide grep for deleted headers is empty.

## Architecture Notes

This sprint's architecture-update.md documents the deletion as a structural
contraction, not a redesign — no new production module is introduced except
the stub `main.cpp`. Key constraint: commit `3c4a8c0a` (already on this branch,
ahead of `master`) already deleted `source/motion/`, `subsystems/nezha_hardware`,
`hal/nezha/*`, `hal/otos/*`, the four `hal/capability` faceplates
(gripper/ports/color_sensor/line_sensor), several dead CMake flags, and some
member-level dead code. The P2 delete ticket's inventory is corrected to
exclude all of that — it targets what's still present:
`source/{runtime,subsystems,commands,drive,telemetry,hal}`,
`com/i2c_bus.{h,cpp}` + `com/i2c_bus_host.cpp` (the parked duplicate; the live
`devices/i2c_bus` stays), `estimation/`,
`types/{arg_schema,command_types,clock*,value_set}`, `kinematics/i_kinematics.h`,
`devices/{bringup_main.cpp,fiber_runner.h}`, `codal.devicebus.json`,
`libraries/{ruckig,tinyekf,cmon-pid}`.
`tests/sim/unit/i2c_bus_clearance_harness.cpp` (`test_i2c_bus_clearance`) is the
one test keeping `com/i2c_bus` alive and is retired alongside it.

## GitHub Issues

(None — tracked via the CLASI issue file linked below.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan (basis: stakeholder decisions
      recorded 2026-07-14 in the linked issue file; see the
      `stakeholder_approval` gate notes)

## Tickets

**Note (stakeholder decision, 2026-07-14):** ticket 002 (P0 spike: serial
baud ceiling) is dropped. The radio relay is the robot's production
interface and its throughput is fixed — raising the USB baud would only let
the bench diverge from the field. Ticket 001's measured sustainable frame
rate through the relay now SETS the telemetry rate budget for both
transports (serial remains at its current 115200 baud, no change). The
ticket number is left as a gap, not reused, to preserve the ticket-history
record.

| # | Title | Depends On |
|---|-------|------------|
| 001 | P0 spike: relay sustained-push telemetry (sets shared telemetry rate budget for both transports) | — |
| 003 | P0 spike: wire-frame budget dry run | — |
| 004 | P1 tag + archive: pre-single-loop rollback artifacts | 001, 003 |
| 005 | P2 delete Elite plumbing + banner-only stub main (one commit) | 001, 003, 004 |

Tickets execute serially in the order listed. Tickets 001 and 003 are
independent of each other (different subsystems: radio/serial telemetry
rate, protos) and could in principle run in parallel, but are listed and
executed in numeric order per the sprint-planner's serial-execution
convention. Ticket 004 must follow both spikes (tags the fully-de-risked
commit). Ticket 005 (the irreversible delete) must follow 004 — it deletes
`codal.devicebus.json`, which ticket 004's devicebus-bringup hex archival
depends on still existing.
