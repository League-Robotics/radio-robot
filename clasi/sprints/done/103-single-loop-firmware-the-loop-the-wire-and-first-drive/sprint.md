---
id: '103'
title: 'Single-loop firmware: the loop, the wire, and first drive'
status: done
branch: sprint/103-single-loop-firmware-the-loop-the-wire-and-first-drive
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
- SUC-009
- SUC-010
issues:
- single-loop-firmware-p3-p7-continuation.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 103: Single-loop firmware: the loop, the wire, and first drive

## Goals

Land phases P3 (the single loop) and P4 (the wire protocol) of
`clasi/issues/single-loop-firmware-p3-p7-continuation.md`, plus the minimal
slice of P5 (host) needed to command the loop — per the 2026-07-14
stakeholder hard scoping rule, **every sprint ends bench-runnable**, so this
sprint is not "P3 only." It ends with the robot on new firmware, driven by
a minimal host twist-sender, verified on the stand.

Sprint 102 left the tree at a banner-only stub (`source/main.cpp`): boots,
identifies, never touches I2C, never energizes a motor. This sprint builds
the real single foreground loop on top of that stub, replacing it — the
`runAndWait`/`markTime`/`sleepUntil` main loop, the pruned twist/config/stop
wire protocol with an always-on ack-ring telemetry return path, and just
enough host tooling to prove it.

## Problem

The pre-102 firmware planned trajectories on-robot through a
fiber/blackboard/router/Configurator orchestration stack that the
2026-07-13/14 code review found was where nearly every defect lived. The
stakeholder decided: host plans, robot follows — a velocity/yaw follower
with continuous, honest telemetry, one sequential program with no fiber and
no on-robot planning. Sprint 102 measured the two open risks (relay push
telemetry behavior + wire-frame budget) and deleted the old stack to a
stub. Nothing drives the wheels yet.

## Solution

Build `source/app/` (Comms, Deadman, Telemetry, Drive, Odometry, a Preamble
boot driver) directly on top of the EXISTING `devices/` leaves
(`I2CBus`, `NezhaMotor`, `Otos`, `ColorSensorLeaf`, `LineSensorLeaf`) —
**not** through `Devices::DeviceBus`/`handles.h`'s fiber-owned handle
abstraction, which this sprint retires (see architecture-update.md Decision
1 for why). Prune `protos/{envelope,telemetry}.proto` to the twist/
config/stop command surface plus an always-on ack-ring (depth 3, per spike
003's measured budget) telemetry return path. Fold the 2026-07-13 code
review's C1 (NAK'd stop write latched as written) and M1 (busy-spin
clearance) fixes into the kept `NezhaMotor`/`I2CBus` leaves. Write a new
`source/main.cpp` implementing the archived plan's one-page main loop
verbatim in shape. Add a minimal host slice (`NezhaProtocol.twist()`/
`stop()` + an ack-ring matcher) and a bench drive script, sufficient to
prove the loop on the stand — full host realignment is sprint 104.

## Success Criteria

On the bench rig (wheels off the ground): telemetry streams from
power-on; a host-sent twist command drives both wheels under velocity PID
in both directions with encoders tracking; an ack for that twist's
`corr_id` is observed in the telemetry ack ring over BOTH direct USB and
the radio relay; killing the host sender causes the deadman to stop the
wheels within one stale window; `grep 'runAndWait\|sleepUntil'
source/main.cpp` shows the complete timing schedule (three settle/clearance
windows plus the pace sleep, matching the archived plan's schedule
one-for-one). The robot ends the sprint ON the new firmware, drivable.

## Scope

### In Scope

- `protos/envelope.proto` + `protos/telemetry.proto` pruned to
  twist/config/stop + ack ring (depth 3) + fault/event bits + a
  `TelemetrySecondary` slow frame; regenerated `source/messages/*` +
  `envelope_pb2`/`telemetry_pb2`; wire test harnesses rewritten to the
  pruned schema (protobuf differential oracle kept).
- `Devices::NezhaMotor`/`Devices::I2CBus` write-path hardening (C1, M1) —
  folds `clasi/issues/nezha-motor-write-path-hardening.md` into this
  sprint's port.
- Retirement of `Devices::DeviceBus`/`handles.h` (the fiber-owned handle
  abstraction) — `source/app/` drives the leaves directly instead.
- `source/app/{comms,deadman,telemetry,drive,odometry,preamble}` (naming
  final at ticket time — lowerCamelCase files/types per project convention)
  built on the bare leaves.
- New `source/main.cpp`: boot loop (telemetry from power-on) +
  `runAndWait`/`markTime`/`sleepUntil` cycle per the archived plan, verbatim
  in shape.
- Minimal host slice: `NezhaProtocol.twist(v_x, omega, duration)` + `stop()`
  + an ack-ring matcher in `host/robot_radio/robot/protocol.py`, plus one
  `tests/bench/` drive script.
- Final bench-gate ticket: on-stand verification per Success Criteria above.

### Out of Scope

- Full host realignment: remaining P5 builders, legacy text/segment-era
  translator deletion, `serial_conn.py` ack-ring hardening beyond the
  minimal matcher, `rig_dev.py`/`rig_soak.py` rewrite — sprint 104.
- Soak-duration verification (this sprint's gate is a bench-session
  duration, not a soak) and TLM drop-rate measurement over sustained load —
  sprint 104 (P6).
- Sim rebuild — sprint 105 (P7), out of scope by construction (sprint 102
  deleted the old sim build; no sim exists to update).
- `config` command arm runtime wiring beyond schema (the `Comms`/dispatch
  path decodes it as a no-op-safe unknown-arm case if not reached this
  sprint — ticket-time decision) — full config-delta application is
  in scope only if a ticket's own acceptance criteria commit to it; see
  ticket 004/008.
- Any baud change (dropped by 2026-07-14 stakeholder decision, sprint 102).

## Test Strategy

Wire-layer: `wire.h` static_asserts (budget pass/fail) plus a rewritten
`wire_codec_harness.cpp`/`test_wire_codec.py`/`test_wire_differential.py`/
`test_wire_fuzz.py` against the pruned schema (protobuf differential oracle
kept — ticket 001). Leaf-level: existing `devices_*` unit tests stay green
through the write-path hardening (ticket 002); the retired
`device_bus_cycle_harness.cpp`/`test_device_bus_cycle.py` are deleted, not
ported (ticket 003 — DeviceBus itself is retired, so there is nothing left
to test). `source/app/` modules get host-buildable unit coverage where the
existing `HOST_BUILD` seam supports it (I2CBus's scripted fakes); no new
sim tier (sprint 105's job). The sprint's real gate is hardware: the final
bench-gate ticket (010) exercises `.claude/rules/hardware-bench-testing.md`'s
standing checklist (sensors alive, wheels drive both directions, round-trip
over the real link) plus this sprint's own additions (ack-ring observation
on both transports, deadman kill-test, the `runAndWait`/`sleepUntil` grep
check).

## Architecture Notes

See architecture-update.md for the full design. Headline decision: this
sprint retires `Devices::DeviceBus`/`handles.h` (built pre-102 as a
fiber-owned, handle-mediated device subsystem) in favor of driving the bare
`devices/` leaves directly from `source/app/`, per the archived plan's
explicit "no fiber, no handles, no staging layer" main-loop sketch — this
resolves a real drift between that sketch (bare-leaf construction) and
`DeviceBus`'s current already-narrowed-but-still-handle-based public
surface (`runPreamble()`/`runCycleOnce()`/`Motor`/`Odometer` handles).

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Prune wire protocol: twist/config/stop + ack-ring telemetry | — |
| 002 | NezhaMotor/I2CBus write-path hardening (C1, M1) | — |
| 003 | Retire DeviceBus/handles.h — leaves become the loop's direct dependency | — |
| 004 | app/Comms and app/Deadman | 001 |
| 005 | app/Telemetry — always-on frame, ack ring, fault bits | 001, 004 |
| 006 | app/Drive, app/Odometry, and minimal OTOS perception | 001, 003 |
| 007 | app/Preamble — boot-time device-detection driver | 003 |
| 008 | Real main.cpp — boot loop and runAndWait cycle | 002, 004, 005, 006, 007 |
| 009 | Minimal host slice — NezhaProtocol.twist/stop + ack-ring matcher | 001 |
| 010 | Bench gate — the robot drives on the new firmware | 008, 009 |

Tickets execute serially in the order listed. 001-003 are the independent
foundation (wire schema, leaf hardening, DeviceBus retirement — no
inter-dependencies among the three, ordered together for a clean "one
device-access pattern at a time" tree state). 004-007 are the `source/app/`
modules. 008 is loop integration. 009 is the host slice (depends only on
001, so it may in principle be pulled earlier by whoever executes this
sprint, but is listed last-but-one to keep the ticket list foundation-
before-features). 010 is the bench gate, strictly last.

Per the hard scoping rule, mid-sprint tickets (001 through 007) may leave
the firmware build broken between when ticket 001 lands and ticket 008
completes the wiring — no stub-main ceremony is required between tickets.
Only ticket 010's gate must pass for the sprint to be considered done.
