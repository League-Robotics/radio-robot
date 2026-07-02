---
id: '065'
title: 'Stop reliability and safety: stop-clause overflow, STOP delivery and motion
  watchdog, OTOS warn-bit fusion gate'
status: done
branch: sprint/065-stop-reliability-and-safety-stop-clause-overflow-stop-delivery-and-motion-watchdog-otos-warn-bit-fusion-gate
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
issues:
- stop-clause-overflow-aborts-process.md
- stop-delivery-and-keepalive-watchdog-architecture.md
- otos-warn-bit-fusion-spin-on-placement-regression.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 065: Stop reliability and safety: stop-clause overflow, STOP delivery and motion watchdog, OTOS warn-bit fusion gate

## Goals

Remove the three highest-severity safety defects outside the encoder
pipeline: the live assert that aborts the sim / panics firmware on stop-clause
overflow, the unreliable STOP delivery + ambient-keepalive watchdog defeat,
and the OTOS warn-bit fusion regression ("spin on placement").

## Problem

- CR-01 (critical): D/T stop double-booking overflows `kMaxStopConds` and
  hits `assert(false)` — aborts the whole Python process hosting the sim and
  panics real firmware mid-drive.
- CR-04/05 (high): TestGUI sends STOP once, fire-and-forget over a link that
  drops 15-50% of lines, and the connection-level `+` keepalive resets the
  firmware watchdog on ANY line — a hung host or dropped STOP means unbounded
  runaway on open-ended motion.
- CR-06 (high): `otosCorrect` fuses OTOS despite persistent WARNING bits;
  EKF gate-recovery then force-snaps pose/heading to frozen garbage within
  ~10 samples — the exact "spin on placement" failure the D9 gate prevented.

## Solution

- De-duplicate stop installation between `begin*()` and
  `Superstructure::requestGoal`; make `addStop` overflow a recoverable `ERR`.
- STOP via acked/retried path in the TestGUI KeyboardDriver + deadman resend;
  make the motion watchdog reset only on `+`/motion verbs and arm host
  keepalive only while a motion source is active; optional VW staleness cap.
- Gate OTOS fusion on warn-bit persistence (fuse through ≤K transient warn
  samples, block after, re-admit after N clean); add a sim warn-bit state so
  the gate is testable.

## Success Criteria

- `D 150 150 300 stop=time:9000 sensor=line0>500` runs in sim without abort
  and honors clauses; no wasted duplicate stop slots.
- Sim/unit tests: dropped-STOP scenario stops via deadman/ack path; ambient
  keepalive alone does NOT keep an open-ended VW alive past the watchdog;
  warn-persistent OTOS is not fused (pose follows encoders), warn-blip is.
- Full default suite green; ARM firmware builds clean.

## Scope

### In Scope

Firmware: `source/commands/MotionCommand.*`, `source/superstructure/
Superstructure.*`, `source/control/PlannerBegin.cpp`, `source/robot/Robot.cpp`
(otosCorrect), `source/state/EKFTiny.*` (gate-recovery interaction only as
needed), sim OTOS warn-bit support. Host: `host/robot_radio/testgui/drive.py`,
`host/robot_radio/io/serial_conn.py` (keepalive arming).

### Out of Scope

Encoder pipeline (sprint 064). Sim OTOS ground-truth fidelity (sprint 066 —
only the warn-bit state is added here). Hardware validation (stakeholder).

## Test Strategy

Sim-tier pytest drives all firmware behavior (stop clauses, watchdog,
fusion gate); testgui unit tests cover KeyboardDriver STOP/deadman and
keepalive arming. Full default suite green before close.

## Architecture Notes

- Watchdog semantics change is behavioral: document in architecture-update
  that `+` (and motion verbs) are the ONLY watchdog resets; bench scripts
  relying on ambient traffic must send `+` while streaming motion (existing
  scripts already do).
- Keep `Stop.*` wire format unchanged; only installation/overflow behavior
  changes.

## GitHub Issues

(none)

## Definition of Ready

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan (auto-approve session)

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Fix D-command stop-clause double-booking and make addStop overflow a recoverable ERR | — |
| 002 | Scope firmware motion-watchdog resets to keepalive and motion verbs only | 001 |
| 003 | Add VW-class velocity-refresh staleness cap independent of the keepalive | 002 |
| 004 | TestGUI KeyboardDriver: deadman STOP resend and focus-loss handling | — |
| 005 | Host keepalive armed only while a motion source is active | 004 |
| 006 | Gate OTOS fusion on warn-bit persistence with sim ABI support | — |

Tickets execute serially in the order listed. 001-003 are the firmware
stop-clause and watchdog cluster (002/003 sequenced after 001 for
file-locality and shared-block continuity, not a hard functional
dependency beyond 003-on-002). 004-005 are the TestGUI/host STOP-delivery
cluster (005 depends on 004's arm/disarm call sites). 006 (OTOS fusion gate)
is fully independent of 001-005 and is sequenced last only to keep the
critical-severity work (CR-01) and the CR-04/05 cluster first.
