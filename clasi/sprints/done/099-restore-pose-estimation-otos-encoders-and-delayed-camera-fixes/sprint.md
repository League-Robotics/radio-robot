---
id: 099
title: 'Restore pose estimation: OTOS, encoders, and delayed camera fixes'
status: done
branch: sprint/099-restore-pose-estimation-otos-encoders-and-delayed-camera-fixes
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
issues:
- restore-pose-estimation-otos-encoders-delayed-camera-fixes.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 099: Restore pose estimation: OTOS, encoders, and delayed camera fixes

## Goals

Reincorporate pose estimation cleanly: tick the OTOS with its state
committed to the Blackboard, publish motor position/velocity/acceleration
to the Blackboard, run a `PoseEstimator` subsystem fusing those observations
into `bb.fusedPose`, and add a new capability — a delayed camera-fix command
("at robot-time T you were at (x,y,theta)") incorporated via a small
pose-history buffer. Implements
`clasi/issues/restore-pose-estimation-otos-encoders-delayed-camera-fixes.md`
(step 1 of `clasi/issues/restore-goto-pursuit-with-pose-estimator.md`;
Planner/GOTO un-parking is steps 2-3, a follow-on sprint).

## CRITICAL DETAIL-PLANNING NOTE — read before detail-planning this sprint

This sprint is detail-planned and executed AFTER Protocol v3 (Sprints 095,
096, 097 — A, B, C) has landed. By then the command plane is BINARY and most
text verbs are retired. The issue's mechanics assume the pre-v3 text wire
surface (a text `FIX <x> <y> <h> <t>` verb, re-registering `poseCommands()`
in the text table, extending the snprintf `handleTlm`). At detail-planning
time these MUST be reconciled to the v3 binary surface: the FIX command
should almost certainly be a binary CommandEnvelope oneof arm (a `PoseFix`
message) dispatched through BinaryChannel rather than a text verb;
pose/OTOS TLM restoration should extend the binary Telemetry reply message,
not the retired snprintf handleTlm; SI/ZERO become binary arms. Keep the
design intent from the issue (history ring, transport-to-present compose,
tunable ekf_r_fix_*, ungated camera update, OTOS re-anchor via
otosSetPoseOut, D1 main-loop consolidation) but express it on the binary
plane.

## Problem

Sprints 093/094 gutted the runtime to Communicator -> Hardware -> Drivetrain;
the pose stack was parked, not deleted. The live loop never ticks the OTOS
or the `PoseEstimator`, so `bb.fusedPose`/`bb.encoderPose`/`bb.otos*` sit at
zero and GOTO/absolute-TURN are off the wire. The stakeholder wants pose
estimation reincorporated cleanly, with the main loop staying clean (pose
incorporation is 1-2 lines) and a new delayed-camera-fix capability added.

## Solution

Per the issue's design decisions D1-D9 (see the issue file for full detail
and rationale):

- **D1**: Consolidate `main.cpp` onto `Rt::MainLoop{hardware, drivetrain,
  poseEstimator}` — the main loop body becomes `loop.tick(bb, now)`. This
  reverses 094's explicit-inline-loop choice; approving this sprint approves
  the reversal.
- **D2**: OTOS tick is a scheduled slot inside `NezhaHardware::tick()` —
  fires only when no 0x46 REQUEST is in flight and the 20 ms read is due
  (new `OtosOdometer::readDue(now)`), structurally ruling out 0x17 traffic
  inside the 0x10 REQUEST/COLLECT window (the bus-hang class).
- **D3**: Per-motor acceleration computed in the `Hal::Motor` base
  (`trackAcceleration(velocity, dtUs)`, EMA alpha=0.25), per project policy
  of generic policy in the base, hardware primitives in leaves.
- **D4**: Fusion gating = bounded innovation-consistency gate in `EkfTiny`
  (chi-square position, sigma-bound heading) plus rejection-streak
  P-inflation recovery. Design is locked only after ticket 002's bench
  session confirms the frozen-pose hazard hypothesis via `otosconn=`.
- **D5**: Camera fix = ring buffer of timestamped `encoderPose` + transport-
  to-present compose + tunable EKF update with new configurable noise
  `ekf_r_fix_xy`/`ekf_r_fix_theta` (zero-as-unset sentinel, matching the
  existing four). Ungated, because the camera is an authoritative absolute
  source — the innovation gate exists to reject the OTOS, not the camera.
- **D6**: Fix timestamp is `t=` robot-clock ms (host maps clocks via PING
  `t=` + RTT/2), not `age=`.
- **D7**: Fix arrives via a new `Rt::Mailbox<Rt::PoseFixCommand> poseFixIn`
  on the Blackboard; the history ring lives private inside `PoseEstimator`.
- **D8**: After applying a fix (and SI), `PoseEstimator` posts the new fused
  pose to `otosSetPoseOut` (the existing `bb.otosSetPoseIn` mailbox);
  `MainLoop` drains it into `hardware.odometer()->setPose()`, keeping the
  OTOS chip's frame agreed with the new anchor.
- **D9 (subject to the binary-plane reconciliation above)**: TLM
  restoration extends the live telemetry surface with `pose=`, `encpose=`,
  `otos=`, `otosconn=` — on the binary plane, this means the Telemetry reply
  message, not the retired snprintf `handleTlm`.

Ticket slicing follows the issue's ~8-ticket sequence (main.cpp
consolidation; OTOS re-tick; per-motor acceleration; encoder-only
PoseEstimator wiring; EkfTiny innovation gate; enable OTOS fusion; camera-fix
mechanism; aprilcam end-to-end bench script) — exact ticket numbers and
dependencies are finalized at detail-planning time, reconciled to whatever
the binary command plane looks like once Sprints 095-097 have landed.

## Success Criteria

- OTOS ticks live with state committed to the Blackboard (`bb.otos`,
  `bb.otosValid`, `bb.otosConnected`, `bb.otosPresent`), verified via a
  sustained (>=10 min) no-hang bench session with 0x17+0x10 bus coexistence.
- Motors publish position/velocity/acceleration to the Blackboard.
- `PoseEstimator` runs live, producing both `encoderPose` (dead reckoning)
  and `fusedPose` (EKF), each observable on the stand.
- The innovation gate closes the frozen-fused-pose hazard: `fusedPose` does
  not freeze or drag to origin when OTOS is static but the robot is moving
  (or vice versa).
- A delayed camera-fix command is accepted, and `fusedPose` converges by the
  correctly-composed amount after a `FIX` with a known offset and a captured
  robot timestamp; a stale-timestamp fix produces no jump.
- The main loop body stays clean per the issue's own bar (pose incorporation
  is 1-2 lines via `Rt::MainLoop::tick()`).
- An aprilcam end-to-end bench/playfield script demonstrates the full path:
  PING clock-sync, tag-pose-to-FIX send, convergence check.

## Scope

### In Scope

- `Rt::MainLoop` consolidation (D1) across `main.cpp` and the sim harness
  (`tests/_infra/sim/sim_api.cpp`), sharing one tick body.
- OTOS re-tick with `readDue()` bus-scheduling (D2), TLM/telemetry
  restoration of `otos=`/`otosconn=` on whatever plane is live at
  detail-planning time (see the critical note above).
- Per-motor acceleration in the `Hal::Motor` base (D3), new
  `MotorState.acceleration` proto field.
- `PoseEstimator` wired encoder-only first, then OTOS-fused after the gate
  lands (D4, D6 in the issue's ticket table).
- `EkfTiny` innovation gate + rejection-streak/P-inflation recovery (D4),
  design locked only after live bench evidence.
- Camera-fix mechanism: `PoseFixCommand`, `poseFixIn` mailbox, pose-history
  ring, transport-to-present compose, tunable-R EKF update, `ekf_r_fix_*`
  config, the FIX command surface (binary arm — see critical note), OTOS
  re-anchor post via `otosSetPoseOut` (D5, D7, D8).
- Re-registering SI/ZERO (`pose_commands.cpp` equivalent) on the surface
  that is live at detail-planning time.
- aprilcam end-to-end bench/playfield script.
- Sim/unit test harness extensions: `ekf_tiny_harness.cpp`,
  `pose_estimator_harness.cpp`, `motor_policy_harness.cpp`,
  `nezha_flipflop_harness.cpp`, and wire-level sim tests adapted from
  `tests/sim/parked-093/unit/`.

### Out of Scope

- Planner/GOTO un-parking and absolute-TURN revival — step 2-3 of
  `restore-goto-pursuit-with-pose-estimator.md`, a follow-on sprint that
  consumes `bb.fusedPose` once this sprint restores it.
- Full `robot_radio`/TestGUI integration of the FIX capability — the issue
  scopes this sprint to firmware verb/arm + a bench script only.
- Any text-plane wire work — by the time this sprint executes, text verbs
  other than the protocol-v3 rump (PING/ID/HELLO/HELP/STOP) no longer exist;
  do not re-introduce a text `FIX`/`SI`/`ZERO` verb.
- The REG_OFFSET write/readback + spin retest — opportunistic during ticket
  002's bench session per the issue, non-blocking, not a sprint acceptance
  item.
- Full EKF replay on camera fix — explicitly rejected in the issue as
  RAM-infeasible (~2 KB free); the transport-to-present compose approach is
  the shipped design, not a placeholder for replay.

## Test Strategy

Sim/unit harnesses extend the existing pattern: `ekf_tiny_harness.cpp`
(gate accept/reject/streak-recovery + threshold characterization),
`pose_estimator_harness.cpp` (fix-replay compose math vs. hand oracles,
interpolation, stale-fix reject, future-timestamp clamp, SI-clears-ring,
consecutive fixes, `otosSetPoseOut` posted), `motor_policy_harness.cpp`
(acceleration EMA), `nezha_flipflop_harness.cpp` (OTOS slot never fires
during COLLECT_DUE, at most one slot per 20 ms). Wire-level sim tests adapt
from `tests/sim/parked-093/unit/` (renamed/updated for the live loop and
whatever command plane is active) plus a new end-to-end fix test: drive
sim, send a fix with a known offset at a captured robot time, assert the
fused pose converges by the composed amount while `encoderPose` stays
untouched; a stale-timestamp fix produces no jump. Bench (standing gate,
robot on the stand, per `.claude/rules/hardware-bench-testing.md`): OTOS
re-tick ticket requires a mandatory sustained no-hang session with
0x17+0x10 coexistence; the encoder-only PoseEstimator ticket requires
`encpose=`-equivalent tracking verification and SI re-anchor; the
fusion-enable ticket requires confirming `fusedPose` does not freeze/drag
to origin on the stand; the camera-fix ticket requires a FIX-accepted +
convergence smoke test; the final ticket requires the aprilcam end-to-end
playfield run. Driven via `robot_radio` `NezhaProtocol` (never lock-step
pyserial, per prior bench-session lessons).

## Architecture Notes

This sprint's design intent (history ring, transport-to-present compose,
tunable `ekf_r_fix_*`, ungated camera update, OTOS re-anchor via
`otosSetPoseOut`, D1 main-loop consolidation) is locked at the issue level,
but its wire expression is NOT — see the critical detail-planning note
above. At detail-planning time, read the landed state of Sprints 095-097
(the actual `CommandEnvelope`/`ReplyEnvelope` shape, which oneof arms exist,
whether Telemetry carries pose fields yet) before finalizing the FIX/SI/ZERO
command surface and the TLM restoration mechanism. D1 reverses 094's
explicit-inline-loop decision — flagged in the issue as a decision this
sprint's approval also approves. RAM: the pose-history ring is
16 B x 24 entries = 384 B (~1.2 s depth, ~2.4x the ~0.5 s camera latency);
check the map file, but remember the project rule that ~98% reported RAM is
normal by design and not itself a risk signal — only flash overflow is a
real budget.

## GitHub Issues

(None — tracked via the CLASI issue file referenced above.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Consolidate main.cpp onto Rt::MainLoop (structural only) | — |
| 002 | OTOS ticks live: readDue() scheduled slot + Blackboard commit | 001 |
| 003 | Per-motor acceleration: Hal::Motor base policy + MotorState.acceleration | — |
| 004 | PoseEstimator wired encoder-only + BodyState/PoseStepped + binary PoseFix arm (SI/ZERO) | 001 |
| 005 | Encoder-delta timestamp fix for PoseEstimator's joint predict | 004 |
| 006 | EkfTiny innovation gate + rejection-streak P-inflation recovery | 002, 004 |
| 007 | Enable OTOS fusion in PoseEstimator (bench-gated hazard close) | 002, 004, 005, 006 |
| 008 | Camera-fix mechanism: history ring, transport-compose, ungated EKF update | 004, 006 |
| 009 | aprilcam end-to-end bench/playfield script: PING sync, FIX send, convergence | 007, 008 |

Tickets execute serially in the order listed. 003 has no real dependency
on 001/002 (per-motor acceleration is independent of the main-loop/OTOS
work) and could run in any position before 006, but is sequenced here for
narrative continuity with the ticket table above.
