---
status: in-progress
sprint: 099
tickets:
- 099-001
- 099-002
- 099-003
- 099-004
- 099-005
- 099-006
- 099-007
- 099-008
- 099-009
---

# Restore Pose Estimation (OTOS + encoders + delayed camera fixes)

## Context

Sprints 093/094 gutted the runtime to Communicator → Hardware → Drivetrain; the pose stack
was parked, not deleted. The live loop ([source/main.cpp:129-155](source/main.cpp#L129-L155))
never ticks the OTOS or the PoseEstimator, so `bb.fusedPose`/`bb.encoderPose`/`bb.otos*` sit
at zero and GOTO/absolute-TURN are off the wire. Eric wants pose estimation reincorporated
cleanly: the OTOS ticked with its state committed to the Blackboard, motors publishing
position/velocity/acceleration to the Blackboard, a PoseEstimator subsystem fusing those
observations into `bb.fusedPose`, and a NEW capability — a delayed camera-fix command
("at robot-time T you were at (x,y,θ)") incorporated via a small pose-history buffer.
The main loop must stay clean: pose incorporation is 1–2 lines.

Delivers step 1 of [clasi/issues/restore-goto-pursuit-with-pose-estimator.md](clasi/issues/restore-goto-pursuit-with-pose-estimator.md)
(Planner/GOTO un-parking = steps 2–3, follow-on sprint). Folds in both `later/` issues:
the frozen-fused-pose hazard (bench-confirm via `otosconn=` BEFORE locking EKF gating)
and notes the REG_OFFSET retest as opportunistic, non-blocking.

## Stakeholder decisions (2026-07-09, confirmed via Q&A)

1. **Scope: pose estimation only.** Planner/GOTO is the next sprint, consuming `bb.fusedPose`.
2. **Camera fix = tunable Kalman update**, not a hard re-anchor: the transported fix enters
   the EKF as a measurement with its own configurable noise R (tiny R ≈ hard re-anchor).
3. **Host side: firmware verb + a bench script** (aprilcam → FIX over the link). Full
   robot_radio/TestGUI integration deferred.

## What already exists (re-wire, don't rewrite)

- `Subsystems::PoseEstimator` ([source/subsystems/pose_estimator.h](source/subsystems/pose_estimator.h)) —
  intact + unit-tested. `tick(now, leftObs, rightObs, otosObs*, poseResetIn)`; produces
  `encoderPose()` (dead reckoning, EKF never writes it) and `fusedPose()` (EkfTiny,
  [source/estimation/ekf_tiny.h](source/estimation/ekf_tiny.h)). Never ticked live.
- `Hal::OtosOdometer` ([source/hal/otos/otos_odometer.h](source/hal/otos/otos_odometer.h)) —
  owned by `NezhaHardware`, `begin()` runs at boot, `tick()`/`pose()` never called. 20 ms
  internal rate limit, lever-arm folded in (092-004 FOLD stands), `connected()`/`fusableThisPass()` ready.
- `Rt::Blackboard` ([source/runtime/blackboard.h](source/runtime/blackboard.h)) — already declares
  `encoderPose`, `fusedPose`, `otos`, `otosValid`, `otosConnected`, `otosPresent` and the
  `poseResetIn`/`otosSetPoseIn`/`otosCommandIn` queues. Only the camera-fix mailbox is new.
- Motor velocity — the Nezha brick exposes position only (0x46, tenths-deg); `NezhaMotor::tick()`
  already differences successive positions over a µs-clock dt with EMA + glitch gates, and
  `bb.motors[]` gets `{connected, position, velocity, applied, …, sampled_at}` every pass.
  Requirement "motors publish state" is already met except **acceleration**.
- `pose_commands.cpp` (SI/ZERO) exists, compiled, unregistered — re-register + extend.

## Design decisions

| # | Decision | Why |
|---|----------|-----|
| D1 | **Consolidate main.cpp onto `Rt::MainLoop{hardware, drivetrain, poseEstimator}`** — main loop body becomes `loop.tick(bb, now)` | Commit step grows ~5 cells; two hand-mirrored tick bodies (main.cpp + sim harness) is a known dual-site hazard; only way to literally satisfy "1–2 lines". ⚠ Reverses 094's explicit-inline choice — approving this plan approves the reversal. |
| D2 | **OTOS tick is a scheduled slot inside `NezhaHardware::tick()`** — fires only when no 0x46 REQUEST is in flight and the 20 ms read is due (new const query `OtosOdometer::readDue(now)`) | Structurally rules out 0x17 traffic inside the 0x10 REQUEST/COLLECT window (bus-hang class); one stolen motor slot per 20 ms is negligible; SimHardware already ticks its odometer internally, so sim/hw converge. |
| D3 | **Per-motor acceleration computed in the `Hal::Motor` base** (`trackAcceleration(velocity, dtUs)`, EMA α=0.25 matching Drivetrain's proven value), leaves call it after updating filtered velocity; new `MotorState.acceleration` proto field | Project policy: generic policy in the base, hardware primitives in leaves. Drivetrain's own `acc=` EMA stays untouched. |
| D4 | **Fusion gating = bounded innovation-consistency gate in EkfTiny** (chi-square position, sigma-bound heading) + rejection-streak P-inflation recovery (prior art: `source_old/state/EKFTiny`); design LOCKED only after the ticket-2 bench session confirms the frozen-pose hypothesis via `otosconn=` | The gate makes the on-stand case behave correctly by itself (static OTOS vs moving prediction → rejected → fusedPose tracks encoders); streak recovery prevents permanent OTOS lockout. Mandated sequencing from the `later/` hazard issue. |
| D5 | **Camera fix = ring buffer of timestamped encoderPose + transport-to-present + tunable EKF update.** Compute `impliedNow = fix ⊕ (encNow ⊖ enc(T))` (rigid-body compose), then apply as an **ungated** EKF position+heading update with new configurable noise `ekf_r_fix_xy`/`ekf_r_fix_theta` (zero-as-unset sentinel like the existing four) | encoderPose is smooth/continuous and never written by EKF/OTOS/fix, so the delta is clean and consecutive fixes compose without buffer invalidation. Tunable R honors decision 2 (tiny R ≈ re-anchor). Ungated because the camera is an authoritative absolute source — the innovation gate exists to reject the OTOS, not the camera. Full EKF replay rejected: RAM-infeasible (~2 KB free). |
| D6 | **Fix timestamp = `t=` robot-clock ms** (host maps clocks via `PING` `t=` + RTT/2), not `age=` | `age=` bakes transport latency/jitter (DAPLink batching, radio) into T at receipt; robot-ms is the established wire currency. |
| D7 | **Fix arrives via new `Rt::Mailbox<Rt::PoseFixCommand> poseFixIn`** on the Blackboard; history ring lives private inside PoseEstimator | Latest-wins is right (newer camera frame supersedes); subsystems own their estimator machinery, handlers only post to queues. |
| D8 | **After applying a fix (and SI), PoseEstimator posts the new fused pose to `otosSetPoseOut`** (the existing `bb.otosSetPoseIn` mailbox, passed as an outbox param); MainLoop drains it into `hardware.odometer()->setPose()` | Keeps the OTOS chip's frame agreeing with the new anchor — otherwise the innovation gate would correctly-and-forever reject the now-offset OTOS (old host-side `otos.align_to` did the same job). |
| D9 | **TLM restoration extends the live `handleTlm`** (`pose=`, `encpose=`, `otos=`, `otosconn=`, integer mm/centideg — no `%f` on newlib-nano), not a STREAM/SNAP revival | `handleTlm` is the live pull surface; widen its `body`/`rbuf` buffers accordingly. |

## Key mechanics

**Loop shape after D1** ([source/main.cpp](source/main.cpp), [source/runtime/main_loop.cpp](source/runtime/main_loop.cpp)):
`MainLoop::tick(bb, now)` = `hardware_.tick(now)` → `drivetrain_.tick(now, bb.segmentIn, bb.replaceIn, bb.driveIn)` →
assemble observations (single sanctioned `fusableThisPass()` call; `odometer->pose()` sample) →
`poseEstimator_.tick(now, motorState(L), motorState(R), otosFusable ? &sample : nullptr, bb.poseResetIn, bb.poseFixIn, bb.otosSetPoseIn)` →
drain `bb.otosSetPoseIn` → odometer → commit (`bb.motors/drivetrain/encoderPose/fusedPose/otos/otosValid/otosConnected/loopNow`).
Sim harness (`tests/_infra/sim/sim_api.cpp`) gains the PoseEstimator member and shares the identical body.
`bb.otosPresent` seeded once at boot.

**History ring + fix replay** (new, in [source/subsystems/pose_estimator.{h,cpp}](source/subsystems/pose_estimator.cpp)):
`PoseHistoryEntry{t, x, y, theta}` — 16 B × 24 entries recorded every 50 ms = 384 B, 1.2 s depth
(≈2.4× the ~0.5 s camera latency; RAM-safe). Tick additions: step 5 drains `poseFixIn` →
`applyPoseFix()`; step 6 records `{now, encX_, encY_, encTheta_}` on the 50 ms cadence.
`applyPoseFix()`: resolve enc(T) with linear x/y + wrapped-angle interpolation (between entries, or
newest-entry→now); reject fixes older than the ring (`fixDropped_` counter); clamp future T to now;
rigid-body compose `impliedNow = fix ⊕ (encNow ⊖ enc(T))`; EKF update with `ekf_r_fix_*`;
post the resulting belief to `otosSetPoseOut` (D8). SI (`kSetPose`) clears the ring (delta
continuity broken); `ZERO enc` does not.

**Wire verb**: `FIX <x> <y> <h> <t>` — mm, mm, centideg, robot-ms (matches SI's units), registered
in a re-registered `poseCommands()` family ([source/commands/pose_commands.cpp](source/commands/pose_commands.cpp))
added to `buildTable()` ([source/runtime/command_router.cpp](source/runtime/command_router.cpp)).
Handler converts centideg→rad, posts `PoseFixCommand` (new POD in
[source/runtime/commands.h](source/runtime/commands.h)) to `bb.poseFixIn`, replies `OK fix …`.

## Ticket slicing (sprint ~095, sequenced)

| ID | Title | Depends | Gate |
|----|-------|---------|------|
| 001 | Consolidate main.cpp onto Rt::MainLoop (no behavior change) | — | sim green; bench smoke (S/MOVE/TLM unchanged) |
| 002 | OTOS ticks again: `readDue()`, sequencer slot, commit `bb.otos/otosValid/otosConnected/otosPresent`, TLM `otos=`/`otosconn=` | 001 | **BENCH MANDATORY**: `otosconn=` hazard evidence + sustained no-hang session (≥10 min 0x17+0x10 coexistence) |
| 003 | Per-motor acceleration: proto field, `Motor::trackAcceleration()` base policy, NezhaMotor/SimMotor call sites | — (parallel) | sim; bench light (plausible values on a ramp) |
| 004 | PoseEstimator wired **encoder-only** (`otosObs = nullptr`), commit `encoderPose/fusedPose`, TLM `pose=`/`encpose=`, re-register SI/ZERO, drain `otosSetPoseIn` | 001 | **BENCH**: `encpose=` tracks stand motion; SI re-anchors |
| 005 | EkfTiny innovation gate + streak/P-inflation (design locked by 002's bench evidence); thresholds characterized in harness, never freehand | 002 | sim only |
| 006 | Enable OTOS fusion (one-token flip: `nullptr` → `otosFusable ? &sample : nullptr`) | 002,004,005 | **BENCH MANDATORY**: `pose=` does not freeze/drag to origin on stand (hazard closed) |
| 007 | Camera-fix mechanism: `PoseFixCommand`, `poseFixIn`, history ring, transport+tunable-R update, `ekf_r_fix_*` config, `FIX` verb, OTOS re-anchor post | 004 (006 for full effect) | sim; bench smoke (`FIX` accepted, `pose=` converges) |
| 008 | aprilcam end-to-end bench script (`tests/bench/` or `tests/playfield/`): PING clock-sync helper, tag-pose→FIX sender, convergence check | 006,007 | **BENCH/PLAYFIELD MANDATORY** |

The ungated-fusion hazard never ships: fusion stays off (nullptr) until the gate exists.

## Test strategy

- **Harnesses** (`tests/sim/unit/`, existing pattern): extend `ekf_tiny_harness.cpp` (gate accept/
  reject/streak-recovery + threshold characterization), `pose_estimator_harness.cpp` (fix-replay
  compose math vs hand oracles, interpolation, stale-fix reject, future clamp, SI-clears-ring,
  consecutive fixes, `otosSetPoseOut` posted), `motor_policy_harness.cpp` (acceleration EMA),
  `nezha_flipflop_harness.cpp` (OTOS slot never fires during COLLECT_DUE; ≤1 slot per 20 ms).
- **Wire-level sim**: adapt from `tests/sim/parked-093/unit/` — `test_dev_loop_pose_estimator.py`
  → `test_main_loop_pose_estimator.py`, `test_pose_estimate_tolerance.py`,
  `test_errored_observation.py`, `test_otos_divergence.py` (SNAP→TLM). New
  `test_pose_fix_end_to_end.py`: drive sim, send FIX with known offset at a captured robot time,
  assert `pose=` converges by the composed amount while `encpose=` is untouched; stale-T fix → no jump.
- **Bench** (standing gate, robot on stand): the four bench rows in the ticket table, run with
  `just build-clean` + `mbdeploy deploy <UID> --hex MICROBIT.hex`, driven via robot_radio
  NezhaProtocol (never lock-step pyserial). Ticket 002's session may opportunistically run the
  REG_OFFSET write/readback + spin retest (`later/otos-reg-offset-bench-retest-deferred.md`) — optional, non-blocking.

## Risks / watch items

- D1 reverses 094's inline-loop decision — approved with this plan.
- TLM line length grows: widen `body`/`rbuf` in `handleTlm`, verify the radio reply path with the longer line.
- RAM: +384 B ring + bb growth — check the map file; ~98% reported RAM is normal, only flash overflow is a real budget.
- `FIX` t parse: uint32 robot-ms (int32 parse bound ≈ 24.8 days uptime — document, acceptable).

## Execution path (CLASI)

This is a sprint, not a direct edit. On approval: dispatch **sprint-planner** with this plan as the
architecture direction (create_sprint/create_ticket are planner-gated), record the stakeholder-approval
gate, cut the 8 tickets above, then execute serially with programmer dispatches. Sprint completion
updates `clasi/issues/restore-goto-pursuit-with-pose-estimator.md` (step 1 done; steps 2–3 = next
sprint) and closes/updates the two `later/` issues per bench outcomes.
