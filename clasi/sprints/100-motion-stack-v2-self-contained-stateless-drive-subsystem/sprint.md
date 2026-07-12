---
id: '100'
title: 'Motion stack v2: self-contained stateless drive subsystem'
status: planning-docs
branch: sprint/100-motion-stack-v2-self-contained-stateless-drive-subsystem
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
- SUC-011
- SUC-012
- SUC-013
- SUC-014
- SUC-015
issues:
- motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 100: Motion stack v2: self-contained stateless drive subsystem

## Goals

Replace the motion pipeline (`Motion::SegmentExecutor`'s 3-phase machine +
divergence replans + dead-time-projected stops) with a self-contained,
stateless motion-control subsystem living entirely in `source/drive/`
(namespace `Drive`), per the stakeholder-approved design in
`clasi/issues/motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md`
(approved 2026-07-12, plan-mode approval + explicit "plan sprints and
execute them" directive — the issue IS the stakeholder-approved plan; see
`architecture-update.md` for the gate record). This sprint is Sprint B in
the issue's own "Sprint packaging & sequencing" section; Sprint A (099,
pose restore) is a separate, already-planned sprint that must be EXECUTED
AND CLOSED before this sprint's execution begins (see Scope/Dependencies
below).

The issue is the source of truth for the design: the layer architecture
(Level 1 planner/tracker in `source/drive/`, Level 2 unchanged wheel
velocity PID at the HAL leaf), the two core header sketches
(`Drive::Drivetrain`, `Drive::MotionPlan`), the control laws with initial
gains/envelopes, the wire schema evolution, the config surface, the wafer
adapter spec, the four-tier test ladder, and the cutover strategy. This
sprint transcribes that design into CLASI tickets; it does not redesign it.

## Problem

The current motion pipeline fails audit against the canonical control
model: feedback is implemented as replanning tuned by ~12 gain-dependent
constants; translation terminates on prediction, not state; the plan is
knowingly infeasible (`v_body_max`=1000 vs ~400 sim / ~620-740 real
plateau); segments carry no pose and no boundary velocities; the
`PoseEstimator` is constructed but never ticked (until sprint 099 restores
that). The one true outer loop ever added (sprint 098's heading PD,
`kp`=6) is the only motion that lands reliably (±1° turns on hardware) —
this design generalizes that pattern to translation and arcs, and puts a
name on it: two explicit control levels, a stateless Level-1 planner/
tracker over an unchanged, bench-proven Level-2 wheel velocity PID.

## Solution

Per the issue's architecture (transcribed, not redesigned — see
`architecture-update.md` for the full 7-step writeup):

- **`source/drive/` (namespace `Drive`)**: a self-contained subsystem with
  no references outside itself except libc/libm and the vendored Ruckig
  headers. It defines its own plain value types (`Pose`, `Twist`,
  `WheelState`, `WheelVelocities`, `Limits`), copies in the differential
  IK/saturation math (from `kinematics/body_kinematics`) and the Ruckig
  wrapper pattern (from `motion/jerk_trajectory`, generalized to
  `solveToExit` with a nonzero target velocity).
- **Stateless except one explicit residue**: `Drive::Drivetrain` (the
  façade: `plan()`, `replan()`, `admit()`, `planVelocity()`) and
  `Drive::MotionPlan` (the immutable solved segment: `referenceAt()`,
  `step()`) are pure functions of their arguments. The caller
  (`Subsystems::Drivetrain`'s thin wafer adapter) owns the pose estimate,
  the clock, the plan value, and a caller-owned `StepState` blob (five
  policy-timer scalars — the one irreducibly historical thing: dwell/
  sustain/replan timers). The wheel-PI integrator (the other genuinely
  stateful control element) stays exactly where it is today, unchanged, at
  Level 2 (`Hal::MotorVelocityPid`).
- **Two levels of control**: Level 1 (`source/drive/`) plans and tracks,
  emitting wheel velocity setpoints; Level 2 (unchanged HAL leaf PID)
  turns setpoints into duty. The boundary is thin, observable, and already
  instrumented (`cmd_vel` vs `vel` on TLM).
- **The wafer adapter** (`Subsystems::Drivetrain`, thinned): the ONLY
  bridge between `source/drive/` and the rest of the firmware — queues,
  blackboard, HAL staging, wire acks, type conversion at the boundary
  (`bb.bodyState` → `Drive::BodyState`, `bb.poseStepped` →
  `StepInput.poseStep`/`poseStepTheta` — both landed by sprint 099). Zero
  control math in the adapter.
- **Wire schema evolution**: `motion.proto` `MotionSegment` grows
  `arc_length`/`delta_heading`/`exit_speed`/`primitive` fields;
  `PlannerConfig` grows fields 15-31 (`Drive::Limits` source); new
  `MotionTrace` (`ReplyEnvelope` arm 11) and `PlanRecord` (arm 10) reply
  arms, `PlanDumpRequest` (`CommandEnvelope` arm 18); `EventNotify` gets a
  real body. The host proxy (`legacy_translate.py`) decomposes legacy
  `MOVE` into ≤3 primitives via a new `primitives_for_move()`, plus a new
  `SEG` proxy verb for real arcs.
- **MOVER (deadman-velocity teleop) cuts over inside this sprint**; BLEND
  streaming merge (`MOVE s=1`) is deferred — this sprint replies `ERR` to
  `stream=true` until a follow-up sprint.
- **Four-tier test ladder**: tier 0 (pure Python against a compiled
  `libdrive_host`, ctypes ABI, mirroring the proven `sim_api.cpp`/
  `firmware.py` pattern) is the primary correctness/interpretability
  instrument; tier 1 (firmware sim through the adapter) validates
  integration; tier 2 (bench, robot on the stand) validates the real
  plant; tier 3 (playfield, camera-verified) validates world-frame
  anchoring. The same compiled control code runs at every tier — a
  failure at tier N replays at tier N-1 from the recorded `TrackRecord`.
- **Hard cutover, no dual stack**: `source/drive/` lands and is fully
  tier-0/tier-1 tested WITHOUT entering the live firmware call path — the
  robot stays drivable on the old stack until one atomic cutover ticket
  swaps the adapter, wire admission, host proxy, and build-list
  (`segment_executor`/`stop_condition` parked on disk, not deleted, until
  bench sign-off).

## Success Criteria

- `source/drive/` compiles standalone (host `libdrive_host` + firmware,
  once wired) with NO includes outside itself except libc/libm and
  vendored Ruckig — enforced by a grep test (ticket 002).
- Tier-0 Python suite: plan-table dumps/plots before anything moves,
  purity/property tests (same inputs → identical output), closed-loop
  convergence against a Python plant model of the level-2 servo (lag,
  stiction, staleness, slip), replay harness for any recorded
  `TrackRecord.in`.
- Tier-1 sim: the wafer adapter, queue precedence, and loop order pass
  under the sim's fault-knob matrix (`motor_lag`, `enc_slip`, `stiction`,
  `trackwidth` error) with lag ON (120-140ms) for tracker/replan
  validation — golden-TLM bit-exactness is preserved on the zero-error
  path, with regeneration as an explicit reviewed cutover step.
- HITL: after cutover, the robot on the stand exercises arcs, pivots,
  chains, and MOVER teleop over the real wire, with `cmd_vel`/`vel`
  telemetry showing the Level-1/Level-2 boundary working as designed; the
  098 pivot acceptance grid (±1°) is re-run and still passes.
- Field: a camera-verified multi-segment chain with live `PoseFix`
  corrections closes the full camera → EKF → tracker loop (tier 3).
- No `kOutputHops`/`kDeadTime`/divergence-constant-family reversal-write
  hazard exists anywhere in the new subsystem (structural: one-sided wheel
  clamp on forward arcs; reverse = new segment after stop + dwell).
- `ReplyEnvelope`/`CommandEnvelope` `kMaxEncodedSize` stays ≤186B for every
  arm after this sprint's schema growth (verified, not assumed — see
  Ticket 001).

## Scope

### In Scope

- `source/drive/` core: `types.h`, `arc_math.{h,cpp}`,
  `master_profile.{h,cpp}` (Ruckig wrapper + `solveToExit`),
  `tracker.{h,cpp}`, `policy.{h,cpp}`, `drivetrain.{h,cpp}`,
  `motion_plan.{h,cpp}` — per the issue's two core header sketches
  verbatim, elaborated to the rest of the directory's supporting files.
- Wire schema growth: `motion.proto`, `planner.proto` (fields 15-31),
  `envelope.proto` (`MotionTrace`, `PlanRecord`, `PlanDumpRequest`),
  `EventNotify`'s real body; `tovez.json` + `gen_boot_config.py` +
  `check_config_sync.py` updates; `PlannerConfigPatch` growth with a
  budget check (split the patch if it busts 186B).
- Tier-0 ctypes ABI (`tests/_infra/drive/drive_api.cpp`) + Python test
  suite (plant model, closed-loop, replay harness, plan-table notebooks).
- The cutover: wafer adapter rewrite (`Subsystems::Drivetrain`), wire
  admission, host proxy decomposition (`legacy_translate.py`
  `primitives_for_move()`, new `SEG` verb), build-list swap.
- MOVER velocity-mode cutover (`planVelocity` + adapter `replaceIn` path).
- Tier-1 fault-knob matrix + lag-on validation; bench arc/pivot/chain
  grids + plateau re-measure + 098 pivot grid re-run; field camera-verified
  chain with live `PoseFix`.
- Cleanup: delete parked files after bench sign-off, reserve retired proto
  fields, retire `heading_kp`/`heading_kd` + `governRatio` segment path.
- Consuming sprint 099's landed shapes: `bb.bodyState` (`msg::PoseEstimate`
  — fused pose + body twist), `bb.poseStepped`/
  `PoseEstimator::lastPoseStep()` (`PoseStep{pos, theta}`, the magnitude of
  any correction applied on the prior tick), and the `PoseFix` envelope arm
  7 (retyped by 099). The wafer adapter converts `bb.bodyState` →
  `Drive::BodyState` and `bb.poseStepped` → `StepInput.poseStep`/
  `poseStepTheta`.

### Out of Scope

- BLEND streaming merge (`MOVE s=1`) — replies `ERR` to `stream=true` until
  a follow-up sprint (stakeholder decision, issue's Context section).
- Sprint 099's own scope (pose restore, OTOS/encoder ticking, the
  `PoseFix` delayed-camera-fix mechanism, `EkfTiny`'s innovation gate) —
  that sprint plans and executes independently, ahead of this one; this
  sprint only CONSUMES its landed shapes.
- Any change to the Level-2 wheel velocity PID (`Hal::MotorVelocityPid` in
  `NezhaMotor`/`SimMotor`) — explicitly kept outside the subsystem per the
  issue's "Two levels of control" decision.
- Full EKF/pose-estimation redesign — pose ownership is entirely outside
  `source/drive/`.

### Dependencies (execution-order precondition, not a ticket dependency)

**Sprint 099 must be EXECUTED AND CLOSED before this sprint's execution
begins.** Sprint 099 is fully planned (9 tickets — see
`clasi/sprints/099-restore-pose-estimation-otos-encoders-and-delayed-camera-fixes/architecture-update.md`)
but not yet executed as of this sprint's planning. Ticket 006 (THE
CUTOVER) below expresses this as an explicit precondition ("sprint 099
closed") rather than a `depends-on` ticket ID, since 099 is a different
sprint, not a ticket in this one. Every other ticket in this sprint (001-
005) is 099-independent (pure host/tier-0 work against `source/drive/` in
isolation) and may be planned, reviewed, and even executed ahead of 099's
closure — only the cutover and everything downstream of it needs 099's
landed shapes for real.

### HITL / hardware note

As of this sprint's planning, the ROBOT is not USB-attached (only the
radio relay dongle is connected; the robot is radio-reachable but not
flashable). Tickets 006, 007, 010, and 011 (HITL-gated) each carry an
explicit precondition that the robot must be moved to USB before their
flashing/bench steps can execute — see each ticket's Preconditions. The
execution flow front-loads every host-side (tier 0/1) ticket first so the
USB-attach window is needed for the smallest possible set of sessions.

## Test Strategy

Four-tier ladder per the issue (see Solution above): tier 0 (pure Python,
no firmware) is the primary, cheapest, most complete correctness
instrument and runs on every ticket that touches `source/drive/`; tier 1
(firmware sim through the adapter) is the integration/adapter gate, run
with the sim's `motor_lag` knob ON (120-140ms) for tracker/replan
validation (the zero-lag path is reserved for golden-TLM bit-exactness
only — validating tracker accuracy at zero lag would repeat the
2026-07-11 false-green failure); tier 2 (bench, robot on the stand, per
`.claude/rules/hardware-bench-testing.md`) is the real-plant acceptance
gate; tier 3 (playfield, camera-verified) is the world-frame closed-loop
gate. Unit harnesses (`tests/sim/unit` C++ pattern) are kept minimal since
tier 0 covers behavior directly: arc math round-trips, `solveToExit`
boundary tuples, the `v_eff` fold invariant, admission verdict table, and
the grep test for `source/drive/`'s isolation (`kOutputHops`/`kDeadTime`/
`msg::`/`MicroBit` absent).

## Architecture Notes

See `architecture-update.md` for the full 7-step writeup. Load-bearing
rules that must not be watered down in any ticket: (1) statelessness —
`Drive::Drivetrain`/`Drive::MotionPlan` are pure functions; the only
mutable state is the caller-owned `StepState` (five policy-timer
scalars); (2) the two-levels decision — the wheel PID stays at the
motors, unchanged; (3) no dead-time compensation, no `kOutputHops`/
`kDeadTime` anywhere in `source/drive/`, enforced by a grep test; (4) no
reversal write-trains structurally (one-sided wheel clamp on forward
arcs; reverse is a new segment after stop + dwell); (5) `source/drive/`
has zero includes outside itself except libc/libm and vendored Ruckig.
Stakeholder approval basis: the issue file IS the stakeholder-approved
plan (approved 2026-07-12, plan-mode approval + explicit "plan sprints
and execute them" directive) — recorded as the `stakeholder_approval`
gate with that note (see `architecture-update.md`'s gate record).

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
| 001 | Wire/config schema: motion, planner, envelope, config protos + tovez.json generators | — |
| 002 | source/drive/ core: types, arc_math, master_profile + isolation grep test | 001 |
| 003 | Drive::Drivetrain facade: plan/admit/replan/planVelocity + MotionPlan query surface | 002 |
| 004 | Tracker + IK/saturate/clamp cascade | 003 |
| 005 | Policy: envelopes + terminal machine + MotionPlan::step() composition | 004 |
| 006 | Tier-0 ctypes ABI + Python test suite (plant model, closed loop, replay, plan-table notebooks) | 005 |
| 007 | THE CUTOVER: wafer adapter, wire admission, host proxy decomposition, golden-TLM regen (HITL; precondition: sprint 099 closed) | 006 |
| 008 | MOVER velocity mode: planVelocity + adapter replaceIn path (HITL) | 007 |
| 009 | Trace/plan-dump wire arms + notebook overlays | 007 |
| 010 | Tier-1 fault-knob matrix + lag-on validation | 007, 008 |
| 011 | Bench: arc_sweep grids, plateau to v_wheel_max, envelope/gain tuning, 098 pivot grid re-run (HITL) | 008, 009, 010 |
| 012 | Field: camera-verified chain + live PoseFix runs (HITL) | 011 |
| 013 | Cleanup: delete parked files, reserve retired fields, retire heading gains + governRatio segment path | 012 |

Tickets execute serially in the order listed. Ticket numbering in this
sprint does not map 1:1 to the driving issue's own 1-12 ticket table --
the issue's ticket 4 ("step(): tracker + policy + terminal machine +
IK/saturate/clamp/PI cascade") was split into tickets 004 (tracker +
IK/saturate/clamp) and 005 (policy + terminal machine + step()
composition), per this sprint's `architecture-update.md` Decision 1;
every ticket after that split shifts by one relative to the issue's own
numbering. HITL-gated tickets (007, 008, 011, 012) each carry an
explicit USB-attach precondition -- see each ticket's own Preconditions
section and the sprint's HITL/hardware note above. Ticket 007 additionally
requires sprint 099 closed before it starts (an execution-order
precondition, not a `depends-on` entry, since 099 is a different sprint).
