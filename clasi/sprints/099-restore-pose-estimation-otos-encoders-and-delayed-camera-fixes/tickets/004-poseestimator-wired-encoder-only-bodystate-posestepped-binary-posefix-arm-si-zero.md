---
id: '004'
title: PoseEstimator wired encoder-only + BodyState/PoseStepped + binary PoseFix arm
  (SI/ZERO)
status: open
use-cases: [SUC-002, SUC-006, SUC-007]
depends-on: ['001']
github-issue: ''
issue: restore-pose-estimation-otos-encoders-delayed-camera-fixes.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# PoseEstimator wired encoder-only + BodyState/PoseStepped + binary PoseFix arm (SI/ZERO)

## Description

`Subsystems::PoseEstimator` is fully implemented and unit-tested but never
ticked live. This ticket wires it into `Rt::MainLoop` in **encoder-only**
mode (`otosObs = nullptr`, literally — OTOS fusion is ticket 007's "one-
token flip", not this ticket's job), and gives it a live wire surface: the
`CommandEnvelope` `pose` arm (field 7) is retyped from the
declared-only, never-implemented `SetPose` to a new `PoseFix` message
(architecture-update.md D5-D8, Decision 1). This ticket implements
`PoseFix`'s `reset`/`zero_encoders` branches only — both reuse
`PoseEstimator`'s existing, already-tested `setPose()`/
`resetEncoderBaseline()` dispatch through the **unchanged** `poseResetIn`
queue. The genuine delayed-fix branch (`reset=false, zero_encoders=false`)
replies `ERR_UNIMPLEMENTED` in this ticket — ticket 008 makes it live,
reusing the same wire arm and message (no second schema change).

This ticket also lands two small, forward-looking Blackboard cells the
architecture doc's Decision 5 justifies in detail: `bb.bodyState` (reuses
the existing `msg::PoseEstimate` shape — fused pose + body twist, the one
cell the *next* sprint's motion-v2 adapter is designed to read directly)
and `bb.poseStepped` (a new tiny `PoseStep` message — the magnitude of
whatever correction was just applied, zero on every other tick). Neither
is on the wire this sprint (in-process only).

**Do not implement OTOS fusion in this ticket** — `PoseEstimator::tick()`
is called with `otosObs = nullptr` unconditionally; `bb.otosValid` and the
`fusableThisPass()`/real-observation assembly are ticket 007's job.

## Acceptance Criteria

- [ ] `Rt::MainLoop`'s constructor grows to `MainLoop(Hardware&,
      Drivetrain&, PoseEstimator&)`; both `main.cpp` and `tests/_infra/
      sim/sim_api.cpp`'s `SimHandle` update their one construction call
      site (`SimHandle` already has a `poseEstimator` member — this is a
      one-line constructor-arg change there, not a new member).
- [ ] `MainLoop::tick()` reads the bound pair's fresh `MotorState` (via
      `bb.drivetrainConfig.left_port`/`right_port - 1`, mirroring `tlm_
      frame.cpp`'s own existing index-derivation pattern) AFTER `hardware_
      .tick(now)` and `drivetrain_.tick(...)`, and calls `poseEstimator_
      .tick(now, leftObs, rightObs, /*otosObs=*/nullptr, bb.poseResetIn,
      bb.otosSetPoseIn)` — passing `bb.otosSetPoseIn` as `PoseEstimator`'s
      new outbox parameter (see below).
- [ ] After `poseEstimator_.tick(...)`, `MainLoop` drains `bb.
      otosSetPoseIn` (if non-empty) into `hardware_.odometer()->
      applySetPose(bb.otosSetPoseIn.take())` — the existing, already-
      implemented `Odometer::applySetPose()` primitive (its own doc
      comment already names this exact call site as "ported verbatim from
      main_loop.cpp's former inline otosSetPoseIn drain").
- [ ] `MainLoop::commit()` gains: `bb.encoderPose = poseEstimator_.
      encoderPose();`, `bb.fusedPose = poseEstimator_.fusedPose();`,
      `bb.poseStepped = poseEstimator_.lastPoseStep();`, and the
      `bb.bodyState` derivation (pose from `bb.fusedPose.pose`, twist via
      `BodyKinematics::forward()` on the SAME bound-pair wheel velocities
      and `poseEstimator_.trackwidth()`, `v_y = 0`, `stamp` from `bb.
      fusedPose.stamp`).
- [ ] `PoseEstimator::tick()`'s signature grows by one parameter:
      `Rt::Mailbox<msg::SetPose>& otosSetPoseOut`. Its existing `step 0`
      (drain `poseResetIn`) is extended: for a `kSetPose` entry, capture
      `fusedPose()` before and after `setPose(pose)`, compute `‖Δp‖`/
      `|Δθ|` into a new private `lastPoseStep_` member (read via the new
      `msg::PoseStep lastPoseStep() const` accessor), and post the
      resulting `fusedPose()` to `otosSetPoseOut`. A `kResetBaseline`
      entry is unchanged (no `PoseStep`, no `otosSetPoseOut` post).
      `lastPoseStep_` resets to `{0, 0}` at the TOP of every `tick()`
      call, so it reflects only the immediately-prior tick's correction.
- [ ] New `drivetrain.proto` messages: `PoseFix` (fields `x`, `y`, `h`,
      `t`, `reset`, `zero_encoders` — exact shape in architecture-
      update.md D5-D8) and `PoseStep` (`pos`, `theta`).
- [ ] `envelope.proto`'s `CommandEnvelope.cmd.pose` (field 7) retypes from
      `SetPose pose = 7` to `PoseFix pose_fix = 7`.
- [ ] `BinaryChannel` gains `handlePose()`: `reset=true` ->
      `Rt::PoseResetCommand{kSetPose, {x,y,h}}` posted to `bb.
      poseResetIn`; `zero_encoders=true` -> `Rt::PoseResetCommand{
      kResetBaseline}` posted to `bb.poseResetIn` (both may be set in one
      message — both branches run); neither set -> `sendError(ERR_
      UNIMPLEMENTED, 7, ...)` (unchanged behavior for the not-yet-live
      delayed-fix case). `CmdKind::POSE`'s dispatch switch case in
      `handle()` calls this new handler instead of the bare
      `ERR_UNIMPLEMENTED` stub.
- [ ] Sim: `test_pose_fix_reset_zero.py` (new) exercises the binary arm
      end to end via `sim_command_on()` — `reset=true` re-anchors `pose=`;
      `zero_encoders=true` does not move `pose=`/`otos=`; a stale/garbage
      neither-flag request replies `ERR unsupported`/`ERR_UNIMPLEMENTED`.
- [ ] Extended `pose_estimator_harness.cpp`: `otosSetPoseOut` is posted
      exactly once per applied `kSetPose`, never on `kResetBaseline`;
      `lastPoseStep()` reports the correct magnitude for a known
      `setPose()` call and zero on every other tick.
- [ ] Full sim suite passes (`uv run python -m pytest`).
- [ ] **BENCH**: on the stand, `encpose`-equivalent (`pose=`/`enc=` on
      TLM) tracks real wheel motion; a binary `PoseFix{reset=true, x, y,
      h}` re-anchors `pose=` within one tick with no phantom jump; a
      `PoseFix{zero_encoders=true}` produces no visible pose jump.

## Implementation Plan

**Approach**: grow `MainLoop`'s constructor and `tick()`/`commit()` per
architecture-update.md's D1 pass-body pseudocode (the encoder-only slice
of it — `otosObs` stays a literal `nullptr` this ticket). Extend
`PoseEstimator::tick()`'s existing `poseResetIn`-drain loop surgically
(it is a small, well-tested function — read `pose_estimator.cpp` in full
before editing). Add the wire arm end to end following the exact pattern
`config`/`get`/`stream` already established across sprints 095-097
(declare schema, add a `handle<Arm>()` helper, wire the dispatch switch
case).

**Files to create**:
- `tests/sim/unit/test_pose_fix_reset_zero.py` — binary arm sim test.

**Files to modify**:
- `protos/drivetrain.proto` — `PoseFix`, `PoseStep` messages.
- `protos/envelope.proto` — retype `pose` arm 7.
- Regenerate `source/messages/{drivetrain,envelope}.h` via
  `scripts/gen_messages.py`.
- `source/runtime/commands.h` — no change needed this ticket (`Rt::
  PoseResetCommand` already carries everything `reset`/`zero_encoders`
  need; `Rt::PoseFixCommand` is ticket 008's addition).
- `source/runtime/blackboard.h` — no change needed this ticket (`bb.
  poseResetIn`/`bb.otosSetPoseIn` already exist; `bb.bodyState`/`bb.
  poseStepped` are new state cells, `bb.poseFixIn` is ticket 008's
  addition).
  - Add `msg::PoseEstimate bodyState;` and `msg::PoseStep poseStepped;`
    to the state-plane section.
- `source/subsystems/pose_estimator.h` — `tick()` gains `otosSetPoseOut`
  param; new `lastPoseStep()` accessor; new private `msg::PoseStep
  lastPoseStep_` (or equivalent float pair) member.
- `source/subsystems/pose_estimator.cpp` — extend the `poseResetIn` drain
  loop as specified above.
- `source/runtime/main_loop.h` — constructor gains `PoseEstimator&`.
- `source/runtime/main_loop.cpp` — `tick()`/`commit()` extensions per
  above.
- `source/main.cpp` — `MainLoop` construction gains `poseEstimator`.
- `tests/_infra/sim/sim_api.cpp` — `SimHandle::loop` construction gains
  `poseEstimator` (member already exists).
- `source/commands/binary_channel.cpp` — `handlePose()`, dispatch switch
  case.

**Testing plan**:
- New `pose_estimator_harness.cpp` cases for `otosSetPoseOut`/
  `lastPoseStep()`.
- New `test_pose_fix_reset_zero.py` sim test.
- Full sim suite.
- Bench per acceptance criteria.

**Documentation updates**: none required by this ticket's own scope
(`docs/protocol-v3.md`'s staleness for the arm-7 retype is tracked as a
sprint-level Open Question, not an individual ticket's job — flag it in
the ticket's completion notes so the team-lead can schedule the doc
update, but do not block this ticket on it).
