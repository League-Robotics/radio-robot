---
id: '004'
title: PoseEstimator wired encoder-only + BodyState/PoseStepped + binary PoseFix arm
  (SI/ZERO)
status: done
use-cases:
- SUC-002
- SUC-006
- SUC-007
depends-on:
- '001'
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

- [x] `Rt::MainLoop`'s constructor grows to `MainLoop(Hardware&,
      Drivetrain&, PoseEstimator&)`; both `main.cpp` and `tests/_infra/
      sim/sim_api.cpp`'s `SimHandle` update their one construction call
      site (`SimHandle` already has a `poseEstimator` member — this is a
      one-line constructor-arg change there, not a new member).
- [x] `MainLoop::tick()` reads the bound pair's fresh `MotorState` (via
      `bb.drivetrainConfig.left_port`/`right_port - 1`, mirroring `tlm_
      frame.cpp`'s own existing index-derivation pattern) AFTER `hardware_
      .tick(now)` and `drivetrain_.tick(...)`, and calls `poseEstimator_
      .tick(now, leftObs, rightObs, /*otosObs=*/nullptr, bb.poseResetIn,
      bb.otosSetPoseIn)` — passing `bb.otosSetPoseIn` as `PoseEstimator`'s
      new outbox parameter (see below).
- [x] After `poseEstimator_.tick(...)`, `MainLoop` drains `bb.
      otosSetPoseIn` (if non-empty) into `hardware_.odometer()->
      applySetPose(bb.otosSetPoseIn.take())` — the existing, already-
      implemented `Odometer::applySetPose()` primitive (its own doc
      comment already names this exact call site as "ported verbatim from
      main_loop.cpp's former inline otosSetPoseIn drain").
- [x] `MainLoop::commit()` gains: `bb.encoderPose = poseEstimator_.
      encoderPose();`, `bb.fusedPose = poseEstimator_.fusedPose();`,
      `bb.poseStepped = poseEstimator_.lastPoseStep();`, and the
      `bb.bodyState` derivation (pose from `bb.fusedPose.pose`, twist via
      `BodyKinematics::forward()` on the SAME bound-pair wheel velocities
      and `poseEstimator_.trackwidth()`, `v_y = 0`, `stamp` from `bb.
      fusedPose.stamp`).
- [x] `PoseEstimator::tick()`'s signature grows by one parameter:
      `Rt::Mailbox<msg::SetPose>& otosSetPoseOut`. Its existing `step 0`
      (drain `poseResetIn`) is extended: for a `kSetPose` entry, capture
      `fusedPose()` before and after `setPose(pose)`, compute `‖Δp‖`/
      `|Δθ|` into a new private `lastPoseStep_` member (read via the new
      `msg::PoseStep lastPoseStep() const` accessor), and post the
      resulting `fusedPose()` to `otosSetPoseOut`. A `kResetBaseline`
      entry is unchanged (no `PoseStep`, no `otosSetPoseOut` post).
      `lastPoseStep_` resets to `{0, 0}` at the TOP of every `tick()`
      call, so it reflects only the immediately-prior tick's correction.
- [x] New `drivetrain.proto` messages: `PoseFix` (fields `x`, `y`, `h`,
      `t`, `reset`, `zero_encoders` — exact shape in architecture-
      update.md D5-D8) and `PoseStep` (`pos`, `theta`).
- [x] `envelope.proto`'s `CommandEnvelope.cmd.pose` (field 7) retypes from
      `SetPose pose = 7` to `PoseFix pose_fix = 7`.
- [x] `BinaryChannel` gains `handlePose()`: `reset=true` ->
      `Rt::PoseResetCommand{kSetPose, {x,y,h}}` posted to `bb.
      poseResetIn`; `zero_encoders=true` -> `Rt::PoseResetCommand{
      kResetBaseline}` posted to `bb.poseResetIn` (both may be set in one
      message — both branches run); neither set -> `sendError(ERR_
      UNIMPLEMENTED, 7, ...)` (unchanged behavior for the not-yet-live
      delayed-fix case). `CmdKind::POSE`'s dispatch switch case in
      `handle()` calls this new handler instead of the bare
      `ERR_UNIMPLEMENTED` stub.
- [x] Sim: `test_pose_fix_reset_zero.py` (new) exercises the binary arm
      end to end via `sim_command_on()` — `reset=true` re-anchors `pose=`;
      `zero_encoders=true` does not move `pose=`/`otos=`; a stale/garbage
      neither-flag request replies `ERR unsupported`/`ERR_UNIMPLEMENTED`.
- [x] Extended `pose_estimator_harness.cpp`: `otosSetPoseOut` is posted
      exactly once per applied `kSetPose`, never on `kResetBaseline`;
      `lastPoseStep()` reports the correct magnitude for a known
      `setPose()` call and zero on every other tick.
- [x] Full sim suite passes (`uv run python -m pytest`).
- [ ] **BENCH** (DEFERRED — no robot USB-attached this session; see
      Completion Notes below): on the stand, `encpose`-equivalent
      (`pose=`/`enc=` on TLM) tracks real wheel motion; a binary
      `PoseFix{reset=true, x, y, h}` re-anchors `pose=` within one tick
      with no phantom jump; a `PoseFix{zero_encoders=true}` produces no
      visible pose jump.

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

## Completion Notes

Implemented per plan, no deviations from the architecture doc's D1/D5-D8
pseudocode.

- `MainLoop::tick()` derives `leftObs`/`rightObs` via `hardware_.
  motorState(leftIdx/rightIdx)` (fresh, post `hardware_.tick()`/
  `drivetrain_.tick()`) and passes them to `poseEstimator_.tick(...,
  /*otosObs=*/nullptr, bb.poseResetIn, bb.otosSetPoseIn)`; drains `bb.
  otosSetPoseIn` into `hardware_.odometer()->applySetPose(...)`
  immediately after. `MainLoop::commit()` re-derives the SAME `leftIdx`/
  `rightIdx` off the freshly-committed `bb.motors[]` (rather than
  threading `leftObs`/`rightObs` through `commit()`'s own signature) to
  build `bb.bodyState`'s twist — `bb.motors` is refreshed from the exact
  same `hardware_` state `tick()`'s own reads came from, with no
  intervening hardware mutation, so this is provably the same value while
  keeping `commit()`'s signature unchanged (mirrors `tlm_frame.cpp`'s own
  `twist=` derivation pattern).
- `PoseEstimator::tick()`'s `kSetPose` drain captures `fusedPose()`
  before/after `setPose()`, computes `lastPoseStep_` via `sqrtf(dx²+dy²)`/
  `fabsf(wrapPi(Δh))`, and posts the re-anchored pose (converted to
  `msg::SetPose`) to `otosSetPoseOut`. `lastPoseStep_` resets to `{0,0}`
  at the top of every `tick()` call.
- `BinaryChannel::handlePose()` posts up to two `Rt::PoseResetCommand`
  entries (one per set flag) to `bb.poseResetIn`, checking each `post()`
  for `ERR_FULL`; neither flag set replies `ERR_UNIMPLEMENTED` field=7
  (delayed-fix branch, ticket 008).
- Regenerated `source/messages/{drivetrain,envelope,wire}.h`/`wire.cpp`
  via `scripts/gen_messages.py` and `host/robot_radio/robot/pb2/*` via
  `scripts/gen_pb2.py`. `CommandEnvelope.cmd`'s oneof-arm enumerator for
  field 7 is generator-derived from the field name (`pose_fix` ->
  `POSE_FIX`), so every call site naming the old `CmdKind::POSE`
  enumerator needed updating too — found via `grep`, not anticipated in
  the plan: `source/commands/binary_channel.cpp`'s dispatch switch,
  `tests/sim/unit/wire_differential_harness.cpp` (2 sites — its sibling
  `DrivetrainCommand::ControlKind::POSE` references, a DIFFERENT enum for
  the unrelated `DrivetrainCommand.control.pose`/`SetPose` arm, were left
  untouched), `tests/sim/unit/wire_codec_harness.cpp`'s
  `scenarioRoundTripDeclaredOnlyPoseAndOtos` (rebuilt to encode/decode a
  `PoseFix` instead of a bare `SetPose`, plus `t`/`reset`/`zero_encoders`
  round-trip assertions), and `tests/sim/unit/test_wire_differential.py`'s
  `test_field_numbers_match_pb2_descriptors` (`"pose"` -> `"pose_fix"` in
  `expected_cmd_numbers`). `tests/sim/unit/test_binary_channel.py`'s
  `test_binary_declared_only_arms_reply_err_unimplemented` parametrize
  case also needed its `pose=SetPose()` kwarg retyped to
  `pose_fix=PoseFix()` (still asserting `ERR_UNIMPLEMENTED` field=7, since
  a default-constructed `PoseFix` has both flags false).
- `kMaxEncodedSize` report (`gen_messages.py`'s own printed budget):
  `CommandEnvelope` total=168B, worst arm still `id`=162B (`pose_fix`
  itself is 27B, up from `SetPose`'s ~17B, nowhere near displacing `id`
  as the worst arm); `ReplyEnvelope` total=171B, worst arm still `tlm`=
  165B, byte-identical to before this ticket (no reply arm touched). Both
  well under the 186B cap; `scripts/check_config_sync.py` reports "OK —
  no drift detected" (`PoseFix`/`PoseStep` are not config `Patch` fields,
  so this check is unaffected, run per the ticket's own instruction as a
  sanity pass).
- Build: `just build` — both the ARM firmware hex (FLASH 86.76%, RAM
  98.33% — normal per project convention, not itself a risk signal) and
  the host sim library (`libfirmware_host`) compiled clean.
- Tests: new `tests/sim/unit/test_pose_fix_reset_zero.py` (5 tests, all
  pass) + extended `tests/sim/unit/pose_estimator_harness.cpp` (new
  scenario `scenarioOtosSetPoseOutAndLastPoseStepMagnitude`, plus the
  `otosSetPoseOut` parameter threaded through every pre-existing
  `pe.tick()` call site — all scenarios pass, compiled standalone with
  `c++ -std=c++20`). Full suite: `uv run python -m pytest` — 1287 passed,
  5 xfailed, 0 failed (baseline was 1282 passed/5 xfailed/0 failed; the 5
  new `test_pose_fix_reset_zero.py` tests account for the delta exactly).
- **BENCH deferred**: no robot is USB-attached to this session. The
  team-lead should schedule a bench pass per this ticket's BENCH
  acceptance criterion before the sprint closes (or explicitly accept the
  deferral if a later ticket's bench session already covers this
  behavior).
- **docs/protocol-v3.md staleness** (flagged per this ticket's own
  Documentation-updates note, not fixed here): §3's arm-7 table row still
  says `SetPose`/`pose`, and §8 still describes the pre-099-004
  `ERR_UNIMPLEMENTED`-for-everything state. Both need a follow-up edit
  once the team-lead schedules the doc pass (architecture-update.md's own
  Open Question 1 already tracks this at the sprint level).
