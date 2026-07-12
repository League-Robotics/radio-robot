---
id: '008'
title: 'Camera-fix mechanism: history ring, transport-compose, ungated EKF update'
status: open
use-cases: [SUC-005, SUC-007]
depends-on: ['004', '006']
github-issue: ''
issue: restore-pose-estimation-otos-encoders-delayed-camera-fixes.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Camera-fix mechanism: history ring, transport-compose, ungated EKF update

## Description

This ticket lands the sprint's headline new capability: a timestamped
delayed camera-fix. Ticket 004 already retyped `CommandEnvelope`'s `pose`
arm to `PoseFix` and implemented its `reset`/`zero_encoders` branches; this
ticket implements the third branch (`reset=false, zero_encoders=false` —
a genuine delayed fix), replacing that branch's current `ERR_UNIMPLEMENTED`
reply with a live dispatch to a new `bb.poseFixIn` mailbox, and implements
`PoseEstimator`'s private pose-history ring + interpolate + rigid-compose
+ ungated EKF update (architecture-update.md D5-D8).

Per D5: the camera is treated as authoritative — this update path is
**deliberately ungated** (never routed through ticket 006's innovation
gate, which exists only to protect against the OTOS). Per D6: the fix
timestamp is robot-clock ms (`t=`), matching the existing `Ack.t`/`PING`
clock-sync convention — no separate clock-sync mechanism is invented here.
Per D7: the fix arrives via a new, dedicated, latest-wins mailbox
(`bb.poseFixIn`) — a newer camera frame supersedes an undrained older one,
by design. Per D8: after applying a fix, `PoseEstimator` posts the
resulting fused pose to `otosSetPoseOut` (the SAME mechanism ticket 004
already wired for SI) so the OTOS chip's own frame stays agreed with the
new anchor.

New `ekf_r_fix_xy`/`ekf_r_fix_theta` config fields follow the **verified**
precedent of their sibling `ekf_r_otos_xy`/`ekf_r_otos_theta` exactly:
wire-tunable (binary `config`/`get`, `Rt::Configurator`) but **not** baked
into `scripts/gen_boot_config.py`/`data/robots/tovez.json` — those two
fields do not flow through the generator chain today either (verified by
reading `gen_boot_config.py`, `tovez.json`, and `check_config_sync.py`;
`check_config_sync.py` explicitly allowlists both siblings as wire-only,
host-pydantic-unmapped, with an inline "register as tunable in a future
sprint" comment). `ekf_r_fix_xy`/`ekf_r_fix_theta` get the SAME
`check_config_sync.py` allowlist treatment — no `gen_boot_config.py`/JSON
change.

## Acceptance Criteria

- [ ] `Rt::PoseFixCommand` (new POD, `source/runtime/commands.h`,
      mirroring `Rt::PoseResetCommand`'s style): `{float x, y, h; uint32_t
      t;}`.
- [ ] `Rt::Blackboard` gains `Mailbox<Rt::PoseFixCommand> poseFixIn;`.
- [ ] `BinaryChannel::handlePose()`'s third branch (neither `reset` nor
      `zero_encoders`) posts `Rt::PoseFixCommand{fix.x, fix.y, fix.h,
      fix.t}` to `bb.poseFixIn` and acks — replacing the `ERR_
      UNIMPLEMENTED` stub ticket 004 left in place.
- [ ] `PoseEstimator` gains a private, fixed-size pose-history ring:
      `PoseHistoryEntry{uint32_t t; float x, y, theta;}`, 24 entries (16B
      each = 384B), recorded from `(encX_, encY_, encTheta_)` — NEVER from
      `fusedPose`/`ekf_` state — every 50ms (a new internal timer,
      independent of the 20ms tick cadence).
- [ ] `PoseEstimator::tick()` gains a `Rt::Mailbox<Rt::PoseFixCommand>&
      poseFixIn` parameter (7th parameter, after `otosSetPoseOut`).
      Draining it (latest-wins, at most one entry per tick — a `Mailbox`,
      never accumulates) applies:
  1. Reject `t` older than the ring's oldest entry (`fixDropped_`
     diagnostic counter, no state change, no crash); clamp `t > now` to
     `now`.
  2. Interpolate `enc(T)`: linear x/y, wrapped-angle theta, between the
     two ring entries bracketing `T` (or newest-entry-to-`now` if `T` is
     more recent than the ring's last snapshot).
  3. Compose in the shared world frame: `implied.x = fix.x + (encNow.x -
     enc(T).x)`, same for `y`; `implied.h = wrap(fix.h + wrap(encNow.theta
     - enc(T).theta))` — exact (not approximate) per architecture-
     update.md's D5-D8 rationale.
  4. Apply as an UNGATED `EkfTiny` position+heading update using
     `ekf_r_fix_xy`/`ekf_r_fix_theta` (zero-as-unset sentinel, matching
     the existing four EKF fields' pattern) — call `EkfTiny::
     updatePosition()`/`updateHeading()` directly (these methods'
     internal gate from ticket 006 applies to EVERY caller — if this
     conflicts with "ungated," add a dedicated ungated entry point on
     `EkfTiny` rather than bypassing the existing gated ones silently;
     resolve this exact mechanism during implementation and document the
     choice in completion notes).
  5. Record the applied step into `lastPoseStep_` (the SAME mechanism
     ticket 004 built for SI).
  6. Post the resulting `fusedPose()` to `otosSetPoseOut` (the SAME
     mechanism ticket 004 built for SI).
- [ ] SI (`reset=true`, ticket 004) clears the ring; `zero_encoders` does
      not; a delayed fix does not either (verify all three against the
      existing/new code, not just by assertion).
- [ ] `protos/drivetrain.proto`'s `DrivetrainConfig` gains `ekf_r_fix_xy =
      42`, `ekf_r_fix_theta = 43`; `protos/config.proto`'s
      `DrivetrainConfigPatch` gains matching `optional float` fields;
      `source/runtime/commands.h`'s `DrivetrainConfigField` enum gains
      `kEkfRFixXy`, `kEkfRFixTheta`; `Rt::Configurator::foldDrivetrain()`
      (`configurator.cpp`) gains two more `if (m & bitOf(...))` lines,
      mirroring the four existing EKF-field lines; `BinaryChannel::
      handleConfigDrivetrain()`/`handleGet()` (`binary_channel.cpp`) gain
      matching lines, mirroring `ekf_r_otos_xy`/`ekf_r_otos_theta`'s own
      exactly.
- [ ] `PoseEstimator::configure()` reads `ekf_r_fix_xy`/`ekf_r_fix_theta`
      with the same zero-as-unset `sentinelOr()` substitution the
      existing four fields use, against a new documented fallback
      constant pair.
- [ ] `scripts/check_config_sync.py` gets a new allowlist entry for
      `("DrivetrainConfigPatch", "ekf_r_fix_xy")`/`"ekf_r_fix_theta"`,
      mirroring the existing `ekf_r_otos_xy`/`ekf_r_otos_theta` entries
      exactly (empty host-pydantic mapping, same inline comment style).
      **Do NOT** add these fields to `scripts/gen_boot_config.py` or
      `data/robots/tovez.json` — verified their siblings do not flow
      through that chain either.
- [ ] New `test_pose_fix_end_to_end.py` (sim): drive, send a fix with a
      known offset at a captured robot time, assert `fusedPose` converges
      by the composed amount while `encoderPose` stays untouched; a
      stale-timestamp fix (`t` older than the ring) produces no jump
      (dropped, counted, not crashed).
- [ ] Extended `pose_estimator_harness.cpp`: interpolation correctness
      (between-entries and newest-to-now cases) vs. hand-computed oracles;
      future-`t` clamp; SI-clears-ring / zero-encoders-does-not /
      fix-does-not; consecutive fixes compose correctly without ring
      invalidation; `otosSetPoseOut` posted exactly once per applied fix.
- [ ] Full sim suite passes.
- [ ] **BENCH smoke**: a `PoseFix` (delayed, `t` set from a real prior
      `PING`) is accepted (`OK`, not `ERR`) and `pose=` visibly converges
      toward the sent value on the stand.
- [ ] RAM check: read the map file after this ticket lands; +384B ring +
      Blackboard growth is expected and budgeted (architecture-update.md's
      Migration Concerns) — confirm no flash overflow (the only real
      budget per project convention).

## Implementation Plan

**Approach**: this is the sprint's most novel ticket — implement the ring
and compose math as small, independently-testable private helpers inside
`PoseEstimator` first (interpolation, world-frame compose), prove them
against hand oracles in the harness, THEN wire the `tick()`-level
draining/application/posting sequence, THEN wire the binary dispatch's
third branch. Resolve the "ungated update through EkfTiny" mechanism
(acceptance criterion 4 above) explicitly — read `ekf_tiny.{h,cpp}` as
ticket 006 leaves it before deciding whether a new ungated entry point is
needed or whether the existing gated methods can be called with the gate
provably inert for this caller (e.g. a very large threshold override is
NOT acceptable — that is gating with extra steps, not "ungated"; prefer a
clean, separate `EkfTiny::updatePositionUngated()`/
`updateHeadingUngated()` pair sharing the same Kalman-update core as a
private helper, if that keeps the public surface honest).

**Files to modify**:
- `source/runtime/commands.h` — `Rt::PoseFixCommand`.
- `source/runtime/blackboard.h` — `bb.poseFixIn`.
- `source/subsystems/pose_estimator.h` — ring type, ring state,
  `poseFixIn` param, interpolate/compose private helpers.
- `source/subsystems/pose_estimator.cpp` — ring recording, drain/apply
  logic.
- `source/estimation/ekf_tiny.h`/`.cpp` — possible ungated entry point
  (see Approach above).
- `protos/drivetrain.proto` — `ekf_r_fix_xy`/`ekf_r_fix_theta`.
- `protos/config.proto` — `DrivetrainConfigPatch` additions.
- Regenerate `source/messages/{drivetrain,config}.h`.
- `source/runtime/commands.h` — `DrivetrainConfigField` enum additions.
- `source/runtime/configurator.cpp` — `foldDrivetrain()` additions.
- `source/commands/binary_channel.cpp` — `handleConfigDrivetrain()`/
  `handleGet()` additions, `handlePose()`'s third branch.
- `scripts/check_config_sync.py` — two new allowlist entries.
- `tests/sim/unit/test_pose_fix_end_to_end.py` — new sim test.

**Files NOT to modify**: `scripts/gen_boot_config.py`,
`data/robots/tovez.json` (verified unnecessary — see Description).

**Testing plan**:
- Extend `pose_estimator_harness.cpp` per acceptance criteria.
- New `test_pose_fix_end_to_end.py`.
- Full sim suite.
- Bench smoke per acceptance criteria.

**Documentation updates**: `docs/protocol-v3.md`'s arm-7 table row and §8
note are stale after this ticket (and after ticket 004) — tracked as a
sprint-level Open Question for the team-lead to schedule, not blocking
this ticket.
