---
id: 008
title: 'Camera-fix mechanism: history ring, transport-compose, ungated EKF update'
status: done
use-cases:
- SUC-005
- SUC-007
depends-on:
- '004'
- '006'
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

- [x] `Rt::PoseFixCommand` (new POD, `source/runtime/commands.h`,
      mirroring `Rt::PoseResetCommand`'s style): `{float x, y, h; uint32_t
      t;}`.
- [x] `Rt::Blackboard` gains `Mailbox<Rt::PoseFixCommand> poseFixIn;`.
- [x] `BinaryChannel::handlePose()`'s third branch (neither `reset` nor
      `zero_encoders`) posts `Rt::PoseFixCommand{fix.x, fix.y, fix.h,
      fix.t}` to `bb.poseFixIn` and acks — replacing the `ERR_
      UNIMPLEMENTED` stub ticket 004 left in place.
- [x] `PoseEstimator` gains a private, fixed-size pose-history ring:
      `PoseHistoryEntry{uint32_t t; float x, y, theta;}`, 24 entries (16B
      each = 384B), recorded from `(encX_, encY_, encTheta_)` — NEVER from
      `fusedPose`/`ekf_` state — every 50ms (a new internal timer,
      independent of the 20ms tick cadence).
- [x] `PoseEstimator::tick()` gains a `Rt::Mailbox<Rt::PoseFixCommand>&
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
- [x] SI (`reset=true`, ticket 004) clears the ring; `zero_encoders` does
      not; a delayed fix does not either (verify all three against the
      existing/new code, not just by assertion).
- [x] `protos/drivetrain.proto`'s `DrivetrainConfig` gains `ekf_r_fix_xy =
      42`, `ekf_r_fix_theta = 43`; `protos/config.proto`'s
      `DrivetrainConfigPatch` gains matching `optional float` fields;
      `source/runtime/commands.h`'s `DrivetrainConfigField` enum gains
      `kEkfRFixXy`, `kEkfRFixTheta`; `Rt::Configurator::foldDrivetrain()`
      (`configurator.cpp`) gains two more `if (m & bitOf(...))` lines,
      mirroring the four existing EKF-field lines; `BinaryChannel::
      handleConfigDrivetrain()`/`handleGet()` (`binary_channel.cpp`) gain
      matching lines, mirroring `ekf_r_otos_xy`/`ekf_r_otos_theta`'s own
      exactly.
- [x] `PoseEstimator::configure()` reads `ekf_r_fix_xy`/`ekf_r_fix_theta`
      with the same zero-as-unset `sentinelOr()` substitution the
      existing four fields use, against a new documented fallback
      constant pair.
- [x] `scripts/check_config_sync.py` gets a new allowlist entry for
      `("DrivetrainConfigPatch", "ekf_r_fix_xy")`/`"ekf_r_fix_theta"`,
      mirroring the existing `ekf_r_otos_xy`/`ekf_r_otos_theta` entries
      exactly (empty host-pydantic mapping, same inline comment style).
      **Do NOT** add these fields to `scripts/gen_boot_config.py` or
      `data/robots/tovez.json` — verified their siblings do not flow
      through that chain either.
- [x] New `test_pose_fix_end_to_end.py` (sim): drive, send a fix with a
      known offset at a captured robot time, assert `fusedPose` converges
      by the composed amount while `encoderPose` stays untouched; a
      stale-timestamp fix (`t` older than the ring) produces no jump
      (dropped, counted, not crashed).
- [x] Extended `pose_estimator_harness.cpp`: interpolation correctness
      (between-entries and newest-to-now cases) vs. hand-computed oracles;
      future-`t` clamp; SI-clears-ring / zero-encoders-does-not /
      fix-does-not; consecutive fixes compose correctly without ring
      invalidation; `otosSetPoseOut` posted exactly once per applied fix.
- [x] Full sim suite passes.
- [ ] **BENCH smoke**: a `PoseFix` (delayed, `t` set from a real prior
      `PING`) is accepted (`OK`, not `ERR`) and `pose=` visibly converges
      toward the sent value on the stand. **DEFERRED** — not run this
      session; see completion notes below.
- [x] RAM check: read the map file after this ticket lands; +384B ring +
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

## Completion Notes

**Ungated-EKF mechanism (AC 4).** Added a dedicated `EkfTiny::
updatePositionUngated(xFix, yFix, rFixXy)`/`updateHeadingUngated(thetaFix,
rFixTheta)` pair (`source/estimation/ekf_tiny.{h,cpp}`). Both the gated
(`updatePosition()`/`updateHeading()`, ticket 006) and ungated methods route
through the SAME private Kalman-update core, split into a "compute gain"
step (innovation, S, S⁻¹, Mahalanobis/sigma statistic, Kalman gain — no
state mutation: `computePositionGain()`/`computeHeadingGain()`) and an
"apply gain" step (the actual x/P mutation: `applyPositionGain()`/
`applyHeadingGain()`). The gated methods gate BETWEEN the two calls
(reject before ever mutating state, exactly as before this ticket); the
ungated methods call the identical pair back-to-back with no gate and never
touch either channel's rejection-streak counter. This keeps the public
surface honest (no giant-threshold "gating with extra steps") while
guaranteeing the two paths can never numerically diverge — they share one
math implementation, not two copies.

**Pre-existing wiring gap found and fixed.** `poseEstimator.configure()`
was never called at boot in EITHER `source/main.cpp` or `tests/_infra/sim/
sim_api.cpp` (only `drivetrain.configure(dtConfig)` was) — present since
099-004 first constructed `poseEstimator` there. Without it, `EkfTiny::
init()` is never reached, so its q/r noise matrices stay at their C++
zero-default forever, `P` never grows off zero, and EVERY `EkfTiny` update
channel (gated OTOS, ungated delayed-fix) silently no-ops via the
numerically-singular-S safety guard. This was discovered while verifying
this ticket's own `test_pose_fix_end_to_end.py` — a delayed fix produced
zero measurable correction until `poseEstimator.configure(dtConfig)` was
added right after the existing `drivetrain.configure(dtConfig)` call in
both files. This is why the pose-estimator harness (which always calls
`pe.configure()` explicitly) never caught the gap, and why no existing
OTOS-fusion test caught it either (ticket 007, OTOS fusion, is not done
yet). Flagged for the team-lead: ticket 007 should double-check this stays
fixed when it lands (it depends on the identical `configure()` call having
run for its own gated OTOS corrections to do anything at all).

**Pre-existing test breakage found and fixed** (making the "neither
reset nor zero_encoders" branch live broke three tests that hard-coded the
old `ERR_UNIMPLEMENTED` stub reply): `test_binary_channel.py`'s
`test_binary_declared_only_arms_reply_err_unimplemented[pose_fix_neither_flag]`
parametrize case removed (only `otos` remains declared-only);
`test_pose_fix_reset_zero.py`'s two neither-flag tests renamed/rewritten to
assert `OK` + `bb.poseFixIn` dispatch instead; `test_wire_differential.py`'s
hand-transcribed `expected_drivetrain_patch` field-number map extended with
`ekf_r_fix_xy`/`ekf_r_fix_theta`. Also found and fixed a latent UBSan crash
in `wire_differential_harness.cpp`'s `cmdEncodeCfgDrivetrain()`: it builds a
`msg::ReplyEnvelope` whose `body` is a raw union with no default member
initializer, and populates every `DrivetrainConfigPatch` field by hand — the
two new fields, left untouched, read as uninitialized stack garbage under
UBSan (`load of value 144, which is not a valid value for type 'const
bool'`) the moment the schema grew past this hand-maintained list. Extended
the harness's CLI (2 more argv floats) and the Python driver
(`encode_cfg_drivetrain()`, new params default to ordinary in-range values
so pre-099-008 callers don't need updating) to populate both new fields.

**Ring/interpolate/compose design.** `PoseHistoryEntry{t, x, y, theta}` × 24
in a plain circular buffer (`ringHead_`/`ringCount_`, no dynamic
allocation). `interpolateEncAt(t, now)` handles three cases: empty ring
(returns live `encX_/encY_/encTheta_` — nothing to interpolate against),
between two bracketing entries, or newest-entry-to-`now` (treating `now`'s
live encoder pose as an implicit extra ring entry) — all three funnel
through one `lerpEncPose()` helper (linear x/y, wrapped-angle theta).
`applyPoseFix()` does reject-or-clamp → interpolate → compose → ungated
EKF update → `lastPoseStep_` → `otosSetPoseOut`, in that order, and never
touches the ring itself. `setPose()` (SI) calls `clearRing()`;
`resetEncoderBaseline()` (ZERO) and `applyPoseFix()` do not.

**Config budget check.** Regenerated `kMaxEncodedSize` report (`gen_
messages.py`) after adding the two new `DrivetrainConfigPatch` fields:
`CommandEnvelope` total unchanged at 168B (`config` arm 34B→44B, still far
under the `id` arm's 162B worst case); `ReplyEnvelope` total unchanged at
171B (`cfg` arm ~36B→46B, still far under the `tlm` arm's 165B worst case).
Both stay well under the 186B cap — no patch-split needed, no exception
thrown.

**RAM/flash.** `just build` succeeds: FLASH 325648B/364KB (87.37%, no
overflow); RAM 120768B/122816B (98.33% — expected/budgeted per project
convention, unchanged posture from prior 099 tickets).

**Verification.** New/extended C++ harness: `pose_estimator_harness.cpp`
gained 7 new scenarios (interpolation between-entries, newest-to-now,
future-t clamp, SI/zero/fix ring-clearing behavior, consecutive fixes,
otosSetPoseOut/lastPoseStep magnitude, stale-fix no-jump) — all 15
scenarios (8 pre-existing + 7 new) pass. New `test_pose_fix_end_to_end.py`
(2 tests) passes. Full sim suite: 1288 passed, 4 xfailed, 1 xpassed, 0
failed (baseline was 1287/4/1/0 — net +1 after removing one now-stale
parametrize case and adding 2 new tests). `check_config_sync.py`: OK.

**BENCH smoke — DEFERRED, not run this session.** Requires the physical
robot on the stand; left unchecked in the Acceptance Criteria above per
the team-lead's dispatch instructions. Needs a real prior `PING` timestamp
and a delayed `PoseFix` sent shortly after, confirming `OK` (not `ERR`) and
`pose=` visibly converging on the wire.

**For the team-lead:** `docs/protocol-v3.md`'s arm-7 table row/§8 note are
now doubly stale (ticket 004's SI/ZERO live-ness AND this ticket's delayed-
fix live-ness) — still not addressed by any ticket in this sprint; flagged
again per the architecture doc's own Open Question 1.
