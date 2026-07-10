---
id: '001'
title: Config and telemetry wire schema (config.proto, telemetry.proto, envelope.proto
  edits)
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Config and telemetry wire schema (config.proto, telemetry.proto, envelope.proto edits)

## Description

Declare the binary wire contract for telemetry and config (M1,
architecture-update.md). This is schema-only — no generator changes, no
runtime behavior, no `BinaryChannel` logic (that is tickets 003-005).

**Approach**:
1. Add `protos/telemetry.proto`: a real `Telemetry` message, curated per
   Decision 6 to the UNION of the STREAM/SNAP text frame's fields (`enc`/
   `vel`/`cmd`/`pose`/`encpose`/`otos`+`otosconn`/`twist`/`mode`/`seq`/
   `now`) and the separate one-shot `TLM` verb's bench-diagnostic fields
   (`acc`/`active`/`conn`/`glitch`/`ts`, sourced the way `handleTlm()` in
   `motion_commands.cpp` already computes them — do not touch that file).
   Presence semantics mirror `Telemetry::TlmFrameInput`'s existing `has*`
   convention (explicit bool flags alongside plain value fields) rather
   than proto3 `optional` — only `cmd` and `otos`(+`otosconn`) are ever
   conditionally absent in the current text implementation; every other
   field group is unconditionally present.
2. Add `protos/config.proto`: a `ConfigTarget` enum (`CONFIG_DRIVETRAIN`,
   `CONFIG_MOTOR_LEFT`, `CONFIG_MOTOR_RIGHT`, `CONFIG_PLANNER`,
   `CONFIG_WATCHDOG` — the last for `sTimeout`, which is NOT one of
   `Rt::ConfigDelta`'s four Configurator targets, see Open Question 4) and
   three curated Patch messages (Decision 2), mirroring ONLY the 15 keys
   `config_commands.cpp`'s `kAllKeys` already registers — never the full
   `DrivetrainConfig`(41)/`MotorConfig`(10)/`PlannerConfig`(10) messages:
   - `DrivetrainConfigPatch`: `trackwidth`, `rotational_slip`, `ekf_q_xy`,
     `ekf_q_theta`, `ekf_r_otos_xy`, `ekf_r_otos_theta` (all `optional`
     float — `Opt<float>` presence signals "this field is being set").
   - `MotorConfigPatch`: `optional float travel_calib` PLUS a `port`/side
     selector field that disambiguates `travel_calib` ONLY (Decision 5 —
     mirrors `ml`/`mr`'s existing per-side addressing); `optional float
     kp`/`ki`/`kff`/`i_max`/`kaw` (the five `Gains` members — these are
     UNCONDITIONALLY applied to both bound motors by ticket 004's
     translation, regardless of `port`; do not make them per-side).
   - `PlannerConfigPatch`: `optional float min_speed`.
   None of these fields are `optional` on the EXISTING generated
   `DrivetrainConfig`/`MotorConfig`/`PlannerConfig` messages — those stay
   completely untouched, so `Opt<T>` never reaches any existing
   `configure()` call site (Decision 2, Risk 4).
3. Edit `protos/envelope.proto`: replace the empty placeholder
   `ConfigDelta`/`ConfigSnapshot`/`Telemetry` messages with real bodies.
   `ConfigDelta`/`ConfigSnapshot` are each a `oneof` over the three Patch
   types plus a bare `uint32` watchdog-window case (`CONFIG_WATCHDOG`) —
   the SAME oneof idiom `CommandEnvelope.cmd` already uses (Decision 4:
   `ConfigSnapshot` additionally carries a `ConfigTarget target` field so
   the reply self-identifies which slice it is). Retype
   `ConfigGet.target` from `uint32` to `ConfigTarget`.
4. Run `python scripts/gen_messages.py` — verify it regenerates
   `source/messages/telemetry.h`, `source/messages/config.h` (new) and
   `source/messages/envelope.h` (changed) without altering the shape of
   any UNRELATED pre-existing header.
5. Verify the generated `static_assert(kMaxEncodedSize<=186)` passes for
   `CommandEnvelope` and `ReplyEnvelope` with these arms now real. If
   `Telemetry` does not fit, apply Decision 6's documented trim order
   (drop `encpose` first, then `otos`+`otosconn`) and record the final
   field list in this ticket's completion notes — do not silently
   narrow the field set without recording why.

**Files to create**: `protos/telemetry.proto`, `protos/config.proto`.
**Files to modify**: `protos/envelope.proto`.
No changes to `scripts/gen_messages.py` — every mechanism this schema
needs (`oneof`-of-submessages, `optional` scalar fields inside a message,
enums) already exists (Step 1 finding, architecture-update.md).

## Acceptance Criteria

- [x] `protos/telemetry.proto`'s `Telemetry` fields are traceable 1:1 to
      either `Telemetry::TlmFrameInput` (text STREAM/SNAP) or
      `handleTlm()`'s own field computation (`motion_commands.cpp`) — no
      field invents new firmware capability.
- [x] `protos/config.proto`'s three Patch messages' fields are traceable
      1:1 to `config_commands.cpp`'s `kAllKeys` (15 keys) — no key outside
      that list is exposed.
- [x] `MotorConfigPatch` has a `port`/side selector that disambiguates
      `travel_calib` only, per Decision 5 — the five `Gains` fields carry
      no per-side selector of their own.
- [x] `ConfigTarget` has a 4th enumerator for the watchdog window
      (`sTimeout`), distinct from the Configurator's three fold targets
      (drivetrain/motor/planner) — see Open Question 4; document in a
      code comment that this target routes to `bb.streamWatchdogWindowIn`,
      not `bb.configIn`, at the `BinaryChannel` boundary (ticket 004).
- [x] `python scripts/gen_messages.py --dry-run` succeeds; diff shows only
      the new/edited files above — zero diff to any unrelated
      pre-existing generated header.
- [x] The generated `static_assert(kMaxEncodedSize<=186)` passes for
      `CommandEnvelope` and `ReplyEnvelope`. If a trim was required to
      make `Telemetry` fit, the final field list and the reason are
      recorded in this ticket's completion notes.
- [x] `just build-sim` and the full existing sim suite (~469 tests) stay
      green — this ticket changes generated headers only; no existing
      test's behavior should change.
- [x] 095's differential codec gate (`tests/sim/unit/test_wire_differential.py`
      and friends) still passes — this schema addition must not regress
      the already-implemented arms' codec correctness.

## Completion Notes

**Files created**: `protos/telemetry.proto` (`Telemetry`), `protos/config.proto`
(`ConfigTarget`, `BoundMotorSide`, `DrivetrainConfigPatch`, `MotorConfigPatch`,
`PlannerConfigPatch`).

**Files edited**: `protos/envelope.proto` (imports telemetry.proto/config.proto;
`Telemetry` placeholder removed in favor of the imported leaf type, the SAME
"declare in the subsystem proto, reference bare from the envelope oneof"
pattern `DrivetrainCommand`/`MotionSegment`/`SetPose`/`OdometerCommand` already
use; `ConfigDelta`/`ConfigSnapshot` placeholders replaced with real
`oneof`-over-Patch-types bodies per Decision 2/4; `ConfigGet.target` retyped
`uint32` -> `ConfigTarget`). Regenerated (via `scripts/gen_messages.py`, no
generator changes): `source/messages/envelope.h` (changed),
`source/messages/telemetry.h` (new), `source/messages/config.h` (new),
`source/messages/layout_checks.h` (additive: 3 new standard-layout
static_asserts for the 3 Patch structs), `source/messages/wire.{h,cpp}`
(changed: new field tables + updated `kMaxEncodedSize` constants). Host pb2
regenerated by the same build step (`scripts/gen_pb2.py`, run automatically by
`just build-sim`): `host/robot_radio/robot/pb2/envelope_pb2.py` (changed),
`config_pb2.py`/`telemetry_pb2.py` (new). `git status` confirmed ZERO diff to
any other pre-existing generated header (common/communicator/drivetrain/
gripper/motion/motor/odometer/planner/ports/sensors.h, layout_checks.cpp).

One pre-existing, hand-written (non-generated) test file needed a matching
2-line fix: `tests/sim/unit/wire_codec_harness.cpp`'s
`scenarioMissingReqRejected()`/`scenarioUnknownFieldSkipped()` read
`ConfigGet.target.val` as a bare `uint64_t` in `checkU64Eq()`; retyping
`target` to `ConfigTarget` (this ticket's own required step 3) broke that
compile. Fixed with `static_cast<uint64_t>(...)`, the exact same cast the
harness already used for `msg::ErrCode` at `scenarioEncodeErr()` (line 477)
-- a direct, minimal, required consequence of the retype, not new behavior.

**Traceability -- config.proto (15 keys)**: grepped `config_commands.cpp`'s
`kAllKeys` array directly; confirmed exactly 15 entries: `tw`, `ml`, `mr`,
`pid.kp`, `pid.ki`, `pid.kff`, `pid.iMax`, `pid.kaw`, `rotSlip`, `ekfQxy`,
`ekfQtheta`, `ekfROtosXy`, `ekfROtosTheta`, `minSpeed`, `sTimeout`. Mapping:
`DrivetrainConfigPatch` (6 fields) <- `tw`/`rotSlip`/`ekfQxy`/`ekfQtheta`/
`ekfROtosXy`/`ekfROtosTheta`; `MotorConfigPatch.travel_calib` + `side` <-
`ml`+`mr` (one field, disambiguated by the new `BoundMotorSide` selector,
per Decision 5); `MotorConfigPatch.kp/ki/kff/i_max/kaw` (5 fields) <-
`pid.kp/ki/kff/iMax/kaw`; `PlannerConfigPatch.min_speed` <- `minSpeed`;
`ConfigDelta`/`ConfigSnapshot`'s bare `watchdog` oneof arm <- `sTimeout`
(routed to `bb.streamWatchdogWindowIn`, per Open Question 4 -- documented in
both `config.proto`'s `ConfigTarget` doc comment and inline on
`ConfigDelta.watchdog`/`ConfigSnapshot.watchdog` in envelope.proto). No key
outside this list is exposed. Flagged, not silently added: this ticket does
NOT attach `(min)`/`(max)`/`(abs_max)` options replicating
`validateCandidate()`'s three business-rule invariants (`trackwidth > 0`,
`rotational_slip` in `{0} ∪ [0.5, 1.0]` -- a non-contiguous domain no single
`(min)`/`(max)` pair can express anyway --, `sTimeout > 0`) -- no existing
config-plane proto message in this tree (including the FULL generated
`DrivetrainConfig`/`MotorConfig`/`PlannerConfig`) uses those options
anywhere; they are reserved in this tree for motion-command fields
(motion.proto). See `config.proto`'s own file-header "Validation note" for
the full rationale; ticket 004 inherits this as a known, documented gap.

**Traceability -- telemetry.proto**: grepped `source/telemetry/tlm_frame.h`'s
`TlmFrameInput` and `motion_commands.cpp`'s `handleTlm()` directly. Every
`Telemetry` field maps 1:1: `now`/`mode`/`seq` <- `TlmFrameInput.now/mode/seq`;
`has_enc`+`enc_left`/`enc_right` <- `hasEnc`+`encLeft`/`encRight`;
`has_vel`+`vel_left`/`vel_right` <- `hasVel`+`velLeft`/`velRight`;
`has_cmd_vel`+`cmd_vel_left`/`cmd_vel_right` <- `hasCmdVel`+`cmdVelLeft`/
`cmdVelRight`; `has_pose`+`pose` <- `hasPose`+`pose`; `has_otos`+`otos`+
`otos_connected` <- `hasOtos`+`otos`+`otosConnected`; `has_twist`+`twist` <-
`hasTwist`+`twist`; `acc_left`/`acc_right`/`active`/`conn_left`/`conn_right`/
`glitch_left`/`glitch_right`/`ts_left`/`ts_right` <- `handleTlm()`'s own
`accL`/`accR`/`dt.busy`/`b.motors[0].connected`/`b.motors[1].connected`/
`glitchL`/`glitchR`/`tsL`/`tsR` (same Blackboard cells `handleTlm()` already
reads: `bb.drivetrain.acc()`/`.busy`, `bb.motors[].connected`/
`.enc_glitch_count`/`.sampled_at`). No invented field.

**186-byte budget -- Telemetry trim outcome**: the first draft (full field
set, including `has_enc_pose`/`enc_pose`) put the generated
`ReplyEnvelope` worst case at 192B (`tlm` arm alone = 186B; +6B non-oneof
`corr_id` = 192B > 186B cap) -- confirmed by `gen_messages.py`'s own
`kMaxEncodedSize` report, NOT hand estimate (095 Decision 6's lesson,
reapplied). Applied Decision 6's documented trim order, FIRST step only:
dropped `has_enc_pose`/`enc_pose` (reconstructable from `enc_left`/
`enc_right` + `twist`). Result: `tlm` arm = 165B, `ReplyEnvelope` total =
171B -- under the 186B cap with 15B of headroom, so Decision 6's SECOND
trim step (`has_otos`/`otos`/`otos_connected`) was NOT needed; those fields
remain in the final schema. Final `Telemetry` field list is exactly what
`telemetry.proto` declares (28 fields): `now`, `mode`, `seq`,
`has_enc`/`enc_left`/`enc_right`, `has_vel`/`vel_left`/`vel_right`,
`has_cmd_vel`/`cmd_vel_left`/`cmd_vel_right`, `has_pose`/`pose`,
`has_otos`/`otos`/`otos_connected`, `has_twist`/`twist`, `acc_left`/
`acc_right`/`active`/`conn_left`/`conn_right`/`glitch_left`/`glitch_right`/
`ts_left`/`ts_right`. Final generated report (`gen_messages.py` stderr):
`CommandEnvelope: ... config=36B, get=4B, stream=10B ... (worst=id=162B) +
non-oneof=6B => total=168B`; `ReplyEnvelope: ... tlm=165B, cfg=38B ...
(worst=tlm=165B) + non-oneof=6B => total=171B`. Both `<=186`; the generated
`static_assert`s in `source/messages/wire.h` hold (confirmed by a green
`just build-sim`).

**Verification run**:
```
python scripts/gen_messages.py --dry-run   # OK, zero diff to unrelated headers
python scripts/gen_messages.py             # OK
just build-sim                             # OK (regenerates pb2 too)
uv run python -m pytest tests/sim -q       # 469 passed in 79.07s (0:01:19)
```
`tests/sim/unit/test_wire_differential.py` and `tests/sim/unit/test_binary_channel.py`
(095's gates) re-run explicitly alongside `test_wire_codec.py`: 156 passed,
no regressions.

## Testing

- **Existing tests to run**: full `tests/sim` suite (`uv run python -m
  pytest tests/sim`), 095's differential/fuzz/range suite specifically
  (`test_wire_differential.py`, `test_binary_channel.py`).
- **New tests to write**: none in this ticket — differential coverage for
  the new messages is ticket 006's job. This ticket only needs the
  generator's own dry-run + static_assert to pass.
- **Verification command**: `python scripts/gen_messages.py --dry-run &&
  just build-sim && uv run python -m pytest tests/sim`
