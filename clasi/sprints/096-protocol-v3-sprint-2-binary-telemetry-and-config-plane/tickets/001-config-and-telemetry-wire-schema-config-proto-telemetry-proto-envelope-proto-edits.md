---
id: '001'
title: Config and telemetry wire schema (config.proto, telemetry.proto, envelope.proto
  edits)
status: open
use-cases: [SUC-001]
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

- [ ] `protos/telemetry.proto`'s `Telemetry` fields are traceable 1:1 to
      either `Telemetry::TlmFrameInput` (text STREAM/SNAP) or
      `handleTlm()`'s own field computation (`motion_commands.cpp`) — no
      field invents new firmware capability.
- [ ] `protos/config.proto`'s three Patch messages' fields are traceable
      1:1 to `config_commands.cpp`'s `kAllKeys` (15 keys) — no key outside
      that list is exposed.
- [ ] `MotorConfigPatch` has a `port`/side selector that disambiguates
      `travel_calib` only, per Decision 5 — the five `Gains` fields carry
      no per-side selector of their own.
- [ ] `ConfigTarget` has a 4th enumerator for the watchdog window
      (`sTimeout`), distinct from the Configurator's three fold targets
      (drivetrain/motor/planner) — see Open Question 4; document in a
      code comment that this target routes to `bb.streamWatchdogWindowIn`,
      not `bb.configIn`, at the `BinaryChannel` boundary (ticket 004).
- [ ] `python scripts/gen_messages.py --dry-run` succeeds; diff shows only
      the new/edited files above — zero diff to any unrelated
      pre-existing generated header.
- [ ] The generated `static_assert(kMaxEncodedSize<=186)` passes for
      `CommandEnvelope` and `ReplyEnvelope`. If a trim was required to
      make `Telemetry` fit, the final field list and the reason are
      recorded in this ticket's completion notes.
- [ ] `just build-sim` and the full existing sim suite (~469 tests) stay
      green — this ticket changes generated headers only; no existing
      test's behavior should change.
- [ ] 095's differential codec gate (`tests/sim/unit/test_wire_differential.py`
      and friends) still passes — this schema addition must not regress
      the already-implemented arms' codec correctness.

## Testing

- **Existing tests to run**: full `tests/sim` suite (`uv run python -m
  pytest tests/sim`), 095's differential/fuzz/range suite specifically
  (`test_wire_differential.py`, `test_binary_channel.py`).
- **New tests to write**: none in this ticket — differential coverage for
  the new messages is ticket 006's job. This ticket only needs the
  generator's own dry-run + static_assert to pass.
- **Verification command**: `python scripts/gen_messages.py --dry-run &&
  just build-sim && uv run python -m pytest tests/sim`
