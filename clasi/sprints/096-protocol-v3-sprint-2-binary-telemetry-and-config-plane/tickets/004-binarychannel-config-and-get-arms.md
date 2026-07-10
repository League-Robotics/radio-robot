---
id: '004'
title: BinaryChannel config and get arms
status: in-progress
use-cases:
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# BinaryChannel config and get arms

## Description

Implement the `config` and `get` arms in `source/commands/binary_channel.cpp`
(M4), replacing their `ERR_UNIMPLEMENTED` stubs. Depends only on ticket
001 (the schema) — this ticket does NOT depend on the periodic-telemetry
tickets (002/003/005); `bb.configIn`/the Configurator need zero changes
and are indifferent to which command family posts to them.

**Approach — `config` arm**:
1. Decode `msg::ConfigDelta`'s oneof (`drivetrain`/`motor`/`planner`/
   `watchdog`). For the populated arm, hand-translate its present
   (`Opt<T>.has`) fields into a freshly-built `Rt::ConfigDelta{target,
   mask, value}` — one `if (patch.field.has) { cfg.field = patch.field.val;
   mask |= bitOf(...); }` per field, mirroring `applyConfigKey()`'s
   (`config_commands.cpp`) existing per-key assignment shape exactly
   (Decision 3). This is DELIBERATELY hand-written, not a new generator
   emission mode — see architecture-update.md Decision 3's full rationale
   ("generated config merge" is satisfied IN EFFECT: no `strcmp`
   string-keyed dispatch survives on the binary path, and the generated
   decoder's `min`/`max`/`abs_max`/`req` validation already replaces the
   hand range checks — but the field-by-field copy into `Rt::ConfigDelta`
   itself is hand code, the same size/shape as `toSegment()`'s existing
   one-directional copy). Do not attempt to make `gen_messages.py` emit
   this merge — that would couple the generator to `Rt::ConfigDelta`'s
   hand-written `*ConfigField` enums, outside its established boundary.
2. `MotorConfigPatch` (Decision 5): if `travel_calib` is present, post ONE
   `Rt::ConfigDelta{target: kMotor, port: <bound index selected by the
   patch's port/side field>}`. If any of `kp`/`ki`/`kff`/`i_max`/`kaw` is
   present, post TWO `Rt::ConfigDelta{target: kMotor}` entries (one per
   bound index — left AND right), mirroring `applyConfigKey()`'s exact
   both-sides behavior for `pid.*` keys. Do not make the `Gains` fields
   per-side-selectable — that would be new capability beyond what
   `config_commands.cpp` exposes today.
3. `watchdog` (the bare `uint32` oneof arm, `sTimeout`): post to
   `bb.streamWatchdogWindowIn` directly — NOT `bb.configIn` — mirroring
   `handleSet()`'s existing special-case handling of `sTimeout` (it is
   NOT one of the Configurator's four fold targets; see Open Question 4).
   Get this branch right and comment it explicitly, mirroring
   `config_commands.h`'s own file-header note about `sTimeout` being "the
   one key that is NOT one of the Configurator's four targets."
4. Reply with `sendAck` on success (mirroring drive/segment/replace/stop);
   a decode/validation failure already yields a typed `Error{code,field}`
   via the generated decoder — no additional hand-written range checks
   needed on this path.

**Approach — `get` arm**:
1. Decode `msg::ConfigGet.target` (now a `ConfigTarget` enum, ticket 001).
2. Read the CURRENT published config cell matching that target
   (`bb.drivetrainConfig`/`bb.motorConfig[leftIdx or rightIdx]`/
   `bb.plannerConfig`/`bb.streamWatchdogWindow`), populate the matching
   Patch (or bare `uint32` for `CONFIG_WATCHDOG`) into a
   `msg::ConfigSnapshot{target, ...}`, and reply — ONE `ConfigTarget`
   slice per reply (Decision 4). Do NOT build a multi-reply-per-request
   mechanism — a client wanting all slices issues multiple `get` requests
   (pipelineable per 095's corr_id-correlated design); see Open
   Question 1.
3. `CONFIG_MOTOR_LEFT`/`CONFIG_MOTOR_RIGHT` read the bound pair exactly as
   `formatConfigKeyFromBb()` does today (`bb.drivetrainConfig.left_port`/
   `right_port`, converted to 0-based indices at this boundary).

**Files to modify**: `source/commands/binary_channel.cpp`.

## Acceptance Criteria

- [ ] Every one of the 15 keys in `config_commands.cpp`'s `kAllKeys`
      round-trips (`config` then `get` on the matching target) correctly
      over the binary path.
- [ ] `ml`/`mr` (per-side `travel_calib`) address the correct bound motor
      independently; `pid.kp`/`ki`/`kff`/`iMax`/`kaw` apply to BOTH bound
      motors identically, mirroring `applyConfigKey()`'s existing
      both-sides behavior (Decision 5).
- [ ] `sTimeout` posts to `bb.streamWatchdogWindowIn`, not `bb.configIn`
      (Open Question 4) — verified by a test that sets it over binary and
      confirms the watchdog window changed, not any `Rt::ConfigDelta`
      target.
- [ ] An out-of-range or malformed field yields a typed `Error{code,
      field}`, never a crash, never a silent drop — via the generated
      decoder's `min`/`max`/`abs_max`/`req` checks, not hand-written range
      checks.
- [ ] `get{target}` replies exactly one `ConfigSnapshot` for that target;
      no multi-reply behavior is introduced.
- [ ] Full sim suite (~469 tests) stays green; 095's differential codec
      gate (`test_wire_differential.py`) does not regress.

## Testing

- **Existing tests to run**: full `tests/sim` suite; `test_binary_channel.py`
  (095's existing BinaryChannel sim test, extend rather than duplicate).
- **New tests to write**: sim-level tests posting each of the 15 keys over
  `config` and reading them back via `get`, asserting the resulting
  `bb.drivetrainConfig`/`bb.motorConfig[]`/`bb.plannerConfig`/
  `bb.streamWatchdogWindow` matches what the equivalent text SET would
  have produced (compare against `applyConfigKey()`'s known behavior, not
  by running the unregistered text handler).
- **Verification command**: `just build-sim && uv run python -m pytest
  tests/sim`
