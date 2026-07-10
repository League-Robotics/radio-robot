---
id: '004'
title: BinaryChannel config and get arms
status: done
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

- [x] Every one of the 15 keys in `config_commands.cpp`'s `kAllKeys`
      round-trips (`config` then `get` on the matching target) correctly
      over the binary path.
- [x] `ml`/`mr` (per-side `travel_calib`) address the correct bound motor
      independently; `pid.kp`/`ki`/`kff`/`iMax`/`kaw` apply to BOTH bound
      motors identically, mirroring `applyConfigKey()`'s existing
      both-sides behavior (Decision 5).
- [x] `sTimeout` posts to `bb.streamWatchdogWindowIn`, not `bb.configIn`
      (Open Question 4) — verified by a test that sets it over binary and
      confirms the watchdog window changed, not any `Rt::ConfigDelta`
      target.
- [x] An out-of-range or malformed field yields a typed `Error{code,
      field}`, never a crash, never a silent drop — via the generated
      decoder's `min`/`max`/`abs_max`/`req` checks, not hand-written range
      checks.
- [x] `get{target}` replies exactly one `ConfigSnapshot` for that target;
      no multi-reply behavior is introduced.
- [x] Full sim suite (~469 tests) stays green; 095's differential codec
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

## Completion Notes

**Implementation**: `source/commands/binary_channel.cpp`'s `CONFIG`/`GET`
arms replace the two `ERR_UNIMPLEMENTED` stubs exactly per the Approach
above -- one hand-written `if (patch.field.has) { ...; mask |= bitOf(...); }`
per field per Patch type, `MotorConfigPatch.side` disambiguating
`travel_calib` only (two `Rt::ConfigDelta{kMotor}` posts for any present
Gains field), `watchdog` posted to `bb.streamWatchdogWindowIn` (never
`bb.configIn`). `GET` reads `bb.drivetrainConfig`/`bb.motorConfig[leftIdx|
rightIdx]`/`bb.plannerConfig`/`bb.streamWatchdogWindow` and replies exactly
one `ConfigSnapshot`. `b.configIn.post()`/two-post failures reply
`ERR_FULL`; an empty `ConfigDelta.patch` replies `ERR_UNKNOWN` field=6; an
out-of-range `ConfigGet.target` enum (no wire-level bound exists for it,
per config.proto's own note) replies `ERR_UNKNOWN` field=1; a missing
`ConfigGet.target` is caught by the generated decoder's `(req)` check
(`ERR_BADARG` field=1) before dispatch ever reaches this file.

**Test-infrastructure finding (flagged, not silently patched)**: neither
`main.cpp` nor `tests/_infra/sim/sim_api.cpp`'s `SimHandle` instantiates a
live `Rt::Configurator` (dormant since sprint 093/094's loop rewrite --
`bb.configIn`'s only consumer, `Configurator::applyOne()`, is never called
anywhere in the runtime; `bb.streamWatchdogWindowIn` likewise has zero
consumers anywhere in `source/`). architecture-update.md's own "Rt::
Configurator: zero changes... it already folds whatever lands on
bb.configIn" language assumes a live Configurator exists; it does not, in
either the real firmware or the previous `sim` test fixture. Without a
drain, `config` then `get` cannot round-trip: `get` reads the published
`bb.*Config` cell, which nothing ever updates. Resolved via a TEST-ONLY
addition to `tests/_infra/sim/sim_api.cpp`'s `SimHandle` (`configurator`/
`poseEstimator` members, both already-linked-but-previously-unused in this
shared library; a `drainConfig()` helper called after `sim_tick()`'s and
`sim_command_on()`'s existing tick, never in `sim_route_no_tick()`) that
instantiates the real, unmodified `Rt::Configurator` class -- rather than
duplicate its field-masked fold logic a third time, or have `BinaryChannel`
write `bb.*Config` cells directly (which would bypass `Drivetrain::
configure()`/`Hal::Motor::configure()`/`PoseEstimator::configure()` and
ship a `config` arm that updates what `get` reports without ever reaching
the simulated hardware). `main.cpp` and `runtime/configurator.{h,cpp}` are
untouched -- production Configurator wiring remains a future ticket's
decision, unchanged by this one. See `sim_api.cpp`'s own `SimHandle` class
comment and `drainConfig()`'s doc comment for the full rationale. No
`gen_messages.py`/`config_commands.cpp`/`command_router.cpp` changes.

**Verification**: `just build` (ARM) -- FLASH 326084 B / 364 KB = 87.48%,
RAM 98.33% (expected per project convention, not a regression signal).
`just build-sim` clean. `tests/sim/unit/test_binary_channel.py`: 41 passed
(20 pre-existing + 21 new/updated). Full `tests/sim`: 492 passed (up from
~469 pre-ticket, consistent with 002/003/004's additions).
`test_wire_differential.py`: 132 passed, unregressed.
