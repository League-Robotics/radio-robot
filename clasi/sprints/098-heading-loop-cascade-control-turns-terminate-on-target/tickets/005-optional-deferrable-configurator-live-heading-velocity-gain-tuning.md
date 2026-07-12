---
id: '005'
title: '[OPTIONAL/DEFERRABLE] Configurator live heading/velocity gain tuning'
status: in-progress
use-cases:
- SUC-003
depends-on:
- '003'
github-issue: ''
issue:
- heading-loop-cascade-control-turns-terminate-on-target.md
- real-robot-motion-calibration-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# [OPTIONAL/DEFERRABLE] Configurator live heading/velocity gain tuning

## ⚠️ OPTIONAL/DEFERRABLE — skip if the overnight run's risk budget is spent

The mandatory path (001→002→003→006) already satisfies the sprint's
acceptance criterion WITHOUT this ticket. Independent of ticket 004 — skip
either, both, or neither without affecting the other. If skipped, ticket
006 notes the deferral and closes the sprint with reflash-based tuning
(ticket 003's own method) as the only tuning path, exactly as sprints
093-097 already operate today.

## Description

Wire a minimal `Rt::Configurator` into `main.cpp`'s live loop so a binary
`SET` config delta actually reaches the running `Drivetrain`/`Hardware`,
cutting heading/velocity gain-tuning iteration from a reflash (~5 minutes)
to a live `SET` (seconds). Additive only — boot config still applies once,
directly, at construction, exactly as today; this does NOT reintroduce
093/094-era full runtime config authority.

Reference: `architecture-update.md` M7, SUC-003. `real-robot-motion-
calibration-undershoot.md`'s "Also discovered" section is the origin of
this gap: binary `SET` already acks into `bb.configIn` (ticket 096), but
nothing has drained it since 093/094 removed runtime config authority.

Depends on 003 — tune against the bench-verified Stage 1 baseline, not a
moving target.

## Acceptance Criteria

- [x] `main.cpp` constructs one `Rt::Configurator`, seeded from the SAME
      boot `msg::DrivetrainConfig`/`msg::PlannerConfig` values already
      passed directly to `drivetrain.configure()`/
      `drivetrain.configureMotion()` at construction — boot behavior is
      PROVABLY unchanged (a freshly booted robot with no `SET` ever sent
      behaves identically to today).
- [x] `main.cpp`'s loop calls `configurator.applyOne(bb)` once per pass
      (mirroring the pre-093/094 pattern) — placed so it drains at most one
      `bb.configIn` delta per pass, matching `Configurator::applyOne()`'s
      own documented one-delta-per-call contract.
- [x] `Rt::Configurator::applyOne()`'s existing `kPlanner` case gains ONE
      new line: `drivetrain_.configureMotion(plannerConfig_);` immediately
      after the `foldPlanner(...)` call, alongside the existing
      `bb.plannerConfig = plannerConfig_;` publish — today that case only
      folds+publishes (a residue of ticket 094-002 relocating
      `Subsystems::Planner` out of `source/`); `Subsystems::Drivetrain` is
      the correct live target now (the Configurator already holds a
      `Drivetrain&`).
- [x] `kMotor`/`kDrivetrain`/`kOdometer`'s existing, already-correct
      fold-and-apply paths are UNCHANGED — this ticket touches the
      `kPlanner` case only.
- [x] SIM ACCEPTANCE: a new scenario drives a `SET`-equivalent config
      delta for `heading_kp` mid-session (via `bb.configIn`/whatever the
      sim harness's existing config-delta injection surface is) and
      confirms the VERY NEXT segment's commanded twist reflects the new
      gain — no restart, no reflash-equivalent.
- [x] Full `uv run python -m pytest` stays green, no regression.
- [ ] HARDWARE ACCEPTANCE: a bench session sends a live `SET` for
      `heading_kp` (or `heading_kd`) over serial/relay and confirms (via
      `TLM`/a subsequent `turn_sweep.py` cell) the change took effect
      WITHOUT a reflash. **NOT YET DONE — reserved for the team-lead's
      hardware pass.** The wire-schema gap noted below (2026-07-11 pass) is
      now CLOSED (2026-07-12 pass, see "Implementation Notes — wire path"
      below): `PlannerConfigPatch.heading_kp`/`heading_kd` are now real wire
      fields, `binary_channel.cpp` decodes them, and
      `host/robot_radio/robot/protocol.py`'s `set_config(headingKp=...)`/
      `get_config("headingKp")` reach them — a real bench `SET headingKp=...`
      should now work. This bullet is the team-lead's hardware confirmation
      of that, not a remaining software gap.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`.
- **New tests to write**: the live-`SET`-changes-live-behavior sim
  scenario itemized above.
- **Verification command**: `uv run python -m pytest`; a bench
  `SET heading_kp=<value>` followed by an immediate re-run of one
  `turn_sweep.py` cell as the hardware confirmation.

## Implementation Plan

**Approach**: Construct-and-tick the existing `Rt::Configurator` class
(already fully implemented, just never instantiated in `main.cpp` since
093/094) plus the one-line `kPlanner` fix.

**Files to modify**: `source/main.cpp`, `source/runtime/configurator.cpp`.

**Files to create**: none.

**Testing plan**: as above.

**Documentation updates**: none required structurally.

## Implementation Notes (programmer, post-implementation)

The `Rt::Configurator` shape the architecture/ticket describes was verified
CORRECT by direct read (`source/runtime/configurator.{h,cpp}`): constructor
signature, `applyOne(bb)`'s one-delta-per-call contract, and the `kPlanner`
case's fold-only (no live-apply) shape all matched exactly. Three places
needed judgment calls or turned out to differ from what the ticket assumed:

1. **`PlannerConfigField`/`foldPlanner()` did not yet cover `heading_kp`/
   `heading_kd` at all** (verified: `source/runtime/commands.h`'s
   `PlannerConfigField` enum stopped at `kMinSpeed`, and
   `configurator.cpp`'s `foldPlanner()` had no matching fold lines) — a real
   gap beyond the ticket's stated one-line `kPlanner` fix, since without a
   mask bit + fold line, a `heading_kp` `Rt::ConfigDelta` folds to nothing
   and the SIM ACCEPTANCE criterion is unsatisfiable. Added `kHeadingKp`/
   `kHeadingKd` to the enum and two fold lines to `foldPlanner()` — a
   mechanical, minimal extension of the SAME existing pattern (every other
   `PlannerConfig` field already has exactly this), confined to the
   `kPlanner` fold path only (`kMotor`/`kDrivetrain`/`kOdometer` untouched,
   per the ticket's own boundary). Documented inline at both sites.
2. **The binary wire `config`/`SET` command cannot carry `heading_kp`/
   `heading_kd` at all, independent of (1)** — verified:
   `commands/binary_channel.cpp`'s `handleConfigPlanner()` only forwards
   `msg::PlannerConfigPatch.min_speed`; that generated wire-message type
   (`protos/config.proto`) declares only that one field. This is a SEPARATE,
   larger gap (a `protos/config.proto` schema change + regen + a
   `binary_channel.cpp` edit) that this ticket's stated file scope
   (`main.cpp`, `configurator.cpp`) does not cover and this implementation
   did NOT attempt. The SIM ACCEPTANCE scenario therefore injects the
   `Rt::ConfigDelta` directly via `bb.configIn.post()` — exactly what the
   ticket text sanctions ("via `bb.configIn`/whatever the sim harness's
   existing config-delta injection surface is") and exactly how scenarios
   1-9 in `configurator_harness.cpp` already inject every other target's
   deltas — rather than round-tripping through the wire `config` command.
   **This means the HARDWARE ACCEPTANCE bullet cannot be satisfied with a
   real `SET heading_kp=...` over serial/relay as written today** — the
   wire schema has no field to carry it. Flagged here for the team-lead to
   triage (scope call: extend this ticket, open a follow-on ticket/issue, or
   accept reflash-only tuning per the ticket's own "OPTIONAL/DEFERRABLE...
   exactly as sprints 093-097 already operate today" fallback) — no issue
   file created by this implementation pass; that decision belongs to the
   team-lead, not the ticket's implementer.
   **UPDATE (2026-07-12 follow-up pass, requested by the team-lead after
   hardware confirmed the Configurator wiring itself doesn't regress
   turns): this gap is now CLOSED — see "Implementation Notes — wire path
   (2026-07-12)" below.**
3. **`main.cpp` had zero `Subsystems::PoseEstimator` instance** — `Rt::
   Configurator`'s constructor requires one (a `kDrivetrain`-scoped delta
   re-propagates to it). Added `static Subsystems::PoseEstimator
   poseEstimator;`, constructed but never ticked (Stage 2/M6's OTOS wiring,
   ticket 098-004, is independent and not landed on this branch) — inert,
   since `PoseEstimator` holds no hardware reference. Mirrors `tests/_infra/
   sim/sim_api.cpp`'s own `SimHandle` (096-004), which added the identical
   instance for the identical reason.
4. **Loop placement**: `configurator.applyOne(bb)` is called right after
   `router.route()` and before `tickTelemetry()`/`hardware.tick()`/
   `drivetrain.tick()` — so a delta routed THIS pass is already live before
   this SAME pass's `drivetrain.tick()` runs (one tick sooner than draining
   after the commit step). Documented inline in `main.cpp` at the call site
   and in the file's header comment.
5. Also added `configurator.publish(bb)` at boot (right after the existing
   `bb.drivetrainConfig = dtConfig;` seed) — mirrors `sim_api.cpp`'s own
   boot sequence, fills in `bb.motorConfig[]`/`bb.plannerConfig`/
   `bb.odometerConfig` (previously always zero-valued in `main.cpp`, never
   set by anything) with the real boot values. Pure telemetry/`GET`-
   visibility fix (`publish()` never calls any subsystem's `configure()`),
   not a control-loop behavior change.

Verification: `just build-sim` and `just build-clean` both succeed;
`uv run python -m pytest tests/sim tests/unit` is 896 passed before and
after (the new scenario is scenario 10 inside `configurator_harness.cpp`,
exercised by the existing single `test_configurator.py` pytest wrapper, so
the collected-test COUNT is unchanged by design — the scenario itself was
directly compiled/run standalone and confirmed passing, then reconfirmed via
the pytest wrapper).

## Implementation Notes — wire path (2026-07-12 follow-up)

Requested by the team-lead after confirming on hardware that the
Configurator wiring itself does not regress turns (12-turn sweep, all
within ±0.9°): close the wire-schema gap from note 2 above so a real
`SET headingKp=...`/`SET headingKd=...` over serial/relay can reach
`bb.configIn` at all.

**Files touched** (beyond `main.cpp`/`configurator.cpp`, already done):
`protos/config.proto` (+2 fields), `source/messages/config.h` (regenerated,
never hand-edited), `host/robot_radio/robot/pb2/config_pb2.py`
(regenerated), `source/commands/binary_channel.cpp` (`handleConfigPlanner()`
+ `handleGet()`'s `CONFIG_PLANNER` arm), `host/robot_radio/robot/protocol.py`
(`_PLANNER_KEYS` + a latent bug fix, see below), `host/robot_radio/io/proxy.py`
(the SAME latent bug fix), `scripts/check_config_sync.py` +
`scripts/config_sync_allowlist.json` (new fields registered, see below), plus
test-suite updates: `tests/sim/unit/wire_differential_harness.cpp`,
`tests/sim/unit/_wire_diff_driver.py`, `tests/sim/unit/test_wire_differential.py`,
`tests/sim/unit/test_wire_fuzz.py`, `tests/sim/unit/test_binary_channel.py`,
`tests/unit/test_protocol_binary_client.py`.

**The exact wire-decode site**: `source/commands/binary_channel.cpp`'s
`handleConfigPlanner(const msg::PlannerConfigPatch& p, ...)` (originally
lines 371-384) — mirrored `min_speed`'s own `if (p.FIELD.has) { delta.planner.
FIELD = p.FIELD.val; delta.mask |= Rt::bitOf(Rt::PlannerConfigField::kFIELD); }`
shape exactly, two more times, for `heading_kp`/`heading_kd`. This is the
SAME `Rt::ConfigDelta` this ticket's earlier pass already taught
`foldPlanner()`/`Configurator::applyOne()` to fold and live-apply — no
change needed there this pass.

**Deviations/extra findings beyond the coordinator's 5 numbered steps**:

1. **`msg::PlannerConfigPatch`'s field-count growing past 1 exposed a
   pre-existing latent bug** in two places that read a `ConfigSnapshot`
   back: `host/robot_radio/robot/protocol.py`'s `_read_config_snapshot_value()`
   and `host/robot_radio/io/proxy.py`'s `_raw_config_snapshot_value()` both
   hardcoded `snapshot.planner.min_speed` in their `if key in _PLANNER_KEYS`
   branch (never `getattr(..., _PLANNER_KEYS[key])`, unlike the
   `_DRIVETRAIN_KEYS`/`_MOTOR_PID_KEYS` branches right above, which were
   already generic) — harmless while `_PLANNER_KEYS` had exactly one entry,
   but adding `headingKp`/`headingKd` to that dict (needed for
   `set_config()`'s kwarg mapping, step 4) would have made EVERY planner GET
   key silently read back `min_speed`'s value. Fixed both to the generic
   `getattr()` form scenarios elsewhere in each file already use.
2. **`handleGet()`'s `CONFIG_PLANNER` snapshot arm did NOT populate
   heading_kp/heading_kd** — not explicitly asked for in the coordinator's 5
   steps, but required to avoid (1) becoming a live footgun: since
   `_PLANNER_KEYS` is shared between `set_config()`/`get_config()`
   (`protocol.py`'s own established one-table-both-directions pattern, which
   the coordinator's step 4 said to "follow... exactly"), leaving `handleGet()`
   un-extended would make `get_config("headingKp")` silently return `0`
   forever regardless of the live value. Added two lines mirroring
   `min_speed`'s own always-`has=true` read-back.
3. **`scripts/check_config_sync.py`'s `PATCH_TO_PYDANTIC` map** (a forced-fail
   gate, not allowlistable by itself) required registering the two new
   `PlannerConfigPatch` fields with an empty target list (`[]`, "no
   host-side pydantic field", mirroring `min_speed`'s own existing entry)
   PLUS a matching `scripts/config_sync_allowlist.json` justification entry
   — this script's own `test_check_config_sync.py` caught the omission
   immediately (a `FAIL unmapped-patch-field`, correctly not allowlistable
   per the script's own design: "edit PATCH_TO_PYDANTIC instead").
4. **A cluster of PINNING tests broke on the schema change, by design** —
   `test_wire_differential.py`'s `test_field_numbers_match_pb2_descriptors_
   096_006_new_messages` hardcodes the exact expected field-number layout
   per message (updated: `{"min_speed": 1, "heading_kp": 2, "heading_kd": 3}`);
   `wire_differential_harness.cpp`'s `encode_cfg_planner`/decode-print CLI
   verbs hand-transcribe one field at a time (extended both, plus every
   Python call site: `_wire_diff_driver.py`, `test_wire_differential.py`
   Direction A/B planner tests — added a NEW partial-presence test,
   `test_direction_a_config_planner_only_heading_kp`, mirroring the existing
   motor-only-`travel_calib` precedent — and `test_wire_fuzz.py`'s two
   `encode_cfg_planner(...)` extreme/ordinary-value call sites);
   `test_protocol_binary_client.py`'s `test_get_config_no_keys_dumps_all_
   five_targets` hardcodes the full expected `get_config()` dump dict
   (updated the fixture + expected dict together). None of this was a
   surprise once found — `test_wire_differential.py`'s own module docstring
   says outright: "A future change to `wire_runtime.{h,cpp}` or the
   generated `wire.{h,cpp}` that breaks a test in this file is a BLOCKING
   regression — fix the codec, do not xfail/skip a real disagreement with
   `google.protobuf`" — exactly what happened, and exactly what was done.

**New end-to-end coverage** (step 5): `tests/sim/unit/test_binary_channel.py`'s
new `test_binary_config_heading_kp_kd_round_trip_reaches_live_drivetrain`
sends a REAL `*B`-armored `CommandEnvelope` through `sim.command_on()` —
`Rt::CommandRouter` → `BinaryChannel::handleConfig()` →
`handleConfigPlanner()` → `bb.configIn` → the sim's own `Rt::Configurator`
(TEST-ONLY, 096-004, drains to exhaustion after every `sim_command_on()`
call) → `Configurator::applyOne()`'s `kPlanner` case (this ticket's fix) →
`bb.plannerConfig` — then a real `get` envelope reads it back via the
now-extended `handleGet()`. Also proves the field-mask clobber-safety
guarantee (a disjoint `heading_kd`-only delta does not touch the
already-set `heading_kp`) over the SAME real wire path.

**What this does NOT (yet) prove directly**: a segment's commanded twist
changing as a DIRECT observable RESULT of a wire-originated `SET`, in one
single test. That specific link — "reaching `Configurator::applyOne()`'s
`kPlanner` case with a real `Rt::ConfigDelta` causes the live `Drivetrain`'s
NEXT segment's commanded twist to change" — is already proven by this
ticket's EARLIER `configurator_harness.cpp` scenario 10 (2026-07-11 pass),
using the exact same `Rt::ConfigDelta` shape `handleConfigPlanner()` now
builds from wire input. Chaining that proof with this pass's new wire
round-trip test covers the full path in two overlapping pieces rather than
one single test. A single test that both sends a real wire `SET` AND
observes the resulting segment's commanded twist would need a new
`Drivetrain::state().cmd()`-equivalent read accessor added to
`tests/_infra/sim/sim_api.cpp`'s ctypes ABI (none exists today — only
measured `vel()`/`true_velocity()`, which lag the commanded value by
Hal::SimMotor's own documented one-tick sample latency plus PID transient
response, making them a much noisier, harder-to-get-right signal for this
specific proof). Judged out of scope for a wire-schema-closing ticket
(new shared ABI surface, not a wire-schema change) per the coordinator's own
stated fallback ("If the existing harness can't easily build a wire
envelope, at minimum assert the wire-decode function maps... so the path is
covered end to end") — the two-piece proof already meets that bar. Flagged
here rather than silently declared sufficient.

Verification (this pass): `just build-sim` and `just build-clean` both
succeed. `uv run python -m pytest tests/sim tests/unit -q` — 898 passed (896
baseline + 2 new tests this pass: `test_direction_a_config_planner_only_
heading_kp` and `test_binary_config_heading_kp_kd_round_trip_reaches_live_
drivetrain`); 0 regressions after fixing the 11 tests that broke on the
schema change (see finding 4 above) and `check_config_sync.py` (finding 3).
