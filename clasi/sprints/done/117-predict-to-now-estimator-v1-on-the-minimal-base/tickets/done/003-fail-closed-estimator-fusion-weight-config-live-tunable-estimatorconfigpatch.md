---
id: '003'
title: Fail-closed estimator fusion-weight config + live-tunable EstimatorConfigPatch
status: done
use-cases:
- SUC-058
depends-on:
- '002'
github-issue: ''
issue: predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fail-closed estimator fusion-weight config + live-tunable EstimatorConfigPatch

## Description

Give `App::StateEstimator`'s (ticket 002) fusion weights two entry
points, mirroring the project's established fail-closed config pipeline
and `OtosConfigPatch`'s live-tuning precedent exactly:

1. **Boot-time, fail-closed, baked default.** A new `estimator` section in
   `data/robots/*.json`, read by `gen_boot_config.py` via `_require()` (the
   same hard-fail-if-missing discipline `output_deadband` established in
   sprint 114 — never a silent bench-placeholder substitution), baked into
   `boot_config.cpp` as `Config::defaultEstimatorConfig()`. Per the
   stakeholder's encoder-only-v1 decision, `weight_heading_otos` and
   `weight_omega_otos` are committed as `0.0` in every robot JSON this
   sprint; `staleness_ms` gets a reasoned placeholder value (document the
   reasoning — e.g. a small multiple of the OTOS read cadence).
2. **Live, volatile tuning.** A new `EstimatorConfigPatch` message
   (`config.proto`) and `ConfigDelta.estimator` oneof arm (field **6** —
   the next free number after `reserved 3, 4;` and `otos = 5`), plus a new
   `ConfigTarget.CONFIG_ESTIMATOR = 6` enum value, dispatched by a new
   branch in `RobotLoop::handleConfig()` that merges present fields onto
   `stateEstimator_.setWeights(...)`'s current values (present-field merge,
   same pattern `applyMotorConfigPatch()`/`applyOtosPatch()` already use) —
   **NOT** persisted into `persistedTuning_`/flash (Design Rationale
   Decision 4, overlay `design/design.md`): a reboot reverts to the baked
   JSON default. Host gets a matching `NezhaProtocol.estimator_config(...)`
   builder, mirroring `otos_config()`'s shape.

Depends on ticket 002 for `StateEstimator::setWeights()` to exist. Also
covers the two `DESIGN.md` docs this sprint's overlay could not seed
(only one `DESIGN.md`-basename slot exists per sprint, already claimed by
`src/firm/app/DESIGN.md`) plus `src/host/robot_radio/DESIGN.md`'s own
live-surface note for the new host method — see `sprint.md`'s Design
Overlay section for the full ownership mapping.

## Acceptance Criteria

- [x] `data/robots/robot_config.schema.json` gains an `estimator` object
      (`weight_heading_otos`, `weight_omega_otos`, `staleness_ms`) with
      `firmware.field`/`kind` mappings, matching the existing
      `output_deadband`-style entries' shape.
- [x] `tovez.json`, `togov.json`, `tovez_nocal.json` all gain a committed
      `estimator` section (`weight_heading_otos: 0.0`, `weight_omega_otos:
      0.0`, a documented `staleness_ms`) — no robot JSON is left without
      one.
- [x] `gen_boot_config.py` reads the new keys via `_require()` (hard
      codegen failure on a missing key, mirroring
      `output_deadband_for_config()`'s existing shape) and bakes
      `Config::defaultEstimatorConfig()` into `boot_config.cpp`.
- [x] A test mirroring `test_gen_boot_config_required_keys.py`'s existing
      pattern confirms a robot JSON missing the `estimator` section fails
      codegen loudly.
- [x] `config.proto`: new `EstimatorConfigPatch` message (`optional float
      weight_heading_otos`, `weight_omega_otos`, `staleness_ms`) and
      `ConfigTarget.CONFIG_ESTIMATOR = 6`.
- [x] `envelope.proto`: `ConfigDelta.oneof patch` gains `EstimatorConfigPatch
      estimator = 6;` (verified against the current file: existing arms are
      `drivetrain=1`, `motor=2`, `otos=5`, with `3, 4` already `reserved`
      — `6` is confirmed free).
- [x] `python build.py` (or equivalent) regenerates `msg::EstimatorConfigPatch`
      and the `ConfigDelta` codec cleanly; `kMaxEnvelopeBytes`/
      `kCommandEnvelopeMaxEncodedSize` re-measured and confirmed to still
      fit `kArmoredBufSize` (same discipline sprint 116 ticket 001
      established for `Move`'s own size growth).
- [x] `RobotLoop::handleConfig()` gains a `PatchKind::ESTIMATOR` branch:
      present fields merge onto `stateEstimator_.setWeights()`'s current
      values (absent fields leave the current value untouched — partial-
      patch semantics matching `MotorConfigPatch`/`OtosConfigPatch`); acks
      OK; does NOT touch `persistedTuning_`.
- [x] `NezhaProtocol.estimator_config(*, weight_heading_otos=None,
      weight_omega_otos=None, staleness_ms=None)` added host-side, mirroring
      `otos_config()`'s builder shape.
- [x] `src/firm/messages/DESIGN.md` updated in place: the envelope-arms
      table gains `ConfigDelta.estimator` (field 6) and the new
      `EstimatorConfigPatch`/`ConfigTarget.CONFIG_ESTIMATOR` entries, plus
      updated size figures if `kMaxEnvelopeBytes` changed.
- [x] `src/firm/config/DESIGN.md` updated in place: the fail-closed
      config-key documentation gains the new `estimator` section, matching
      the existing `output_deadband` precedent's write-up shape.
- [x] `src/host/robot_radio/DESIGN.md` updated in place: a note that
      `NezhaProtocol.estimator_config(...)` is a new live-tuning surface,
      mirroring the existing `otos_config()` entry.

## Implementation Plan

**Approach.** Mirror `OtosConfigPatch` end to end — it is the closest
existing precedent for "a small, optional-field patch message, live-
applied inside `handleConfig()`, with a host-side builder." Do not add
these weights to `Config::TuningStore`/`persistedTuning_` (Decision 4):
this keeps `kConfigSchemaVersion` unchanged and avoids a flash-wear/
write-frequency policy question for a knob whose only committed value
this sprint is `0.0`.

**Files to modify:**
- `data/robots/robot_config.schema.json`, `tovez.json`, `togov.json`,
  `tovez_nocal.json` — new `estimator` section.
- `src/scripts/gen_boot_config.py` — `estimator_config_for_config()` (or
  similarly named) helper using `_require()`; wire into
  `Config::defaultEstimatorConfig()`'s generated body.
- `src/firm/config/boot_config.h` — declare `Config::defaultEstimatorConfig()`
  and its return struct.
- `src/protos/config.proto` — `EstimatorConfigPatch` message,
  `ConfigTarget.CONFIG_ESTIMATOR`.
- `src/protos/envelope.proto` — `ConfigDelta.oneof patch`'s new `estimator
  = 6` arm.
- `src/firm/app/robot_loop.cpp`/`robot_loop.h` — `handleConfig()`'s new
  branch; `main.cpp` constructs `StateEstimator` from the baked config
  (coordinate with ticket 004, which owns the actual `main.cpp`/
  `sim_harness.h` wiring — this ticket only needs `Config::
  defaultEstimatorConfig()` to exist and be readable, not necessarily
  consumed yet).
- `src/host/robot_radio/robot/protocol.py` — `estimator_config(...)`.
- `src/firm/messages/DESIGN.md`, `src/firm/config/DESIGN.md`,
  `src/host/robot_radio/DESIGN.md` — direct edits per Acceptance Criteria.

**Documentation updates:** the three `DESIGN.md` files above.

## Testing

- **Existing tests to run**: `src/tests/sim/unit/test_gen_boot_config_required_keys.py`,
  `test_config_gate.py`, `test_app_robot_loop.py`, full
  `uv run python -m pytest`.
- **New tests to write**: fail-closed missing-key codegen test (mirrors
  `test_gen_boot_config_required_keys.py`); `RobotLoop::handleConfig()`
  ESTIMATOR-branch unit test (accept, ack OK, partial-patch merge,
  `persistedTuning_` untouched); host `estimator_config()` envelope-
  building test.
- **Verification command**: `uv run python -m pytest src/tests/sim/unit/`.
