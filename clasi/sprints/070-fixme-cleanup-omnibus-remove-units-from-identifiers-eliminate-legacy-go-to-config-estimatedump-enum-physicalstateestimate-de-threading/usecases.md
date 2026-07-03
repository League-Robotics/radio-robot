---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 070 Use Cases

Scope note: this sprint is a pure internal refactor (no behavioral change). Its
"use cases" are contracts that must be preserved byte-identically, plus one
maintainer-facing cleanup action. Where a sprint change narrows an existing
top-level UC in `docs/usecases.md`, that UC is cited as Parent; those parent
UCs predate protocol v2 (still describe `SO`/`SZ`/`K*` verbs) and are already
known-stale (068's Open Question 4 flagged the same gap for UC-006) — this
sprint does not repair that staleness, it only narrows the parts it touches.

The units-rename issue (`remove-units-from-identifier-names.md`) is
recommended for a separate sprint 071 (see architecture-update.md, Open
Questions) and is NOT covered by any SUC below.

---

## SUC-001: Maintainer eliminates dead legacy go-to tolerance config

Parent: UC-015 (Drive to Relative XY Position), UC-014 (Tune Calibration
Parameters at Runtime)

- **Actor**: Firmware maintainer (source-level change, not a runtime actor)
- **Preconditions**: `RobotConfig::turnThresholdMm`/`doneTolMm` and their
  `turnThr`/`doneTol` SET/GET keys exist but have zero live consumers (Planner
  reads `turnInPlaceGate`/`arriveTolMm` for actual go-to behavior instead —
  confirmed by sprint 067's independent audit and re-confirmed this sprint by
  direct grep: no `getTurnThreshold()`/`getDoneTol()` call exists anywhere).
- **Main Flow**:
  1. Maintainer removes `turnThresholdMm`/`doneTolMm` from `RobotConfig`
     (`Config.h`), their defaults (`DefaultConfig.cpp`), their `msg::PlannerConfig`
     projection (`PlannerConfig.cpp`, `protos/planner.proto`), and their
     `ConfigRegistry` rows (`turnThr`, `doneTol`).
  2. `SET turnThr=...`/`SET doneTol=...` now reply `ERR badkey` (was previously
     accepted and silently ignored, since nothing consumed the value); `GET`
     dumps no longer include these two keys.
  3. Maintainer updates the two existing backward-compat regression tests
     (`test_legacy_turnThr_still_present`, `test_legacy_doneTol_still_present`)
     to assert the new `ERR badkey` behavior, and regenerates the golden
     default-config fixture.
- **Postconditions**: Live go-to behavior (`G` command pre-rotate / arrival
  gating via `turnInPlaceGate`/`arriveTolMm`) is byte-identical. The two dead
  keys are gone from the wire vocabulary. This deliberately reverses a sprint
  011 decision to retain the keys for calibration-script back-compat — see
  architecture-update.md Design Rationale for why that reversal is judged safe
  now (zero confirmed consumers, and the FIXME marker removed is itself a
  stakeholder-authored instruction to eliminate the legacy fields).
- **Acceptance Criteria**:
  - [ ] `grep -rn "turnThresholdMm\|doneTolMm\|\"turnThr\"\|\"doneTol\"" source/`
        (excluding comments/docs) returns nothing.
  - [ ] `SET turnThr=1` / `SET doneTol=1` reply `ERR badkey`.
  - [ ] Full test suite green; TLM output byte-identical (this change touches
        no TLM field).
  - [ ] `docs/protocol-v2.md`, `docs/design/message-inventory.md`,
        `docs/overview.md`, `docs/architecture.md` updated to drop/replace the
        two keys.

---

## SUC-002: Diagnostic operator reads three pose estimates via `DBG EST`

Parent: UC-006 (Query and Zero Dead-Reckoning Odometry)

- **Actor**: Bench/diagnostic operator (or automated test) issuing `DBG EST`
- **Preconditions**: Firmware built with `EstimateDump::source` as
  `enum class EstimateSource { Encoder, Optical, Fused }` instead of
  `const char*`.
- **Main Flow**:
  1. Operator sends `DBG EST`.
  2. `handleDbgEst()` calls `dumpEstimates()`, which fills three
     `EstimateDump` slots tagged `EstimateSource::Encoder/Optical/Fused`.
  3. At the single emit point, a `toString(EstimateSource)` mapping converts
     each tag to `"enc"`/`"otos"`/`"fuse"` for the `snprintf` call.
  4. Firmware replies with the same three `EST <tag> x=.. y=.. h=.. ...` lines
     as before, followed by `OK dbg est`.
- **Postconditions**: Reply text is byte-identical to pre-refactor output.
  `EstimateDump::source` is now compile-time-checked (no stray string typo
  possible) and the string mapping exists in exactly one place.
- **Acceptance Criteria**:
  - [ ] `DBG EST` reply text byte-identical before/after (verified by existing
        `DBG EST` test coverage, or new coverage if none exists).
  - [ ] No other file constructs an `EstimateDump` with a raw string literal.

---

## SUC-003: Maintainer clears the FIXME backlog down to the units-rename set

Parent: none (process/quality use case internal to this sprint's own issue)

- **Actor**: Firmware maintainer
- **Preconditions**: `grep -ri FIXME source/` currently lists ~14 markers:
  legacy-config (SUC-001), the enum (SUC-002), 8 units-suffix markers
  (deferred to sprint 071), 2 historical non-live references
  (`StopCondition.cpp`, `ColorUtil.cpp`), and 2 previously-untracked markers
  found during this sprint's planning (`ArgSchema.h`'s `ArgKind`/`ArgType`
  duplication question, `OutputState.h`'s undocumented `digitalDirty`/
  `analogDirty` flags).
- **Main Flow**:
  1. Maintainer resolves the two previously-untracked markers: documents why
     `ArgKind` (schema layer) and `ArgType` (runtime tagged-union layer) are
     intentionally separate rather than merging them (removes the `FIXME`,
     replaces with a resolved rationale comment); documents that
     `digitalDirty`/`analogDirty` are currently dead (no producer or consumer
     found anywhere in `source/`), same disposition as sprint 067's "document
     dead keys, don't fix them."
  2. Maintainer rewords the two historical references
     (`StopCondition.cpp:20`, `ColorUtil.cpp:4`) so they no longer contain the
     literal string `FIXME` (they already describe an already-resolved issue,
     per the issue's own text).
  3. Maintainer verifies `grep -ri FIXME source/` now contains only the 8
     units-suffix markers explicitly tracked in
     `remove-units-from-identifier-names.md`.
- **Postconditions**: Every live `FIXME` remaining in `source/` after this
  sprint is tracked in an issue (the deferred units-rename issue); zero
  untracked markers remain.
- **Acceptance Criteria**:
  - [ ] `grep -ri FIXME source/` output, after this sprint, is exactly the 8
        units-suffix markers cross-referenced in
        `remove-units-from-identifier-names.md`.
  - [ ] No behavior change (both resolved markers are comment-only edits).

---

## SUC-004: Runtime `SET` of kinematics/noise config still reaches the pose estimator live

Parent: UC-014 (Tune Calibration Parameters at Runtime)

- **Actor**: Python host / operator issuing `SET tw=`, `SET rotSlip=`, or any
  `SET ekfQ*`/`ekfR*` key
- **Preconditions**: `PhysicalStateEstimate` no longer takes a
  `HardwareState&`/`ActualState&` parameter on any method; trackwidth and
  rotational slip are supplied via a new `setKinematics(trackwidthMm,
  rotationalSlip)` call instead of as `addOdometryObservation()` parameters.
- **Main Flow**:
  1. Operator sends `SET tw=130` (or `rotSlip=`, or any `ekfQ*`/`ekfR*` key).
  2. `ConfigRegistry` commits the value into `Robot::config` (the single live
     `RobotConfig` instance) exactly as before — no registry change.
  3. On the very next `Drive::tickUpdate()`, `Drive` reads the now-updated
     `_robCfg.trackwidthMm`/`.rotationalSlip` (still a live reference, per
     sprint 067) and calls `_est.setKinematics(...)` unconditionally, every
     tick — the same cadence at which `Drive` already read these fields today,
     just relayed through a named setter instead of positional observation
     parameters. EKF noise fields continue to reach `PhysicalStateEstimate`
     via the unchanged `setNoise()` path built in sprint 067.
  4. The next `addOdometryObservation()` call uses the freshly-set kinematics.
- **Postconditions**: `SET tw=`/`SET rotSlip=` take effect on the next control
  tick — identical timing to pre-refactor behavior (sprint 067's live-update
  guarantee is preserved, not just re-tested).
- **Acceptance Criteria**:
  - [ ] A sim test asserts `SET tw=<x>` changes `Odometry::predict()`'s next-
        tick output by the expected amount (mirrors 067's own regression
        methodology — fresh `Sim()`/`ZERO enc`, not a bare `ZERO`).
  - [ ] `SET ekfQxy=...`/etc. still propagate via `Drive::configure()` →
        `_est.setNoise()`, unchanged from sprint 067.

---

## SUC-005: Three-pose telemetry stays byte-identical across the de-threading

Parent: UC-006 (Query and Zero Dead-Reckoning Odometry)

- **Actor**: TestGUI / any TLM consumer
- **Preconditions**: `PhysicalStateEstimate`'s observation methods
  (`addOdometryObservation`, `addOtosObservation`, `resetPose`, `zero`) now
  take explicit `PoseEstimate&` output parameters per call instead of a
  `HardwareState&`, because investigation found `resetPose`/`zero` have two
  independent live call sites with two different destinations (`Drive`'s
  private `_hw` via the message-contract path, and `Robot::state.actual` via
  the legacy `SI`/`OV`/`ZERO pose` command path) that must remain distinct.
- **Main Flow**:
  1. `Drive::tickUpdate()` calls `_est.addOdometryObservation(_hw.encMm[1],
     _hw.encMm[0], now, _hw.encoder, _hw.fused)` (was:
     `addOdometryObservation(_hw, trackwidth, rotSlip, now)`).
  2. `Drive::tickUpdate()`'s OTOS branch calls
     `_est.addOtosObservation(p.x, p.y, p.h, vel.v_mmps, vel.omega_rads, 0.0f,
     now, _hw.optical, _hw.fused)`.
  3. `LoopTickOnce.cpp`'s existing per-tick sync (STEP 2b, unchanged) copies
     `drive.state()`'s `fused`/`encoder`/`optical` fields into
     `Robot::state.actual`, exactly as before.
  4. `SystemCommands::handleSI`/`handleZero` call `_est.resetPose(...)`/
     `_est.zero(...)` directly against `robot->state.actual.{encoder,fused}`,
     using the SAME shared `Odometry` instance (so `_prevEncL/R`/`_ekf`
     internal state resets once, as today) — `SI`'s existing dual-path
     staging (`drive.apply(SetPose)` also resetting `_hw` independently) is
     unchanged.
- **Postconditions**: `encpose=`/`otos=`/`pose=` TLM fields and the golden TLM
  capture are byte-identical to pre-refactor output; `DBG OTOS`, `SI`, `OV`,
  `ZERO pose`/`ZERO enc` behave identically.
- **Acceptance Criteria**:
  - [ ] `tests/_infra/golden_tlm_capture.json` requires no regeneration (no
        TLM field/format change).
  - [ ] Full test suite green, including `test_sim_otos_lever_arm.py`,
        `test_ekf_odometry_commands_coverage.py`, and the `SI`/`ZERO`
        command tests.
