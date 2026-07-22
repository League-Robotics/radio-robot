---
id: '117'
title: Predict-to-now estimator v1 on the minimal base
status: closed
branch: sprint/117-predict-to-now-estimator-v1-on-the-minimal-base
worktree: false
use-cases: []
issues:
- predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 117: Predict-to-now estimator v1 on the minimal base

## Goals

**Re-scope note (carried from the source issue, 2026-07-21):** the
predict-to-now issue's original mechanism — on-chip measurement rings,
ring-dump commands, capture builds — is superseded by the minimal-firmware
gut (sprints 115-116): the tightened telemetry frame, timestamped and
emitted every loop iteration, logged host-side, is now the dataset. What
stands from the source issue and is in scope here: the estimator core
(`whereAmI()`/`stateAt(t)`), wheel + body peer estimates, ZOH v1
extrapolation, and the leave-one-out one-step-ahead RMS validation
methodology — now run over the host TLM log instead of dumped rings. Fake
OTOS, external/camera pose + clock sync, and the remaining-distance
trajectory controller are the issue's further-out goals; they are noted
here as roadmap context but are **not** detail-planned or built in this
sprint (see Out of Scope).

- Build `App::StateEstimator`: `wheelAt(wheel, t)`, `bodyAt(t)`,
  `whereAmI()` (= `bodyAt(now)`), `wheelNow(wheel)`, `reset(x, y,
  heading)`, `innovations()` — wheel and body state as peer first-class
  estimates, each with its own residual stream.
- Ship ZOH v1 extrapolation (`distance = basis.position + basis.velocity
  × age`; heading = fused heading + fused omega × age — the existing
  `headingLead()` equation promoted to full state) plus a v1
  complementary-blend fusion (config-tunable weights, staleness gating).
- Validate with the stakeholder's methodology: leave-one-out one-step-
  ahead RMS analysis, run in sim first and then over a real bench capture
  of the host-logged TLM stream (via sprint 115's `tlm_log.py`), broken
  out by pattern phase (steady, ramp, reversal, pivot).
- Wire the estimator into the loop's kPace block (after
  `applyOtosSample()`/`odom_.integrate()`) without regressing motion
  timing.

## Problem

Weeks of motion-control tuning never produced a completing tour; the two
standing blockers (turn non-termination, terminal straight-leg wedge) were
both terminal-behavior failures of the executor's completion machinery —
now deleted by sprints 115-116. The firmware has never fused measurements
or answered "where is the robot right now" — control consumed raw
per-cycle deltas, and `measurement_ring.h` existed but nothing published
into it. The abandoned predict-to-now arc's own ring-capture/dump plan is
now unnecessary: the post-gut minimal base already emits a complete
timestamped frame every cycle and the host already logs it (sprint 115),
so the estimator can be validated directly against that log.

## Solution

Build `App::StateEstimator` as pure computation over already-published
measurement state — no new on-chip rings, reading from the same frame
data sprint 115 already assembles into `RobotLoop::frame_` each cycle.
Wheel and body estimates are peers, each independently valid/stale. v1 is
plain zero-order-hold extrapolation from the newest sample; fusion v1 is a
simple complementary blend (not an EKF), weights fail-closed and
live-tunable via `handleConfig()`. Validate the way the stakeholder
specified: drive varied motion patterns, capture the TLM log, and for
every measurement k, exclude it, extrapolate from k−1, and compare against
actual k — walking the whole log, per stream — then RMS the one-step-ahead
errors by pattern phase and propagate them through position integration to
project leg-level accumulated error. Run this in sim first (fast
iteration, no hardware risk) and then on the bench. Detailed module
boundaries, exact file layout, and sim/bench sequencing are established at
Detail Mode for this sprint, re-derived from the post-gut/post-protocol
base rather than inherited wholesale from the source issue's original
(ring-oriented) per-stage sprint table.

## Success Criteria

- `App::StateEstimator` compiles into firmware and the host cross-check
  target (`libfirmware_host`), with wheel and body estimates each backed
  by their own residual stream.
- Leave-one-out one-step-ahead RMS analysis runs end-to-end over a real
  bench TLM-log capture spanning steady/ramp/reversal/pivot motion, both
  directions, turns and straights.
- The ZOH lag signature (`a·k` velocity error, `½a·k²` distance error
  during ramps) is checked against theory — this is the evidence that
  decides whether a fit-based (non-ZOH) predictor is warranted later.
- RMS tables and accept thresholds are reviewed and ratified by the
  stakeholder from the real data — not pre-committed before the capture.
- Estimator wired into the cycle at the correct placement with no
  measurable motion-timing regression (encoder tracking vs. commanded
  speed unchanged from pre-estimator runs).
- Sim and bench notebook results cross-checked against the firmware
  estimator replayed through `libfirmware_host.dylib` to float noise.

## Scope

### In Scope

- `App::StateEstimator` core: `wheelAt`/`bodyAt`/`whereAmI`/`wheelNow`/
  `reset`/`innovations`; `WheelEstimate`/`BodyEstimate` peer state
  structs.
- ZOH v1 extrapolation; v1 complementary-blend fusion with fail-closed,
  live-tunable weights; staleness gating.
- Cycle placement wiring (kPace block, after OTOS/odometry integration).
- Capture tooling and the leave-one-out RMS notebook, reading from the
  sprint-115 `tlm_log.py` CSV output (not on-chip ring dumps).
- Confirming `PING`'s `t=` clock-sync activation (landed with 115/116's
  protocol work) is sufficient for this sprint's needs, or identifying
  what remains — full external-pose clock-sync build-out is out of scope
  here (see below).

### Out of Scope (future work, noted but not planned here)

- **Fake OTOS** test device (`Devices::PoseSensor` extraction,
  `FakeOtos`, `ROBOT_FAKE_OTOS` build seam) — a later sprint once the
  estimator core is bench-proven.
- **External/camera pose source** (`PoseFix` revival + velocity extension
  + firmware consumer) and its clock-sync build-out beyond the `PING t=`
  activation already landing in 115/116.
- **The remaining-distance trajectory controller** — the source issue's
  stated end goal (replacing the deleted Executor's completion machinery,
  closing the turn-non-termination and terminal-wedge blocker issues) —
  is explicitly deferred until the estimator is bench-proven per the
  stakeholder's stated sequencing ("full arc planned up front, estimator
  first; the controller sprint is detailed only after the estimator gate").
- Any on-chip measurement-ring or ring-dump mechanism — superseded by the
  telemetry-log dataset per the re-scope note above.

## Test Strategy

`uv run python -m pytest` + sim suite; `just build-clean`; `mbdeploy
deploy` (hex by full UID); hardware bench gate per
`.claude/rules/hardware-bench-testing.md`. Core proof, sim first then
bench: capture a TLM-log CSV over varied motion patterns (steady, ramp,
reversal, pivot; both directions; turns and straights) via sprint 115's
logging tool; run the leave-one-out one-step-ahead walk per stream
(encoder, OTOS when present); RMS-analyze by pattern phase; propagate
per-step error through position integration to project leg-level
accumulated error; cross-check the same data replayed through
`libfirmware_host.dylib` against the notebook to float noise.

## Architecture

**Substantial** — this sprint adds a new module (`App::StateEstimator`),
introduces a new cross-module wire dependency (`ConfigDelta.estimator`,
a new `EstimatorConfigPatch` oneof arm spanning `messages/` →
`config/` → `app/`), and adds new fail-closed config keys (`data/robots/
*.json` → `gen_boot_config.py` → baked `boot_config.cpp`). That is 3+
modules touched (`app/`, `messages/`, `config/`, plus host tooling under
`src/tests/`) and a genuine new cross-module dependency — comfortably
past the compact tier's "one module, no new dependency" ceiling. Per the
project's design-doc-set opt-in, the full content lives in this sprint's
`design/` overlay (`docs/design/design.md` and `src/firm/app/DESIGN.md`,
seeded/edited/diffed on `master` before ticketing — see this document's
own "Design Overlay" section below for exactly which docs are overlaid
vs. directly ticket-owned), not in this section. Full 7-step methodology,
diagram included, in the overlay.

### Architecture Overview

See `clasi/sprints/117-predict-to-now-estimator-v1-on-the-minimal-base/design/design.md`
and `.../design/DESIGN.md` (the co-located `src/firm/app/DESIGN.md`
overlay copy — resolved via that directory's `_sources.json` manifest)
for the complete Architecture Overview, Design Rationale, and Migration
Concerns — this sprint's architecture content is authored directly into
the overlay per the `architecture-authoring` skill's Mode 2a, not
duplicated here.

### Design Rationale

See the overlay (above).

### Migration Concerns

See the overlay (above).

## Use Cases

### SUC-056: Host clock-sync converges using the PING `t=` robot-clock stamp
Parent: UC-001

- **Actor**: Python host (`ClockSync`)
- **Preconditions**: Robot firmware running, `PING`/`HELLO` text plane
  reachable over serial or the radio relay.
- **Main Flow**:
  1. Host sends the bare text command `PING`.
  2. Firmware replies `OK pong t=<ms>` — the robot's own `Devices::Clock`
     time at reply-formatting time, appended to the existing `OK pong`
     liveness reply.
  3. Host's `ClockSync.ping_burst()` records the `(t0, t1, t_robot)`
     sample per exchange, as it already does when `t=` is present
     (`_parse_pong_t()` already tolerates and requires it).
  4. After a burst (default 5 pings), `ClockSync.to_host_time()`/
     `to_robot_time()` become usable (min-RTT offset, and a skew fit once
     ≥2 samples span enough robot time).
- **Postconditions**: `ClockSync` holds a non-`None` offset estimate;
  repeated bursts refine skew.
- **Acceptance Criteria**:
  - [ ] `PING`'s reply is `OK pong t=<ms>` on both serial and radio-relay
        transports; `<ms>` reflects the firmware's own clock, not a
        constant.
  - [ ] A host-side activation test (sim or bench) drives a `ClockSync`
        instance against the live/simulated firmware and confirms a
        non-`None` `best_offset()` after one burst.
  - [ ] `docs/protocol-v4.md` §2.4 no longer documents the "AS-BUILT
        divergence" gap — `PING`'s shipped reply matches its documented
        contract.

### SUC-057: Query predicted wheel/body state at an arbitrary instant (ZOH v1)
Parent: UC-001

- **Actor**: `App::RobotLoop` (internal caller) / a unit test (external,
  hand-fed caller)
- **Preconditions**: `App::StateEstimator` has ingested at least one
  `update(frame, now)` call for the wheel/body peer being queried.
- **Main Flow**:
  1. Caller calls `wheelAt(wheel, t)` for a wheel and a query time `t`
     at or after the wheel's last basis time.
  2. Estimator extrapolates `distance = basis.position + basis.velocity ×
     (t − basis.basisTime)`, holding `basis.velocity` constant (ZOH).
  3. Caller calls `bodyAt(t)` — the body's ZOH extrapolation generalizes
     the deleted `HeadingSource::headingLead()` equation
     (`heading = basis.heading + basis.omega × age`) to the full pose
     (x, y, heading, v_x, v_y, omega).
  4. `whereAmI(now)` is exactly `bodyAt(now)`; `wheelNow(wheel)` returns
     the wheel's raw basis reading with no extrapolation.
- **Postconditions**: A query before any `update()` call returns
  `valid = false` (fail-closed, never a stale zero-initialized guess
  presented as real).
- **Acceptance Criteria**:
  - [ ] `wheelAt`/`bodyAt` reproduce the constant-velocity ZOH formula
        exactly against hand-fed basis + query-time fixtures (no clock,
        no bus — pure computation, unit-testable standalone).
  - [ ] `valid` is `false` before the first `update()` call and `true`
        after, for both wheel and body peers independently.
  - [ ] `reset(x, y, heading)` re-anchors the body peer's world pose
        (mirrors `Odometry::reset()`'s teleport semantics) without
        disturbing wheel-peer state.
  - [ ] `innovations()` reports the most recent OTOS-vs-predicted
        heading/omega residual, computed even while its fusion weight is
        0 (diagnostic, not fed back into the estimate at v1).

### SUC-058: Live-tune estimator fusion weights via a CONFIG patch, fail-closed baked defaults
Parent: UC-014

- **Actor**: Python host (bench tuning session) / `Config`'s boot-time
  codegen (fail-closed default path)
- **Preconditions**: Robot firmware booted with a fail-closed
  `estimator` section present in the active robot's
  `data/robots/*.json` (missing key ⇒ codegen fails loudly, matching the
  `output_deadband` precedent — never a silent bench-placeholder
  substitution).
- **Main Flow**:
  1. At boot, `Config::defaultEstimatorConfig()` (baked by
     `gen_boot_config.py` from the robot JSON) constructs
     `App::StateEstimator` with its fail-closed weight defaults
     (`weight_heading_otos = 0.0`, `weight_omega_otos = 0.0` this
     sprint, per the stakeholder's encoder-only-v1 decision).
  2. Host sends `ConfigDelta{estimator: EstimatorConfigPatch{...}}`.
  3. `RobotLoop::handleConfig()` merges present fields onto the
     estimator's live weights (mirrors `OtosConfigPatch`'s merge-then-
     apply pattern) and acks OK.
  4. The new weights take effect on the next `update()` call; they are
     NOT persisted to flash (Decision 4, overlay) — a reboot reverts to
     the baked JSON default.
- **Postconditions**: Estimator fusion behavior reflects the live-tuned
  weights until the next reboot.
- **Acceptance Criteria**:
  - [ ] A robot JSON missing the `estimator` section fails codegen
        loudly (mirrors `test_gen_boot_config_required_keys.py`'s
        existing pattern), not a silent bench-placeholder substitution.
  - [ ] A `ConfigDelta{estimator: ...}` patch is accepted, acked OK, and
        its present fields are readable back via the estimator's own
        live weight state in a unit test.
  - [ ] An absent field in the patch leaves that weight's CURRENT value
        untouched (partial-patch semantics, matching `MotorConfigPatch`/
        `OtosConfigPatch`).

### SUC-059: Estimator ticks every cycle with no motion-timing regression
Parent: UC-001

- **Actor**: `App::RobotLoop::cycle()`
- **Preconditions**: Firmware built with `App::StateEstimator` wired into
  the composition root.
- **Main Flow**:
  1. Each cycle's trailing `kPace` block runs OTOS sampling, odometry
     integration, and line/color polling as today.
  2. Immediately after `frame_.pose` is staged, `RobotLoop` calls
     `stateEstimator_.update(frame_, nowUs)`.
  3. The call is bounded, non-sleeping, non-bus-touching (pure float
     math over already-staged `frame_` data) — it never grows the
     `kPace` block's bus-touching surface.
- **Postconditions**: Encoder tracking-vs-commanded-speed accuracy is
  unchanged from a pre-estimator build (bench comparison).
  `stateEstimator_` holds a fresh, valid estimate every cycle once
  warmed up.
- **Acceptance Criteria**:
  - [ ] `grep 'runAndWait\|sleepUntil' app/robot_loop.cpp` is unchanged
        by this sprint — no new wait introduced by wiring the estimator
        in.
  - [ ] A sim/unit test on `App::RobotLoop` asserts the estimator holds
        `valid = true` wheel and body estimates after warm-up and that
        cycle timing (measured the same way existing timing tests
        measure it) is unaffected.

### SUC-060: Estimator tracks simulated plant truth during varied MOVE patterns
Parent: UC-001

- **Actor**: `src/tests/sim/system/` scenario test
- **Preconditions**: `SimApi`/`SimPlant` composition root wired with
  `App::StateEstimator`.
- **Main Flow**:
  1. Drive a scripted sequence of MOVE patterns through `SimApi` — both
     directions, steps, reversals, pivots.
  2. At each step, compare `StateEstimator::whereAmI()`/`wheelNow()`
     against `SimPlant`'s own ground-truth wheel/body state.
- **Postconditions**: Estimate-vs-truth error stays within a documented
  tolerance across the whole pattern set.
- **Acceptance Criteria**:
  - [ ] A sim system test drives the full pattern set (steady, ramp,
        reversal, pivot; both directions) and asserts bounded
        estimate-vs-truth error per stream (wheel distance/velocity,
        body heading/omega).
  - [ ] (Stretch, "if cheap" per the source issue) a throwaway
        host-build replay harness, mirroring the existing
        `sim/unit/*_harness.cpp` convention, cross-checks the firmware
        estimator's own output against the Python one-step-ahead
        reference (SUC-061) to float noise on the same input sequence.

### SUC-061: Leave-one-out one-step-ahead RMS validation over a captured TLM log
Parent: UC-001

- **Actor**: Engineer running `estimator_validation.ipynb`
- **Preconditions**: A TLM-log CSV exists (sim or bench capture, via
  sprint 115's `tlm_log.py`) spanning steady/ramp/reversal/pivot motion,
  both directions, turns and straights.
- **Main Flow**:
  1. `estimator_capture.py` drives the varied MOVE pattern set while
     `tlm_log.py` streams frames to CSV (sim-first, then bench).
  2. The notebook loads the CSV and, per stream (each wheel's position/
     velocity, body heading), walks it with the pure-Python
     `one_step_ahead.py` reference: for every sample k, exclude it, take
     k−1 as ZOH basis, predict k's timestamp, diff against actual k.
  3. RMS the one-step-ahead errors, broken out by pattern phase (steady,
     ramp, reversal, pivot).
  4. Check the ZOH lag signature (`a·k` velocity error, `½a·k²` distance
     error on ramps) against theory.
  5. Propagate per-step error through position integration to project a
     leg-level accumulated position/heading error.
- **Postconditions**: RMS tables, phase breakdown, ZOH-lag-signature
  check, and a leg-level error projection are produced as notebook
  output — proposed accept thresholds, not yet stakeholder-ratified.
- **Acceptance Criteria**:
  - [ ] `one_step_ahead.py` (pure functions, no I/O) has unit tests
        covering ZOH prediction math and staleness edge cases,
        independent of the C++ estimator.
  - [ ] The notebook runs end-to-end against a real (or sim-substitute)
        captured CSV and produces per-stream, per-phase RMS tables plus
        the ZOH-lag-signature check and the leg-level error projection.

### SUC-062: Stakeholder ratifies RMS accept thresholds from real (or sim-substitute) data
Parent: UC-014

- **Actor**: Eric (stakeholder)
- **Preconditions**: SUC-061's notebook output exists for a bench (or,
  if the motor-bus disconnect from the 116 gate has not been resolved by
  execution time, sim-substitute) capture.
- **Main Flow**:
  1. Engineer presents the RMS tables and ZOH-lag-signature check.
  2. Stakeholder reviews and either ratifies thresholds or requests
     further capture/analysis.
- **Postconditions**: Accept thresholds are either ratified or explicitly
  left open for a follow-up — never silently self-ratified by the
  notebook or the executing agent.
- **Acceptance Criteria**:
  - [ ] `docs/bench-checklists/sprint-117-estimator-v1.md` exists,
        structured for a stakeholder-run real-hardware re-verification
        once the motor-bus disconnect (`bench-motor-bus-disconnect-
        during-116-gate.md`) is resolved.
  - [ ] If real hardware is available at execution time and its motor
        bus has recovered, the real capture is attempted FIRST (checking
        `conn=`/bus-health telemetry flags before driving); sim-mode
        capture is used as the dataset only if the bus is still down,
        with that fact recorded, not silently substituted.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [x] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan — **pending; this is the
      stakeholder-review wall this sprint stops at.**

## Design Overlay

This sprint seeded its overlay from the already-opted-in canonical
design-doc set, edited in place, diffed, and committed on `master`
before ticketing — following the same mechanics sprint 116 established
(that sprint opted in belatedly, mid-planning; this one uses the same
lifecycle from the start).

**Overlaid** (seeded pristine, edited in place, diffed, committed in
`clasi/sprints/117-predict-to-now-estimator-v1-on-the-minimal-base/design/`):
- `docs/design/design.md` (system doc) — project-overview sentence,
  subsystem map's `app/` row (StateEstimator added), the 117-landing
  paragraph in §5's firmware-tree overview, the cycle-flow step 4
  update, and §6's open-questions bullet.
- `src/firm/app/DESIGN.md` (co-located) — `StateEstimator`'s full
  purpose/boundary/cycle-placement/interface detail, the `handleConfig`
  config-patch-coverage bullet extended to `EstimatorConfigPatch`, and
  two new Open Questions (wire-visibility deferral, persisted-tuning
  deferral).

**Not overlaid — to be edited directly on the canonical doc during
execution, by the ticket that owns the change** (same reason as 116:
the overlay directory is flat and keyed by filename, so only one
`DESIGN.md`-named file can occupy this sprint's overlay slot — `app/`
above is it):
- `src/firm/messages/DESIGN.md` — the new `ConfigDelta.estimator` arm
  (field 6) and `EstimatorConfigPatch`/`ConfigTarget.CONFIG_ESTIMATOR`
  entries. Owner: ticket 003.
- `src/firm/config/DESIGN.md` — the new fail-closed `estimator` JSON
  section and `Config::defaultEstimatorConfig()`. Owner: ticket 003.
- `src/host/robot_radio/DESIGN.md` — the new `NezhaProtocol.
  estimator_config(...)` live-tuning method (mirrors `otos_config()`)
  and the PING/clock-sync activation note. Owners: ticket 001 (PING/
  clock-sync) and ticket 003 (estimator_config()) each add their own
  bullet.
- `src/tests/DESIGN.md` — the new `estimator_capture.py` bench script,
  `tools/one_step_ahead.py` reference module, `estimator_validation.
  ipynb` notebook, and the new `sim/unit`/`sim/system` harness pairs.
  Owner: ticket 006 (tooling) with ticket 005/008 adding their own
  bullets for the sim-harness and bench-checklist pieces respectively.

At sprint close, `overlay.apply()` copies the two overlaid files above
onto their canonical targets; the four directly-edited docs are already
at their canonical location by then and need no apply step.

## Tickets

**Planning-stage ticket breakdown — stakeholder review pending.** Per
this sprint's phase gate, ticket FILES are not yet created
(`create_ticket` runs during the ticketing phase, after the
stakeholder-approval gate below is recorded); this table is the
dependency-ordered plan the team-lead/stakeholder reviews before that
phase advances. Numbering, scope, and dependencies below are final
unless the review changes them.

| # | Title | Depends On | Use Case(s) | Issue |
|---|-------|------------|-------------|-------|
| 001 | `PING t=<ms>` firmware timestamp + `docs/protocol-v4.md` update + host clock-sync activation | — | SUC-056 | predict-to-now... |
| 002 | `App::StateEstimator` core module (`state_estimator.{h,cpp}`): `WheelEstimate`/`BodyEstimate`/`Innovations` structs, `wheelAt`/`bodyAt`/`whereAmI`/`wheelNow`/`reset`/`innovations`/`setWeights`, ZOH v1 extrapolation, v1 complementary blend scaffold (weights injected, not yet fed by live config) | — | SUC-057 | predict-to-now... |
| 003 | Fail-closed estimator fusion-weight config (`data/robots/*.json` → `gen_boot_config.py` → `boot_config.cpp`) + live-tunable `EstimatorConfigPatch` (`config.proto`/`envelope.proto` arm 6) + `RobotLoop::handleConfig` branch + host `estimator_config()` | 002 | SUC-058 | predict-to-now... |
| 004 | Cycle placement wiring — `main.cpp`/`src/sim/sim_harness.h` construct `StateEstimator` from baked config; `RobotLoop::cycle()` calls `update()` in the trailing `kPace` block after `frame_.pose` is staged; timing-regression check | 002, 003 | SUC-059 | predict-to-now... |
| 005 | Sim system scenario — estimator tracks `SimPlant` truth across varied MOVE patterns (steady/ramp/reversal/pivot, both directions); stretch: throwaway `libfirmware_host` replay-harness cross-check against ticket 006's Python reference | 004 | SUC-060 | predict-to-now... |
| 006 | Host analysis tooling — `src/tests/bench/estimator_capture.py` (varied MOVE patterns + `tlm_log.py` capture, sim-first); `src/tests/tools/one_step_ahead.py` pure-Python ZOH reference; `src/tests/unit/test_one_step_ahead.py` | 002 | SUC-061 | predict-to-now... |
| 007 | Leave-one-out RMS validation notebook — `src/tests/notebooks/estimator_validation.ipynb`: per-stream/per-phase RMS, ZOH-lag-signature check, leg-level error projection | 006 | SUC-061 | predict-to-now... |
| 008 | Bench gate — attempt real capture (check `conn=`/bus-health flags first, per `bench-motor-bus-disconnect-during-116-gate.md`); sim-mode capture as the dataset if the bus is still down; full RMS run; `docs/bench-checklists/sprint-117-estimator-v1.md` for stakeholder ratification | 004, 006, 007 | SUC-062 | predict-to-now..., bench-motor-bus-disconnect-during-116-gate |

Tickets execute serially in the order listed (001 and 002 have no
inter-dependency and could run in parallel if the sprint opts into
worktree execution; 003-004 form the integration chain; 005-007 build
validation in parallel once their own dependencies are met; 008 is the
closing gate).
