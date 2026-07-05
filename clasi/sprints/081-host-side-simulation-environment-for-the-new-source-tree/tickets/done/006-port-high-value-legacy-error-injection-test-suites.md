---
id: '006'
title: Port high-value legacy error-injection test suites
status: done
use-cases:
- SUC-006
depends-on:
- '005'
github-issue: ''
issue: host-side-simulation-environment-for-the-new-tree-design-write-up.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Port high-value legacy error-injection test suites

## Description

`tests_old/simulation/` holds pre-rebuild encoder-error, OTOS-error, and
stiction/lag suites that the old system's error models (sprints 058, 069,
072, 073 in the pre-greenfield tree) already proved out — this is the
harness's clearest test-value delivery, and the encoder-wedge history makes
deterministic, off-hardware error-model regression coverage genuinely
valuable (not merely nice-to-have). This ticket ports the highest-value
subset onto the new tree's `Sim`/`sim_conn` API (ticket 005).

Depends on ticket 005 (the Python wrapper, fixtures, and first tests must
exist to port additional tests against).

## Acceptance Criteria

- [x] Encoder-error suite ported (per-wheel scale error, slip, Gaussian
      noise knobs — confirm reported encoder diverges from true encoder by
      the configured amount, and that zeroing the knobs restores agreement).
- [x] OTOS-error suite ported (noise/scale/drift knobs — confirm the
      `SimOdometer` accumulator diverges from true pose independently of
      the encoder error model, per `Hal::PhysicsWorld`'s two-independent-
      accumulators design).
- [x] Stiction/lag suite ported (stiction gate + first-order motor lag
      response envelopes from ticket 003's `PhysicsWorld`).
- [x] Every ported test is adapted to the new `Sim`/`sim_conn` API and the
      new tree's naming — **no ported test references a pre-rename
      `Hal::NezhaHal`/`...ToHalCommand`/`DevLoopState::hal` name, and no
      ported test reintroduces a unit-suffixed identifier**
      (`.claude/rules/naming-and-style.md`); grep the ported files for both
      before considering the ticket done.
- [x] Ported tests are placed under `tests/sim/unit/` or `tests/sim/system/`
      per `tests/CLAUDE.md`'s domain split (whole-robot scenario assertions
      go under `system/`; narrower per-model assertions under `unit/`).
- [x] EKF/fusion-dependent tests from the legacy suite are **explicitly
      excluded**, with a comment stating why (no firmware consumer of OTOS
      exists yet in the new tree — `architecture-update.md`'s "OTOS gap"
      note) — not silently skipped and not mis-asserted against a fusion
      path that doesn't exist.
- [x] `uv run python -m pytest tests/sim` remains green with the ported
      suites included.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` — the ported
  suites must not regress anything from tickets 001-005.
- **New tests to write**: the ported encoder-error, OTOS-error, and
  stiction/lag suites themselves (this ticket's entire content).
- **Verification command**: `uv run python -m pytest tests/sim -q`.

## Implementation Plan

**Approach:**

1. Read `tests_old/simulation/`'s encoder-error, OTOS-error, and
   stiction/lag test files in full; identify which specific assertions are
   "high-value" (test a real, previously-hard-won error-model behavior) vs.
   which were incidental to the old harness's own now-obsolete API shape.
2. Port each selected test's assertions onto ticket 005's `Sim` wrapper and
   ticket 003's error-knob setters, translating any old symbol name
   (`Hal::NezhaHal`-era or otherwise pre-rename) to its current equivalent
   — never propagate a stale name into the new tree (per this sprint's own
   reconciliation discipline, `architecture-update.md`'s "Reconciliation"
   section).
3. Explicitly mark and skip (with a comment, not silently) any legacy
   assertion that depends on EKF/fusion — no firmware consumer of OTOS
   exists yet.
4. Place ported files per `tests/CLAUDE.md`'s `unit/`/`system/` split.
5. Run the full suite; confirm determinism (re-running the ported suite
   twice produces identical results, consistent with ticket 003/005's
   determinism gate).

**Files to create:**
- New test files under `tests/sim/unit/` and/or `tests/sim/system/`
  (encoder-error, OTOS-error, stiction/lag — exact filenames chosen by the
  implementer to match the existing naming convention in those
  directories).

**Files to modify:** none expected outside `tests/sim/` itself.

**Testing plan:** see "Testing" section above.

**Documentation updates:** none required — this ticket is pure test content
with no architectural or wire-visible surface. If porting surfaces a
genuine error-model gap (a legacy assertion that cannot be reproduced
faithfully against the new `PhysicsWorld`), flag it as a new `clasi/issues/`
entry rather than silently weakening the assertion.

## Closing Notes

Three new files landed under `tests/sim/unit/` (all are narrow, single-model
assertions per `tests/CLAUDE.md`'s domain split — nothing here is a
whole-robot scenario, so nothing went under `system/`):

- `test_encoder_error_injection.py` — `Hal::PhysicsWorld`'s per-wheel
  reported-encoder scale error / slip / Gaussian noise (6 tests).
- `test_otos_error_injection.py` — `Hal::SimOdometer`'s noise/scale/drift
  knobs, including two stationary-robot drift tests that isolate the
  "accumulates every tick, independent of motion" behavior cleanly (8
  tests). Carries the full legacy-suite disposition note (ported vs.
  excluded, and why) in its module docstring.
- `test_stiction_and_motor_lag.py` — the PWM dead-zone gate + first-order
  lag filter, both of which act on TRUE velocity/encoders, ahead of the
  encoder-report-error model (6 tests).

**Adaptation, not verbatim port.** None of the three new files compile a
standalone `PhysicsWorld` harness the way their legacy namesakes did
(`test_physics_world_basic.py`/`test_physics_world_stiction.py`) — ticket
005's `Sim` Python wrapper already exposes every knob these suites need, so
every new test drives `DEV M <port> DUTY <duty>` (open-loop) through a real
`Sim`/`SimHandle` instance and reads back via the wrapper's ground-truth/
errored-observation accessors. This is a stronger isolation claim than the
legacy closed-loop version could make: with `DEV M DUTY` there is no PID
loop reading the perturbed reported encoder back into the plant, so
"encoder-report-error does not touch true pose" is now a bit-level, not
approximate, guarantee within the new tests.

**Excluded, and why (see `test_otos_error_injection.py`'s module docstring
for the itemized file-by-file list):**
- EKF/fusion-dependent (`pose=`, `encpose=`, `Odometry::correct()`) — no
  `Odometry`/`Planner`/EKF loop exists in `source/` this sprint
  (architecture-update.md's "OTOS gap"). Affects
  `test_069_004_encoder_otos_knobs.py`, `test_068_004_zero_error_three_pose_agreement.py`,
  `test_070_004_sim_errors_from_cal.py`, `test_otos_fusion.py`, and the
  `pose=`-asserting lines of `test_069_knob_telemetry_sweep.py`.
- No equivalent command surface (`T`/`RT`/`D`/`VW`/`X`, `SIMSET`/`SIMGET`,
  `RobotConfig.rotationalSlip`) — `source/commands/` only has `DEV M`/
  `DEV DT`/`DEV WD`/liveness/`SET`/`GET`/`TLM` this sprint. Affects
  `test_rt_slip.py`, `test_073_002_setslip_decouple.py`'s SIMSET/RT parts,
  `test_069_rt_90deg_body_scrub.py`, `test_073_rt_angle_sweep.py`, all
  three `test_072_00{1,3,4}_*.py` D-drive/StopCondition/Planner files, and
  `test_069_knob_telemetry_sweep.py`'s `GROUND_TRUTH_SCRUB`/
  `PHYSICAL_ASYMMETRY` groups.
- No equivalent feature at all in the new tree, by the ported class's own
  documented decision to drop it (`sim_odometer.h`'s file header: lever-arm/
  bench-OTOS/lift machinery is explicitly out of this sprint's scope) —
  `test_sim_otos_lever_arm.py`, `test_sim_otos_heading_reset.py`,
  `test_sim_hardware_bench_otos.py`, `test_bench_otos.py`.
- Out of this ticket's "error-injection behavior" scope by kind —
  `test_fit_sim_error_model.py` (a `scipy`-based calibration-fitting TOOL
  test, not an error-model behavior test).

**No new genuine error-model gap found; one pre-existing documentation
inconsistency worth a look, not a regression.** `Hal::SimOdometer::
setLinearNoiseSigma`/`setYawNoiseSigma` (`sim_odometer.h`, and
`firmware.py`'s `set_otos_linear_noise`/`set_otos_yaw_noise`) are tagged
`// [mm]`/`// [rad]`, but `sim_odometer.cpp`'s `tick()` actually applies the
sigma as a FRACTIONAL/multiplicative noise term on the per-tick delta
(`noisyDC = dC * (1.0f + gaussian(sigma))`), not an additive mm/rad noise
term — the unit tag reads like an absolute-magnitude noise but the
behavior is a relative one. This is byte-for-byte inherited from
`source_old/hal/sim/SimOdometer.cpp:169` (confirmed via grep — not
something ticket 003's port introduced), so it is not a regression and
this ticket's tests treat the knob correctly as fractional. Flagging it
only because the misleading unit tag could trip up a future reader/test
author; the team-lead may want a small follow-up doc-fix issue (retag the
comment, or leave a one-line note in `sim_odometer.h`) — not urgent enough
to block this ticket.

**Verification:** `uv run python -m pytest tests/sim` — 51 passed (31
pre-existing + 20 new). Full default `uv run python -m pytest` (`tests/sim`
+ `tests/unit` per `pyproject.toml`) — 52 passed, no regression. The three
new files' 20 tests were run twice back-to-back with identical results
(determinism). Grepped clean for `NezhaHal`/`ToHalCommand`/
`hasHalCommand`/`DevLoopState.hal` and for unit-suffixed identifiers.
