---
id: '006'
title: Comprehensive per-knob telemetry sweep test for the SIMSET registry
status: open
use-cases:
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
depends-on:
- '004'
github-issue: ''
issue: sim-error-model-runtime-settable-hardware-fit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Comprehensive per-knob telemetry sweep test for the SIMSET registry

## Description

Sprint success criteria (`sprint.md`) requires "a test [that] sweeps each
knob and observes the corresponding telemetry change." This ticket delivers
that single, comprehensive, REGISTRY-DRIVEN test —
`tests/simulation/system/test_069_knob_telemetry_sweep.py` — rather than one
bespoke test per knob (individual per-group tests already exist from tickets
001/002/004; this ticket is the cross-cutting completeness sweep the
architecture doc calls for in its "New tests" list).

By this ticket, every knob added in tickets 002-004 is `SIMSET`/`SIMGET`-able
(and the seven EKF fields from ticket 001 are `SET`/`GET`-able). This test
enumerates the ACTUAL registered `SIMSET` key set at runtime via a bare
`SIMGET` (no-arg dump), NOT a hardcoded Python list of key names — so a
future sprint that adds a new `kSimRegistry[]` row is covered by this test
automatically, without a corresponding Python-side edit, as long as the new
row's key is also added to this test's key→expected-telemetry-field mapping
(the one piece that cannot be auto-discovered: which TLM field a given
knob's perturbation should visibly move).

## Acceptance Criteria

- [ ] `tests/simulation/system/test_069_knob_telemetry_sweep.py`: for EVERY
      key returned by a bare `SIMGET` (no args) against a freshly-constructed
      sim, the test:
      1. Records a baseline TLM frame (`SNAP` or one `STREAM` tick) with all
         knobs at their default (no-op) values.
      2. Sends `SIMSET <key>=<non-default-value>` for that key alone (all
         other knobs remain default).
      3. Drives the sim for a short, fixed maneuver (e.g. a short `D`
         straight-line drive, or `RT` for rotation-only knobs) so the
         perturbation has an observable effect.
      4. Asserts the knob's designated TLM field diverges from the baseline
         frame's value for that field, and that fields NOT expected to be
         affected by this knob remain within a tight tolerance of baseline
         (isolation check — mirrors SUC-002/SUC-005's "without perturbing
         X" acceptance shape).
      5. Resets the sim (or re-`SIMSET`s the key back to default) before the
         next key in the sweep, so knobs don't accumulate across iterations.
- [ ] A key→(expected-TLM-field, maneuver-type) mapping table lives in the
      test file itself (e.g. a Python dict), covering every key introduced
      by tickets 001-004: the seven `SET`-only EKF keys (expected field:
      fused `pose=`'s divergence from `otos=`/`encpose=` under a deliberate
      disagreement — reuse ticket 001's existing fusion-behavior test
      pattern rather than re-deriving it here if that's simpler), and every
      `SIMSET` key from tickets 002-004 (`bodyRotScrub`/`bodyLinScrub` →
      true-pose rotation/distance; `trackwidthMm` → heading-rate
      discrepancy; `motorOffsetL`/`motorOffsetR` → per-wheel encoder
      divergence; `encScaleErrL`/`R`, `encSlipL`/`R`, `encNoiseL`/`R` →
      `enc=`; `otosLinScaleErr`/`otosAngScaleErr`/`otosLinNoise`/
      `otosYawNoise`/`otosLinDriftMmS`/`otosYawDriftDegS` → `otos=`).
  - [ ] If the test discovers via `SIMGET` a key NOT present in its mapping
        table (e.g. a future addition), it FAILS LOUDLY with a clear
        "unmapped key" message rather than silently skipping it — this is
        what makes the test's "automatic coverage of future additions"
        claim actually enforced, not aspirational.
- [ ] Full default suite green: `uv run python -m pytest`.

## Testing

- **Existing tests to run**: full default suite (this ticket's own test is
  additive; it should not perturb any other test's sim state given the
  per-key reset in step 5 above).
- **New tests to write**: `test_069_knob_telemetry_sweep.py` as described.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Drive the sweep from the wire itself (`SIMGET` with no args)
rather than a hand-maintained key list, so the test's coverage claim is
verifiable and self-updating for `SIMSET` keys. The EKF (`SET`)-key half of
the sweep can reasonably reuse ticket 001's existing fusion-behavior test
scaffolding rather than re-implementing a generic "perturb and observe"
loop for a fundamentally different (fusion-weighting, not plant-error)
mechanism — note this design choice in the test file's docstring so a
future reader understands why the EKF keys aren't enumerated via `SIMGET`
(they are `SET`, not `SIMSET`, keys, and are covered by a separate,
existing test).

**Files to create**:
- `tests/simulation/system/test_069_knob_telemetry_sweep.py`.

**Testing plan**:
- Run the new sweep test in isolation first to validate the per-key
  reset/isolation logic before running it as part of the full suite.
- Full `uv run python -m pytest`.

**Documentation updates**: none beyond the test file's own docstring
explaining its registry-driven design and the EKF/SIMSET split.
