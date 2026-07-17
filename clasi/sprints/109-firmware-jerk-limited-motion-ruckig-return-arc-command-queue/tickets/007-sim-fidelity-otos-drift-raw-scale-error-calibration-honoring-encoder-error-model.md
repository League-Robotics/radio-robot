---
id: '007'
title: 'Sim fidelity: OTOS drift/raw-scale-error + calibration honoring + encoder
  error model'
status: open
use-cases: [SUC-002, SUC-004, SUC-005]
depends-on: ['002', '004', '006']
github-issue: ''
issue: sim-honors-otos-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim fidelity: OTOS drift/raw-scale-error + calibration honoring + encoder error model

## Description

This ticket makes the sim a faithful-enough testbed for ticket 009's
decisive gate: without it, the sim OTOS is ideal (only sprint-108's
drift/noise fault knobs, no scale error, no honoring of calibration
scalars), and "turns within 1° with drift/noise enabled, exact with it
disabled" would be an easier test than the real one. Depends on ticket
002 (SET/GET must work in Sim to dial in fault knobs and push
calibration) and ticket 004 (the calibration wire path must exist for the
sim to honor it) and ticket 006 (the full motion stack needs to be
in place before sim-fidelity tests are meaningful).

1. Give the simulated OTOS chip a modeled RAW scale error (per-axis
   linear/angular factor, default configurable) — per the stakeholder's
   own framing (issue, 2026-07-16): "if you are simulating the OTOS / the
   I2C bus, you should be simulating calibrations." `SimPlant`'s OTOS
   burst-read response becomes `truth * rawError`; the firmware's scalar
   register (written via ticket 004's `OtosConfigPatch`) corrects it back.
   Net = truth when calibrated, a proportional error when mis-calibrated.
2. Expose the raw-error factor via the ctypes ABI (a sim fault-condition
   knob, like the existing OTOS drift knob) so a test / the TestGUI Sim
   Errors panel can dial in a chip that needs calibrating.
3. Encoder error model: extend `SimPlant`'s existing rest-jitter hook with
   per-wheel tick quantization + scale mismatch + slip events (this is
   the "encoder error model" half of ticket 009's "with realistic
   encoder error" requirement — ticket 002 already restored the
   `enc_scale_err_l/r` ABI knob; this ticket is the fidelity work behind
   it plus the additional quantization/slip pieces).
4. OTOS drift/noise (heading random-walk + rate noise + configurable
   bias [deg/min], linear scale error) configurable via SIMSET keys —
   remember the ≤8-kv-per-message SIMSET truncation gotcha
   (`.clasi/knowledge`/memory) and chunk accordingly if more than 8 keys
   are needed across the new knobs.

## Acceptance Criteria

- [ ] Simulated OTOS models a raw scale error (linear + angular, per-axis,
      default-configurable); `SimPlant`'s OTOS burst-read response =
      `truth * rawError`.
- [ ] A sim test sets a raw OTOS scale error, confirms the uncalibrated
      pose diverges from truth, applies `OtosConfigPatch` (ticket 004),
      confirms pose converges to truth — this is SUC-005's second
      acceptance criterion, now testable end-to-end.
- [ ] Raw-error factor exposed via the ctypes ABI as a new fault-condition
      knob (parallel to the existing OTOS drift knob).
- [ ] Encoder error model extended: per-wheel tick quantization + scale
      mismatch + slip events, alongside the existing rest-jitter hook.
- [ ] All new fault knobs are settable via SIMSET, chunked to ≤8 kv pairs
      per message if the total knob count exceeds 8.
- [ ] This ticket does not touch `src/firm/` (sim-plant/Python-only
      change) — no `DESIGN.md` update required under the sprint's
      standing rule; if `src/sim/`-side design docs exist, update them
      instead (check for a `src/sim/DESIGN.md` or equivalent before
      concluding none exists).
- [ ] `test_error_divergence.py`'s `enc_scale_err` test (un-skipped by
      ticket 002) actually exercises the newly-added fidelity, not just
      the ABI knob's existence.

## Testing

- **Existing tests to run**: ticket 002's un-skipped
  `test_calibration_push_on_connect.py` / `test_error_divergence.py`
  (must now pass against REAL fidelity, not just a no-op-shaped bridge).
- **New tests to write**: raw-OTOS-scale-error-then-calibration-corrects
  sim test (SUC-005's acceptance criterion); encoder quantization/slip
  unit tests; SIMSET chunking test if the new knob count pushes past 8
  keys.
- **Verification command**: `uv run python -m pytest tests/ -k "otos or
  calibration or encoder_error"`.

## Implementation Plan

**Approach**: Additive fidelity layer inside `SimPlant`'s existing OTOS
and encoder response paths — no new top-level sim architecture, per the
sprint.md Architecture table's framing of sim fidelity as "plant/harness
fidelity, not firmware behavior."

**Files to modify**:
- `src/sim/sim_plant.{h,cpp}` (raw OTOS scale error, encoder tick
  quantization/scale/slip)
- `tests/_infra/sim/` (ctypes ABI knob for raw-error factor)
- `tests/testgui/test_calibration_push_on_connect.py`,
  `tests/testgui/test_error_divergence.py` (verify against real fidelity)

**Testing plan**: as above.

**Documentation updates**: none in `src/firm/`; check for a `src/sim/`
design doc and update if one exists.
